"""Chinese order -> validated `workcell.task.v1`. Standard library only; the model runs in its own ROCm process.

Order of trust: exact rules first (instant, no model), the local Qwen worker for everything else, and a deterministic
check after the model, because the model once put "放左盘" into tray A. The result is never a motor command.
"""
import json
import os
import re
import subprocess
import threading
import time
from pathlib import Path

from .tasks import guard_instruction, result, validate

TRAYS = (('A', r'右(?:边的?|侧的?)?(?:托?盘|盒|托盘)|A\s*(?:托?盘|盒)'), ('B', r'左(?:边的?|侧的?)?(?:托?盘|盒|托盘)|B\s*(?:托?盘|盒)'))
# M6 first: '小螺丝' must not be read as '螺丝'. 'M6'/'M12' are rewritten to '小'/'大' by normalize() (else 'M6螺丝' reads as
# six screws). A bare part word is the M12 part, as before the M6 parts existed. (2026-10-31)
PARTS = (('screw_m6', r'小\s*(?:螺丝|螺钉|螺栓)'), ('nut_m6', r'小\s*(?:螺母|螺帽)'),
         ('screw', r'(?<!小)大?\s*(?:螺丝|螺钉|螺栓)'), ('nut', r'(?<!小)大?\s*(?:螺母|螺帽)'))
ALL_PARTS = re.compile(r'(?:所有|全部)的?(?:零件|东西|物料|工件)')
NUMBERS = {'一': 1, '二': 2, '两': 2, '三': 3, '四': 4, '五': 5, '六': 6, '七': 7, '八': 8, '九': 9, '十': 10}


SKIP_COOLING = re.compile(r'(?:不用|不要|不必|无需|别|不)\s*(?:再)?\s*(?:等待?|管)\s*(?:温度|降温|冷却)?(?:下降|降下来)?(?:了|结束|完)?|跳过\s*(?:降温|冷却|等待)|不\s*(?:用|必)?\s*(?:降温|冷却)|直接开始|马上开始|立刻开始|现在就开始|不等降温')


def split_skip_cooling(text):
    """-> (text without the 'do not wait for cooling' phrases, whether the operator asked to skip the cooling wait)."""
    cleaned = SKIP_COOLING.sub('，', text)
    return re.sub(r'[，,\s]{2,}', '，', cleaned).strip('，, '), cleaned != text


def normalize(text):
    """Tray words to the canonical 'A盒'/'B盒': the right tray is A (source), the left tray is B."""
    text = re.sub(r'[Mm]\s*6(?!\d)', '小', text)
    text = re.sub(r'[Mm]\s*12(?!\d)', '大', text)
    for name, pattern in TRAYS:
        text = re.sub(pattern, name + '盒', text)
    # "放回右边" / "放到左侧": a bare side after the verb is also a tray
    text = re.sub(r'(放(?:回|到|入|进|在)?)\s*右[边侧]?(?![的盘盒托])', r'\1A盒', text)
    text = re.sub(r'(放(?:回|到|入|进|在)?)\s*左[边侧]?(?![的盘盒托])', r'\1B盒', text)
    return text


def trays_named(text):
    return sorted(set(re.findall(r'([AB])盒', normalize(text))))


def _quantity(token):
    if token.isdigit():
        return int(token)
    if token == '十':
        return 10
    return NUMBERS.get(token)


DEFINITE = re.compile(r'(?P<src>[AB])盒子?(?:里|中)?的?(?P<part>' + '|'.join(p for _, p in PARTS) + ')')
CLAUSE = re.compile(r'(?P<quantity>\d+|[一二两三四五六七八九十]|全部|所有)\s*(?:颗|枚|个|只|件)?\s*(?P<part>' + '|'.join(p for _, p in PARTS) + r')')


def fast_parse(text):
    """Exact-pattern order, or None. Each clause needs an explicit quantity and part; the tray is the one the sentence names."""
    text = normalize(text)
    # 大/小 directly before a part word is its size; anywhere else it is an attribute the rules do not handle
    plain = re.sub(r'[大小]\s*(?=螺丝|螺钉|螺栓|螺母|螺帽)', '', text)
    if re.search(r'不要|别|不|慢|快|长|短|大|小|红|蓝|左边的|右边的|如果', plain.replace('不要动', '')):
        return None
    clauses = [c for c in re.split(r'然后|再|接着|之后|同时|并且|，|,|；|;|。', text) if c.strip()]
    steps, whole, inherit, used_definite = [], 0, [], False
    for clause in clauses:
        if ALL_PARTS.search(clause):
            # "把所有零件放到B盒": every class, all of each; the tray is the one this clause (or a later one) names
            named = re.findall(r'([AB])盒', clause)
            steps += [dict(part_class=name, quantity='all', region='all', destination=named[-1] if named else None) for name in ('screw', 'nut', 'screw_m6', 'nut_m6')]
            whole += 4
            continue
        if re.fullmatch(r'\s*(?:' + '|'.join(p for _, p in PARTS) + r')\s*(?:不要动|不动|保持不动|留着|留下)\s*', clause):
            continue  # an exclusion, read again below
        named = re.findall(r'([AB])盒', clause)
        found = list(CLAUSE.finditer(clause))
        definite = None if found else DEFINITE.search(clause)
        if definite:
            used_definite = True
            # "把左边盘子的螺母放回右边": no number, but the tray names the whole group. Quantity is 'all' of that tray, the destination
            # is the tray named after the verb, or the other tray for "放过去/放回去".
            source = definite['src']
            target = re.search(r'放(?:回|到|入|进|在)?\s*([AB])盒', clause)
            destination = target[1] if target else ('B' if source == 'A' else 'A') if re.search(r'放过去|放回去|拿过去|挪过去|移过去', clause) else None
            part = next(name for name, pattern in PARTS if re.fullmatch(pattern, definite['part']))
            steps.append(dict(part_class=part, quantity='all', region='left' if source == 'B' else 'right', destination=destination))
            continue
        if not found:
            if not named:
                return None
            for step in steps:  # a trailing "放B盒" gives the tray to the steps still without one
                step['destination'] = step['destination'] or named[-1]
            continue
        for match in found:
            part = next(name for name, pattern in PARTS if re.fullmatch(pattern, match['part']))
            quantity = 'all' if match['quantity'] in ('全部', '所有') else _quantity(match['quantity'])
            if quantity is None:
                return None
            steps.append(dict(part_class=part, quantity=quantity, region='all', destination=named[-1] if named else None))
            inherit.append(steps[-1])
    # "把A，B和C放到左盘": the tray comes with the last clause. When the whole sentence names exactly one tray, the quantity clauses before it
    # go there too (the same rule correct_destinations applies to a model answer). Not with a "左盘的螺母" clause: that tray is a source.
    trays = set(re.findall(r'([AB])盒', text))
    if len(trays) == 1 and not used_definite:
        for step in inherit:
            step['destination'] = step['destination'] or next(iter(trays))
    if not steps or any(s['destination'] is None for s in steps) or len(steps) > 4 + len(re.findall('不要动', text)):
        return None
    exclude = [name for name, pattern in PARTS if re.search(r'(?:' + pattern + r')不要动', text)]
    words = len(re.findall('|'.join(p for _, p in PARTS), text))
    if words + whole != len(steps) + len(exclude):
        return None  # a part word the rules did not turn into a step: leave the sentence to the model
    steps = [s for s in steps if s['part_class'] not in exclude]
    return result('plan', steps, exclude) if steps else None


def correct_destinations(task, text):
    """If the sentence names exactly one tray, every step goes there; a differing model answer is overridden and reported."""
    named = trays_named(text)
    if task['action'] != 'plan' or len(named) != 1:
        return task, False
    changed = any(s['destination'] != named[0] for s in task['steps'])
    if changed:
        task = dict(task, steps=[dict(s, destination=named[0]) for s in task['steps']])
    return validate(task), changed


def correct_sizes(task, text):
    """The sentence decides M6 vs M12: the model (its LoRA saw only the M12 classes) may answer 'screw' for '小螺丝'.

    Each step takes the size of the part word at the same position in the sentence when both name the same kind; a sentence
    whose part words cannot be matched one to one is not guessed at."""
    if task['action'] != 'plan':
        return task, False
    text = normalize(text)
    if ALL_PARTS.search(text):
        return task, False
    words = [next(name for name, pattern in PARTS if re.fullmatch(pattern, m.group(0))) for m in re.finditer('|'.join(p for _, p in PARTS), text)]
    words = [w for w in words if w not in task['exclude']]
    if not any(w.endswith('_m6') for w in words) and not any(s['part_class'].endswith('_m6') for s in task['steps']):
        return task, False
    if len(words) != len(task['steps']) or any(w.split('_')[0] != s['part_class'].split('_')[0] for w, s in zip(words, task['steps'])):
        raise ValueError('CLARIFICATION_REQUIRED')
    changed = any(w != s['part_class'] for w, s in zip(words, task['steps']))
    if changed:
        task = validate(dict(task, steps=[dict(s, part_class=w) for w, s in zip(words, task['steps'])]))
    return task, changed


class QwenWorker:
    """The resident ROCm Qwen2.5-VL worker (`workcell.model_server`), one JSON line per request."""

    def __init__(self, app_root, launcher=None):
        app = Path(app_root).resolve()
        launcher = launcher or str(Path.home() / '.local/share/aipc-dev/aipc.sh')
        command = [launcher, 'gpu', 'env', f'PYTHONPATH={app}/integration:{app}/.task-tune-deps', str(app / '.venv-ai-labs/bin/python'), '-u', '-m',
                   'workcell.model_server', '--app-root', str(app), '--adapter', str(app / 'reports/task_lora_v2_20261020/adapter')]
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, cwd=str(app / 'integration'))
        self.lock = threading.Lock()
        self.counter = 0
        ready = self.process.stdout.readline()
        if '"MODEL_READY"' not in ready:
            raise RuntimeError('Qwen worker did not start')
        self.info = json.loads(ready)

    def ask(self, instruction, context=None):
        with self.lock:
            self.counter += 1
            self.process.stdin.write(json.dumps(dict(request_id=str(self.counter), instruction=instruction, context=context or {}), ensure_ascii=False) + '\n')
            self.process.stdin.flush()
            reply = json.loads(self.process.stdout.readline())
        if reply.get('event') == 'MODEL_ERROR':
            raise ValueError(reply.get('error', 'model error'))
        return reply

    def review(self, facts, image_files=(), max_new_tokens=180, mode='review'):
        """Post-run result review by the same resident model (images + recorded facts -> Chinese text). Read-only."""
        with self.lock:
            self.counter += 1
            self.process.stdin.write(json.dumps(dict(op='review', request_id=str(self.counter), facts=facts, image_files=[str(f) for f in image_files],
                                                     max_new_tokens=max_new_tokens, mode=mode), ensure_ascii=False) + '\n')
            self.process.stdin.flush()
            reply = json.loads(self.process.stdout.readline())
        if reply.get('event') != 'REVIEW_RESULT':
            raise ValueError(reply.get('error', 'review failed'))
        return reply

    def close(self):
        try:
            self.process.stdin.write('{"op":"shutdown"}\n');self.process.stdin.flush();self.process.wait(5)
        except (OSError, subprocess.TimeoutExpired):
            self.process.kill()


def understand(text, worker=None, context=None):
    """-> dict(task, source 'rule'|'qwen', seconds, corrected, normalized). Raises ValueError for what cannot be expressed."""
    started = time.perf_counter()
    original = text
    text, skip_cooling = split_skip_cooling(text)
    normalized = normalize(text)
    task = fast_parse(text)
    if task is not None:
        return dict(task=task, source='rule', seconds=time.perf_counter() - started, corrected=False, normalized=normalized, skip_cooling=skip_cooling)
    if worker is None:
        raise ValueError('这句话需要语言模型，但模型没有启动')
    reply = worker.ask(normalized, context)
    task = validate(reply['intent'])
    task, corrected = correct_destinations(task, text)
    task, resized = correct_sizes(task, text)
    corrected = corrected or resized
    if task['action'] == 'plan':
        guard_instruction(task, text)
    return dict(task=task, source='qwen', seconds=time.perf_counter() - started, model_seconds=reply.get('generation_s'), corrected=corrected, normalized=normalized, skip_cooling=skip_cooling)


FRIENDLY = {
    'Empty closure: grasp not confirmed': '夹爪空夹，未确认抓住零件',
    'Actual arm pose differs from reviewed return start': '返回起点的实测关节位置尚未到位',
    'six-axis endpoint did not settle': '关节未在限定时间内稳定到位',
    'CLARIFICATION_REQUIRED_EXPLICIT_QUANTITY': '这句话没说数量。请说明几颗，比如“两颗”，或者说“全部”。',
    'TARGET_COUNT_INSUFFICIENT': '盘里的零件数量不够。',
    'CLARIFICATION_REQUIRED': '这句话还不够明确，请补充数量、类别或目的盘。',
}


def friendly_error(error):
    text = str(error)
    unlocalised = re.search(r'Unlocalised (\w+) \(geometry rejected\) (\d+) px from a target/slot', text)
    if unlocalised:
        names = dict(screw='大螺丝', nut='大螺母', screw_m6='小螺丝', nut_m6='小螺母')
        part = names.get(unlocalised[1], unlocalised[1])
        return (f'规划被拦下：检测到一个无法可靠定位的{part}，距抓取点或放置槽约 {unlocalised[2]} 像素。'
                '请检查是否重复识别、遮挡或倾斜，调整后重新观察；本批未开始运动。')
    for code, message in FRIENDLY.items():
        if code in text:
            return message
    return text if isinstance(error, str) else f'{type(error).__name__}: {text}'
