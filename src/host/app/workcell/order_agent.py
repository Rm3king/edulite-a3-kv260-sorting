"""Order agent: understand -> observe -> plan -> act -> verify -> decide, every decision with a Chinese reason.

It never commands a motor itself. Motion is the reviewed `wrist_servo_trial` session; the agent decides what to ask for,
when to wait (heat), whether to try again (what the cameras show afterwards), and when to stop and call the operator. Removing
torque from a held arm stays with the operator: `confirm_supported` only writes the marker the operator asked for.
"""
import json
import re
import subprocess
import sys
import time
from pathlib import Path

from .order_language import understand, friendly_error
from .tasks import CLASSES
from .order_planner import PEAK_LIMIT_C, ZH, batches, cooling_needed_min, COOL_C_PER_MIN, expand, unsupported_reason

TRANSIENT = ('RETURN_CAPTURE_REJECTED', 'RETURN_VISION_REJECTED', 'RETURN_BUDGET_OR_TEMPERATURE', 'RETURN_CAMERA_RESTARTED')
MAX_RETRIES = 2
PRE_ENABLE = ('Scene changed before enable', 'Vision worker did not become ready', 'moved beyond correspondence window')  # checks that run before any motor is enabled
MAX_COOL_MIN = 20.


def disabled_confirmed(summary):
    """A completed owner must prove all seven resets and zero final control state."""
    controller = summary.get('final_controller') or {}
    resets = summary.get('reset_confirmed') or []
    return (len(resets) == 7 and all(type(x) is int for x in resets) and sorted(resets) == list(range(1, 8))
            and summary.get('stop_errors') == {} and summary.get('final_active_mask') == 0
            and all(controller.get(k) == 0 for k in ('control', 'fault', 'queue_counts')))


def read_batch_completion(outcome):
    """Read only this batch's completion evidence after its monitor process exited."""
    if not outcome.get('root'):
        return outcome
    try:
        data = json.loads((Path(outcome['root']) / 'completed.json').read_text())
        summary = next(row['summary'] for row in data['results'] if 'summary' in row)
    except (OSError, ValueError, KeyError, StopIteration, TypeError):
        return outcome
    outcome['disabled_confirmed'] = disabled_confirmed(summary)
    outcome['task_error'] = summary.get('task_outcome_error')
    if summary.get('error'):
        outcome['error'] = summary['error']
        outcome['fault'] = True
    if summary.get('status') == 'FAIL_DISABLE_CONFIRMATION':
        outcome['fault'] = True
    if isinstance(summary.get('nut_statuses'), list):
        outcome['statuses'] = summary['nut_statuses']
    return outcome


class Events:
    """Timeline shown to the operator and written to disk; `listener` lets an interface follow it live."""

    def __init__(self, path=None, listener=None):
        self.items, self.path, self.listener, self.t0 = [], path, listener, time.monotonic()

    def __call__(self, kind, text, **data):
        item = dict(t=round(time.monotonic() - self.t0, 1), kind=kind, text=text, **data)
        self.items.append(item)
        if self.path:
            with open(self.path, 'a') as stream:stream.write(json.dumps(item, ensure_ascii=False) + '\n')
        if self.listener:self.listener(item)
        return item


STEP_ZH = dict(approach='下降接近', close='夹取', lift='抬起', transfer='搬运', place='放置', open='松开', exit='离开')


class OrderAgent:
    def __init__(self, app_root, worker=None, events=None, observe=None, temperatures=None, run_batch=None, verify=None, sleep=time.sleep):
        self.app = Path(app_root).resolve()
        self.worker, self.events, self.sleep = worker, events or Events(), sleep
        self._observe, self._temperatures, self._run_batch, self._verify = observe or self.observe_scene, temperatures or self.arm_temperatures, run_batch or self.run_batch, verify or self.verify_placement
        self.session = None
        self.reverse_small_nut = False
        self.state = 'IDLE'
        self.last_root = None
        self.continuous = True   # board-side continuous wrist correction (M12 parts, M6 nut); the M6 screw stays stop-and-look
        self.alignments, self.order_text, self.before_points, self.last_temps, self.last_review = [], '', None, None, None

    # ---- perception and arm state (real implementations; tests replace them)
    def _session(self):
        if self.session is None:
            from .live_session import Session
            config = json.loads((self.app / 'integration/config/live_workflow.json').read_text())
            sys.path[:0] = [p for p in (config['runtime_root'], config['can_root']) if p not in sys.path]  # board runtime modules the session imports
            overrides = dict(joint_time_law='cruise_c3')
            if self.reverse_small_nut:
                from .reverse_transfer import overrides as reverse_overrides
                overrides.update(reverse_overrides(config))
            self.session = Session(self.app, overrides=overrides)
        return self.session

    def observe_scene(self):
        from .live_session import scene_objects
        s = self._session()
        points = s.snapshot()
        frame = json.loads(points.read_text())
        return dict(objects=scene_objects(frame, s.config, CLASSES), frame=frame, points=str(points))

    def arm_temperatures(self):
        s = self._session()
        out = self.app / 'reports' / ('agent_joints_' + time.strftime('%Y%m%d_%H%M%S'))
        out.mkdir()
        s.capture_joints(out)
        feedback = json.loads((out / 'connection_setup.json').read_text())['feedback']
        latest = {f['motor']: f for f in feedback}
        if len(latest) != 7:
            raise RuntimeError('没有读到停止状态下的七个电机温度')
        return {m: f['temperature'] for m, f in latest.items()}

    def verify_placement(self, done):
        from .placement_report import verify_placement
        s = self._session()
        frame = json.loads(s.snapshot().read_text())
        before = json.loads(Path(self.before_points).read_text()) if self.reverse_small_nut and self.before_points else None
        report = verify_placement(frame, s.config, done, s.config['destinations']['pixels'], before=before)
        if self.reverse_small_nut:
            from .reverse_transfer import verify_counts
            report = verify_counts(report, before, frame, s.config)
        return report

    def run_batch(self, parts, slots, skip_thermal=False):
        """One enabled session through `wrist_servo_trial`; returns what happened, never raises for a held arm."""
        integration = self.app / 'integration'
        command = [str(Path.home() / '.local/share/aipc-dev/aipc.sh'), 'sdk', 'env', f'PYTHONPATH={integration}', 'python', '-u', '-m', 'workcell.wrist_servo_trial',
                   '--parts', *parts, '--destination-slots', *map(str, slots), '--skip-thermal-gate' if skip_thermal else '--no-cooling-wait', '--execute-attended',
                   *(['--continuous'] if self.continuous else [])]
        if self.reverse_small_nut:
            command.append('--reverse-small-nut')
        process = subprocess.Popen(command, cwd=str(integration), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        outcome = dict(statuses=[], root=None, fault=False, reduced_to=None, log=[])
        current = -1
        for line in process.stdout:
            line = line.rstrip()
            outcome['log'].append(line)
            if line.startswith('COOLING_TIMEOUT_REDUCING'):
                outcome['reduced_to'] = int(line.split()[-1])
            elif 'VISUAL_SERVO_FAILED' in line or 'ALARM_ARM_HELD' in line:
                if not outcome['fault']:
                    self.events('fault', '机械臂进入故障保持：需要托住后确认', raw=line[:200])
                outcome['fault'] = True
            elif line.startswith('END_SESSION'):
                outcome['statuses'] = re.findall(r"'([A-Z_]+)'", line.split(' ', 2)[2])
            elif line.startswith('TIMELINE '):
                # `TIMELINE <name> <seconds>`; a stray line (2026-11-02 03:00: float('"stage":') ended the whole order) must not stop the batch
                try:
                    name, seconds = line.split()[1:3]
                    outcome.setdefault('timeline', []).append((name, float(seconds)))
                except ValueError:
                    pass
            elif line.startswith('BATCH_FINISHED'):
                outcome['root'] = line.split(' ', 1)[1].strip()
            elif line.startswith('SESSION') and 'ONLINE_GRASP_TASK' in line:
                task = re.search(r'"name": "([^"]+)"', line).group(1)
                step = task.rsplit('_', 1)[-1]
                if step == 'approach':
                    current += 1   # the board names every part of a chain screw_0/nut_0: count the descents instead
                name = ZH.get(parts[current], '') if 0 <= current < len(parts) else ''
                self.events('progress', ('返回支撑位' if task == 'return_supported' else f"{name} · {STEP_ZH.get(step, step)}") if name or task == 'return_supported' else '执行：' + task.replace('_', ' '))
            elif line.startswith('ALIGNMENT '):
                # every part's wrist alignment, read from the board's full event log once the arm has stopped
                try:a = json.loads(line.split(' ', 1)[1])
                except ValueError:continue
                k = a.get('part') if isinstance(a.get('part'), int) else -1
                name = ZH.get(parts[k], parts[k]) if 0 <= k < len(parts) else f'第{k + 1}件'
                if a['mode'] == 'continuous':
                    error = a.get('error_px')
                    residual = f'{error:.1f} px' if error is not None else '未取得'
                    status = {'CONVERGED': '收敛', 'RETURN_VISION_LOST': '定位丢失，已返回'}.get(a['status'], a['status'])
                    text = f"{name}：板端连续对准 {a['seconds']:.2f} s（{status}，残差 {residual}）"
                else:
                    text = f"{name}：停顿对准 {a['rounds']} 轮后抓取"
                a = dict(a, text=text, part=parts[k] if 0 <= k < len(parts) else None)
                if a not in self.alignments:
                    self.alignments.append(a);self.events('align', text, **{x: a[x] for x in ('mode', 'seconds', 'rounds', 'status') if x in a})
            elif line.startswith('SESSION') and 'CONTINUOUS_SERVO_END' in line:
                end = re.search(r'"nut": (\d+), "status": "([A-Z_]+)", "seconds": ([0-9.]+), "error_px": ([0-9.]+)', line)
                if end:
                    k, status, seconds, error = int(end[1]), end[2], float(end[3]), float(end[4])
                    name = ZH.get(parts[k], parts[k]) if k < len(parts) else f'第{k + 1}件'
                    text = f"{name}：板端连续对准 {seconds:.2f} s（{'收敛' if status == 'CONVERGED' else status}，残差 {error:.1f} px）"
                    self.alignments.append(dict(text=text, part=parts[k] if k < len(parts) else None, seconds=seconds, status=status, mode='continuous'))
                    self.events('align', text, part=k, seconds=seconds, status=status, mode='continuous')
            elif line.startswith('SESSION') and 'VISUAL_SERVO_REPLY' in line:
                reply = re.search(r'"nut": (\d+), "iteration": (\d+), "action": "([A-Z]+)"', line)
                if reply and reply[3] == 'GRASP':
                    k = int(reply[1])
                    text = f"{ZH.get(parts[k], parts[k]) if k < len(parts) else f'第{k + 1}件'}：停顿对准 {int(reply[2]) + 1} 轮后抓取"
                    self.alignments.append(dict(text=text, part=parts[k] if k < len(parts) else None, rounds=int(reply[2]) + 1, mode='stop_and_look'))
                    self.events('align', text, part=k, rounds=int(reply[2]) + 1, mode='stop_and_look')
        outcome['returncode'] = process.wait()
        self._report_timeline(outcome.get('timeline') or [])
        if outcome['returncode']:
            failure = next((Path(l.split(' ', 1)[1]) for l in outcome['log'] if l.startswith('BATCH_FINISHED')), None)
            reports = sorted((self.app / 'reports').glob('four_integrated_*'), key=lambda p: p.stat().st_mtime)
            try:outcome['error'] = json.loads((reports[-1] / 'failure.json').read_text())['error']
            except (IndexError, OSError, ValueError, KeyError):outcome['error'] = (outcome['log'] or ['未知错误'])[-1][:200]
        self.last_root = outcome['root']
        return read_batch_completion(outcome)

    def _report_timeline(self, marks):
        """Where the time went before the arm was enabled: the step names with the seconds each took (steps under 1 s left out)."""
        before = []
        for name, seconds in marks:
            before.append((name, seconds))
            if name == 'owner_launched':
                break
        else:
            return
        steps = [f'{name} {seconds - (before[i - 1][1] if i else 0.):.0f}s' for i, (name, seconds) in enumerate(before) if seconds - (before[i - 1][1] if i else 0.) >= 1.]
        self.events('timing', f'下指令到使能 {before[-1][1]:.0f}s：' + '，'.join(steps))

    # Run folders on the board whose owner process is still alive. The marker must go into exactly the folder of the held owner:
    # 2026-11-01 it went into `last_root` (the previous, finished run), was reported as written, and the held arm stayed held.
    LIVE_OWNERS = ("for o in /home/ubuntu/aipc_wrist_servo/*/owner.json; do "
                   "p=$(sed -n 's/.*\"pid\": *\\([0-9][0-9]*\\).*/\\1/p' \"$o\"); "
                   "[ -n \"$p\" ] && kill -0 \"$p\" 2>/dev/null && basename \"$(dirname \"$o\")\"; done; true")

    def live_owner_roots(self, session=None):
        s = session or self._session()
        out = subprocess.run(['bash', s.config['ssh_helper'], self.LIVE_OWNERS], check=True, capture_output=True, text=True, timeout=20).stdout
        return [line.strip() for line in out.splitlines() if line.strip().startswith('four_integrated_') or line.strip().startswith('wrist_')]

    def confirm_supported(self, root=None):
        """Operator says the arm is supported: write the board marker the held owner is waiting for, into the folder of the live owner."""
        s = self._session()
        live = self.live_owner_roots(s)
        if root is not None:
            name = Path(root).name
        elif len(live) == 1:
            name = live[0]
        else:
            self.events('operator', '没有写入承托标记：' + ('板上没有存活的执行者' if not live else f'板上有 {len(live)} 个执行者，无法确定是哪一个，请到板上手动确认'))
            raise RuntimeError('SUPPORT_MARKER_TARGET_UNCERTAIN live_owners=' + ','.join(live))
        if name not in live:
            self.events('operator', f'没有写入承托标记：{name} 的执行者已经结束（存活的是 {live or "无"}）')
            raise RuntimeError('SUPPORT_MARKER_TARGET_NOT_LIVE ' + name)
        marker = f'/home/ubuntu/aipc_wrist_servo/{name}/execution/SUPPORTED_DISABLE'
        subprocess.run(['bash', s.config['ssh_helper'], f'touch {marker} && test -e {marker}'], check=True, capture_output=True, timeout=20)
        self.events('operator', f'已按操作员确认写入承托标记（{name}），板端开始失能')

    # ---- understand + plan (no motion)
    def plan(self, text, skip_cooling=False):
        self.state = 'PLANNING'
        if self.worker is None and not self._rule_ok(text):
            self.events('think', '这句话规则解析不了，启动本地千问理解')
        understood = understand(text, self.worker)
        task = understood['task']
        steps = '，'.join(f"{ZH[s['part_class']]}×{'全部' if s['quantity'] == 'all' else s['quantity']} → {s['destination']}盘（{'左' if s['destination'] == 'B' else '右'}）" for s in task['steps'])
        self.events('understand', f"理解订单（{'规则' if understood['source'] == 'rule' else '本地千问'}，{understood['seconds']:.1f}s）：" + (steps or task['question'] or task['action'])
                    + ('；已纠正目的盘' if understood['corrected'] else ''), task=task)
        if task['action'] != 'plan':
            self.state = 'IDLE'
            return dict(task=task, understood=understood, steps=[], batches=[])
        refusal = unsupported_reason(task)
        if refusal:
            self.events('result', refusal)
            self.state = 'IDLE'
            return dict(task=dict(task, action='clarify', question=refusal), understood=understood, steps=[], batches=[], refused=True)
        self.order_text = text
        from .reverse_transfer import requested
        reverse = requested(task)
        if self.reverse_small_nut != reverse:
            self.reverse_small_nut, self.session = reverse, None
        # the motor temperatures come over ssh (~2 s) while the camera snapshot is taken
        from concurrent.futures import ThreadPoolExecutor
        with ThreadPoolExecutor(max_workers=1) as pool:
            temperatures = pool.submit(self._temperatures)
            scene = self._observe()
            temps_early = temperatures.result()
        self.before_points = scene.get('points')
        counts = {c: sum(o['part_class'] == c for o in scene['objects']) for c in CLASSES}
        self.events('observe', '看到源盘里 ' + '、'.join(f'{ZH[c]} {n}' for c, n in counts.items() if n) if any(counts.values()) else '源盘里没有看到零件', counts=counts)
        parts = expand(task, scene['objects'])
        if self.reverse_small_nut:
            from .reverse_transfer import check_destination
            if parts != ['nut_m6']:
                raise ValueError('反向素材只搬一颗小螺母，请保证左盘只有一颗')
            check_destination(scene['frame'], self._session().config)
        temps = self.last_temps = temps_early
        self.events('observe', f'机械臂温度：J2 {temps[2]:.0f}°C，J3 {temps[3]:.0f}°C（失能状态读取）', temperatures=temps)
        skip = bool(skip_cooling or understood.get('skip_cooling'))
        if skip:
            self.events('decide', '按指令跳过降温：不等待，直接开始；板端 95°C 仍会强制返回')
        plan = batches(temps, parts, skip_cooling=skip)
        if self.reverse_small_nut:
            from .reverse_transfer import SLOT
            plan[0]['slots'] = [SLOT]
            self.events('decide', '从左盘取一颗小螺母，放到右盘空位；结束后核对两盘数量变化')
        for index, batch in enumerate(plan):
            self.events('decide', f'第 {index + 1} 批 {len(batch["parts"])} 件：' + batch['reason'], batch=batch)
        self.state = 'READY'
        return dict(task=task, understood=understood, parts=parts, temperatures=temps, batches=plan, skip_cooling=skip)

    @staticmethod
    def _rule_ok(text):
        from .order_language import fast_parse
        return fast_parse(text) is not None

    # ---- act, verify, decide
    def execute(self, plan, operator_ready):
        if not operator_ready:
            raise ValueError('需要操作员确认机械臂在支撑位后才能开始')
        self.state = 'RUNNING'
        self.alignments = []
        pending = [dict(part=p, tries=0) for p in plan['parts']]  # in order; a slot is chosen per attempt from the free ones
        done, failed = [], []
        while pending:
            temps = self.last_temps = self._temperatures()
            skip = plan.get('skip_cooling', False)
            batch = batches(temps, [x['part'] for x in pending], occupied=[s for _, s in done], skip_cooling=skip)[0]
            if batch['cool_before_min'] > 0:
                self._cool(batch)
            parts, slots = batch['parts'], batch['slots']
            if self.reverse_small_nut:
                from .reverse_transfer import SLOT
                slots = [SLOT]
            self.events('act', f"开始抓取 {len(parts)} 件（" + '、'.join(ZH[p] for p in parts) + f'），目标槽 {slots}', parts=parts, slots=slots)
            outcome = self._run_batch(parts, slots, skip) if skip else self._run_batch(parts, slots)
            if outcome['returncode'] and not outcome['fault'] and any(k in outcome.get('error', '') for k in PRE_ENABLE):
                # Aborted by a check before any motor was enabled (a part moved, a worker was slow): safe to look again and retry.
                for x in pending[:len(parts)]:x['tries'] += 1
                self.events('decide', f"使能前的检查没通过（{outcome['error'][:60]}），没有动机械臂；重新观察后重试")
                if pending[0]['tries'] > MAX_RETRIES:
                    self.state = 'ERROR';self.events('stop', '连续多次在使能前被拦下，停止，请检查零件是否被碰动');return self._report(plan, done, pending, failed)
                self.sleep(2.);continue
            if outcome['fault'] or outcome['returncode']:
                # A later failure must not erase earlier, camera-confirmed placements.
                items = pending[:len(parts)]
                statuses = outcome['statuses']
                completed = [(x, slots[k]) for k, x in enumerate(items)
                             if k < len(statuses) and statuses[k].startswith('COMPLETED')]
                try:seen = self._confirm([(x['part'], slot) for x, slot in completed])
                except Exception as error:
                    seen = [False] * len(completed)
                    self.events('observe', '故障后目标槽复核失败，保留未确认状态', raw_error=str(error))
                for (x, slot), ok in zip(completed, seen):
                    if ok:done.append((x['part'], slot))
                    else:failed.append((x['part'], slot, '已执行搬运，但视觉未在目标槽确认'))
                    pending.remove(x)
                if outcome.get('task_error') and len(statuses) < len(items):
                    k = len(statuses);x = items[k]
                    failed.append((x['part'], slots[k], friendly_error(outcome['task_error'])))
                    pending.remove(x)
                stopped = outcome.get('disabled_confirmed', False)
                self.state = 'STOPPED' if stopped else ('FAULT_HELD' if outcome['fault'] else 'ERROR')
                if stopped:
                    self.events('disabled', '已收到七轴失能确认，控制输出与故障码均为0')
                reason = ('本批出现故障，已确认七轴失能，后续任务已停止' if stopped else
                          '本批出现故障，停止后续任务；是否撤力矩由操作员确认承托后决定' if outcome['fault'] else '本批出错')
                causes = [friendly_error(outcome[k]) for k in ('task_error', 'error') if outcome.get(k)]
                if causes:reason += '：' + '；'.join(causes)
                self.events('stop', reason, outcome=dict(statuses=outcome['statuses']), raw_error=outcome.get('error'))
                return self._report(plan, done, pending, failed, stop_reason=reason)
            count = outcome['reduced_to'] or len(parts)
            items = pending[:count]
            statuses = (outcome['statuses'] + ['NOT_ATTEMPTED'] * count)[:count]
            completed = [(x, slots[k]) for k, (x, st) in enumerate(zip(items, statuses)) if st.startswith('COMPLETED')]
            seen = self._confirm([(x['part'], slot) for x, slot in completed])
            placed = {id(x) for (x, slot), ok in zip(completed, seen) if ok}
            for (x, slot), ok in zip(completed, seen):
                if ok:
                    done.append((x['part'], slot))
            for k, (x, st) in enumerate(zip(items, statuses)):
                if id(x) in placed:
                    pending.remove(x);continue
                x['tries'] += 1
                reason = '完成但视觉没有在槽里确认' if st.startswith('COMPLETED') else f'状态 {st}'
                retryable = st in TRANSIENT or st == 'NOT_ATTEMPTED'
                if skip and st == 'RETURN_BUDGET_OR_TEMPERATURE':
                    retryable = False;reason = '温度到线，按指令没有等降温'
                if not retryable:
                    # Motion finished but the camera does not show it (or a real failure): a second pick could collide with a part
                    # that is in fact in the slot, so it is reported for the operator, never retried.
                    failed.append((x['part'], slots[k], reason));pending.remove(x)
                    self.events('decide', f"{ZH[x['part']]}：{reason}，不自动重试，请人工核对")
                elif x['tries'] > MAX_RETRIES:
                    failed.append((x['part'], slots[k], reason));pending.remove(x)
                    self.events('decide', f"{ZH[x['part']]} 已尝试 {x['tries']} 次，放弃：{reason}")
                else:
                    self.events('decide', f"{ZH[x['part']]}（槽 {slots[k]}）{reason}，第 {x['tries']} 次重试前重新观察")
        self.state = 'DONE'
        return self._report(plan, done, pending, failed)

    def _confirm(self, completed):
        """Placement seen in the head camera; a miss is looked at again before it is believed (detection flickers)."""
        if not completed:
            return []
        seen = [False] * len(completed)
        for look in range(3):
            report = self._verify([dict(part_class=p, slot=s) for p, s in completed])
            found = {(x['part_class'], x['slot']) for x in report['parts'] if x['status'] == 'PLACED'}
            seen = [(p, s) in found for p, s in completed]
            if all(seen):
                break
            self.events('observe', f'第 {look + 1} 次核对：{sum(seen)}/{len(completed)} 件在槽里，再看一次')
            self.sleep(1.)
        else:
            pass
        if all(seen):
            self.events('verify', f'视觉确认：{len(completed)} 件都在目标槽里')
            if self.reverse_small_nut and report.get('tray_total_counts'):
                totals = report['tray_total_counts']
                self.events('verify', f"两盘核对：左盘 {totals['before']['B']} → {totals['after']['B']} 件，"
                            f"右盘 {totals['before']['A']} → {totals['after']['A']} 件；小螺母已从左盘转入右盘")
        return seen

    def _cool(self, batch):
        waited = 0.
        self.events('decide', batch['reason'] + '；机械臂失能等待降温')
        while waited < MAX_COOL_MIN:
            temps = self._temperatures()
            need = cooling_needed_min(temps, batch['parts'], COOL_C_PER_MIN)
            self.events('progress', f"降温中：J2 {temps[2]:.0f}°C，预计还需 {need:.1f} 分钟", temperatures=temps)
            if need <= 0:
                return
            self.sleep(min(30., max(need * 60, 5.)));waited += .5
        raise RuntimeError('等待降温超时')

    def _report(self, plan, done, pending, failed, stop_reason=None):
        total = len(plan['parts'])
        text = f'完成 {len(done)}/{total} 件' + (f'；放弃 {len(failed)} 件：' + '；'.join(f'{ZH[p]}（{r}）' for p, _, r in failed) if failed else '')
        self.events('result', text, done=len(done), total=total, failed=len(failed))
        report = dict(done=done, failed=failed, pending=pending, total=total, text=text, state=self.state)
        if stop_reason:
            report['stop_reason'] = stop_reason
        report['review'] = self._review(report)
        return report

    def _review(self, report):
        """VLM result review after the run (read-only). Camera only: no motor temperatures are read here (after a fault the
        board still holds the arm), the last reading taken by the run is used."""
        from .vlm_review import review
        after = None
        try:
            after = self._observe().get('points')
        except Exception as error:
            self.events('observe', f'复核取图失败：{error}'[:120])
        if self.worker is not None and not report.get('stop_reason'):
            self.events('think', '本地千问视觉模型复核结果中（执行前/后两张图 + 运行记录）')
        try:trays = json.loads((self.app / 'integration/config/live_workflow.json').read_text()).get('trays')
        except (OSError, ValueError):trays = None
        result = review(self.worker, self.order_text, report, self.alignments, self.before_points, after, self.last_temps, trays,
                        direction='B->A' if self.reverse_small_nut else 'A->B')
        self.last_review = result
        label = '视觉大模型复核' if result['source'] == 'vlm' else '运行记录'
        self.events('review', f"{label}：{result['summary']}", source=result['source'], seconds=round(result['seconds'], 1), images=len(result['images']))
        return result


def main():
    import argparse
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--app-root', type=Path, default=Path(__file__).resolve().parents[2])
    p.add_argument('--instruction', required=True)
    p.add_argument('--execute', action='store_true', help='run it; the operator must have the arm supported at the start pose')
    p.add_argument('--no-model', action='store_true')
    a = p.parse_args()
    worker = None
    if not a.no_model:
        from .order_language import QwenWorker, fast_parse
        if fast_parse(a.instruction) is None:
            worker = QwenWorker(a.app_root)
    agent = OrderAgent(a.app_root, worker=worker, events=Events(listener=lambda e: print(f"[{e['t']:6.1f}s] {e['kind']:10s} {e['text']}", flush=True)))
    try:
        plan = agent.plan(a.instruction)
        if a.execute and plan['batches']:
            agent.execute(plan, operator_ready=True)
    finally:
        if worker:worker.close()


if __name__ == '__main__':
    main()
