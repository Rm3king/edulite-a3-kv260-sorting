"""Versioned language-to-skill contract, independent of model and motor APIs."""
import json
import math
import re

VERSION = 'workcell.task.v1'
# M12 parts keep the original names; the M6 parts were added 2026-10-31 (the four classes of the head-camera detector)
CLASSES = ('screw', 'nut', 'screw_m6', 'nut_m6')
SYSTEM = '''你是螺丝分拣工作站的任务规划器。输出一个JSON，不输出解释。
结构固定：{"version":"workcell.task.v1","action":"plan","steps":[{"part_class":"screw","quantity":2,"region":"right","destination":"B"}],"exclude":["nut"],"question":""}。
action只有plan/clarify/pause/resume。plan含1到4步；其他动作steps为空。
每步字段严格为part_class(screw/nut/screw_m6/nut_m6)、quantity(1到20的整数或"all")、region(all/left/right)、destination(A/B)。
螺丝、螺钉、螺栓、大螺丝、M12螺丝是screw，螺母、大螺母、M12螺母是nut；小螺丝、M6螺丝是screw_m6，小螺母、M6螺母是nut_m6。没说左右则all。左右指操作画面左右。
"所有零件/全部零件"为四步：screw、nut、screw_m6、nut_m6，数量都是all。
"全部/所有/剩余"数量为all。没有明确数量也没有"全部"时clarify，不猜。
没有目的盒或未知盒C时clarify。未说明类别时clarify。未知物件、颜色/长短等尚不支持的属性需clarify。
"然后/再"拆成顺序步骤。"螺母不要动/留下螺母"则exclude含nut；未要求排除时exclude为空。
"剩下的"可以引用context.last_part_class；没有历史类别则clarify。
"暂停/停一下"为pause，"继续/恢复"为resume。这些动作question为空。
clarify必须用question提出简短中文问题，exclude为空。计划question为空。
对象选择、空间坐标和碰撞检查由工具执行；不能输出坐标、关节角、力矩或任意代码。'''


def validate(value):
    if isinstance(value, str):
        text = value.strip()
        if text.startswith('```'):
            text = text.split('\n', 1)[-1].rsplit('```', 1)[0].strip()
        def unique(pairs):
            result = {}
            for k, v in pairs:
                if k in result:
                    raise ValueError('DUPLICATE_KEY')
                result[k] = v
            return result
        value = json.loads(text, object_pairs_hook=unique)
    if type(value) is not dict or set(value) != {'version','action','steps','exclude','question'}:
        raise ValueError('TASK_FIELDS')
    if value['version'] != VERSION or value['action'] not in ('plan','clarify','pause','resume'):
        raise ValueError('TASK_VERSION_OR_ACTION')
    if type(value['question']) is not str or len(value['question']) > 200:
        raise ValueError('QUESTION')
    if type(value['exclude']) is not list or any(x not in CLASSES for x in value['exclude']) or len(set(value['exclude'])) != len(value['exclude']):
        raise ValueError('EXCLUSIONS')
    steps = value['steps']
    if type(steps) is not list or not 0 <= len(steps) <= 4:
        raise ValueError('STEP_COUNT')
    if value['action'] == 'plan':
        if not steps or value['question']:
            raise ValueError('EMPTY_PLAN')
    elif steps or value['exclude'] or bool(value['question']) != (value['action']=='clarify'):
        raise ValueError('NONPLAN_CONTENT')
    for step in steps:
        if type(step) is not dict or set(step) != {'part_class','quantity','region','destination'}:
            raise ValueError('STEP_FIELDS')
        if step['part_class'] not in CLASSES or step['part_class'] in value['exclude']:
            raise ValueError('EXCLUDED_OR_UNKNOWN_CLASS')
        q = step['quantity']
        if q != 'all' and not (type(q) is int and 1 <= q <= 20):
            raise ValueError('QUANTITY')
        if step['region'] not in ('all','left','right') or step['destination'] not in ('A','B'):
            raise ValueError('REGION_OR_DESTINATION')
    return value


def result(action='plan', steps=None, exclude=None, question=''):
    return validate(dict(version=VERSION,action=action,steps=steps or [],exclude=exclude or [],question=question))


def semantic_key(value):
    """Clarification wording is not an exact-match language metric."""
    v = validate(value)
    return {k:x for k,x in v.items() if k != 'question'}


def guard_instruction(value,instruction):
    """Independent check for the observed model error: inventing a quantity.

    This does not measure semantic accuracy or replace the task model. A
    structurally valid guessed count must still not become a tool request.
    """
    v=validate(value)
    explicit=re.search(r'全部|所有|剩余|剩下|其余|[0-9一二两三四五六七八九十]+\s*(?:颗|枚|个|只|件)',instruction)
    if v['action']=='plan' and explicit is None:raise ValueError('CLARIFICATION_REQUIRED_EXPLICIT_QUANTITY')
    return v


def control_intent(instruction):
    """Unambiguous operator controls bypass model inference; task requests do not.

    Exact complete phrases only: negation and compound instructions fall through
    to the normal task-understanding path. This is not a model accuracy claim.
    """
    text=re.sub(r'[\s。！!？?，,]+','',instruction)
    pause={'暂停','暂停当前操作','暂停当前任务','先停一下','先别动','停止当前任务','现在停一停'}
    resume={'继续','恢复任务','继续执行','接着执行未完成任务','接上之前的任务','恢复之前的操作'}
    if text in pause:return result('pause')
    if text in resume:return result('resume')
    return None


def bind(value, scene, verified=()):
    """Bind only current instance IDs. Does not imply physical authorization."""
    v = validate(value)
    if v['action'] != 'plan':
        return []
    if type(scene.get('scene_id')) is not str or not scene['scene_id']:
        raise ValueError('SCENE_ID')
    width = scene['image_width_px']
    if type(width) not in (int,float) or not math.isfinite(width) or width <= 0:
        raise ValueError('IMAGE_WIDTH')
    objects = scene['objects']
    ids = [x['object_id'] for x in objects]
    if len(ids) != len(set(ids)):
        raise ValueError('DUPLICATE_INSTANCE_ID')
    used = set(verified)
    actions = []
    for step in v['steps']:
        eligible = []
        for o in objects:
            if o['object_id'] in used or o['part_class'] != step['part_class']:
                continue
            u = o['center_uv_px'][0]
            if not math.isfinite(u) or not 0 <= u < width:
                raise ValueError('OBJECT_PIXEL')
            if step['region']=='left' and u>=width/2 or step['region']=='right' and u<width/2:
                continue
            eligible.append(o)
        eligible.sort(key=lambda x:(-x.get('grasp_quality',0),x['object_id']))
        n = len(eligible) if step['quantity']=='all' else step['quantity']
        if not n or len(eligible) < n:
            raise ValueError('TARGET_COUNT_INSUFFICIENT')
        for o in eligible[:n]:
            actions.append(dict(skill='pick_place',scene_id=scene['scene_id'],object_id=o['object_id'],
                                part_class=o['part_class'],destination=step['destination'],
                                geometry_status=o.get('geometry_status','UNLOCALIZED'),
                                physical_authorized=False))
            used.add(o['object_id'])
    return actions


class Ledger:
    """Verify evidence before counting; retain episode identity across retries."""
    def __init__(self, maximum_retries=2):
        self.verified=set(); self.attempts={}; self.events=[]; self.maximum_retries=maximum_retries
    def begin(self, object_id):
        if object_id in self.verified:
            raise ValueError('ALREADY_VERIFIED')
        n=self.attempts.get(object_id,0)+1
        if n>self.maximum_retries+1:
            raise ValueError('RETRY_BUDGET')
        self.attempts[object_id]=n
        self.events.append(dict(event='REQUESTED',object_id=object_id,attempt=n))
    def verify(self, object_id, outcome, evidence):
        if object_id not in self.attempts or outcome not in ('SUCCESS','FAIL','UNKNOWN') or not evidence:
            raise ValueError('VERIFICATION_EVIDENCE')
        if outcome=='SUCCESS':self.verified.add(object_id)
        self.events.append(dict(event='VERIFIED',object_id=object_id,outcome=outcome,evidence=evidence))
