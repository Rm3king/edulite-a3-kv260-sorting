"""Goal-state inventory contract and deterministic reconciliation. No hardware I/O."""
import json
import math
import re

VERSION = 'workcell.inventory_goal.v1'
SYSTEM = '将用户的最终数量要求转换为JSON，仅输出JSON。\n可执行格式：{"action":"set_count","tray":"B","quantity":3}。tray只能A/B，quantity是0到5的整数。只支持螺丝最终数量，螺母不动。\n需要澄清时格式：{"action":"clarify","question":"具体的中文问题"}，不要带tray或quantity。\n右盘=A盒，左盘=B盒。“留/保留/剩/恰好有/最终有/补足/凑够”表示最终数量。“清空A盒螺丝”是A目标0。多退少补由工具根据现场决定，不输出搬运步骤。\n只说“搬N颗到B”是搬运数量，不是最终数量，必须澄清。没有数量、没有盒、未知物件或要求排列朝向/最左最上等附加条件，必须澄清，不可忽略。\n只提取用户最后确认的目标，否定的数量不能采用。'

EXAMPLES = [
    ('让B盒最终有三颗螺丝。', {'action':'set_count','tray':'B','quantity':3}),
    ('左盘留下两颗螺栓，多的还到右盘。', {'action':'set_count','tray':'B','quantity':2}),
    ('A盒的螺丝全部拿到B盒，让A盒清空。', {'action':'set_count','tray':'A','quantity':0}),
    ('搬三颗螺丝到B盒。', {'action':'clarify','question':'是搬入三颗，还是让B盒最终共三颗？'}),
    ('B盒只要两颗，不要三颗。', {'action':'set_count','tray':'B','quantity':2}),
    ('让B盒有三颗螺丝，头都朝左。', {'action':'clarify','question':'暂不支持指定螺丝头朝向，是否只调整最终数量？'}),
]


def decode_model_output(raw):
    # Model predicts the semantic goal; this adapter supplies the version and
    # invariant screw-only/preserve-nut policy, never guesses tray or quantity.
    if isinstance(raw,str):
        text=raw.strip()
        if text.startswith('```'):text=text.split('\n',1)[-1].rsplit('```',1)[0].strip()
        def unique(pairs):
            out={}
            for k,v in pairs:
                if k in out:raise ValueError('重复模型字段')
                out[k]=v
            return out
        raw=json.loads(text,object_pairs_hook=unique)
    if not isinstance(raw,dict):raise ValueError('模型目标不是JSON对象')
    if raw.get('action')=='set_count' and set(raw)=={'action','tray','quantity'}:
        return validate(dict(version=VERSION,action='reconcile',tray=raw['tray'],part_class='screw',quantity=raw['quantity'],preserve=['nut'],question=''))
    if raw.get('action')=='clarify' and set(raw)=={'action','question'}:
        return validate(dict(version=VERSION,action='clarify',tray=None,part_class=None,quantity=None,preserve=[],question=raw['question']))
    raise ValueError('模型目标字段不符合合同')



def validate(value):
    if isinstance(value, str):
        text=value.strip()
        if text.startswith('```'):text=text.split('\n',1)[-1].rsplit('```',1)[0].strip()
        def unique(pairs):
            d={}
            for k,v in pairs:
                if k in d:raise ValueError('重复目标字段')
                d[k]=v
            return d
        value=json.loads(text,object_pairs_hook=unique)
    fields={'version','action','tray','part_class','quantity','preserve','question'}
    if not isinstance(value,dict) or set(value)!=fields or value['version']!=VERSION:
        raise ValueError('目标任务合同版本或字段错误')
    if not isinstance(value['question'],str) or len(value['question'])>200:raise ValueError('目标问题格式错误')
    if value['action']=='clarify':
        if any(value[k] is not None for k in ('tray','part_class','quantity')) or value['preserve']!=[] or not value['question']:
            raise ValueError('澄清不能附带执行目标')
    elif value['action']=='reconcile':
        if (value['tray'] not in ('A','B') or value['part_class']!='screw' or
            type(value['quantity']) is not int or not 0<=value['quantity']<=5 or
            value['preserve']!=['nut'] or value['question']):
            raise ValueError('当前目标任务支持A/B盒最终0～5颗螺丝，螺母不动')
    else:raise ValueError('目标动作不支持')
    return value


def guard_goal_instruction(intent, instruction):
    """Refuse unsupported/underspecified requests independently of model output.

    This never infers a destination or a target count. Rejections are labelled
    as guard decisions, not evidence of language-model semantic correctness.
    """
    v=validate(intent)
    if v['action']!='reconcile':return v
    question=None
    mentioned=set(x.upper() for x in re.findall(r'([ABab])\s*盒',instruction))
    if re.search(r'左(?:边|侧)?(?:的)?(?:托)?盘',instruction):mentioned.add('B')
    if re.search(r'右(?:边|侧)?(?:的)?(?:托)?盘',instruction):mentioned.add('A')
    if len(mentioned)==1 and v['tray'] not in mentioned:
        question='模型识别的盒子与指令不一致，请直接指定A盒或B盒并重新理解。'
    elif re.search(r'最左|最右|最上|最下|朝向|朝[上下左右]|头.*朝|横着|竖着|平行|整齐|排列',instruction):
        question='这一版只调整最终数量，位置选择和朝向排列尚未开放；是否仅调整数量？'
    elif not re.search(r'螺丝|螺栓|螺钉',instruction):
        question='请明确要调整螺丝的最终数量。'
    elif not (re.search(r'[0-9零一二两三四五六七八九十]+\s*(?:颗|枚|个|只|件)',instruction) or '清空' in instruction):
        question='请明确任务结束后盒里应有几颗螺丝。'
    elif not re.search(r'多退少补|恰好|最终|最后|保留|留下|留[零一二两三四五六七八九十0-9]|留.*颗|补足|凑够|补到|凑到|调整到|保持|维持|总共|一共|清空|(?:让|希望|使).*(?:盒|盘).*有',instruction):
        question='是搬入指定数量，还是让盒里最终共有这个数量？'
    if question:return validate(dict(version=VERSION,action='clarify',tray=None,part_class=None,quantity=None,preserve=[],question=question))
    return v


def point_in_polygon(uv, polygon):
    x,y=uv;answer=False
    for (xa,ya),(xb,yb) in zip(polygon,polygon[1:]+polygon[:1]):
        if (ya>y)!=(yb>y) and x<(xb-xa)*(y-ya)/(yb-ya)+xa:answer=not answer
    return answer


def inventory(snapshot, config):
    """Count localized, visible detections; unknown target geometry blocks a goal."""
    trays={key:[] for key in ('A','B')};outside=[];excluded=[];ids=set()
    for obj in snapshot['objects']:
        if obj['part_class'] not in ('screw','nut'):continue
        index=obj['detection_index']
        if index in ids:raise ValueError('重复检测编号')
        ids.add(index);uv=obj.get('center_uv_px')
        if not isinstance(uv,(list,tuple)) or len(uv)!=2 or any(type(v) not in (float,int) or not math.isfinite(v) for v in uv):
            raise ValueError('检测到零件但中心定位失败，请重新观察')
        regions=[key for key in trays if point_in_polygon(uv,config['trays'][key]['roi_uv'])]
        if len(regions)>1:raise ValueError('两盘识别区域重叠')
        if not regions:outside.append(index);continue
        # The supported arm overlaps B in the image. Its tips are sometimes
        # detected as screws. Exclude only the known robot region AND depth
        # clearly above a single-layer tray part; image position alone is not
        # enough. Keep every exclusion in evidence, and reject unknown depth.
        box=config.get('excluded_robot_tip_xyxy');xyz=obj.get('camera_xyz_m')
        plane=config['trays'][regions[0]].get('plane')
        if box and plane and xyz is not None and len(xyz)==3 and all(math.isfinite(v) for v in xyz):
            height=sum(a*b for a,b in zip(plane['normal_camera'],xyz))+plane['offset_m']
            if box[0]<=uv[0]<=box[2] and box[1]<=uv[1]<=box[3] and height>.045:
                excluded.append(dict(detection_index=index,reason='KNOWN_ROBOT_REGION_ABOVE_SINGLE_LAYER',height_m=height));continue
        if obj.get('confidence',0)<config.get('inventory_min_confidence',.45):
            raise ValueError('盘内存在低置信度零件，暂不能确认数量')
        # Robot-tip exclusion is only used for selecting grasp locations. Never
        # silently remove a detected part from the count to satisfy a goal.
        trays[regions[0]].append(obj)
    return dict(trays=trays,counts={key:{cls:sum(o['part_class']==cls for o in values) for cls in ('screw','nut')} for key,values in trays.items()},outside_detection_indices=outside,robot_exclusions=excluded)


def stable_inventory(snapshots, config):
    from .live_session import unchanged
    if len(snapshots)<3 or len({s['frame_number'] for s in snapshots})!=len(snapshots):
        raise ValueError('需要至少三张不同帧核对数量')
    observations=[inventory(s,config) for s in snapshots]
    first=observations[0]
    for obs in observations[1:]:
        if obs['counts']!=first['counts'] or any(not unchanged(first['trays'][k],obs['trays'][k],8.) for k in ('A','B')):
            raise ValueError('连续图像数量或位置不稳定，请保持摆放不动后重试')
    return observations[-1]


def reconcile(goal, observation):
    goal=validate(goal)
    if goal['action']!='reconcile':raise ValueError('目标尚待澄清')
    target=goal['tray'];other='A' if target=='B' else 'B'
    existing=observation['counts'][target]['screw'];delta=goal['quantity']-existing
    result=dict(goal=goal,counts=observation['counts'],current=existing,desired=goal['quantity'],delta=delta,
                moves=abs(delta),source=other if delta>0 else target,destination=target if delta>0 else other,
                status='SATISFIED' if delta==0 else 'NEEDS_TRANSFER',shortage=0)
    available=observation['counts'][result['source']]['screw']
    if available<abs(delta):result.update(status='INSUFFICIENT_PARTS',shortage=abs(delta)-available)
    return result


def unchanged_nuts(before, after, max_pixels=10.):
    from .live_session import unchanged
    return all(unchanged([o for o in before['trays'][k] if o['part_class']=='nut'],
                         [o for o in after['trays'][k] if o['part_class']=='nut'],max_pixels) for k in ('A','B'))


def verify(goal, before, after):
    decision=reconcile(goal,after)
    total=lambda obs:sum(obs['counts'][k]['screw'] for k in ('A','B'))
    if total(before)!=total(after):return dict(status='UNKNOWN_COUNT_CHANGED',decision=decision,reason='两盘螺丝总数改变，不能把漏检或掉落当成完成')
    if not unchanged_nuts(before,after):return dict(status='PRESERVED_PART_CHANGED',decision=decision,reason='螺母位置或数量改变，需要复核')
    return dict(status='VERIFIED_COUNT_GOAL' if decision['status']=='SATISFIED' else 'GOAL_NOT_REACHED',
                decision=decision, placement_verified=False,
                verification_scope='Visible counts, screw conservation and preserved nut positions only',
                placement_note='数量符合不代表零件已完全落在盒底；搭壁、倾斜或滚动仍需落位核对。')


def describe(decision):
    counts=decision['counts'];head=f"当前 A：{counts['A']['screw']}颗螺丝；B：{counts['B']['screw']}颗螺丝。"
    if decision['status']=='SATISFIED':return head+' 数量已满足，无需调整数量；落位状态需另行核对。'
    if decision['status']=='INSUFFICIENT_PARTS':return head+f" 还缺{decision['shortage']}颗，本轮不运动。"
    return head+f" 将从{decision['source']}盒搬{decision['moves']}颗到{decision['destination']}盒；螺母保持原处。"
