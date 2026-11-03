"""Task intent validation and result accounting, independent of model backend."""
import json
import re
from . import wire

def parse_intent(raw):
    wire.require(type(raw) is str and len(raw)<8192,'invalid model output')
    text=raw.strip()
    if text.startswith('```'):
        text=re.sub(r'^```(?:json)?\s*','',text);text=re.sub(r'\s*```$','',text)
    try:value=json.loads(text,object_pairs_hook=wire.unique_object)
    except (ValueError,TypeError) as exc:raise wire.ProtocolError('model output is not one JSON object') from exc
    wire.require(type(value) is dict,'intent must be an object')
    wire.payload_fields(value,['action','part_class','count','region','destination'])
    wire.require(value['action'] in ('sort','reject'),'unknown action')
    wire.require(value['part_class'] in ('screw','nut','none'),'unknown part class')
    wire.integer(value['count'],0,20)
    wire.require(value['region'] in ('all','left','right','none'),'unknown region')
    wire.require(value['destination'] in ('A','B','none'),'unknown destination')
    if value['action']=='sort':
        wire.require(value['part_class']!='none' and value['count']>0 and value['region']!='none' and value['destination']!='none','incomplete sort intent')
    else:
        wire.require(value==dict(action='reject',part_class='none',count=0,region='none',destination='none'),'invalid reject intent')
    return value

def select_objects(intent,scene):
    wire.require(intent['action']=='sort','intent rejected')
    width=scene['image_width_px']
    chosen=[]
    for obj in scene['objects']:
        if obj['part_class']!=intent['part_class']:continue
        x=obj['center_uv_px'][0]
        if intent['region']=='left' and x>=width/2:continue
        if intent['region']=='right' and x<width/2:continue
        if not obj.get('geometry_valid',False):continue
        chosen.append(obj)
    chosen.sort(key=lambda x:x['object_id'])
    wire.require(len(chosen)>=intent['count'],'TARGET_COUNT_INSUFFICIENT')
    # Left/right is explicitly image-view coordinates in this version.
    return chosen[:intent['count']]

class Workflow:
    def __init__(self,maximum_retries=2):
        self.state='OBSERVE';self.verified=set();self.attempts={};self.maximum_retries=maximum_retries
    def start(self,object_id):
        wire.require(self.state in ('OBSERVE','NEXT','RETRY'),'workflow not ready')
        wire.require(object_id not in self.verified,'object already counted')
        self.current=object_id;self.attempts[object_id]=self.attempts.get(object_id,0)+1
        wire.require(self.attempts[object_id]<=self.maximum_retries+1,'retry budget exceeded')
        self.state='WAIT_EXECUTION'
    def motion_done(self):
        wire.require(self.state=='WAIT_EXECUTION','unexpected motion completion')
        self.state='VERIFY'
    def verify(self,result,evidence_id):
        wire.require(self.state=='VERIFY','verify only after motion')
        wire.require(result in ('SUCCESS','FAIL','UNKNOWN'),'invalid result')
        wire.identifier(evidence_id)
        if result=='SUCCESS':self.verified.add(self.current);self.state='NEXT'
        else:self.state='RETRY' if self.attempts[self.current]<=self.maximum_retries else 'FAILED'
        return self.state

def planning_fixture(scene_id='test_scene',object_id='screw_01',model_id='MOCK_MODEL',deadline=0,task_id='pick_001'):
    import math
    return dict(task_id=task_id,scene_id=scene_id,object_id=object_id,model_id=model_id,
        calibration_id='UNSET',layout_id='UNSET',tool_id='UNSET',frame='robot_base',
        grasp_tcp=dict(position_m=[-.17,.12,.10],quaternion_xyzw=[0,0,math.sin(math.pi/8),math.cos(math.pi/8)]),
        place_tcp=dict(position_m=[-.17,.12,.18],quaternion_xyzw=[0,0,math.sin(math.pi/8),math.cos(math.pi/8)]),
        approach_height_m=.08,lift_height_m=.08,gripper_preset_id='release_20261019_empty',part_class='screw',
        valid_until_kv_monotonic_ns=deadline,
        planning_start_joint_rad=[math.radians(x) for x in [-60,10,-15,15,20,-10]]+[1.6])


def validate_instruction(raw,instruction):
    """Check explicit user constraints independently of the language model.

    Supported v1 Chinese requests contain a numbered class and A/B destination.
    Missing slots ask for clarification instead of accepting a guessed count.
    """
    value=parse_intent(raw)
    primary=re.split(r'[，,。]',instruction)[0]
    wire.require(not any(term in primary for term in ('不要','禁止','别把','不许')), 'NEGATED_ACTION_REQUIRES_CLARIFICATION')
    wire.require(not any(term in instruction for term in ('然后','接着','再把','至少','至多','最多','最少')), 'MULTISTEP_OR_QUANTITY_QUALIFIER_NOT_SUPPORTED')
    count_words={'一':1,'二':2,'两':2,'三':3,'四':4,'五':5,'六':6,'七':7,'八':8,'九':9,'十':10}
    match=re.search(r'(\d{1,2}|[一二两三四五六七八九十])\s*[颗个枚只]?\s*(螺丝|螺钉|螺栓|螺母)',instruction)
    wire.require(match is not None,'CLARIFICATION_REQUIRED_EXPLICIT_COUNT_AND_CLASS')
    count=int(match[1]) if match[1].isdigit() else count_words[match[1]]
    category='nut' if match[2]=='螺母' else 'screw'
    # Reject multiple positive numbered actions until a multi-step schema exists.
    wire.require(len(re.findall(r'(?:\d{1,2}|[一二两三四五六七八九十])\s*[颗个枚只]?\s*(?:螺丝|螺钉|螺栓|螺母)',instruction))==1,'CLARIFICATION_REQUIRED_MULTIPLE_ACTIONS')
    target=re.search(r'(?:放到|放进|放入|放在|放至|送到|移到|搬到)\s*([A-Za-z])\s*盒',instruction)
    wire.require(target is not None and target[1] in ('A','B'),'INVALID_OR_MISSING_DESTINATION')
    region='right' if '右侧' in instruction or '右边' in instruction else 'left' if '左侧' in instruction or '左边' in instruction else 'all'
    # Region is optional in the user command. Its documented default is all;
    # never let a model's image-location description invent a narrower filter.
    if region=='all':value=dict(value,region='all')
    expected=dict(action='sort',part_class=category,count=count,region=region,destination=target[1])
    wire.require(value==expected,'MODEL_INTENT_DISAGREES_WITH_EXPLICIT_INSTRUCTION')
    return value
