#!/usr/bin/python3
"""Local Qwen task parsing on a recorded screw scene, output is never a motor command."""
import argparse
import json
import time
from pathlib import Path
from screwcell.tasks import parse_intent,select_objects

p=argparse.ArgumentParser();p.add_argument('--app-root',type=Path,required=True);p.add_argument('--scene',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
out=a.output;out.mkdir(parents=True,exist_ok=False);app=a.app_root.resolve();scene=json.loads(a.scene.read_text())
import torch
from PIL import Image
from transformers import AutoProcessor,Qwen2_5_VLForConditionalGeneration
assert torch.cuda.is_available() and torch.version.hip
model_dir=app/'models/qwen25vl3b'
revision=json.loads((app/'configs/vlm-revision.json').read_text())
t=time.perf_counter();processor=AutoProcessor.from_pretrained(str(model_dir),local_files_only=True,min_pixels=256*28*28,max_pixels=512*28*28)
model=Qwen2_5_VLForConditionalGeneration.from_pretrained(str(model_dir),local_files_only=True,torch_dtype=torch.float16,attn_implementation='sdpa').to('cuda').eval()
load_s=time.perf_counter()-t
reject=dict(action='reject',part_class='none',count=0,region='none',destination='none')
cases=[('把两颗螺丝放到B盒，螺母不要动。',dict(action='sort',part_class='screw',count=2,region='all',destination='B')),
       ('把五颗螺丝放到B盒。',dict(action='sort',part_class='screw',count=5,region='all',destination='B')),
       ('把一颗螺丝放到C盒。',reject),('把螺丝放到B盒。',reject)]
rows=[];image=Image.open(scene['image_file']).convert('RGB')
context=[dict(object_id=o['object_id'],part_class=o['part_class']) for o in scene['objects']]
for instruction,expected in cases:
    prompt='''你是本地机械臂任务意图解析器。只返回一个JSON对象，不输出解释或思考。
允许的动作sort/reject，类别screw/nut，区域all/left/right（未说区域则all，左右指图像左右），目的盒A/B。
用户必须明确类别、数量和目的盒；数量缺失、未知盒C或其他不能表达的要求都拒绝。
只解析用户要求的数量，不擅自改少；可见目标数量不足交给后续程序检查。
正常输出字段且仅这些字段：{"action":"sort","part_class":"screw","count":2,"region":"all","destination":"B"}。
拒绝统一输出：{"action":"reject","part_class":"none","count":0,"region":"none","destination":"none"}。
不要输出像素、三维坐标、关节角或代码。
当前检测对象：'''+json.dumps(context,ensure_ascii=False)+'\n用户指令：'+instruction
    messages=[dict(role='user',content=[dict(type='image'),dict(type='text',text=prompt)])]
    text=processor.apply_chat_template(messages,tokenize=False,add_generation_prompt=True)
    inputs=processor(text=[text],images=[image],return_tensors='pt').to('cuda')
    torch.cuda.synchronize();start=time.perf_counter()
    with torch.inference_mode():ids=model.generate(**inputs,max_new_tokens=128,do_sample=False)
    torch.cuda.synchronize();elapsed=time.perf_counter()-start
    raw=processor.batch_decode(ids[:,inputs['input_ids'].shape[1]:],skip_special_tokens=True)[0]
    row=dict(instruction=instruction,raw=raw,generation_s=elapsed,expected=expected)
    try:
        value=parse_intent(raw);row.update(parsed=value,contract_valid=True,matches_development_expectation=value==expected)
        if value['action']=='sort':
            try:row['selected_object_ids']=[o['object_id'] for o in select_objects(value,scene)]
            except ValueError as exc:row['selection_rejection']=str(exc)
    except ValueError as exc:row.update(contract_valid=False,error=str(exc),matches_development_expectation=False)
    rows.append(row)
    (out/'progress.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(row,ensure_ascii=False),flush=True)
report=dict(status='LOCAL_VLM_TASK_DEVELOPMENT_CASES',model=revision,load_s=load_s,torch=torch.__version__,hip=torch.version.hip,
            gpu=torch.cuda.get_device_name(0),peak_allocated_bytes=torch.cuda.max_memory_allocated(),rows=rows,
            passed=sum(r['matches_development_expectation'] for r in rows),total=len(rows),
            motor_commands_sent=0,camera_opened=False,scope='Four development prompts on one existing scene, not general language accuracy benchmark')
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='rows'},ensure_ascii=False,indent=2))
