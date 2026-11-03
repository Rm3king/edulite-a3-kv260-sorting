"""Batched, raw-output-preserving synthetic task evaluation on local ROCm.

The frozen test set has already informed development. Scores are engineering
regression evidence, not human-reviewed or independent real-world accuracy.
"""
import argparse
import json
import time
from pathlib import Path
from .dataset import prompt,step
from .tasks import SYSTEM,validate,semantic_key,result


def extra_cases():
    return [
        dict(id='development_composition_0',instruction='把一颗螺丝放到B盒，螺母不要动。',context={},expected=result(steps=[step(q=1)],exclude=['nut'])),
        dict(id='development_composition_1',instruction='右边的两枚螺钉移至A盒；螺母维持原处。',context={},expected=result(steps=[step(q=2,r='right',d='A')],exclude=['nut'])),
        dict(id='development_composition_2',instruction='先把一颗螺母送入B盒，再将两颗螺钉移到A盒。',context={},expected=result(steps=[step('nut',1,'all','B'),step('screw',2,'all','A')])),
        dict(id='development_composition_3',instruction='把绿色的螺母全部送去A盒。',context={},expected=result('clarify',question='颜色尚不支持。')),
        dict(id='development_composition_4',instruction='现在停一停。',context={},expected=result('pause')),
        dict(id='development_composition_5',instruction='接上之前的任务。',context={},expected=result('resume')),
        dict(id='development_composition_6',instruction='剩余的都送进A盒。',context=dict(last_part_class='nut'),expected=result(steps=[step('nut','all','all','A')])),
        dict(id='development_composition_7',instruction='请帮我分类。',context={},expected=result('clarify',question='请明确类别、数量与盒子。'))]


def run(a):
    import torch
    from transformers import AutoProcessor,Qwen2_5_VLForConditionalGeneration
    from peft import PeftModel
    app=Path(a.app_root);out=Path(a.output);out.mkdir(parents=True,exist_ok=False);torch.set_num_threads(4)
    if not torch.cuda.is_available() or not torch.version.hip:raise RuntimeError('AMD_ROCM_REQUIRED')
    tokenizer=AutoProcessor.from_pretrained(str(app/'models/qwen25vl3b'),local_files_only=True,use_fast=False).tokenizer
    tokenizer.padding_side='left'
    model=Qwen2_5_VLForConditionalGeneration.from_pretrained(str(app/'models/qwen25vl3b'),local_files_only=True,torch_dtype=torch.bfloat16,attn_implementation='sdpa').to('cuda')
    model=PeftModel.from_pretrained(model,a.adapter);model.eval()
    rows=[] if a.extra_only else [json.loads(x) for x in Path(a.data).read_text().splitlines()]
    rows+=extra_cases();answers=[]
    for first in range(0,len(rows),a.batch_size):
        batch=rows[first:first+a.batch_size]
        texts=[tokenizer.apply_chat_template([dict(role='system',content=SYSTEM),dict(role='user',content=prompt(row))],tokenize=False,add_generation_prompt=True) for row in batch]
        inputs=tokenizer(texts,return_tensors='pt',padding=True).to('cuda');torch.cuda.synchronize();begin=time.perf_counter()
        with torch.inference_mode():tokens=model.generate(**inputs,max_new_tokens=220,do_sample=False,use_cache=True,temperature=1.,top_p=1.,top_k=0)
        torch.cuda.synchronize();seconds=time.perf_counter()-begin
        for row,token in zip(batch,tokens):
            raw=tokenizer.decode(token[inputs['input_ids'].shape[1]:],skip_special_tokens=True)
            record=dict(**row,raw=raw,batch_generation_s=seconds)
            try:
                value=validate(raw);record.update(contract_valid=True,semantic_correct=semantic_key(value)==semantic_key(row['expected']))
            except (ValueError,TypeError,KeyError) as e:record.update(contract_valid=False,semantic_correct=False,error=str(e))
            answers.append(record)
        (out/'generations.json').write_text(json.dumps(answers,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(dict(processed=len(answers),total=len(rows),batch_s=seconds)),flush=True)
    report=dict(status='MEASURED_SYNTHETIC_DEVELOPMENT_REGRESSION',adapter=a.adapter,batch_size=a.batch_size,
        sets={name:dict(cases=len(s),contract_valid=sum(x['contract_valid'] for x in s),semantic_correct=sum(x['semantic_correct'] for x in s)) for name,s in [('frozen_test',[x for x in answers if not x['id'].startswith('development_')]),('development_compositions',[x for x in answers if x['id'].startswith('development_')])]},
        human_reviewed=False,test_used_in_development=True,scope='Synthetic text/tool-output regression; not real language accuracy, visual grounding or physical grasp rate',motor_commands_sent=0)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--app-root',required=True);p.add_argument('--adapter',required=True);p.add_argument('--data',required=True);p.add_argument('--output',required=True);p.add_argument('--batch-size',type=int,default=4);p.add_argument('--extra-only',action='store_true');run(p.parse_args())
