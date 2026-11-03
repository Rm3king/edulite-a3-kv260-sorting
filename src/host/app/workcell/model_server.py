"""Resident ROCm VLM JSON-lines worker. No motion or network API."""
import contextlib
import argparse
import json
import sys
import time
from pathlib import Path
from .tasks import SYSTEM,validate,bind,guard_instruction,control_intent

REVIEW_SYSTEM='''你是螺丝分拣工作站的结果复核员。你会看到头部相机的俯视图（事实第一行说明每张图是什么）和系统记录的事实。
用中文写2到3句结果报告：先说完成了几件；再说目标盘里的零件（数量以事实中的头部相机检测为准，你看图描述外观）与记录是否一致。
全部完成时不要给建议。有未完成的零件时，只用事实里写的原因解释，并给一条具体建议。
只依据图片和事实，不编造原因和数字；图片看不清或与记录不一致时直接说明。不要输出坐标、关节角、力矩或任何运动指令。'''


DESCRIBE_SYSTEM='你是视觉助手。只根据图片回答问题，看不清就说看不清，不要编造。'


def review(request,processor,model,adapter,torch):
    """Post-run result review (2026-10-31): images + recorded facts -> short Chinese report. Read-only; never a command."""
    from PIL import Image
    images=[Image.open(f).convert('RGB') for f in request.get('image_files',[])[:2]]
    content=[dict(type='image') for _ in images]+[dict(type='text',text=(request['facts'] if request.get('mode')=='describe' else '系统记录的事实：\n'+request['facts']))]
    system=DESCRIBE_SYSTEM if request.get('mode')=='describe' else REVIEW_SYSTEM
    text=processor.apply_chat_template([dict(role='system',content=system),dict(role='user',content=content)],tokenize=False,add_generation_prompt=True)
    inputs=(processor(text=[text],images=images,return_tensors='pt') if images else processor.tokenizer(text,return_tensors='pt')).to('cuda')
    torch.cuda.synchronize();started=time.perf_counter()
    # the LoRA was trained for the task JSON contract; the review uses the base model
    with (model.disable_adapter() if adapter else contextlib.nullcontext()),torch.inference_mode():
        tokens=model.generate(**inputs,max_new_tokens=int(request.get('max_new_tokens',180)),do_sample=False,use_cache=True,temperature=1.,top_p=1.,top_k=0)
    torch.cuda.synchronize()
    summary=processor.tokenizer.decode(tokens[0,inputs['input_ids'].shape[1]:],skip_special_tokens=True).strip()
    return dict(event='REVIEW_RESULT',request_id=request.get('request_id'),summary=summary,generation_s=time.perf_counter()-started,
                images_used=len(images),motor_commands_sent=0,physical_authorized=False)


def serve(a):
    import torch
    from transformers import AutoProcessor,Qwen2_5_VLForConditionalGeneration
    app=Path(a.app_root);torch.set_num_threads(4)
    if not torch.cuda.is_available() or not torch.version.hip:raise RuntimeError('AMD_ROCM_REQUIRED')
    started=time.perf_counter();processor=AutoProcessor.from_pretrained(str(app/'models/qwen25vl3b'),local_files_only=True,use_fast=False,min_pixels=128*28*28,max_pixels=384*28*28)
    model=Qwen2_5_VLForConditionalGeneration.from_pretrained(str(app/'models/qwen25vl3b'),local_files_only=True,torch_dtype=torch.bfloat16,attn_implementation='sdpa').to('cuda')
    if a.adapter:
        from peft import PeftModel
        model=PeftModel.from_pretrained(model,a.adapter)
    model.eval()
    print(json.dumps(dict(event='MODEL_READY',gpu=torch.cuda.get_device_name(0),hip=torch.version.hip,load_s=time.perf_counter()-started,adapter=a.adapter,physical_authorized=False)),flush=True)
    for line in sys.stdin:
        request_id=None;raw=None;instruction=None
        try:
            request=json.loads(line);request_id=request.get('request_id')
            if request.get('op')=='shutdown':break
            if request.get('op')=='review':
                print(json.dumps(review(request,processor,model,a.adapter,torch),ensure_ascii=False),flush=True)
                continue
            instruction=request['instruction'];context=request.get('context',{})
            control=control_intent(instruction)
            if control is not None:
                print(json.dumps(dict(event='MODEL_RESULT',request_id=request_id,instruction=instruction,raw=json.dumps(control,ensure_ascii=False),intent=control,actions=[],generation_s=0.,visual_input_used=False,motor_commands_sent=0,physical_authorized=False,source='OPERATOR_CONTROL_NO_MODEL'),ensure_ascii=False),flush=True)
                continue
            content=[dict(type='text',text='context='+json.dumps(context,ensure_ascii=False,separators=(',',':'))+'\n指令：'+instruction)]
            image=None
            if request.get('image_file'):
                from PIL import Image
                image=Image.open(request['image_file']).convert('RGB');content.insert(0,dict(type='image'))
            goal_mode=request.get('contract')=='inventory_goal'
            if goal_mode:
                from .inventory_goal import SYSTEM as goal_system,decode_model_output as validate_goal,guard_goal_instruction,EXAMPLES
            messages=[dict(role='system',content=goal_system if goal_mode else SYSTEM),dict(role='user',content=content)]
            if goal_mode:
                examples=[]
                for prompt,answer in EXAMPLES:
                    examples.extend([dict(role='user',content=prompt),dict(role='assistant',content=json.dumps(answer,ensure_ascii=False,separators=(',',':')))])
                messages=messages[:1]+examples+messages[1:]
            text=processor.apply_chat_template(messages,tokenize=False,add_generation_prompt=True)
            inputs=(processor(text=[text],images=[image],return_tensors='pt') if image else processor.tokenizer(text,return_tensors='pt')).to('cuda')
            torch.cuda.synchronize();started=time.perf_counter()
            # The existing LoRA was trained for v1 move-count tasks. Use the
            # same local base model for the new contract, no new weights/env.
            adapter_context=model.disable_adapter() if goal_mode and a.adapter else contextlib.nullcontext()
            with adapter_context,torch.inference_mode():tokens=model.generate(**inputs,max_new_tokens=220,do_sample=False,use_cache=True,temperature=1.,top_p=1.,top_k=0,prompt_lookup_num_tokens=10)  # JSON copies the schema from the prompt: 9.5 s -> 3.8 s, identical greedy output (measured, text-only)
            torch.cuda.synchronize();raw=processor.tokenizer.decode(tokens[0,inputs['input_ids'].shape[1]:],skip_special_tokens=True)
            model_value=validate_goal(raw) if goal_mode else validate(raw)
            value=guard_goal_instruction(model_value,instruction) if goal_mode else guard_instruction(model_value,instruction)
            actions=bind(value,request['scene'],context.get('verified_object_ids',[])) if request.get('scene') and not goal_mode else []
            print(json.dumps(dict(event='MODEL_RESULT',request_id=request_id,instruction=instruction,raw=raw,intent=value,actions=actions,
                generation_s=time.perf_counter()-started,intent_guard_applied=value!=model_value,visual_input_used=image is not None,motor_commands_sent=0,physical_authorized=False),ensure_ascii=False),flush=True)
        except Exception as e:print(json.dumps(dict(event='MODEL_ERROR',request_id=request_id,instruction=instruction,raw=raw,error=str(e)),ensure_ascii=False),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--app-root',required=True);p.add_argument('--adapter');serve(p.parse_args())
