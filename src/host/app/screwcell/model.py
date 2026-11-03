"""Resident local VLM adapter. Heavy dependencies are imported only on construction."""
import json
import time
from pathlib import Path
from .tasks import validate_instruction,select_objects

class LocalTaskModel:
    def __init__(self,app_root):
        import torch
        from transformers import AutoProcessor,Qwen2_5_VLForConditionalGeneration
        self.torch=torch;app=Path(app_root).resolve()
        if not torch.cuda.is_available() or not torch.version.hip:
            raise RuntimeError('Verified AMD ROCm environment required')
        self.revision=json.loads((app/'configs/vlm-revision.json').read_text())
        model_dir=app/'models/qwen25vl3b';start=time.perf_counter()
        self.processor=AutoProcessor.from_pretrained(str(model_dir),local_files_only=True,use_fast=False,min_pixels=256*28*28,max_pixels=512*28*28)
        self.model=Qwen2_5_VLForConditionalGeneration.from_pretrained(str(model_dir),local_files_only=True,torch_dtype=torch.float16,attn_implementation='sdpa').to('cuda').eval()
        self.load_s=time.perf_counter()-start
    def infer(self,instruction,scene):
        from PIL import Image
        torch=self.torch
        context=[dict(object_id=o['object_id'],part_class=o['part_class']) for o in scene['objects']]
        prompt='''你是本地机械臂任务意图解析器。只返回一个JSON对象，不输出解释或思考。
允许的动作sort/reject，类别screw/nut，区域all/left/right（未说区域则all，左右指图像左右），目的盒A/B。
用户必须明确类别、数量和目的盒；数量缺失、未知盒C或其他不能表达的要求都拒绝。
只解析用户要求的数量，不擅自改少；可见目标数量不足交给后续程序检查。
正常输出字段且仅这些字段：{"action":"sort","part_class":"screw","count":2,"region":"all","destination":"B"}。
拒绝统一输出：{"action":"reject","part_class":"none","count":0,"region":"none","destination":"none"}。
不要输出像素、三维坐标、关节角或代码。
当前检测对象：'''+json.dumps(context,ensure_ascii=False)+'\n用户指令：'+instruction
        messages=[dict(role='user',content=[dict(type='image'),dict(type='text',text=prompt)])]
        text=self.processor.apply_chat_template(messages,tokenize=False,add_generation_prompt=True)
        image=Image.open(scene['image_file']).convert('RGB')
        inputs=self.processor(text=[text],images=[image],return_tensors='pt').to('cuda')
        torch.cuda.synchronize();start=time.perf_counter();start_ns=time.monotonic_ns()
        with torch.inference_mode():
            ids=self.model.generate(**inputs,max_new_tokens=128,do_sample=False,temperature=1.0,top_p=1.0)
        torch.cuda.synchronize();elapsed=time.perf_counter()-start
        raw=self.processor.batch_decode(ids[:,inputs['input_ids'].shape[1]:],skip_special_tokens=True)[0]
        result=dict(instruction=instruction,raw=raw,model=self.revision,load_s=self.load_s,generation_s=elapsed,
            inference_start_ns=start_ns,inference_end_ns=time.monotonic_ns(),scene_id=scene['scene_id'],
            camera_opened=False,motor_commands_sent=0,physical_execution_allowed=False)
        try:
            intent=validate_instruction(raw,instruction);selected=select_objects(intent,scene)
            result.update(decision='READY_FOR_FRESH_LOCALIZATION',intent=intent,selected_object_ids=[x['object_id'] for x in selected])
        except ValueError as exc:result.update(decision='REJECTED_OR_CLARIFY',reason=str(exc))
        return result
