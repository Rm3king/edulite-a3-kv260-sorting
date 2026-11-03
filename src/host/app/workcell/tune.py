"""ROCm Qwen instruction LoRA pilot. Text/tool adaptation, not grasp-policy training.

The base checkpoint stays local and frozen. Outputs are a separate adapter,
raw before/after generations, supervised losses and a real HIP profiler trace.
"""
import argparse
import contextlib
import gzip
import hashlib
import json
import random
import time
from pathlib import Path
from .dataset import prompt
from .tasks import SYSTEM, validate, semantic_key

def run(a):
    import torch
    from transformers import AutoProcessor, Qwen2_5_VLForConditionalGeneration
    from peft import LoraConfig, get_peft_model, PeftModel
    if not torch.cuda.is_available() or not torch.version.hip:raise RuntimeError('ROCM_GPU_REQUIRED')
    torch.set_num_threads(4);torch.manual_seed(20261020)
    root=Path(a.output);root.mkdir(parents=True,exist_ok=False)
    data=Path(a.data);app=Path(a.app_root);model_dir=app/'models/qwen25vl3b'
    basefiles={str(x.name):hashlib.sha256(x.read_bytes()).hexdigest() for x in model_dir.glob('*.safetensors')}
    revision=json.loads((app/'configs/vlm-revision.json').read_text())
    processor=AutoProcessor.from_pretrained(str(model_dir),local_files_only=True,use_fast=False)
    tokenizer=processor.tokenizer
    model=Qwen2_5_VLForConditionalGeneration.from_pretrained(str(model_dir),local_files_only=True,
            torch_dtype=torch.bfloat16,attn_implementation='sdpa').to('cuda')
    rows={s:[json.loads(x) for x in (data/(s+'.jsonl')).read_text().splitlines()] for s in ['train','validation','test']}
    # Freeze IDs before any training; no validation-driven selection on these IDs.
    rng=random.Random(3187);test=rng.sample(rows['test'],min(a.eval_cases,len(rows['test'])))
    (root/'evaluation_ids.json').write_text(json.dumps([x['id'] for x in test],indent=2)+'\n')
    def messages(row, assistant=False):
        m=[dict(role='system',content=SYSTEM),dict(role='user',content=prompt(row))]
        if assistant:m.append(dict(role='assistant',content=json.dumps(row['expected'],ensure_ascii=False,separators=(',',':'))))
        return m
    def evaluate(name):
        model.eval();answers=[]
        for i,row in enumerate(test):
            text=tokenizer.apply_chat_template(messages(row),tokenize=False,add_generation_prompt=True)
            inputs=tokenizer(text,return_tensors='pt').to('cuda')
            torch.cuda.synchronize();begin=time.perf_counter()
            with torch.inference_mode():
                tokens=model.generate(**inputs,max_new_tokens=200,do_sample=False,use_cache=True,
                                      temperature=1.,top_p=1.,top_k=0)
            torch.cuda.synchronize();elapsed=time.perf_counter()-begin
            raw=tokenizer.decode(tokens[0,inputs['input_ids'].shape[1]:],skip_special_tokens=True)
            r=dict(id=row['id'],instruction=row['instruction'],expected=row['expected'],raw=raw,elapsed_s=elapsed)
            try:
                parsed=validate(raw);r.update(contract_valid=True,semantic_correct=semantic_key(parsed)==semantic_key(row['expected']))
            except (ValueError,TypeError,KeyError) as e:r.update(contract_valid=False,semantic_correct=False,error=str(e))
            answers.append(r)
            (root/(name+'.json')).write_text(json.dumps(answers,ensure_ascii=False,indent=2)+'\n')
            print(json.dumps(dict(stage=name,index=i+1,total=len(test),correct=r['semantic_correct'],seconds=elapsed)),flush=True)
        return dict(cases=len(answers),contract_valid=sum(x['contract_valid'] for x in answers),
                    semantic_correct=sum(x['semantic_correct'] for x in answers),mean_s=sum(x['elapsed_s'] for x in answers)/len(answers))
    if a.initial_adapter:model=PeftModel.from_pretrained(model,a.initial_adapter,is_trainable=True)
    base=evaluate('initial_adapter' if a.initial_adapter else 'base')
    if not a.initial_adapter:
        lora=LoraConfig(r=8,lora_alpha=16,lora_dropout=.05,target_modules=['q_proj','v_proj'],bias='none',task_type='CAUSAL_LM')
        model=get_peft_model(model,lora)
    trainable=sum(p.numel() for p in model.parameters() if p.requires_grad)
    model.gradient_checkpointing_enable(gradient_checkpointing_kwargs={'use_reentrant':False})
    model.enable_input_require_grads();model.config.use_cache=False;model.train()
    optimizer=torch.optim.AdamW([p for p in model.parameters() if p.requires_grad],lr=a.learning_rate,weight_decay=.01)
    losses=[];train_order=list(rows['train']);rng.shuffle(train_order);started=time.perf_counter()
    for i in range(a.steps):
        if a.balanced:
            category=rng.choices(['positive','exclude','sequence','clarify','control','context'],weights=[35,20,15,15,10,5])[0]
            candidates=[r for r in train_order if ('positive' if r['family'].startswith('positive') else 'control' if r['family'] in ('pause','resume','control') else r['family'])==category]
            row=rng.choice(candidates)
        else:row=train_order[i%len(train_order)]
        before=tokenizer.apply_chat_template(messages(row),tokenize=False,add_generation_prompt=True)
        whole=tokenizer.apply_chat_template(messages(row,True),tokenize=False,add_generation_prompt=False)
        inputs=tokenizer(whole,return_tensors='pt').to('cuda');labels=inputs['input_ids'].clone()
        prefix=tokenizer(before,return_tensors='pt')['input_ids'].shape[1]
        labels[:,:prefix]=-100
        if inputs['input_ids'].shape[1]>1536 or (labels!=-100).sum()==0:raise ValueError('SUPERVISION_LENGTH')
        optimizer.zero_grad(set_to_none=True)
        profiler=torch.profiler.profile(activities=[torch.profiler.ProfilerActivity.CPU,torch.profiler.ProfilerActivity.CUDA]) if i==0 else contextlib.nullcontext()
        begin=time.perf_counter()
        with profiler as trace:
            loss=model(**inputs,labels=labels,use_cache=False).loss
            if not torch.isfinite(loss):raise RuntimeError('NONFINITE_LOSS')
            loss.backward();torch.nn.utils.clip_grad_norm_([p for p in model.parameters() if p.requires_grad],1.)
            optimizer.step()
        torch.cuda.synchronize()
        if i==0:
            trace.export_chrome_trace(str(root/'rocm_training_trace.json'))
            (root/'operators.txt').write_text(trace.key_averages().table(sort_by='self_cuda_time_total',row_limit=35))
        record=dict(step=i+1,id=row['id'],family=row['family'],loss=float(loss.detach()),seconds=time.perf_counter()-begin,supervised_tokens=int((labels!=-100).sum()))
        losses.append(record);(root/'losses.json').write_text(json.dumps(losses,indent=2)+'\n');print(json.dumps(dict(stage='train',**record)),flush=True)
    train_s=time.perf_counter()-started
    model.save_pretrained(str(root/'adapter'),safe_serialization=True)
    model.gradient_checkpointing_disable();model.config.use_cache=True
    adapted=evaluate('adapted')
    assert basefiles=={str(x.name):hashlib.sha256(x.read_bytes()).hexdigest() for x in model_dir.glob('*.safetensors')}
    trace=json.loads((root/'rocm_training_trace.json').read_text())
    events=trace.get('traceEvents',[]);kernels=[x for x in events if str(x.get('cat','')).lower() in ('kernel','gpu_kernel')]
    report=dict(status='PASS_ROCM_LORA_TRAIN_SAVE_RELOADABLE_ARTIFACT',model=revision,base_checkpoint_unchanged=True,
        torch=torch.__version__,hip=torch.version.hip,gpu=torch.cuda.get_device_name(0),steps=a.steps,train_s=train_s,initial_adapter=a.initial_adapter,balanced=a.balanced,learning_rate=a.learning_rate,
        trainable_parameters=trainable,peak_allocated_bytes=torch.cuda.max_memory_allocated(),before=base,after=adapted,
        actual_gpu_kernel_events=len(kernels),scope='Text instruction/tool JSON pilot on synthetic held-out wording; not visual adaptation, human language benchmark or robot policy',
        deploy_recommendation='CANDIDATE_ONLY_REQUIRES_HUMAN_REVIEW_AND_REAL_SCENE_EVALUATION',motor_commands_sent=0)
    (root/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
    with (root/'rocm_training_trace.json').open('rb') as src,gzip.open(root/'rocm_training_trace.json.gz','wb',compresslevel=3) as dst:dst.write(src.read())

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--app-root',required=True);p.add_argument('--data',required=True);p.add_argument('--output',required=True)
    p.add_argument('--steps',type=int,default=96);p.add_argument('--eval-cases',type=int,default=36)
    p.add_argument('--initial-adapter');p.add_argument('--balanced',action='store_true');p.add_argument('--learning-rate',type=float,default=2e-4);run(p.parse_args())
