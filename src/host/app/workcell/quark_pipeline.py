"""AMD Quark detector PTQ and Ryzen AI ORT deployment, in separate environments.

Quantization calibration uses training images only. Runtime verifies actual EP
profile events; merely requesting a provider is not an NPU execution claim.
"""
import argparse
import hashlib
import json
import time
from pathlib import Path
import numpy as np
from PIL import Image

def tensor(path):
    image=Image.open(path).convert('RGB');w,h=image.size;ratio=640/max(w,h);new=(round(w*ratio),round(h*ratio))
    resized=image.resize(new,Image.Resampling.BILINEAR);canvas=Image.new('RGB',(640,640),(114,114,114));canvas.paste(resized,((640-new[0])//2,(640-new[1])//2))
    return np.asarray(canvas).transpose(2,0,1)[None].astype(np.float32)/255

def quantize(a):
    import onnx
    from onnxruntime.quantization import CalibrationDataReader
    from quark.onnx import ModelQuantizer
    from quark.onnx.quantization.config import Config,get_default_config
    app=Path(a.app_root);out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    inputs=onnx.load(a.model).graph.input;name=inputs[0].name
    files=sorted((app/'reports/yolo_batch25_baseline_001/dataset/images/train').glob('*'))[:20]
    if not files:raise ValueError('NO_TRAINING_CALIBRATION_IMAGES')
    class Reader(CalibrationDataReader):
        def __init__(self):self.iterator=iter(files)
        def get_next(self):
            path=next(self.iterator,None)
            return None if path is None else {name:tensor(path)}
    start=time.perf_counter();global_config=get_default_config(a.precision)
    excluded=[]
    if a.keep_head_float:
        excluded=[n.name for n in onnx.load(a.model).graph.node if n.name.startswith('/model.23/')]
        global_config.nodes_to_exclude=excluded;global_config.per_channel=True
    config=Config(global_quant_config=global_config)
    ModelQuantizer(config).quantize_model(a.model,str(out/'detector_a8w8.onnx'),calibration_data_reader=Reader())
    model=onnx.load(out/'detector_a8w8.onnx')
    # The SDK ORT is older than the development quantizer. This graph uses only
    # opset13 tensors/attributes; record an explicit IR compatibility change.
    original_ir=model.ir_version
    if model.ir_version>10:model.ir_version=10;onnx.checker.check_model(model);onnx.save(model,out/'detector_a8w8.onnx')
    r=dict(status='PASS_AMD_QUARK_A8W8_PTQ',elapsed_s=time.perf_counter()-start,input=str(a.model),
        model_sha256=hashlib.sha256((out/'detector_a8w8.onnx').read_bytes()).hexdigest(),
        calibration=[dict(file=str(x),sha256=hashlib.sha256(x.read_bytes()).hexdigest()) for x in files],
        calibration_scope='Training split only; not five held-out replay images',original_ir=original_ir,output_ir=model.ir_version,
        precision=a.precision,excluded_nodes=excluded,per_channel=global_config.per_channel,real_motor_commands_sent=0)
    (out/'quantization.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2),flush=True)

def deploy(a):
    import onnxruntime as ort
    app=Path(a.app_root);out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    data=app/'data/screw_nut_test_v1';files=[data/x['file'] for x in json.loads((data/'manifest.json').read_text())['images']]
    options=ort.SessionOptions();options.intra_op_num_threads=4;options.log_severity_level=3
    cpu=ort.InferenceSession(a.model,sess_options=options,providers=['CPUExecutionProvider'])
    options=ort.SessionOptions();options.intra_op_num_threads=4;options.enable_profiling=True;options.profile_file_prefix=str(out/'ryzenai_profile');options.log_severity_level=3
    provider={'cacheDir':str(out/'compiler_cache'),'cacheKey':'screwseg_a8w8_v1'}
    if a.config:provider['config_file']=a.config
    start=time.perf_counter();npu=ort.InferenceSession(a.model,sess_options=options,providers=[('VitisAIExecutionProvider',provider),'CPUExecutionProvider']);compile_s=time.perf_counter()-start
    name=npu.get_inputs()[0].name;times=[];differences=[]
    for path in files:
        feed={name:tensor(path)};expected=cpu.run(None,feed);npu.run(None,feed)
        for _ in range(3):
            start=time.perf_counter();actual=npu.run(None,feed);times.append((time.perf_counter()-start)*1000)
        differences.append(dict(file=str(path),heads=[dict(shape=list(x.shape),maximum_abs_error=float(np.max(np.abs(x-y))),rmse=float(np.sqrt(np.mean((x-y)**2)))) for x,y in zip(actual,expected)]))
    profile=Path(npu.end_profiling());events=json.loads(profile.read_text());counts={};partitions=0
    for e in events:
        if e.get('cat')=='Node':
            ep=e.get('args',{}).get('provider','unknown');counts[ep]=counts.get(ep,0)+1
            if ep=='VitisAIExecutionProvider' and 'vitis_ai_ep' in e.get('name','').lower():partitions+=1
    if not partitions:raise RuntimeError('NO_ACTUAL_NPU_PARTITION_EVENTS')
    r=dict(status='PASS_ACTUAL_RYZEN_AI_NPU_DETECTOR_EXECUTION',ort_version=ort.__version__,providers=npu.get_providers(),
        compile_s=compile_s,profile_node_events=counts,npu_partition_events=partitions,times_ms=times,
        p50_ms=float(np.percentile(times,50)),p95_ms=float(np.percentile(times,95)),cpu_quantized_output_comparison=differences,
        scope='Hybrid NPU/CPU raw-output comparison on five old frames; NOT quantized mask/grasp accuracy acceptance or pure-NPU latency',motor_commands_sent=0)
    (out/'report.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['quantize','deploy']);p.add_argument('--app-root',required=True);p.add_argument('--model',required=True);p.add_argument('--output',required=True);p.add_argument('--config')
    p.add_argument('--precision',choices=['A8W8','A16W8'],default='A8W8');p.add_argument('--keep-head-float',action='store_true');a=p.parse_args()
    try:(quantize if a.mode=='quantize' else deploy)(a)
    except Exception as e:
        out=Path(a.output);out.mkdir(parents=True,exist_ok=True);(out/'failure.json').write_text(json.dumps(dict(status='FAILED_RETAINED',mode=a.mode,error=repr(e)),indent=2)+'\n');raise
