"""Actual frozen screw/nut model CPU/AMD GPU benchmark and ONNX export.

Times include preprocessing/postprocessing and result copies, fixed FP32 and
thread count. Five existing frames are a replay, not a fresh accuracy dataset.
"""
import argparse
import gzip
import hashlib
import json
import os
import shutil
import time
from pathlib import Path

def run(a):
    app=Path(a.app_root);out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    os.environ.update(YOLO_AUTOINSTALL='false',YOLO_CONFIG_DIR=str(out/'ultralytics'),MPLBACKEND='Agg')
    import torch
    import cv2
    import numpy as np
    from ultralytics import YOLO
    torch.set_num_threads(4)
    if not torch.cuda.is_available() or not torch.version.hip:raise RuntimeError('AMD_ROCM_REQUIRED')
    checkpoint=app/'reports/yolo_batch25_baseline_001/train/weights/best.pt';digest=hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    data=app/'data/screw_nut_test_v1';manifest=json.loads((data/'manifest.json').read_text());images=[cv2.imread(str(data/x['file'])) for x in manifest['images']]
    results={};examples={}
    for device,name in [('cpu','CPU_FP32'),(0,'AMD_GPU_FP32')]:
        model=YOLO(str(checkpoint));times=[]
        def predict(im):
            r=model.predict(im,device=device,imgsz=640,conf=.25,iou=.7,retina_masks=True,quantize=32,half=False,verbose=False)[0]
            return dict(classes=r.boxes.cls.cpu().numpy().astype(int).tolist(),boxes=r.boxes.xyxy.cpu().numpy().tolist(),scores=r.boxes.conf.cpu().numpy().tolist())
        for _ in range(3):predict(images[0])
        for _ in range(a.repeats):
            for i,image in enumerate(images):
                if device!='cpu':torch.cuda.synchronize()
                begin=time.perf_counter_ns();r=predict(image)
                if device!='cpu':torch.cuda.synchronize()
                times.append((time.perf_counter_ns()-begin)/1e6);examples[str(i)]=r
        results[name]=dict(samples=len(times),times_ms=times,p50_ms=float(np.percentile(times,50)),p95_ms=float(np.percentile(times,95)),predictions=dict(examples))
        if device!='cpu':
            with torch.profiler.profile(activities=[torch.profiler.ProfilerActivity.CPU,torch.profiler.ProfilerActivity.CUDA]) as trace:predict(images[0]);torch.cuda.synchronize()
            trace.export_chrome_trace(str(out/'rocm_detector_trace.json'))
            events=json.loads((out/'rocm_detector_trace.json').read_text()).get('traceEvents',[])
            results[name]['gpu_kernel_events']=sum(x.get('cat','').lower() in ('kernel','gpu_kernel') for x in events)
            with gzip.open(out/'rocm_detector_trace.json.gz','wb',compresslevel=3) as f:f.write((out/'rocm_detector_trace.json').read_bytes())
        del model
        if device!='cpu':torch.cuda.empty_cache()
    export={}
    try:
        shutil.copy2(checkpoint,out/'detector.pt');model=YOLO(str(out/'detector.pt'))
        path=model.export(format='onnx',imgsz=640,opset=13,dynamic=False,simplify=False,device='cpu',half=False)
        export=dict(status='PASS_FP32_ONNX_EXPORT',path=str(path),sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())
    except Exception as e:export=dict(status='FAILED_EXPORT',error=repr(e))
    assert hashlib.sha256(checkpoint.read_bytes()).hexdigest()==digest
    r=dict(status='PASS_AMD_ROCM_DETECTOR_REPLAY_BENCHMARK',model_sha256=digest,checkpoint_unchanged=True,
        torch=torch.__version__,hip=torch.version.hip,gpu=torch.cuda.get_device_name(0),threads=4,precision='FP32_BOTH',results=results,
        median_speedup=results['CPU_FP32']['p50_ms']/results['AMD_GPU_FP32']['p50_ms'],onnx=export,
        scope='Existing five saved test images; includes pre/postprocessing, excludes camera capture; not independent accuracy or production FPS',motor_commands_sent=0)
    (out/'report.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--app-root',required=True);p.add_argument('--output',required=True);p.add_argument('--repeats',type=int,default=3);run(p.parse_args())
