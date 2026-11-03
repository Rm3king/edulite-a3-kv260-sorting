#!/usr/bin/env python3
"""Single-sample (batch=1) detection latency of the production part detector, split into preprocessing / model inference /
post-processing / total, on the AMD GPU (ROCm PyTorch FP32) and on the Ryzen AI NPU (Quark A8W8, VitisAI EP).

    GPU:  aipc.sh gpu .venv-yolo/bin/python detector_latency_breakdown.py gpu --frames F... --out X.json
    NPU:  aipc.sh npu env PYTHONPATH=src/host/app python -m ... detector_latency_breakdown.py npu --frames F... --out X.json

Input: 1920x1080 head-camera colour frames (the production input), 1280-px network input, confidence 0.25, NMS IoU 0.7.
The total includes the full-resolution instance masks that the grasp-point geometry consumes.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np

APP = Path(__file__).resolve().parents[2]


def quantiles(v):
    v = sorted(v);pick = lambda p: v[min(len(v) - 1, int(p * len(v)))]
    return dict(n=len(v), p50=round(pick(.5), 2), p95=round(pick(.95), 2), p99=round(pick(.99), 2), max=round(v[-1], 2))


def gpu(a):
    import torch
    from PIL import Image
    from ultralytics import YOLO
    model = YOLO(str(Path(a.weights)))
    images = [np.asarray(Image.open(f).convert('RGB'))[:, :, ::-1] for f in a.frames]
    model.predict(images[0], imgsz=1280, device=0, verbose=False)
    pre, inf, post, total = [], [], [], []
    print('POWER_WINDOW_START', flush=True)
    for i in range(a.count):
        torch.cuda.synchronize();t = time.perf_counter()
        r = model.predict(images[i % len(images)], imgsz=1280, device=0, conf=.25, iou=.7, retina_masks=True, verbose=False)[0]
        masks = (r.masks.data.cpu().numpy() > .5) if r.masks is not None else None      # what the live detector hands to the geometry
        torch.cuda.synchronize();total.append((time.perf_counter() - t) * 1e3)
        pre.append(r.speed['preprocess']);inf.append(r.speed['inference']);post.append(r.speed['postprocess'])
    print('POWER_WINDOW_END', flush=True)
    return dict(backend='gpu_rocm_pytorch_fp32', device=torch.cuda.get_device_name(0), preprocess_ms=quantiles(pre), inference_ms=quantiles(inf),
                postprocess_ms=quantiles(post), total_ms=quantiles(total))


def npu(a):
    from workcell.npu_parts import Detector, decode, full_masks, letterbox, load_rgb
    d = Detector(a.weights, npu=True, cache=Path(a.weights).parent / 'eval_a8w8' / 'vitisai_cache')
    images = [load_rgb(f) for f in a.frames]
    d(images[0])
    pre, inf, post, total = [], [], [], []
    print('POWER_WINDOW_START', flush=True)
    for i in range(a.count):
        rgb = images[i % len(images)];t0 = time.perf_counter()
        x, r, pad = letterbox(rgb);t1 = time.perf_counter()
        o0, o1 = d.session.run(None, {d.name: x});t2 = time.perf_counter()
        dets = decode(o0, o1, r, pad, masks=True);full_masks(dets, rgb.shape[:2], r, pad);t3 = time.perf_counter()
        pre.append((t1 - t0) * 1e3);inf.append((t2 - t1) * 1e3);post.append((t3 - t2) * 1e3);total.append((t3 - t0) * 1e3)
    print('POWER_WINDOW_END', flush=True)
    return dict(backend='npu_vitisai_a8w8', providers=d.providers, preprocess_ms=quantiles(pre), inference_ms=quantiles(inf),
                postprocess_ms=quantiles(post), total_ms=quantiles(total))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('mode', choices=['gpu', 'npu']);p.add_argument('--weights', required=True);p.add_argument('--frames', nargs='+', required=True)
    p.add_argument('--count', type=int, default=100);p.add_argument('--out', required=True)
    a = p.parse_args()
    r = dict(gpu=gpu, npu=npu)[a.mode](a)
    r.update(frames=len(a.frames), samples=a.count, input='1920x1080 RGB frame, network 1280 px (736x1280 letterbox)', conf=.25, iou=.7, batch=1)
    Path(a.out).write_text(json.dumps(r, indent=2) + '\n');print(json.dumps(r))
