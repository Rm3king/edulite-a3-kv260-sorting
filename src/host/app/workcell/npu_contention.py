"""Does the detector slow down while the local VLM generates on the same GPU? GPU (PyTorch/ROCm, production) vs NPU (INT8).

    python -m workcell.npu_contention gpu  --frames F... --out X.json      (.venv-yolo, via aipc.sh gpu)
    python -m workcell.npu_contention npu  --frames F... --out X.json      (ryzen-ai, via aipc.sh npu)
    python -m workcell.npu_contention load --seconds S --out X.json        (edulite-sdk: Qwen2.5-VL reviews back to back)
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np

APP = Path(__file__).resolve().parents[2]


def summary(times):
    return dict(n=len(times), p50=float(np.percentile(times, 50)), p95=float(np.percentile(times, 95)), max=float(max(times)))


def gpu(a):
    import torch
    from ultralytics import YOLO
    model = YOLO(str(APP / 'runs/parts_v2/interim1/train/weights/best.pt'))
    from PIL import Image
    images = [np.asarray(Image.open(f).convert('RGB'))[:, :, ::-1] for f in a.frames]   # BGR, as the detector feeds cv2 frames
    model.predict(images[0], imgsz=1280, device=0, verbose=False)
    times = []
    for i in range(a.count):
        torch.cuda.synchronize();started = time.perf_counter()
        model.predict(images[i % len(images)], imgsz=1280, device=0, verbose=False, conf=.25, iou=.7)
        torch.cuda.synchronize();times.append((time.perf_counter() - started) * 1e3)
    return dict(backend='gpu_rocm_pytorch_fp32', device=torch.cuda.get_device_name(0), **summary(times))


def npu(a):
    from .npu_parts import Detector, load_rgb
    d = Detector(APP / 'reports/npu_parts_v2_20261031/parts_v2_a8w8_736x1280.onnx', npu=True, cache=APP / 'reports/npu_parts_v2_20261031/eval_a8w8/vitisai_cache')
    images = [load_rgb(f) for f in a.frames]
    d(images[0])
    times = []
    for i in range(a.count):
        started = time.perf_counter()
        d(images[i % len(images)])                  # preprocessing + NPU + float decode + masks: the whole detection
        times.append((time.perf_counter() - started) * 1e3)
    return dict(backend='npu_vitisai_a8w8', providers=d.providers, **summary(times))


def load(a):
    from .order_language import QwenWorker
    worker = QwenWorker(APP)
    frames = sorted((APP / 'reports/parts_live_20261030_211325').glob('frame_1*_color_review_*.jpg'))[:2]
    end, runs = time.monotonic() + a.seconds, []
    Path(a.out).with_suffix('.ready').write_text('ready')
    while time.monotonic() < end:
        r = worker.review('图1：执行前的源盘；图2：执行后的目标盘。\n结果：完成 4/4 件', frames, max_new_tokens=160)
        runs.append(r['generation_s'])
    worker.close()
    return dict(backend='qwen25vl_review_load', reviews=len(runs), generation_s=summary(runs))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('mode', choices=['gpu', 'npu', 'load']);p.add_argument('--frames', nargs='*', default=[]);p.add_argument('--count', type=int, default=60)
    p.add_argument('--seconds', type=float, default=120.);p.add_argument('--out', required=True)
    a = p.parse_args()
    r = dict(gpu=gpu, npu=npu, load=load)[a.mode](a)
    Path(a.out).write_text(json.dumps(r, indent=2) + '\n');print(json.dumps(r))
