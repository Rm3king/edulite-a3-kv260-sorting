"""Four-class part detector (parts_v2 YOLO11n-seg) on the Ryzen AI NPU: AMD Quark INT8 PTQ, VitisAI EP, numpy post-processing.

Why the NPU: the head-camera detector runs all the time (~24 frames/s) while the local Qwen2.5-VL model (result review,
order understanding) needs the same Radeon 890M GPU and ~8 GB of its memory. Moving the always-on perception to the NPU
leaves the GPU to the large model. The head camera only gives the coarse grasp position; the wrist camera closes the last
millimetres (continuous: +-15 mm, stop-and-look: +-35 mm), so INT8 is acceptable if the centres stay within a few mm.

Same input as production: a 1920x1080 frame scaled to 1280x720 and padded to 1280x736 (grey 114), RGB, /255.

    quantize  (.venv-quark):     python -m workcell.npu_parts quantize --model X.onnx --output DIR [--precision A8W8|A16W8]
    evaluate  (ryzen-ai-1.8.0):  python -m workcell.npu_parts evaluate --float X.onnx --quant Y.onnx --frames F1.png ... --output DIR
"""
import argparse
import hashlib
import json
import time
from pathlib import Path

import numpy as np

H, W = 736, 1280
NAMES = ('screw', 'nut', 'screw_m6', 'nut_m6')
CONF, IOU = .25, .7          # the production detector's thresholds (configs/parts_v2_20261028_model.json)


def letterbox(rgb):
    """HxWx3 uint8 RGB (1920x1080) -> (1,3,736,1280) float32 tensor, scale, (pad_x, pad_y)."""
    from PIL import Image
    h, w = rgb.shape[:2]
    r = min(W / w, H / h)
    nw, nh = round(w * r), round(h * r)
    image = Image.fromarray(rgb).resize((nw, nh), Image.Resampling.BILINEAR)
    canvas = np.full((H, W, 3), 114, np.uint8)
    px, py = (W - nw) // 2, (H - nh) // 2
    canvas[py:py + nh, px:px + nw] = np.asarray(image)
    return canvas.transpose(2, 0, 1)[None].astype(np.float32) / 255., r, (px, py)


def load_rgb(path):
    from PIL import Image
    return np.asarray(Image.open(path).convert('RGB'))


def nms(boxes, scores, iou):
    order, keep = scores.argsort()[::-1], []
    while order.size:
        i = order[0];keep.append(i)
        xx1 = np.maximum(boxes[i, 0], boxes[order[1:], 0]);yy1 = np.maximum(boxes[i, 1], boxes[order[1:], 1])
        xx2 = np.minimum(boxes[i, 2], boxes[order[1:], 2]);yy2 = np.minimum(boxes[i, 3], boxes[order[1:], 3])
        inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
        area = lambda b: (b[..., 2] - b[..., 0]) * (b[..., 3] - b[..., 1])
        order = order[1:][inter / (area(boxes[i]) + area(boxes[order[1:]]) - inter + 1e-9) <= iou]
    return keep


def decode(out0, out1, scale, pad, conf=CONF, iou=IOU, masks=True):
    """YOLO-seg ONNX outputs -> [dict(part_class, confidence, box_xyxy_px, center_uv_px, mask_center_uv_px)] in frame pixels.

    NMS per class (Ultralytics default, agnostic=False)."""
    p = out0[0].T                                     # (N, 4 + nc + 32)
    cls_scores = p[:, 4:4 + len(NAMES)]
    cls, score = cls_scores.argmax(1), cls_scores.max(1)
    sel = score > conf
    p, cls, score = p[sel], cls[sel], score[sel]
    xywh = p[:, :4]
    boxes = np.stack([xywh[:, 0] - xywh[:, 2] / 2, xywh[:, 1] - xywh[:, 3] / 2, xywh[:, 0] + xywh[:, 2] / 2, xywh[:, 1] + xywh[:, 3] / 2], 1)
    keep = []
    for c in np.unique(cls):
        idx = np.nonzero(cls == c)[0]
        keep += list(idx[nms(boxes[idx], score[idx], iou)])
    keep = sorted(keep, key=lambda i: -score[i])[:300]
    protos = out1[0]                                  # (32, H/4, W/4)
    out = []
    for i in keep:
        b = boxes[i]
        frame_box = [(b[0] - pad[0]) / scale, (b[1] - pad[1]) / scale, (b[2] - pad[0]) / scale, (b[3] - pad[1]) / scale]
        d = dict(part_class=NAMES[cls[i]], confidence=float(score[i]), box_xyxy_px=[float(v) for v in frame_box],
                 center_uv_px=[(frame_box[0] + frame_box[2]) / 2, (frame_box[1] + frame_box[3]) / 2])
        if masks:
            # the mask only inside the box (what Ultralytics keeps): proto coefficients over the box window, not the whole map
            sy, sx = protos.shape[1] / H, protos.shape[2] / W
            x0, y0 = max(0, int(b[0] * sx)), max(0, int(b[1] * sy))
            x1, y1 = min(protos.shape[2], int(np.ceil(b[2] * sx))), min(protos.shape[1], int(np.ceil(b[3] * sy)))
            window = protos[:, y0:y1, x0:x1]
            logits = (p[i, 4 + len(NAMES):] @ window.reshape(32, -1)).reshape(window.shape[1:])
            d['mask_window'] = (logits.astype(np.float32), (x0, y0, x1, y1))   # proto-resolution logits, for full-res masks
            ys, xs = np.nonzero(logits > 0.)            # sigmoid > 0.5
            xs, ys = xs + x0, ys + y0
            if len(xs):
                d['mask_center_uv_px'] = [float(((xs.mean() + .5) / sx - pad[0]) / scale), float(((ys.mean() + .5) / sy - pad[1]) / scale)]
                d['mask_area_proto_px'] = int(len(xs))
        out.append(d)
    return out


class Detector:
    """One ONNX session; VitisAI EP (NPU) with CPU fallback, or plain CPU."""
    def __init__(self, model, npu=True, cache=None):
        import onnxruntime as ort
        o = ort.SessionOptions();o.intra_op_num_threads = 4;o.log_severity_level = 3
        providers = ['CPUExecutionProvider']
        if npu:
            provider = dict(cacheDir=str(cache or Path(model).parent / 'vitisai_cache'), cacheKey=Path(model).stem)
            providers = [('VitisAIExecutionProvider', provider), 'CPUExecutionProvider']
        started = time.perf_counter()
        self.session = ort.InferenceSession(str(model), sess_options=o, providers=providers)
        self.compile_s = time.perf_counter() - started
        self.name = self.session.get_inputs()[0].name
        self.providers = self.session.get_providers()

    def __call__(self, rgb, masks=True):
        x, r, pad = letterbox(rgb)
        started = time.perf_counter()
        out0, out1 = self.session.run(None, {self.name: x})
        infer_ms = (time.perf_counter() - started) * 1e3
        return decode(out0, out1, r, pad, masks=masks), infer_ms


def full_masks(dets, shape, scale, pad):
    """Full-resolution 0/1 masks (N, h, w) as the production detector's retina_masks: each box window's logits are scaled from
    the 1/4 proto grid through the letterbox back to frame pixels (bilinear), thresholded at sigmoid 0.5, kept inside the box."""
    import cv2
    h, w = shape
    out = np.zeros((len(dets), h, w), np.uint8)
    for k, d in enumerate(dets):
        if 'mask_window' not in d:
            continue
        logits, (x0, y0, x1, y1) = d['mask_window']
        # proto pixel edges -> letterbox pixels (x4) -> frame pixels
        fx0, fy0 = (x0 * 4 - pad[0]) / scale, (y0 * 4 - pad[1]) / scale
        fx1, fy1 = (x1 * 4 - pad[0]) / scale, (y1 * 4 - pad[1]) / scale
        ix0, iy0, ix1, iy1 = max(0, int(np.floor(fx0))), max(0, int(np.floor(fy0))), min(w, int(np.ceil(fx1))), min(h, int(np.ceil(fy1)))
        if ix1 <= ix0 or iy1 <= iy0 or logits.size == 0:
            continue
        up = cv2.resize(logits, (ix1 - ix0, iy1 - iy0), interpolation=cv2.INTER_LINEAR)
        m = (up > 0).astype(np.uint8)
        bx0, by0, bx1, by1 = d['box_xyxy_px']
        keep = np.zeros_like(m)
        kx0, ky0 = max(0, int(round(bx0)) - ix0), max(0, int(round(by0)) - iy0)
        kx1, ky1 = min(m.shape[1], int(round(bx1)) - ix0), min(m.shape[0], int(round(by1)) - iy0)
        keep[ky0:ky1, kx0:kx1] = 1
        out[k, iy0:iy1, ix0:ix1] = m * keep
    return out


def serve(a):
    """NPU detection service for the live detector (another environment): one JSON line per frame.

    request {"id", "frame": /dev/shm/...npy (uint8 HxWx3 BGR)} -> reply {"id", "out": .npz (classes, scores, boxes, masks), "infer_ms"}.
    The reply's masks are full resolution (as Ultralytics retina_masks), written to the same tmpfs."""
    import sys
    d = Detector(a.quant, npu=True, cache=Path(a.quant).parent / 'eval_a8w8' / 'vitisai_cache')
    print(json.dumps(dict(event='NPU_READY', providers=d.providers, compile_s=d.compile_s, model=str(a.quant),
                          model_sha256=hashlib.sha256(Path(a.quant).read_bytes()).hexdigest())), flush=True)
    for line in sys.stdin:
        try:
            req = json.loads(line)
            if req.get('op') == 'shutdown':
                break
            bgr = np.load(req['frame'])
            rgb = np.ascontiguousarray(bgr[:, :, ::-1])
            started = time.perf_counter()
            x, r, pad = letterbox(rgb)
            out0, out1 = d.session.run(None, {d.name: x})
            dets = decode(out0, out1, r, pad, masks=True)
            masks = full_masks(dets, bgr.shape[:2], r, pad)
            infer_ms = (time.perf_counter() - started) * 1e3
            path = req['frame'].replace('.npy', '_out.npz')
            np.savez(path, classes=np.array([NAMES.index(x['part_class']) for x in dets], np.int64),
                     scores=np.array([x['confidence'] for x in dets], np.float32).reshape(-1),
                     boxes=np.array([x['box_xyxy_px'] for x in dets], np.float32).reshape(-1, 4), masks=masks)
            print(json.dumps(dict(id=req.get('id'), out=path, infer_ms=infer_ms, n=len(dets))), flush=True)
        except Exception as error:
            print(json.dumps(dict(id=None, error=f'{type(error).__name__}: {error}'[:300])), flush=True)


def quantize(a):
    """AMD Quark PTQ on our own training images (parts_v2 train split), same letterbox as production."""
    import onnx
    from onnxruntime.quantization import CalibrationDataReader
    from quark.onnx import ModelQuantizer
    from quark.onnx.quantization.config import Config, get_default_config
    out = Path(a.output);out.mkdir(parents=True, exist_ok=True)
    every = sorted(f for f in Path(a.calibration).iterdir() if f.suffix.lower() in ('.png', '.jpg', '.jpeg'))
    files = every[::max(1, len(every) // a.calibration_count)][:a.calibration_count]   # spread over the whole split
    if not files:
        raise ValueError('NO_CALIBRATION_IMAGES')
    name = onnx.load(a.model).graph.input[0].name

    class Reader(CalibrationDataReader):
        def __init__(self):
            self.it = iter(files)

        def get_next(self):
            f = next(self.it, None)
            return None if f is None else {name: letterbox(load_rgb(f))[0]}
    config = get_default_config(a.precision)
    # The head's decode (reshape/concat/DFL/box arithmetic/sigmoid) stays float: its last Concat holds box coordinates (0-1280)
    # and class scores (0-1) in one tensor, and one INT8 scale for both zeroed every score (A8W8, 2026-10-31: 216 of 216
    # detections lost). The convolutions, where the compute is, are still quantized for the NPU.
    excluded = [n.name for n in onnx.load(a.model).graph.node
                if n.name.startswith('/model.23/') and not any(k in n.name for k in ('/cv2.', '/cv3.', '/cv4.', '/proto/'))]
    config.nodes_to_exclude = excluded
    target = out / f'parts_v2_{a.precision.lower()}_736x1280.onnx'
    started = time.perf_counter()
    ModelQuantizer(Config(global_quant_config=config)).quantize_model(a.model, str(target), calibration_data_reader=Reader())
    model = onnx.load(target)
    original_ir = model.ir_version
    if model.ir_version > 10:          # the Ryzen AI ORT is older than the quantizer's
        model.ir_version = 10;onnx.checker.check_model(model);onnx.save(model, target)
    report = dict(status='PASS_AMD_QUARK_PTQ', precision=a.precision, elapsed_s=time.perf_counter() - started, input=str(a.model),
                  output=str(target), output_sha256=hashlib.sha256(target.read_bytes()).hexdigest(), original_ir=original_ir,
                  calibration=[f.name for f in files], calibration_scope='parts_v2 training split only', float_decode_nodes=len(excluded))
    (out / f'quantize_{a.precision.lower()}.json').write_text(json.dumps(report, indent=2) + '\n');print(json.dumps(report, indent=2))


def match(reference, test, max_px=40.):
    """Greedy same-class matching by centre distance: -> (pairs, unmatched_reference, unmatched_test)."""
    pairs, used = [], set()
    for r in sorted(reference, key=lambda d: -d['confidence']):
        best = None
        for j, t in enumerate(test):
            if j in used or t['part_class'] != r['part_class']:
                continue
            dist = float(np.hypot(*np.subtract(r['center_uv_px'], t['center_uv_px'])))
            if dist <= max_px and (best is None or dist < best[1]):
                best = (j, dist)
        if best:
            used.add(best[0]);pairs.append((r, test[best[0]], best[1]))
    return pairs, [r for r in reference if not any(r is p[0] for p in pairs)], [t for j, t in enumerate(test) if j not in used]


def evaluate(a):
    """NPU INT8 vs the float model (CPU) on saved live frames: detections, centres, latency, and the NPU's own profile."""
    out = Path(a.output);out.mkdir(parents=True, exist_ok=True)
    ref = Detector(a.float, npu=False)
    npu = Detector(a.quant, npu=True, cache=out / 'vitisai_cache')
    rows, times = [], dict(cpu_fp32=[], npu=[])
    for f in a.frames:
        rgb = load_rgb(f)
        r, tr = ref(rgb)
        npu(rgb)                                               # first run per shape can include lazy init
        q, tq = npu(rgb)
        times['cpu_fp32'].append(tr);times['npu'].append(tq)
        pairs, miss, extra = match(r, q)
        rows.append(dict(frame=str(f), reference=len(r), npu=len(q), matched=len(pairs), missed=[(d['part_class'], round(d['confidence'], 2)) for d in miss],
                         extra=[(d['part_class'], round(d['confidence'], 2)) for d in extra],
                         box_center_px=[round(p[2], 2) for p in pairs],
                         mask_center_px=[round(float(np.hypot(*np.subtract(p[0]['mask_center_uv_px'], p[1]['mask_center_uv_px']))), 2)
                                         for p in pairs if 'mask_center_uv_px' in p[0] and 'mask_center_uv_px' in p[1]],
                         per_class={c: [round(p[2], 2) for p in pairs if p[0]['part_class'] == c] for c in NAMES}))
    # the NPU really ran: count VitisAI partitions in a profiled session
    import onnxruntime as ort
    o = ort.SessionOptions();o.enable_profiling = True;o.profile_file_prefix = str(out / 'npu_profile');o.log_severity_level = 3
    s = ort.InferenceSession(str(a.quant), sess_options=o, providers=[('VitisAIExecutionProvider', dict(cacheDir=str(out / 'vitisai_cache'), cacheKey=Path(a.quant).stem)), 'CPUExecutionProvider'])
    s.run(None, {s.get_inputs()[0].name: letterbox(load_rgb(a.frames[0]))[0]})
    events = json.loads(Path(s.end_profiling()).read_text())
    by_ep = {}
    for e in events:
        if e.get('cat') == 'Node':
            ep = e.get('args', {}).get('provider', 'unknown');by_ep[ep] = by_ep.get(ep, 0) + 1
    all_box = [d for r in rows for d in r['box_center_px']];all_mask = [d for r in rows for d in r['mask_center_px']]
    q = lambda v, p: float(np.percentile(v, p)) if v else None
    report = dict(status='NPU_VS_FLOAT_EVALUATED', float_model=str(a.float), quant_model=str(a.quant), frames=len(rows),
                  providers=npu.providers, npu_compile_s=npu.compile_s, profile_node_events_by_provider=by_ep,
                  reference_detections=sum(r['reference'] for r in rows), matched=sum(r['matched'] for r in rows),
                  missed=sum(len(r['missed']) for r in rows), extra=sum(len(r['extra']) for r in rows),
                  box_center_px=dict(p50=q(all_box, 50), p95=q(all_box, 95), max=max(all_box) if all_box else None),
                  mask_center_px=dict(p50=q(all_mask, 50), p95=q(all_mask, 95), max=max(all_mask) if all_mask else None),
                  per_class_box_center_px_max={c: max([d for r in rows for d in r['per_class'][c]], default=None) for c in NAMES},
                  latency_ms={k: dict(p50=q(v, 50), p95=q(v, 95)) for k, v in times.items()}, rows=rows,
                  scope='quantization effect only: INT8 on the NPU vs the same float graph on the CPU, same preprocessing, live frames not in training')
    (out / 'evaluation.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'rows'}, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('mode', choices=['quantize', 'evaluate', 'serve'])
    p.add_argument('--model');p.add_argument('--output', required=True);p.add_argument('--precision', default='A8W8', choices=['A8W8', 'A16W8'])
    p.add_argument('--calibration');p.add_argument('--calibration-count', type=int, default=64)
    p.add_argument('--float');p.add_argument('--quant');p.add_argument('--frames', nargs='*')
    a = p.parse_args()
    dict(quantize=quantize, evaluate=evaluate, serve=serve)[a.mode](a)
