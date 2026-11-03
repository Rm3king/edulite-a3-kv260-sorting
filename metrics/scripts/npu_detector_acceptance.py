"""NPU detector acceptance at the grasp-target level: the production GPU prediction saved with each snapshot vs the NPU
prediction of the same colour frame, both through the same locate_parts (depth -> base-frame surface point).

    aipc.sh gpu .venv-yolo/bin/python src/npu_detector_acceptance.py --out DIR [--limit N]

Reports per class: detections matched, |base_surface_xyz| difference in mm, grasp-allowed agreement. No camera, no robot.
"""
import argparse
import glob
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from detect_parts_live import NpuDetector
from detect_parts_rgbd_live import CALIBRATION, ROOT
from live_part_geometry import load_transform, locate_parts

NAMES = {0: 'screw', 1: 'nut', 2: 'screw_m6', 3: 'nut_m6'}
CHECKPOINT = 'bbe3f4000909d5ea'   # runs/parts_v2/interim1 best.pt, the production model and the NPU model's source


def frames(limit):
    out = []
    for points in sorted(glob.glob(str(ROOT / 'aipc_app/reports/parts_live_202610*/frame_*_points.json'))):
        if 'm6view' in points or points.endswith('.raw.json'):
            continue
        stem = points[:-len('_points.json')]
        session = Path(points).parent / 'report.json'
        # only sessions run with the checkpoint the NPU model was quantized from (older sessions saved the 2-class model's output)
        if (Path(stem + '_depth.npy').exists() and Path(stem + '_prediction.npz').exists() and Path(stem + '_color.png').exists() and session.exists()
                and json.loads(session.read_text()).get('detector_sha256', '').startswith(CHECKPOINT)):
            out.append((stem, session))
    step = max(1, len(out) // limit)
    return out[::step][:limit]


def surface(objects):
    return [o for o in objects if o.get('base_xyz_m') is not None]


def main():
    ap = argparse.ArgumentParser(description=__doc__);ap.add_argument('--out', type=Path, required=True);ap.add_argument('--limit', type=int, default=80)
    a = ap.parse_args();a.out.mkdir(parents=True, exist_ok=True)
    calibration, matrix = load_transform(CALIBRATION)
    policy = json.loads((ROOT / 'aipc_app/configs/parts_depth_draft_v1.json').read_text())
    npu = NpuDetector()
    rows, diffs, times = [], {c: [] for c in NAMES.values()}, []
    try:
        for stem, session in frames(a.limit):
            live = json.loads(session.read_text())
            scale, intrinsics = live.get('depth_scale_m'), live.get('intrinsics')   # what the live detector used for this frame
            if not scale or not intrinsics:
                continue
            bgr = cv2.imread(stem + '_color.png');depth = np.load(stem + '_depth.npy')
            with np.load(stem + '_prediction.npz') as z:
                gpu_prediction = {k: z[k] for k in ('classes', 'scores', 'boxes', 'masks')}
            npu_prediction, ms = npu.predict(bgr);times.append(ms)
            g = surface(locate_parts(gpu_prediction, depth, scale, intrinsics, policy, matrix))
            n = surface(locate_parts(npu_prediction, depth, scale, intrinsics, policy, matrix))
            used, pairs = set(), []
            for o in g:
                best = min(((j, np.linalg.norm(np.subtract(o['base_xyz_m'], m['base_xyz_m']))) for j, m in enumerate(n)
                            if j not in used and m['part_class'] == o['part_class']), key=lambda x: x[1], default=None)
                if best and best[1] < .03:
                    used.add(best[0]);pairs.append((o, n[best[0]], best[1] * 1e3))
                    diffs[o['part_class']].append(best[1] * 1e3)
            rows.append(dict(frame=stem, gpu=len(g), npu=len(n), matched=len(pairs), npu_ms=ms,
                             grasp_allowed_agree=sum(p[0].get('robot_grasp_allowed') == p[1].get('robot_grasp_allowed') for p in pairs),
                             mm=[round(p[2], 2) for p in pairs], gpu_only=[o['part_class'] for o in g if not any(o is p[0] for p in pairs)],
                             npu_only=[m['part_class'] for j, m in enumerate(n) if j not in used]))
    finally:
        npu.close()
    allmm = [d for v in diffs.values() for d in v]
    q = lambda v, p: round(float(np.percentile(v, p)), 2) if v else None
    report = dict(status='NPU_DETECTOR_GRASP_TARGET_ACCEPTANCE', frames=len(rows), npu=npu.environment,
                  gpu_surface_points=sum(r['gpu'] for r in rows), npu_surface_points=sum(r['npu'] for r in rows), matched=sum(r['matched'] for r in rows),
                  grasp_allowed_agree=sum(r['grasp_allowed_agree'] for r in rows),
                  gpu_only=sum(len(r['gpu_only']) for r in rows), npu_only=sum(len(r['npu_only']) for r in rows),
                  base_xyz_difference_mm=dict(p50=q(allmm, 50), p95=q(allmm, 95), max=round(max(allmm), 2) if allmm else None),
                  per_class_mm={c: dict(n=len(v), p50=q(v, 50), p95=q(v, 95), max=round(max(v), 2) if v else None) for c, v in diffs.items()},
                  npu_predict_ms=dict(p50=q(times, 50), p95=q(times, 95)), rows=rows,
                  scope='reference = the production GPU prediction saved with each snapshot; same depth and locate_parts for both')
    (a.out / 'acceptance.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'rows'}, indent=2))


if __name__ == '__main__':
    main()
