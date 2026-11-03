"""Train the 4-class screw/nut segmentation model from data/parts_inhouse_v2 (reviewed labels only), split by capture session.

    python src/train_parts_v2.py --run NAME [--imgsz 1280 --epochs 80]

Validation sessions are whole sessions, never frames of a session that is also in training.
"""
import argparse
import json
import shutil
from pathlib import Path

APP = Path(__file__).resolve().parents[1]
DATA = APP / 'data' / 'parts_inhouse_v2'
VAL_SESSIONS = {'m20261027_214112', 's20261025_153017', 's20261026_210704'}
NAMES = ['screw', 'nut', 'screw_m6', 'nut_m6']


def build(out):
    shutil.rmtree(out, ignore_errors=True)
    counts = {'train': 0, 'val': 0}
    for label in sorted((DATA / 'labels_yolo').glob('*.txt')):
        split = 'val' if label.stem.rsplit('_', 1)[0] in VAL_SESSIONS else 'train'
        for kind, source in (('images', DATA / 'images' / (label.stem + '.png')), ('labels', label)):
            target = out / kind / split / source.name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.symlink_to(source)
        counts[split] += 1
    (out / 'data.yaml').write_text(f"path: {out}\ntrain: images/train\nval: images/val\nnames: {dict(enumerate(NAMES))}\n")
    return counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True)
    parser.add_argument('--imgsz', type=int, default=1280)
    parser.add_argument('--epochs', type=int, default=80)
    parser.add_argument('--batch', type=int, default=8)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--resume', action='store_true', help='continue the run from its last.pt (same dataset split)')
    args = parser.parse_args()
    out = APP / 'runs' / 'parts_v2' / args.run
    counts = build(out / 'dataset') if not args.resume else dict(resumed=True)
    print('split', counts, flush=True)
    from ultralytics import YOLO
    if args.resume:
        YOLO(str(out / 'train' / 'weights' / 'last.pt')).train(resume=True, workers=args.workers, batch=args.batch, device=0)
    else:
      YOLO(str(APP / 'models' / 'yolo11n-seg.pt')).train(data=str(out / 'dataset' / 'data.yaml'), imgsz=args.imgsz, epochs=args.epochs, batch=args.batch, device=0, workers=4,
                project=str(out), name='train', exist_ok=True, patience=25, seed=0, close_mosaic=10, degrees=15, flipud=.5, fliplr=.5,
                hsv_v=.4, hsv_s=.5, plots=True)
    metrics = YOLO(str(out / 'train' / 'weights' / 'best.pt')).val(data=str(out / 'dataset' / 'data.yaml'), imgsz=args.imgsz, device=0, split='val')
    (out / 'val_metrics.json').write_text(json.dumps(dict(split=counts, imgsz=args.imgsz, box_map50_95=metrics.box.map, box_map50=metrics.box.map50,
        mask_map50_95=metrics.seg.map, mask_map50=metrics.seg.map50, per_class_mask_map50_95=dict(zip(NAMES, [float(x) for x in metrics.seg.maps]))), indent=2))


if __name__ == '__main__':
    main()
