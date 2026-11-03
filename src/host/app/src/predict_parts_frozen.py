"""Predict saved test images with an already frozen screw/nut segmentation model.

Default is dry-run. --run performs ROCm inference only, never training, and saves
native-resolution masks separately from human annotations for later scoring.
"""
import argparse
from collections import Counter
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import time

import cv2
import numpy as np

from annotate_parts import CLASSES, COLORS


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')


def review_inputs(dataset, plan_path):
    manifest = json.loads((dataset / 'manifest.json').read_text())
    plan = json.loads(plan_path.read_text())
    checkpoint = Path(plan['checkpoint']).resolve()
    if digest(checkpoint) != plan['checkpoint_sha256']:
        raise ValueError('Frozen checkpoint hash differs; preserve it and review before testing.')
    if manifest['classes'] != {'0': 'screw', '1': 'nut'}:
        raise ValueError('Expected class mapping 0=screw, 1=nut.')
    scene_ids = [entry['scene_id'] for entry in manifest['images']]
    if len(scene_ids) != len(set(scene_ids)) or set(scene_ids) != set(plan['planned_scenes']):
        raise ValueError('Test scenes must match the frozen plan without duplicates.')
    for entry in manifest['images']:
        image_path = dataset / entry['file']
        image = cv2.imread(str(image_path))
        if image is None or image.shape[:2] != (entry['height'], entry['width']):
            raise ValueError(f'Missing or changed image dimensions: {image_path}')
        if digest(image_path) != entry['image_sha256'] or digest(Path(entry['source_npz'])) != entry['source_npz_sha256']:
            raise ValueError(f'Image/source RGB-D hash differs: {entry["scene_id"]}')
    if not 0 < plan['confidence_threshold'] < 1 or not 0 < plan['same_class_mask_iou_threshold'] <= 1:
        raise ValueError('Invalid frozen evaluation thresholds.')
    return manifest, plan, checkpoint


def predict_dataset(dataset, plan_path, output, run=False):
    dataset, output = dataset.resolve(), output.resolve()
    if output.exists() or output.is_relative_to(dataset):
        raise ValueError('Use a new output directory outside the source dataset.')
    manifest, plan, checkpoint = review_inputs(dataset, plan_path)
    print(json.dumps({'scenes': [e['scene_id'] for e in manifest['images']],
                      'checkpoint_sha256': plan['checkpoint_sha256'],
                      'confidence_threshold': plan['confidence_threshold']}, indent=2))
    if not run:
        print('DRY RUN: checked inputs; no inference, outputs, labels, training, camera or robot access.')
        return
    import torch
    if not torch.version.hip or not torch.cuda.is_available():
        raise RuntimeError('Use aipc.sh gpu with the isolated YOLO environment and a working ROCm GPU.')
    output.mkdir(parents=True, exist_ok=False)
    for name in ('inputs', 'images', 'masks'):
        (output / name).mkdir()
    write_json(output / 'source_manifest.json', manifest)
    write_json(output / 'frozen_plan.json', plan)
    shutil.copy2(Path(__file__), output / 'predictor_source.py')
    os.environ.update(YOLO_CONFIG_DIR=str(output / 'ultralytics_config'), YOLO_AUTOINSTALL='false', MPLBACKEND='Agg')
    from ultralytics import YOLO, __version__, settings
    settings.update({key: False for key in ('sync', 'hub', 'wandb', 'mlflow', 'clearml', 'comet', 'neptune', 'dvc', 'tensorboard') if key in settings})
    report = {'started_at': datetime.now().astimezone().isoformat(), 'status': 'RUNNING',
              'scope': 'FROZEN_MODEL_FRESH_IMAGE_PREDICTIONS_NOT_YET_SCORED',
              'dataset': str(dataset), 'checkpoint': str(checkpoint), 'checkpoint_sha256': digest(checkpoint),
              'plan_sha256': digest(plan_path), 'source_manifest_sha256': digest(dataset / 'manifest.json'),
              'predictor_sha256': digest(Path(__file__)), 'python': sys.executable,
              'torch': torch.__version__, 'hip': torch.version.hip, 'gpu': torch.cuda.get_device_name(0),
              'ultralytics': __version__, 'confidence_threshold': plan['confidence_threshold'],
              'nms_iou_threshold': 0.7, 'future_mask_match_iou_threshold': plan['same_class_mask_iou_threshold'],
              'imgsz': 640, 'native_resolution_masks': True, 'training_performed': False,
              'human_labels_written': False, 'camera_or_robot_access': False,
              'accuracy_metrics': 'NOT_COMPUTED_WITHOUT_REVIEWED_HUMAN_LABELS', 'images': []}
    started = time.monotonic()
    write_json(output / 'report.json', report)
    try:
        model = YOLO(str(checkpoint), task='segment')
        if model.names != CLASSES:
            raise ValueError('Frozen checkpoint classes differ from the dataset.')
        tiles = []
        for entry in manifest['images']:
            source = dataset / entry['file']
            snapshot = output / 'inputs' / source.name
            shutil.copy2(source, snapshot)
            if digest(snapshot) != entry['image_sha256']:
                raise ValueError(f'Image changed while copying: {source}')
            image = cv2.imread(str(snapshot))
            result = model.predict(str(snapshot), device=0, imgsz=640, conf=plan['confidence_threshold'],
                                   iou=0.7, retina_masks=True, verbose=False)[0]
            classes = result.boxes.cls.cpu().numpy().astype(np.int64)
            scores = result.boxes.conf.cpu().numpy()
            boxes = result.boxes.xyxy.cpu().numpy()
            masks = ((result.masks.data.cpu().numpy() > 0.5).astype(np.uint8) if result.masks is not None
                     else np.zeros((0, entry['height'], entry['width']), np.uint8))
            if len(masks) != len(classes) or masks.shape[1:] != image.shape[:2]:
                raise ValueError('Prediction masks do not match the original image dimensions.')
            if any(int(cid) not in CLASSES for cid in classes):
                raise ValueError('Unexpected prediction class.')
            archive = output / 'masks' / (source.stem + '.npz')
            np.savez_compressed(archive, masks=masks, class_ids=classes, confidences=scores, boxes_xyxy=boxes)
            counts = Counter(int(cid) for cid in classes)
            drawn = image.copy()
            for cid, score, box, mask in zip(classes, scores, boxes, masks):
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                color = COLORS[int(cid)]
                cv2.drawContours(drawn, contours, -1, color, 2)
                x, y = int(box[0]), max(15, int(box[1]) - 3)
                cv2.putText(drawn, f'{CLASSES[int(cid)]} {score:.2f}', (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.4, color, 1, cv2.LINE_AA)
            title = f'{entry["scene_id"]} | screw={counts[0]} nut={counts[1]} | MODEL conf>={plan["confidence_threshold"]}'
            tile = cv2.copyMakeBorder(drawn, 32, 0, 0, 0, cv2.BORDER_CONSTANT, value=(245, 245, 245))
            cv2.putText(tile, title, (8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (20, 20, 20), 1, cv2.LINE_AA)
            if not cv2.imwrite(str(output / 'images' / source.name), tile):
                raise OSError('Cannot write prediction preview.')
            tiles.append(tile)
            report['images'].append({'scene_id': entry['scene_id'], 'image_sha256': entry['image_sha256'],
                                     'mask_archive': str(archive.relative_to(output)), 'mask_archive_sha256': digest(archive),
                                     'counts': {name: counts[cid] for cid, name in CLASSES.items()},
                                     'objects': [{'class_id': int(cid), 'confidence': float(score), 'box_xyxy': box.tolist()}
                                                 for cid, score, box in zip(classes, scores, boxes)]})
        if len(tiles) % 2:
            tiles.append(np.full_like(tiles[0], 245))
        sheet = np.vstack([np.hstack(tiles[i:i + 2]) for i in range(0, len(tiles), 2)])
        if not cv2.imwrite(str(output / 'contact_sheet.jpg'), sheet):
            raise OSError('Cannot write prediction overview.')
        if digest(checkpoint) != plan['checkpoint_sha256']:
            raise ValueError('Checkpoint changed during inference.')
        for entry in manifest['images']:
            if digest(dataset / entry['file']) != entry['image_sha256']:
                raise ValueError('Source image changed during inference.')
        report['status'] = 'PREDICTED_AWAITING_INDEPENDENT_HUMAN_LABELS'
    except Exception as error:
        report.update(status='FAILED', error_type=type(error).__name__, error=str(error))
        raise
    finally:
        report['elapsed_s'] = time.monotonic() - started
        report['timing_scope'] = 'Whole inference script, not a controlled latency benchmark.'
        report['finished_at'] = datetime.now().astimezone().isoformat()
        write_json(output / 'report.json', report)
    print(json.dumps([{'scene_id': row['scene_id'], **row['counts']} for row in report['images']], indent=2))
    print('Predictions saved separately. Human labels remain untouched; no accuracy claim or robot control.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, default=Path('aipc_app/data/screw_nut_test_v1'))
    parser.add_argument('--plan', type=Path, default=Path('aipc_app/reports/preview-capture-20261017-152340/capture_plan.json'))
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    predict_dataset(args.dataset, args.plan, args.out, args.run)


if __name__ == '__main__':
    main()
