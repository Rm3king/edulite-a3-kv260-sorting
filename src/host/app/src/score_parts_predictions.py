"""Score frozen mask archives against independently saved screw/nut annotations.

Default is dry-run. --run saves a new report and comparison images. No model
loading, inference, training, camera, or robot access; source files stay intact.
"""
import argparse
from collections import Counter
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil

import cv2
import numpy as np

from annotate_parts import CLASSES, COLORS, image_and_labels


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def match_masks(objects, masks, classes, threshold):
    """Use the same descending-IoU class-aware greedy rule as the baseline."""
    shape = masks.shape[1:]
    truth_masks = []
    for obj in objects:
        mask = np.zeros(shape, np.uint8)
        cv2.fillPoly(mask, [np.asarray(obj['points'], np.int32)], 1)
        truth_masks.append(mask.astype(bool))
    candidates = []
    for pi, pred in enumerate(masks.astype(bool)):
        for gi, truth in enumerate(truth_masks):
            if classes[pi] != objects[gi]['class_id']:
                continue
            union = np.count_nonzero(pred | truth)
            iou = float(np.count_nonzero(pred & truth) / union) if union else 0.0
            if iou >= threshold:
                candidates.append((iou, pi, gi))
    used_pred, used_truth, matches = set(), set(), []
    for iou, pi, gi in sorted(candidates, reverse=True):
        if pi not in used_pred and gi not in used_truth:
            used_pred.add(pi)
            used_truth.add(gi)
            matches.append({'prediction_index': pi, 'annotation_index': gi,
                            'class_id': int(classes[pi]), 'mask_iou': iou})
    counts = {}
    for cid, name in CLASSES.items():
        tp = sum(classes[pi] == cid for pi in used_pred)
        counts[name] = {'tp': int(tp), 'fp': int(np.count_nonzero(classes == cid) - tp),
                        'fn': int(sum(obj['class_id'] == cid for obj in objects) - tp)}
    return {'counts': counts, 'matches': matches,
            'unmatched_prediction_indices': sorted(set(range(len(masks))) - used_pred),
            'unmatched_annotation_indices': sorted(set(range(len(objects))) - used_truth)}


def load_reviewed_inputs(dataset, predictions):
    manifest_path = dataset / 'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    report_path = predictions / 'report.json'
    prediction_report = json.loads(report_path.read_text())
    plan = json.loads((predictions / 'frozen_plan.json').read_text())
    if prediction_report['status'] != 'PREDICTED_AWAITING_INDEPENDENT_HUMAN_LABELS':
        raise ValueError('Expected an intact completed frozen prediction run.')
    if manifest['classes'] != {'0': 'screw', '1': 'nut'}:
        raise ValueError('Class mapping differs.')
    if digest(manifest_path) != prediction_report['source_manifest_sha256']:
        raise ValueError('Dataset manifest changed after prediction.')
    if manifest != json.loads((predictions / 'source_manifest.json').read_text()):
        raise ValueError('Dataset does not match the frozen source snapshot.')
    if digest(predictions / 'frozen_plan.json') != prediction_report['plan_sha256']:
        raise ValueError('Frozen plan hash differs.')
    if digest(predictions / 'predictor_source.py') != prediction_report['predictor_sha256']:
        raise ValueError('Frozen predictor source snapshot changed.')
    if (prediction_report['checkpoint_sha256'] != plan['checkpoint_sha256'] or
            digest(Path(prediction_report['checkpoint'])) != plan['checkpoint_sha256']):
        raise ValueError('Frozen model changed.')
    if (prediction_report['confidence_threshold'] != plan['confidence_threshold'] or
            prediction_report['future_mask_match_iou_threshold'] != plan['same_class_mask_iou_threshold']):
        raise ValueError('Evaluation thresholds changed.')
    entries = {entry['scene_id']: entry for entry in manifest['images']}
    rows = {row['scene_id']: row for row in prediction_report['images']}
    if (len(entries) != len(manifest['images']) or len(rows) != len(prediction_report['images']) or
            set(entries) != set(rows) or set(entries) != set(plan['planned_scenes'])):
        raise ValueError('Scenes differ or repeat across labels, predictions and plan.')
    loaded, source_hashes = [], {str(manifest_path): digest(manifest_path), str(report_path): digest(report_path)}
    for scene, entry in entries.items():
        image, objects, reviewed = image_and_labels(dataset, entry)
        if not reviewed:
            raise ValueError(f'Missing independent reviewed annotations: {scene}')
        stem = Path(entry['file']).stem
        state_path = dataset / 'annotations' / (stem + '.json')
        state = json.loads(state_path.read_text())
        if state['scene_id'] != scene or state['confirmed_empty'] != (not objects):
            raise ValueError(f'Annotation state inconsistent: {scene}')
        row = rows[scene]
        archive_path = predictions / row['mask_archive']
        if digest(archive_path) != row['mask_archive_sha256'] or row['image_sha256'] != entry['image_sha256']:
            raise ValueError(f'Prediction archive/image hash mismatch: {scene}')
        if digest(predictions / 'inputs' / Path(entry['file']).name) != entry['image_sha256']:
            raise ValueError(f'Prediction input snapshot changed: {scene}')
        if digest(Path(entry['source_npz'])) != entry['source_npz_sha256']:
            raise ValueError(f'Original RGB-D frame changed: {scene}')
        with np.load(archive_path, allow_pickle=False) as archive:
            masks = archive['masks'].copy()
            classes = archive['class_ids'].copy()
            confidences = archive['confidences'].copy()
        if (masks.shape != (len(classes), entry['height'], entry['width']) or
                confidences.shape != classes.shape or classes.ndim != 1 or
                not np.issubdtype(classes.dtype, np.integer) or
                not all(int(cid) in CLASSES for cid in classes) or not np.isin(masks, [0, 1]).all() or
                not np.isfinite(confidences).all() or
                not ((confidences >= plan['confidence_threshold']) & (confidences <= 1)).all()):
            raise ValueError(f'Invalid masks, classes or confidence values: {scene}')
        actual_counts = Counter(int(cid) for cid in classes)
        if row['counts'] != {name: actual_counts[cid] for cid, name in CLASSES.items()}:
            raise ValueError(f'Archive counts disagree with prediction report: {scene}')
        for path in (dataset / entry['file'], state_path, dataset / 'labels' / (stem + '.txt'), archive_path):
            source_hashes[str(path)] = digest(path)
        loaded.append((entry, image, objects, masks, classes))
    return loaded, plan, prediction_report, source_hashes


def score_dataset(dataset, predictions, output, run=False):
    dataset, predictions, output = dataset.resolve(), predictions.resolve(), output.resolve()
    if output.exists() or output.is_relative_to(dataset) or output.is_relative_to(predictions):
        raise ValueError('Use a new output directory outside the source dataset and predictions.')
    loaded, plan, prediction_report, source_hashes = load_reviewed_inputs(dataset, predictions)
    images, totals, all_ious = [], {name: Counter(tp=0, fp=0, fn=0) for name in CLASSES.values()}, []
    for entry, image, objects, masks, classes in loaded:
        result = match_masks(objects, masks, classes, plan['same_class_mask_iou_threshold'])
        result.update(scene_id=entry['scene_id'], human_objects=len(objects), predicted_objects=len(masks))
        images.append(result)
        for name, counts in result['counts'].items():
            totals[name].update(counts)
        all_ious.extend(match['mask_iou'] for match in result['matches'])
    for cid, name in CLASSES.items():
        counts = totals[name]
        counts['precision'] = counts['tp'] / (counts['tp'] + counts['fp']) if counts['tp'] + counts['fp'] else None
        counts['recall'] = counts['tp'] / (counts['tp'] + counts['fn']) if counts['tp'] + counts['fn'] else None
        ious = [m['mask_iou'] for image in images for m in image['matches'] if m['class_id'] == cid]
        counts['matched_mask_iou_mean'] = float(np.mean(ious)) if ious else None
    print(json.dumps({'counts': totals, 'images': len(images), 'writes_enabled': run}, indent=2))
    if not run:
        print('DRY RUN: computed scores from saved files; no outputs, inference, training or hardware actions.')
        return
    output.mkdir(parents=True, exist_ok=False)
    (output / 'comparisons').mkdir()
    (output / 'annotations_snapshot').mkdir()
    comparisons = []
    for (entry, image, objects, masks, classes), result in zip(loaded, images):
        truth, predicted = image.copy(), image.copy()
        for obj in objects:
            cv2.polylines(truth, [np.asarray(obj['points'], np.int32)], True, COLORS[obj['class_id']], 2)
        for mask, cid in zip(masks, classes):
            contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(predicted, contours, -1, COLORS[int(cid)], 2)
        pair = np.hstack((truth, predicted))
        pair = cv2.copyMakeBorder(pair, 36, 0, 0, 0, cv2.BORDER_CONSTANT, value=(245, 245, 245))
        tp, fp, fn = (sum(counts[key] for counts in result['counts'].values()) for key in ('tp', 'fp', 'fn'))
        title = f'{entry["scene_id"]} | LEFT: human | RIGHT: frozen model | TP={tp} FP={fp} FN={fn}'
        cv2.putText(pair, title, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, .52, (20, 20, 20), 1, cv2.LINE_AA)
        name = Path(entry['file']).name
        if not cv2.imwrite(str(output / 'comparisons' / name), pair):
            raise OSError('Cannot write comparison.')
        comparisons.append(pair)
        stem = Path(name).stem
        for folder, suffix in (('annotations', '.json'), ('labels', '.txt')):
            shutil.copy2(dataset / folder / (stem + suffix), output / 'annotations_snapshot' / (stem + suffix))
    cv2.imwrite(str(output / 'comparison_sheet.jpg'), np.vstack(comparisons))
    if any(digest(Path(path)) != expected for path, expected in source_hashes.items()):
        raise ValueError('A source label or prediction changed during scoring; preserve outputs and review.')
    report = {'scored_at': datetime.now().astimezone().isoformat(),
              'status': 'SCORED_FIXED_THRESHOLD_FRESH_IMAGES',
              'scope': 'FIVE_FRESH_ARRANGEMENTS_NOT_FINAL_GENERALIZATION_BENCHMARK',
              'dataset': str(dataset), 'predictions': str(predictions),
              'checkpoint_sha256': prediction_report['checkpoint_sha256'],
              'confidence_threshold': plan['confidence_threshold'],
              'same_class_mask_iou_threshold': plan['same_class_mask_iou_threshold'],
              'matching': 'Class-aware greedy by descending native-resolution raster mask IoU; not COCO mAP.',
              'counts': totals, 'matched_mask_iou_mean': float(np.mean(all_ious)) if all_ious else None,
              'matched_mask_iou_min': min(all_ious) if all_ious else None,
              'confirmed_empty_images': sum(not row[2] for row in loaded), 'images': images,
              'source_sha256': source_hashes, 'source_files_unchanged': True,
              'scorer_sha256': digest(Path(__file__)), 'inference_or_training_performed': False,
              'camera_or_robot_access': False,
              'limitations': 'Five images, manual coarse outlines, one background. No millimeter accuracy, grasp success or robot calibration measured.'}
    (output / 'evaluation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    shutil.copy2(Path(__file__), output / 'scorer_source.py')
    print(f'Scores and comparisons saved: {output}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, default=Path('aipc_app/data/screw_nut_test_v1'))
    parser.add_argument('--predictions', type=Path, default=Path('aipc_app/reports/parts-fresh-test-20261017-153442/predictions'))
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    score_dataset(args.dataset, args.predictions, args.out, args.run)


if __name__ == '__main__':
    main()
