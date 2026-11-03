"""Compute camera-frame XYZ from recorded screw pixels, depth and intrinsics.

Default is dry-run. --run saves reports/figures only. Uses RealSense SDK pure
deprojection without opening a camera. No robot-frame pose or motion is produced.
"""
import argparse
import csv
from datetime import datetime
import importlib.metadata
import json
import math
from pathlib import Path
import shutil

import cv2
import numpy as np
import pyrealsense2 as rs

from score_parts_predictions import digest, load_reviewed_inputs


def sdk_intrinsics(record, image_shape):
    if (record['height'], record['width']) != tuple(image_shape):
        raise ValueError('Recorded intrinsics do not match aligned image dimensions.')
    if not all(math.isfinite(record[key]) for key in ('fx', 'fy', 'cx', 'cy')) or record['fx'] <= 0 or record['fy'] <= 0:
        raise ValueError('Invalid focal length or principal point.')
    coefficients = record['coeffs']
    if len(coefficients) != 5 or not all(math.isfinite(value) for value in coefficients):
        raise ValueError('Expected five finite distortion coefficients.')
    name = record['distortion'].removeprefix('distortion.')
    # These models explicitly support deprojection in the SDK documentation.
    # Reject unverified models before calling native code, which may assert.
    if name not in ('none', 'inverse_brown_conrady'):
        raise ValueError(f'Deprojection model not verified by this entry: {name}')
    intr = rs.intrinsics()
    intr.width, intr.height = record['width'], record['height']
    intr.fx, intr.fy, intr.ppx, intr.ppy = record['fx'], record['fy'], record['cx'], record['cy']
    intr.model, intr.coeffs = getattr(rs.distortion, name), coefficients
    return intr


def validate_policy(policy):
    keys = ('min_depth_m', 'max_depth_m', 'min_retained_fraction', 'max_p90_minus_p10_m',
            'mad_multiplier', 'min_outlier_window_m')
    if not all(math.isfinite(policy[key]) for key in keys):
        raise ValueError('Depth policy contains non-finite values.')
    if not (0 < policy['min_depth_m'] < policy['max_depth_m'] and
            0 < policy['min_retained_fraction'] <= 1 and policy['min_valid_samples'] >= 1 and
            policy['max_p90_minus_p10_m'] > 0 and policy['mad_multiplier'] > 0 and policy['min_outlier_window_m'] > 0):
        raise ValueError('Invalid depth policy.')


def camera_point(depth, mask, center_uv, depth_scale, intrinsic_record, width_px, policy):
    validate_policy(policy)
    if depth.dtype != np.uint16 or depth.ndim != 2 or mask.shape != depth.shape:
        raise ValueError('Expected aligned uint16 depth and a same-size mask.')
    if not math.isfinite(depth_scale) or depth_scale <= 0:
        raise ValueError('Depth scale must be finite and positive.')
    center = np.asarray(center_uv, dtype=float)
    if center.shape != (2,) or not np.isfinite(center).all():
        raise ValueError('Expected a finite u/v pixel pair.')
    u, v = center
    if not (0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]):
        raise ValueError('Pixel lies outside the aligned frame.')
    if not mask[int(v), int(u)]:
        raise ValueError('Candidate pixel is outside the screw mask.')
    intr = sdk_intrinsics(intrinsic_record, depth.shape)
    radius = max(1, min(3, int(width_px * .25)))
    yy, xx = np.indices(depth.shape)
    core = cv2.erode((mask > 0).astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    patch = core & ((xx - u)**2 + (yy - v)**2 <= radius**2)
    raw = depth[patch]
    values = raw.astype(float) * depth_scale
    in_range = (raw > 0) & (values >= policy['min_depth_m']) & (values <= policy['max_depth_m'])
    values = values[in_range]
    median = float(np.median(values)) if len(values) else None
    mad = float(np.median(np.abs(values - median))) if len(values) else None
    window = max(policy['min_outlier_window_m'], policy['mad_multiplier'] * 1.4826 * mad) if len(values) else None
    retained = values[np.abs(values - median) <= window + 1e-12] if len(values) else values
    fraction = len(retained) / len(raw) if len(raw) else 0.0
    spread = float(np.percentile(retained, 90) - np.percentile(retained, 10)) if len(retained) else None
    result = {'status': 'INSUFFICIENT_LOCAL_DEPTH', 'center_uv_px': center.tolist(),
              'camera_xyz_m': None, 'base_xyz_m': None, 'robot_grasp_allowed': False,
              'metric_accuracy_verified': False, 'patch_radius_px': radius,
              'patch_pixels': int(len(raw)), 'nonzero_pixels': int(np.count_nonzero(raw)),
              'in_range_pixels': int(len(values)), 'retained_pixels': int(len(retained)),
              'retained_fraction': float(fraction), 'initial_z_median_m': median,
              'initial_z_mad_m': mad, 'outlier_window_m': window, 'retained_p90_minus_p10_m': spread}
    if len(retained) < policy['min_valid_samples'] or fraction < policy['min_retained_fraction']:
        return result
    if spread > policy['max_p90_minus_p10_m']:
        result['status'] = 'LOCAL_DEPTH_SPREAD_TOO_LARGE'
        return result
    z = float(np.median(retained))
    xyz = np.asarray(rs.rs2_deproject_pixel_to_point(intr, [float(u), float(v)], z), dtype=float)
    if xyz.shape != (3,) or not np.isfinite(xyz).all() or xyz[2] <= 0:
        raise ValueError('SDK returned an invalid camera point.')
    result.update(status='CAMERA_XYZ_COMPUTED_ABSOLUTE_ACCURACY_UNVERIFIED', camera_xyz_m=xyz.tolist(),
                  camera_xyz_mm=(xyz * 1000).tolist(), depth_used_m=z)
    return result


def compute(dataset, predictions, candidates_path, policy_path, output, run=False):
    dataset, predictions, output = dataset.resolve(), predictions.resolve(), output.resolve()
    if output.exists() or output.is_relative_to(dataset) or output.is_relative_to(predictions):
        raise ValueError('Use a new output directory outside source data and predictions.')
    loaded, _, prediction_report, source_hashes = load_reviewed_inputs(dataset, predictions)
    candidates = json.loads(candidates_path.read_text())
    policy = json.loads(policy_path.read_text())
    validate_policy(policy)
    if (candidates['checkpoint_sha256'] != prediction_report['checkpoint_sha256'] or
            candidates['source_manifest_sha256'] != digest(dataset / 'manifest.json') or
            digest(candidates_path.parent / 'generator_source.py') != candidates['generator_sha256']):
        raise ValueError('Pixel candidates do not match the frozen source/model.')
    by_scene = {row['scene_id']: row for row in candidates['images']}
    if len(by_scene) != len(candidates['images']) or set(by_scene) != {row[0]['scene_id'] for row in loaded}:
        raise ValueError('Pixel candidate scene IDs differ or repeat.')
    source_hashes.update({str(candidates_path): digest(candidates_path), str(policy_path): digest(policy_path)})
    records, figures = [], []
    for entry, image, _, masks, classes in loaded:
        metadata_path = Path(entry['source_npz']).parent / 'metadata.json'
        metadata = json.loads(metadata_path.read_text())
        matches = [row for row in metadata if row['index'] == entry['source_frame_index']]
        if len(matches) != 1 or matches[0].get('capture_mode') != 'MANUAL_PREVIEW_KEYPRESS':
            raise ValueError('Expected metadata from the known depth-aligned-to-color capture entry.')
        metadata = matches[0]
        if metadata['depth_scale_m'] != entry['depth_scale_m']:
            raise ValueError('Depth scale changed after export.')
        source_hashes[str(metadata_path)] = digest(metadata_path)
        source_hashes[entry['source_npz']] = digest(Path(entry['source_npz']))
        with np.load(entry['source_npz'], allow_pickle=False) as frame:
            depth = frame['depth_raw'].copy()
        results, drawn = [], image.copy()
        for number, candidate in enumerate(by_scene[entry['scene_id']]['candidates'], 1):
            index = candidate['prediction_index']
            if not 0 <= index < len(masks) or classes[index] != 0:
                raise ValueError('Candidate does not refer to a predicted screw.')
            point = camera_point(depth, masks[index], candidate['center_uv_px'], metadata['depth_scale_m'],
                                 metadata['intrinsics'], candidate['shaft_width_px_approx'], policy)
            point.update(candidate_id=f'S{number}', prediction_index=index)
            results.append(point)
            center = tuple(np.asarray(candidate['center_uv_px'], int))
            color = (0, 0, 255) if point['camera_xyz_m'] is not None else (0, 165, 255)
            cv2.circle(drawn, center, 4, color, -1, cv2.LINE_AA)
            cv2.putText(drawn, point['candidate_id'], (center[0] + 6, center[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1, cv2.LINE_AA)
        panel = np.full((image.shape[0], 330, 3), 245, np.uint8)
        cv2.putText(panel, 'CAMERA XYZ (mm)', (12, 26), cv2.FONT_HERSHEY_SIMPLEX, .6, (20, 20, 20), 1, cv2.LINE_AA)
        for number, point in enumerate(results):
            y = 58 + number * 103
            text = [f'{point["candidate_id"]}  u,v={point["center_uv_px"]}']
            if point['camera_xyz_m'] is not None:
                x, yy, z = point['camera_xyz_mm']
                text.extend((f'X {x:+.1f}  Y {yy:+.1f}  Z {z:.1f}', f'depth samples: {point["retained_pixels"]}/{point["patch_pixels"]}'))
            else:
                text.append('NO XYZ: depth gates rejected')
            for line, value in enumerate(text):
                cv2.putText(panel, value, (12, y + 23 * line), cv2.FONT_HERSHEY_SIMPLEX, .45, (20, 20, 20), 1, cv2.LINE_AA)
        if not results:
            cv2.putText(panel, 'No screw candidates', (12, 65), cv2.FONT_HERSHEY_SIMPLEX, .5, (20, 20, 20), 1, cv2.LINE_AA)
        picture = cv2.copyMakeBorder(np.hstack((drawn, panel)), 60, 0, 0, 0, cv2.BORDER_CONSTANT, value=(245, 245, 245))
        for line, text in enumerate((f'{entry["scene_id"]} | RECORDED camera frame only; base coordinates unavailable',
                                     'X: right   Y: down   Z: forward | Physical depth accuracy not validated')):
            cv2.putText(picture, text, (10, 23 + 24 * line), cv2.FONT_HERSHEY_SIMPLEX, .48, (20, 20, 20), 1, cv2.LINE_AA)
        figures.append((Path(entry['file']).name, picture))
        records.append({'scene_id': entry['scene_id'], 'source_npz': entry['source_npz'],
                        'metadata_sha256': digest(metadata_path), 'intrinsics': metadata['intrinsics'], 'points': results})
    computed = sum(point['camera_xyz_m'] is not None for row in records for point in row['points'])
    total = sum(len(row['points']) for row in records)
    print(json.dumps({'camera_points_computed': computed, 'rejected_points': total - computed, 'writes_enabled': run}, indent=2))
    if not run:
        print('DRY RUN: recorded data and SDK deprojection only; no outputs, camera or robot access.')
        return
    output.mkdir(parents=True, exist_ok=False)
    for name, figure in figures:
        if not cv2.imwrite(str(output / name), figure):
            raise OSError('Cannot write coordinate figure.')
    tiles = [figure for _, figure in figures]
    if len(tiles) % 2:
        tiles.append(np.full_like(tiles[0], 245))
    cv2.imwrite(str(output / 'contact_sheet.jpg'), np.vstack([np.hstack(tiles[i:i + 2]) for i in range(0, len(tiles), 2)]))
    with (output / 'camera_points_mm.csv').open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['scene_id', 'candidate_id', 'u_px', 'v_px', 'camera_x_mm', 'camera_y_mm', 'camera_z_mm', 'status'])
        for row in records:
            for point in row['points']:
                xyz = point.get('camera_xyz_mm', ['', '', ''])
                writer.writerow([row['scene_id'], point['candidate_id'], *point['center_uv_px'], *xyz, point['status']])
    if any(digest(Path(path)) != expected for path, expected in source_hashes.items()):
        raise ValueError('A source changed during computation; preserve output for review.')
    report = {'created_at': datetime.now().astimezone().isoformat(), 'scope': 'RECORDED_CAMERA_XYZ_NOT_ROBOT_CALIBRATION',
              'camera_points_computed': computed, 'rejected_points': total - computed,
              'camera_frame': 'Aligned depth-to-color optical frame at original capture: X right, Y down, Z forward; meters.',
              'method': 'SDK rs2_deproject_pixel_to_point at shaft-center pixel, using gated local median depth and recorded intrinsics/distortion.',
              'pyrealsense2_version': importlib.metadata.version('pyrealsense2'), 'depth_policy': policy,
              'generator_sha256': digest(Path(__file__)), 'checkpoint_sha256': prediction_report['checkpoint_sha256'],
              'source_sha256': source_hashes, 'source_files_unchanged': True, 'images': records,
              'camera_opened': False, 'robot_commands_sent': False, 'model_inference_or_training_performed': False,
              'base_transform_used': False, 'tcp_pose_computed': False, 'robot_grasp_allowed': False,
              'limitations': 'Input gates are provisional. No absolute metal depth accuracy, current live scene, robot coordinates, TCP, wall clearance or grasp success verified.',
              'sdk_reference': 'https://dev.realsenseai.com/docs/projection-in-realsense-sdk-2-0/'}
    (output / 'camera_points.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    shutil.copy2(Path(__file__), output / 'generator_source.py')
    print(f'Saved recorded camera coordinates: {output}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, default=Path('aipc_app/data/screw_nut_test_v1'))
    parser.add_argument('--predictions', type=Path, default=Path('aipc_app/reports/parts-fresh-test-20261017-153442/predictions'))
    parser.add_argument('--candidates', type=Path, default=Path('aipc_app/reports/parts-fresh-test-20261017-153442/grasp_pixel_preview_v1/grasp_preview.json'))
    parser.add_argument('--depth-policy', type=Path, default=Path('aipc_app/configs/parts_depth_draft_v1.json'))
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    compute(args.dataset, args.predictions, args.candidates, args.depth_policy, args.out, args.run)


if __name__ == '__main__':
    main()
