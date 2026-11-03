"""Fit a diagnostic support plane from a reviewed empty RGB-D scene. Offline only."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from live_part_geometry import load_transform
from workplane_geometry import pixel_ray, robust_plane


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--dataset', type=Path, required=True)
    ap.add_argument('--config', type=Path, required=True)
    ap.add_argument('--calibration', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--run', action='store_true')
    a = ap.parse_args()
    config = json.loads(a.config.read_text()); scene = config['empty_scene']
    meta_path = a.dataset/'raw'/f'{scene}_capture.json'; meta = json.loads(meta_path.read_text())
    annotation = json.loads((a.dataset/'annotations'/f'{scene}.json').read_text())
    if not (meta.get('user_confirmed_empty_and_hand_removed') and annotation.get('reviewed')
            and annotation.get('confirmed_empty') and not annotation['objects']):
        raise ValueError('Reviewed empty support scene required')
    image_path = a.dataset/'images'/f'{scene}.png'; depth_path = a.dataset/'raw'/f'{scene}_depth.npy'
    for path in (image_path, depth_path):
        if hashlib.sha256(path.read_bytes()).hexdigest() != meta['file_sha256'][str(path.relative_to(a.dataset))]:
            raise ValueError('Source frame changed')
    image = cv2.imread(str(image_path)); depth = np.load(depth_path, allow_pickle=False)
    intr = meta['intrinsics']; scale = meta['depth_scale_m']
    if image is None or depth.dtype != np.uint16 or image.shape[:2] != depth.shape:
        raise ValueError('Color-aligned uint16 depth required')
    calibration, matrix = load_transform(a.calibration)
    if calibration['calibration_id'] != meta['calibration_id']:
        raise ValueError('Different calibration record')
    roi = np.asarray(config['fit_roi_uv'], np.int32)
    from annotate_parts import validate_polygon
    validate_polygon(roi.tolist(), intr['width'], intr['height'])
    mask = np.zeros(depth.shape, np.uint8); cv2.fillPoly(mask, [roi], 1)
    valid = (mask > 0) & (depth > 0) & (depth*scale > .15) & (depth*scale < 2)
    if valid.sum()/mask.sum() < .8:
        raise ValueError('Too little valid empty-tray depth')
    yy, xx = np.indices(depth.shape)
    sampled = valid & (xx % 6 == 0) & (yy % 6 == 0)
    v, u = np.nonzero(sampled)
    points = np.array([pixel_ray([int(x),int(y)], intr)*float(depth[y,x])*scale for x,y in zip(u,v)])
    # Spatial tiles, not random neighboring pixel split. Still one sensor frame,
    # so held-out residuals are internal consistency, not independent truth.
    held = ((u//80 + v//80) % 2 == 0)
    plane = robust_plane(points[~held], threshold_m=config['ransac_threshold_m'])
    n = np.asarray(plane['normal_camera']); d = plane['offset_m']
    residual = points[held] @ n + d
    plane['heldout_abs_residual_p95_m'] = float(np.percentile(abs(residual),95))
    plane['heldout_rmse_m'] = float(np.sqrt(np.mean(residual**2)))
    plane['normal_base'] = (matrix[:3,:3] @ n).tolist()
    plane['offset_base_m'] = float(d - np.asarray(plane['normal_base']) @ matrix[:3,3])
    accepted = (plane['inlier_fraction'] >= .9 and plane['heldout_abs_residual_p95_m'] <= config['max_heldout_p95_m'])
    report = dict(schema='aipc.workplane.candidate.v1', created_at=datetime.now().astimezone().isoformat(),
                  status='CANDIDATE_INTERNAL_FIT_PASS' if accepted else 'REJECTED_INTERNAL_FIT',
                  plane=plane, config=config, intrinsics=intr, depth_scale_m=scale,
                  camera_serial=meta['camera_serial'], calibration_id=calibration['calibration_id'],
                  source_scene=scene, valid_depth_fraction=float(valid.sum()/mask.sum()),
                  fit_samples=int((~held).sum()), heldout_samples=int(held.sum()),
                  robot_grasp_allowed=False, metric_accuracy_verified=False,
                  heights_measured=False, camera_opened=False, robot_commands_sent=0,
                  note='Manual interior support fit region only, not reachable ROI, tray wall clearance or motion approval.',
                  source_sha256={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in (image_path,depth_path,meta_path,a.config,a.calibration,Path(__file__),Path(__file__).with_name('workplane_geometry.py'))})
    print(json.dumps(dict(status=report['status'], plane=plane),indent=2))
    if a.run:
        a.out.mkdir(parents=True,exist_ok=False)
        (a.out/'workplane.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        cv2.polylines(image,[roi],True,(0,220,255),3)
        cv2.putText(image,'SUPPORT FIT REGION ONLY - NOT ROBOT REACHABLE ROI',(40,50),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,0,255),2)
        cv2.imwrite(str(a.out/'support_fit_region.png'),image)


if __name__ == '__main__': main()
