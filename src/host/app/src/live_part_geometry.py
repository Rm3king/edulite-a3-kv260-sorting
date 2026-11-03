"""Read-only RGB-D part surface estimates. No robot connection or commands."""
import json
from pathlib import Path
import sys
import cv2
import numpy as np
import pyrealsense2 as rs
from compute_camera_points import camera_point, sdk_intrinsics, validate_policy
from preview_part_grasps import shaft_candidate
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'integration'))
from workcell.instance_masks import refine


def load_transform(path):
    record = json.loads(Path(path).read_text())
    if record.get('schema') != 'aipc.camera_to_base.candidate.v1' or record.get('units') != 'm':
        raise ValueError('Expected camera-to-base candidate with metre units')
    if record.get('source_frame') != 'D435i color optical frame (x right, y down, z forward)':
        raise ValueError('Expected color optical source frame')
    matrix = np.asarray(record['T_base_camera'], float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all() or
        not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-10) or
        not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6) or
        not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-6)):
        raise ValueError('Invalid rigid camera-to-base transform')
    return record, matrix


def transform_point(matrix, point):
    p = np.asarray(point, float)
    if p.shape != (3,) or not np.isfinite(p).all():
        raise ValueError('Finite camera XYZ required')
    return (matrix[:3, :3] @ p + matrix[:3, 3]).tolist()


def nut_surface_point(depth, mask, scale, intrinsics, policy):
    """Estimate nut top-face centre from its annulus, never the hole depth.

    Assumes a roughly face-up, isolated nut. The returned point is an estimated
    visible surface location, not the nut's mass centre or a validated grasp.
    """
    validate_policy(policy)
    if depth.dtype != np.uint16 or mask.shape != depth.shape or not np.isfinite(scale) or scale <= 0:
        raise ValueError('Aligned uint16 depth and finite positive scale required')
    binary = (mask > 0).astype(np.uint8)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(binary)
    if n < 2:
        raise ValueError('Empty nut mask')
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    body = (labels == largest).astype(np.uint8)
    if body.sum() < 30 or body.sum() < .9 * binary.sum():
        raise ValueError('Small or fragmented nut mask')
    if body[0].any() or body[-1].any() or body[:, 0].any() or body[:, -1].any():
        raise ValueError('Nut touches image edge')
    contours, _ = cv2.findContours(body, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = max(contours, key=cv2.contourArea)
    filled = np.zeros_like(body); cv2.drawContours(filled, [contour], -1, 1, -1)
    m = cv2.moments(filled); center = np.array([m['m10']/m['m00'], m['m01']/m['m00']])
    _, sides, _ = cv2.minAreaRect(contour)
    if min(sides) < 8 or max(sides)/min(sides) > 1.8:
        raise ValueError('Nut face too small or too tilted')
    radius = min(sides)/2
    yy, xx = np.indices(depth.shape)
    radial = np.hypot(xx-center[0], yy-center[1])/radius
    ring = (cv2.erode(body, np.ones((3, 3), np.uint8)) > 0) & (radial >= .65) & (radial <= .9)
    raw = depth[ring]; values = raw.astype(float)*scale
    good = (raw > 0) & (values >= policy['min_depth_m']) & (values <= policy['max_depth_m'])
    z = values[good]
    result = dict(center_uv_px=center.tolist(), camera_xyz_m=None, status='NUT_RING_DEPTH_REJECTED',
                  method='outer-mask centroid and gated annular median surface depth',
                  ring_pixels=int(len(raw)), robot_grasp_allowed=False, metric_accuracy_verified=False)
    if not len(z): return result
    median = np.median(z); mad = np.median(abs(z-median))
    window = max(policy['min_outlier_window_m'], policy['mad_multiplier']*1.4826*mad)
    retained = z[abs(z-median) <= window+1e-12]
    fraction = len(retained)/len(raw) if len(raw) else 0
    spread = float(np.percentile(retained, 90)-np.percentile(retained, 10)) if len(retained) else None
    result.update(retained_pixels=len(retained), retained_fraction=fraction, spread_m=spread)
    if (len(retained) < policy['min_valid_samples'] or fraction < policy['min_retained_fraction'] or
        spread is None or spread > policy['max_p90_minus_p10_m']): return result
    intr = sdk_intrinsics(intrinsics, depth.shape); z = float(np.median(retained))
    xyz = rs.rs2_deproject_pixel_to_point(intr, center.tolist(), z)
    if not np.isfinite(xyz).all(): raise ValueError('Invalid deprojection')
    result.update(camera_xyz_m=xyz, depth_used_m=z, status='NUT_SURFACE_ESTIMATED')
    return result


def locate_parts(prediction, depth, scale, intrinsics, policy, matrix):
    rows = []
    for index, mask, refinement in refine(prediction):
        cid, confidence, box = prediction['classes'][index], prediction['scores'][index], prediction['boxes'][index]
        row = dict(detection_index=index, part_class={0:'screw', 1:'nut', 2:'screw_m6', 3:'nut_m6'}[int(cid)], confidence=float(confidence),
                   box_xyxy_px=box.tolist(), camera_xyz_m=None, base_xyz_m=None,
                   geometry_valid=False, robot_grasp_allowed=False, point_kind='estimated_visible_surface',
                   mask_refinement=refinement)
        try:
            if int(cid) in (0, 2):   # a screw, either size, is located along its shaft
                candidate = shaft_candidate(mask)
                point = camera_point(depth, mask, candidate['center_uv_px'], scale, intrinsics, candidate['shaft_width_px_approx'], policy)
                row['pixel_geometry'] = candidate
            else:
                point = nut_surface_point(depth, mask, scale, intrinsics, policy)
            row.update(camera_geometry=point, center_uv_px=point['center_uv_px'], geometry_status=point['status'])
            if point['camera_xyz_m'] is not None:
                row.update(camera_xyz_m=point['camera_xyz_m'], base_xyz_m=transform_point(matrix, point['camera_xyz_m']), geometry_valid=True)
        except ValueError as error:
            row['geometry_status'] = str(error)
        rows.append(row)
    return rows
