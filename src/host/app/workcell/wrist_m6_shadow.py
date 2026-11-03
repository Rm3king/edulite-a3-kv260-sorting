"""Host M6 screw shadow recovery; the shared board/nut locator stays byte-identical."""
import cv2
import numpy as np

from .wrist_m6_localization import (
    FINGER_BOXES, LIMITS, LOCAL_LEVEL, _geometry, head_direction,
    local_mask, locate_m6_near, tray_level,
)


def _bright_mask(gray):
    """Positive local contrast keeps reflective metal and excludes the cast shadow."""
    f = gray.astype(np.float32)
    mask = (cv2.GaussianBlur(f, (0, 0), 1.5) - tray_level(f) > LOCAL_LEVEL).astype(np.uint8) * 255
    for x0, y0, x1, y1 in FINGER_BOXES:
        mask[y0:y1, x0:x1] = 0
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))


def _bright_screw_body(bright_mask, component, original_length, original_axis):
    """Require one full-size body, >=70% length, aligned axis and a visibly wider head.

    A highlight alone cannot supply a head direction. All original size and head
    width requirements still apply, and ambiguous multiple bodies are refused.
    """
    lim = LIMITS['screw_m6']
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        cv2.bitwise_and(bright_mask, component.astype(np.uint8) * 255), connectivity=8)
    bodies = []
    for i in range(1, count):
        if stats[i, cv2.CC_STAT_AREA] < lim['min_area']:
            continue
        ys, xs = np.nonzero(labels == i)
        pts = np.column_stack([xs, ys])
        centre, axis, length, width, _ = _geometry(pts)
        if (not max(lim['min_length'], .7 * original_length) <= length <= lim['max_length']
                or width < lim['min_width'] or length / width < lim['min_aspect']
                or abs((axis - original_axis + 90.) % 180. - 90.) > 15.):
            continue
        bodies.append((pts, centre, axis))
    if len(bodies) != 1:
        return None
    pts, centre, axis = bodies[0]
    u = np.array([np.cos(np.radians(axis)), np.sin(np.radians(axis))])
    sign, _ = head_direction(pts, centre, u)
    return pts if sign is not None else None


def locate_m6_screw_view(gray, expected, radius, open_px):
    """Refine only complete, head-ambiguous candidates, then run the original locator.

    Retaining all other components lets the original neighbour/radius checks see
    the same clutter. The shared locator still supplies the grip point and all
    acceptance gates; no head hint or relaxed threshold is introduced.
    """
    mask = local_mask(gray, open_px=open_px)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    lim = LIMITS['screw_m6']
    bright = None
    recovered_boxes = []
    unclear = 0
    for i in range(1, count):
        x, y, w, h, area = stats[i]
        if area < lim['min_area']:
            continue
        dx, dy = max(x - expected[0], 0., expected[0] - (x + w)), max(y - expected[1], 0., expected[1] - (y + h))
        if np.hypot(dx, dy) > radius:
            continue
        ys, xs = np.nonzero(labels[y:y + h, x:x + w] == i)
        pts = np.column_stack([xs + x, ys + y])
        centre, axis, length, width, _ = _geometry(pts)
        if (not lim['min_length'] <= length <= lim['max_length']
                or width < lim['min_width'] or length / width < lim['min_aspect']):
            continue
        u = np.array([np.cos(np.radians(axis)), np.sin(np.radians(axis))])
        if head_direction(pts, centre, u)[0] is not None:
            continue
        unclear += 1
        if x == 0 or y == 0 or x + w == mask.shape[1] or y + h == mask.shape[0]:
            continue  # Never reconstruct a head from a clipped body.
        if bright is None:
            bright = _bright_mask(gray)
        body = _bright_screw_body(bright, labels == i, length, axis)
        if body is None:
            continue
        mask[labels == i] = 0
        mask[body[:, 1], body[:, 0]] = 255
        low, high = body.min(axis=0), body.max(axis=0)
        recovered_boxes.append([int(v) for v in (*low, *(high - low + 1))])
    try:
        observation = locate_m6_near(gray, expected, 'screw_m6', radius, open_px=0, _closed=mask)
    except ValueError as error:
        if unclear and str(error).startswith('No screw_m6'):
            raise ValueError('M6 screw head direction ambiguous (shadow or insufficient body evidence)') from error
        raise
    if observation['bbox'] in recovered_boxes:
        observation['shadow_removed'] = True
    return observation
