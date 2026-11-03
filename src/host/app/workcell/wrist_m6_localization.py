"""Wrist-camera localisation of the M6 screws / nuts (zinc parts), host side with OpenCV.

On the wrist camera the tray is a smooth mid-grey surface with a strong brightness gradient and the zinc parts are only slightly
brighter, with sharp specular edges: a global threshold (the dark-body method of the M12 screw, or Otsu on the bright side)
merges the lit background. The parts are therefore segmented by LOCAL contrast: |Gaussian(1.5) - Gaussian(18)| above a level
(the wide blur via tray_level),
fingertip/gripper regions blanked, 9x9 close, connected components. Observation keys follow locate_screw_near.
Size limits are provisional (first real M6 wrist frames, 2026-10-28: M6 T-nut about 80x80 px at the hover height) and are set
from the saved `raw_frames` of every run.
"""
import cv2
import numpy as np

FINGER_BOXES = ((0, 0, 200, 100), (335, 0, 560, 100))
LOCAL_LEVEL = 14.          # grey levels of local deviation; tray texture stays below 10 (99th percentile of a part-free tray: to be confirmed)
LIMITS = dict(
    screw_m6=dict(min_length=70., max_length=320., min_width=8., min_aspect=1.7, min_area=700.),
    nut_m6=dict(min_area=1200., max_area=9000., max_aspect=2.2, min_fill=.35),   # 6 real frames: fill .51-.63, one with a merged reflection .42
)
NEIGHBOUR_AMBIGUOUS_PX = 25.


SCREW_OPEN_PX = 19   # lamp glints on the tray (dots of 12-17 px) appear in some frames and bridge to a screw; its mask is 25+ px wide


def tray_level(f, sigma=18., pad=80):
    """The smooth tray brightness: Gaussian(sigma) computed at half resolution on a reflect-padded frame.

    A sigma-18 blur is a low-pass far below the half-resolution Nyquist limit; this form takes 2 ms instead of 89 ms on the
    KV260 A53 (the board loop runs this file unchanged). Against the full-resolution blur on 693 saved M6 cases (both kinds,
    three seeds; 2026-10-30): identical accept/reject and error text in 693/693, centres within 0.28 px (p99 0.15 px)."""
    p = cv2.copyMakeBorder(f, pad, pad, pad, pad, cv2.BORDER_REFLECT_101)
    h, w = p.shape
    small = cv2.GaussianBlur(cv2.resize(p, (w // 2, h // 2), interpolation=cv2.INTER_AREA), (0, 0), sigma / 2)
    return cv2.resize(small, (w, h), interpolation=cv2.INTER_LINEAR)[pad:h - pad, pad:w - pad]


def local_mask(gray, exclude=FINGER_BOXES, level=LOCAL_LEVEL, open_px=0, closed=None):
    """Closed 0/255 mask of pixels that differ from the smooth tray by more than `level`; fingertips excluded.

    `closed`: this function's own result for the same frame with open_px=0 (the two blurs are most of the cost)."""
    if closed is not None:
        return cv2.morphologyEx(closed, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_px, open_px))) if open_px else closed
    f = gray.astype(np.float32)
    deviation = np.abs(cv2.GaussianBlur(f, (0, 0), 1.5) - tray_level(f))
    mask = (deviation > level).astype(np.uint8) * 255
    for x0, y0, x1, y1 in exclude:
        mask[y0:y1, x0:x1] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    if open_px:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_px, open_px)))
    return mask


def _geometry(mask_pixels_xy):
    pts = mask_pixels_xy.astype(float)
    mean = pts.mean(0)
    _, vectors = np.linalg.eigh(np.cov((pts - mean).T))
    u = vectors[:, 1]
    if u[0] < 0:
        u = -u
    normal = np.array([-u[1], u[0]])
    along, across = (pts - mean) @ u, (pts - mean) @ normal
    centre = mean + u * (along.min() + along.max()) / 2
    return centre, float(np.degrees(np.arctan2(u[1], u[0])) % 180.), float(along.max() - along.min()), float(across.max() - across.min()), mean


HEAD_GRIP_PX = 27.          # grip this far (about 5.4 mm) behind the head end, measured from the head end itself: a glint at the thread end must not move it
HEAD_WIDTH_RATIO = 1.15     # the cap head must be at least this much wider than the other end, or the end is called unclear


def head_direction(pts, centre, u):
    """(+1 if the wider (head) end lies at +u, -1 at -u, and its along-axis coordinate from `centre`); sign None when the two end quarters are too alike."""
    normal = np.array([-u[1], u[0]])
    along, across = (pts - centre) @ u, (pts - centre) @ normal
    lo, hi = along.min(), along.max()
    quarter = (hi - lo) / 4.
    widths = []
    for sel in (along >= hi - quarter, along <= lo + quarter):
        widths.append(float(across[sel].max() - across[sel].min()) if sel.sum() > 3 else 0.)
    plus, minus = widths
    if max(plus, minus) < HEAD_WIDTH_RATIO * max(min(plus, minus), 1.):
        return None, None
    # the 3rd/97th percentile, not the extreme pixel: a glint touching the head end adds a few pixels there
    return (1 if plus > minus else -1), float(np.percentile(along, 97) if plus > minus else np.percentile(along, 3))


def locate_m6_near(gray, expected, kind, radius=220., limits=None, exclude=FINGER_BOXES, level=LOCAL_LEVEL, open_px=None, _closed=None,
                   head_hint=None):
    """Dict(center_px, axis_deg, cut, length_px, width_px, bbox, score, gap, distance, head_dir) of the `kind` nearest `expected`.

    `head_hint` (screw only): image direction toward the head found in earlier frames of the same screw. Used only when the two
    ends look alike (the shadow of a near-vertical screw widens both ends: 2026-10-30 run11, 52 px wide, every frame after the
    arm moved was rejected). Without it (the AI PC servo) the result is unchanged."""
    if kind not in LIMITS:
        raise ValueError('kind must be screw_m6 or nut_m6')
    lim = dict(LIMITS[kind], **(limits or {}))
    expected = np.asarray(expected, float)
    if open_px is None and kind == 'screw_m6':
        # the opening removes glints but cuts the thin neck of a small (distant) screw: then look again without it
        closed = local_mask(gray, exclude, level, 0)          # shared by both attempts
        try:
            return locate_m6_near(gray, expected, kind, radius, limits, exclude, level, SCREW_OPEN_PX, closed, head_hint)
        except ValueError:
            return locate_m6_near(gray, expected, kind, radius, limits, exclude, level, 0, closed, head_hint)
    mask = local_mask(gray, exclude, level, open_px or 0, _closed)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    choices = []
    for i in range(1, count):
        x, y, w, h, area = stats[i]
        if area < lim['min_area'] or area > lim.get('max_area', 1e9):
            continue
        dx, dy = max(x - expected[0], 0., expected[0] - (x + w)), max(y - expected[1], 0., expected[1] - (y + h))
        if np.hypot(dx, dy) > radius:
            continue
        ys, xs = np.nonzero(labels[y:y + h, x:x + w] == i)
        centre, axis, length, width, mean = _geometry(np.stack([xs + x, ys + y], 1))
        fill = float(area) / max(length * width, 1.)
        if kind == 'screw_m6':
            if not lim['min_length'] <= length <= lim['max_length'] or width < lim['min_width'] or length / width < lim['min_aspect']:
                continue
        else:
            if length / max(width, 1.) > lim['max_aspect'] or fill < lim['min_fill']:
                continue
            centre = mean                          # a nut is gripped across its centre
        if kind == 'screw_m6':
            u = np.array([np.cos(np.radians(axis)), np.sin(np.radians(axis))])
            pts = np.stack([xs + x, ys + y], 1).astype(float)
            sign, head_end = head_direction(pts, centre, u)
            hinted = False
            if sign is None and head_hint is not None and abs(float(u @ np.asarray(head_hint, float))) > .9:
                sign = 1 if u @ np.asarray(head_hint, float) > 0 else -1
                along = (pts - centre) @ u
                head_end, hinted = float(np.percentile(along, 97) if sign > 0 else np.percentile(along, 3)), True
            if sign is None:
                continue
            centre = centre + u * (head_end - sign * HEAD_GRIP_PX)   # grip just below the head, not on the thread
        contour = cv2.findContours((labels[y:y + h, x:x + w] == i).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0] + [x, y]
        gap = max(0., -cv2.pointPolygonTest(contour, (float(expected[0]), float(expected[1])), True))
        if gap > radius:
            continue
        cut = x == 0 or y == 0 or x + w == mask.shape[1] or y + h == mask.shape[0]
        choices.append(dict(center_px=[float(centre[0]), float(centre[1])], axis_deg=axis, cut=bool(cut), length_px=length, width_px=width,
                            bbox=[int(x), int(y), int(w), int(h)], score=fill, gap=float(gap),
                            distance=float(np.hypot(centre[0] - expected[0], centre[1] - expected[1])), threshold=float(level),
                            part_class=kind, provisional=True,
                            **(dict(head_dir=[float(u[0] * sign), float(u[1] * sign)], head_from_hint=hinted) if kind == 'screw_m6' else {})))
    if not choices:
        raise ValueError(f'No {kind} near the expected position')
    choices.sort(key=lambda c: (c['gap'], c['distance']))
    if len(choices) > 1 and choices[1]['gap'] < choices[0]['gap'] + NEIGHBOUR_AMBIGUOUS_PX:
        raise ValueError(f'More than one {kind} near the expected position')
    return choices[0]
