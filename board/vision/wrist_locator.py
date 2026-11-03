"""Screw observation from the PL mask, on the board: the part of workcell.wrist_screw_localization.locate_screw_near after the
mask (components with holes filled, pixel principal axis, extent, choice of the nearest body), without OpenCV.

Same parameters, result keys and refusals as the host function, so the host servo logic can take these observations unchanged.
Differences from OpenCV, by construction small: the contour area is Pick's estimate from pixel and boundary counts, and the gap
from `expected` is measured to the nearest boundary pixel centre instead of the contour polygon.
"""
import ctypes
from pathlib import Path

import numpy as np

import vision_pre as vp

W, H = vp.W, vp.H
NEIGHBOUR_AMBIGUOUS_PX = 25.
NEIGHBOUR_CLEARANCE_PX = 50.
MAX_LABELS = 4096


class Blobs:
    def __init__(self):
        self.lib = vp._library('libvision_blobs.so', ['vision_blobs.c'])
        self.lib.vision_blobs.restype = ctypes.c_int
        self.labels = np.zeros(W * H, np.int32)
        self.stats = np.zeros((MAX_LABELS, 10), np.int64)
        self.work = np.zeros(W * H * 9, np.uint8)

    def __call__(self, packed):
        packed = np.ascontiguousarray(packed, np.uint8)
        n = self.lib.vision_blobs(packed.ctypes.data_as(ctypes.c_void_p), self.labels.ctypes.data_as(ctypes.c_void_p), MAX_LABELS,
                                  self.stats.ctypes.data_as(ctypes.c_void_p), self.work.ctypes.data_as(ctypes.c_void_p))
        if n < 0:
            raise ValueError('Too many dark bodies in the wrist image')
        return self.labels.reshape(H, W), self.stats[:n]


_blobs = None


def locate_screw_in_mask(packed, expected, radius=220., min_length=90., max_length=700., min_aspect=2.0, threshold=None):
    """Dict(center_px, axis_deg, cut, length_px, width_px, bbox, score, gap, distance) of the elongated body nearest `expected`."""
    global _blobs
    if _blobs is None:
        _blobs = Blobs()
    labels, stats = _blobs(packed)
    expected = np.asarray(expected, float)
    ex, ey = int(round(expected[0])), int(round(expected[1]))
    choices = []
    for i, (pixels, boundary, sx, sy, x0, x1, y0, y1, border, _) in enumerate(stats):
        area = pixels - boundary / 2. - 1.          # Pick: polygon through the boundary pixel centres
        if area < 400:
            continue
        # cheap reject before touching pixels: the box diagonal bounds the length, the box distance bounds the gap
        if np.hypot(x1 - x0 + 1, y1 - y0 + 1) < min_length:
            continue
        dx = max(x0 - expected[0], 0., expected[0] - x1)
        dy = max(y0 - expected[1], 0., expected[1] - y1)
        if np.hypot(dx, dy) > radius + 1:
            continue
        sub = labels[y0:y1 + 1, x0:x1 + 1] == i + 1
        ys, xs = np.nonzero(sub)
        pts = np.stack([xs + x0, ys + y0], 1).astype(float)
        mean = pts.mean(0)
        _, vectors = np.linalg.eigh(np.cov((pts - mean).T))
        u = vectors[:, 1]
        if u[0] < 0:
            u = -u
        normal = np.array([-u[1], u[0]])
        along, across = (pts - mean) @ u, (pts - mean) @ normal
        length, width = float(along.max() - along.min()), float(across.max() - across.min())
        if not min_length <= length <= max_length or width < 6 or length / width < min_aspect:
            continue
        if 0 <= ex < W and 0 <= ey < H and labels[ey, ex] == i + 1:
            gap = 0.
        else:
            edge = sub & ~(np.pad(sub, 1)[2:, 1:-1] & np.pad(sub, 1)[:-2, 1:-1] & np.pad(sub, 1)[1:-1, 2:] & np.pad(sub, 1)[1:-1, :-2])
            by, bx = np.nonzero(edge)
            gap = float(np.min(np.hypot(bx + x0 - expected[0], by + y0 - expected[1])))
        if gap > radius:
            continue
        center = mean + u * (along.min() + along.max()) / 2
        choices.append(dict(center_px=[float(center[0]), float(center[1])], axis_deg=float(np.degrees(np.arctan2(u[1], u[0])) % 180.),
                            cut=bool(border), length_px=length, width_px=width,
                            bbox=[int(x0), int(y0), int(x1 - x0 + 1), int(y1 - y0 + 1)], score=float(area / (length * width)),
                            gap=gap, distance=float(np.hypot(center[0] - expected[0], center[1] - expected[1]))))
    if not choices:
        raise ValueError('No complete elongated dark body near the expected screw position')
    choices.sort(key=lambda c: (c['gap'], c['distance']))
    if len(choices) > 1:
        if choices[1]['gap'] < choices[0]['gap'] + NEIGHBOUR_AMBIGUOUS_PX:
            raise ValueError('More than one elongated dark body near the expected screw position')
        if choices[1]['gap'] < NEIGHBOUR_CLEARANCE_PX:
            raise ValueError('Neighbouring screw too close to the open jaws')
    best = choices[0]
    if threshold is not None:
        best['threshold'] = float(threshold)
    return best


def locate_screw_near(engine, gray, expected, radius=220., **kw):
    """Board version of the host function: PL (or ARM fallback) mask, then the observation above."""
    r = engine.run(gray, vp.Params(roi=vp.roi_around(expected, radius)))
    return locate_screw_in_mask(r['mask'], expected, radius, threshold=r['level'], **kw)


# M6 parts are bright zinc on the blue tray: the same kernel with the bright side and the Otsu level not capped. The size and
# shape limits below are PROVISIONAL (scaled from the M12 screw at the same hover height: M12x60 ~280 px long, so ~4.7 px/mm);
# they must be set from real wrist frames of M6 parts before any grasp uses them.
M6_LIMITS = dict(
    screw_m6=dict(min_length=60., max_length=260., min_width=8., min_aspect=2.5, min_area=250),
    nut_m6=dict(min_area=500, max_area=4000, max_aspect=1.6, min_solidity=.8),
)


def locate_m6_in_mask(packed, expected, kind, radius=220., limits=None, threshold=None):
    """Nearest M6 screw (elongated) or M6 nut (compact, near-hexagonal blob) in a bright-side mask. PROVISIONAL limits."""
    global _blobs
    if _blobs is None:
        _blobs = Blobs()
    lim = dict(M6_LIMITS[kind], **(limits or {}))
    labels, stats = _blobs(packed)
    expected = np.asarray(expected, float)
    choices = []
    for i, (pixels, boundary, sx, sy, x0, x1, y0, y1, border, _) in enumerate(stats):
        area = pixels - boundary / 2. - 1.
        if area < lim['min_area'] or ('max_area' in lim and area > lim['max_area']):
            continue
        dx = max(x0 - expected[0], 0., expected[0] - x1)
        dy = max(y0 - expected[1], 0., expected[1] - y1)
        if np.hypot(dx, dy) > radius:
            continue
        sub = labels[y0:y1 + 1, x0:x1 + 1] == i + 1
        ys, xs = np.nonzero(sub)
        pts = np.stack([xs + x0, ys + y0], 1).astype(float)
        mean = pts.mean(0)
        _, vectors = np.linalg.eigh(np.cov((pts - mean).T))
        u = vectors[:, 1]
        if u[0] < 0:
            u = -u
        normal = np.array([-u[1], u[0]])
        along, across = (pts - mean) @ u, (pts - mean) @ normal
        length, width = float(along.max() - along.min()), float(across.max() - across.min())
        if kind == 'screw_m6':
            if not lim['min_length'] <= length <= lim['max_length'] or width < lim['min_width'] or length / max(width, 1.) < lim['min_aspect']:
                continue
            center = mean + u * (along.min() + along.max()) / 2
        else:
            if length / max(width, 1.) > lim['max_aspect'] or area / max(length * width, 1.) < lim['min_solidity'] * .75:
                continue
            center = mean                       # a nut is gripped across its centre
        dist = float(np.hypot(center[0] - expected[0], center[1] - expected[1]))
        choices.append(dict(part_class=kind, center_px=[float(center[0]), float(center[1])],
                            axis_deg=float(np.degrees(np.arctan2(u[1], u[0])) % 180.), cut=bool(border), length_px=length,
                            width_px=width, area_px=float(area), bbox=[int(x0), int(y0), int(x1 - x0 + 1), int(y1 - y0 + 1)],
                            distance=dist, provisional=True))
    if not choices:
        raise ValueError(f'No {kind} near the expected position')
    choices.sort(key=lambda c: c['distance'])
    if len(choices) > 1 and choices[1]['distance'] < choices[0]['distance'] + NEIGHBOUR_AMBIGUOUS_PX:
        raise ValueError(f'More than one {kind} near the expected position')
    best = choices[0]
    if threshold is not None:
        best['threshold'] = float(threshold)
    return best


def locate_m6_near(engine, gray, expected, kind, radius=220., limits=None):
    r = engine.run(gray, vp.Params(roi=vp.roi_around(expected, radius), cap=255, polarity=1))
    return locate_m6_in_mask(r['mask'], expected, kind, radius, limits, threshold=r['level'])
