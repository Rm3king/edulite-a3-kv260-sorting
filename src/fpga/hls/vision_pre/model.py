"""Integer reference model of the wrist-camera pre-processing run by the PL kernel `vision_pre`.

Stages: finger-box fill, 11-tap Gaussian blur (sigma 1.5), Otsu level over a region of interest (capped, or given), threshold of the
whole frame (dark or bright side), 7x7 close. The HLS kernel must reproduce `preprocess()` bit for bit; `reference()` is the OpenCV
computation of `workcell.wrist_screw_localization.locate_screw_near` up to the same mask, used to show the integer model is equivalent.
Contours, moments and screw geometry stay on the ARM.

    python3 model.py FRAME.png [...]        # level / mask mismatch against OpenCV
    python3 model.py --vectors DIR FRAME... # test vectors for the HLS test bench
"""
import sys
from fractions import Fraction
import numpy as np
import cv2

W, H = 640, 400
SIGMA, TAPS, COEF_BITS = 1.5, 11, 12
COEF = [4, 31, 147, 448, 872, 1092, 872, 448, 147, 31, 4]
FINGER_BOXES = ((0, 0, 200, 100), (360, 0, 560, 95))
FILL, CAP = 200, 60


def kernel():
    x = np.arange(TAPS) - TAPS // 2
    k = np.exp(-x * x / (2 * SIGMA * SIGMA))
    k /= k.sum()
    q = np.round(k * (1 << COEF_BITS)).astype(np.int64)
    q[TAPS // 2] += (1 << COEF_BITS) - q.sum()
    return q


assert list(kernel()) == COEF


def blur(gray):
    """Separable fixed-point Gaussian, reflect-101 borders: horizontal pass keeps 4 fractional bits, vertical pass rounds to 8 bits."""
    r = TAPS // 2
    g = gray.astype(np.int64)
    p = np.pad(g, ((0, 0), (r, r)), mode='reflect')
    h = sum(COEF[i] * p[:, i:i + g.shape[1]] for i in range(TAPS))
    h = (h + (1 << (COEF_BITS - 4 - 1))) >> (COEF_BITS - 4)
    p = np.pad(h, ((r, r), (0, 0)), mode='reflect')
    v = sum(COEF[i] * p[i:i + g.shape[0], :] for i in range(TAPS))
    return ((v + (1 << (COEF_BITS + 4 - 1))) >> (COEF_BITS + 4)).astype(np.uint8)


def otsu(hist):
    """Otsu level as OpenCV computes it (first maximum of the between-class variance), exact: score = (s0*w1 - s1*w0)^2 / (w0*w1)."""
    n, total = int(sum(hist)), sum(i * int(h) for i, h in enumerate(hist))
    best, level, w0, s0 = None, 0, 0, 0
    for t in range(256):
        w0 += int(hist[t]); s0 += t * int(hist[t])
        w1 = n - w0
        if w0 == 0 or w1 == 0:
            continue
        score = Fraction((s0 * w1 - (total - s0) * w0) ** 2, w0 * w1)
        if best is None or score > best:
            best, level = score, t
    return level


def fill_boxes(gray, boxes=FINGER_BOXES, fill=FILL):
    out = gray.copy()
    for x0, y0, x1, y1 in boxes:
        out[y0:y1, x0:x1] = fill
    return out


def close7_rows(bits):
    """7x7 close (dilate then erode, outside pixels neutral) on a 0/1 array, with the row-wide shift/OR/AND the PL uses."""
    h, w = bits.shape
    rows = [int(''.join('1' if b else '0' for b in r[::-1]), 2) for r in bits]   # bit x = pixel x
    full = (1 << w) - 1

    def hshift(v, d, fill_ones):
        s = v >> d if d >= 0 else (v << -d) & full           # bit x of result = bit x+d of v
        if fill_ones:
            s |= (full << (w - d)) & full if d > 0 else (((1 << -d) - 1) if d < 0 else 0)
        return s & full

    hd = []
    for v in rows:
        o = 0
        for d in range(-3, 4):
            o |= hshift(v, d, False)
        hd.append(o)
    dil = []
    for y in range(h):
        o = 0
        for j in range(y - 3, y + 4):
            if 0 <= j < h:
                o |= hd[j]
        dil.append(o)
    he = []
    for v in dil:
        o = full
        for d in range(-3, 4):
            o &= hshift(v, d, True)
        he.append(o)
    out = []
    for y in range(h):
        o = full
        for j in range(y - 3, y + 4):
            if 0 <= j < h:
                o &= he[j]
        out.append(o)
    res = np.zeros((h, w), np.uint8)
    for y, v in enumerate(out):
        for x in range(w):
            res[y, x] = (v >> x) & 1
    return res


def preprocess(gray, roi, boxes=FINGER_BOXES, fill=FILL, cap=CAP, level_mode=0, manual=0, polarity=0, close=1):
    """roi = (x, y, w, h). Returns (mask 0/1 array, level)."""
    masked = fill_boxes(gray, boxes, fill)
    if level_mode == 0:
        x, y, w, h = roi
        level = min(otsu(np.bincount(blur(masked[y:y + h, x:x + w]).ravel(), minlength=256)), cap)
    else:
        level = manual
    b = blur(masked)
    t = (b > level) if polarity else (b < level)
    t = t.astype(np.uint8)
    return (close7_rows(t) if close else t), int(level)


def reference(gray, roi, boxes=FINGER_BOXES, fill=FILL, cap=CAP):
    """The OpenCV computation of locate_screw_near up to the mask (dark screw, Otsu capped)."""
    masked = fill_boxes(gray, boxes, fill)
    x, y, w, h = roi
    window = cv2.GaussianBlur(masked[y:y + h, x:x + w], (0, 0), 1.5)
    level, _ = cv2.threshold(window, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    level = min(level, float(cap))
    mask = (cv2.GaussianBlur(masked, (0, 0), 1.5) < level).astype(np.uint8) * 255
    return (cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8)) > 0).astype(np.uint8), level


def roi_for(expected, radius=220):
    ex, ey = expected
    x0, y0 = max(0, int(ex - radius)), max(0, int(ey - radius))
    x1, y1 = min(W, int(ex + radius)), min(H, int(ey + radius))
    return (x0, y0, x1 - x0, y1 - y0)


def write_vectors(directory, frames):
    """Index line: name roi_x roi_y roi_w roi_h level_mode manual polarity close cap level count."""
    import os, random
    os.makedirs(directory, exist_ok=True)
    rnd = random.Random(7)
    lines = []
    for n, path in enumerate(frames):
        g = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        assert g.shape == (H, W)
        kind = n % 6
        roi = roi_for((rnd.randint(150, 500), rnd.randint(120, 300)))
        cap, mode, manual, pol = CAP, 0, 0, 0
        if kind == 3:      # manual level, either side
            mode, manual, pol = 1, rnd.randint(70, 140), n % 12 == 3
        elif kind == 4:    # bright parts, Otsu not capped (the M6 setting)
            cap, pol = 255, 1
        elif kind == 5:    # small region of interest at an unaligned column, Otsu not capped
            cap, roi = 255, (rnd.randint(1, 600) | 1, rnd.randint(0, 380), 12 + rnd.randint(0, 20), 12 + rnd.randint(0, 8))
            roi = (min(roi[0], W - roi[2]), min(roi[1], H - roi[3]), roi[2], roi[3])
        close = 0 if n % 5 == 4 else 1
        mask, level = preprocess(g, roi, cap=cap, level_mode=mode, manual=manual, polarity=pol, close=close)
        base = f'{directory}/v{n:03d}'
        g.tofile(base + '.img')
        np.packbits(mask, axis=1, bitorder='little').tofile(base + '.mask')
        lines.append(f"v{n:03d} {roi[0]} {roi[1]} {roi[2]} {roi[3]} {mode} {manual} {int(pol)} {close} {cap} {level} {int(mask.sum())}")
    open(directory + '/index.txt', 'w').write('\n'.join(lines) + '\n')


if __name__ == '__main__':
    args = sys.argv[1:]
    if args and args[0] == '--vectors':
        write_vectors(args[1], args[2:])
    else:
        for path in args:
            g = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            for e in ((320, 200), (200, 250), (450, 180)):
                roi = roi_for(e)
                m, lv = preprocess(g, roi)
                rm, rlv = reference(g, roi)
                print(path, e, 'level', lv, rlv, 'mask diff px', int((m != rm).sum()), 'of', int(rm.sum()))
