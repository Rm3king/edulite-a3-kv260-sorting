"""Wrist-camera localisation of a screw lying flat: centre of its extent and its axis in the image.

A screw is a long dark body on a bright tray. Unlike a nut it is not tracked with a template (it is not
rotationally symmetric); the dark body is segmented in every frame near the expected position and its
oriented bounding rectangle gives the centre (middle of the whole screw, head included) and the axis.
A screw seen from the working height is about 280 px long and often runs off the frame. When the body is cut
(`cut=True`) only its offset ACROSS the axis is measurable, so the caller must correct that component only and
leave the position along the axis to the scene camera. The axis is also reported for review and later yaw
correction.
"""
import cv2
import numpy as np


def _dark_mask(window):
    """Dark pixels of one window: Otsu between the dark body and the bright tray, kept below an absolute bound."""
    blurred = cv2.GaussianBlur(window, (0, 0), 1.5)
    level, _ = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    level = min(level, 60.)  # the screw is near black (<50); shadows and the fingertip edge reach ~100 and must stay out
    mask = (blurred < level).astype(np.uint8) * 255
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8)), level


# Fingertips are fixed in the wrist image (camera and fingers move together): x0, y0, x1, y1 at the open jaw.
FINGER_BOXES = ((0, 0, 200, 100), (360, 0, 560, 95))


def _pixel_geometry(shape, contour):
    """Centre of the extent along the principal axis, axis in [0, 180) deg, length and width of the filled body.

    Taken from the filled pixels, not from cv2.minAreaRect: the threads make that rectangle's angle jump by 3 deg between
    frames, which moved its centre by 6 px; the pixel principal axis moved the centre by 0.6 px on the same frames.
    """
    mask = np.zeros(shape, np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, -1)
    ys, xs = np.nonzero(mask)
    pts = np.stack([xs, ys], 1).astype(float)
    mean = pts.mean(0)
    _, vectors = np.linalg.eigh(np.cov((pts - mean).T))
    u = vectors[:, 1]
    if u[0] < 0:
        u = -u
    normal = np.array([-u[1], u[0]])
    along, across = (pts - mean) @ u, (pts - mean) @ normal
    center = mean + u * (along.min() + along.max()) / 2
    return float(center[0]), float(center[1]), float(np.degrees(np.arctan2(u[1], u[0])) % 180.), float(along.max() - along.min()), float(across.max() - across.min())


# Two screws whose outlines are within 25 px of the same distance from `expected` cannot be told apart. A neighbour closer
# than 50 px (~10 mm at hover height) to the grasp point is judged too close for the open jaws: a conservative guess, the
# clearance was not measured.
NEIGHBOUR_AMBIGUOUS_PX = 25.
NEIGHBOUR_CLEARANCE_PX = 50.


def locate_screw_near(gray, expected, radius=220., min_length=90., max_length=700., min_aspect=2.0, exclude=FINGER_BOXES):
    """Dict(center_px, axis_deg, length_px, width_px, bbox, score) of the elongated dark body nearest `expected`.

    axis_deg is the long-axis direction in image coordinates, in [0, 180), 0 = image x axis, 90 = image y axis.
    `cut` is True when the body runs off the image: its centre is then not the middle of the screw. The whole image is
    segmented (a search window would cut a neighbouring screw and move its centre toward `expected`), and bodies are
    ranked by the gap between `expected` and the body outline, 0 when `expected` lies on it. Neighbouring screws are
    normal on the tray: the nearest body is taken when the next one is clearly farther. Raises ValueError unless one
    plausible screw is found.
    """
    expected = np.asarray(expected, float)
    x0, y0 = max(0, int(expected[0] - radius)), max(0, int(expected[1] - radius))
    x1, y1 = min(gray.shape[1], int(expected[0] + radius)), min(gray.shape[0], int(expected[1] + radius))
    masked = gray.copy()
    for bx0, by0, bx1, by1 in exclude:
        masked[by0:by1, bx0:bx1] = 200  # bright: never part of the dark body
    if masked[y0:y1, x0:x1].size == 0:
        raise ValueError('Expected screw position outside the image')
    _, level = _dark_mask(masked[y0:y1, x0:x1])  # the threshold comes from the neighbourhood, it is applied to the whole image
    mask = cv2.morphologyEx((cv2.GaussianBlur(masked, (0, 0), 1.5) < level).astype(np.uint8) * 255, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    choices = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 400:
            continue
        cx, cy, axis, length, width = _pixel_geometry(masked.shape, contour)
        if not min_length <= length <= max_length or width < 6 or length / width < min_aspect:
            continue
        gap = max(0., -cv2.pointPolygonTest(contour, (float(expected[0]), float(expected[1])), True))
        if gap > radius:
            continue
        edge = lambda p: p[0] in (0, masked.shape[1] - 1) or p[1] in (0, masked.shape[0] - 1)
        cut = any(edge(p) for p in contour.reshape(-1, 2))  # runs off the image
        fill = area / (length * width)
        choices.append(dict(center_px=[cx, cy], axis_deg=axis, cut=bool(cut), length_px=float(length), width_px=float(width),
                            bbox=[int(v) for v in cv2.boundingRect(contour)], score=float(fill), gap=float(gap),
                            distance=float(np.hypot(cx - expected[0], cy - expected[1]))))
    if not choices:
        raise ValueError('No complete elongated dark body near the expected screw position')
    choices.sort(key=lambda c: (c['gap'], c['distance']))
    if len(choices) > 1:
        if choices[1]['gap'] < choices[0]['gap'] + NEIGHBOUR_AMBIGUOUS_PX:
            raise ValueError('More than one elongated dark body near the expected screw position')
        if choices[1]['gap'] < NEIGHBOUR_CLEARANCE_PX:
            raise ValueError('Neighbouring screw too close to the open jaws')
    best = choices[0]
    best['threshold'] = float(level)
    return best


ALONG_AXIS_LIMIT_PX = 6.


def visible_tip(observation, image_shape=(400, 640), margin=6.):
    """End of the body that is inside the image (the other end is cut), or None when neither/both ends are."""
    axis = np.deg2rad(observation['axis_deg'])
    u = np.array([np.cos(axis), np.sin(axis)])
    c = np.asarray(observation['center_px'], float)
    ends = [c + u * observation['length_px'] / 2, c - u * observation['length_px'] / 2]
    inside = [bool(margin < e[0] < image_shape[1] - margin and margin < e[1] < image_shape[0] - margin) for e in ends]
    if inside == [True, False]:
        return ends[0], -u
    if inside == [False, True]:
        return ends[1], u
    return None


def across_axis_error(goal, observations, screw_length_px=None, image_shape=(400, 640)):
    """(error vector to correct, centre it implies, spread) from per-frame observations of one stationary pose.

    A complete body gives the full 2-D error from the middle of its extent. A cut body with one end still in
    the image and a known screw length places the middle half a length from that end. A body cut at both ends
    only gives its offset across the axis, so the along-axis error is removed instead of guessing.
    """
    goal = np.asarray(goal, float)
    centers = np.array([o['center_px'] for o in observations], float)
    axis = np.deg2rad(float(np.median([o['axis_deg'] for o in observations])))
    normal = np.array([-np.sin(axis), np.cos(axis)])
    center = np.median(centers, axis=0)
    spread_across = float(np.max(np.abs((centers - center) @ normal)))
    if any(o['cut'] for o in observations):
        tips = [visible_tip(o, image_shape) for o in observations] if screw_length_px else []
        if tips and all(t is not None for t in tips):
            middles = np.array([t[0] + t[1] * screw_length_px / 2 for t in tips])
            middle = np.median(middles, axis=0)
            along = np.array([np.cos(axis), np.sin(axis)])
            # One end edge fixes the along-axis estimate, so it is noisier than the full outline (measured 4 px,
            # 0.9 mm, on settled frames). It only places the grasp on the middle of a 69 mm screw: allow 6 px
            # there, while the across-axis position, which the jaws need, keeps the strict image-stability limit.
            if float(np.max(np.abs((middles - middle) @ along))) > ALONG_AXIS_LIMIT_PX:
                raise ValueError('Screw end position unstable')
            return goal - middle, middle, float(np.max(np.abs((middles - middle) @ normal)))
        error = ((goal - center) @ normal) * normal
        return error, goal - error, spread_across
    return goal - center, center, float(np.max(np.linalg.norm(centers - center, axis=1)))


def axis_difference_deg(a, b):
    """Smallest angle between two undirected axes, in degrees."""
    d = abs(a - b) % 180.
    return min(d, 180. - d)
