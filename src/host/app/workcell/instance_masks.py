"""Resolve duplicate instances and cross-instance disconnected mask leakage.

Original arrays remain untouched. Unexplained fragments remain for the normal
geometry rejection; this does not infer hidden parts or fill missing masks.
"""
import cv2
import numpy as np


def refine(prediction):
    masks = np.asarray(prediction['masks'], dtype=np.uint8)
    classes, scores = prediction['classes'], prediction['scores']
    components, major = {}, {}
    for i, mask in enumerate(masks):
        _, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if len(stats) <= 1:
            continue
        largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        components[i] = (labels, stats, largest)
        major[i] = labels == largest

    def body_iou(i, j):
        if i not in major or j not in major:
            return 0.
        return np.count_nonzero(major[i] & major[j]) / max(1, np.count_nonzero(major[i] | major[j]))

    def reciprocal_leak(i, j):
        # Two distinct dominant bodies can each contain a disconnected piece
        # of the other. Require both dominance and reverse ownership evidence;
        # a lone ambiguous/fragmented instance still cannot be simplified.
        for k in (i, j):
            if k not in components or np.count_nonzero(major[k]) < .6 * np.count_nonzero(masks[k]):
                return False
        labels, stats, largest = components[j]
        return any(c != largest and np.count_nonzero((labels == c) & major[i]) / max(1, stats[c, cv2.CC_STAT_AREA]) >= .9
                   for c in range(1, len(stats)))

    kept, audit = [], {}
    for i in sorted(range(len(masks)), key=lambda k: (-float(scores[k]), k)):
        duplicate = None
        for j in kept:
            cross_class = classes[i] != classes[j]
            if cross_class:
                # Class-wise detector NMS can label one body twice (M6 screw/nut).
                # Only resolve near-identical, single-body masks and boxes with
                # a clear score margin. Distinct/fragmented or ambiguous objects
                # remain visible to the normal geometry rejection.
                boxes = prediction.get('boxes')
                if (boxes is None or float(scores[j]) - float(scores[i]) < .2
                        or any(k not in components or len(components[k][1]) != 2 for k in (i, j))):
                    continue
                a, b = boxes[i], boxes[j]
                intersection = max(0., min(a[2], b[2]) - max(a[0], b[0])) * max(0., min(a[3], b[3]) - max(a[1], b[1]))
                union_box = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - intersection
                if intersection / max(1., union_box) <= .9:
                    continue
            union = np.count_nonzero(masks[i] | masks[j])
            iou = np.count_nonzero(masks[i] & masks[j]) / max(1, union)
            # Whole-mask overlap alone merges adjacent objects when masks leak
            # into each other. Their independently observed bodies must agree.
            threshold = .9 if cross_class else .5
            if iou > threshold and body_iou(i, j) > threshold:
                duplicate = (j, iou)
                break
        if duplicate is not None:
            j, iou = duplicate
            audit[j]['suppressed_duplicates'].append(dict(index=i, mask_iou=iou, body_iou=body_iou(i,j), confidence=float(scores[i])))
            if classes[i] != classes[j]:
                audit[j]['suppressed_duplicates'][-1].update(cross_class=True, suppressed_class=int(classes[i]), kept_class=int(classes[j]))
        else:
            kept.append(i)
            audit[i] = dict(suppressed_duplicates=[], removed_fragments=[])
    result = []
    for i in sorted(kept):
        cleaned = masks[i].copy()
        if i in components:
            labels, stats, largest = components[i]
            total = np.count_nonzero(cleaned)
            # Require a dominant body, and independently observed ownership of
            # every removed disconnected fragment by a different same-class body.
            if stats[largest, cv2.CC_STAT_AREA] >= .6 * total:
                for component in range(1, len(stats)):
                    if component == largest:
                        continue
                    fragment = labels == component
                    area = int(stats[component, cv2.CC_STAT_AREA])
                    for j in kept:
                        if j == i or classes[j] != classes[i] or j not in major:
                            continue
                        overlap = np.count_nonzero(major[i] & major[j]) / max(1, np.count_nonzero(major[i] | major[j]))
                        coverage = np.count_nonzero(fragment & major[j]) / max(1, area)
                        reciprocal = reciprocal_leak(i, j)
                        if overlap < .1 and coverage >= .9 and (stats[largest, cv2.CC_STAT_AREA] >= .75 * total or reciprocal):
                            cleaned[fragment] = 0
                            audit[i]['removed_fragments'].append(dict(pixels=area, owner_index=j, coverage=coverage, reciprocal_ownership=reciprocal))
                            break
        result.append((i, cleaned, audit[i]))
    return result
