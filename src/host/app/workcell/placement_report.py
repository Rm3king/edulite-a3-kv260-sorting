"""Acceptance check after a run: did each part end in its planned destination slot, and how far from the slot centre?

Uses only the head-camera detections of the frame taken after the run and the slot pixels of the configuration.
Deviation in mm uses the median millimetres-per-pixel of the same frame (from the detections' own base coordinates),
so it is a camera-based estimate of the same kind as the pick positions, not an independent ruler measurement.
"""
import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np

from .live_session import inside


def mm_per_pixel(objects, min_pixel_distance=80.):
    scales = []
    for a, b in itertools.combinations(objects, 2):
        pixels = math.dist(a['center_uv_px'], b['center_uv_px'])
        if pixels >= min_pixel_distance:
            scales.append(1000. * math.dist(a['base_xyz_m'][:2], b['base_xyz_m'][:2]) / pixels)
    return float(np.median(scales)) if scales else None


def verify_placement(points, config, expected, slots_px, match_radius_px=70., before=None, same_part_px=40.):
    """expected: [dict(part_class, slot)]; returns a report dict, never raises on a missing part.

    `before` is the frame taken before the run: a part already in the destination there (same class, within `same_part_px`) is
    reported as `preexisting`, not as an unexpected extra."""
    objects = [o for o in points['objects'] if o['part_class'] in ('screw', 'nut', 'screw_m6', 'nut_m6') and o.get('base_xyz_m') and o.get('center_uv_px') is not None]
    scale = mm_per_pixel(objects)
    destination = [o for o in objects if not inside(o['center_uv_px'], config['source_roi_uv'])]
    taken, rows = set(), []
    for item in expected:
        slot = slots_px[item['slot']]
        near = sorted(((math.dist(o['center_uv_px'], slot), i) for i, o in enumerate(destination) if i not in taken),
                      key=lambda t: t[0])
        row = dict(part_class=item['part_class'], slot=item['slot'])
        if not near or near[0][0] > match_radius_px:
            rows.append(dict(row, status='MISSING'))
            continue
        distance, index = near[0]
        taken.add(index)
        found = destination[index]
        ok = found['part_class'] == item['part_class']
        rows.append(dict(row, status='PLACED' if ok else 'WRONG_CLASS', found_class=found['part_class'],
                         deviation_px=distance, deviation_mm=None if scale is None else distance * scale))
    old = [o for o in (before or {}).get('objects', []) if o['part_class'] in ('screw', 'nut', 'screw_m6', 'nut_m6') and o.get('center_uv_px') is not None and not inside(o['center_uv_px'], config['source_roi_uv'])]
    leftover = [(i, o) for i, o in enumerate(destination) if i not in taken]
    preexisting = [dict(part_class=o['part_class'], center_uv_px=o['center_uv_px']) for i, o in leftover
                   if any(a['part_class'] == o['part_class'] and math.dist(a['center_uv_px'], o['center_uv_px']) <= same_part_px for a in old)]
    extra = [dict(part_class=o['part_class'], center_uv_px=o['center_uv_px']) for i, o in leftover
             if not any(a['part_class'] == o['part_class'] and math.dist(a['center_uv_px'], o['center_uv_px']) <= same_part_px for a in old)]
    placed = [r for r in rows if r['status'] == 'PLACED']
    worst = max((r['deviation_mm'] for r in placed if r['deviation_mm'] is not None), default=None)
    summary = f"验收 {len(placed)}/{len(rows)}" + ('' if worst is None else f"，最大偏差 {worst:.1f} mm") + (f"，多出 {len(extra)} 件" if extra else '') + (f"（另有 {len(preexisting)} 件运行前就在目的盘）" if preexisting else '')
    return dict(summary=summary, placed=len(placed), expected=len(rows), worst_deviation_mm=worst, mm_per_pixel=scale,
                parts=rows, unexpected=extra, preexisting=preexisting, basis='head camera detections, camera-estimated, not an independent measurement')


def main():
    points = json.loads(Path(sys.argv[1]).read_text())
    config = json.loads(Path(sys.argv[2]).read_text())
    expected = [dict(part_class=a.split(':')[0], slot=int(a.split(':')[1])) for a in sys.argv[3:]]
    print(json.dumps(verify_placement(points, config, expected, config['destinations']['pixels']), ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
