"""Replay recorded wrist frames through the board locator: PL kernel and ARM computation side by side.

    python3 replay.py DIR      # DIR/index.json: [{"name": ..., "expected": [x, y]}], DIR/NAME.raw (640x400 grey)

Writes DIR/results.json: per frame the PL and ARM observation (or refusal), whether their masks are identical, and timings.
"""
import json
import sys
import time
from pathlib import Path

import numpy as np

import vision_pre as vp
import wrist_locator as wl


def main(directory):
    d = Path(directory)
    index = json.loads((d / 'index.json').read_text())
    pl, arm = vp.VisionPL(), vp.VisionARM()
    rows = []
    for item in index:
        gray = np.fromfile(d / (item['name'] + '.raw'), np.uint8).reshape(vp.H, vp.W)
        p = vp.Params(roi=vp.roi_around(item['expected'], item.get('radius', 220.)))
        row = dict(name=item['name'])
        masks = {}
        for engine in (pl, arm):
            t = time.perf_counter()
            r = engine.run(gray, p)
            t_mask = time.perf_counter()
            try:
                obs = wl.locate_screw_in_mask(r['mask'], item['expected'], item.get('radius', 220.), threshold=r['level'])
                err = None
            except ValueError as error:
                obs, err = None, str(error)
            t_obs = time.perf_counter()
            masks[engine.name] = r['mask']
            row[engine.name] = dict(observation=obs, refusal=err, level=r['level'], count=r['count'],
                                    mask_ms=(t_mask - t) * 1e3, locate_ms=(t_obs - t_mask) * 1e3,
                                    kernel_ms=r.get('kernel_seconds', 0.) * 1e3)
        row['masks_identical'] = bool(np.array_equal(masks['pl'], masks['arm']))
        rows.append(row)
    (d / 'results.json').write_text(json.dumps(rows))
    same = sum(r['masks_identical'] for r in rows)
    for name in ('pl', 'arm'):
        q = np.percentile([r[name]['mask_ms'] for r in rows], [50, 95, 99, 100])
        print(name, 'mask ms p50 %.2f p95 %.2f p99 %.2f max %.2f' % tuple(q), 'locate ms p50 %.2f' % np.median([r[name]['locate_ms'] for r in rows]))
    print('frames', len(rows), 'identical PL/ARM masks', same)


if __name__ == '__main__':
    main(sys.argv[1])
