"""Capture one manually posed Q18 view over the existing read-only camera tunnel.

Run with .venv-camera Python; no Qt, robot or device-control imports.
Each burst is ONE intrinsic-calibration pose, not three independent views.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
from urllib.request import urlopen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:18766')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--capture', action='store_true')
    args = parser.parse_args()
    if not args.capture:
        print(json.dumps(dict(status='DRY_RUN', robot_commands=0)))
        return
    import cv2
    import numpy as np
    args.out.mkdir(parents=True, exist_ok=False)
    records = []
    for index in range(3):
        with urlopen(args.url.rstrip('/')+'/frame.jpg', timeout=3) as response:
            data = response.read()
            headers = dict(response.headers)
        record = dict(index=index, headers=headers,
                      sha256=hashlib.sha256(data).hexdigest())
        (args.out/f'frame_{index}.jpg').write_bytes(data)
        gray = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
        if gray is None or gray.shape != (400, 640):
            raise RuntimeError('Unexpected camera image format')
        found, corners = cv2.findChessboardCornersSB(
            gray, (17, 17), flags=cv2.CALIB_CB_NORMALIZE_IMAGE |
            cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        record['found'] = bool(found)
        if found:
            record['corners_px'] = corners.reshape(-1, 2).tolist()
            overlay = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(overlay, (17, 17), corners, True)
            cv2.imwrite(str(args.out/f'overlay_{index}.jpg'), overlay)
        records.append(record)
        time.sleep(.2)
    # Detection is a candidate, not an automatic calibration accuracy verdict.
    fresh = (len({r['headers'].get('X-Frame-Sequence') for r in records}) == 3
             and len({r['headers'].get('X-Camera-Session') for r in records}) == 1
             and all(0 <= float(r['headers']['X-Board-Frame-Age-S']) < 1
                     for r in records))
    result = dict(board='Q18-100-4.5', inner_corners=[17, 17], square_size_m=.0045,
                  image_size=[640, 400], frames=records, fresh_distinct_frames=fresh,
                  status='REVIEW_REQUIRED', selected_frame=None,
                  calibration_verified=False, robot_commands=0,
                  note='Select one sharp frame after review; do not average moving views.')
    (args.out/'observations.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(out=str(args.out), found=[r['found'] for r in records],
                          fresh_distinct_frames=fresh, status=result['status'])))


if __name__ == '__main__':
    main()
