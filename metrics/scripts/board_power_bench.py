#!/usr/bin/env python3
"""KV260 board power while the wrist-image mask runs on the ARM core or in the PL (runs on the KV260; no motor, no CAN).

    cd ~/kv260_can_deploy && python3 board_power_bench.py --mode idle|arm|pl --seconds 20 --frame /tmp/pd_frame.raw --out x.json

Power is read from the board's INA260 (hwmon `ina260_u14`, `power1_input`) at about 20 Hz by a thread of the same process. The load is the
same mask computation on the same 640x400 frame (ARM: single-core C -O3; PL: the HLS kernel with 0.5 ms sleep polling), back to back.
Scope: the INA260 measures the K26 module/board supply branch it sits on; the arm, the cameras and the AI PC are not included.
"""
import argparse
import glob
import json
import threading
import time

import numpy as np

import vision_pre as vp


def ina():
    for h in glob.glob('/sys/class/hwmon/hwmon*'):
        if open(h + '/name').read().strip() == 'ina260_u14':
            return h
    raise SystemExit('ina260_u14 not found')


def main():
    ap = argparse.ArgumentParser();ap.add_argument('--mode', choices=['idle', 'arm', 'pl'], required=True);ap.add_argument('--seconds', type=float, default=20.)
    ap.add_argument('--frame', default='/tmp/pd_frame.raw');ap.add_argument('--out', required=True)
    a = ap.parse_args()
    hw = ina();samples, stop = [], threading.Event()

    def sample():
        while not stop.is_set():
            samples.append((time.time(), int(open(hw + '/power1_input').read()) / 1e6, int(open(hw + '/in1_input').read()), int(open(hw + '/curr1_input').read())))
            time.sleep(.05)
    threading.Thread(target=sample, daemon=True).start()
    gray = np.fromfile(a.frame, np.uint8).reshape(vp.H, vp.W);p = vp.Params(roi=vp.roi_around((320, 200)))
    call = None
    if a.mode == 'arm':
        arm = vp.VisionARM();call = lambda: arm.run(gray, p)
    elif a.mode == 'pl':
        pl = vp.VisionPL();call = lambda: pl.run(gray, p, poll_sleep=.0005)
    time.sleep(1.)                                   # settle
    t0, frames = time.time(), 0
    while time.time() - t0 < a.seconds:
        if call:
            call();frames += 1
        else:
            time.sleep(.05)
    t1 = time.time();stop.set()
    w = [x[1] for x in samples if t0 <= x[0] <= t1]
    r = dict(mode=a.mode, seconds=t1 - t0, frames=frames, fps=frames / (t1 - t0) if frames else 0., samples=len(w), mean_w=sum(w) / len(w), min_w=min(w), max_w=max(w),
             bus_mv=sum(x[2] for x in samples if t0 <= x[0] <= t1) / len(w), sensor=hw.split('/')[-1] + ' ina260_u14')
    r['energy_j'] = r['mean_w'] * r['seconds']
    if frames:
        r['energy_per_frame_mj'] = r['energy_j'] / frames * 1e3
    open(a.out, 'w').write(json.dumps(r, indent=1))
    print(json.dumps(r))


if __name__ == '__main__':
    main()
