#!/usr/bin/env python3
"""Power of the AI PC SoC while a command runs, from the amdgpu hwmon power sensor (runs on the AI PC; read-only).

    python3 power_probe.py idle --seconds 30 --out idle.json
    python3 power_probe.py run  --label gpu_detect --frames 300 --out gpu.json -- <command that prints POWER_WINDOW_START / POWER_WINDOW_END>

`run` samples at 10 Hz for the whole command but averages only between the command's own POWER_WINDOW_START and POWER_WINDOW_END lines
(so model loading or compilation is not counted). energy per frame = mean power x window / frames.
Scope: the sensor (`power1_average`, label `PPT`) is the SoC package power of the Ryzen AI 9 HX PRO 370 (CPU + Radeon 890M + the platform's
management view of the NPU); it excludes the display, the fans, the SSD and the arm. The NPU has no separate sensor, so GPU and NPU runs
are compared by their SoC package power.
"""
import argparse
import glob
import json
import subprocess
import sys
import threading
import time


def sensor():
    for h in glob.glob('/sys/class/hwmon/hwmon*'):
        if open(h + '/name').read().strip() == 'amdgpu' and glob.glob(h + '/power1_average'):
            return h
    raise SystemExit('amdgpu hwmon power sensor not found')


class Sampler:
    def __init__(self, hz=10.):
        self.hwmon, self.samples, self.stop = sensor(), [], threading.Event()
        self.label = open(self.hwmon + '/power1_label').read().strip() if glob.glob(self.hwmon + '/power1_label') else ''
        self.thread = threading.Thread(target=self.loop, args=(1. / hz,), daemon=True)
        self.thread.start()

    def loop(self, period):
        while not self.stop.is_set():
            self.samples.append((time.time(), int(open(self.hwmon + '/power1_average').read()) / 1e6))
            time.sleep(period)

    def window(self, t0, t1):
        w = [p for t, p in self.samples if t0 <= t <= t1]
        return dict(samples=len(w), mean_w=sum(w) / len(w), min_w=min(w), max_w=max(w), seconds=t1 - t0, energy_j=sum(w) / len(w) * (t1 - t0)) if w else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mode', choices=['idle', 'run']);ap.add_argument('--seconds', type=float, default=30.);ap.add_argument('--label', default='');ap.add_argument('--frames', type=int)
    ap.add_argument('--out', required=True)
    a, rest = ap.parse_known_args()
    a.command = rest[1:] if rest[:1] == ['--'] else rest
    s = Sampler()
    if a.mode == 'idle':
        t0 = time.time();time.sleep(a.seconds);t1 = time.time()
    else:
        marks = {}
        p = subprocess.Popen(a.command, stdout=subprocess.PIPE, text=True)
        for line in p.stdout:
            for key in ('POWER_WINDOW_START', 'POWER_WINDOW_END'):
                if line.startswith(key):
                    marks[key] = time.time()
        p.wait()
        t0, t1 = marks['POWER_WINDOW_START'], marks['POWER_WINDOW_END']
    s.stop.set()
    r = dict(label=a.label or a.mode, sensor=dict(hwmon=s.hwmon, power1_label=s.label), **s.window(t0, t1))
    if a.frames:
        r.update(frames=a.frames, energy_per_frame_j=r['energy_j'] / a.frames, fps=a.frames / r['seconds'])
    open(a.out, 'w').write(json.dumps(r, indent=1))
    print(json.dumps(r))


if __name__ == '__main__':
    sys.exit(main())
