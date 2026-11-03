#!/usr/bin/env python3
"""Read the PL's own transmit-timing monitor (tx_timing_monitor.sv, registers 0x100-0x1bc). Runs on the KV260.

    cd ~/kv260_can_deploy && python3 tx_timing_report.py status
    python3 tx_timing_report.py arm [--period-us 5000]     # clear statistics, enable measuring (before a run)
    python3 tx_timing_report.py read --out timing.json     # after the run: per-axis statistics + quantile bounds
    python3 tx_timing_report.py stop                       # disable measuring (statistics are kept)

The monitor is observation-only (no output is connected to the motion permit, the CAN output gate or reset). It timestamps, with the PL's
1 us counter, each CAN transmit-command request and keeps per axis: the interval to the previous request, the interval's deviation from
the nominal period (min/max interval, peak, mean, RMS), the peak lateness of the request against its due time, the number of intervals
> 1.5 periods, the number of deviations > 50 us and a 16-bucket histogram of the deviation (<=1,2,5,10,25,50,100,250,500,1000,2000,5000,
10000,20000,50000 us, more). Counted frames: MIT frames of axes 1-6 and the gripper's 0x7016 position frames; in CSP mode (production)
the counted axis is therefore the gripper (axis index 6), sent in the same 5 ms scheduler tick as the six joints.
The timestamp is the CAN core's transmit-command request, not a wire start-of-frame time.
Only registers 0x100-0x140 and 0x180-0x1bc are touched; the owner process does not use them, so this can run beside a run.
"""
import argparse
import json
import math
import sys
import time

from canctl import CanDevice

BUCKETS = [1, 2, 5, 10, 25, 50, 100, 250, 500, 1000, 2000, 5000, 10000, 20000, 50000, None]


def quantile_bound(hist, q):
    """Upper bound (us) of the histogram bucket that contains quantile q of the deviations."""
    total = sum(hist)
    if not total:
        return None
    need, seen = q * total, 0
    for count, upper in zip(hist, BUCKETS):
        seen += count
        if seen >= need:
            return upper if upper is not None else float('inf')


def axis_stats(d, axis):
    d.write(0x108, axis)
    n = d.read(0x114)
    sum_abs = d.read(0x124) | d.read(0x128) << 32
    sum_sq = d.read(0x12c) | d.read(0x130) << 32
    hist = [d.read(0x180 + 4 * i) for i in range(16)]
    return dict(axis_index=axis, commands=d.read(0x110), intervals=n,
                min_interval_us=d.read(0x118) if n else None, max_interval_us=d.read(0x11c) if n else None, peak_abs_deviation_us=d.read(0x120),
                mean_abs_deviation_us=(sum_abs / n if n else None), rms_deviation_us=(math.sqrt(sum_sq / n) if n else None),
                peak_lateness_us=d.read(0x134), intervals_over_1p5_periods=d.read(0x13c), deviations_over_50us=d.read(0x140),
                histogram=dict(zip(['<=%dus' % b if b else '>50000us' for b in BUCKETS], hist)),
                deviation_bound_us=dict(p50=quantile_bound(hist, .5), p95=quantile_bound(hist, .95), p99=quantile_bound(hist, .99), p999=quantile_bound(hist, .999)))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('command', choices=['status', 'arm', 'read', 'stop']);p.add_argument('--period-us', type=int, default=5000);p.add_argument('--out')
    a = p.parse_args()
    d = CanDevice()
    try:
        if d.read(0x100) != 0x54494d31:
            raise SystemExit('TIM1 signature not found: the loaded firmware has no tx_timing_monitor; nothing written')
        if a.command == 'arm':
            d.write(0x104, 0)                       # disable (the period can only be changed while disabled)
            d.write(0x10c, a.period_us)
            d.write(0x104, 3)                       # bit1 clears the statistics, bit0 enables measuring
        elif a.command == 'stop':
            d.write(0x104, 0)
        report = dict(time=time.strftime('%Y-%m-%d %H:%M:%S'), signature=hex(d.read(0x100)), enabled=d.read(0x104) & 1, nominal_period_us=d.read(0x10c),
                      pl_time_us=d.read(0x408))
        if a.command in ('status', 'read'):
            report['axes'] = [axis_stats(d, i) for i in range(7)]
        text = json.dumps(report, indent=1)
        if a.out:
            open(a.out, 'w').write(text)
        print(text)
    finally:
        d.close()


if __name__ == '__main__':
    sys.exit(main())
