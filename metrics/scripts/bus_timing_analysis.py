#!/usr/bin/env python3
"""Wire-level CAN timing from a logic-analyzer edge export (Kingst LA2016, CAN-H on one channel, 50 MHz, threshold 2.8 V).

    python3 bus_timing_analysis.py capture.csv --out bus_timing.json [--figure-dir DIR]

Input: the "Export data" CSV of KingstVIS (`Time[s], Channel N`; one row per edge; high = CAN dominant for CAN-H).
Steps: segment frames (a recessive stretch longer than 9.5 bit times ends a frame) -> run lengths to bits -> destuff -> extended-ID decode (ID, DLC)
-> classify the KV260's own frames: MIT command frames of motors 1-6 (type 1) and the CSP position frames of motors 1-7 (type 18, parameter 0x7016)
-> per axis, the start-of-frame to start-of-frame interval and its deviation from the nominal 5 ms period.
Reported: the enable phase (50 Hz hold frames before the PL stream starts) is separated from the steady stream (everything after the last interval > 10 ms of
axis 1); interval and deviation percentiles for the steady stream and for the whole capture; how many large deviations coincide with an interleaved
position-read transaction (type 17, higher arbitration priority than the stream's type 18); bus occupancy; bit time.
All times are wire start-of-frame times at 20 ns resolution.
"""
import argparse
import json
import statistics
import sys
from collections import Counter

import numpy as np

NOMINAL = 5e-3


def load(path):
    t, lvl = [], []
    with open(path) as fh:
        next(fh)
        for line in fh:
            a, b = line.split(',')
            t.append(float(a));lvl.append(int(b))
    return np.array(t), np.array(lvl, dtype=np.int8)


def deglitch(t, lvl, minimum=0.3e-6):
    """Drop pulses shorter than `minimum` (both edges)."""
    keep = np.ones(len(t), bool)
    width = np.diff(t, append=t[-1] + 1)
    i = 0
    short = np.where(width < minimum)[0]
    for i in short:
        if keep[i] and i + 1 < len(t) and keep[i + 1]:
            keep[i] = keep[i + 1] = False
    return t[keep], lvl[keep]


def frames(t, lvl, bit, idle_bits=9.5):
    """Yield (start_time, edges_times, edge_levels) per frame; dominant = level 1."""
    width = np.diff(t, append=t[-1] + 1)
    starts = np.where((lvl == 1) & np.concatenate(([True], (np.diff(t) * 1) > 0)))[0]
    out, i, n = [], 0, len(t)
    while i < n:
        if lvl[i] != 1:
            i += 1;continue
        j = i
        while j + 1 < n and not (lvl[j] == 0 and width[j] > idle_bits * bit):
            j += 1
        out.append((i, j));i = j + 1
    return out


def decode(t, lvl, i, j, bit):
    bits = []
    for k in range(i, j):
        n = int(round((t[k + 1] - t[k]) / bit))
        bits.extend([0 if lvl[k] == 1 else 1] * max(n, 1))      # CAN logic: dominant = 0
    pos, run, last, out = 0, 0, None, []
    def take(count):
        nonlocal pos, run, last
        value = 0
        for _ in range(count):
            while True:
                if pos >= len(bits):
                    raise IndexError
                b = bits[pos];pos += 1
                if run == 5:                                  # stuff bit
                    run = 1;last = b
                    continue
                run = run + 1 if b == last else 1
                last = b
                break
            value = value << 1 | b
        return value
    try:
        take(1)                                               # SOF
        base = take(11);take(1);ide = take(1)
        if not ide:
            return None
        ext = take(18);take(1);take(2)
        dlc = take(4)
        data = [take(8) for _ in range(min(dlc, 2))]
        return dict(id=base << 18 | ext, dlc=dlc, data=data)
    except IndexError:
        return None


def pct(v, q):
    return float(np.percentile(v, q))


def interval_stats(starts):
    """Interval and |interval - nominal| percentiles; streaming window = intervals of a continuous run (<= 10 ms)."""
    d = np.diff(starts)
    def block(x):
        if len(x) == 0:
            return None
        dev = np.abs(x - NOMINAL) * 1e6
        return dict(intervals=len(x), interval_ms=dict(min=float(x.min() * 1e3), p50=pct(x, 50) * 1e3, p95=pct(x, 95) * 1e3, p99=pct(x, 99) * 1e3, p999=pct(x, 99.9) * 1e3, max=float(x.max() * 1e3)),
                    deviation_us=dict(p50=pct(dev, 50), p95=pct(dev, 95), p99=pct(dev, 99), p999=pct(dev, 99.9), max=float(dev.max()), rms=float(np.sqrt((dev ** 2).mean()))),
                    over_50us=int((dev > 50).sum()), over_250us=int((dev > 250).sum()), over_1ms=int((dev > 1000).sum()))
    return dict(all=block(d), streaming=block(d[d <= 10e-3]), gaps_over_10ms=int((d > 10e-3).sum()), longest_gap_ms=float(d.max() * 1e3))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv');ap.add_argument('--out', required=True);ap.add_argument('--figure-dir')
    a = ap.parse_args()
    t, lvl = load(a.csv)
    t, lvl = t[1:], lvl[1:]                                   # first row is the pre-trigger level
    t, lvl = deglitch(t, lvl)
    width = np.diff(t)
    # bit time: same-polarity (rising to rising) intervals of 2..12 bits
    rise = t[lvl == 1]
    per = np.diff(rise)
    per = per[(per > 1.5e-6) & (per < 12e-6)]
    bit = float(np.median(per / np.round(per / 1e-6)))
    fr = frames(t, lvl, bit)
    rows = []
    for i, j in fr:
        d = decode(t, lvl, i, j, bit)
        rows.append((t[i], t[j] - t[i], d))
    ids = Counter()
    axis = {k: [] for k in range(1, 8)}
    ok = 0
    for start, dur, d in rows:
        if d is None:
            continue
        ok += 1
        ident = d['id'];ids[ident] += 1
        typ, motor = ident >> 24 & 31, ident & 255
        if typ == 1 and 1 <= motor <= 6:
            axis[motor].append(start)                       # MIT command frame
        elif typ == 18 and 1 <= motor <= 7 and d['data'][:2] == [0x16, 0x70]:
            axis[motor].append(start)                       # CSP position frame (parameter 0x7016), the production stream
    total = t[-1] - t[0]
    occupancy = sum(dur for _, dur, _ in rows) / total
    report = dict(capture=a.csv.split('/')[-1], span_s=float(total), edges=int(len(t)), frames=len(rows), frames_decoded=ok, bit_time_us=bit * 1e6, bit_rate_mbps=1 / bit / 1e6,
                  bus_occupancy_percent=occupancy * 100, frames_per_second=len(rows) / total,
                  id_counts={f'0x{k:08X}': v for k, v in ids.most_common(24)}, axes={})
    for m, s in axis.items():
        if len(s) > 10:
            report['axes'][f'motor_{m}'] = dict(frames=len(s), **interval_stats(np.array(s)))
    # steady stream of axis 1 (the first frame of each 5 ms tick)
    first = np.array(axis[1]);gap = np.where(np.diff(first) > 10e-3)[0]
    steady = first[gap[-1] + 1:] if len(gap) else first
    dev = np.abs(np.diff(steady) - NOMINAL) * 1e6
    reads = np.array([st for st, _, d in rows if d and (d['id'] >> 24 & 31) == 17])
    between = lambda k: bool(((reads >= steady[k] - 300e-6) & (reads <= steady[k + 1])).any()) if len(reads) else False
    big = np.where(dev > 250)[0];small = np.where(dev <= 50)[0]
    ds = np.diff(steady)
    in_steady = [(st, du) for st, du, d in rows if steady[0] <= st <= steady[-1]]
    report['steady_stream'] = dict(
        start_s=float(steady[0]), end_s=float(steady[-1]), seconds=float(steady[-1] - steady[0]), ticks=int(len(steady)),
        enable_phase=dict(first_axis1_frame_s=float(first[0]), frames_before_stream=int(len(first) - len(steady)), intervals_over_10ms=int(len(gap))),
        interval_ms=dict(p50=pct(ds, 50) * 1e3, p95=pct(ds, 95) * 1e3, p99=pct(ds, 99) * 1e3, p999=pct(ds, 99.9) * 1e3, max=float(ds.max() * 1e3)),
        deviation_us=dict(p50=pct(dev, 50), p95=pct(dev, 95), p99=pct(dev, 99), p999=pct(dev, 99.9), max=float(dev.max()), rms=float(np.sqrt((dev ** 2).mean()))),
        over_50us=int((dev > 50).sum()), over_250us=int(len(big)), over_1ms=int((dev > 1000).sum()),
        large_deviation_with_read_between=int(sum(between(k) for k in big)), small_deviation_with_read_between=int(sum(between(k) for k in small)), small_deviation_total=int(len(small)),
        bus_occupancy_percent=float(100 * sum(du for _, du in in_steady) / (steady[-1] - steady[0])), frames_per_second=float(len(in_steady) / (steady[-1] - steady[0])))
    if a.figure_dir:
        np.savez_compressed(a.figure_dir + '/bus_frames.npz', start=np.array([r[0] for r in rows]), duration=np.array([r[1] for r in rows]),
                            ident=np.array([r[2]['id'] if r[2] else -1 for r in rows], dtype=np.int64), steady_start=steady[0])
    open(a.out, 'w').write(json.dumps(report, indent=1))
    print(json.dumps({k: v for k, v in report.items() if k != 'axes'}, indent=1))
    print(json.dumps(report['steady_stream'], indent=1))
    for m, v in report['axes'].items():
        s = v['streaming']
        print(m, 'frames', v['frames'], 'streaming intervals', s['intervals'], 'dev us p50 %.1f p95 %.1f p99 %.1f max %.1f' % (s['deviation_us']['p50'], s['deviation_us']['p95'], s['deviation_us']['p99'], s['deviation_us']['max']),
              'gaps>10ms', v['gaps_over_10ms'], 'longest gap %.1f ms' % v['longest_gap_ms'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
