#!/usr/bin/env python3
"""Aggregate the KV260 owner's run records (runs on the KV260 with the system python3; read-only).

    ssh kv260 python3 - --root /home/ubuntu/aipc_wrist_servo --out - < board_run_stats.py   (or copy it and run it there)

Per run (execution/result.json.gz): the controller heartbeat's largest gap, the continuous-alignment results (time, status, frame
age), the stop-and-look servo replies, the jaw observations. Output: per-run rows and quantiles split by alignment mode.
"""
import argparse
import glob
import gzip
import json
import os
import statistics


def quantiles(values):
    values = sorted(values)
    if not values:
        return None
    pick = lambda p: values[min(len(values) - 1, int(p * len(values)))]
    return dict(n=len(values), p50=pick(.5), p95=pick(.95), p99=pick(.99), max=values[-1])


def load(path):
    try:
        return json.loads(gzip.decompress(open(path, 'rb').read()))
    except (OSError, ValueError):
        return None


def main():
    p = argparse.ArgumentParser();p.add_argument('--root', default='/home/ubuntu/aipc_wrist_servo');p.add_argument('--since', default='20261029_000000')
    p.add_argument('--out', default='-');a = p.parse_args()
    rows, gaps = [], dict(stop_and_look=[], continuous=[], continuous_isolated=[])
    align, ages, cycles, rounds, jaw = {}, [], [], [], {}
    for d in sorted(glob.glob(os.path.join(a.root, 'four_integrated_*'))):
        name = os.path.basename(d)[len('four_integrated_'):]
        if name < a.since:
            continue
        r = load(os.path.join(d, 'execution/result.json.gz'))
        if not r or r.get('heartbeat_max_gap_s') is None:
            continue
        cont = r.get('continuous') or []
        mode = 'continuous' if cont else 'stop_and_look'
        gap = r['heartbeat_max_gap_s'] * 1e3
        gaps[mode].append(gap)
        if cont and name >= '20261030_164940':       # from the first run with the vision process on cores 2-3, nice 10
            gaps['continuous_isolated'].append(gap)
        classes = {}
        for index, job in enumerate(['job.json'] + sorted(os.path.basename(x) for x in glob.glob(os.path.join(d, 'nut*_job.json')))):
            try:
                v = json.load(open(os.path.join(d, job)))['visual_servo']['continuous']['vision']
                classes[index] = '%s_%s' % (v['part_class'], v.get('part_size', 'm12'))
            except (OSError, KeyError, ValueError, TypeError):
                pass
        for c in cont:
            key = classes.get(c.get('nut'), 'unknown')
            align.setdefault(key, []).append(dict(seconds=c['seconds'], status=c['status']))
            for t in c.get('trace', []):
                ages.append(t['age_ms'])
            cycles += c.get('cycle_ms', [])[1:]
        for e in r.get('events', []):
            if e.get('event') == 'VISUAL_SERVO_REPLY' and e.get('action') == 'GRASP':
                rounds.append(e.get('iteration', 0) + 1)
        for o in r.get('jaw_observations', []):
            if o['task'].endswith('_close'):
                jaw.setdefault('close', []).append(o['actual_rad'])
        rows.append(dict(run=name, mode=mode, status=str(r.get('status')), heartbeat_max_gap_ms=round(gap, 2), parts=r.get('nut_statuses')))
    report = dict(runs=len(rows), heartbeat_max_gap_ms={k: quantiles(v) for k, v in gaps.items()},
                  continuous_alignment_seconds={k: dict(attempts=len(v), converged=sum(x['status'] == 'CONVERGED' for x in v), converged_seconds=quantiles([x['seconds'] for x in v if x['status'] == 'CONVERGED']))
                                                for k, v in align.items()},
                  frame_age_ms=quantiles(ages), loop_cycle_ms=quantiles(cycles), stop_and_look_rounds=quantiles(rounds), rows=rows)
    text = json.dumps(report, indent=1)
    print(text) if a.out == '-' else open(a.out, 'w').write(text)


if __name__ == '__main__':
    main()
