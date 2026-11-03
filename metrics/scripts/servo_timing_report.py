#!/usr/bin/python3
"""Per-part hover-to-descent time from board run records: stop-and-look servo vs continuous correction.

    python3 servo_timing_report.py /home/ubuntu/aipc_wrist_servo/four_integrated_2026*/execution/result.json.gz

For every part: the time from the end of the flight to the hover (PATH_COMPLETE of the departure) to the start of the
descent (ONLINE_GRASP_TASK *_approach), the number of stops (visual replies; a continuous part has none) and its mode.
Read-only; the records are the owner's own event log.
"""
import gzip, json, sys


def parts(events):
    out, cur = [], None
    for e in events:
        name, task = e.get('event'), e.get('name') or e.get('task')
        if name in ('PATH_COMPLETE',) and cur is None:
            cur = dict(hover=e['time'], replies=0, mode='stop_and_look')
        elif cur is not None and name == 'VISUAL_SERVO_REPLY':
            cur['replies'] += 1
        elif cur is not None and name == 'CONTINUOUS_SERVO_START':
            cur['mode'] = 'continuous'
        elif cur is not None and name == 'CONTINUOUS_SERVO_END':
            cur['continuous_status'] = e.get('status')
        elif cur is not None and name == 'ONLINE_GRASP_TASK' and str(task).endswith('_approach'):
            cur['hover_to_descent_s'] = e['time'] - cur.pop('hover')
            out.append(cur)
            cur = None
        elif name in ('CHAIN_NEXT_NUT', 'REBASE'):
            cur = None
        elif cur is not None and name in ('VISUAL_SERVO_FAILED', 'VERIFIED_START_RETURN'):
            cur = None
    return out


def quantile(v, p):
    return sorted(v)[min(len(v) - 1, int(p * len(v)))] if v else None


if __name__ == '__main__':
    rows = []
    for f in sys.argv[1:]:
        try:
            r = json.loads(gzip.decompress(open(f, 'rb').read()))
        except (OSError, ValueError):
            continue
        for p in parts(r.get('events', [])):
            rows.append(dict(p, run=f.split('/')[-3]))
    for mode in ('stop_and_look', 'continuous'):
        v = [x['hover_to_descent_s'] for x in rows if x['mode'] == mode]
        stops = [x['replies'] for x in rows if x['mode'] == mode]
        print(json.dumps(dict(mode=mode, parts=len(v), hover_to_descent_s=dict(p50=quantile(v, .5), p95=quantile(v, .95), max=max(v) if v else None),
                              stops_p50=quantile(stops, .5))))
    print(json.dumps(dict(rows=rows)))
