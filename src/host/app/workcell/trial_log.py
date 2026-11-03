"""Append one row per attended run to reports/task_trials.csv, built only from the run folder's own files (no hardware access).

    python -m workcell.trial_log reports/four_integrated_...        # (re)build the row of a finished folder
    python -m workcell.trial_log --note "part pushed by hand" ROOT   # add an operator note to its row

Success is `COMPLETED` for every attempted part AND a placement report with every expected part PLACED. `intervention` is left empty
unless the operator adds a note: the run itself cannot know that a person moved a part or held the arm.
"""
import csv
import json
import sys
import time
from pathlib import Path

FIELDS = ['time', 'run', 'parts', 'outcome', 'placed', 'expected', 'worst_deviation_mm', 'preexisting', 'unexpected', 'launch_s', 'end_s',
          'motion_s', 'temp_max_c', 'note']


def read(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def row_for(root, parts=None, note=''):
    root = Path(root)
    marks = {m['name']: m['t'] for m in read(root / 'timeline.json') or []}
    placement, failure, results = read(root / 'placement_report.json'), read(root / 'failure.json'), read(root / 'results.json') or []
    statuses = [r['status'] for r in results if isinstance(r, dict) and 'status' in r]
    summary = next((r['summary'] for r in results if isinstance(r, dict) and 'summary' in r), {}) or {}
    if failure or (root / 'FAULT_ARM_HELD.json').exists():
        outcome = 'FAULT' if (root / 'FAULT_ARM_HELD.json').exists() else 'FAILED_BEFORE_MOTION' if 'owner_launched' not in marks else 'FAILED'
    elif statuses and all(str(s).startswith('COMPLETED') for s in statuses) and placement and placement['placed'] == placement['expected']:
        outcome = 'SUCCESS'
    elif statuses and all(str(s).startswith('COMPLETED') for s in statuses):
        outcome = 'MOVED_NOT_VERIFIED'
    else:
        outcome = 'PARTIAL'
    temps = [v for key, v in (summary.get('temperatures_c') or {}).items()] if isinstance(summary.get('temperatures_c'), dict) else []
    launch, end = marks.get('owner_launched'), marks.get('end')
    return dict(time=time.strftime('%Y-%m-%d %H:%M:%S'), run=root.name, parts=' '.join(parts or []), outcome=outcome,
                placed=placement and placement['placed'], expected=placement and placement['expected'],
                worst_deviation_mm=placement and placement['worst_deviation_mm'] and round(placement['worst_deviation_mm'], 1),
                preexisting=placement and len(placement.get('preexisting', [])), unexpected=placement and len(placement['unexpected']),
                launch_s=launch, end_s=end, motion_s=None if launch is None or end is None else round(end - launch, 1),
                temp_max_c=max(temps) if temps else None, note=note)


def append(root, parts=None, note='', csv_path=None):
    csv_path = Path(csv_path or Path(root).resolve().parent / 'task_trials.csv')
    rows = []
    if csv_path.exists():
        with csv_path.open() as handle:
            rows = [r for r in csv.DictReader(handle) if r['run'] != Path(root).name]
    rows.append({k: '' if v is None else v for k, v in row_for(root, parts, note).items()})
    with csv_path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    return csv_path


if __name__ == '__main__':
    args = sys.argv[1:]
    note = ''
    if args[:1] == ['--note']:
        note, args = args[1], args[2:]
    print(append(args[0], note=note))
