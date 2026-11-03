"""Single M6 nut B -> A filming task; reuse the normal planner, wrist servo and motion owner."""
import copy
import json
import math
import subprocess
from pathlib import Path

SLOT = 4
INTERIOR_UV = [1450., 385.]  # middle of the free area between the existing right-tray nuts


def requested(task):
    steps = task.get('steps', [])
    return (task.get('action') == 'plan' and len(steps) == 1
            and steps[0]['part_class'] == 'nut_m6' and steps[0]['region'] == 'left'
            and steps[0]['destination'] == 'A' and steps[0]['quantity'] in (1, 'all'))


def overrides(config):
    source, target = config['trays']['B'], config['trays']['A']
    pixels = copy.deepcopy(target['slots_uv'][:5])
    pixels[SLOT] = list(INTERIOR_UV)
    # The left pickup uses the opposite symmetric jaw orientation. Choose it
    # before the existing planner applies its tool-frame hover bias, so a later
    # IK orientation flip cannot reverse the meaning of that bias.
    refs = copy.deepcopy(config['reference_objects'])
    n = source['plane']['normal_base']
    for obj in refs:
        if obj.get('axis_base'):
            a = obj['axis_base']; dot = sum(x*y for x, y in zip(a, n))
            a = [x-dot*y for x, y in zip(a, n)]; length = math.sqrt(sum(x*x for x in a))
            obj['axis_base'] = [-x/length for x in a]
    return dict(reverse_small_nut=True, goal_direction='B->A', source_roi_uv=source['roi_uv'],
                destination_floor=target['plane'], reference_objects=refs,
                destinations=dict(pixels=pixels, rotation=target['rotation'],
                                  scope='Single M6 nut B->A; interior A slot 4, fresh occupancy and geometry required'))


def fixed_marker(obj, snapshot, config):
    """The tiny white cross sticker in this calibrated scene is sometimes called an M6 nut.

    Exclude only its small footprint and near-floor depth from inventory; raw
    detections remain in the scene and swept geometry. A nut over the sticker,
    a larger footprint or a different calibration is not excluded.
    """
    if (snapshot.get('calibration_id') != 'q18_scene_20261026_132258'
            or obj.get('detected_class', obj.get('part_class')) != 'nut_m6'):
        return False
    uv, box, xyz = obj.get('center_uv_px'), obj.get('box_xyxy_px'), obj.get('camera_xyz_m')
    if not uv or not box or not xyz or not all(math.isfinite(x) for x in (*uv, *box, *xyz)):
        return False
    if (math.dist(uv, [342., 594.]) > 6. or not (328. <= box[0] < box[2] <= 356.
            and 580. <= box[1] < box[3] <= 607.) or max(box[2]-box[0], box[3]-box[1]) > 26.):
        return False
    plane = config['trays']['B']['plane']
    height = sum(a*b for a, b in zip(plane['normal_camera'], xyz)) + plane['offset_m']
    return 0. <= height <= .0065


def tray_objects(snapshot, config, tray=None):
    """Use the existing depth-aware robot exclusion; retain all four real classes."""
    from .inventory_goal import inventory
    original = {o['detection_index']: o for o in snapshot['objects']}
    marker_ids = {i for i, obj in original.items() if fixed_marker(obj, snapshot, config)}
    snapshot['fixture_marker_exclusions'] = sorted(marker_ids)
    normalized = [dict(o, part_class=o['part_class'].split('_')[0]) for o in original.values()
                  if o['part_class'].split('_')[0] in ('screw', 'nut') and o['detection_index'] not in marker_ids]
    observed = inventory(dict(snapshot, objects=normalized), config)
    return [original[o['detection_index']] for key in ((tray,) if tray else ('A', 'B'))
            for o in observed['trays'][key]]


def check_destination(snapshot, config):
    point = config['destinations']['pixels'][SLOT]
    objects = tray_objects(snapshot, config, 'A')
    if any(math.dist(point, o['center_uv_px']) <= 70. for o in objects):
        raise ValueError('右盘预定落点已有零件，未开始反向搬运')


def verify_counts(report, before, after, config):
    """An old nut in A must never count as the newly transferred nut."""
    counts, totals = {}, {}
    for name, frame in [('before', before), ('after', after)]:
        counts[name] = {tray: sum(o['part_class'] == 'nut_m6' for o in tray_objects(frame, config, tray))
                        for tray in ('A', 'B')}
        totals[name] = {tray: len(tray_objects(frame, config, tray)) for tray in ('A', 'B')}
    ok = (counts['before']['B'] == 1 and counts['after']['B'] == 0
          and counts['after']['A'] == counts['before']['A'] + 1)
    report['transfer_counts'] = counts
    report['tray_total_counts'] = totals
    if not ok:
        for part in report['parts']:
            if part['status'] == 'PLACED':
                part['status'] = 'UNCONFIRMED_TRANSFER'
        report['placed'] = 0
        report['summary'] = '未确认左盘减少一颗、右盘增加一颗小螺母'
    return report


def prepare_surfaces(out, config):
    """Reuse validated surface projection separately for each tray; keep every obstacle."""
    out = Path(out)
    points = Path(json.loads((out/'input.json').read_text())['points'])
    snapshot = json.loads(points.read_text())
    work = json.loads(Path(config['workplane']).read_text())
    app = Path(__file__).resolve().parents[2]
    reports = {}
    for tray in ('A', 'B'):
        local = copy.deepcopy(work)
        local['plane'] = config['trays'][tray]['plane']
        local['config']['fit_roi_uv'] = config['trays'][tray]['roi_uv']
        plane_file = out/('workplane_'+tray+'.json')
        plane_file.write_text(json.dumps(local))
        target = out/('surface_'+tray)
        subprocess.run([str(app/'.venv-camera/bin/python'), str(app/'src/preview_workplane_points.py'),
                        '--points', str(points), '--workplane', str(plane_file), '--heights', config['heights'],
                        '--calibration', config['calibration'], '--out', str(target)], check=True, stdout=subprocess.DEVNULL)
        reports[tray] = json.loads((target/'surface_points.json').read_text())
    from .live_session import inside
    by_tray = {key: {o['detection_index']: o for o in report['objects']} for key, report in reports.items()}
    rows = []
    for raw in snapshot['objects']:
        tray = 'B' if raw.get('center_uv_px') and inside(raw['center_uv_px'], config['trays']['B']['roi_uv']) else 'A'
        rows.append(dict(by_tray[tray][raw['detection_index']], tray=tray))
    target = out/'target_geometry';target.mkdir()
    (target/'surface_points.json').write_text(json.dumps(dict(reports['B'], objects=rows,
        note='Existing per-tray planes; original calibration, heights and collision checks retained'), indent=2))


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser();p.add_argument('--out', required=True);p.add_argument('--config', required=True)
    args = p.parse_args()
    prepare_surfaces(args.out, json.loads(Path(args.config).read_text()))
