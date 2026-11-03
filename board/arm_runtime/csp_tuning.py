"""Explicit, volatile, disabled-only J2/J3 tuning. Never enables any motor.

Default validates arguments offline. --run commissions the write/read/restore
path. --leave-candidate additionally leaves ONE reviewed parameter for an
attended motion comparison; the original values are saved before any write.
"""
import argparse
import fcntl
import json
import math
import struct
import time
from pathlib import Path
from motion_executor import CanDevice, Session, parameter
from csp_commission import bits

PARAMETERS = {'filter': (0x7021, .05, .15), 'spd_kp': (0x701f, 4., 6.),
              'spd_ki': (0x7020, .01, .02), 'j3_loc_kp': (0x701e, 20., 30.)}


def check_candidate(name, value):
    index, lo, hi = PARAMETERS[name]
    if not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError(f'{name} must be finite in {lo}..{hi}')
    return index, (3,) if name == 'j3_loc_kp' else (2, 3)


def refresh(s, mid):
    stamp = time.monotonic()
    s.send(parameter(mid, 0x7028, 6000, True))
    deadline = stamp + .08
    while time.monotonic() < deadline:
        s.pump(.002)
        if s.feedback.get(mid, {}).get('time', -1) >= stamp:
            f = s.require(mid, 0)
            if f['temperature'] >= 65:
                raise RuntimeError('Cool while supported and disabled before tuning')
            return dict(f)
    raise TimeoutError(f'Fresh disabled feedback missing for J{mid}')


def commission(name, value, out, leave=False):
    index, axes = check_candidate(name, value)
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    report = dict(status='RUNNING', parameter=name, index=hex(index), candidate=value,
                  leave_candidate=leave, enable_commands=0, disable_commands=0,
                  flash_writes=0, zero_writes=0, commands=[], frames=[], feedback=[],
                  readbacks={}, reset_confirmed=[], stop_errors={}, originals={})
    changed = []
    with open('/tmp/kv260_gripper_commission.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        d = CanDevice()
        s = Session(d, report)
        owned = False
        try:
            if d.read(0x470) != 0x60003:
                raise RuntimeError('Only non-aborting ABI 0x60003 permits this test')
            if any(d.read(a) for a in (0x404, 0x418, 0x5a0, 0x5a8, 0x800, 0xb40, 0x41c)):
                raise RuntimeError('Require released idle controller; never take over another owner')
            report['before'] = d.status()
            owned = True
            d.configure('normal')
            positions = [s.read(mid, 0x7019) for mid in range(1, 8)]
            modes = [s.read(mid, 0x7005, True) for mid in range(1, 8)]
            if any(m != 5 for m in modes):
                raise RuntimeError('Existing CSP run mode required; no mode change in this test')
            report['originals'] = {str(mid): s.read(mid, index) for mid in axes}
            # Persist rollback material before installing guards or writing gains.
            (out/'originals.json').write_text(json.dumps(report, indent=2)+'\n')
            d.stop()
            d.write(0x5a0, 127)
            d.write(0x5a4, 100000)
            for mid, q in enumerate(positions, 1):
                for off, v in ((0, 1), (4, bits(q-.01)), (8, bits(q+.01)),
                               (12, bits(.05)), (16, bits(.1)), (20, bits(.1)), (28, 1)):
                    d.write(d.pp_base(mid)+off, v)
            d.configure('normal')
            report['disabled_feedback'] = [refresh(s, mid) for mid in range(1, 8)]
            for mid in axes:
                refresh(s, mid)
                # Include the axis even when readback fails: a write might have arrived.
                changed.append(mid)
                s.write_verified(mid, index, value)
            report['candidate_readback'] = {str(mid): s.read(mid, index) for mid in axes}
            report['status'] = 'PASS_CANDIDATE_READBACK'
        except Exception as exc:
            report.update(status='FAIL', error=repr(exc))
        finally:
            if changed and not (leave and report['status'] == 'PASS_CANDIDATE_READBACK'):
                try:
                    for mid in changed:
                        refresh(s, mid)
                        s.write_verified(mid, index, report['originals'][str(mid)])
                    report['restored'] = {str(mid): s.read(mid, index) for mid in changed}
                    if report['status'].startswith('PASS'):
                        report['status'] = 'PASS_WRITE_READ_RESTORE_DISABLED'
                except Exception as exc:
                    report.update(status='FAIL_RESTORE_REQUIRED', restore_error=repr(exc))
            if owned:
                d.stop()
                d.write(0x5a0, 0)
                for mid in range(1, 8):
                    d.write(d.pp_base(mid), 0)
            report['after'] = d.status()
            report['final_active_mask'] = d.read(0x5a8)
            d.close()
            (out/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('parameter', choices=PARAMETERS)
    ap.add_argument('value', type=float)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--run', action='store_true')
    ap.add_argument('--leave-candidate', action='store_true')
    a = ap.parse_args()
    check_candidate(a.parameter, a.value)
    if not a.run:
        print('OFFLINE_VALID_ARGUMENTS; no device opened')
        return
    r = commission(a.parameter, a.value, a.out, a.leave_candidate)
    print(json.dumps({k: v for k, v in r.items() if k not in ('commands', 'frames', 'feedback')}, indent=2))
    raise SystemExit(not r['status'].startswith('PASS'))


if __name__ == '__main__':
    main()
