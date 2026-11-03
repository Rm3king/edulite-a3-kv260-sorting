"""Attended empty-jaw CSP comparison, fixed elevated route, supported return.

No tuning or hardware access on import; gains are commissioned separately while
disabled. No object approach, closing, automatic retry, or protection removal.
"""
import argparse
import fcntl
import gzip
import json
import math
import signal
from pathlib import Path
from calibration_motion import motion_gc_scope
from continuous_transfer import TransferDevice, path_digest
from visual_pick_trial import GraspExecutor
from joint_trajectory import JointPlanner, Q24
from septic_job import verify_septic_job
from septic_trajectory import extrema
from motion_executor import CanDevice, job_digest
from control_continuity import retain_until_supported

POLICY = 'attended_empty_csp_abba_v1'
NAMES = ['departure', 'transfer_empty', 'transfer_empty_reverse', 'return_supported']


def verify(job, p):
    if job.get('policy') != POLICY or job.get('startup_no_reset') is not True or job.get('motion_mask') != 127:
        raise ValueError('Explicit empty-jaw comparison policy required')
    if job.get('targets') or [t['name'] for t in job['tasks']] != NAMES:
        raise ValueError('Fixed empty route required')
    if any(v > hi for v, hi in zip(p.velocity, [.575, .345, .345, .575, .575, .575, .25])):
        raise ValueError('Comparison cannot increase commissioned path speeds')
    if any(v > hi for v, hi in zip(p.current, [1.5, 5.5, 4.5, 2.5, 1., 1., .65])):
        raise ValueError('Comparison cannot increase current ceilings')
    if max(abs(a-b) for a, b in zip(job['tasks'][-1]['goal_rad'], job['start_rad'])) > 1e-9:
        raise ValueError('Exact supported return required')
    audit = job['geometry_audit']
    if (audit.get('status') != 'PASS_SAMPLED_WITH_MONOTONIC_EXISTING_START_CONTACT_DEPARTURE'
            or audit.get('unexpected') or audit['trajectory_sha256'] != path_digest(job)):
        raise ValueError('Quantized swept geometry must match this route')
    if not .08 <= job.get('minimum_transfer_tcp_clearance_m', 0) <= .5:
        raise ValueError('Measured table clearance required for elevated transfer')
    prepared = verify_septic_job(job, p, return_prepared=True)
    if not 0 < sum(r['plan']['duration_s'] for r in prepared) <= 60:
        raise ValueError('Bounded comparison duration required')
    for i in range(7):
        lo, hi = job['motor_lower_rad'][i], job['motor_upper_rad'][i]
        legal = sorted((p.lower[i]*p.direction[i]+p.offset[i], p.upper[i]*p.direction[i]+p.offset[i]))
        if not legal[0] <= lo < hi <= legal[1]:
            raise ValueError('Invalid guard interval')
        for r in prepared:
            a, b = extrema(r['words'][8*i:8*i+8])
            if a/Q24 < lo or b/Q24 > hi:
                raise ValueError('Encoded path outside guard')
            if i == 6 and max(abs(a/Q24-1.12), abs(b/Q24-1.12)) > 1e-6:
                raise ValueError('Gripper must stay open throughout comparison')
    return prepared


def execute(job, p, out):
    prepared = verify(job, p)
    if not job.get('hardware_start_verified'):
        raise ValueError('Fresh supported physical start required')
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    report = dict(status='RUNNING', commands=[], frames=[], feedback=[], readbacks={},
                  reset_confirmed=[], stop_errors={}, job_sha256=job_digest(job),
                  csp_experiment=job['experiment'], flash_writes=0, zero_writes=0)
    (out/'job.json').write_text(json.dumps(job, indent=2)+'\n')
    supported = False
    with motion_gc_scope(), open('/tmp/kv260_gripper_commission.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        d = TransferDevice(CanDevice())
        e = GraspExecutor(p, d, report, prepared)
        try:
            if d.read(0x470) != 0x60003 or any(d.read(a) for a in (0x404, 0x418, 0x5a0, 0x5a8, 0x800, 0xb40)):
                raise RuntimeError('Idle ABI6.3 controller required')
            d.configure('normal')
            try:
                report['parameters_before'] = {str(mid): {hex(idx): e.s.read(mid, idx)
                    for idx in (0x701e, 0x701f, 0x7020, 0x7021)} for mid in (2, 3)}
            finally:
                d.stop()
            for mid, values in job['experiment']['expected_parameters'].items():
                for index, value in values.items():
                    if not math.isclose(report['parameters_before'][mid][index], value, abs_tol=1e-6):
                        raise RuntimeError('Parameter differs from reviewed experiment')
            e.prepare(job)
            if max(e.s.require(mid, 2)['temperature'] for mid in range(1, 8)) >= 65:
                raise RuntimeError('Start temperature too high for comparison')
            e.settle_arm(job['start_rad'], seconds=.15)
            supported = True
            report['parameters_after_prepare'] = {}
            for mid, values in job['experiment']['expected_parameters'].items():
                report['parameters_after_prepare'][mid] = {}
                for index, value in values.items():
                    observed = e.s.read(int(mid), int(index, 16))
                    report['parameters_after_prepare'][mid][index] = observed
                    if not math.isclose(observed, value, abs_tol=1e-6):
                        raise RuntimeError('CSP preparation changed experimental parameter')
            e.release_host_hold()
            supported = False
            for task in job['tasks']:
                report['active_task'] = task['name']
                e.event('TASK', name=task['name'])
                e.execute_segments(task['segments'])
            report['return_endpoint'] = e.settle_arm(job['start_rad'], seconds=.15)
            supported = True
            report['status'] = 'PASS_EMPTY_TRANSFER_RETURN'
            e.event('VERIFIED_START_RETURN', status=report['status'])
        except (Exception, KeyboardInterrupt) as exc:
            report.update(status='FAIL', error=repr(exc))
            e.event('CSP_COMPARISON_FAILED', error=repr(exc))
            if report.get('enable_attempted') and not supported:
                # Complete only the already-validated elevated route when the
                # controller and all feedback still permit that cached return.
                try:
                    if not e.started or d.read(0x418) or e.transport.status()['fault'] or d.read(0x5a8) != 127:
                        raise RuntimeError('Cached return unavailable')
                    for mid in range(1, 8):
                        e.s.require(mid, 2)
                    remaining = e.transport.prepared[e.transport.prepared_index:]
                    e.state = 'HOLDING'
                    if remaining:
                        e.execute_segments([r['plan'] for r in remaining])
                    report['return_endpoint'] = e.settle_arm(job['start_rad'], seconds=.15)
                    supported = True
                    report['completed_cached_return'] = True
                except (Exception, KeyboardInterrupt) as recovery:
                    report['cached_return_error'] = repr(recovery)
                    retain_until_supported(e, out, report)
                    supported = True
        finally:
            if report.get('owned'):
                if report.get('enable_attempted'):
                    if not supported:
                        retain_until_supported(e, out, report)
                    # User authorizes disable after verified supported return.
                    try:
                        e.s.stop_all()
                    except (Exception, KeyboardInterrupt) as exc:
                        report['stop_errors']['fatal'] = repr(exc)
                        retain_until_supported(e, out, report)
                    if sorted(report['reset_confirmed']) != list(range(1, 8)) or report['stop_errors']:
                        report['status'] = 'FAIL_DISABLE_CONFIRMATION'
                else:
                    d.stop()
                    d.write(0x5a0, 0)
                    for mid in range(1, 8):
                        d.write(d.pp_base(mid), 0)
            report['final_controller'] = d.status()
            report['final_active_mask'] = d.read(0x5a8)
            d.close()
            (out/'result.json.gz').write_bytes(gzip.compress(json.dumps(report).encode(), compresslevel=1))
            summary = {k: report.get(k) for k in ('status', 'error', 'cached_return_error', 'reset_confirmed',
                        'stop_errors', 'final_controller', 'final_active_mask', 'parameters_before')}
            (out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
            print(json.dumps(summary), flush=True)
    return report


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    for key in ('job', 'profile', 'out'):
        ap.add_argument('--'+key, type=Path, required=True)
    ap.add_argument('--execute-attended', action='store_true')
    a = ap.parse_args()
    job = json.loads(a.job.read_text())
    p = JointPlanner(a.profile)
    records = verify(job, p)
    print(json.dumps(dict(status='OFFLINE_PATH_VALID', segments=len(records))), flush=True)
    if a.execute_attended:
        def interrupt(*args):
            raise KeyboardInterrupt('Retain until supported')
        signal.signal(signal.SIGTERM, interrupt)
        r = execute(job, p, a.out)
        raise SystemExit(r['status'] != 'PASS_EMPTY_TRANSFER_RETURN')


if __name__ == '__main__':
    main()
