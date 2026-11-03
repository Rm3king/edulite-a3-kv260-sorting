#!/usr/bin/python3
"""Six-axis calibration commissioning. Default: offline plan/validation only.

The seventh polynomial channel is a format placeholder, never a gripper target.
No hardware is opened on import or by the default CLI. This entry is deliberately
limited to one small wrist excursion and a return; it is not a hand-eye solver.
"""
import argparse
import gc
import fcntl
import json
import math
import signal
import time
from pathlib import Path
from contextlib import contextmanager

from joint_trajectory import JointPlanner
from live_motion import current_reference
from motion_executor import MoveJExecutor, PathVelocityMonitor, job_digest
from arm_commission import Session, parameter
from a3_protocol import enable, disable
from canctl import CanDevice
from csp_commission import bits, SettlingWindow
from control_continuity import retain_until_supported
from septic_job import SCHEMA, verify_septic_job
from septic_trajectory import movej_c3
from septic_transport import SepticTransport

AXES = tuple(range(1, 7))
MASK = 0x3f


@contextmanager
def motion_gc_scope():
    """Match the released executor: collect before ownership, not during motion.

    This removes one possible host pause; it does not certify Linux scheduling
    latency or establish the cause of the observed PL lease timeout.
    """
    was_enabled = gc.isenabled()
    gc.collect()
    gc.disable()
    try:
        yield
    finally:
        if was_enabled:
            gc.enable()


class ArmOnlyDevice:
    """Reject gripper controls before the driver's MMIO enqueue side effects."""
    def __init__(self, device):
        self.device = device
        self.last_heartbeat = None
        self.max_heartbeat_gap_s = 0.
        self.heartbeat_gaps = []

    def heartbeat(self):
        now = time.monotonic()
        if self.last_heartbeat is not None:
            gap = now-self.last_heartbeat
            self.max_heartbeat_gap_s = max(self.max_heartbeat_gap_s, gap)
            if gap > .02 and len(self.heartbeat_gaps) < 32:
                self.heartbeat_gaps.append(dict(time=now, gap_s=gap))
        self.last_heartbeat = now
        return self.device.heartbeat()

    def __getattr__(self, name):
        return getattr(self.device, name)

    def enqueue(self, identifier, payload, *args, **kwargs):
        mid, kind = identifier & 255, identifier >> 24
        if mid == 7:
            # Only MECH_POS queries are required for before/after evidence.
            if kind != 17 or bytes(payload) != bytes.fromhex('1970000000000000'):
                raise ValueError('gripper control forbidden during board calibration')
        elif mid not in AXES or kind not in (3, 4, 17, 18):
            raise ValueError('unsupported calibration CAN command')
        if kind == 18:
            index = int.from_bytes(bytes(payload)[:2], 'little')
            if index not in (0x7005, 0x7017, 0x7018, 0x7028, 0x7016):
                raise ValueError('parameter outside fixed CSP commissioning set')
        return self.device.enqueue(identifier, payload, *args, **kwargs)

    def write(self, address, value):
        if address == 0x5a0 and value not in (0, MASK):
            raise ValueError('calibration requires exactly six arm permits')
        if 0x680 <= address < 0x6a0 or 0xb40 <= address < 0xd00:
            raise ValueError('gripper bank / seven-axis MIT writes forbidden')
        return self.device.write(address, value)


def make_job(capture, p, degrees=3., wrist_delta_deg=None, expanded_wrist=False, lift_delta_deg=None):
    if not math.isfinite(degrees) or not 0 < abs(degrees) <= 3:
        raise ValueError('first wrist excursion must be nonzero and at most 3 degrees')
    if capture.get('status') != 'PASS_READ_ONLY_ALL_7':
        raise ValueError('successful read-only capture required')
    by = {mid: [x['decoded']['position_motor_rad'] for x in capture['transactions']
                if x['motor'] == mid and x.get('decoded', {}).get('valid')] for mid in range(1, 8)}
    if any(len(v) != 3 or not all(math.isfinite(x) for x in v) or max(v)-min(v) > .002 for v in by.values()):
        raise ValueError('three stable actual readings per motor required')
    actual = [by[mid][-1] for mid in range(1, 8)]
    # Keep the actual gripper reading unmodified in evidence, entirely outside
    # the seven-column planner. It may lie outside the old jaw endpoint profile.
    placeholder = p.offset[6]
    start, adjustment = current_reference(p, actual[:6] + [placeholder])
    start[6] = 0.
    lifted = list(start)
    if lift_delta_deg is not None:
        if len(lift_delta_deg)!=2 or not all(math.isfinite(x) for x in lift_delta_deg) or not (10<=lift_delta_deg[0]<=25 and -35<=lift_delta_deg[1]<=-20):
            raise ValueError('reviewed shoulder/elbow lift range required')
        lifted[1]+=math.radians(lift_delta_deg[0]);lifted[2]+=math.radians(lift_delta_deg[1])
        expanded_wrist=True
    goal = list(lifted)
    if wrist_delta_deg is None:
        goal[5] += math.radians(degrees)
    else:
        limit = 30 if expanded_wrist else 6
        if len(wrist_delta_deg) != 3 or any(not math.isfinite(x) or abs(x) > limit for x in wrist_delta_deg) or (not any(wrist_delta_deg) and lift_delta_deg is None):
            raise ValueError('three finite wrist offsets within reviewed envelope required')
        for i, delta in enumerate(wrist_delta_deg, 3):
            goal[i] += math.radians(delta)
    p.positions(goal)
    duration = 6. if expanded_wrist else 4.
    tasks = [dict(name='wrist_out', segments=movej_c3(p, start, goal, duration), goal_rad=goal),
             dict(name='return_to_supported_start', segments=movej_c3(p, goal, start, duration),
                  goal_rad=start, return_to_live_start=True)]
    waypoints = [start, goal]
    if lift_delta_deg is not None:
        elbow = list(start);elbow[2]=lifted[2]
        # Open the elbow upward first; restore wrist before lowering shoulder/elbow.
        waypoints=[start,elbow,lifted,goal]
        outward=[];returning=[]
        for a,b in zip(waypoints,waypoints[1:]): outward.extend(movej_c3(p,a,b,6.))
        for a,b in zip(waypoints[::-1],waypoints[-2::-1]): returning.extend(movej_c3(p,a,b,6.))
        tasks[0]['segments']=outward;tasks[1]['segments']=returning
    a, b = p.to_motor(start), p.to_motor(goal)
    lower, upper = [], []
    for i in range(7):
        lo, hi = sorted((p.lower[i]*p.direction[i]+p.offset[i], p.upper[i]*p.direction[i]+p.offset[i]))
        values=[p.to_motor(q)[i] for q in waypoints]
        lower.append(max(lo, min(values)-.02))
        upper.append(min(hi, max(values)+.02))
    job = dict(schema=SCHEMA, calibration_policy='six_axis_wrist_probe_v1', profile_id=p.profile_id,
               motion_mask=MASK, start_rad=start, start_motor_rad=actual[:6],
               observed_gripper_motor_rad=actual[6], unused_channel_7_rad=0.,
               start_reference_adjustment_motor_rad=adjustment[:6],
               motor_lower_rad=lower, motor_upper_rad=upper, tasks=tasks,
               hardware_start_verified=not capture.get('synthetic_for_planning_only', False))
    if wrist_delta_deg is not None:
        job.update(calibration_policy='six_axis_wrist_expanded_v1' if expanded_wrist else 'six_axis_wrist_sample_v1', sample_at_out=True)
    if lift_delta_deg is not None:
        job.update(calibration_policy='six_axis_lifted_sample_v1',sample_at_out=True,lift_delta_deg=lift_delta_deg,lift_waypoints_rad=waypoints)
    verify_job(job, p)
    return job


def verify_job(job, p):
    lifted = job.get('calibration_policy') == 'six_axis_lifted_sample_v1'
    expanded = lifted or job.get('calibration_policy') == 'six_axis_wrist_expanded_v1'
    sampling = expanded or job.get('calibration_policy') == 'six_axis_wrist_sample_v1'
    if job.get('calibration_policy') not in ('six_axis_wrist_probe_v1', 'six_axis_wrist_sample_v1', 'six_axis_wrist_expanded_v1', 'six_axis_lifted_sample_v1') or job.get('motion_mask') != MASK:
        raise ValueError('six-axis calibration policy/mask required')
    if len(job['tasks']) != 2 or len(job['start_motor_rad']) != 6:
        raise ValueError('one excursion and one return required')
    start = p.positions(job['start_rad'])
    if start[6] != 0 or job.get('unused_channel_7_rad') != 0:
        raise ValueError('seventh channel must be an unused zero placeholder')
    out, back = job['tasks']
    if back['goal_rad'] != start or not back.get('return_to_live_start'):
        raise ValueError('return must end at the reviewed start')
    deltas = [out['goal_rad'][i]-start[i] for i in range(3, 6)]
    if (not lifted and not any(abs(x) > 1e-12 for x in deltas)) or any(abs(x) > math.radians(30 if expanded else (6 if sampling else 3))+1e-12 for x in deltas):
        raise ValueError('wrist excursion exceeds reviewed envelope')
    if lifted:
        w=job['lift_waypoints_rad'];d=job['lift_delta_deg']
        if len(w)!=4 or w[0]!=start or w[-1]!=out['goal_rad'] or not (10<=d[0]<=25 and -35<=d[1]<=-20):
            raise ValueError('lift route mismatch')
        elbow=list(start);elbow[2]+=math.radians(d[1])
        shoulder=list(elbow);shoulder[1]+=math.radians(d[0])
        if w[1]!=elbow or w[2]!=shoulder or w[3][:3]!=shoulder[:3] or w[3][6]!=0 or deltas[0]>1e-12:
            raise ValueError('must lift elbow first and avoid positive wrist4 interference')
        for task,route in ((out,w),(back,w[::-1])):
            expected=[]
            for a,b in zip(route,route[1:]): expected.extend(movej_c3(p,a,b,6.))
            if task['segments']!=expected: raise ValueError('unreviewed lifted route coefficients')
    if sampling and job.get('sample_at_out') is not True:
        raise ValueError('sampling requires the image/feedback handshake')
    for task in job['tasks']:
        if sum(s['duration_s'] for s in task['segments']) < (6 if expanded else 4):
            raise ValueError('each leg must last at least four seconds')
        for segment in task['segments']:
            for i in ((0,6) if lifted else ((0, 1, 2, 6) if sampling else (0, 1, 2, 3, 4, 6))):
                row = segment['coefficients_rad'][i]
                if abs(row[0]-start[i]) > 1e-12 or any(abs(x) > 1e-12 for x in row[1:]):
                    raise ValueError('unreviewed axis motion')
    # Validate quantized trajectories as well as endpoint metadata before I/O.
    prepared = verify_septic_job(job, p, return_prepared=True)
    for i in range(6):
        legal = sorted((p.lower[i]*p.direction[i]+p.offset[i], p.upper[i]*p.direction[i]+p.offset[i]))
        lo, hi = job['motor_lower_rad'][i], job['motor_upper_rad'][i]
        if not legal[0] <= lo < hi <= legal[1]:
            raise ValueError('guard interval outside commissioned limits')
        for record in prepared:
            from septic_trajectory import extrema
            low, high = extrema(record['words'][8*i:8*i+8])
            if low/(1 << 24) < lo or high/(1 << 24) > hi:
                raise ValueError('encoded path outside selected guard interval')
    return prepared


def capture_while_holding(e, job, out, before):
    """Bracket a host camera capture with actual stopped joint feedback."""
    # Hash was computed before hardware access; avoid reserializing the whole
    # trajectory while servicing a 100 ms controller lease.
    ready = dict(job_sha256=e.report['job_sha256'], endpoint=before, timeout_s=15.)
    (out/'capture_ready.tmp').write_text(json.dumps(ready))
    (out/'capture_ready.tmp').replace(out/'capture_ready.json')
    e.event('READY_FOR_CAMERA_CAPTURE', job_sha256=ready['job_sha256'])
    deadline = time.monotonic()+15.
    while time.monotonic() < deadline:
        e.pump(.003)
        e.diagnostics()
        path = out/'capture_ack.json'
        if path.is_file():
            ack = json.loads(path.read_text())
            if ack.get('job_sha256') != ready['job_sha256'] or ack.get('status') != 'CAPTURED':
                raise RuntimeError('camera capture rejected or handshake mismatched')
            after = e.settle(job['tasks'][0]['goal_rad'])
            drift = max(abs(a-b) for a, b in zip(before['actual_motor_rad'], after['actual_motor_rad']))
            if drift > .001:
                raise RuntimeError('joint feedback moved during camera exposure interval')
            return dict(before=before, after=after, max_bracket_drift_rad=drift, camera_ack=ack)
    raise TimeoutError('camera capture handshake timeout; finish reviewed return')


class ArmOnlySession(Session):
    def stop_all(self):
        # Only called after verified supported return or explicit support marker.
        self.stopping = True
        self.dev.configure('normal')
        sent = {}
        for mid in reversed(AXES):
            sent[mid] = time.monotonic()
            self.send(disable(mid))
        self.pump(.06)
        for mid in reversed(AXES):
            f = self.feedback.get(mid, {})
            if f.get('time', -1) < sent[mid] or f.get('mode') != 0:
                sent[mid] = time.monotonic()
                self.send(disable(mid))
                self.pump(.06)
                f = self.feedback.get(mid, {})
            if f.get('time', -1) >= sent[mid] and f.get('mode') == 0 and not f.get('fault'):
                self.report['reset_confirmed'].append(mid)
            else:
                self.report['stop_errors'][str(mid)] = 'no fresh fault-free disabled feedback'
        self.dev.stop()
        self.dev.write(0x5a0, 0)
        for mid in AXES:
            self.dev.write(self.dev.pp_base(mid), 0)


class CalibrationExecutor(MoveJExecutor):
    def __init__(self, p, dev, report, prepared):
        super().__init__(p, dev, report, SepticTransport, prepared)
        self.s = ArmOnlySession(dev, report)
        self.s.velocity_monitor = PathVelocityMonitor(p, report)

    def refresh_disabled_feedback(self, mid):
        stamp = time.monotonic()
        self.s.send(parameter(mid, 0x7028, 6000, True))
        deadline = stamp+.025
        while time.monotonic() < deadline:
            self.s.pump(.002)
            feedback = self.s.feedback.get(mid)
            if feedback and feedback['time'] >= stamp:
                self.s.require(mid, 0)
                return
        raise TimeoutError(f'Axis {mid}: no new disabled feedback after refresh')

    def pump(self, seconds=.003):
        status = super().pump(seconds)
        if self.d.read(0x5a0) != MASK or self.d.read(0x5a8) != MASK:
            raise RuntimeError('six-axis permission/active mask changed')
        if self.started and status['mask'] != MASK:
            raise RuntimeError('PL stream did not retain the six-axis mask')
        return status

    def prepare(self, job):
        p, d, s = self.p, self.d, self.s
        if (d.read(0x470) not in (0x60002,0x60003) or d.read(0x404) or d.read(0x5a0) or
                d.read(0x5a8) or d.read(0x800)&1 or d.read(0xb40)&1):
            raise RuntimeError('stopped released controller with no active owner required')
        d.configure('normal')
        self.report['owned'] = True
        for mid in AXES:
            s.send(disable(mid))
            s.pump(.015)
            s.require(mid, 0)
        actual = [s.read(mid, 0x7019) for mid in AXES]
        self.report['gripper_before_motor_rad'] = s.read(7, 0x7019)
        if abs(self.report['gripper_before_motor_rad']-job['observed_gripper_motor_rad']) > .002:
            raise RuntimeError('gripper position changed since mounting review')
        if max(abs(a-b) for a, b in zip(actual, job['start_motor_rad'])) > .002:
            raise RuntimeError('arm moved since review: recapture/replan while disabled')
        d.stop()
        d.write(0x5a0, MASK)
        d.write(0x5a4, 100000)
        for mid in AXES:
            i = mid-1
            lo, hi = job['motor_lower_rad'][i], job['motor_upper_rad'][i]
            s.plans[mid] = dict(lower=lo-.02, upper=hi+.02)
            for off, val in [(0, 1), (4, bits(lo)), (8, bits(hi)), (12, bits(p.csp_velocity[i])),
                             (16, bits(p.acceleration[i])), (20, bits(p.current[i])), (28, 1)]:
                address = d.pp_base(mid)+off
                d.write(address, val)
                if d.read(address) != val:
                    raise RuntimeError('six-axis guard readback mismatch')
        d.configure('normal')
        for mid in AXES:
            for idx, value, integer in [(0x7005, 5, True), (0x7017, p.csp_velocity[mid-1], False),
                                        (0x7018, p.current[mid-1], False), (0x7028, 6000, True)]:
                s.write_verified(mid, idx, value, integer)
        fresh = [s.read(mid, 0x7019) for mid in AXES]
        if max(abs(a-b) for a, b in zip(fresh, actual)) > .002:
            raise RuntimeError('disabled arm moved during preparation')
        self.report['enable_capture_motor_rad'] = fresh
        for mid, value in zip(AXES, p.to_motor(job['start_rad'])[:6]):
            self.refresh_disabled_feedback(mid)
            s.send(parameter(mid, 0x7016, value))
            s.pump(.003)
            s.targets[mid] = value
            if d.read(d.pp_base(mid)+24) != 63:
                raise RuntimeError('six-axis CSP preparation incomplete')
        # Refresh all disabled feedback in one batch before grouped enable.
        # No assumption that a reply must arrive within a fixed three ms sleep.
        refresh_stamps = {}
        for mid in AXES:
            refresh_stamps[mid] = time.monotonic()
            s.send(parameter(mid, 0x7028, 6000, True))
        deadline = time.monotonic()+.04
        while True:
            s.pump(.002)
            if all(s.feedback.get(mid, {}).get('time', -1) >= refresh_stamps[mid] for mid in AXES):
                for mid in AXES: s.require(mid, 0)
                break
            if time.monotonic() > deadline:
                raise TimeoutError('missing fresh disabled feedback before grouped enable')
        self.report['enable_attempted'] = True
        stamps = {}
        for mid in AXES:
            stamps[mid] = time.monotonic()
            s.send(enable(mid))
            s.send(parameter(mid, 0x7016, s.targets[mid]))
        s.pump(.02)
        for mid in AXES:
            if s.require(mid, 2)['time'] < stamps[mid]:
                raise RuntimeError('missing fresh six-axis enable acknowledgement')
        s.active.update(AXES)
        s.pump(.1)
        if d.read(0x5a8) != MASK:
            raise RuntimeError('six-axis active mask mismatch')
        self.state = 'HOLDING'
        self.event('SIX_AXES_HOLDING_GRIPPER_EXCLUDED')
        # Host must keep refreshing targets during the initial settling window.
        # PL cannot own this hold until its first segment has been started.
        self.report['stream_initial_counters'] = dict(rx_overflow=d.read(0x42c), expired=d.read(0x424))

    def settle(self, goal, seconds=.3, tolerance=.004):
        motor = self.p.to_motor(goal)[:6]
        window = SettlingWindow(tolerance, seconds)
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            self.pump(.01)
            errors = [abs(self.s.require(mid, 2)['position']-motor[mid-1]) for mid in AXES]
            if window.observe(time.monotonic(), errors):
                actual = [self.s.read(mid, 0x7019) for mid in AXES]
                if max(abs(a-b) for a, b in zip(actual, motor)) > tolerance:
                    raise RuntimeError('six-axis Type17 endpoint residual')
                return dict(actual_motor_rad=actual, feedback_errors_rad=errors, time=time.monotonic())
        raise RuntimeError('six-axis endpoint did not settle')


def recover_or_retain(e, job, out, report):
    try:
        if e.d.read(0x418) or e.transport.status()['fault'] or e.d.read(0x5a8) != MASK:
            raise RuntimeError('six-axis controller cannot finish cached return')
        if not e.started:
            # Preparation failed before PL was started: no automatic reset.
            raise RuntimeError('no running prevalidated return')
        for mid in AXES:
            e.s.require(mid, 2)
        remaining = e.transport.prepared[e.transport.prepared_index:]
        e.state = 'HOLDING'
        if remaining:
            e.execute_segments([r['plan'] for r in remaining])
        else:
            deadline = time.monotonic()+15
            while not e.pump(.003)['holding']:
                if time.monotonic() > deadline:
                    raise TimeoutError('cached six-axis return deadline')
        report['recovery_endpoint'] = e.settle(job['start_rad'])
        report['completed_cached_return_before_disable'] = True
    except (Exception, KeyboardInterrupt) as exc:
        report['cached_return_error'] = repr(exc)
        retain_until_supported(e, out, report)
    report['safe_to_disable_after_return_or_support'] = True


def execute(job, p, out, *, job_verifier=None):
    prepared = (verify_job if job_verifier is None else job_verifier)(job, p)
    if not job.get('hardware_start_verified'):
        raise ValueError('synthetic capture cannot execute')
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    report = dict(status='RUNNING', commands=[], frames=[], feedback=[], readbacks={},
                  reset_confirmed=[], stop_errors={}, job_sha256=job_digest(job), motion_mask=MASK,
                  calibration_complete=False, flash_writes=0, zero_writes=0)
    (out/'job.json').write_text(json.dumps(job, indent=2)+'\n')
    with motion_gc_scope(), open('/tmp/kv260_gripper_commission.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        d = ArmOnlyDevice(CanDevice())
        e = CalibrationExecutor(p, d, report, prepared)
        try:
            e.prepare(job)
            report['initial_hold_endpoint'] = e.settle(job['start_rad'])
            e.release_host_hold()
            for task in job['tasks']:
                e.event('CALIBRATION_PROBE_TASK', name=task['name'])
                e.execute_segments(task['segments'])
                result = e.settle(task['goal_rad'])
                report.setdefault('endpoints', []).append(dict(name=task['name'], **result))
                if job.get('sample_at_out') and task is job['tasks'][0]:
                    report['paired_capture'] = capture_while_holding(e, job, out, result)
            report['safe_to_disable_after_return_or_support'] = True
            report['gripper_after_motor_rad'] = e.s.read(7, 0x7019)
            report['gripper_drift_motor_rad'] = report['gripper_after_motor_rad']-report['gripper_before_motor_rad']
            if abs(report['gripper_drift_motor_rad']) > .002:
                raise RuntimeError('uncommanded gripper drift: mounting/retention must be checked')
            report['status'] = 'PASS_SIX_AXIS_WRIST_RETURN_ONLY'
        except (Exception, KeyboardInterrupt) as exc:
            report.update(status='FAIL', error=repr(exc))
            e.event('CALIBRATION_PROBE_FAILED', error=repr(exc))
            if report.get('enable_attempted'):
                recover_or_retain(e, job, out, report)
        finally:
            if report.get('owned'):
                if report.get('enable_attempted'):
                    # Both success and recovery require verified return/support.
                    if not report.get('safe_to_disable_after_return_or_support'):
                        retain_until_supported(e, out, report)
                    try:
                        e.s.stop_all()
                    except (Exception, KeyboardInterrupt) as exc:
                        report['stop_errors']['fatal'] = repr(exc)
                        retain_until_supported(e, out, report)
                    if sorted(report['reset_confirmed']) != list(AXES) or report['stop_errors']:
                        report['status'] = 'FAIL_DISABLE_CONFIRMATION'
                    elif report['status'] == 'RECOVERY_FAILED_RETAINING_CONTROL':
                        report['status'] = 'FAIL_SUPPORTED_DISABLED'
                else:
                    d.stop()
                    d.write(0x5a0, 0)
                    for mid in AXES:
                        d.write(d.pp_base(mid), 0)
            report['final_controller'] = d.status()
            report['final_motion_mask'] = d.read(0x5a0)
            report['gripper_control_commands'] = sum(c['id'] & 255 == 7 and c['id'] >> 24 != 17 for c in report['commands'])
            report['host_heartbeat_max_gap_s'] = d.max_heartbeat_gap_s
            report['host_heartbeat_gaps_over_20ms'] = d.heartbeat_gaps
            (out/'result.json').write_text(json.dumps(report, indent=2)+'\n')
            d.close()
    return report


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--profile', required=True)
    source = ap.add_mutually_exclusive_group(required=True)
    source.add_argument('--capture')
    source.add_argument('--job')
    ap.add_argument('--out', required=True)
    ap.add_argument('--execute-attended', action='store_true')
    ap.add_argument('--wrist-delta-deg', nargs=3, type=float)
    a = ap.parse_args()
    p = JointPlanner(a.profile)
    if a.capture:
        if a.execute_attended:
            ap.error('review a saved --job before execution')
        job = make_job(json.loads(Path(a.capture).read_text()), p, wrist_delta_deg=a.wrist_delta_deg)
    else:
        job = json.loads(Path(a.job).read_text())
    prepared = verify_job(job, p)
    if a.execute_attended:
        def interrupt(*args):
            raise KeyboardInterrupt('operator interruption: retain or complete supported return')
        signal.signal(signal.SIGTERM, interrupt)
        result = execute(job, p, a.out)
        print(json.dumps({k: result.get(k) for k in ('status', 'error', 'gripper_control_commands', 'reset_confirmed')}))
        return 0 if result['status'].startswith('PASS_') else 1
    target = Path(a.out)
    with target.open('x') as stream:
        json.dump(job, stream, indent=2)
        stream.write('\n')
    print(json.dumps(dict(status='OFFLINE_VALIDATED_NO_HARDWARE', segments=len(prepared), motion_mask=MASK,
                          job_sha256=job_digest(job), nominal_seconds=sum(s['duration_s'] for t in job['tasks'] for s in t['segments']))))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
