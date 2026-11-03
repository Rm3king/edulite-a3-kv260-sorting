"""Attended wrist-camera pose collection, fixed measured jaw, explicit disable.

Default CLI validates offline. Camera I/O runs in a prestarted worker; the
hardware owner continues servicing the controller. Images never command motion.
"""
import argparse
import fcntl
import gzip
import hashlib
import json
import queue
import signal
import threading
import time
from pathlib import Path
from urllib.request import urlopen

from calibration_motion import motion_gc_scope
from continuous_transfer import TransferDevice, path_digest
from control_continuity import WaitStateWriter, await_disable_approval, retain_until_supported
from joint_trajectory import JointPlanner, Q24
from motion_executor import job_digest
from septic_job import verify_septic_job
from septic_trajectory import extrema
from visual_pick_trial import GraspExecutor
from canctl import CanDevice

POLICY = 'attended_fixed_jaw_wrist_handeye_v1'


def verify(job, planner):
    if job.get('policy') != POLICY or job.get('startup_no_reset') is not True or job.get('motion_mask') != 127:
        raise ValueError('Explicit attended fixed-jaw policy required')
    jaw = job['start_rad'][6]
    if not -.006 <= jaw <= 1.70 or abs(planner.to_motor(job['start_rad'])[6]-job['start_motor_rad'][6]) > 1e-7:
        raise ValueError('Measured jaw must remain fixed')
    if planner.current[6] > .300001 or any(v > .350001 for v in planner.velocity[:6]):
        raise ValueError('Calibration current/speed envelope')
    if not 2 <= len(job['tasks']) <= 24 or job['tasks'][-1]['goal_rad'] != job['start_rad'] or not job['tasks'][-1].get('return_to_live_start'):
        raise ValueError('Bounded sequence and return to observed start required')
    audit = job['geometry_audit']
    if audit.get('status') != 'PASS' or audit.get('unexpected') or audit.get('trajectory_sha256') != path_digest(job):
        raise ValueError('Exact trajectory geometry review required')
    if not audit.get('wrist_camera_proxy_checked'):
        raise ValueError('New camera envelope must be reviewed')
    sequences = [job['tasks']] + [[r] for r in job['recoveries'].values()]
    for tasks in sequences:
        for task in tasks:
            for segment in task['segments']:
                row = segment['coefficients_rad'][6]
                if abs(row[0]-jaw) > 1e-9 or any(abs(v) > 1e-12 for v in row[1:]):
                    raise ValueError('Jaw motion forbidden')
    records = verify_septic_job(job, planner, return_prepared=True)
    returns = {}
    for task in job['tasks'][:-1]:
        branch = job['recoveries'][task['name']]
        if branch['start_rad'] != task['goal_rad'] or branch['goal_rad'] != job['start_rad']:
            raise ValueError('Return branch must start at captured endpoint')
        returns[task['name']] = verify_septic_job(dict(schema=job['schema'], profile_id=job['profile_id'],
            start_rad=branch['start_rad'], tasks=[branch]), planner, return_prepared=True)
    for group in [records] + list(returns.values()):
        for record in group:
            for i in range(7):
                lo, hi = extrema(record['words'][8*i:8*i+8])
                legal = sorted((planner.lower[i]*planner.direction[i]+planner.offset[i],
                                planner.upper[i]*planner.direction[i]+planner.offset[i]))
                lower, upper = job['motor_lower_rad'][i], job['motor_upper_rad'][i]
                if not legal[0] <= lower < upper <= legal[1] or lo/Q24 < lower or hi/Q24 > upper:
                    raise ValueError('Encoded trajectory outside reviewed bounds')
    return records, returns


class CameraWorker:
    def __init__(self, out):
        self.out = Path(out)
        self.requests = queue.Queue()
        self.results = queue.Queue()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while True:
            request = self.requests.get()
            if request is None:
                return
            try:
                folder = self.out/request['name']
                folder.mkdir(exist_ok=False)
                frames = []
                seen = set()
                camera_session = None
                deadline = time.monotonic()+2.
                while len(frames) < 3 and time.monotonic() < deadline:
                    with urlopen('http://127.0.0.1:8766/frame.jpg', timeout=.35) as response:
                        raw = response.read()
                        headers = dict(response.headers)
                    stamp = int(headers['X-Board-Received-Monotonic-Ns'])
                    sequence = int(headers['X-Frame-Sequence'])
                    session = headers['X-Camera-Session']
                    if camera_session is not None and session != camera_session:
                        raise RuntimeError('Camera restarted within stationary burst')
                    camera_session = session
                    if stamp <= request['after_ns'] or sequence in seen:
                        time.sleep(.025)
                        continue
                    if not 0 <= (time.monotonic_ns()-stamp)*1e-9 < .5:
                        raise RuntimeError('Camera frame stale')
                    seen.add(sequence)
                    filename = f'frame_{len(frames)}.jpg'
                    (folder/filename).write_bytes(raw)
                    frames.append(dict(file=filename, headers=headers, sha256=hashlib.sha256(raw).hexdigest()))
                    if len(frames) < 3:
                        time.sleep(.01)
                if len(frames) != 3:
                    raise RuntimeError('Three fresh frames not received before deadline')
                self.results.put(dict(name=request['name'], status='CAPTURED', frames=frames))
            except Exception as error:
                self.results.put(dict(name=request['name'], status='FAILED', error=repr(error)))


def stationary_capture(executor, worker, name, before):
    after_ns = time.monotonic_ns()
    worker.requests.put(dict(name=name, after_ns=after_ns))
    deadline = time.monotonic()+3.
    while time.monotonic() < deadline:
        executor.pump(.003)
        executor.diagnostics()
        try:
            result = worker.results.get_nowait()
        except queue.Empty:
            continue
        if result['name'] != name:
            raise RuntimeError('Capture worker request mismatch')
        after = [executor.s.read(i, 0x7019) for i in range(1, 7)]
        result.update(before=before, after_actual_motor_rad=after,
                      bracket_start_monotonic_ns=after_ns, bracket_end_monotonic_ns=time.monotonic_ns(),
                      drift_rad=max(abs(a-b) for a, b in zip(before['actual_motor_rad'], after)))
        if result['drift_rad'] > .002:
            result.update(status='FAILED', error='Joint motion during image interval')
        return result
    return dict(name=name, status='FAILED', error='Bounded capture timeout')


def review_initial_capture(executor, out, result):
    """Bounded host image review; timeout/rejection takes a prepared return."""
    marker = Path(out)/'initial_capture_review.json'
    deadline = time.monotonic()+5.
    while time.monotonic() < deadline:
        executor.pump(.003)
        executor.diagnostics()
        if marker.exists():
            review = json.loads(marker.read_text())
            return (review.get('full_grid_found') is True
                    and review.get('corner_count') == 289
                    and review.get('job_sha256') == result['job_sha256']
                    and review.get('image_sha256') == result['frames'][1]['sha256'])
    return False


def execute(job, planner, out, validated):
    records, returns = validated
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    report = dict(status='RUNNING', commands=[], frames=[], feedback=[], readbacks={}, reset_confirmed=[],
                  stop_errors={}, job_sha256=job_digest(job), flash_writes=0, zero_writes=0)
    (out/'job.json').write_text(json.dumps(job))
    supported = False
    with motion_gc_scope(), open('/tmp/kv260_gripper_commission.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        device = TransferDevice(CanDevice())
        device.marker_jaw_hold = job['start_motor_rad'][6]
        executor = GraspExecutor(planner, device, report, records)
        executor.wait_state_writer = WaitStateWriter(out)
        worker = CameraWorker(out)
        try:
            executor.prepare(job)
            executor.settle_arm(job['start_rad'], seconds=.15)
            executor.release_host_hold()
            for task in job['tasks']:
                name = task['name']
                report['active_task'] = name
                executor.event('HAND_EYE_TASK', name=name)
                executor.execute_segments(task['segments'])
                before = executor.settle_arm(task['goal_rad'], seconds=.2)
                report['last_completed_task'] = name
                if task.get('capture'):
                    result = stationary_capture(executor, worker, name, before)
                    result.update(split=task['split'], job_sha256=report['job_sha256'])
                    # Small metadata only; JPEG I/O stays in the prestarted worker.
                    (out/f'{name}_paired.json').write_text(json.dumps(result))
                    report.setdefault('captures', []).append(result)
                    executor.event('HAND_EYE_CAPTURE', name=name, status=result['status'])
                    if task is job['tasks'][0] and job.get('initial_capture_review') and result['status'] == 'CAPTURED':
                        accepted = review_initial_capture(executor, out, result)
                        executor.event('INITIAL_CAPTURE_REVIEW', accepted=accepted)
                        if not accepted:
                            result.update(status='FAILED', error='Initial full-board framing not accepted')
                    temperatures = [executor.s.require(i, 2)['temperature'] for i in range(1, 8)]
                    if result['status'] != 'CAPTURED' or max(temperatures) >= 70:
                        branch = job['recoveries'][name]
                        executor.transport.prepared = returns[name]
                        executor.transport.prepared_index = 0
                        executor.execute_segments(branch['segments'])
                        report['return_endpoint'] = executor.settle_arm(job['start_rad'])
                        supported = True
                        report['status'] = 'EARLY_RETURN_CAPTURE_OR_TEMPERATURE'
                        break
                else:
                    report['return_endpoint'] = before
                    supported = True
            else:
                report['status'] = 'COMPLETED_HAND_EYE_CAPTURE_RETURN'
        except (Exception, KeyboardInterrupt) as error:
            report.update(status='FAILED', error=repr(error))
            executor.event('HAND_EYE_FAILED', error=repr(error))
            if report.get('enable_attempted'):
                retain_until_supported(executor, out, report)
                supported = True
        finally:
            if report.get('owned'):
                if report.get('enable_attempted'):
                    if not supported:
                        retain_until_supported(executor, out, report)
                    executor.event('HAND_EYE_RETURN_WAIT_DISABLE', status=report['status'])
                    await_disable_approval(executor, out, report)
                    try:
                        executor.s.stop_all()
                    except (Exception, KeyboardInterrupt) as error:
                        report['stop_errors']['fatal'] = repr(error)
                        retain_until_supported(executor, out, report)
                    if sorted(report['reset_confirmed']) != list(range(1, 8)) or report['stop_errors']:
                        report['status'] = 'FAIL_DISABLE_CONFIRMATION'
                else:
                    device.stop()
                    device.write(0x5a0, 0)
                    for mid in range(1, 8):
                        device.write(device.pp_base(mid), 0)
            worker.requests.put(None)
            executor.wait_state_writer.close()
            report.update(final_controller=device.status(), final_active_mask=device.read(0x5a8),
                          heartbeat_max_gap_s=device.max_gap)
            device.close()
            summary = {k: report.get(k) for k in ('status', 'error', 'reset_confirmed', 'stop_errors',
                       'final_controller', 'final_active_mask', 'heartbeat_max_gap_s')}
            (out/'runtime_summary.json').write_text(json.dumps(summary))
            (out/'result.json.gz').write_bytes(gzip.compress(json.dumps(report).encode()))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('job', 'profile', 'out'):
        parser.add_argument('--'+key, type=Path, required=True)
    parser.add_argument('--execute-attended', action='store_true')
    args = parser.parse_args()
    job = json.loads(args.job.read_text())
    planner = JointPlanner(args.profile)
    validated = verify(job, planner)
    if not args.execute_attended:
        print(json.dumps(dict(status='OFFLINE_VALIDATED', segments=len(validated[0]), returns=len(validated[1]))))
    else:
        def interrupt(*unused):
            raise KeyboardInterrupt('Operator interruption; retain until supported')
        signal.signal(signal.SIGTERM, interrupt)
        signal.signal(signal.SIGINT, interrupt)
        execute(job, planner, args.out, validated)
