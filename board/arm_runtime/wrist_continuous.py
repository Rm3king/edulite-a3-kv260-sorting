"""Continuous wrist correction on the motion owner (experimental; a job opts in with job['visual_servo']['continuous']).

No stop-and-look and no image round trip to the AI PC: the arm flies to the hover of the reviewed job and, without settling,
the board vision process (contest/vision_pre/board/wrist_vision_loop.py) steers the PL correction channel. When the part has
been centred for a few frames the offsets are frozen and the job's own reviewed descent and grasp run with them; after the
lift the channel is switched off and the offsets slew back to zero during the transfer.

Safety rests on the PL: per-joint offset limits, per-tick slew, lease freeze and the segment-bound check (fault 6) bound
whatever the vision process writes, and the planner audits the job's geometry at the corners of the correction box before
anything is enabled. A lost part, a timeout or a vision error switches the channel off, waits until the offsets are zero
and takes the job's reviewed return; a failed grasp zeroes the offsets before the reviewed recovery.
"""
import json
import math
import os
import subprocess
import sys
import time
import uuid
from pathlib import Path

Q24 = 1 << 24
COMMAND = Path('/dev/shm/wvl_command.json')
STATE = Path('/dev/shm/wvl_state.json')
BUNDLE = Path('/home/ubuntu/kv260_can_deploy')
CTRL, STEP, LEASE, TARGETS, APPLIED, REJECTS, LIMITS = 0x840, 0x848, 0x84c, 0x850, 0x870, 0x898, 0x8a0


def signed(value):
    return ((value & 0xffffffff) ^ 0x80000000) - 0x80000000


_PRESENT = []


def channel_present():
    """Cached: the check hashes the 7.8 MB bitstream (45 ms on the A53)."""
    if not _PRESENT:
        try:
            from canctl import correction_channel_present
        except ImportError:
            _PRESENT.append(False)
        else:
            _PRESENT.append(correction_channel_present())
    return _PRESENT[0]


def check_before_enable():
    """Run the slow firmware checks before any axis is enabled. After enable nothing may stall the feedback pump: two
    uncached checks inside configure() (channel_present + canctl's own write whitelist, 2 x 45 ms) exceeded the 80 ms
    Type2 freshness limit on 2026-10-30 (runs 2 and 4: 'Axis 1: missing/stale Type2', PL feedback watchdog fault 8)."""
    if not channel_present():
        raise RuntimeError('Deployed firmware has no online correction channel')
    import canctl
    if hasattr(canctl, '_correction_bundle') and not canctl._correction_bundle():
        raise RuntimeError('canctl does not allow the correction registers for this bundle')


LIMIT_CEILING_RAD, STEP_CEILING_RAD, LEASE_CEILING_US = .25, .003, 500000     # the PL refuses anything above these


def validate_config(job):
    """Checked with the job before anything is enabled: bounds inside the PL ceilings, the limits cover the correction box
    through A, a passed tube audit at the four corners, and loop parameters in range."""
    import math
    cfg = job['visual_servo']['continuous']
    v = cfg['vision']
    limits, A = cfg['limits_rad'], v['A_motor_rad_per_tool_m']
    if len(limits) != 6 or not all(math.isfinite(x) and 0 <= x <= LIMIT_CEILING_RAD for x in limits) or not any(limits):
        raise ValueError('continuous: joint limits outside the PL ceiling')
    if not 0 < cfg['step_rad'] <= STEP_CEILING_RAD or not 1 <= cfg['lease_us'] <= LEASE_CEILING_US:
        raise ValueError('continuous: step or lease outside the PL ceiling')
    if len(A) != 6 or any(len(r) != 2 or not all(math.isfinite(x) for x in r) for r in A):
        raise ValueError('continuous: correction map must be 6x2')
    m = v['max_correction_m']
    if not 0 < m <= .03 or m != cfg['max_correction_m']:
        raise ValueError('continuous: correction box')
    for j in range(6):
        if abs(A[j][0]) * m + abs(A[j][1]) * m > limits[j] + 1e-12:
            raise ValueError('continuous: joint %d limit does not cover the correction box' % (j + 1))
    audit = cfg.get('tube_audit') or {}
    if not audit.get('passed') or len(audit.get('corners', [])) != 4 or not all(str(c.get('status', '')).startswith('PASS') for c in audit['corners']):
        raise ValueError('continuous: no passed tube audit')
    if v['part_class'] not in ('screw', 'nut') or v.get('part_size', 'm12') not in ('m12', 'm6') or not 0 < v['gain'] <= 1 or not 0 < cfg['timeout_s'] <= 10:
        raise ValueError('continuous: loop parameters')
    if cfg['converge_frames'] < 1 or cfg['lost_frames'] < 1 or v['tolerance_px'] <= 0:
        raise ValueError('continuous: convergence parameters')
    if v.get('control', 'integral') not in ('integral', 'position') or not 0 <= v.get('capture_lag_s', .1) <= 1 or not 0 < v.get('settled_m', .001) <= .005 or not 0 <= v.get('smooth_s', 0.) <= .5 or not 0 <= v.get('along_weight', .3) <= 1:
        raise ValueError('continuous: control law parameters')
    if not 0 <= cfg.get('fade_s', 0) <= 2:
        raise ValueError('continuous: fade time')
    if not 0 <= cfg.get('track_only_s', 0) <= 30:
        raise ValueError('continuous: track-only time')


def merged_config(jobs):
    """One PL configuration for a chain: the widest per-joint limit, the largest step, the shortest lease of its continuous jobs."""
    cfgs = [j['visual_servo']['continuous'] for j in jobs if j.get('visual_servo', {}).get('continuous')]
    if not cfgs:
        return None
    return dict(limits_rad=[max(c['limits_rad'][i] for c in cfgs) for i in range(6)], step_rad=max(c['step_rad'] for c in cfgs),
                lease_us=min(c['lease_us'] for c in cfgs))


def configure(d, cfg):
    """Before the PL stream starts (the PL locks these while running); every value is read back."""
    if not channel_present():
        raise RuntimeError('Deployed firmware has no online correction channel')
    if d.read(CTRL) & 1:
        raise RuntimeError('Correction channel enabled before configuration')
    rejects = d.read(REJECTS)
    writes = [(LIMITS + 4 * j, int(round(v * Q24))) for j, v in enumerate(cfg['limits_rad'])]
    writes += [(STEP, int(round(cfg['step_rad'] * Q24))), (LEASE, int(cfg['lease_us']))]
    for address, value in writes:
        if value < 0:
            raise ValueError('negative correction bound')
        d.write(address, value)
        if d.read(address) != value:
            raise RuntimeError('Correction configuration readback mismatch at %#x' % address)
    if d.read(REJECTS) != rejects:
        raise RuntimeError('PL refused a correction configuration write')


def enable(d):
    rejects = d.read(REJECTS)
    d.write(CTRL, 1)
    if not d.read(CTRL) & 1 or d.read(REJECTS) != rejects:
        raise RuntimeError('PL refused to enable the correction channel')


def disable(d):
    d.write(CTRL, 0)


class Fade:
    """Takes the frozen offsets back to zero along a raised cosine over `seconds` (zero speed at both ends), written by the
    owner itself on every pump of the transfer; then switches the channel off.

    Switching off right after the lift let the PL return the offsets at its full step: 32 mrad on J1/J6 in 0.13 s with an
    abrupt start and stop, seen as a shake of the lifted screw (2026-10-30 run15). The PL still bounds every step; target
    writes renew its lease. `finish()` (before the place) switches off whatever is left."""
    def __init__(self, executor, seconds):
        self.e, self.seconds, self.done = executor, float(seconds), False
        self.start = applied_rad(executor.d)
        self.t0 = time.monotonic()
        self.pump_original = executor.pump
        executor.pump = self.pump

    def pump(self, *a, **k):
        status = self.pump_original(*a, **k)
        if not self.done:
            self.step()
        return status

    def step(self):
        u = (time.monotonic() - self.t0) / self.seconds
        if u >= 1.:
            self.finish()
            return
        f = .5 + .5 * math.cos(math.pi * u)
        for j, v in enumerate(self.start):
            self.e.d.write(TARGETS + 4 * j, round(v * f * Q24) & 0xffffffff)

    def finish(self):
        if not self.done:
            self.done = True
            for j in range(6):
                self.e.d.write(TARGETS + 4 * j, 0)
            disable(self.e.d)
            self.e.pump = self.pump_original


def applied_rad(d):
    return [signed(d.read(APPLIED + 4 * j)) / Q24 for j in range(6)]


def wait_zero(executor, timeout=3.):
    """Pump the controller until the PL has slewed every offset back to zero (channel disabled)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.pump(.005)
        executor.diagnostics()
        if not any(applied_rad(executor.d)):
            return
    raise RuntimeError('Correction offsets did not return to zero')


def offset_goal(executor, goal):
    """A joint goal moved by the offsets the PL currently adds (motor offsets divided by the joint direction)."""
    p, off = executor.p, applied_rad(executor.d)
    return [g + (off[i] / p.direction[i] if i < 6 else 0.) for i, g in enumerate(goal)]


def write_json(path, value):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value))
    tmp.replace(path)


def read_state():
    try:
        return json.loads(STATE.read_text())
    except (OSError, ValueError):
        return None


class VisionLoop:
    """The board vision process, started before anything is enabled and ended with the session."""
    def __init__(self, out, wait=True, log=None):
        """`wait=False`: start the process only (the owner does this at its very start, while it still loads and validates
        the jobs: 5.9 s of single-core work on the board that the vision process's ~4 s start-up and warm-up hides behind);
        `wait_ready()` is then called where the loop is needed, before anything is enabled."""
        for f in (COMMAND, STATE):
            f.unlink(missing_ok=True)
        script = BUNDLE / 'wrist_vision_loop.py'
        if not script.exists():
            raise RuntimeError('wrist_vision_loop.py is not deployed')
        self.log = open(log or Path(out) / 'vision_loop.log', 'w')
        self.engine = None
        self.process = subprocess.Popen(['/usr/bin/python3', str(script), 'serve'], cwd=str(BUNDLE), stdin=subprocess.DEVNULL,
                                        stdout=self.log, stderr=subprocess.STDOUT, env=dict(os.environ, PYTHONUNBUFFERED='1'))
        if wait:
            self.wait_ready()

    def wait_ready(self, timeout=12.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = read_state()
            if state and state.get('status') == 'READY':
                self.engine = state.get('engine')
                return
            if self.process.poll() is not None:
                raise RuntimeError('Vision loop exited during start')
            time.sleep(.05)
        self.close()
        raise RuntimeError('Vision loop did not become ready')

    def command(self, mode, cid, params=None):
        write_json(COMMAND, dict(id=cid, mode=mode, params=params))

    def close(self):
        try:
            write_json(COMMAND, dict(id=None, mode='exit'))
            self.process.wait(timeout=2)
        except Exception:
            self.process.terminate()
        self.log.close()


def servo_continuous(index, job, digest, records, returns, executor, loop, out, report):
    """Hover, centre on the fly, then the reviewed grasp; returns the part's status like servo_nut."""
    from wrist_servo_motion import execute_grasp_tail
    from continuous_transfer import WRIST_RETURN_TEMPERATURE_C
    cfg = job['visual_servo']['continuous']
    task = job['tasks'][0]
    executor.execute_segments(task['segments'])
    enable(executor.d)
    cid = uuid.uuid4().hex
    started = time.monotonic()
    loop.command('run', cid, cfg['vision'])
    executor.event('CONTINUOUS_SERVO_START', nut=index, engine=loop.engine)
    status, last = None, None
    while status is None:
        executor.pump(.004)
        executor.diagnostics()
        state = read_state() or {}
        if state.get('id') == cid:
            last = state
        elapsed = time.monotonic() - started
        if cfg.get('track_only_s'):
            # commissioning: follow the part for this long (an operator may move it), then return without grasping
            if last and last.get('status') == 'ERROR':
                status = 'RETURN_VISION_ERROR'
            elif elapsed > cfg['track_only_s']:
                status = 'TRACK_ONLY_DONE'
            continue
        if last and last.get('status') == 'ERROR':
            status = 'RETURN_VISION_ERROR'
        elif last and last.get('lost', 0) >= cfg['lost_frames']:
            status = 'RETURN_VISION_LOST'
        elif last and last.get('converged_frames', 0) >= cfg['converge_frames']:
            status = 'CONVERGED'
        elif elapsed > cfg['timeout_s']:
            status = 'RETURN_CONTINUOUS_TIMEOUT'
        elif max(executor.s.require(i, 2)['temperature'] for i in range(1, 8)) >= WRIST_RETURN_TEMPERATURE_C:
            status = 'RETURN_BUDGET_OR_TEMPERATURE'
    loop.command('freeze' if status == 'CONVERGED' else 'stop', cid)
    summary = {k: last.get(k) for k in ('frames', 'lost', 'error_px', 'cumulative_tool_xy_m', 'applied_rad', 'center_px')} if last else {}
    latencies = (last or {}).get('latency_ms') or []
    for k in ('control', 'trace', 'cycle_ms', 'frame_ms', 'locate_ms', 'last_error', 'rejects'):
        if last and k in last:
            summary[k] = last[k]
    summary.update(seconds=time.monotonic() - started, frame_latency_ms_p50=sorted(latencies)[len(latencies) // 2] if latencies else None,
                   frame_latency_ms_max=max(latencies) if latencies else None)
    report.setdefault('continuous', []).append(dict(nut=index, status=status, **summary))
    executor.event('CONTINUOUS_SERVO_END', nut=index, status=status, seconds=round(summary['seconds'], 3), error_px=summary.get('error_px'))
    if status != 'CONVERGED':
        disable(executor.d)
        wait_zero(executor)
        branch, return_records = job['recoveries'][task['name']], returns[task['name']]
        executor.transport.prepared = return_records
        executor.transport.prepared_index = 0
        executor.execute_segments(branch['segments'])
        report['return_endpoint'] = executor.settle_arm(job['chain']['support_rad'] if job.get('chain') else job['start_rad'])
        return status
    # the reviewed tail of this very job, with the frozen offsets; pose checks include them, the offsets end after the lift
    reply = dict(candidate=job, full_records=records, records=records[len(task['segments']):])
    return execute_grasp_tail(executor, reply, out, report, continuous=sys.modules[__name__])
