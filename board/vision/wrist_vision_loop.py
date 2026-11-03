"""Board-side continuous wrist correction loop (experimental): camera -> PL mask -> locator -> bounded PL joint offsets.

    python3 wrist_vision_loop.py serve            # resident; waits for commands from the motion owner
    python3 wrist_vision_loop.py replay DIR ...   # offline: run the locator on saved frames, print centres and timing

Runs beside the motion owner on the KV260, never through the AI PC. Each fresh wrist frame (local camera service) is reduced
to a mask by the PL pre-processing kernel (ARM fallback when the PL kernel is unavailable), the part is located with the board
locator, and the pixel error drives an integral correction in the tool plane:

    step = gain * solve(J_pixels_per_m, error)        cumulative += step, |cumulative| <= max_correction_m
    offset_motor = A @ cumulative                      written as Q8.24 to the PL correction targets (850..864)

The PL bounds every joint offset (limit, per-tick slew, lease) whatever this process writes; this process cannot enable the
channel (the owner does). Commands and state go through two small JSON files in /dev/shm (atomic replace):

    command: {"id", "mode": "run"|"freeze"|"stop", "params": {...}}       state: {"id", "status", "frames", ...}

Params come from the reviewed job (job['visual_servo']['continuous']) and are passed unchanged by the owner.
"""
import io
import json
import mmap
import os
import struct
import sys
import time
from pathlib import Path
from urllib.request import urlopen

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import vision_pre as vp          # noqa: E402
import wrist_locator as wl       # noqa: E402

COMMAND = Path('/dev/shm/wvl_command.json')
STATE = Path('/dev/shm/wvl_state.json')
CAMERA = 'http://127.0.0.1:8766/frame.jpg'
Q24 = 1 << 24
TARGETS, APPLIED, CTRL = 0x850, 0x870, 0x840


def write_json(path, value):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value))
    tmp.replace(path)


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


class Camera:
    """Latest frame of the board camera service, each sequence number once; grey 640x400."""
    def __init__(self):
        from PIL import Image
        self.Image, self.last = Image, None

    def frame(self, timeout=.5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with urlopen(CAMERA, timeout=.3) as response:
                raw = response.read()
                seq = int(response.headers['X-Frame-Sequence'])
                stamp = int(response.headers['X-Board-Received-Monotonic-Ns'])
            if seq == self.last:
                time.sleep(.005)
                continue
            self.last = seq
            image = self.Image.open(io.BytesIO(raw))
            image.draft('L', (vp.W, vp.H))
            gray = np.asarray(image.convert('L'), np.uint8)
            if gray.shape != (vp.H, vp.W):
                raise ValueError('wrist frame is not 640x400')
            return gray, seq, stamp
        raise TimeoutError('no fresh wrist frame')


def hole_center(sub, dark, min_area=150):
    """(x, y, area) of the nut's hole in its bounding box, or None: inside the body (the blob labels fill holes) but not dark.

    The host defines the nut centre as the hole centre (wrist_nut_localization.locate fits the hole); the body centroid
    differs from it by 5-20 px because the oblique wrist view sees one side of the 10 mm thick nut."""
    hole = sub & ~dark
    ys, xs = np.nonzero(hole)
    if len(xs) < min_area:
        return None
    return float(xs.mean()), float(ys.mean()), int(len(xs))


def locate_nut_in_mask(packed, expected, radius=220., min_area=2500, max_area=40000, max_aspect=1.8, min_fill=.55):
    """Dict(center_px, ...) of the compact dark body (M12 nut) nearest `expected`; centre = pixel centroid (gripped across it)."""
    if wl._blobs is None:
        wl._blobs = wl.Blobs()
    labels, stats = wl._blobs(packed)
    dark = vp.unpack(packed)
    expected = np.asarray(expected, float)
    choices = []
    for i, (pixels, boundary, sx, sy, x0, x1, y0, y1, border, _) in enumerate(stats):
        area = pixels - boundary / 2. - 1.
        if not min_area <= area <= max_area:
            continue
        dx = max(x0 - expected[0], 0., expected[0] - x1)
        dy = max(y0 - expected[1], 0., expected[1] - y1)
        if np.hypot(dx, dy) > radius:
            continue
        sub = labels[y0:y1 + 1, x0:x1 + 1] == i + 1
        ys, xs = np.nonzero(sub)
        pts = np.stack([xs + x0, ys + y0], 1).astype(float)
        mean = pts.mean(0)
        evals, vectors = np.linalg.eigh(np.cov((pts - mean).T))
        u = vectors[:, 1]
        along, across = (pts - mean) @ u, (pts - mean) @ np.array([-u[1], u[0]])
        length, width = float(np.ptp(along)), float(np.ptp(across))
        fill = float(area / max(length * width, 1.))
        if length / max(width, 1.) > max_aspect or fill < min_fill:
            continue
        hole = hole_center(sub, dark[y0:y1 + 1, x0:x1 + 1])
        if hole is None:
            continue
        center = np.array([hole[0] + x0, hole[1] + y0])
        choices.append(dict(center_px=[float(center[0]), float(center[1])], body_center_px=[float(mean[0]), float(mean[1])], cut=bool(border),
                            length_px=length, width_px=width, score=fill, hole_area_px=hole[2], distance=float(np.hypot(*(center - expected)))))
    if not choices:
        raise ValueError('No compact dark body (nut) with a visible hole near the expected position')
    choices.sort(key=lambda c: c['distance'])
    if len(choices) > 1 and choices[1]['distance'] < choices[0]['distance'] + wl.NEIGHBOUR_AMBIGUOUS_PX:
        raise ValueError('More than one nut near the expected position')
    return choices[0]


class Tracker:
    """Per-part observation, the same definitions as the AI PC servo (wrist_servo_host):

    M12 screw: PL pre-processing kernel + board locator (wrist_locator, matches the host locate_screw_near).
    M12 nut:   the host's own code (wrist_nut_localization, OpenCV on the board): rim centre by locate_near once,
               then the appearance template is tracked frame to frame (track_template), re-located after repeated misses.
    M6 screw / M6 nut: the host's own wrist_m6_localization (local contrast, head-end grip point, compact nut).
    A centre that jumps away from the recent ones (a glint flipping the screw head end, a reflection merging with a nut)
    is not used; after `max_rejects` such frames in a row the history restarts."""
    # nut 12 px (was 6): with the arm's own motion compensated the residual still reached 6 px while moving, and the
    # rejected frames left 0.5-1 s gaps, seen as two separate adjustments (run15, 4.8 s)
    JUMP_PX = dict(screw=15., nut=12.)

    def __init__(self, engine, kind, size, expected, radius):
        self.engine, self.kind, self.size, self.radius = engine, kind, size, radius
        self.expected = np.asarray(expected, float)
        self.template = self.offset = None
        self.history, self.rejects, self.misses = [], 0, 0

    def locate(self, gray):
        if self.size == 'm12' and self.kind == 'screw':
            return wl.locate_screw_near(self.engine, gray, self.expected, self.radius)
        import cv2  # noqa: F401  (OpenCV is only needed from here on)
        if self.size == 'm6':
            from wrist_m6_localization import locate_m6_near
            if self.kind == 'screw':
                # the head end found in earlier frames decides frames whose two ends look alike (run11: lost after the arm moved)
                o = locate_m6_near(gray, self.expected, 'screw_m6', self.radius, head_hint=getattr(self, 'head_hint', None))
                if not o.get('head_from_hint') and o.get('head_dir'):
                    self.head_hint = o['head_dir']
                return o
            return locate_m6_near(gray, self.expected, 'nut_m6', self.radius)
        from wrist_nut_localization import locate_near, track_template
        if self.template is None or self.misses >= 5:
            # Use the reviewed job's search radius, as the screw/M6 branches do.
            # The locator's 65 px default can exclude a visible rim before tracking starts.
            d = locate_near(gray, self.expected, self.radius)
            x, y, w, h = d['bbox']
            m = min(8, x - 1, y - 1, 639 - x - w, 399 - y - h)
            x, y, w, h = x - m, y - m, w + 2 * m, h + 2 * m
            if min(x, y) <= 0 or x + w >= 640 or y + h >= 400:
                raise ValueError('Template clipped')
            self.template, self.offset = gray[y:y + h, x:x + w].copy(), np.asarray(d['center_px'], float) - [x, y]
            self.misses = 0
            return d
        try:
            d = track_template(gray, self.template, self.offset, minimum_score=.78, search_center=self.expected, search_radius=25.)
        except ValueError:
            # find the rim again in THIS frame and take a new template there: waiting for 5 misses left 0.5-1 s gaps
            # while the arm moved (run16: 15 template rejections, the nut took 5.5 s in visible steps)
            self.template = None
            return self.locate(gray)
        self.misses = 0
        # as the AI PC servo does each iteration: take the template again where it matched, so it follows the changing view
        h, w = self.template.shape
        x, y = np.rint(np.asarray(d['center_px'], float) - self.offset).astype(int)
        if d.get('score', 0.) >= .85 and 0 < x and 0 < y and x + w < 640 and y + h < 400:
            self.template = gray[y:y + h, x:x + w].copy()
            self.offset = np.asarray(d['center_px'], float) - [x, y]
        return d

    def observe(self, gray, shift=None):
        """`shift`: image shift of the part caused by the correction in force at capture (J @ offset). The jump checks
        compare the part where it would be without the correction, so the arm's own motion is not a jump
        (run5: a 17 px move between two frames of a 7 mm correction was rejected, 0.42 s without an update)."""
        o = self.locate(gray)
        c = np.asarray(o['center_px'], float) - (np.zeros(2) if shift is None else np.asarray(shift, float))
        if not self.history:
            # like the host's frame agreement: a first sighting is used only once a second frame confirms it
            # (one frame can lock onto a neighbouring part 200 px away)
            self.raw = [r for r in getattr(self, 'raw', [])[-2:]]
            agree = [r for r in self.raw if np.linalg.norm(c - r) <= self.JUMP_PX[self.kind]]
            self.raw.append(c)
            if not agree:
                raise ValueError('Waiting for a second frame to agree')
            self.raw = []
        elif np.linalg.norm(c - np.median(self.history, axis=0)) > self.JUMP_PX[self.kind]:
            self.rejects += 1
            if self.rejects >= 5:
                self.history, self.rejects = [], 0
            raise ValueError('Observation jumped away from the recent ones')
        self.rejects = 0
        self.history = (self.history + [c])[-3:]
        self.expected = np.asarray(o['center_px'], float)
        return o


LOST_FRAMES = Path('/dev/shm/wvl_lost')


def keep_lost_frame(gray, index, keep=4):
    """The first frames the locator could not use (run10: an M6 screw lost after a 12 mm correction, no image kept)."""
    if index >= keep:
        return
    try:
        from PIL import Image
        LOST_FRAMES.mkdir(exist_ok=True)
        if index == 0:
            for old in LOST_FRAMES.glob('*.png'):
                old.unlink()
        Image.fromarray(gray).save(LOST_FRAMES / ('lost_%d.png' % index))
    except Exception:
        pass


def pixel_error(observation, target, kind, along_weight=.3):
    """Target minus centre. A screw is steered across its axis; along it with `along_weight`: weakly for an M12 screw
    (gripped anywhere on the shank), fully for an M6 screw (gripped just below the head: run8 converged 14 px = ~3 mm
    along the axis away from that point while reporting 4.4 px with the weak weight)."""
    e = np.asarray(target, float) - np.asarray(observation['center_px'], float)
    if kind != 'screw':
        return e
    a = np.radians(observation['axis_deg'])
    u = np.array([np.cos(a), np.sin(a)])
    n = np.array([-u[1], u[0]])
    return n * (n @ e) + along_weight * u * (u @ e) * (0. if observation.get('cut') else 1.)


class Correction:
    """The PL correction targets through the kv260-can UIO window (the owner enables the channel; the PL bounds it).
    Only constructed when the deployed bundle says its bitstream has the channel: on older firmware any access to these
    registers, reads included, is an AXI error that can abort the A53."""
    def __init__(self, regs):
        self.regs = regs

    def enabled(self):
        return struct.unpack_from('<I', self.regs, CTRL)[0] & 1

    def write(self, offsets_rad):
        for j, v in enumerate(offsets_rad):
            struct.pack_into('<i', self.regs, TARGETS + 4 * j, int(round(float(v) * Q24)))

    def applied(self):
        return [struct.unpack_from('<i', self.regs, APPLIED + 4 * j)[0] / Q24 for j in range(6)]


class ObserveOnly:
    """No correction channel: locate and report, never touch a motion register."""
    def enabled(self):
        return 0

    def write(self, offsets_rad):
        raise RuntimeError('observe-only loop cannot write corrections')

    def applied(self):
        return None


class AppliedHistory:
    """The PL's applied offsets (tool plane, via pinv(A)) sampled at 100 Hz, so a frame is compared with the correction
    the arm actually had when it was taken, not with the one commanded since."""
    def __init__(self, correction, A, period=.01, keep=300):
        import threading
        self.correction, self.pinv, self.period = correction, np.linalg.pinv(A), period
        self.samples, self.lock, self.stop = [], threading.Lock(), threading.Event()
        self.keep = keep
        self.sample()
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def sample(self):
        a = self.correction.applied()
        xy = self.pinv @ np.asarray(a, float) if a is not None else np.zeros(2)
        with self.lock:
            self.samples = (self.samples + [(time.monotonic_ns(), xy)])[-self.keep:]
        return xy

    def loop(self):
        while not self.stop.wait(self.period):
            self.sample()

    def at(self, t_ns):
        with self.lock:
            older = [xy for t, xy in self.samples if t <= t_ns]
            return (older[-1] if older else self.samples[0][1]).copy()

    def now(self):
        with self.lock:
            return self.samples[-1][1].copy()

    def close(self):
        self.stop.set()
        self.thread.join(1)


class Smoother:
    """Writes the PL targets at 100 Hz, following the loop's goal through a first-order filter (time constant `tau`).

    Without it each new goal (one per frame, ~15 Hz) is reached at the PL's constant slew and the arm stops until the next
    frame: a stop-go staircase seen as small pauses and shaking (run5). The PL limits still bound every value written.
    The writes renew the PL lease, so they stop when the loop has not set a goal for `stale_s`: a hung loop still lets
    the lease expire and the PL freeze the offsets."""
    def __init__(self, correction, A, tau, period=.01, stale_s=.3):
        import threading
        self.correction, self.A, self.period, self.stale = correction, np.asarray(A, float), period, stale_s
        self.alpha = 1. - np.exp(-period / tau)
        self.goal = self.command = np.zeros(2)
        self.updated = time.monotonic()
        self.lock, self.stop = threading.Lock(), threading.Event()
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def set(self, goal):
        with self.lock:
            self.goal, self.updated = np.asarray(goal, float).copy(), time.monotonic()

    def loop(self):
        while not self.stop.wait(self.period):
            with self.lock:
                if time.monotonic() - self.updated > self.stale:
                    continue
                self.command = self.command + self.alpha * (self.goal - self.command)
                command = self.command.copy()
            if self.correction.enabled():
                self.correction.write(self.A @ command)

    def close(self):
        self.stop.set()
        self.thread.join(1)


def run(command, engine, camera, correction):
    p = command['params']
    kind, size, target = p['part_class'], p.get('part_size', 'm12'), np.asarray(p['target_px'], float)
    J = np.asarray(p['J_pixels_per_m'], float)
    A = np.asarray(p['A_motor_rad_per_tool_m'], float)
    gain, limit, tol = float(p['gain']), float(p['max_correction_m']), float(p['tolerance_px'])
    tracker = Tracker(engine, kind, size, p['coarse_center_px'], float(p['radius_px']))
    # 'position' (default since 2026-10-30 run3): the target is where the part is, estimated from the frame and the offset the
    # arm had when it was taken (camera + motor lag `capture_lag_s`). The older 'integral' law adds gain*error to the TARGET every
    # frame; with ~0.3 s between a command and its image it keeps adding while the arm is still on its way and overshoots
    # (run3: two visible swings before the descent).
    position = p.get('control', 'integral') == 'position'
    lag_ns = int(float(p.get('capture_lag_s', .1)) * 1e9)
    settle = float(p.get('settled_m', .001))
    history = AppliedHistory(correction, A) if position else None
    smooth = float(p.get('smooth_s', 0.))
    along = float(p.get('along_weight', .3))
    smoother = Smoother(correction, A, smooth) if smooth > 0 and not isinstance(correction, ObserveOnly) else None
    cumulative = np.zeros(2)
    state = dict(id=command['id'], status='RUNNING', frames=0, lost=0, converged_frames=0, error_px=None, cumulative_tool_xy_m=[0., 0.],
                 latency_ms=[], started=time.monotonic(), control='position' if position else 'integral', trace=[])
    try:
        _loop(command, camera, correction, tracker, target, J, A, gain, limit, tol, position, lag_ns, settle, history, cumulative, state, kind,
              smoother, along)
    finally:
        if smoother:
            smoother.close()
        if history:
            history.close()


def _loop(command, camera, correction, tracker, target, J, A, gain, limit, tol, position, lag_ns, settle, history, cumulative, state, kind,
          smoother=None, along=.3):
    while True:
        current = read_json(COMMAND)
        if current is None or current.get('id') != command['id'] or current.get('mode') != 'run':
            state['status'] = 'FROZEN' if current and current.get('id') == command['id'] and current.get('mode') == 'freeze' else 'STOPPED'
            write_json(STATE, state)
            return
        t0 = time.monotonic()
        gray, seq, stamp = camera.frame()
        t1 = time.monotonic()
        had = history.at(stamp - lag_ns) if position else None
        try:
            o = tracker.observe(gray, J @ had if position else None)
        except ValueError as error:
            keep_lost_frame(gray, state['lost'])
            reasons = state.setdefault('rejects', {})
            reasons[str(error)[:40]] = reasons.get(str(error)[:40], 0) + 1
            state.update(lost=state['lost'] + 1, last_error=str(error), converged_frames=0,
                         cycle_ms=(state.get('cycle_ms', []) + [round((time.monotonic() - t0) * 1e3, 1)])[-200:])
            write_json(STATE, state)
            continue
        t2 = time.monotonic()
        e = pixel_error(o, target, kind, along)
        if position:
            # J maps a tool-plane correction to the image shift of the part: the correction that centres it is the one in
            # force at capture plus solve(J, e); move the target part of the way there
            cumulative = cumulative + gain * (had + np.linalg.solve(J, e) - cumulative)
        else:
            cumulative = cumulative + gain * np.linalg.solve(J, e)
        norm = np.linalg.norm(cumulative)
        if norm > limit:
            cumulative *= limit / norm
        if smoother:
            smoother.set(cumulative)
        elif correction.enabled():
            correction.write(A @ cumulative)
        err = float(np.linalg.norm(e))
        # converged only when the arm is also where the target is (else a frozen target would leave it short)
        arrived = not position or np.linalg.norm(history.now() - cumulative) <= settle
        latency = (time.monotonic_ns() - stamp) / 1e6
        state.update(frames=state['frames'] + 1, lost=0, error_px=err, center_px=o['center_px'], cumulative_tool_xy_m=cumulative.tolist(),
                     converged_frames=state['converged_frames'] + 1 if err <= tol and arrived else 0, frame_seq=seq, applied_rad=correction.applied(),
                     trace=(state['trace'] + [dict(t=round(time.monotonic() - state['started'], 3), err=round(err, 2), c=[round(v, 1) for v in o['center_px']],
                                                    target_mm=[round(v * 1e3, 2) for v in cumulative],
                                                    applied_mm=[round(v * 1e3, 2) for v in history.now()] if history else None,
                                                    age_ms=round(latency, 1))])[-200:],
                     latency_ms=(state['latency_ms'] + [latency])[-200:],
                     cycle_ms=(state.get('cycle_ms', []) + [round((time.monotonic() - t0) * 1e3, 1)])[-200:],
                     frame_ms=(state.get('frame_ms', []) + [round((t1 - t0) * 1e3, 1)])[-200:],
                     locate_ms=(state.get('locate_ms', []) + [round((t2 - t1) * 1e3, 1)])[-200:])
        write_json(STATE, state)


def warm_up(engine, camera):
    """First calls are slow (run5: the first frame took 1.1 s): run every locator once on a live frame before READY,
    i.e. before the owner enables anything. Results are discarded; failures (no part in view) are expected."""
    out = {}
    try:
        gray = camera.frame(timeout=2.)[0]
    except Exception as error:          # no camera: the run reports it
        return dict(error=repr(error))
    for kind, size in (('screw', 'm12'), ('nut', 'm12'), ('screw', 'm6'), ('nut', 'm6')):
        t = time.perf_counter()
        try:
            Tracker(engine, kind, size, (320., 200.), 220.).locate(gray)
        except Exception:
            pass
        out[kind + '_' + size + '_ms'] = round((time.perf_counter() - t) * 1e3, 1)
    camera.last = None
    return out


def isolate():
    """Keep this process off the motion owner's CPUs. With the loop running, the owner's largest heartbeat gap rose from
    21-39 ms (29 stop-and-look runs) to 44-61 ms (7 continuous runs) against a 100 ms PL lease, and one run stalled 0.74 s
    (2026-10-30 run13): cores 2-3 only, lower priority, two OpenCV threads."""
    done = {}
    try:
        os.sched_setaffinity(0, {2, 3})
        done['cpus'] = sorted(os.sched_getaffinity(0))
    except (AttributeError, OSError) as error:
        done['cpus_error'] = repr(error)
    try:
        done['nice'] = os.nice(10)
    except OSError as error:
        done['nice_error'] = repr(error)
    try:
        import cv2
        cv2.setNumThreads(2)
        done['cv2_threads'] = cv2.getNumThreads()
    except ImportError:
        pass
    return done


def exit_with_parent(period=.5):
    """The owner starts this process before it loads its jobs; if the owner is gone (a failed validation, a crash, SIGTERM) this
    process must not stay behind holding the camera service and the correction window."""
    import threading
    parent = os.getppid()

    def watch():
        while True:
            time.sleep(period)
            if os.getppid() != parent:
                os._exit(1)
    threading.Thread(target=watch, daemon=True).start()


def serve():
    exit_with_parent()
    isolation = isolate()
    engine = vp.open_best()
    regs = getattr(engine, 'regs', None)
    if regs is None:
        dev, size = vp._uio()
        fd = os.open(dev, os.O_RDWR | os.O_SYNC)
        regs = mmap.mmap(fd, 0x1000, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
    try:
        from canctl import correction_channel_present
        present = correction_channel_present(HERE)
    except ImportError:              # an older bundle: its firmware has no channel
        present = False
    correction, camera = (Correction(regs) if present else ObserveOnly()), Camera()
    warm = warm_up(engine, camera)
    write_json(STATE, dict(id=None, status='READY', warm_up=warm, isolation=isolation, engine=engine.name, pid=os.getpid(), correction_channel=present))
    handled = None
    while True:
        command = read_json(COMMAND)
        if command and command.get('mode') == 'exit':
            return
        if command and command.get('mode') == 'run' and command.get('id') != handled:
            handled = command['id']
            try:
                run(command, engine, camera, correction)
            except Exception as error:     # report and wait for the next command; the owner decides what the arm does
                write_json(STATE, dict(id=command['id'], status='ERROR', error=repr(error)))
        time.sleep(.01)


def replay(folders):
    engine = vp.open_best()
    for folder in folders:
        folder = Path(folder)
        config = json.loads((folder / 'servo_config.json').read_text()) if (folder / 'servo_config.json').exists() else {}
        kind, size = config.get('part_class', 'screw'), config.get('part_size', 'm12')
        from PIL import Image
        for frame in sorted(folder.glob('iteration_*/frame_0.png')):
            gray = np.asarray(Image.open(frame).convert('L'), np.uint8)
            vin = json.loads((frame.parent / 'vision_input.json').read_text()) if (frame.parent / 'vision_input.json').exists() else {}
            expected = vin.get('predicted_px', config.get('coarse_center_px', [320, 200]))
            t = time.perf_counter()
            radius = float(vin.get('radius_px', 220.)) if size == 'm12' else (110. if kind == 'nut' else float(vin.get('radius_px', 220.)))
            try:
                tracker = Tracker(engine, kind, size, expected, radius)
                try:
                    tracker.observe(gray)          # a single saved frame: the second sighting below confirms the first
                except ValueError as first:
                    if 'second frame' not in str(first):
                        raise
                o = tracker.observe(gray)
                print(json.dumps(dict(frame=str(frame), kind=kind, size=size, center_px=o['center_px'], ms=round((time.perf_counter() - t) * 1e3, 1))))
            except ValueError as error:
                print(json.dumps(dict(frame=str(frame), kind=kind, size=size, error=str(error))))


if __name__ == '__main__':
    if sys.argv[1:2] == ['serve']:
        serve()
    elif sys.argv[1:2] == ['replay']:
        replay(sys.argv[2:])
    else:
        print(__doc__)
