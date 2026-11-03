"""Reusable live workflow, with separate prepare and attended execute actions.

Normal startup never enables a motor. The board runtime is content-addressed:
only job data changes between trials, not FPGA firmware or Python source.
"""
import argparse
import contextlib
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import threading
import time
import uuid


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    path = Path(path)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def inside(uv, polygon):
    x, y = uv
    result = False
    for (xa, ya), (xb, yb) in zip(polygon, polygon[1:] + polygon[:1]):
        if (ya > y) != (yb > y) and x < (xb - xa) * (y - ya) / (yb - ya) + xa:
            result = not result
    return result


def scene_objects(snapshot, config, classes=('screw', 'nut')):
    if config.get('reverse_small_nut'):
        from .reverse_transfer import tray_objects
        return [o for o in tray_objects(snapshot, config, 'B') if o['part_class'] in classes]
    # This is tray identity/robot exclusion, not a gripper-to-wall clearance test.
    box = config['excluded_robot_tip_xyxy']
    rows = []
    for o in snapshot['objects']:
        if o['part_class'] not in classes:
            continue
        if o.get('center_uv_px') is None:
            continue   # no pixel centre (geometry rejected, e.g. a nut seen edge-on): cannot be a scene object
        if inside(o['center_uv_px'], config['source_roi_uv']) and not (box[0] <= o['center_uv_px'][0] <= box[2] and box[1] <= o['center_uv_px'][1] <= box[3]):
            rows.append(o)
    return rows


def select(snapshot, config, screws, nuts):
    if any(type(n) is not int or n < 0 for n in (screws, nuts)) or not 1 <= screws + nuts <= 5:
        raise ValueError('请选择合计 1～5 件零件')
    objects = scene_objects(snapshot, config)
    chosen = []
    for cls, count in [('screw', screws), ('nut', nuts)]:
        candidates = sorted((o for o in objects if o['part_class'] == cls), key=lambda o: o['detection_index'])
        if len(candidates) < count:
            raise ValueError(f'{cls}: 需要 {count}，当前识别到 {len(candidates)}')
        chosen.extend(candidates[:count])
    return chosen


def select_reviewed_part(snapshot, config, uv, part_class='nut'):
    candidates=[o for o in scene_objects(snapshot,config) if o['part_class']==part_class
                and math.dist(o['center_uv_px'],uv)<6.]
    if len(candidates)!=1:raise ValueError('Reviewed wrist target moved or is ambiguous')
    return candidates


def select_reviewed_nut(snapshot, config, uv):
    return select_reviewed_part(snapshot,config,uv,'nut')


def counts_from_intent(intent):
    """Supported fixture semantics; reject rather than reuse previous counts."""
    from .tasks import validate
    intent=validate(intent)
    if intent['action']!='plan':raise ValueError('命令尚未形成可执行计划')
    classes=[step['part_class'] for step in intent['steps']]
    if classes!=sorted(classes,key=lambda value:value!='screw'):
        raise ValueError('当前批次支持先螺丝后螺母；不能自动改写你的次序')
    counts={'screw':0,'nut':0}
    for step in intent['steps']:
        if step['destination']!='B' or step['region'] not in ('all','right') or type(step['quantity']) is not int:
            raise ValueError('当前实物入口支持明确数量、右盘到左盘B盒；请调整命令')
        counts[step['part_class']]+=step['quantity']
    if not 1<=sum(counts.values())<=5:raise ValueError('实物单批支持合计1～5件')
    return counts['screw'],counts['nut']


def unchanged(reference, current, max_pixels=10.):
    """Match all source objects, including objects not selected for picking.

    The head-camera detection of one resting screw moves by up to 7 px between frames (2026-10-27 snapshots), so 6 px aborted
    runs with 'Scene changed' while nothing had moved; a hand or a slid part moves far more, and the wrist servo corrects what is left.
    """
    for cls in ('screw', 'nut'):
        a = [o for o in reference if o['part_class'] == cls]
        b = [o for o in current if o['part_class'] == cls]
        if len(a) != len(b):
            return False
        if not a:
            continue
        # Scenes are limited to a few isolated parts; bound combinatorial work.
        if len(a) > 8:
            return False
        def distance(u,v):
            ua=(u.get('pixel_geometry') or {}).get('axis_unit_uv')
            va=(v.get('pixel_geometry') or {}).get('axis_unit_uv')
            if cls=='screw' and (ua is not None or va is not None):
                if ua is None or va is None:return float('inf')
                if abs(sum(x*y for x,y in zip(ua,va)))<math.cos(math.radians(5)):return float('inf')
            return sum((x-y)**2 for x,y in zip(u['center_uv_px'],v['center_uv_px']))
        if min(max(distance(u,v) for u, v in zip(a, order))
               for order in itertools.permutations(b)) > max_pixels**2:
            return False
    return True


class Session:
    def __init__(self, app_root, emit=print, overrides=None):
        self.app = Path(app_root).resolve()
        self.root = self.app / 'integration'
        self.config_file = self.root / 'config/live_workflow.json'
        # Explicit, recorded run-time overrides (e.g. joint_time_law); the file stays the only other source.
        self.overrides = dict(overrides or {})
        self.config = {**read(self.config_file), **self.overrides}
        self.emit = emit
        self.runtime = None
        self.runtime_lock = threading.Lock()  # one MotionRuntime (pinocchio model): planning and forward kinematics take turns
        self.release = None

    def file_config(self):
        return {**read(self.config_file), **self.overrides}

    def event(self, event, **fields):
        self.emit(json.dumps(dict(event=event, **fields), ensure_ascii=False))

    @contextlib.contextmanager
    def stage(self, out, name):
        started = time.perf_counter()
        self.event('STAGE_START', stage=name)
        try:
            yield
        finally:
            timing = read(out/'timings.json') if (out/'timings.json').exists() else {}
            timing[name] = time.perf_counter() - started
            write(out/'timings.json', timing)
            self.event('STAGE_END', stage=name, seconds=timing[name])

    def remote(self, code, payload=None, timeout=30):
        command = self.config['board_python'] + ' -c ' + shlex.quote(code)
        r = subprocess.run(['bash', self.config['ssh_helper'], command],
                           input=json.dumps(payload or {}), capture_output=True, text=True, timeout=timeout)
        if r.returncode:
            raise RuntimeError(r.stderr[-2000:] or r.stdout[-2000:])
        return json.loads(r.stdout)

    def ensure_board(self):
        """Reuse loaded PL, or load the saved package once after a cold boot."""
        code = '''import pathlib,json,fcntl,subprocess,sys
sys.path.insert(0,'/home/ubuntu/kv260_can_deploy')
from canctl import CanDevice
with open('/tmp/kv260_gripper_commission.lock','a') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 nodes=[p for p in pathlib.Path('/sys/class/uio').glob('uio*/name') if p.read_text().strip()=='kv260-can']
 loaded=False;log=''
 if not nodes:
  p=subprocess.run(['sudo','-n','/usr/local/sbin/kv260-project-admin','deploy'],capture_output=True,text=True,check=True,timeout=120)
  loaded=True;log=p.stdout
 try:d=CanDevice()
 except PermissionError:
  subprocess.run(['sudo','-n','/usr/local/sbin/kv260-project-admin','grant-uio'],capture_output=True,text=True,check=True)
  d=CanDevice()
 try:
  version=d.read(0x470)
  if version not in (0x60002,0x60003):raise RuntimeError('Unexpected firmware; do not overwrite automatically: '+hex(version))
  state=d.status();active=d.read(0x5a8)
 finally:d.close()
 print(json.dumps(dict(firmware_version=hex(version),fpga_reloaded=loaded,control=state['control'],active_mask=active,fault=state['fault'],enable_commands_sent=0,boot_load_log=log)))
'''
        result = self.remote(code, timeout=140)
        write(self.app/'reports/live_board_startup.json', result)
        return result

    def board_status(self):
        return self.remote('''import sys,json
sys.path.insert(0,'/home/ubuntu/kv260_can_deploy')
from canctl import CanDevice
d=CanDevice()
try:print(json.dumps(dict(registers=d.status(),firmware_version=hex(d.read(0x470)),active_mask=d.read(0x5a8),register_writes=0,motor_commands_sent=0)))
finally:d.close()
''')

    def camera(self):
        active = read(self.app/'reports/rgbd_surface_preview_active.json')
        try:
            cmd = Path(f"/proc/{active['pid']}/cmdline").read_bytes()
        except FileNotFoundError:
            # The detector process is gone (2026-11-01 02:51: its window was closed and it exited; every order then failed with a
            # bare FileNotFoundError). Start it again once, the same idempotent startup the operator would run, and re-read.
            from .live_startup import ensure_camera
            ensure_camera(self.app)
            active = read(self.app/'reports/rgbd_surface_preview_active.json')
            try:cmd = Path(f"/proc/{active['pid']}/cmdline").read_bytes()
            except FileNotFoundError as error:raise RuntimeError('相机进程已退出且自动重启失败，请运行 live_startup') from error
        if str(active['report_dir']).encode() not in cmd or b'detect_parts_rgbd_live.py' not in cmd:
            raise RuntimeError('相机进程已变化，请重新启动识别')
        return active, Path(active['report_dir'])

    def snapshot(self):
        active, folder = self.camera()
        # New = not there before the request. Frame numbers restart when the detector restarts the RealSense pipeline after
        # a colour stall (2026-10-30 13:03: 101659 -> ~1000), so ordering by number timed out on every later snapshot.
        before = {p.name for p in folder.glob('frame_*_points.json')}
        os.kill(active['pid'], signal.SIGUSR1)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            candidates = [p for p in folder.glob('frame_*_points.json') if p.name not in before]
            if candidates:
                p = max(candidates, key=lambda p: p.stat().st_mtime_ns)
                snapshot = read(p)
                age = time.monotonic() - snapshot['host_acquired_monotonic_s']
                if not 0 <= age < 3 or not snapshot['synchronized']:
                    raise ValueError('新图像不同步或已过期')
                if self.config.get('part_size', 'm12') == 'm6':
                    self.map_m6(p, snapshot)
                return p
            time.sleep(.08)
        raise TimeoutError('相机保存新帧超时')

    M6_AS_KIND = {'screw_m6': 'screw', 'nut_m6': 'nut'}

    def map_m6(self, path, snapshot):
        """part_size 'm6': the whole chain downstream sees the M6 detections as screw / nut (same grasp procedure and descent).

        Only the M6 detections are kept; detection indices are unchanged, `detected_class` and `size_class` keep the origin, and the
        detector's own file is saved next to it as *_points.raw.json. M12 parts are kept as `screw_m12`/`nut_m12` obstacles only."""
        raw = path.with_name(path.name.replace('_points.json', '_points.raw.json'))
        if not raw.exists():
            raw.write_text(path.read_text())
        objects = [dict(o, part_class=self.M6_AS_KIND[o['part_class']], detected_class=o['part_class'], size_class='m6')
                   for o in snapshot['objects'] if o['part_class'] in self.M6_AS_KIND]
        # M12 parts stay in the scene as obstacles only, renamed so that nothing downstream takes them for a target
        objects += [dict(o, part_class=o['part_class'] + '_m12', detected_class=o['part_class'], size_class='m12')
                    for o in snapshot['objects'] if o['part_class'] in ('screw', 'nut')]
        write(path, dict(snapshot, objects=objects, part_size_mapped='m6', raw_points=raw.name))

    def inventory_frames(self):
        """Read an existing pipeline only, never open another camera or robot."""
        _,folder=self.camera();frames=[];deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            try:frame=read(folder/'latest.json')
            except (OSError,ValueError):time.sleep(.05);continue
            if not frame['synchronized'] or not 0<=time.monotonic()-frame['host_acquired_monotonic_s']<2:
                raise ValueError('相机图像过期或不同步')
            if not frames or frame['frame_number']!=frames[-1]['frame_number']:
                frames.append(frame)
                if len(frames)==3:return frames
            time.sleep(.15)
        raise TimeoutError('等待三张不同的相机图像超时')

    def preview_goal(self, goal):
        from .inventory_goal import stable_inventory,reconcile,describe
        if self.file_config()!=self.config:raise ValueError('配置已改变，请重启任务进程')
        frames=self.inventory_frames();observed=stable_inventory(frames,self.config)
        decision=reconcile(goal,observed)
        out=self.app/'reports'/('inventory_goal_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]);out.mkdir()
        write(out/'frames.json',frames);write(out/'inventory.json',observed);write(out/'decision.json',decision)
        return dict(directory=str(out),decision=decision,description=describe(decision),motor_commands_sent=0,
                    physical_execution_enabled=self.config.get('inventory_goal_execution_enabled',False))

    def verify_goal(self, directory):
        from .inventory_goal import stable_inventory,verify
        out=Path(directory).resolve()
        if out.parent!=self.app/'reports' or not out.name.startswith('live_task_'):raise ValueError('Unknown task')
        meta=read(out/'prepared.json')
        if not meta.get('goal') or meta['replay_only']:raise ValueError('No live goal to verify')
        state=read(out/'latest_remote_state.json')
        if state.get('alive') or not state.get('summary'):raise ValueError('请先完成返回和本轮承托失能确认，再核对目标')
        if (state['summary'].get('status') not in ('COMPLETED_TRANSFER_MOTION_VERIFY_IMAGES','TASK_FAILED_RETURNED_SUPPORTED') or
            state['summary'].get('stop_errors') or state['summary'].get('hold_hardware_fault') or
            state['summary'].get('final_controller',{}).get('fault',1)!=0 or
            sorted(state['summary'].get('reset_confirmed',[]))!=list(range(1,8))):
            raise ValueError('控制流程有故障，须先处理，不能自动判定任务完成')
        frames=self.inventory_frames();after=stable_inventory(frames,self.config)
        result=verify(meta['goal'],meta['inventory_before'],after)
        write(out/'goal_after_frames.json',frames);write(out/'goal_verification.json',result)
        return result

    def capture_joints(self, out):
        # Stopped-only reconnect after a power cycle: no motor enable/reset.
        # Apply guard configuration BEFORE CAN starts (never live MMIO setup).
        if self.release is None:
            write(out/'release.json', self.install_release())
        code = '''import json,fcntl,sys,subprocess
x=json.load(sys.stdin);sys.path[:0]=[x['release'],'/home/ubuntu/kv260_motion_runtime/src','/home/ubuntu/kv260_can_deploy']
from canctl import CanDevice
from arm_commission import Session
from stopped_connection import setup_stopped_feedback
with open('/tmp/kv260_gripper_commission.lock','a') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);d=CanDevice()
 report=dict(commands=[],frames=[],feedback=[],readbacks={},reset_confirmed=[],stop_errors={});s=Session(d,report)
 try:state=setup_stopped_feedback(d,s)
 finally:d.close()
 p=subprocess.run(['/usr/bin/python3','read_arm.py','--rounds','3'],cwd='/home/ubuntu/kv260_can_deploy',capture_output=True,text=True,check=True)
 print(json.dumps(dict(state=state,capture=json.loads(p.stdout),connection_setup=report)))
'''
        value = self.remote(code, dict(release=self.release))
        write(out/'board_state.json', value['state'])
        write(out/'board_capture.json', value['capture'])
        write(out/'connection_setup.json', value['connection_setup'])

    def prepare(self, screws=1, nuts=0, replay=None, goal=None, *, wrist_nut_trial=None, source_nut_uv_px=None, destination_slot_index=None, chain=None, source_part_class='nut', defer_geometry=False, captured=None, replay_points=None):
        """`captured=(points_file, joints_dir)` reuses a frame and stopped-arm capture the caller took moments ago.
        `defer_geometry` returns after planning with a `finish` callable that runs the geometry audit and bundle export."""
        if wrist_nut_trial is not None and (screws!=0 or nuts!=1 or replay is not None or goal is not None):
            raise ValueError('Wrist commissioning is limited to one fresh attended nut')
        if source_part_class not in ('nut','screw'):raise ValueError('Unknown part class')
        if source_nut_uv_px is not None and ((screws,nuts)!=((1,0) if source_part_class=='screw' else (0,1)) or (replay is not None and chain is None) or goal is not None):
            raise ValueError('Explicit source selection requires one fresh part of the stated class')
        started = time.perf_counter()
        if self.file_config() != self.config:
            raise ValueError('固定配置已改变，请重启准备进程以加载新版本')
        binding_paths = [self.config_file, Path(self.config['workplane']), Path(self.config['calibration']), Path(self.config['heights'])]
        binding = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in binding_paths}
        out = self.app/'reports'/('live_task_' + time.strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6])
        out.mkdir()
        import copy
        run_config = copy.deepcopy(self.config)
        if wrist_nut_trial is not None:
            run_config['wrist_nut_trial']=copy.deepcopy(wrist_nut_trial)
        if chain is not None:
            if source_nut_uv_px is None:raise ValueError('A continuous chain is an explicit-nut session')
            run_config['chain']=copy.deepcopy(chain)
        # Capture the configuration; a config edit cannot alter a prepared job.
        write(out/'workflow_config.json', run_config)
        write(out/'profile.json', self.config['profile'])
        write(out/'destinations.json', self.config['destinations'])
        slot=destination_slot_index
        if wrist_nut_trial is not None and 'destination_slot_index' in wrist_nut_trial:
            if slot is not None:raise ValueError('Duplicate destination selection')
            slot=wrist_nut_trial['destination_slot_index']
        if slot is not None:
            if screws+nuts!=1:raise ValueError('Explicit slot selection is single-part only')
            destinations=copy.deepcopy(self.config['destinations'])
            if type(slot) is not int or not 0<=slot<len(destinations['pixels']):raise ValueError('Unknown placement slot')
            destinations['pixels']=[destinations['pixels'][slot]]
            write(out/'destinations.json',destinations)
        self.event('RUN_DIRECTORY', directory=str(out), replay_only=bool(replay))
        try:
            with self.stage(out, 'capture'):
                if captured is not None and not replay:
                    points = Path(captured[0])
                    write(out/'board_capture.json', read(Path(captured[1])/'board_capture.json'))
                elif replay:
                    replay = Path(replay)
                    points = Path(replay_points) if replay_points else Path(read(replay/'input.json')['points'])
                    write(out/'board_capture.json', read(replay/'board_capture.json'))
                else:
                    points = self.snapshot()
                    # For a goal, count first: satisfied/shortage must not even
                    # connect to the arm or start preparing a motion job.
                    if goal is None:self.capture_joints(out)
                snapshot = read(points)
                if goal is not None:
                    from .inventory_goal import inventory, stable_inventory, reconcile
                    observations=[snapshot] if replay else self.inventory_frames()
                    # Saving a snapshot and reading latest.json may yield the
                    # same frame. Keep the saved frame once; inventory_frames
                    # still guarantees three distinct observations.
                    observed=inventory(snapshot,self.config) if replay else stable_inventory(
                        [snapshot]+[f for f in observations if f['frame_number'] != snapshot['frame_number']], self.config)
                    decision=reconcile(goal,observed)
                    write(out/'goal_plan.json',decision)
                    write(out/'goal_before.json',observed)
                    write(out/'inventory_frames.json',observations)
                    self.event('GOAL_DECISION',directory=str(out),decision=decision)
                    if decision['status']!='NEEDS_TRANSFER':return dict(directory=str(out),**decision,motor_commands_sent=0)
                    source=decision['source'];destination=decision['destination']
                    selected=sorted((o for o in observed['trays'][source] if o['part_class']=='screw'),key=lambda o:(-o['confidence'],o['detection_index']))[:decision['moves']]
                    screws,nuts=decision['moves'],0
                    run_config['goal_direction']=source+'->'+destination
                    run_config['destination_floor']=self.config['trays'][destination]['plane']
                    write(out/'workflow_config.json',run_config)
                    if not replay:self.capture_joints(out)
                else:
                    selected = select(snapshot, self.config, screws, nuts)
                    if source_nut_uv_px is not None:selected=select_reviewed_part(snapshot,self.config,source_nut_uv_px,source_part_class)
                    if wrist_nut_trial is not None and 'source_uv_px' in wrist_nut_trial:
                        selected=select_reviewed_nut(snapshot,self.config,wrist_nut_trial['source_uv_px'])
                write(out/'input.json', dict(points=str(points)))
                write(out/'scene_review.json', dict(accepted_detection_indices=[o['detection_index'] for o in selected],
                    allow_plane_extrapolation_indices=[o['detection_index'] for o in selected],
                    source_roi_uv=self.config['source_roi_uv'], excluded_robot_tip_xyxy=self.config['excluded_robot_tip_xyxy'],
                    plane_extrapolation_is_unverified=True, box_wall_check=False))
            with self.stage(out, 'surface_geometry'):
                command = [str(self.app/'.venv-camera/bin/python'), str(self.app/'src/preview_workplane_points.py'),
                    '--points', str(points), '--workplane', self.config['workplane'], '--heights', self.config['heights'],
                    '--calibration', self.config['calibration'], '--out', str(out/'target_geometry')]
                if goal is not None:
                    command=[str(self.app/'.venv-camera/bin/python'),'-m','workcell.tray_geometry','--out',str(out),'--config',str(out/'workflow_config.json')]
                elif self.config.get('reverse_small_nut'):
                    command=[str(self.app/'.venv-camera/bin/python'),'-m','workcell.reverse_transfer','--out',str(out),'--config',str(out/'workflow_config.json')]
                self.run_logged(command, out/'surface_geometry.log',cwd=self.root)
            with self.stage(out, 'planning'):
                from motion_runtime import MotionRuntime
                from .live_planner import prepare
                if self.runtime is None:
                    self.runtime = MotionRuntime()
                with self.runtime_lock, (out/'planning.log').open('w') as log, contextlib.redirect_stdout(log):
                    prepare(out, run_config, self.runtime)
                if replay:
                    job = read(out/'job_pending_geometry.json')
                    job['hardware_start_verified'] = False
                    write(out/'job_pending_geometry.json', job)
            def finish():
                try:
                    with self.stage(out, 'self_table_part_geometry'):
                        command = [str(Path.home()/'.local/share/aipc-dev/aipc.sh'), 'sim', 'python', '-u', '-m',
                                   'workcell.live_geometry', '--out', str(out), '--config', str(out/'workflow_config.json')]
                        self.run_logged(command, out/'geometry.log', cwd=self.root)
                    with self.stage(out, 'bundle_verification'):
                        from continuous_transfer import export_bundle
                        from joint_trajectory import JointPlanner
                        sha = export_bundle(read(out/'job.json'), JointPlanner(out/'profile.json'), out/'prepared_bundle.json')
                        write(out/'bundle_review.json', dict(bundle_sha256=sha))
                    if binding != {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in binding_paths}:
                        raise ValueError('配置在准备期间改变，请重新准备')
                    meta = dict(status='PREPARED_REPLAY_ONLY' if replay else 'PREPARED_NOT_EXECUTED', replay_only=bool(replay),
                                screws=screws, nuts=nuts, created_monotonic_s=time.monotonic(),
                                prepared_total_s=time.perf_counter()-started, source_objects=scene_objects(snapshot, self.config),
                                input_bindings=binding, motor_commands_sent=0)
                    if goal is not None:
                        meta.update(goal=goal,goal_direction=run_config['goal_direction'],inventory_before=observed,source_objects=observed['trays']['A']+observed['trays']['B'])
                    write(out/'prepared.json', meta)
                    self.event('PREPARED', directory=str(out), **{k:v for k,v in meta.items() if k not in ('source_objects','input_bindings','inventory_before')})
                    return dict(directory=str(out), **meta)
                except Exception as error:
                    write(out/'failure.json', dict(error=str(error), stage_timings=read(out/'timings.json')))
                    raise
            if defer_geometry:
                return dict(directory=str(out), finish=finish)
            return finish()
        except Exception as error:
            write(out/'failure.json', dict(error=str(error), stage_timings=read(out/'timings.json')))
            raise

    @staticmethod
    def run_logged(command, path, cwd=None):
        with Path(path).open('w') as log:
            p = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, cwd=cwd)
        if p.returncode:
            raise RuntimeError(str(path) + ': ' + Path(path).read_text()[-1800:])

    def install_release(self):
        from continuous_transfer import source_hashes, DEPENDENCIES
        import importlib
        hashes = source_hashes()
        release = digest(hashes)
        directory = self.config['board_root'] + '/releases/' + release
        probe = self.remote('''import json,sys,pathlib,hashlib
x=json.load(sys.stdin);p=pathlib.Path(x['directory'])
ok=p.is_dir() and all((p/(n+'.py')).is_file() and hashlib.sha256((p/(n+'.py')).read_bytes()).hexdigest()==h for n,h in x['hashes'].items())
print(json.dumps(dict(ready=ok)))''', dict(directory=directory, hashes=hashes))
        if not probe['ready']:
            files = {n+'.py': Path(importlib.import_module(n).__file__).read_text() for n in DEPENDENCIES}
            result = self.remote('''import json,sys,pathlib,hashlib,os,uuid,shutil
x=json.load(sys.stdin);p=pathlib.Path(x['directory']);p.parent.mkdir(parents=True,exist_ok=True)
if p.exists():raise RuntimeError('Immutable release exists but hash differs')
t=p.with_name(p.name+'.tmp_'+uuid.uuid4().hex);t.mkdir()
try:
 for n,content in x['files'].items():
  f=t/n;f.write_text(content)
  if hashlib.sha256(f.read_bytes()).hexdigest()!=x['hashes'][n[:-3]]:raise RuntimeError('source hash mismatch')
 (t/'libkv260_mmio.so').symlink_to('/home/ubuntu/kv260_can_deploy/libkv260_mmio.so')
 t.rename(p)
finally:
 if t.exists():shutil.rmtree(t)
print(json.dumps(dict(installed=True)))''', dict(directory=directory, hashes=hashes, files=files))
        self.release = directory
        return dict(release=directory, source_uploaded=not probe['ready'], fpga_reloaded=False)

    def execute(self, directory, attended=False, *, single_reverse_trial=False, commissioning_goal=None):
        if attended is not True:
            raise ValueError('Explicit attended execute action required')
        out = Path(directory).resolve()
        if out.parent != self.app/'reports' or not out.name.startswith('live_task_'):
            raise ValueError('Unknown workflow directory')
        meta = read(out/'prepared.json')
        if meta['replay_only']:
            raise ValueError('Replay preparation cannot execute')
        if type(single_reverse_trial) is not bool:
            raise ValueError('Reverse trial requires an explicit boolean')
        if single_reverse_trial:
            # Explicit attended commissioning API, not exposed by the UI/server.
            # Keep general goal execution closed until physical validation.
            from .inventory_goal import reconcile
            decision = reconcile(meta['goal'], meta['inventory_before'])
            if (decision['status'] != 'NEEDS_TRANSFER' or decision['moves'] != 1 or
                (decision['source'], decision['destination']) != ('B', 'A') or
                meta.get('goal_direction') != 'B->A' or
                meta.get('screws') != 1 or meta.get('nuts') != 0):
                raise ValueError('Commissioning is limited to one B->A screw')
        if commissioning_goal is not None:
            # Bind attended commissioning to the exact freshly authorized goal.
            # This is a maintenance API; the general UI/server stays gated.
            from .inventory_goal import validate, reconcile
            expected = validate(commissioning_goal)
            if expected != meta.get('goal'):
                raise ValueError('Commissioning goal differs from the prepared goal')
            decision = reconcile(expected, meta['inventory_before'])
            if (decision['status'] != 'NEEDS_TRANSFER' or not 1 <= decision['moves'] <= 5 or
                meta.get('goal_direction') != decision['source']+'->'+decision['destination'] or
                meta.get('screws') != decision['moves'] or meta.get('nuts') != 0):
                raise ValueError('Commissioning plan differs from the authorized goal')
        if meta.get('goal') and not self.config.get('inventory_goal_execution_enabled',False) and not single_reverse_trial and commissioning_goal is None:
            raise ValueError('目标数量任务已可预览和准备；双向实物路径尚待现场验证，当前不启动电机')
        if (out/'owner.json').exists():
            raise ValueError('This task was already launched; query its state')
        if not 0 <= time.monotonic()-meta['created_monotonic_s'] <= 120:
            raise ValueError('Prepared task expired; prepare from a fresh scene')
        for p, sha in meta['input_bindings'].items():
            if hashlib.sha256(Path(p).read_bytes()).hexdigest() != sha:
                raise ValueError('Configuration changed; prepare again')
        if single_reverse_trial:
            write(out/'commissioning.json', dict(scope='SINGLE_B_TO_A_SCREW', attended=True,
                  general_goal_execution_enabled=self.config.get('inventory_goal_execution_enabled',False)))
        elif commissioning_goal is not None:
            write(out/'commissioning.json', dict(scope='EXACT_ATTENDED_INVENTORY_GOAL', attended=True,
                  goal=expected, decision=decision,
                  general_goal_execution_enabled=self.config.get('inventory_goal_execution_enabled',False)))
        with self.stage(out, 'release_reuse'):
            release = self.install_release()
            write(out/'release.json', release)
        remote = self.config['board_root'] + '/jobs/' + out.name
        with self.stage(out, 'job_upload'):
            files = {n: (out/n).read_text() for n in ('job.json', 'profile.json', 'prepared_bundle.json')}
            self.remote('''import json,sys,pathlib
x=json.load(sys.stdin);p=pathlib.Path(x['directory']);p.mkdir(parents=True,exist_ok=False)
for n,t in x['files'].items():(p/n).write_text(t)
print(json.dumps(dict(uploaded=True)))''', dict(directory=remote, files=files))
        with self.stage(out, 'fresh_scene'):
            _, folder = self.camera()
            current = read(folder/'latest.json')
            if not current['synchronized'] or not 0 <= time.monotonic()-current['host_acquired_monotonic_s'] < 2:
                raise ValueError('Camera frame stale')
            if meta.get('goal'):
                from .inventory_goal import inventory
                observed=inventory(current,self.config)
                current_objects=observed['trays']['A']+observed['trays']['B']
            else:current_objects=scene_objects(current,self.config)
            if not unchanged(meta['source_objects'], current_objects):
                raise ValueError('零件摆位已变化，请重新准备')
            write(out/'prelaunch_scene.json', current)
        with self.stage(out, 'launch'):
            code = '''import json,sys,pathlib,fcntl,os,subprocess
x=json.load(sys.stdin);p=pathlib.Path(x['directory']);release=pathlib.Path(x['release'])
sys.path.insert(0,str(release));from canctl import CanDevice
with open('/tmp/kv260_gripper_commission.lock','a') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);d=CanDevice()
 try:
  if any(d.read(a) for a in [0x404,0x418,0x5a0,0x5a8,0x800,0xb40]):raise RuntimeError('Controller not ready')
 finally:d.close()
 # An exclusive marker prevents retrying a lost SSH response from launching twice.
 (p/'launch_claim').open('x').close()
 cmd=[sys.executable,'-u',str(release/'continuous_transfer.py'),'--job',str(p/'job.json'),'--profile',str(p/'profile.json'),'--out',str(p/'execution'),'--bundle',str(p/'prepared_bundle.json'),'--bundle-sha256',x['sha'],'--execute-attended']
 with (p/'execution.log').open('x') as log:
  child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True,env=dict(os.environ,PYTHONPATH=str(release)+':/home/ubuntu/kv260_motion_runtime/src:/home/ubuntu/kv260_can_deploy'))
 owner=dict(pid=child.pid,remote=str(p),release=str(release));(p/'owner.json').write_text(json.dumps(owner))
 print(json.dumps(owner))
'''
            owner = self.remote(code, dict(directory=remote, release=self.release, sha=read(out/'bundle_review.json')['bundle_sha256']))
            write(out/'owner.json', owner)
            write(self.app/'reports/live_active_task.json', dict(directory=str(out), owner=owner))
        self.event('STARTED', directory=str(out), **owner)
        return owner

    def poll(self, directory):
        out = Path(directory)
        owner = read(out/'owner.json')
        result = self.remote('''import json,sys,pathlib
x=json.load(sys.stdin);p=pathlib.Path(x['remote']);proc=pathlib.Path('/proc/'+str(x['pid'])+'/cmdline')
cmd=proc.read_bytes() if proc.exists() else b''
r=dict(alive=str(p/'execution').encode() in cmd and str(pathlib.Path(x['release'])/'continuous_transfer.py').encode() in cmd,log_tail=(p/'execution.log').read_text()[-3500:],snapshots=[json.loads(f.read_text()) for f in (p/'execution').glob('snapshot_*.json')])
for name,key in [('operator_wait_state.json','operator_wait'),('runtime_summary.json','summary')]:
 f=p/'execution'/name
 if f.exists():
  try:r[key]=json.loads(f.read_text())
  except ValueError:pass
print(json.dumps(r))''', owner)
        write(out/'latest_remote_state.json', result)
        # Camera evidence is requested asynchronously and never gates movement.
        seen_path = out/'observed_waypoints.json'
        seen = read(seen_path) if seen_path.exists() else []
        fresh = [row for row in result.get('snapshots', []) if row['index'] not in seen]
        if fresh:
            try:
                active, folder = self.camera()
                frame = read(folder/'latest.json')['frame_number']
                os.kill(active['pid'], signal.SIGUSR1)
                with (out/'camera_requests.jsonl').open('a') as log:
                    log.write(json.dumps(dict(after_frame=frame, waypoints=fresh, folder=str(folder), hardware_synchronized=False))+'\n')
                write(seen_path, seen+[row['index'] for row in fresh])
            except (OSError, ValueError, KeyError) as error:
                result['camera_evidence_error'] = str(error)
        if result.get('summary'):
            pointer = self.app/'reports/live_active_task.json'
            if pointer.exists() and read(pointer)['directory'] == str(out):
                pointer.unlink()
        return result

    def disable_supported(self, directory, confirmation):
        if confirmation != '已承托，可以失能':
            raise ValueError('本轮必须明确确认：已承托，可以失能')
        out = Path(directory)
        owner = read(out/'owner.json')
        result = self.remote('''import json,sys,pathlib
x=json.load(sys.stdin);p=pathlib.Path(x['remote']);cmd=pathlib.Path('/proc/'+str(x['pid'])+'/cmdline').read_bytes()
if str(pathlib.Path(x['release'])/'continuous_transfer.py').encode() not in cmd or str(p/'execution').encode() not in cmd:raise RuntimeError('owner mismatch')
if not (p/'execution/operator_wait_state.json').exists():raise RuntimeError('Not awaiting approval')
(p/'execution/SUPPORTED_DISABLE').write_text('Fresh user confirmation: supported, disable authorized.\\n')
print(json.dumps(dict(requested=True)))''', owner)
        write(out/'disable_approval.json', dict(confirmation=confirmation, owner=owner, **result))
        return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--app-root', required=True)
    p.add_argument('--serve', action='store_true')
    p.add_argument('--replay')
    p.add_argument('--screws', type=int, default=1)
    p.add_argument('--nuts', type=int, default=0)
    a = p.parse_args()
    session = Session(a.app_root, lambda value: print(value, flush=True))
    if not a.serve:
        session.prepare(a.screws, a.nuts, a.replay)
        return
    session.event('WORKFLOW_READY', motor_commands_sent=0)
    for line in sys.stdin:
        request_id = None
        try:
            request = json.loads(line)
            request_id = request.get('request_id')
            op = request['op']
            if op == 'shutdown':
                break
            if op == 'prepare':
                result = session.prepare(request.get('screws', 1), request.get('nuts', 0), request.get('replay'), request.get('goal'))
            elif op == 'preview_goal':
                result=session.preview_goal(request['goal'])
            elif op == 'verify_goal':
                result=session.verify_goal(request['directory'])
            elif op == 'execute':
                result = session.execute(request['directory'], request.get('attended', False))
            elif op == 'poll':
                result = session.poll(request['directory'])
            elif op == 'supported_disable':
                result = session.disable_supported(request['directory'], request.get('confirmation'))
            elif op == 'install_release':
                result = session.install_release()
            elif op == 'board_status':
                result = session.board_status()
            else:
                raise ValueError('Unknown operation')
            session.event('WORKFLOW_RESULT', request_id=request_id, op=op, result=result)
        except Exception as error:
            session.event('WORKFLOW_ERROR', request_id=request_id, error=str(error))


if __name__ == '__main__':
    main()
