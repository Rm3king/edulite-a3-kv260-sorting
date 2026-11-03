"""Idempotent camera startup; no robot connection or motor command."""
import fcntl
import json
import os
from pathlib import Path
import subprocess
import time
from .live_session import read, write


def ensure_camera(app):
    app = Path(app).resolve()
    pointer = app/'reports/rgbd_surface_preview_active.json'
    with (app/'reports/live_camera_start.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try:
            active = read(pointer)
            cmd = Path(f"/proc/{active['pid']}/cmdline").read_bytes()
            if b'detect_parts_rgbd_live.py' in cmd and active['report_dir'].encode() in cmd:
                # Do not start a competing camera pipeline even if this one stalls.
                try:frame = read(Path(active['report_dir'])/'latest.json')
                except (OSError, ValueError) as error:
                    raise RuntimeError('Existing camera process has no readable frame yet') from error
                age = time.monotonic()-frame['host_acquired_monotonic_s']
                if not 0 <= age < 3:
                    raise RuntimeError('Existing camera process has stale frames; inspect it instead of opening another pipeline')
                return dict(camera_reused=True, pid=active['pid'], frame_age_s=age)
        except (FileNotFoundError, KeyError, json.JSONDecodeError):
            pass
        output = app/'reports'/('parts_live_'+time.strftime('%Y%m%d_%H%M%S'))
        log_path = output.with_suffix('.log')
        command = [str(Path.home()/'.local/share/aipc-dev/aipc.sh'), 'gpu', str(app/'.venv-yolo/bin/python'), '-u',
                   str(app/'src/detect_parts_rgbd_live.py'), '--model-plan', str(app/'configs/parts_v2_20261028_model.json'),
                   '--calibration', str(read(app/'integration/config/live_workflow.json')['calibration']),
                   '--run', '--out', str(output), '--backend', read(app/'integration/config/live_workflow.json').get('detector_backend', 'gpu')]
        with log_path.open('x') as log:
            child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
        active = dict(pid=child.pid, report_dir=str(output), log=str(log_path), command=command,
                      status='STARTING', display=os.environ.get('DISPLAY'))
        write(pointer, active)
        deadline = time.monotonic()+45
        while time.monotonic() < deadline:
            if child.poll() is not None:
                raise RuntimeError('Camera exited: '+str(log_path))
            if (output/'latest.json').exists():
                active['status']='RUNNING_RGBD_COORDINATE_PREVIEW';write(pointer,active)
                return dict(camera_reused=False, pid=child.pid)
            time.sleep(.2)
        raise TimeoutError('Camera starting slowly; process retained, log: '+str(log_path))


if __name__ == '__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--app-root',required=True);p.add_argument('--board',action='store_true');a=p.parse_args()
    print(json.dumps(ensure_camera(a.app_root)))
    if a.board:
        from .live_session import Session
        try:print(json.dumps(Session(a.app_root).ensure_board()))
        except Exception as error:
            # The UI and local Qwen remain usable while the board is offline.
            print(json.dumps(dict(board_ready=False,error=str(error),motor_commands_sent=0)))
