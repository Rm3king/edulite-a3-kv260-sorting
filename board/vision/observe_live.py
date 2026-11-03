#!/usr/bin/python3
"""Live observe-only run of the board vision loop: correction channel disabled (stream stopped, arm powered off), no register written by this script."""
import json, os, subprocess, sys, time
from pathlib import Path
B = Path.home() / 'kv260_can_deploy'
C, S = Path('/dev/shm/wvl_command.json'), Path('/dev/shm/wvl_state.json')
def put(d):
    t = C.with_suffix('.tmp'); t.write_text(json.dumps(d)); os.replace(t, C)
for f in (C, S):
    f.unlink(missing_ok=True)
p = subprocess.Popen(['/usr/bin/python3', str(B / 'wrist_vision_loop.py'), 'serve'], cwd=str(B))
try:
    for _ in range(200):
        time.sleep(.1)
        if S.exists() and json.loads(S.read_text()).get('status') == 'READY':
            break
    print('READY', S.read_text())
    J = [[4973, -306], [-422, -4540]]; A = [[0, 0]] * 6
    for k, (kind, size, radius) in enumerate([('screw', 'm12', 220.), ('nut', 'm12', 160.), ('screw', 'm6', 220.), ('nut', 'm6', 110.)]):
        put(dict(id='obs%d' % k, mode='run', params=dict(part_class=kind, part_size=size, target_px=[320, 200], J_pixels_per_m=J, A_motor_rad_per_tool_m=A,
                                                          coarse_center_px=[320, 200], radius_px=radius, gain=.4, max_correction_m=.015, tolerance_px=6.)))
        time.sleep(6)
        st = json.loads(S.read_text())
        print(kind, size, {k2: st.get(k2) for k2 in ('status', 'frames', 'lost', 'error_px', 'center_px', 'latency_ms', 'cycle_ms', 'frame_ms', 'locate_ms', 'last_error', 'correction_channel')})
        put(dict(id='stop%d' % k, mode='stop')); time.sleep(.5)
    put(dict(id='x', mode='exit')); p.wait(5)
finally:
    if p.poll() is None:
        p.terminate(); p.wait(5)
