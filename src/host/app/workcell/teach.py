"""Teach mode for the console: zero-gravity hand guiding on the board, pose and trajectory recording.

The motion is the board's reviewed `freedrive.py` (six joints Kp=0, measured-pose gravity, damping, torque caps, PL lease and
feedback-freshness guards, operator-ended). This module only starts it, ends it with its own `finish_supported` control command, reads its
live file (10 Hz) and stores what the operator records. It sends no motion command and replays nothing.
"""
import json
import subprocess
import threading
import time
import uuid
from pathlib import Path

BOARD = '/home/ubuntu/aipc_teach'
FREEDRIVE_DIR = '/home/ubuntu/freedrive_20261029'
BOARD_PYTHON = '/home/ubuntu/kv260_motion_runtime/.venv/bin/python'
CONFIRMATION = '已托住，开始零重力'
# The [f] keeps pgrep -f from matching the remote shell whose own command line contains the pattern.
LIVE_OWNERS = ("for o in /home/ubuntu/aipc_wrist_servo/*/owner.json; do p=$(sed -n 's/.*\"pid\": *\\([0-9][0-9]*\\).*/\\1/p' \"$o\"); "
               "[ -n \"$p\" ] && kill -0 \"$p\" 2>/dev/null && basename \"$(dirname \"$o\")\"; done; true")


# After an arm power cycle the motors report their factory run mode (0), and freedrive refuses to enter MIT without the CSP baseline (mode 5).
# control_benchmark.initialize_disabled_csp is the reviewed normalization for exactly that: verified mode writes while the drives and the
# controller are inactive, no target, enable, zero or Flash command. It only writes a motor whose mode is not 5 yet.
BASELINE = '''cd %(dir)s && mkdir -p %(board)s && [ -f %(board)s/profile.json ] || cp $(ls -t /home/ubuntu/aipc_wrist_servo/*/profile.json | head -1) %(board)s/profile.json
PYTHONPATH=$PWD:/home/ubuntu/kv260_can_deploy timeout 90 %(python)s - <<EOF
import json
from control_benchmark import initialize_disabled_csp
from joint_trajectory import JointPlanner
p = JointPlanner("%(board)s/profile.json")
lo, hi = [], []
for a, b, d, o in zip(p.lower, p.upper, p.direction, p.offset):
    lower, upper = sorted((a * d + o, b * d + o)); lo.append(lower); hi.append(upper)
r = initialize_disabled_csp(p, dict(motor_lower_rad=lo, motor_upper_rad=hi))
print("BASELINE", json.dumps(dict(before=r["mode_before"], after=r["mode_after"])))
EOF'''


class TeachError(ValueError):
    pass


def joint_from_motor(motor, directions, offsets):
    """Motor-side radians to joint-side radians (the profile's direction and offset)."""
    return [(m - o) / d for m, d, o in zip(motor, directions, offsets)]


class Teach:
    def __init__(self, app, ssh_helper, run=subprocess.run, clock=time.time, popen=subprocess.Popen):
        self.app, self.helper, self._run, self.clock, self._popen = Path(app), ssh_helper, run, clock, popen
        self.stream, self.alive, self.fault = None, False, None
        self.lock = threading.Lock()
        self.state, self.run_id, self.started, self.error = 'IDLE', None, None, None
        self.poses, self.marks, self.last, self.profile = [], [], None, None
        self.trajectories, self.final = [], None

    # ---- board access (one ssh call each)
    def remote(self, command, timeout=25):
        out = self._run(['bash', self.helper, command], capture_output=True, text=True, timeout=timeout)
        return out.stdout

    def precheck(self):
        owners = [l.strip() for l in self.remote(LIVE_OWNERS).splitlines() if l.strip()]
        running = self.remote('pgrep -f "[f]reedrive.py --profile" >/dev/null && echo yes || echo no').strip() == 'yes'
        status = json.loads(self.remote('cd ~/kv260_can_deploy && python3 canctl.py status'))
        return dict(live_owners=owners, freedrive_running=running, control=status['control'], fault=status['fault'])

    def start(self, confirmation, busy=False):
        if confirmation != CONFIRMATION:
            raise TeachError('需要操作员确认：必须用手托住机械臂')
        if busy:
            raise TeachError('抓取任务正在执行，不能同时进入零重力')
        with self.lock:
            if self.state == 'ACTIVE':
                raise TeachError('零重力已经在运行')
            pre = self.precheck()
            if pre['live_owners'] or pre['freedrive_running'] or pre['control'] != 0 or pre['fault'] != 0:
                raise TeachError(f'板子不空闲，未启动零重力：{pre}')
            text = self.remote(BASELINE % dict(dir=FREEDRIVE_DIR, board=BOARD, python=BOARD_PYTHON), timeout=120)
            line = next((l for l in text.splitlines() if l.startswith('BASELINE ')), None)
            if line is None or any(v != 5 for v in json.loads(line[9:])['after'].values()):
                raise TeachError(f'电机没有进入 CSP 基线模式，未启动零重力：{text[-300:]}')
            run = time.strftime('teach_%Y%m%d_%H%M%S')
            root = f'{BOARD}/{run}'
            script = (f'set -e; mkdir -p {root}; [ -f {BOARD}/profile.json ] || cp $(ls -t /home/ubuntu/aipc_wrist_servo/*/profile.json | head -1) {BOARD}/profile.json; '
                      f'cd {FREEDRIVE_DIR}; PYTHONPATH=$PWD:/home/ubuntu/kv260_can_deploy nohup setsid taskset -c 2,3 {BOARD_PYTHON} -u freedrive.py --profile {BOARD}/profile.json '
                      f'--output {root}/out.json --control {root}/control.json --live {root}/live.json --hands-supporting-attended '
                      f'> {root}/log.txt 2>&1 < /dev/null & echo $! > {root}/pid; echo launched')
            if 'launched' not in self.remote(script):
                raise TeachError('零重力没有启动')
            self.run_id, self.started, self.error, self.final, self.fault = run, self.clock(), None, None, None
            self.poses, self.marks, self.trajectories, self.last = [], [], [], None
            self.state = 'STARTING'
            profile = json.loads(self.remote(f'cat {BOARD}/profile.json'))
            self.profile = (profile['motor_directions'], profile['motor_offsets_rad'])
        self.open_stream()
        deadline = self.clock() + 25
        while self.clock() < deadline:
            time.sleep(.3)
            snap = self.poll()
            if self.state == 'ACTIVE':
                return snap
            if self.state in ('FAILED', 'ENDED'):
                break
        raise TeachError(f'零重力没有在 25 秒内进入拖动状态：{self.error or self.tail()}')

    def tail(self):
        return self.remote(f'tail -n 5 {BOARD}/{self.run_id}/log.txt')[-400:]

    def open_stream(self):
        """One long-lived ssh that follows the live file (about 10 Hz) and reports the end of the board process. The earlier design polled with
        a new ssh login 2-3 times a second; on the board's four A53 cores that load coincided with 'stale feedback' stops at enable (2026-11-01)."""
        root = f'{BOARD}/{self.run_id}'
        script = (f'tail -n 0 -F {root}/live.json 2>/dev/null & TP=$!; '
                  f'(tail -n 0 -F {root}/log.txt 2>/dev/null | grep --line-buffered -E "RETAINING_CONTROL|HARDWARE_OUTPUT_STOPPED" | '
                  f'while read l; do echo __FAULT__; done) & FP=$!; '
                  f'while kill -0 $(cat {root}/pid 2>/dev/null) 2>/dev/null; do sleep 1; done; echo __ENDED__; kill $TP $FP 2>/dev/null')
        self.alive = True
        self.stream = self._popen(['bash', self.helper, script], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        threading.Thread(target=self._read_stream, args=(self.stream,), daemon=True).start()

    def _read_stream(self, stream):
        ended = False
        for line in stream.stdout:
            line = line.strip()
            if line == '__ENDED__':
                ended = True
                break
            if line == '__FAULT__':
                with self.lock:
                    self.fault = '板端零重力已停止输出（多半是 CAN 反馈中断）：请托住机械臂，点“结束”'
                continue
            if not line.startswith('{'):
                continue
            try:
                live = json.loads(line)
                motor = live['actual_motor_rad']
                joints = joint_from_motor(motor, *self.profile)
            except (ValueError, KeyError):
                continue
            with self.lock:
                self.last = dict(phase=live['phase'], elapsed_s=live['elapsed_s'], motor_rad=motor, joint_rad=joints,
                                 joint_deg=[round(x * 57.29578, 1) for x in joints], gripper_rad=motor[6],
                                 torque_saturated=live.get('torque_saturated'), finish_pending=live.get('finish_pending'))
                if self.state == 'STARTING' and live['phase'] == 'DRAG':
                    self.state = 'ACTIVE'
        with self.lock:
            self.alive = False
            if self.state in ('STARTING', 'ACTIVE', 'FINISHING'):
                if not ended:   # the ssh link ended, not the board process: it may still be running
                    self.state, self.error = 'LOST', '与板子的连接中断，零重力可能仍在运行：请托住机械臂并检查板子'
                else:
                    self.state = 'ENDED' if self.last or self.state == 'FINISHING' else 'FAILED'
                    if self.state == 'FAILED':
                        self.error = self.error or '零重力进程已退出'

    def poll(self):
        """Latest live sample; no board access (the stream thread keeps it current)."""
        with self.lock:
            return dict(self.snapshot_locked())

    def snapshot_locked(self):
        return dict(state=self.state, run=self.run_id, error=self.error, fault=self.fault, live=self.last, poses=self.poses, marks=self.marks,
                    trajectories=[dict(name=t['name'], samples=len(t['samples']), seconds=t['seconds']) for t in self.trajectories], final=self.final)

    def snapshot(self):
        return self.poll()

    # ---- recording
    def record_pose(self, name):
        name = str(name).strip()
        if not name or len(name) > 30:
            raise TeachError('位置名称为空或过长')
        snap = self.poll()
        if self.state != 'ACTIVE' or not snap.get('live'):
            raise TeachError('零重力没有运行，无法记录位置')
        live = snap['live']
        with self.lock:
            self.poses = [p for p in self.poses if p['name'] != name] + [dict(name=name, elapsed_s=live['elapsed_s'], motor_rad=live['motor_rad'],
                                                                              joint_rad=live['joint_rad'], joint_deg=live['joint_deg'])]
        self.save()
        return self.poses[-1]

    def mark(self, action, name=''):
        snap = self.poll()
        if self.state != 'ACTIVE' or not snap.get('live'):
            raise TeachError('零重力没有运行')
        t = snap['live']['elapsed_s']
        with self.lock:
            if action == 'start':
                if self.marks and 'end' not in self.marks[-1]:
                    raise TeachError('已经在录制轨迹')
                self.marks.append(dict(name=str(name).strip() or f'轨迹{len(self.marks) + 1}', start=t))
            elif action == 'stop':
                if not self.marks or 'end' in self.marks[-1]:
                    raise TeachError('没有正在录制的轨迹')
                self.marks[-1]['end'] = t
            else:
                raise TeachError('未知的轨迹操作')
        return self.marks[-1]

    # ---- ending
    def finish(self, timeout=30):
        """End zero gravity at once. The operator's click on "托稳，结束零重力" is the support confirmation: write freedrive's own
        SUPPORTED_DISABLE marker, then ask the process to stop (SIGTERM by its PID). Its failure path keeps gravity compensation until it
        sees the marker, disables all seven axes, confirms every reset and exits (about a second). Nothing waits for the arm to hold still."""
        with self.lock:
            if self.state not in ('ACTIVE', 'FINISHING'):
                raise TeachError('零重力没有运行')
            if self.marks and 'end' not in self.marks[-1]:
                self.marks[-1]['end'] = self.last['elapsed_s'] if self.last else self.marks[-1]['start']
            self.state = 'FINISHING'
            root = f'{BOARD}/{self.run_id}'
            # In its own fault hold (log shows RETAINING_CONTROL) the process finishes by itself once it sees the marker: a SIGTERM there would
            # interrupt its stop sequence (2026-11-01 16:11: PL fault left at 1, no record written).
            answer = self.remote(f"touch {root}/SUPPORTED_DISABLE; if grep -q RETAINING_CONTROL {root}/log.txt; then echo signalled; else "
                                 f"pid=$(pgrep -f '[f]reedrive.py --profile .*--output {root}/out.json' | head -1); "
                                 f"if [ -n \"$pid\" ]; then kill -TERM $pid && echo signalled; else echo no_process; fi; fi")
        if 'signalled' not in answer and 'no_process' not in answer:
            raise TeachError('没有能向板上的零重力进程发出结束请求')
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            time.sleep(.5)
            self.poll()
            if self.state in ('ENDED', 'FAILED'):
                return self.collect()
            if self.state == 'LOST':
                raise TeachError(self.error)
        raise TeachError('零重力还没有结束：请托住机械臂，并检查板子')

    def collect(self):
        root = f'{BOARD}/{self.run_id}'
        text = self.remote("python3 -c \"import json;r=json.load(open('%s/out.json'));print(json.dumps(dict(status=r.get('status'),error=r.get('error'),"
                           "reset=r.get('reset_confirmed'),stop_errors=r.get('stop_errors'),controller=r.get('final_controller'),"
                           "samples=[[s['elapsed_s'],s['actual_motor_rad']] for s in r.get('samples',[])])))\"" % root, timeout=60)
        out = json.loads(text)
        samples = out.pop('samples')
        with self.lock:
            for m in self.marks:
                if 'end' in m:
                    rows = [s for s in samples if m['start'] <= s[0] <= m['end']]
                    self.trajectories.append(dict(name=m['name'], seconds=round(m['end'] - m['start'], 2),
                                                  samples=[dict(t=round(s[0] - m['start'], 2), motor_rad=s[1],
                                                                joint_rad=joint_from_motor(s[1], *self.profile)) for s in rows]))
            ctrl = out.get('controller') or {}
            stopped = 'operator interrupt' in str(out.get('error'))        # the console's own immediate stop is a SIGTERM
            self.final = dict(status='OPERATOR_STOP' if stopped else out.get('status'), disabled=(sorted(out.get('reset') or []) == list(range(1, 8)) and not out.get('stop_errors')
                                                                     and ctrl.get('control') == 0 and ctrl.get('fault') == 0), error=None if stopped else out.get('error'))
        self.save()
        return self.final

    def save(self):
        folder = self.app / 'reports/teach' / (self.run_id or 'none')
        folder.mkdir(parents=True, exist_ok=True)
        (folder / 'teach.json').write_text(json.dumps(dict(run=self.run_id, poses=self.poses, marks=self.marks, trajectories=self.trajectories,
                                                          final=self.final, directions=self.profile and self.profile[0],
                                                          offsets=self.profile and self.profile[1]), ensure_ascii=False, indent=1))
