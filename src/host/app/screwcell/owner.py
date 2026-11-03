"""Isolated owner dispatch. Default mock; hardware requires explicit KV mode.

Only physical_owner imports the deployed actuator runtime. MuJoCo and numerical
planning remain separate backends and cannot acquire a hardware execution lease.
"""
import asyncio
import concurrent.futures
import multiprocessing as mp
import queue
import time
import uuid
from pathlib import Path

STAGES = ['AT_APPROACH', 'GRIP_CLOSED', 'LIFTED', 'PLACED', 'MOTION_DONE']

def planner(payload, runtime_root, profile):
    import sys
    sys.path.insert(0, runtime_root)
    from motion_runtime import MotionRuntime, pose_values
    from joint_trajectory import JointPlanner
    from septic_trajectory import movej_c3
    import numpy as np
    import pinocchio as pin
    r = MotionRuntime()
    p = JointPlanner(profile)
    if payload['model_id'] != r.model_id:
        raise ValueError('MODEL_MISMATCH')
    start = payload['planning_start_joint_rad']
    p.positions(start)
    seed = start[:6]
    goals = []
    for key in ('grasp_tcp', 'place_tcp'):
        target = payload[key]
        rot = pin.Quaternion(np.asarray(target['quaternion_xyzw'], dtype=float)).matrix()
        target_pose = target['position_m'] + pin.rpy.matrixToRpy(rot).tolist()
        try:
            q = r.ik(target_pose, seed).tolist()
        except ValueError:
            q = r.ik(target_pose, np.deg2rad([-35, 91, -67, 66, 0, -10]).tolist()).tolist()
        goals.append(q)
        seed = q
    # Numerical joint-path feasibility only: no box/clearance or straight descent claim.
    plans = []
    q = start
    for goal in goals + [start[:6]]:
        end = goal + [start[6]]
        plans += movej_c3(p, q, end)
        q = end
    fk_root = Path(runtime_root).parents[1]/'kv260_fk_batch/source'
    sys.path.insert(0, str(fk_root))
    from client import BatchFK
    device = BatchFK(fk_root/'libfk_batch.so', True)
    audits = []
    try:
        for part in plans:
            audits.append(device.audit(np.asarray(part['coefficients_rad'])[:6], [-1, 1, -1, 1, -1, 1], min(4097, part['ticks']*8+1), 'pl'))
    finally:
        device.close()
    if any(x['status'] or x['violations'] for x in audits):
        raise ValueError('SAMPLED_FK_REGION_REPORT_FAILED')
    return dict(pl_fk_samples=sum(x['samples'] for x in audits), pl_fk_batches=len(audits),
                sampled_model_region_m=[-1, 1, -1, 1, -1, 1],
                backend='kv260-plan-only', model_id=r.model_id,
                goals_joint_rad=goals, segments=len(plans),
                duration_s=sum(x['duration_s'] for x in plans),
                planning_origin='EXPLICIT_OFFLINE_JOINT_FIXTURE_NOT_LIVE_FEEDBACK',
                physical_execution_supported=False, collision_checked=False,
                cartesian_approach_checked=False, motor_commands_sent=0)

def worker(commands, results, mode, runtime_root, profile, tracking='', evidence_root=''):
    if mode == 'mujoco':
        from workcell.sim_owner import worker as simulation_worker
        return simulation_worker(commands,results,runtime_root,evidence_root)
    if mode == 'kv260-motion':
        from .physical_owner import worker as physical_worker
        return physical_worker(commands, results, runtime_root, profile, tracking, evidence_root)
    state = 'IDLE'
    task_id = None
    phase = 0
    next_phase = 0
    lease = 0
    executions = 0
    from .clocks import PLClock
    clock = PLClock()
    def event(name, **data):
        results.put(dict(event=name, time_ns=time.monotonic_ns(), task_id=task_id,
                         backend=mode, simulated=True, **data))
    while True:
        try:
            command = commands.get(timeout=.02)
        except queue.Empty:
            command = None
        if command:
            token, op, payload = command
            try:
                value = {}
                if op == 'SHUTDOWN':
                    results.put(dict(token=token, value={'stopped': True})); return
                if op == 'PREPARE':
                    if mode == 'mock':
                        value = dict(backend='mock', duration_s=.6, segments=5,
                                     physical_execution_supported=False, motor_commands_sent=0)
                    else:
                        value = planner(payload, runtime_root, profile)
                elif op == 'EXECUTE':
                    if mode != 'mock':
                        raise ValueError('PHYSICAL_EXECUTION_UNAVAILABLE_IN_PLAN_ONLY_RELEASE')
                    if state not in ('IDLE', 'MOTION_DONE'):
                        raise ValueError('OWNER_BUSY')
                    task_id = payload['task_id']; state = 'RUNNING'; phase = 0
                    next_phase = time.monotonic()+.12; lease = time.monotonic()+.5
                    executions += 1; event('STARTED')
                elif op == 'HEARTBEAT':
                    lease = time.monotonic()+.5
                elif op == 'PAUSE':
                    if state == 'RUNNING':
                        state = 'HOLDING'; event('HOLDING', reason=payload.get('reason', 'REQUESTED'))
                elif op == 'RESUME':
                    if state != 'HOLDING':
                        raise ValueError('NOT_HOLDING')
                    state = 'RUNNING'; next_phase = time.monotonic()+.12; lease = time.monotonic()+.5
                    event('RESUMED')
                elif op == 'RETURN_SUPPORTED':
                    if mode != 'mock':
                        raise ValueError('PHYSICAL_EXECUTION_UNAVAILABLE_IN_PLAN_ONLY_RELEASE')
                    if state != 'HOLDING':
                        raise ValueError('NOT_HOLDING')
                    state = 'IDLE'; event('RETURNED_SUPPORTED')
                elif op == 'STATUS':
                    value = dict(state=state, task_id=task_id, executions=executions,
                                 physical_execution_supported=False, motor_commands_sent=0)
                    if mode == 'kv260-plan-only':
                        import sys
                        sys.path.insert(0, str(Path(runtime_root).parents[1]/'kv260_can_deploy'))
                        from canctl import CanDevice
                        d = CanDevice()
                        try:
                            before_ns = time.monotonic_ns()
                            snapshot = d.status()
                            after_ns = time.monotonic_ns()
                            value['pl_clock_mapping'] = clock.sample(snapshot['time_us'], before_ns, after_ns)
                            value['pl_read_only'] = dict(status=snapshot, abi=d.read(0x470),
                                motion_mask=d.read(0x5a0), active_motors=d.read(0x5a8),
                                mit_flags=d.read(0xb40), timing_signature=d.read(0x100))
                        finally:
                            d.close()
                else:
                    raise ValueError('UNKNOWN_OWNER_OPERATION')
                results.put(dict(token=token, value=value))
            except Exception as exc:
                results.put(dict(token=token, error=str(exc)))
        now = time.monotonic()
        if state == 'RUNNING' and now > lease:
            state = 'HOLDING'; event('HOLDING', reason='LINK_LOST_MOCK_NO_TORQUE_OUTPUT')
        if state == 'RUNNING' and now >= next_phase:
            state_name = STAGES[phase]; event(state_name)
            phase += 1; next_phase = now+.12
            if phase == len(STAGES):
                state = 'MOTION_DONE'

class Owner:
    def __init__(self, mode='mock', runtime_root='', profile='', tracking='', evidence_root=''):
        ctx = mp.get_context('spawn')
        self.commands = ctx.Queue(64); self.results = ctx.Queue(256)
        self.process = ctx.Process(target=worker, args=(self.commands, self.results, mode, runtime_root, profile, tracking, evidence_root))
        self.process.start()
        self.pending = {}; self.events = asyncio.Queue(256)
        self.reader = asyncio.create_task(self.collect())
    async def collect(self):
        while self.process.is_alive():
            try:
                result = await asyncio.to_thread(self.results.get, True, .1)
            except queue.Empty:
                continue
            if 'event' in result:
                await self.events.put(result)
            else:
                future = self.pending.pop(result['token'], None)
                if future is not None and not future.done():
                    if 'error' in result:
                        future.set_exception(ValueError(result['error']))
                    else:
                        future.set_result(result['value'])
        for future in self.pending.values():
            if not future.done():
                future.set_exception(RuntimeError('OWNER_PROCESS_EXITED'))
    async def call(self, op, payload=None):
        token = uuid.uuid4().hex
        future = asyncio.get_running_loop().create_future(); self.pending[token] = future
        try:
            self.commands.put_nowait((token, op, payload or {}))
            return await asyncio.wait_for(asyncio.shield(future), 240 if op in ('PREPARE','PREPARE_SKILL','EXECUTE') else 30)
        finally:
            self.pending.pop(token, None)
    async def close(self):
        if self.process.is_alive():
            await self.call('SHUTDOWN')
            await asyncio.to_thread(self.process.join, 3)
        self.reader.cancel()
        await asyncio.gather(self.reader, return_exceptions=True)
        self.commands.close(); self.results.close()
