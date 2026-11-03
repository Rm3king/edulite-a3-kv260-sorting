"""MuJoCo owner for the existing TCP task/telemetry protocol. Simulation only."""
import json
import queue
import time
from pathlib import Path
import numpy as np
from .simulation import Simulation
from .tool_geometry import TOOL_ID

class ReturnRequested(Exception):pass

def worker(commands,results,urdf,evidence):
    root=Path(evidence);root.mkdir(parents=True,exist_ok=True)
    sim=Simulation(urdf,root/'idle');state='IDLE';task=None;prepared=None;lease=0.;executions=0;last_result=None
    sim.defer_render=True
    def event(name,**data):results.put(dict(event=name,task_id=task,time_ns=time.monotonic_ns(),backend='mujoco',simulated=True,**data))
    def status():return dict(state=state,task_id=task,executions=executions,physical_execution_supported=False,motor_commands_sent=0,
        simulated=True,snapshot=sim.snapshot(state),last_result=last_result,stop_model='SIM_INSTANT_REFERENCE_HOLD_NOT_PHYSICAL_BRAKING')
    def dispatch(item):
        nonlocal state,task,prepared,lease,executions,sim
        token,op,p=item;value={}
        try:
            if op=='STATUS':value=status()
            elif op=='HEARTBEAT':lease=time.monotonic()+.5
            elif op=='PREPARE':
                if state not in ('IDLE','MOTION_DONE'):raise ValueError('SIM_BUSY')
                if p['calibration_id']!='SIM_CALIBRATION' or p['layout_id']!='SIM_LAYOUT' or p['tool_id']!='SIM_'+TOOL_ID:raise ValueError('SIMULATION_IDS_REQUIRED')
                snap=sim.snapshot();name=p['object_id']
                if name not in snap['objects']:raise ValueError('SIM_OBJECT_MISSING')
                pos=np.asarray(p['grasp_tcp']['position_m'])
                if np.linalg.norm(pos-np.asarray(snap['objects'][name]))>.005:raise ValueError('SIM_TARGET_MOVED')
                quat=np.asarray(p['grasp_tcp']['quaternion_xyzw']);mat=np.zeros(9);sim.mj.mju_quat2Mat(mat,np.r_[quat[3],quat[:3]])
                sim.ik(pos+[0,0,p['approach_height_m']],mat.reshape(3,3))
                sim.ik(pos,mat.reshape(3,3))
                pq=np.asarray(p['place_tcp']['quaternion_xyzw']);pm=np.zeros(9);sim.mj.mju_quat2Mat(pm,np.r_[pq[3],pq[:3]])
                pp=np.asarray(p['place_tcp']['position_m']);sim.ik(pp+[0,0,p['lift_height_m']],pm.reshape(3,3));sim.ik(pp,pm.reshape(3,3))
                audit=sim.preflight_pick(name,pp,mat.reshape(3,3),pm.reshape(3,3))
                prepared=p;task=p['task_id'];value=dict(backend='mujoco',simulated=True,physical_execution_supported=False,motor_commands_sent=0,object_id=name,model_scope=sim.model_scope,nominal_path_audit=audit)
            elif op=='EXECUTE':
                if prepared is None or task!=p['task_id'] or state not in ('IDLE','MOTION_DONE'):raise ValueError('SIM_UNPREPARED')
                state='RUNNING';executions+=1;lease=time.monotonic()+.5;event('STARTED')
            elif op=='PAUSE':
                if state=='RUNNING':state='HOLDING';event('HOLDING',reason=p.get('reason','REQUESTED'))
            elif op=='RESUME':
                if state!='HOLDING':raise ValueError('NOT_HOLDING')
                state='RUNNING';lease=time.monotonic()+.5;event('RESUMED')
            elif op=='RETURN_SUPPORTED':
                if state!='HOLDING':raise ValueError('NOT_HOLDING')
                state='RETURNING';lease=time.monotonic()+.5;event('RETURNING')
            elif op=='SHUTDOWN':
                results.put(dict(token=token,value={'stopped':True}));raise SystemExit()
            else:raise ValueError('UNKNOWN_SIM_OPERATION')
            results.put(dict(token=token,value=value))
        except (ValueError,RuntimeError) as e:results.put(dict(token=token,error=str(e)))
    def poll():
        nonlocal state
        while True:
            try:dispatch(commands.get_nowait())
            except queue.Empty:break
        if state=='RUNNING' and time.monotonic()>lease:state='HOLDING';event('HOLDING',reason='LINK_LOST_SIMULATION')
        while state=='HOLDING':
            try:dispatch(commands.get(timeout=.01))
            except queue.Empty:pass
            sim.inside_poll=True
            try:sim.control(sim.reference,np.zeros(6),sim.grip,'HOLDING')
            finally:sim.inside_poll=False
        if state=='RETURNING':raise ReturnRequested()
    while True:
        try:dispatch(commands.get(timeout=.02))
        except queue.Empty:continue
        if state=='RUNNING' and prepared is not None:
            sim.output=root/task;sim.output.mkdir(parents=True,exist_ok=False);sim.rows=[];sim.frames=[];home=sim.reference.copy();sim.poll_hook=poll
            try:
                q=np.asarray(prepared['grasp_tcp']['quaternion_xyzw']);rot=np.zeros(9);sim.mj.mju_quat2Mat(rot,np.r_[q[3],q[:3]])
                pq=np.asarray(prepared['place_tcp']['quaternion_xyzw']);pm=np.zeros(9);sim.mj.mju_quat2Mat(pm,np.r_[pq[3],pq[:3]])
                last_result=sim.run_pick(prepared['object_id'],prepared['place_tcp']['position_m'],rot.reshape(3,3),pm.reshape(3,3))
                state='MOTION_DONE';event('MOTION_DONE',simulation_grasp_success=bool(last_result['lift_detected'] and last_result['destination_occupied'] and last_result['geometry_pass']))
            except ReturnRequested:
                sim.poll_hook=None;sim.movej(home,phase='SIM_ABORT_RETURN');state='MOTION_DONE';last_result=dict(status='SIM_ABORT_RETURN_NO_GRASP_SUCCESS')
                event('MOTION_DONE',simulation_grasp_success=False)
            except Exception as e:
                sim.save_failure(e)
                state='HOLDING_FAULT';last_result=dict(status='SIMULATION_ERROR',error=str(e));event('HOLDING_FAULT',error=str(e))
            finally:sim.poll_hook=None;prepared=None
