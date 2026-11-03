"""Run real TCP v2 task messages through a contact-physics simulation backend.

Always binds loopback, always MuJoCo. No hardware target/enable flag exists.
The reported verification is simulator body-state evidence, never camera proof.
"""
import argparse
import asyncio
import contextlib
import hashlib
import json
import math
import os
import signal
import sys
import time
from pathlib import Path
from screwcell.service import Service
from screwcell.owner import Owner
from screwcell.client import Client
from screwcell.tasks import planning_fixture
from .tasks import result,validate,bind,Ledger
from .tool_geometry import TOOL_ID

async def run(a):
    out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    plan=validate(json.loads(Path(a.plan).read_text())) if a.plan else result(steps=[dict(part_class='screw',quantity=1,region='all',destination='B')])
    if plan['action']!='plan':raise ValueError('SIMULATION_REQUIRES_PLAN')
    owner=Owner('mujoco',a.urdf,'','',str(out/'motion'))
    from .simulation import model_identity
    service=Service(owner,str(out/'state.db'),'mujoco');service.configured_model_id=model_identity(a.urdf)
    server=await asyncio.start_server(service.control,'127.0.0.1',0)
    port=server.sockets[0].getsockname()[1];c=Client('127.0.0.1',port,'simulation_workbench')
    stop=asyncio.Event();hb=None;ledger=Ledger();records=[];commands=asyncio.Queue();manual_pause=False
    loop=asyncio.get_running_loop()
    current_task=asyncio.current_task()
    loop.add_signal_handler(signal.SIGTERM,current_task.cancel)
    def stdin_ready():
        line=sys.stdin.readline()
        if not line:
            with contextlib.suppress(Exception):loop.remove_reader(sys.stdin.fileno())
            return
        try:commands.put_nowait(json.loads(line)['command'])
        except (ValueError,KeyError):pass
    try:
        with contextlib.suppress(Exception):loop.add_reader(sys.stdin.fileno(),stdin_ready)
        caps=await c.connect();assert caps['backend']=='mujoco' and not caps['physical_execution_supported'];await c.sync(5)
        hb=asyncio.create_task(c.heartbeat(stop))
        snap=(await c.call('STATUS'))['payload']['owner']['snapshot']
        scene=dict(scene_id='sim_scene_0',source_kind='synthetic',calibration_id='SIM_CALIBRATION',image_width_px=800,
                   objects=[dict(object_id=n,part_class='screw' if n.startswith('screw') else 'nut',center_uv_px=[400+(p[0]+.18)*780,270-p[1]*780],geometry_status='SIM_GROUND_TRUTH') for n,p in snap['objects'].items()])
        actions=bind(plan,scene);(out/'bound_actions.json').write_text(json.dumps(actions,ensure_ascii=False,indent=2)+'\n')
        for index,action in enumerate(actions):
            name=action['object_id'];task='sim_pick_'+str(index)
            now=(await c.call('STATUS'))['payload']['owner']['snapshot'];pos=now['objects'][name]
            yaw=now.get('object_yaw_rad',{}).get(name,0.)
            quat=[0,0,math.sin(yaw/2),math.cos(yaw/2)]
            s=dict(scene,scene_id='sim_scene_'+str(index));await c.call('SCENE',{k:s[k] for k in ['scene_id','source_kind','objects','calibration_id']})
            payload=planning_fixture(s['scene_id'],name,caps['model_id'],c.deadline(60),task)
            destination_y=-.12 if action['destination']=='B' else .12
            place=[-.22,destination_y+[0,-.026,.026][index%3],pos[2]+.04]
            place_yaw=math.pi/2 if action['destination']=='B' else 0.
            place_quat=[0,0,math.sin(place_yaw/2),math.cos(place_yaw/2)]
            payload.update(calibration_id='SIM_CALIBRATION',layout_id='SIM_LAYOUT',tool_id='SIM_'+TOOL_ID,
                part_class=action['part_class'],
                lift_height_m=.06,
                approach_height_m=.10-float(os.environ.get('A3_SIM_INSERT_MM','0'))/1000,
                grasp_tcp=dict(position_m=pos,quaternion_xyzw=quat),place_tcp=dict(position_m=place,quaternion_xyzw=place_quat),
                planning_start_joint_rad=now['q_rad']+[.35])
            prepared=await c.call('PREPARE_PICK_PLACE',payload)
            assert prepared['payload']['status']=='PLANNED',prepared
            ledger.begin(name);await c.call('HEARTBEAT')
            response=await c.call('EXECUTE_PLAN',{k:prepared['payload'][k] for k in ['plan_id','plan_digest']});assert response['payload']['status']=='STARTED',response
            begin=time.monotonic();paused=False;resumed=False;pause_at=None
            while time.monotonic()-begin<180:
                current=(await c.call('STATUS'))['payload']['owner'];records.append(current)
                print(json.dumps(dict(stage='telemetry',snapshot=current['snapshot'],state=current['state'])),flush=True)
                if current['state']=='MOTION_DONE':break
                if current['state']=='HOLDING_FAULT':raise RuntimeError(str(current['last_result']))
                if a.test_pause and not paused and time.monotonic()-begin>.25:
                    await c.call('PAUSE',dict(task_id=task));paused=True;pause_at=time.monotonic()
                if a.test_pause and paused and not resumed and current['state']=='HOLDING' and time.monotonic()-pause_at>.3:
                    await c.call('RESUME',dict(task_id=task));resumed=True
                while not commands.empty():
                    command=await commands.get()
                    if command in ('PAUSE','RESUME','RETURN_SUPPORTED'):
                        answer=await c.call(command,dict(task_id=task));print(json.dumps(dict(stage='operator',command=command,response=answer)),flush=True)
                await asyncio.sleep(.05)
            else:raise TimeoutError('SIMULATION_COMPLETION_TIMEOUT')
            for _ in range(20):
                q=await c.call('QUERY_TASK',dict(task_id=task))
                if q['payload']['state']=='MOTION_DONE':break
                await asyncio.sleep(.02)
            physical=current['last_result'];success=physical.get('lift_detected',False) and physical.get('destination_occupied',False) and physical.get('geometry_pass',False)
            evidence='sim_body_state_'+task
            answer=await c.call('VERIFY_RESULT',dict(task_id=task,result='SUCCESS' if success else 'FAIL',evidence_id=evidence))
            assert answer['payload']['status']=='VERIFIED'
            ledger.verify(name,'SUCCESS' if success else 'FAIL',evidence)
            print(json.dumps(dict(stage='completed',task_id=task,simulation_success=success,replay_directory=str(out/'motion'/task))),flush=True)
        report=dict(status='PASS_SIMULATED_TASK_PROTOCOL_AND_CONTACT_GRASP' if len(ledger.verified)==len(actions) else 'FAIL_SIMULATED_CONTACT_GRASP',
            requested=len(actions),verified_simulated=len(ledger.verified),task=plan,ledger=ledger.events,telemetry=records,
            endpoint=dict(host='127.0.0.1',port=port),real_motor_commands_sent=0,
            scope='Actual TCP requests and MuJoCo frictional contact; verification from simulation body states, not real visual success')
        (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ('telemetry','ledger')},ensure_ascii=False),flush=True)
        if len(ledger.verified)!=len(actions):raise RuntimeError('SIMULATED_CONTACT_GRASP_FAILED')
    except Exception as e:
        (out/'failure.json').write_text(json.dumps(dict(status='FAILED_RETAINED',error=repr(e),ledger=ledger.events,verified_simulated=len(ledger.verified),real_motor_commands_sent=0),ensure_ascii=False,indent=2)+'\n')
        raise
    finally:
        with contextlib.suppress(Exception):loop.remove_reader(sys.stdin.fileno())
        stop.set()
        if hb:await asyncio.gather(hb,return_exceptions=True)
        await c.close();server.close();await server.wait_closed();await service.close()
    # Heavy EGL context creation must not block the protocol owner's heartbeat
    # handling. Physics snapshots are recorded during execution and rendered
    # only after the simulation service has stopped.
    from .simulation import render_saved
    started=time.perf_counter()
    for directory in sorted((out/'motion').glob('sim_pick_*')):
        if (directory/'render_scene.xml').exists():
            await asyncio.to_thread(render_saved,directory)
            print(json.dumps(dict(stage='replay_images_ready',replay_directory=str(directory))),flush=True)
    report['deferred_render_s']=time.perf_counter()-started
    report['graphics_scope']='Rendered recorded physics states after protocol service shutdown; excluded from control lease'
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--urdf',required=True);p.add_argument('--output',required=True);p.add_argument('--plan');p.add_argument('--test-pause',action='store_true')
    asyncio.run(run(p.parse_args()))
