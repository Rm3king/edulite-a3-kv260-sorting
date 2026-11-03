"""Run real transport handshake/timing/plan-only or explicitly MOCK workflow."""
import argparse
import asyncio
import json
import time
import uuid
from pathlib import Path
from .client import Client
from .tasks import planning_fixture

async def run(a):
    c=Client(a.host,a.port,'aipc_probe_'+uuid.uuid4().hex[:8]);rows=[]
    try:
        caps=await c.connect();sync=await c.sync(a.samples)
        scene='fixture_'+uuid.uuid4().hex[:8];task='task_'+uuid.uuid4().hex[:8]
        rows.append(await c.call('SCENE',dict(scene_id=scene,source_kind='synthetic',calibration_id='UNSET',objects=[dict(object_id='screw_01',part_class='screw')])))
        request=c.message('PREPARE_PICK_PLACE',planning_fixture(scene,model_id=caps['model_id'],deadline=c.deadline(50),task_id=task))
        t=time.perf_counter();reply=await c.send_message(request);elapsed=time.perf_counter()-t
        rows.append(reply)
        replay=await c.send_message(request);assert replay==reply
        state=await c.call('STATUS');rows.append(state)
        if a.mock_execute:
            assert caps['backend']=='mock'
            assert reply['payload']['status']=='PLANNED',reply
            stop=asyncio.Event();hb=asyncio.create_task(c.heartbeat(stop))
            try:
                p=reply['payload'];msg=c.message('EXECUTE_PLAN',dict(plan_id=p['plan_id'],plan_digest=p['plan_digest']))
                rows.append(await c.send_message(msg));rows.append(await c.send_message(msg))
                for _ in range(60):
                    await asyncio.sleep(.05);query=await c.call('QUERY_TASK',dict(task_id=task))
                    if query['payload']['state']=='MOTION_DONE':break
                else:raise RuntimeError('mock task did not complete')
                rows.append(query);rows.append(await c.call('VERIFY_RESULT',dict(task_id=task,result='SUCCESS',evidence_id='SYNTHETIC_TEST_ORACLE')))
                rows.append(await c.call('STATUS'))
                assert rows[-1]['payload']['owner']['executions']==1
            finally:
                stop.set();await hb
        output=dict(status='PASS' if reply['payload']['status']=='PLANNED' else 'FAIL',
                    capabilities=caps,sync=sync,planning_call_s=elapsed,duplicate_prepare_exact_reply=True,
                    transport_scope=a.transport_scope,mock_execution=a.mock_execute,physical_motion=False,
                    source='EXPLICIT_SYNTHETIC_TARGETS_NO_CAMERA_CALIBRATION',responses=rows)
        Path(a.output).write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps({k:v for k,v in output.items() if k not in ('responses','sync')},indent=2),flush=True)
        assert output['status']=='PASS',reply
    finally:await c.close()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=46000)
    p.add_argument('--samples',type=int,default=30);p.add_argument('--output',required=True);p.add_argument('--mock-execute',action='store_true')
    p.add_argument('--transport-scope',default='LOCAL_TEST');asyncio.run(run(p.parse_args()))
