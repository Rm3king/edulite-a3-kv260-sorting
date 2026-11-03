"""Loopback protocol responsiveness with real numerical compilation, no hardware.

Uses the explicit mock backend and a numerical owner fixture. PREPARE exercises
the registered air-pick compiler, not calibrated visual pick/place execution.
All timings include spawn/import/cache/transfer; real capture and PL FK absent.
"""
import argparse
import asyncio
import gzip
import hashlib
import json
import time
from pathlib import Path
from screwcell.preparation import NumericalPreparation
from screwcell.service import Service
from screwcell.client import Client
from screwcell.tasks import planning_fixture

class OfflineOwner:
    def __init__(self,a,out):
        self.a=a;self.out=out;self.events=asyncio.Queue();self.state='IDLE';self.progress={};self.calls=0;self.capture=json.loads(Path(a.capture).read_text())
    async def call(self,op,p=None):
        if op=='STATUS':return dict(state=self.state,preparation_progress=self.progress,physical_execution_supported=False,motor_commands_sent=0)
        if op in ('HEARTBEAT','SHUTDOWN'):return dict(stopped=op=='SHUTDOWN')
        if op!='PREPARE':raise ValueError('OFFLINE_NUMERICAL_ONLY_NO_EXECUTION')
        self.calls+=1;self.state='PREPARING';self.task=p['task_id'];started=time.perf_counter()
        job=NumericalPreparation(runtime=self.a.runtime,profile=self.a.profile,tracking=self.a.tracking,
             capture=self.capture,cache_path=str(self.out/'encoding_cache.json.gz'),workers=self.a.workers)
        try:
            while True:
                v=job.poll(.005)
                if v and 'progress'in v:
                    self.progress=v['progress'];await self.events.put(dict(event='PREPARATION_PROGRESS',task_id=self.task,time_ns=time.monotonic_ns(),backend='mock',simulated=True,**self.progress))
                if v and 'error'in v:
                    (self.out/(self.task+'.failure.json')).write_text(json.dumps(v,indent=2)+'\n');raise ValueError(v['error'])
                if v and 'result'in v:compiled=v['result'];break
                await asyncio.sleep(.005)
            bank=compiled['bank'];branches=compiled['branches'];words=dict(main=[list(r['words']) for r in bank['records']],branches={str(i):{k:[list(r['words']) for r in b[k]] for k in ('brake','resume','suffix')} for i,b in branches.items()})
            raw=json.dumps(words,sort_keys=True,separators=(',',':')).encode();sha=hashlib.sha256(raw).hexdigest()
            (self.out/(self.task+'.words.json.gz')).write_bytes(gzip.compress(raw,compresslevel=3))
            return dict(compiled['metrics'],prepared_digest=sha,word_digest=sha,segments=len(bank['records']),checkpoints=len(branches),
                parent_numerical_total_s=time.perf_counter()-started,physical_execution_supported=False,motor_commands_sent=0,
                scope='Real numerical registered air-pick compilation under mock protocol; no live capture, PL audit, or actuators')
        finally:self.state='IDLE'
    async def close(self):pass

async def run(a):
    out=Path(a.output);out.mkdir(parents=True,exist_ok=False);owner=OfflineOwner(a,out);service=Service(owner,str(out/'state.db'),'mock')
    server=await asyncio.start_server(service.control,'127.0.0.1',0);port=server.sockets[0].getsockname()[1];c=Client(port=port);observer=Client(port=port,client_id='preparation_observer',role='observer');rounds=[]
    try:
        caps=await c.connect();assert not caps['physical_execution_supported'];await c.sync(3);await observer.connect()
        for i in range(a.rounds):
            if i==2:
                for transaction in owner.capture['transactions']:
                    if transaction['motor']==1:transaction['decoded']['position_motor_rad']+=.01
                owner.capture['synthetic_for_planning_only']=True
            scene='numeric_scene_'+str(i);await c.call('SCENE',dict(scene_id=scene,source_kind='synthetic',objects=[dict(object_id='screw_01',part_class='screw')]))
            payload=planning_fixture(scene_id=scene,deadline=c.deadline(60),task_id='compile_'+str(i));msg=c.message('PREPARE_PICK_PLACE',payload)
            started=time.perf_counter();pending=asyncio.create_task(c.send_message(msg));samples=[]
            while not pending.done():
                t=time.perf_counter();status=await observer.call('STATUS');latency=(time.perf_counter()-t)*1000
                sync=await observer.sync(1);samples.append(dict(elapsed_s=time.perf_counter()-started,status_ms=latency,sync_rtt_ms=sync['best_rtt_ns']/1e6,**status['payload']['owner']))
                await asyncio.sleep(.02)
            reply=await pending;elapsed=time.perf_counter()-started;assert reply['payload']['status']=='PLANNED',reply
            before=owner.calls;assert await c.send_message(msg)==reply;assert owner.calls==before
            query=await c.call('QUERY_TASK',dict(task_id=payload['task_id']));assert any(x['event']=='PREPARATION_PROGRESS' for x in query['payload']['events'])
            rows=[s for s in samples if s['state']=='PREPARING'];assert rows,'NO_OBSERVATION_DURING_REAL_COMPILATION'
            rounds.append(dict(index=i,start_fixture='RECORDED_PLUS_SYNTHETIC_MOTOR1_0.01RAD' if i==2 else 'RECORDED',rpc_roundtrip_s=elapsed,report=reply['payload']['report'],status_samples=len(rows),max_status_ms=max(s['status_ms'] for s in rows),samples=samples,duplicate_request_did_not_recompile=True))
            (out/'rounds.json').write_text(json.dumps(rounds,indent=2)+'\n')
        report=dict(status='PASS_REAL_NUMERICAL_COMPILATION_AND_LOOPBACK_RESPONSIVENESS',rounds=[{k:v for k,v in r.items() if k!='samples'} for r in rounds],
            physical_backend=False,motor_commands_sent=0,scope='Mock protocol plus actual numerical process; includes cache IO/import/IPC, excludes physical capture/PL FK/network hardware')
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
    except Exception as e:
        (out/'failure.json').write_text(json.dumps(dict(error=repr(e)),indent=2)+'\n');raise
    finally:
        await c.close();await observer.close();server.close();await server.wait_closed();await service.close()

if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ['runtime','profile','tracking','capture','output']:p.add_argument('--'+name,required=True)
    p.add_argument('--rounds',type=int,default=3);p.add_argument('--workers',type=int,default=4);asyncio.run(run(p.parse_args()))
