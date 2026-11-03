"""Real live-capture/PL-audit preparation measurements; never EXECUTE."""
import argparse,asyncio,json,time,uuid
from pathlib import Path
from screwcell.client import Client
async def run(a):
 out=Path.home()/'Wenq_WorkSpace/EDULITE_A3_0211abadec527575/aipc_app/reports/preparation_board_20261021';out.mkdir(exist_ok=True)
 c=Client('192.168.26.2',client_id='aipc_physical_acceptance');o=Client('192.168.26.2',client_id='preparation_observer',role='observer');h=Client('192.168.26.2',client_id='aipc_physical_acceptance')
 rounds=[]
 try:
  caps=await c.connect();await c.sync(20);await o.connect();await h.connect()
  for i in range(a.rounds):
   task='prepare_'+str(i)+'_'+uuid.uuid4().hex[:8];start=time.perf_counter();msg=c.message('PREPARE_SKILL',dict(task_id=task,skill_id='empty_pick_return_v1',model_id=caps['model_id'],valid_until_kv_monotonic_ns=c.deadline(600)))
   pending=asyncio.create_task(c.send_message(msg));samples=[]
   while not pending.done():
    t=time.perf_counter();reply=await o.call('STATUS');status_ms=(time.perf_counter()-t)*1000
    t=time.perf_counter();alive=await h.call('HEARTBEAT');heartbeat_ms=(time.perf_counter()-t)*1000;assert alive['payload']['status']=='ALIVE'
    sync=await o.sync(1);samples.append(dict(elapsed_s=time.perf_counter()-start,rpc_ms=status_ms,heartbeat_ms=heartbeat_ms,sync_rtt_ms=sync['best_rtt_ns']/1e6,owner=reply['payload']['owner']));await asyncio.sleep(.05)
   prepared=await pending;elapsed=time.perf_counter()-start
   record=dict(index=i,task_id=task,commands=[dict(type='PREPARE_SKILL',reply=prepared)],rpc_s=elapsed,samples=samples)
   (out/(a.prefix+str(i)+'.json')).write_text(json.dumps(record,indent=2)+'\n')
   assert prepared['payload']['status']=='PLANNED',prepared
   assert await c.send_message(msg)==prepared
   q=await c.call('QUERY_TASK',dict(task_id=task));record['events']=q['payload']['events']
   assert any(x['event']=='PREPARATION_PROGRESS' for x in record['events'])
   status=(await o.call('STATUS'))['payload']['owner'];assert status['executions']==0 and status['pl_read_only']['active_motors']==0
   rows=[s for s in samples if s['owner']['state']=='PREPARING'];assert rows
   report=prepared['payload']['report'];summary=dict(index=i,task_id=task,rpc_s=elapsed,report=report,preparing_status_samples=len(rows),max_status_rpc_ms=max(s['rpc_ms'] for s in samples),duplicate_prepare_exact_reply=True,execution_count=status['executions'])
   rounds.append(summary);(out/(a.prefix+'summary.json')).write_text(json.dumps(rounds,indent=2)+'\n');(out/(a.prefix+str(i)+'.json')).write_text(json.dumps(record,indent=2)+'\n');print(json.dumps(summary),flush=True)
 finally:await c.close();await o.close();await h.close()
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--rounds',type=int,default=2);p.add_argument('--prefix',default='prepare_');asyncio.run(run(p.parse_args()))
