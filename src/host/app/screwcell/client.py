"""Reusable v2 client; explicit mock/planning requests, no motor API."""
import asyncio
import contextlib
import time
from . import wire

class Client:
    def __init__(self, host='127.0.0.1', port=46000, client_id='aipc_screwcell', role='control'):
        self.host=host; self.port=port; self.client_id=client_id; self.role=role
        self.session=''; self.sequence=time.monotonic_ns(); self.lock=asyncio.Lock()
    async def connect(self):
        # Only the side-effect-free HELLO handshake is retried. Never replay an
        # EXECUTE on reconnect; query the existing task instead.
        for attempt in range(20):
            try:
                self.reader,self.writer=await asyncio.wait_for(asyncio.open_connection(self.host,self.port),2)
                hello=await self.call('HELLO',dict(client_id=self.client_id,role=self.role))
                wire.require(hello['payload']['status']=='HELLO','HELLO rejected')
                self.session=hello['session_id']; self.capabilities=hello['payload']
                return self.capabilities
            except (ConnectionError, OSError, asyncio.TimeoutError, asyncio.IncompleteReadError):
                await self.close()
                if attempt==19:raise
                await asyncio.sleep(.1)
    def message(self, kind, payload=None, request_id=None):
        self.sequence+=1
        return wire.envelope(kind,payload,self.session,self.sequence,request_id)
    async def send_message(self,message):
        async with self.lock:
            await wire.write(self.writer,message)
            result=await wire.read(self.reader,250 if message['type'] in ('PREPARE_PICK_PLACE','PREPARE_SKILL','EXECUTE_PLAN') else 35)
            wire.require(result['request_id']==message['request_id'],'response request mismatch')
            return result
    async def call(self,kind,payload=None,request_id=None):
        return await self.send_message(self.message(kind,payload,request_id))
    async def sync(self,count=20):
        rows=[]
        for _ in range(count):
            t1=time.monotonic_ns(); result=await self.call('SYNC',dict(t1_ns=t1));t4=time.monotonic_ns()
            p=result['payload'];wire.require(p['status']=='SYNC' and p['t1_ns']==t1,'invalid sync response')
            t2,t3=p['t2_ns'],p['t3_ns']
            rtt=(t4-t1)-(t3-t2)
            wire.require(t1<=t4 and t2<=t3 and rtt>=0,'invalid clock exchange')
            rows.append(dict(t1_ns=t1,t2_ns=t2,t3_ns=t3,t4_ns=t4,rtt_ns=rtt,offset_ns=((t2-t1)+(t3-t4))/2))
        best=min(rows,key=lambda r:r['rtt_ns']);self.offset_ns=best['offset_ns']
        return dict(samples=rows,best_rtt_ns=best['rtt_ns'],estimated_offset_ns=best['offset_ns'],
                    nominal_offset_uncertainty_ns=best['rtt_ns']/2,
                    scope='Software four-timestamp estimate; path asymmetry uncalibrated; not camera exposure or PL tick sync')
    def deadline(self,seconds=10):
        wire.require(hasattr(self,'offset_ns'),'synchronize before preparing target')
        return int(time.monotonic_ns()+self.offset_ns+seconds*1e9)
    async def heartbeat(self,stop):
        while not stop.is_set():
            result=await self.call('HEARTBEAT')
            wire.require(result['payload']['status']=='ALIVE','heartbeat rejected')
            try:await asyncio.wait_for(stop.wait(),.1)
            except asyncio.TimeoutError:pass
    async def close(self):
        writer=getattr(self,'writer',None)
        if writer is not None:
            writer.close()
            with contextlib.suppress(ConnectionError,OSError,asyncio.TimeoutError):
                await asyncio.wait_for(writer.wait_closed(),1)
