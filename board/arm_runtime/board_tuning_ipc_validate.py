#!/usr/bin/python3
"""Real board CAN/IPC stress, internal loopback only; no motor commands."""
import argparse,fcntl,json,multiprocessing,time
from pathlib import Path
from autotune import upload_evidence,CanDevice


def stalled_reader(conn):
    try:
        command,_=conn.recv()
        if command!='analyze':raise ValueError('analyze required')
        time.sleep(.2);conn.send(('analysis_ready',None));chunks=0;count=0
        while True:
            name,items=conn.recv()
            if name=='end':break
            chunks+=1;count+=len(items)
            if chunks%100==0:time.sleep(.05)
            conn.send(('chunk_ok',None))
        time.sleep(.2);conn.send(('ok',dict(chunks=chunks,items=count)))
    finally:conn.close()


def run(out):
    lock=open('/tmp/kv260_gripper_commission.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    d=CanDevice();proc=None;conn=None
    try:
        if d.read(0x404) or d.read(0x5a8):raise RuntimeError('stopped/disarmed controller required')
        # Like the real supervisor, build fixtures and spawn before enabling
        # the controller lease; allocation itself is not the IPC under test.
        report=dict(feedback=[dict(payload='x'*512,n=i) for i in range(40000)],
            segment_submissions=[dict(payload='x'*30000)]*50)
        ctx=multiprocessing.get_context('spawn');conn,child=ctx.Pipe()
        proc=ctx.Process(target=stalled_reader,args=(child,));proc.start();child.close()
        # All external TX is inhibited by the CAN engine in internal mode.
        d.configure('internal');initial=d.status();next_tx=time.monotonic();last=next_tx;sent=0;received=0;maximum_gap=0.
        def pump(seconds):
            nonlocal next_tx,last,sent,received,maximum_gap
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                now=time.monotonic();maximum_gap=max(maximum_gap,now-last);last=now;d.heartbeat()
                if d.read(0x418):raise RuntimeError('internal CAN controller fault')
                if now>=next_tx and (d.read(0x41c)&65535)<12:
                    d.enqueue(0x1ffffd00,sent.to_bytes(8,'big'),due_us=(d.read(0x408)+150)&0xffffffff,ttl_us=100000)
                    sent+=1;next_tx=now+1/1400
                frame=d.receive()
                if frame:
                    if frame['id']!=0x1ffffd00 or frame['data']!=received.to_bytes(8,'big').hex():raise AssertionError('internal frame order/payload')
                    received+=1
                time.sleep(.0001)
        def wait():
            deadline=time.monotonic()+30
            while not conn.poll():
                pump(.001)
                if time.monotonic()>deadline or not proc.is_alive():raise TimeoutError('stress worker')
            return conn.recv()
        # Do not count construction of the test fixture as a live IPC interval.
        # No actuator is armed, but still heartbeat immediately before upload.
        d.heartbeat();last=time.monotonic();begin=last
        metrics=upload_evidence(conn,report,{},dict(rx_lost=0),wait,pump)
        next_tx=float('inf')
        until=time.monotonic()+.1
        while received<sent:
            pump(.001)
            if time.monotonic()>until:raise TimeoutError('internal final drain')
        final=d.status()
        if final['rx_overflow']!=initial['rx_overflow'] or final['expired']!=initial['expired']:raise AssertionError('lost/expired internal frames')
        if maximum_gap>=.08:raise AssertionError('CAN service gap exceeded original stale threshold')
        return dict(status='PASS_INTERNAL_CAN_WITH_STALLED_ANALYSIS',external_tx_enabled=False,motor_enable_sent=False,
            sent=sent,received=received,maximum_can_service_gap_s=maximum_gap,elapsed_s=time.monotonic()-begin,
            rx_overflow_delta=final['rx_overflow']-initial['rx_overflow'],expired_delta=final['expired']-initial['expired'],worker=metrics)
    finally:
        d.stop();d.close();lock.close()
        if conn is not None:conn.close()
        if proc is not None:
            proc.join(timeout=2)
            if proc.is_alive():proc.terminate();proc.join()


if __name__=='__main__':
    a=argparse.ArgumentParser(description=__doc__);a.add_argument('--output',required=True);x=a.parse_args()
    result=run(x.output);Path(x.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
