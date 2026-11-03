#!/usr/bin/python3
"""Stopped-board internal CAN RX burst test; never external TX or motor enable."""
import argparse,fcntl,json,time
from pathlib import Path
from canctl import CanDevice


def run(count=100):
    if not 17<=count<=256:raise ValueError('burst must exercise expanded RX FIFO')
    lock=open('/tmp/kv260_gripper_commission.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    d=CanDevice();result={'status':'RUNNING','external_tx_enabled':False,'motor_enable_sent':False}
    try:
        if d.read(0x404) or d.read(0x5a8):raise RuntimeError('stopped/disarmed controller required')
        d.configure('internal');initial=d.status();sent=0;begin=time.monotonic()
        while sent<count:
            d.heartbeat()
            if d.read(0x41c)&65535>=14:time.sleep(.0003);continue
            ident=0x1ffffe00+sent;payload=sent.to_bytes(8,'big')
            d.enqueue(ident,payload,due_us=(d.read(0x408)+150)&0xffffffff,ttl_us=100000);sent+=1
        deadline=time.monotonic()+.15
        while d.read(0x428)-initial['received']<count:
            d.heartbeat()
            if time.monotonic()>deadline:raise TimeoutError('RX burst not fully received')
            time.sleep(.0003)
        occupancy=d.read(0x41c)>>16
        if occupancy!=count or d.read(0x42c)!=initial['rx_overflow']:raise AssertionError('RX capacity/overflow mismatch')
        received=[]
        for i in range(count):
            d.heartbeat();frame=d.receive()
            if not frame or frame['id']!=0x1ffffe00+i or frame['data']!=i.to_bytes(8,'big').hex():raise AssertionError('RX burst order/payload mismatch')
            received.append(frame)
        if d.read(0x41c)>>16:raise AssertionError('RX not empty after drain')
        result.update(status='PASS_INTERNAL_RX_BURST',frames=count,maximum_undrained_occupancy=occupancy,
            rx_overflow_delta=d.read(0x42c)-initial['rx_overflow'],expired_delta=d.read(0x424)-initial['expired'],
            elapsed_seconds=time.monotonic()-begin,received_frames=received)
    finally:
        d.stop();result['final_controller']=d.status();d.close();lock.close()
    return result


if __name__=='__main__':
    a=argparse.ArgumentParser(description=__doc__);a.add_argument('--count',type=int,default=100);a.add_argument('--output',required=True);x=a.parse_args()
    r=run(x.count);Path(x.output).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='received_frames'},indent=2))
