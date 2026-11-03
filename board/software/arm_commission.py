#!/usr/bin/python3
"""Attended, supported-arm PP commissioning on v4. No zero/Flash/fault-clear.

Prepare and enable one motor at a time while refreshing already enabled axes.
Move tests return to measured starts; zero tests return to existing calibrated zero.
Every action ends with a bounded reset sequence.
No unattended keep-enabled service and no automatic current increases.
"""
import argparse
from collections import deque
import fcntl
import json
import math
from pathlib import Path
import signal
import struct
import time
from a3_protocol import enable, disable, decode_feedback, motor_id
from canctl import CanDevice
from joint_safety import LIMITS, joint_to_motor

SPEED=.02
ACCEL=.1
TIMEOUT=6000
STEP=.01
ZERO_ORDER=(4,5,6,7,1,2,3)
ZERO_CURRENT=(1.5,3.,3.,2.5,1.,1.,.5)


class VelocityMonitor:
    """Sampled diagnostics, not a certified instantaneous speed bound.

    Keep a gross single-sample trip, qualify modest estimator excursions by
    duration, and independently check encoder displacement over >=80 ms.
    Commanded PP speeds are <=0.06 rad/s; position LSB is 25.14/65535 rad.
    """
    def __init__(self):
        self.history={};self.high_since={}

    def observe(self,f):
        mid=f['motor'];now=f['time'];velocity=abs(f['velocity'])
        h=self.history.setdefault(mid,deque())
        if h and not 0<now-h[-1][0]<=.08:raise RuntimeError(f'Axis {mid}: velocity observation gap/order')
        if velocity>.8:raise RuntimeError(f'Axis {mid}: gross reported velocity')
        if velocity>.2:
            self.high_since.setdefault(mid,now)
            if now-self.high_since[mid]>=.1:raise RuntimeError(f'Axis {mid}: sustained reported velocity')
        else:self.high_since.pop(mid,None)
        h.append((now,f['position']))
        while len(h)>1 and now-h[1][0]>=.08:h.popleft()
        dt=now-h[0][0]
        if dt>=.08 and abs(f['position']-h[0][1])>.15*dt+2*25.14/65535:
            raise RuntimeError(f'Axis {mid}: encoder window velocity')
        return velocity>.2


def parameter(mid,index,value=None,integer=False):
    motor_id(mid)
    if value is None:return 0x1100fd00|mid,struct.pack('<H',index)+bytes(6)
    if integer:
        if type(value) is not int or not 0<=value<=0xffffffff:raise ValueError('uint32 required')
        payload=struct.pack('<I',value)
    else:
        if not math.isfinite(value):raise ValueError('finite parameter required')
        payload=struct.pack('<f',value)
    return 0x1200fd00|mid,struct.pack('<H',index)+bytes(2)+payload


def readback_matches(index,wanted,actual,integer=False):
    if integer:return actual==wanted
    # RS00 loc_ref can differ from MECH_POS-derived hold by an encoder-sized
    # increment while disabled. 0.1 mrad is 1% of this test's commanded step.
    tolerance=.0001 if index==0x7016 else 1e-6
    return math.isfinite(actual) and math.isclose(actual,wanted,rel_tol=0 if index==0x7016 else 1e-5,abs_tol=tolerance)


def make_plan(mid,start,config):
    motor_id(mid)
    if mid==7:
        g=config['gripper'];lo,hi=sorted((g['closed_motor_rad'],g['open_motor_rad']))
    else:lo,hi=sorted(joint_to_motor(mid,q) for q in LIMITS[mid])
    if not math.isfinite(start) or not lo<=start<=hi:raise ValueError('Initial position outside measured/SDK limits')
    target=start+(STEP if start<(lo+hi)/2 else -STEP)
    if not lo+.002<=target<=hi-.002:raise ValueError('Insufficient interior travel')
    return dict(start=start,target=target,lower=max(lo,min(start,target)-.01),
                upper=min(hi,max(start,target)+.01),speed=SPEED,acceleration=ACCEL,
                current=.5 if mid<=6 else .3,timeout=TIMEOUT)


def zero_plan(mid,start,config):
    p=make_plan(mid,start,config)
    limits=sorted(joint_to_motor(mid,q) for q in LIMITS[mid]) if mid<7 else sorted((config['gripper']['open_motor_rad'],config['gripper']['closed_motor_rad']))
    if not limits[0]<=0<=limits[1]:raise ValueError('Calibrated zero is outside permitted travel')
    p.update(target=0.,lower=max(limits[0],min(start,0.)-.01),upper=min(limits[1],max(start,0.)+.01),
             current=ZERO_CURRENT[mid-1],speed=.04 if mid==7 else .06,acceleration=.15)
    # Shoulder/elbow zeros are SDK interval endpoints. Approach them last and
    # more slowly, retaining the existing limits and feedback abort envelope.
    if mid in (2,3):p.update(speed=.01,acceleration=.03)
    return p


class Session:
    def __init__(self,dev,report):
        self.dev=dev;self.report=report;self.feedback={};self.responses=[]
        self.active=set();self.targets={};self.plans={};self.last_refresh=0.;self.stopping=False
        self.velocity_monitor=VelocityMonitor();self.target_owner='host'

    def send(self,packet):
        ident,data=packet;mid=ident&255
        if not 1<=mid<=7 or ident>>24 not in (3,4,17,18):raise ValueError('Commissioning packet rejected')
        self.dev.heartbeat();self.dev.enqueue(ident,data)
        self.report['commands'].append(dict(id=ident,data=data.hex(),time=time.monotonic()))

    def require(self,mid,mode=None):
        f=self.feedback.get(mid)
        if not f or time.monotonic()-f['time']>.08:raise RuntimeError(f'Axis {mid}: missing/stale Type2')
        if f['fault'] or (mode is not None and f['mode']!=mode):raise RuntimeError(f'Axis {mid}: fault/mode {f}')
        return f

    def pump(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            self.dev.heartbeat()
            if self.dev.read(0x418) or self.dev.read(56) or self.dev.read(60):raise RuntimeError('PL/CAN fault')
            frame=self.dev.receive()
            if frame:
                frame['time']=time.monotonic();self.report['frames'].append(frame)
                if frame['extended'] and not frame['rtr'] and frame['id']>>24==2:
                    mid=frame['id']>>8&255
                    f=decode_feedback(frame['id'],bytes.fromhex(frame['data']),model='RS00' if mid<=3 else 'EL05')
                    f['time']=frame['time'];f['pl_rx_us']=frame['timestamp_us'];self.feedback[mid]=f;self.report['feedback'].append(f)
                    if f['fault'] and not self.stopping:raise RuntimeError(f'Axis {mid}: motor fault {f}')
                    if mid in self.active and not self.stopping:
                        p=self.plans[mid]
                        if f['mode']!=2 or not p['lower']-.002<=f['position']<=p['upper']+.002:
                            raise RuntimeError(f'Axis {mid}: feedback envelope/mode violation {f}')
                        if self.velocity_monitor.observe(f):
                            self.report.setdefault('velocity_estimator_excursions',[]).append(dict(f))
                else:self.responses.append(frame)
            elif not self.stopping:time.sleep(.0003)
            if not self.stopping and self.target_owner=='host' and self.active and time.monotonic()-self.last_refresh>=.02:
                self.last_refresh=time.monotonic()
                for mid in sorted(self.active):
                    self.require(mid,2);self.send(parameter(mid,0x7016,self.targets[mid]))

    def read(self,mid,index,integer=False):
        self.responses.clear();self.send(parameter(mid,index));end=time.monotonic()+.08
        while time.monotonic()<end:
            self.pump(.002)
            for f in self.responses:
                data=bytes.fromhex(f['data'])
                if f['extended'] and not f['rtr'] and f['id']==0x110000fd+(mid<<8) and len(data)==8 and data[:2]==struct.pack('<H',index):
                    value=struct.unpack('<I' if integer else '<f',data[4:])[0]
                    if not math.isfinite(value):raise ValueError('Nonfinite readback')
                    return value
        raise TimeoutError(f'Axis {mid}: parameter {index:#x} timeout')

    def write_verified(self,mid,index,value,integer=False):
        self.send(parameter(mid,index,value,integer));self.pump(.003)
        observed=[]
        for attempt in range(8 if index==0x7016 else 1):
            actual=self.read(mid,index,integer);observed.append(actual)
            if readback_matches(index,value,actual,integer):break
            self.pump(.005)
        self.report.setdefault('readback_samples',[]).append(dict(motor=mid,index=hex(index),wanted=value,observed=observed))
        if not readback_matches(index,value,actual,integer):
            raise RuntimeError(f'Axis {mid}: parameter {index:#x} readback mismatch {actual}')
        self.report['readbacks'].setdefault(str(mid),{})[hex(index)]=actual

    def stop_all(self):
        self.stopping=True
        self.report['stop_started_time']=time.monotonic()
        # Cancel every pending hold before reset commands. Clear PL active bits
        # so feedback expiry cannot block later stop frames during the handshake.
        self.dev.configure('normal')
        sent={}
        for mid in reversed(range(1,8)):
            try:
                stamp=time.monotonic();self.send(disable(mid));sent[mid]=stamp
            except Exception as exc:self.report['stop_errors'][str(mid)]=repr(exc)
        try:self.pump(.06)
        except Exception as exc:self.report['stop_errors']['receive']=repr(exc)
        for mid in reversed(range(1,8)):
            def confirmed():
                f=self.feedback.get(mid)
                return f and mid in sent and f['time']>=sent[mid] and f['mode']==0
            if not confirmed():
                try:
                    if self.dev.read(0x418) or self.dev.read(0x404)&3!=3:self.dev.configure('normal')
                    stamp=time.monotonic();self.send(disable(mid));sent[mid]=stamp;self.pump(.06)
                except Exception as exc:self.report['stop_errors'][str(mid)]=repr(exc)
            if confirmed():
                self.report['stop_errors'].pop(str(mid),None)
                self.report['reset_confirmed'].append(mid)
                if self.feedback[mid].get('fault',0):self.report['stop_errors'][str(mid)]='Reset with motor fault'
            else:self.report['stop_errors'].setdefault(str(mid),'No fresh reset acknowledgement after retry')
        self.dev.stop();self.dev.write(0x5a0,0)
        for mid in range(1,8):self.dev.write(self.dev.pp_base(mid),0)
        self.report['final']=self.dev.status()
        self.report['final_motion_mask']=self.dev.read(0x5a0)
        self.report['final_pp_permits']=[self.dev.read(self.dev.pp_base(mid)) for mid in range(1,8)]


def run(action,move_axes=None):
    if move_axes is None:move_axes=list(range(1,8))
    if not move_axes or len(set(move_axes))!=len(move_axes) or any(type(m) is not int or not 1<=m<=7 for m in move_axes):raise ValueError('Unique move axes 1..7 required')
    report=dict(status='RUNNING',action=action,move_axes=move_axes,commands=[],frames=[],feedback=[],plans={},readbacks={},
                stages=[],reset_confirmed=[],stop_errors={},flash_writes=0,zero_writes=0)
    dev=CanDevice();session=Session(dev,report);started=False
    try:
        if dev.read(0x470)!=0x40000:raise RuntimeError('v4 required')
        if dev.read(0x404)!=0 or dev.read(0x5a0)!=0:raise RuntimeError('Controller must already be stopped with no motion permit')
        config=json.loads(Path(__file__).with_name('a3_commissioning.json').read_text())
        dev.configure('normal');started=True
        for mid in range(1,8):
            session.send(disable(mid));session.pump(.015);session.require(mid,0)
            p=session.read(mid,0x7019);session.plans[mid]=(zero_plan if action=='zero' else make_plan)(mid,p,config)
        report['plans']=session.plans
        if action=='probe':report['status']='PASS_SEVEN_RESET_AND_POSITION_PROBE';return report
        dev.stop();dev.write(0x5a0,127);dev.write(0x5a4,100000)
        for mid,p in session.plans.items():
            base=dev.pp_base(mid);dev.write(base,1)
            for offset,key in ((4,'lower'),(8,'upper'),(12,'speed'),(16,'acceleration'),(20,'current')):
                bits=struct.unpack('<I',struct.pack('<f',p[key]))[0];dev.write(base+offset,bits)
                if dev.read(base+offset)!=bits:raise RuntimeError('PL bank readback mismatch')
        dev.configure('normal')
        for mid in range(1,8):
            p=session.plans[mid]
            session.send(disable(mid));session.pump(.015);session.require(mid,0)
            fresh=session.read(mid,0x7019)
            if abs(fresh-p['start'])>.003:raise RuntimeError(f'Axis {mid}: moved during preparation')
            for index,key,integer in ((0x7005,None,True),(0x7024,'speed',False),(0x7025,'acceleration',False),
                                       (0x7018,'current',False),(0x7028,'timeout',True),(0x7016,'start',False)):
                session.write_verified(mid,index,1 if key is None else p[key],integer)
            session.require(mid,0)
            if dev.read(dev.pp_base(mid)+24)!=63:raise RuntimeError('PL preparation incomplete')
            session.targets[mid]=p['start']
            # No automatic refresh of this axis until first fresh active reply is checked.
            before=time.monotonic();session.send(enable(mid));session.pump(.01)
            f=session.require(mid,2)
            if f['time']<before or abs(f['position']-p['start'])>.01:raise RuntimeError('Unexpected enable displacement')
            session.active.add(mid);session.pump(.25)
            actual=session.read(mid,0x7019)
            if abs(actual-p['start'])>.005:raise RuntimeError(f'Axis {mid}: unable to hold start at limited current')
            report['stages'].append(dict(stage='hold',motor=mid,actual=actual,error=actual-p['start']))
        session.pump(.3)
        report['all_enabled_mask']=dev.read(0x5a8)
        if report['all_enabled_mask']!=127:raise RuntimeError('Not all seven active')
        report['all_axis_hold_positions']={}
        for mid in range(1,8):
            actual=session.read(mid,0x7019);session.require(mid,2)
            report['all_axis_hold_positions'][str(mid)]=actual
            if abs(actual-session.targets[mid])>.005:raise RuntimeError(f'Axis {mid}: simultaneous hold drift')
        if action=='move':
            for mid in move_axes:
                p=session.plans[mid]
                for target in (p['target'],p['start']):
                    session.targets[mid]=target;start=time.monotonic();settled=0
                    while time.monotonic()-start<3.:
                        session.pump(.02);actual=session.read(mid,0x7019)
                        f=session.require(mid,2)
                        settled=settled+1 if abs(actual-target)<.003 and abs(f['velocity'])<.03 else 0
                        # Other axes must continue holding while this axis moves.
                        for other in session.active-{mid}:
                            if abs(session.require(other,2)['position']-session.targets[other])>.007:
                                raise RuntimeError(f'Axis {other}: hold drift during axis {mid} motion')
                        if settled>=5:break
                    else:raise TimeoutError(f'Axis {mid}: target not reached; no automatic current increase')
                    report['stages'].append(dict(stage='move',motor=mid,target=target,actual=actual,
                                                 error=actual-target,elapsed=time.monotonic()-start))
        if action=='zero':
            # Move wrist and base before approaching shoulder/elbow endpoints.
            # One attempt, fixed current; no fault recovery or current escalation.
            report['zero_order']=list(ZERO_ORDER)
            diagnostic_axis=1
            for mid in ZERO_ORDER:
                p=session.plans[mid];session.targets[mid]=0.;begin=time.monotonic()
                timeout=4+abs(p['start'])/p['speed']*3
                settled_since=None;progress_stamp=begin;previous_error=abs(p['start']);last_print=begin;last_telemetry=begin
                while time.monotonic()-begin<timeout:
                    session.pump(.02);actual=session.read(mid,0x7019);session.require(mid,2)
                    for other in session.active-{mid}:
                        if abs(session.require(other,2)['position']-session.targets[other])>.007:
                            raise RuntimeError(f'Axis {other}: hold drift during zero return {mid}')
                    now=time.monotonic()
                    if now-last_telemetry>=.25:
                        telemetry=dict(motor=mid,time=now,position_rad=actual,
                            iq_filtered_a=session.read(mid,0x701a),
                            reference_rad=session.read(mid,0x7016),
                            bus_voltage_v=session.read(mid,0x701c))
                        report.setdefault('zero_telemetry',[]).append(telemetry)
                        stamp=time.monotonic()
                        measured_position=session.read(diagnostic_axis,0x7019)
                        measured_velocity=session.read(diagnostic_axis,0x701b)
                        report.setdefault('velocity_crosschecks',[]).append(dict(motor=diagnostic_axis,
                            begin_time=stamp,end_time=time.monotonic(),position_rad=measured_position,
                            velocity_parameter_rad_s=measured_velocity,type2=dict(session.require(diagnostic_axis,2))))
                        diagnostic_axis=diagnostic_axis%7+1
                        if not readback_matches(0x7016,0.,telemetry['reference_rad']):
                            raise RuntimeError(f'Axis {mid}: zero reference readback mismatch')
                        last_telemetry=now
                    settled_since=(now if settled_since is None else settled_since) if abs(actual)<.002 else None
                    if settled_since is not None and now-settled_since>=2.:break
                    if now-last_print>2:
                        print(json.dumps({'event':'return_zero_progress','motor':mid,'actual_rad':actual,'target_rad':0.,'elapsed_s':now-begin}),flush=True);last_print=now
                    if now-progress_stamp>=3:
                        error=abs(actual)
                        if error>.003 and previous_error-error<.001:raise RuntimeError(f'Axis {mid}: no progress toward zero; no automatic current increase')
                        progress_stamp=now;previous_error=error
                else:raise TimeoutError(f'Axis {mid}: zero target not reached')
                report['stages'].append(dict(stage='zero',motor=mid,target=0.,actual=actual,error=actual,elapsed=time.monotonic()-begin))
                print(json.dumps({'event':'axis_at_zero','motor':mid,'actual_rad':actual}),flush=True)
            session.pump(.5)
            report['zero_positions_before_stop']={str(mid):session.read(mid,0x7019) for mid in range(1,8)}
            if any(abs(q)>.003 for q in report['zero_positions_before_stop'].values()):raise RuntimeError('Final zero hold tolerance exceeded')
        report['status']='PASS_SEVEN_SUPPORTED_'+action.upper()
    except Exception as exc:report.update(status='FAIL',error=repr(exc))
    finally:
        if started:
            try:session.stop_all()
            except Exception as exc:report['stop_errors']['controller']=repr(exc);dev.stop()
            if report['stop_errors'] or len(report['reset_confirmed'])!=7:report['status']='FAIL'
        dev.close()
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['probe','hold','move','zero'])
    parser.add_argument('--supported-attended',action='store_true',help='Required现场承托与监护确认')
    parser.add_argument('--output',required=True)
    parser.add_argument('--move-axes',default='1,2,3,4,5,6,7',help='Ordered subset for motion; all seven are still held')
    args=parser.parse_args()
    move_axes=[int(x) for x in args.move_axes.split(',')]
    if not args.supported_attended:parser.error('Supported arm and attending operator required, including reset probe')
    def interrupted(*unused):raise RuntimeError('Interrupted; resetting supported arm')
    signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGINT,interrupted)
    with open('/tmp/kv260_gripper_commission.lock','w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);result=run(args.action,move_axes)
    Path(args.output).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('commands','frames','feedback')},indent=2))
    raise SystemExit(0 if result['status'].startswith('PASS') else 1)
