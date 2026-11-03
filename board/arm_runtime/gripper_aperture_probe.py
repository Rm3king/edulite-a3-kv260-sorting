"""Attended, empty gripper opening with all arm axes disabled. Offline default.

This establishes a measured aperture, not a grasp or a calibrated mm mapping.
Only motor 7 may be addressed; stopping it cannot remove arm holding torque
because the six arm axes must remain disabled throughout this procedure.
"""
import argparse
import fcntl
import json
import math
from pathlib import Path
import signal
import struct
import time
from calibration_motion import motion_gc_scope
from canctl import CanDevice
from gripper_commission import Session, parameter
from a3_protocol import enable, disable

CLOSED = 1.677834153175354
OPEN = -.005880276206880808


def validate(start, target):
    if not all(math.isfinite(x) for x in (start, target)):
        raise ValueError('Finite motor angles required')
    if not CLOSED-.06 <= start <= CLOSED+.025:
        raise ValueError('Only an observed empty near-closed start is supported')
    if not .35 <= start-target <= .55 or not OPEN+.05 < target < CLOSED-.1:
        raise ValueError('Only a moderate inward opening is supported')
    return dict(observed_start_rad=start, initial_target_rad=min(start, CLOSED-.000001),
                target_rad=target, lower=target-.02, upper=CLOSED,
                speed=.08, acceleration=.2, current=.3, width_mm=None)


def validate_working_open(start):
    """Empty jaws only: restore the already commissioned 1.12rad opening.

    The jaws may be left wider (e.g. pushed by hand) or narrower than 1.12; the
    move is the same slow, current-limited empty-jaw positioning either way.
    """
    if not math.isfinite(start) or not 1.0<=start<=1.70:
        raise ValueError('Observed empty opening must be between 1.0rad and closed')
    return dict(observed_start_rad=start,initial_target_rad=min(start,CLOSED-.000001),
                target_rad=1.12,lower=min(start,1.12)-.02,upper=CLOSED,speed=.08,
                acceleration=.2,current=.3,width_mm=None)


class GripperOnlyDevice:
    def __init__(self, device):
        self.device=device
        self.last_heartbeat=None
        self.max_gap=0.

    def __getattr__(self, name):
        return getattr(self.device, name)

    def heartbeat(self):
        now=time.monotonic()
        if self.last_heartbeat is not None:self.max_gap=max(self.max_gap, now-self.last_heartbeat)
        self.last_heartbeat=now
        return self.device.heartbeat()

    def enqueue(self, identifier, payload, *args, **kwargs):
        kind=identifier>>24
        if identifier&255 != 7 or kind not in (3,4,17,18):
            raise ValueError('Only motor 7 PP control permitted')
        if kind in (17,18):
            index=int.from_bytes(bytes(payload)[:2], 'little')
            allowed=(0x7005,0x7016,0x7018,0x7024,0x7025,0x7028) if kind==18 else (0x7005,0x7016,0x7018,0x7019,0x7024,0x7025,0x7028)
            if index not in allowed:raise ValueError('Unreviewed parameter')
        return self.device.enqueue(identifier,payload,*args,**kwargs)

    def write(self,address,value):
        if address==0x5a0 and value not in (0,64):raise ValueError('Only gripper permit')
        if 0x5c0<=address<0x680 or 0x800<=address<0xd00:raise ValueError('Arm/trajectory writes forbidden')
        return self.device.write(address,value)


def run(plan, out):
    out=Path(out)
    out.mkdir(exist_ok=False,parents=True)
    report=dict(status='RUNNING',plan=plan,commands=[],frames=[],feedback=[],parameter_readbacks={})
    with motion_gc_scope(), open('/tmp/kv260_gripper_commission.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        d=GripperOnlyDevice(CanDevice());s=Session(d,report);owned=False
        try:
            if d.read(0x470) not in (0x60002,0x60003) or any(d.read(x) for x in (0x404,0x418,0x5a0,0x5a8,0x800,0xb40)):
                raise RuntimeError('Released stopped controller and disabled arm required')
            d.configure('normal');owned=True
            s.send(disable(7));s.pump(.03);s.require_feedback(0)
            actual=s.read(0x7019);report['actual_start_rad']=actual
            if abs(actual-plan['observed_start_rad'])>.002:raise RuntimeError('Gripper changed since review')
            d.stop();d.write(0x5a0,64);d.write(0x5a4,100000);d.write(0x680,1);d.write(0x69c,0)
            if d.read(0x69c)!=0:raise RuntimeError('Motor 7 PP mode guard required')
            for address,value in [(0x684,plan['lower']),(0x688,plan['upper']),(0x68c,plan['speed']),(0x690,plan['acceleration']),(0x694,plan['current'])]:
                bits=struct.unpack('<I',struct.pack('<f',value))[0];d.write(address,bits)
                if d.read(address)!=bits:raise RuntimeError('Guard readback')
            d.configure('normal')
            for index,value,integer in [(0x7005,1,True),(0x7024,plan['speed'],False),(0x7025,plan['acceleration'],False),(0x7018,plan['current'],False),(0x7028,6000,True)]:
                s.write_verified(index,value,integer)
            # Disabled loc_ref may mirror MECH_POS outside the old closed
            # endpoint. Command only the legal inward value, never expand limits.
            s.require_feedback(0);s.send(parameter(0x7016,plan['initial_target_rad']));s.pump(.005)
            if d.read(0x698)!=63:raise RuntimeError('PP preparation incomplete')
            s.send(enable(7));s.pump(.02);s.require_feedback(2)
            for name,target,timeout in [('initial_inward_hold',plan['initial_target_rad'],2.),('moderate_open',plan['target_rad'],9.)]:
                begin=time.monotonic();stable_since=None;previous=None
                while time.monotonic()-begin<timeout:
                    s.send(parameter(0x7016,target));s.pump(.02);s.require_feedback(2)
                    f=s.feedback
                    if d.read(0x5a8)!=64 or d.read(0x5a0)!=64:raise RuntimeError('Unexpected active arm')
                    # One velocity estimate can spike on its own (seen: -0.31 rad/s while the
                    # position changed at 0.08 rad/s); only count it with position support.
                    stamp=s.feedback_time
                    moved=None if previous is None or stamp<=previous[1] else abs(f['position']-previous[0])/(stamp-previous[1])
                    previous=(f['position'],stamp)
                    fast=abs(f['velocity'])>.2 and (moved is None or moved>.1)
                    if not plan['lower']-.002<=f['position']<=max(actual,plan['upper'])+.003 or fast:
                        raise RuntimeError('Gripper envelope/velocity violation')
                    if abs(f['position']-target)<.005 and abs(f['velocity'])<.03:
                        stable_since=stable_since or time.monotonic()
                        if time.monotonic()-stable_since>=.2:break
                    else:stable_since=None
                else:raise TimeoutError(name+' did not settle; no current increase')
                report.setdefault('legs',[]).append(dict(name=name,target_rad=target,actual_rad=s.read(0x7019)))
            report['status']='PASS_MODERATE_OPEN_ONLY_WIDTH_UNMEASURED'
        except (Exception,KeyboardInterrupt) as error:
            report.update(status='FAIL',error=repr(error))
        finally:
            if owned:
                # Motor 7 only, empty jaws, arm disabled: bounded stop is safe.
                try:
                    s.send(disable(7));s.pump(.04);s.require_feedback(0)
                    report['disabled_confirmed']=True
                    report['final_motor_rad']=s.read(0x7019)
                    d.stop();d.write(0x5a0,0);d.write(0x680,0)
                except Exception as error:
                    report.update(status='FAIL_STOP_CONFIRMATION',stop_error=repr(error))
            report['final_controller']=d.status()
            report['active_mask']=d.read(0x5a8);report['motion_mask']=d.read(0x5a0)
            report['heartbeat_max_gap_s']=d.max_gap
            d.close()
            (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start',type=float,required=True)
    parser.add_argument('--target',type=float,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--execute-attended',action='store_true')
    a=parser.parse_args();plan=validate(a.start,a.target)
    if not a.execute_attended:print(json.dumps(dict(status='VALIDATED_NO_HARDWARE',plan=plan)));return
    def interrupt(*args):raise KeyboardInterrupt('Stop empty gripper only')
    signal.signal(signal.SIGTERM,interrupt)
    r=run(plan,a.out)
    print(json.dumps({k:r.get(k) for k in ('status','error','final_motor_rad','disabled_confirmed','heartbeat_max_gap_s')}))


if __name__=='__main__':main()
