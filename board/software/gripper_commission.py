#!/usr/bin/python3
"""Bounded motor-7 PP commissioning. --probe never enables or writes params.

--move requires an attended, clear gripper and stable disabled arm. All targets
are relative to freshly measured position, clipped inside measured endpoints.
Only motor 7 is addressed. No Flash save, zeroing or fault-clear commands.
"""
import argparse
import fcntl
import json
import math
from pathlib import Path
import signal
import struct
import time

from a3_protocol import enable, disable, decode_feedback
from canctl import CanDevice
from joint_safety import GripperRange

SPEED, ACCEL, CURRENT, TIMEOUT_TICKS = 0.08, 0.2, 0.3, 6000


def parameter(index, value=None, integer=False):
    payload = struct.pack('<H', index)+bytes(2)
    if value is None:
        return 0x1100fd07, payload+bytes(4)
    if integer:
        if type(value) is not int or not 0 <= value <= 0xffffffff: raise ValueError('uint32 required')
        data = struct.pack('<I', value)
    else:
        if not math.isfinite(value): raise ValueError('finite float required')
        data = struct.pack('<f', value)
    return 0x1200fd07, payload+data


def targets(position, closed, opened):
    bounds = GripperRange(closed, opened)
    if not bounds.lower <= position <= bounds.upper: raise ValueError('position outside measured endpoints')
    # Move towards the interior, never towards the nearest mechanical stop.
    sign = 1 if position < (bounds.lower+bounds.upper)/2 else -1
    end = position+sign*0.05
    if not bounds.lower+0.01 <= end <= bounds.upper-0.01: raise ValueError('insufficient interior travel')
    return end, max(bounds.lower, min(position,end)-0.01), min(bounds.upper, max(position,end)+0.01)


def repeat_targets(position, closed, opened):
    bounds=GripperRange(closed,opened)
    if not bounds.lower+.01 <= position <= bounds.upper-.01:
        raise ValueError('Return position must be inside endpoints with 0.01 rad margin')
    end,lower,upper=targets(position,closed,opened)
    return [end,position]*3,lower,upper


class Session:
    def __init__(self, device, report):
        self.dev, self.report = device, report
        self.feedback = None
        self.feedback_time = 0.
        self.responses = []

    def pump(self, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            self.dev.heartbeat()
            if self.dev.read(0x418): raise RuntimeError('PL fault: '+str(self.dev.read(0x418)))
            if self.dev.read(14*4) or self.dev.read(15*4): raise RuntimeError('CAN RX/TX error')
            frame = self.dev.receive()
            if frame:
                frame['host_monotonic_s'] = time.monotonic()
                self.report['frames'].append(frame)
                if frame['extended'] and not frame['rtr'] and frame['id'] >> 24 == 2:
                    feedback = decode_feedback(frame['id'], bytes.fromhex(frame['data']), model='EL05')
                    if feedback['motor'] != 7: raise RuntimeError('Unexpected other motor feedback')
                    self.feedback = feedback
                    self.feedback_time = time.monotonic()
                    self.report['feedback'].append({**feedback, 'host_monotonic_s': self.feedback_time})
                    if feedback['fault']: raise RuntimeError('Motor fault: '+str(feedback))
                else:
                    self.responses.append(frame)
            else: time.sleep(.0003)

    def send(self, packet):
        ident, data = packet
        if ident & 255 != 7: raise ValueError('This tool can only address motor 7')
        self.dev.heartbeat()
        self.dev.enqueue(ident, data, ttl_us=20000)
        self.report['commands'].append({'id':ident,'data':data.hex(),'host_monotonic_s':time.monotonic()})

    def read(self, index, integer=False):
        self.responses.clear()
        self.send(parameter(index))
        end = time.monotonic()+0.08
        while time.monotonic() < end:
            self.pump(.003)
            for f in self.responses:
                payload=bytes.fromhex(f['data'])
                if f['extended'] and not f['rtr'] and f['id']==0x110007fd and len(payload)==8 and struct.unpack('<H',payload[:2])[0]==index:
                    value=struct.unpack('<I' if integer else '<f',payload[4:])[0]
                    if not math.isfinite(value): raise ValueError('Nonfinite parameter reply')
                    return value
        raise TimeoutError('No parameter reply for '+hex(index))

    def write_verified(self, index, value, integer=False):
        self.send(parameter(index,value,integer));self.pump(.01)
        actual=self.read(index,integer)
        if (actual != value if integer else not math.isclose(actual,value,rel_tol=1e-5,abs_tol=1e-6)):
            raise RuntimeError(f'Parameter readback mismatch {index:#x}: wanted {value}, got {actual}')
        self.report['parameter_readbacks'][hex(index)]=actual

    def require_feedback(self, mode=None):
        if self.feedback is None or time.monotonic()-self.feedback_time>0.08:
            raise RuntimeError('Missing/stale real Type2 feedback')
        if self.feedback['fault'] or (mode is not None and self.feedback['mode']!=mode):
            raise RuntimeError('Unexpected motor state: '+str(self.feedback))


def run(move, repeat=False):
    report={'status':'RUNNING','motor':7,'attended_move_requested':move,'commands':[],
            'frames':[],'feedback':[],'parameter_readbacks':{},'motion_commands_sent':0,
            'enable_commands_sent':0,'flash_writes_sent':0,'zero_commands_sent':0,
            'legs':[],'velocity_crosscheck':[],'repeat_requested':repeat}
    dev=CanDevice(); session=Session(dev,report)
    enabled=False
    try:
        dev.configure('normal')
        session.send(disable(7));session.pump(.03);session.require_feedback(mode=0)
        report['initial_position_rad']=initial=session.read(0x7019)
        report['initial_parameters']={hex(index):session.read(index,integer)
            for index,integer in [(0x7005,True),(0x7016,False),(0x7024,False),(0x7025,False),(0x7018,False),(0x7028,True)]}
        if not move:
            report['status']='PASS_PROBE_RESET_TYPE2_AND_READ_ONLY_PARAMETERS'
            return report
        if dev.read(0x470) not in (0x30000,0x40000):raise RuntimeError('Need v3 PP hardware guard')
        config=json.loads(Path(__file__).with_name('a3_commissioning.json').read_text())
        g=config['gripper'];target,lower,upper=targets(initial,g['closed_motor_rad'],g['open_motor_rad'])
        sequence=[target];speed=SPEED
        if repeat:
            sequence,lower,upper=repeat_targets(initial,g['closed_motor_rad'],g['open_motor_rad'])
            speed=.04
        report['plan']={'start_rad':initial,'target_rad':target,'delta_rad':target-initial,
            'guard_min_rad':lower,'guard_max_rad':upper,'speed_rad_s':speed,
            'acceleration_rad_s2':ACCEL,'current_limit_a':CURRENT,'timeout_ticks':TIMEOUT_TICKS,
            'targets_rad':sequence,'software_velocity_abort_rad_s':.15 if repeat else .2}
        dev.stop();dev.write(0x5a0,64);dev.write(0x5a4,100000);dev.write(0x680,1)
        expected={0x684:lower,0x688:upper,0x68c:speed,0x690:ACCEL,0x694:CURRENT}
        for address,value in expected.items():
            bits=struct.unpack('<I',struct.pack('<f',value))[0];dev.write(address,bits)
            if dev.read(address)!=bits:raise RuntimeError('PL PP configuration readback mismatch')
        dev.configure('normal')
        session.send(disable(7));session.pump(.02);session.require_feedback(mode=0)
        fresh=session.read(0x7019)
        if abs(fresh-initial)>.002:raise RuntimeError('Gripper moved during setup')
        session.write_verified(0x7005,1,True)
        session.write_verified(0x7024,speed)
        session.write_verified(0x7025,ACCEL)
        session.write_verified(0x7018,CURRENT)
        session.write_verified(0x7028,TIMEOUT_TICKS,True)
        # Refresh reset feedback before the guarded hold target and enable.
        # Type18 replies are expected to update real Type2 during setup.
        session.require_feedback(mode=0)
        session.write_verified(0x7016,initial)
        session.require_feedback(mode=0)
        report['prepared_mask']=dev.read(0x698)
        session.send(enable(7));enabled=True;session.pump(.02);session.require_feedback(mode=2)
        # Hold current position first. Abort if enable itself causes movement.
        for _ in range(5):
            session.send(parameter(0x7016,initial));session.pump(.02);session.require_feedback(mode=2)
            if abs(session.feedback['position']-initial)>.01:raise RuntimeError('Unexpected displacement on enable')
        for number,target in enumerate(sequence,1):
            begin=time.monotonic();settled=0;first_feedback=len(report['feedback'])
            limit=.15 if repeat else .2
            while time.monotonic()-begin<(3.5 if repeat else 2.5):
                session.send(parameter(0x7016,target));session.pump(.02);session.require_feedback(mode=2)
                p=session.feedback['position']
                if not lower-.002<=p<=upper+.002 or abs(session.feedback['velocity'])>limit:
                    raise RuntimeError('Feedback position/velocity envelope violated')
                if repeat:
                    fb=dict(session.feedback);stamp=session.feedback_time
                    read_begin=time.monotonic();velocity=session.read(0x701b)
                    report['velocity_crosscheck'].append({'leg':number,'type2_velocity_rad_s':fb['velocity'],
                        'type2_host_time':stamp,'parameter_velocity_rad_s':velocity,
                        'parameter_read_begin':read_begin,'parameter_read_end':time.monotonic()})
                    if abs(velocity)>limit:raise RuntimeError('MECH_VEL exceeds software abort threshold')
                if abs(p-target)<.005 and abs(session.feedback['velocity'])<.03:settled+=1
                else:settled=0
                if settled>=5:break
            else:raise TimeoutError('Target not reached within bounded time; no automatic current increase')
            actual=session.read(0x7019)
            if abs(actual-target)>.005:raise RuntimeError('Final MECH_POS outside target tolerance')
            report['legs'].append({'number':number,'target_rad':target,'actual_rad':actual,
                'error_rad':actual-target,'elapsed_s':time.monotonic()-begin,
                'feedback_begin':first_feedback,'feedback_end':len(report['feedback'])})
        report['final_position_before_disable_rad']=actual
        report['status']='PASS_THREE_GRIPPER_ROUND_TRIPS' if repeat else 'PASS_SMALL_GRIPPER_PP_MOVE'
    except Exception as exc:
        report['status']='FAIL';report['error']=repr(exc)
    finally:
        try:
            if enabled or move:
                if dev.read(0x418) or dev.read(0x404)&3!=3:dev.configure('normal')
                session.send(disable(7));session.pump(.03);session.require_feedback(mode=0)
                report['disable_confirmed']=True
            dev.stop();dev.write(0x5a0,0)
            if dev.read(0x470) in (0x30000,0x40000):dev.write(0x680,0)
            report['final']=dev.status()
            report['final_motion_mask']=dev.read(0x5a0)
        except Exception as exc:
            report['shutdown_error']=repr(exc);report['status']='FAIL'
            dev.stop();dev.write(0x5a0,0)
            if dev.read(0x470) in (0x30000,0x40000):dev.write(0x680,0)
            report['final']=dev.status();report['final_motion_mask']=dev.read(0x5a0)
        finally:dev.close()
        report['enable_commands_sent']=sum(c['id']>>24==3 for c in report['commands'])
        report['parameter_writes_sent']=sum(c['id']>>24==18 for c in report['commands'])
        report['motion_commands_sent']=sum(c['id']>>24==18 and c['data'].startswith('1670') for c in report['commands'])
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    action=parser.add_mutually_exclusive_group(required=True)
    action.add_argument('--probe',action='store_true');action.add_argument('--move',action='store_true')
    action.add_argument('--repeat',action='store_true',help='Three 0.05rad round trips at 0.04rad/s; motor 7 only')
    parser.add_argument('--output',required=True);args=parser.parse_args()
    def interrupted(*unused):raise RuntimeError('Interrupted; stopping motor 7')
    signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGINT,interrupted)
    with open('/tmp/kv260_gripper_commission.lock','w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        result=run(args.move or args.repeat,repeat=args.repeat)
    Path(args.output).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('frames','feedback','commands','velocity_crosscheck')},indent=2))
    raise SystemExit(0 if result['status'].startswith('PASS') else 1)
