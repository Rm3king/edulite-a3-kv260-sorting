#!/usr/bin/python3
"""Read only Type17 parameters from arm motors 1..6. No enable/disable/writes.

RUN_MODE is a configured control mode, NOT an enabled/reset state indication.
"""
import fcntl
import json
import math
import struct
import time
from canctl import CanDevice
from joint_safety import motor_to_joint, LIMITS

PARAMETERS = ((0x7005, 'run_mode_config', True), (0x7019, 'position_motor_rad', False),
              (0x701b, 'velocity_parameter_rad_s', False), (0x7018, 'current_limit_a', False),
              (0x7028, 'can_timeout_ticks', True))


def decode(frame, motor, index, integer):
    data=bytes.fromhex(frame['data'])
    if not (frame['extended'] and not frame['rtr'] and frame['id']==0x110000fd+(motor<<8)
            and len(data)==8 and struct.unpack('<H',data[:2])[0]==index):return None
    value=struct.unpack('<I' if integer else '<f',data[4:])[0]
    if not math.isfinite(value):raise ValueError('Nonfinite parameter')
    return value


def run():
    report={'status':'RUNNING','enable_commands_sent':0,'disable_commands_sent':0,
            'parameter_writes_sent':0,'transactions':[],'motors':{},
            'note':'Type17 snapshots only; run_mode_config does not prove servo enable state'}
    dev=CanDevice()
    try:
        if dev.read(0x404)!=0 or dev.read(0x5a0)!=0:
            raise RuntimeError('Probe requires stopped controller and no motion permission')
        dev.configure('normal')
        for motor in range(1,7):
            values={}
            for index,name,integer in PARAMETERS:
                ident=0x1100fd00|motor;data=struct.pack('<H',index)+bytes(6)
                dev.heartbeat();dev.enqueue(ident,data)
                transaction={'motor':motor,'parameter':hex(index),'request_id':ident,'request_data':data.hex()}
                report['transactions'].append(transaction)
                deadline=time.monotonic()+.06
                while time.monotonic()<deadline:
                    dev.heartbeat()
                    if dev.read(0x418) or dev.read(14*4) or dev.read(15*4):raise RuntimeError('CAN fault/error')
                    frame=dev.receive()
                    if frame:
                        value=decode(frame,motor,index,integer)
                        if value is not None:
                            transaction.update(response=frame,value=value);values[name]=value;break
                    else:time.sleep(.0003)
                else:raise TimeoutError(f'No reply motor={motor} index={index:#x}')
            q=motor_to_joint(motor,values['position_motor_rad'])
            values.update(position_joint_rad=q,within_sdk_joint_limits=LIMITS[motor][0]<=q<=LIMITS[motor][1])
            report['motors'][str(motor)]=values
        report['status']='PASS_30_READ_ONLY_PARAMETERS'
    except Exception as exc:
        report.update(status='FAIL',error=repr(exc))
    finally:
        dev.stop();report['final']=dev.status();dev.close()
    return report


if __name__=='__main__':
    with open('/tmp/kv260_gripper_commission.lock','w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        result=run()
    print(json.dumps(result,indent=2,allow_nan=False))
    raise SystemExit(0 if result['status'].startswith('PASS') else 1)
