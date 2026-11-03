#!/usr/bin/python3
"""Pure A3 classic-CAN encoding. No driver imports, sockets, or motor writes.

These are motor-coordinate protocol limits, not calibrated robot joint limits.
Physical commands require the separate measured-joint/limit/control-mode checks.
"""
import math
import struct
RANGES={'RS00':(33.0,14.0),'EL05':(50.0,6.0),'RS05':(50.0,5.5)}
def motor_id(value):
    if type(value) is not int or not 1<=value<=7:raise ValueError('A3 motor ID must be 1..7')
    return value

def quantize(value, lo, hi):
    if not math.isfinite(value) or not lo<=value<=hi:raise ValueError('non-finite/out-of-range command')
    return int((value-lo)*65535/(hi-lo))

def motion(motor,position,velocity,kp,kd,torque=0.0,model='RS00'):
    motor_id(motor);vmax,tmax=RANGES[model]
    tq=quantize(torque,-tmax,tmax)
    return ((1<<24)|(tq<<8)|motor,struct.pack('>HHHH',quantize(position,-12.57,12.57),
            quantize(velocity,-vmax,vmax),quantize(kp,0,500),quantize(kd,0,5)))

def enable(motor,host=0xfd):return ((3<<24)|(host<<8)|motor_id(motor),bytes(8))
def disable(motor,host=0xfd):return ((4<<24)|(host<<8)|motor_id(motor),bytes(8))
def read_position(motor,host=0xfd):return ((17<<24)|(host<<8)|motor_id(motor),struct.pack('<H',0x7019)+bytes(6))

def decode_feedback(identifier,data,model='RS00',host=0xfd):
    if not 0<=identifier<=0x1fffffff or identifier>>24!=2 or identifier&255!=host or len(data)!=8:
        raise ValueError('not an A3 Type2 frame addressed to this host')
    motor=(identifier>>8)&255;motor_id(motor)
    p,v,t,temp=struct.unpack('>HHHH',data);vmax,tmax=RANGES[model]
    return dict(motor=motor,position=p*25.14/65535-12.57,velocity=v*2*vmax/65535-vmax,
                torque=t*2*tmax/65535-tmax,temperature=temp/10,
                mode=(identifier>>22)&3,fault=(identifier>>16)&63)
