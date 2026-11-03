#!/usr/bin/python3
"""KV260 PL CAN control via a named UIO device. Default command is read-only.

No socket opens and no CAN writes occur on import. selftest disconnects the
physical TX output in hardware and loops the CAN core internally.
"""
import argparse
import ctypes
import json
import mmap
import math
import os
from pathlib import Path
import struct
import time

MASK=0xffffffff
SAFE_SUBMIT_ABI=0x60003
TUNING_FLOAT_BOUNDS={0x701f:(0x40800000,0x40c00000),
                     0x7020:(0x3c23d70a,0x3ca3d70a),
                     0x7021:(0x3d4ccccd,0x3e19999a)}
class CommandRejectedError(RuntimeError):
    """PL did not enqueue the frame; ordinary refusal, not a bus exception."""
    pass
def version_mit_running(dev):
    # Guard before touching a register absent from old images (can SIGBUS).
    return dev.read(0x470) in (0x60000,0x60001,0x60002,0x60003) and bool(dev.read(0xb40)&1)
class CanDevice:
    def __init__(self, uio=None):
        if uio is None:
            matches=[p.parent.name for p in Path('/sys/class/uio').glob('uio*/name')
                     if p.read_text().strip()=='kv260-can']
            if len(matches)!=1:
                raise RuntimeError(f'Expected one kv260-can UIO device, found {matches}; load matching firmware first')
            uio='/dev/'+matches[0]
        # Use explicit volatile 32-bit accesses: Python struct packing may clear
        # the destination before packing, causing extra MMIO writes/side effects.
        library=Path(__file__).with_name('libkv260_mmio.so')
        if not library.is_file():
            raise RuntimeError('Build libkv260_mmio.so from mmio.c on KV260 first; see README')
        self.io=ctypes.CDLL(str(library))
        self.io.mmio_write32.argtypes=[ctypes.c_void_p,ctypes.c_uint,ctypes.c_uint32]
        self.io.mmio_write32.restype=None
        self.io.mmio_read32.argtypes=[ctypes.c_void_p,ctypes.c_uint]
        self.io.mmio_read32.restype=ctypes.c_uint32
        self.fd=os.open(uio,os.O_RDWR|os.O_SYNC)
        self.mem=mmap.mmap(self.fd,4096,mmap.MAP_SHARED,mmap.PROT_READ|mmap.PROT_WRITE,offset=0)
        self.base=ctypes.addressof(ctypes.c_char.from_buffer(self.mem))
        if self.read(0x400)!=0x43414e31:
            self.close();raise RuntimeError('CAN firmware signature mismatch; no registers written')
    def close(self):
        self.mem.close();os.close(self.fd)
    def read(self, offset):
        if offset%4 or not 0<=offset<4096:raise ValueError('Aligned UIO offset required')
        if offset in (0x474,0x478) and self.read(0x470)!=SAFE_SUBMIT_ABI:
            raise ValueError('Safe submission status requires ABI 0x60003')
        return self.io.mmio_read32(self.base,offset)
    def write(self, offset, value):
        if offset%4 or not 0<=offset<4096:raise ValueError('Aligned UIO offset required')
        # Route every ordinary driver TX commit through a checked non-aborting
        # mailbox on v6.3, including callers which still use legacy 0x458.
        if offset in (0x458,0x45c):
            version=self.read(0x470)
            if offset==0x45c and version!=SAFE_SUBMIT_ABI:
                raise ValueError('Safe submission requires ABI 0x60003')
            if version==SAFE_SUBMIT_ABI:
                sequence=self.read(0x478)
                self.io.mmio_write32(self.base,0x45c,value&MASK)
                status=self.read(0x474);after=self.read(0x478)
                if after!=((sequence+1)&MASK):
                    raise RuntimeError('CAN submission ownership/sequence mismatch')
                if status:
                    raise CommandRejectedError(f'PL refused CAN frame: status={status}, sequence={after}; no packet enqueued')
                return
        # Match trajectory_stream / mit_trajectory_stream writable registers.
        # Writing status (notably 0x800 / 0xb40) returns AXI SLVERR and can
        # cause an unrecoverable A53 external abort. Reject before any MMIO.
        # 840/848/84c/850..864/8a0..8b4: online correction channel. On that firmware a refused write is ignored
        # and counted by the PL, never SLVERR. Older firmware has no such registers (any access there is SLVERR,
        # reads included), so callers first check correction_channel_present(): the deployment bundle that ships a
        # bitstream with the channel carries correction_channel.json bound to that bitstream's hash.
        correction=(offset in (0x840,0x848,0x84c) or 0x850<=offset<=0x864 or 0x8a0<=offset<=0x8b4) and _correction_bundle()
        if offset>=0x800 and not (offset in (0x820,0xb60) or correction
                or 0x900<=offset<=0x9b0 or 0xa00<=offset<=0xb20
                or 0xc00<=offset<=0xf6c):
            raise ValueError('Read-only or reserved trajectory register')
        # These registers return AXI SLVERR when configured while CAN runs.
        # Reject in Python before MMIO; an external bus abort is not a normal
        # recoverable Python exception on every deployed A53 kernel.
        standby=(offset in (0x5a0,0x5a4) or 0x600<=offset<0x670
                 or 0x680<=offset<=0x69c or 0x700<=offset<0x7c0)
        if standby:
            if self.read(0x404)!=0:raise ValueError('Stop CAN before configuring motion guard registers')
            if offset==0x698 or 0x700<=offset<0x7c0 and offset%32==24:
                raise ValueError('PP readiness register is read-only')
            if offset==0x5a4 and not 0<(value&MASK)<=1000000:
                raise ValueError('Feedback timeout must be 1..1000000 us')
            if (offset==0x69c or 0x700<=offset<0x7c0 and offset%32==28) and (value&MASK)>2:
                raise ValueError('Unsupported protected motor mode')
        self.io.mmio_write32(self.base,offset,value&MASK)
    def status(self):
        names={0x400:'magic',0x404:'control',0x408:'time_us',0x40c:'core_status',0x410:'lease_age_us',
               0x414:'lease_us',0x418:'fault',0x41c:'queue_counts',0x420:'submitted',0x424:'expired',
               0x428:'received',0x42c:'rx_overflow',0x430:'last_submit_us',0x434:'last_lateness_us',0x49c:'motor_seen'}
        return {v:self.read(k) for k,v in names.items()}
    def heartbeat(self): self.write(0x410,1)
    def configure(self, mode='listen'):
        if mode not in ('listen','normal','internal'):raise ValueError(mode)
        self.write(0x404,0);time.sleep(.002)
        self.write(0x418,0)
        self.write(31*4,0x80)
        self.write(0,0x09)
        self.write(6*4,4);self.write(7*4,0x16)
        for i in range(16,20):self.write(i*4,0)
        for i in range(20,24):self.write(i*4,255)
        self.write(0,{'listen':0x0a,'normal':0x08,'internal':0x0c}[mode])
        wanted={'listen':0x0a,'normal':0x08,'internal':0x0c}[mode]
        actual=[self.read(off) for off in (31*4,0,6*4,7*4)]
        if actual != [0x80,wanted,4,0x16]:
            self.stop();raise RuntimeError(f'CAN configuration readback mismatch: {actual}')
        self.write(0x414,100000)
        self.write(0x404,{'listen':4,'normal':7,'internal':15}[mode])
    def stop(self): self.write(0x404,0)
    def check_motion_guard(self, identifier, payload):
        """Preflight v2/v3/v4 guarded frames before MMIO writes (SLVERR can SIGBUS).

        Hardware repeats these checks at commit and monitors feedback thereafter.
        This cannot eliminate a feedback/fault race between preflight and commit.
        """
        kind=identifier>>24
        if kind==26 and not self.read(0x404)&8:
            raise ValueError('Type26 version replies alias Type2 feedback; unsupported by current feedback table')
        if self.read(0x404)&8:return
        if version_mit_running(self):
            if kind in (1,18):raise ValueError('PL owns MIT/CSP targets while stream runs')
        if kind==18:
            if self.read(0x470)>=0x50000 and self.read(0x800)&1:raise ValueError('PL owns targets while trajectory runs')
            self.check_pp_guard(identifier,payload)
            return
        if kind not in (1,3):return
        version=self.read(0x470)
        if version not in (0x20000,0x30000,0x40000,0x50000,0x50001,0x60000,0x60001,0x60002,0x60003):raise RuntimeError('Motion requires known guard firmware')
        mid=identifier&255
        if not 1<=mid<=7 or not self.read(0x5a0)&(1<<(mid-1)):
            raise ValueError('Axis is not permitted for motion/enable')
        if kind==3:
            if payload!=bytes(8) or identifier>>8&65535!=0xfd:raise ValueError('Malformed enable')
            if mid==7 or version>=0x40000:
                base=self.pp_base(mid)
                if version not in (0x30000,0x40000,0x50000,0x50001,0x60000,0x60001,0x60002,0x60003) or not self.read(base) or self.read(base+24)!=63:
                    raise ValueError('Axis PP preparation incomplete')
                self.check_motor_feedback(mid)
            return
        if version>=0x40000 and self.read(self.pp_base(mid))&1:
            bank=self.pp_base(mid)
            if version not in (0x60000,0x60001,0x60002,0x60003) or self.read(bank+28)!=2 or self.read(bank+24)&31!=31:
                raise ValueError('Type1 blocked outside prepared MIT bank')
        if len(payload)!=8:raise ValueError('Type1 requires DLC8')
        base=0x500+(mid-1)*16
        feedback_id=self.read(base);stamp=self.read(base+12)
        age=(self.read(0x408)-stamp)&MASK;timeout=self.read(0x5a4)
        if not self.read(0x49c)&(1<<(mid-1)) or (feedback_id>>16)&63 or not 0<=age<timeout:
            raise ValueError('Missing, stale or faulted Type2 feedback')
        p,v,kp,kd=struct.unpack('>HHHH',payload);torque=(identifier>>8)&65535
        words=[self.read(0x600+(mid-1)*16+i*4) for i in range(4)]
        if any(not (word&65535)<=value<=(word>>16) for value,word in zip((p,v,torque),words)) or kp>(words[3]&65535) or kd>(words[3]>>16):
            raise ValueError('Motion exceeds configured encoded limits')
    @staticmethod
    def pp_base(mid):
        if type(mid) is not int or not 1<=mid<=7:raise ValueError('Motor ID 1..7 required')
        return 0x680 if mid==7 else 0x700+(mid-1)*32
    def check_gripper_feedback(self):
        self.check_motor_feedback(7)
    def check_motor_feedback(self,mid):
        base=0x500+(mid-1)*16
        ident=self.read(base);age=(self.read(0x408)-self.read(base+12))&MASK
        if not self.read(0x49c)&(1<<(mid-1)) or (ident>>16)&63 or age>=self.read(0x5a4):
            raise ValueError('Missing/stale/faulted axis Type2 feedback')
    def check_pp_guard(self,identifier,payload):
        version=self.read(0x470);mid=identifier&255
        if version not in (0x30000,0x40000,0x50000,0x50001,0x60000,0x60001,0x60002,0x60003):raise ValueError('PP requires known per-axis guard')
        if identifier>>8!=0x1200fd or not 1<=mid<=7 or (version==0x30000 and mid!=7) or len(payload)!=8 or payload[2:4]!=bytes(2):
            raise ValueError('Only whitelisted axis PP writes permitted')
        base=self.pp_base(mid)
        if not self.read(base)&1 or not self.read(0x5a0)&(1<<(mid-1)):raise ValueError('PP not permitted')
        parameter=struct.unpack('<H',payload[:2])[0]
        integer=struct.unpack('<I',payload[4:])[0];value=struct.unpack('<f',payload[4:])[0]
        active=bool(self.read(0x5a8)&(1<<(mid-1)))
        mit=version in (0x60000,0x60001,0x60002,0x60003) and self.read(base+28)==2
        if mit and parameter not in (0x7005,0x700b,0x7028):raise ValueError('Parameter not allowed in MIT mode')
        if parameter==0x7016:
            lo,hi=[struct.unpack('<f',struct.pack('<I',self.read(a)))[0] for a in (base+4,base+8)]
            if not math.isfinite(value) or not lo<=value<=hi:raise ValueError('PP position outside configured range')
            self.check_motor_feedback(mid)
        elif active:raise ValueError('Cannot reconfigure active axis')
        elif parameter==0x7005:
            if integer!=(0 if mit else 5 if version>=0x50000 and self.read(base+28) else 1):raise ValueError('Run mode differs from protected bank')
        elif parameter==0x7028:
            if not 2000<=integer<=20000:raise ValueError('CAN timeout must be uint32 2000..20000')
        elif parameter==0x700b:
            maximum=struct.unpack('<f',struct.pack('<I',self.read(base+16)))[0]
            if not mit or not math.isfinite(value) or not 0<value<=maximum:raise ValueError('MIT torque limit outside configured cap')
        elif parameter==0x701e:
            if version not in (0x60001,0x60002,0x60003) or mid!=3 or self.read(base+28)!=1 or self.read(0x5a8) or not math.isfinite(value) or not 20<=value<=30:
                raise ValueError('CSP location gain requires v6.1, joint3, all disabled and gain20..30')
            self.check_motor_feedback(mid)
            if self.read(0x500+(mid-1)*16)>>22&3:raise ValueError('CSP gain requires fresh disabled motor state')
        elif parameter in TUNING_FLOAT_BOUNDS:
            low,high=TUNING_FLOAT_BOUNDS[parameter]
            if (version!=SAFE_SUBMIT_ABI or mid not in (2,3) or self.read(base+28)!=1
                    or self.read(0x5a8) or not low<=integer<=high):
                raise ValueError('Volatile speed tuning requires ABI6.3, J2/J3, disabled CSP and bounded gain')
            self.check_motor_feedback(mid)
            if self.read(0x500+(mid-1)*16)>>22&3:
                raise ValueError('Speed tuning requires fresh disabled motor state')
        elif parameter in (0x7017,0x7024,0x7025,0x7018):
            csp=version>=0x50000 and self.read(base+28)==1
            if (parameter==0x7017 and not csp) or (parameter in (0x7024,0x7025) and csp):
                raise ValueError('Speed/acceleration parameter differs from protected mode')
            maximum=self.read(base+{0x7017:12,0x7024:12,0x7025:16,0x7018:20}[parameter])
            if not math.isfinite(value) or value<=0 or integer>maximum:raise ValueError('PP speed/acceleration/current outside limits')
        else:raise ValueError('Parameter is not on PP whitelist')
    def enqueue(self, identifier, payload, due_us=None, ttl_us=20000):
        if not 0<=identifier<=0x1fffffff or len(payload)>8:raise ValueError('classic extended CAN only')
        if not 1<=ttl_us<=1000000:raise ValueError('TTL outside 1 us..1 s')
        if self.read(0x418) or self.read(0x404)&3!=3:raise RuntimeError('CAN disabled/faulted; reconfigure explicitly')
        self.check_motion_guard(identifier,payload)
        if self.read(0x41c)&0xffff>=16:raise BufferError('TX queue full')
        now=self.read(0x408)
        if due_us is None:due_us=(now+1000)&MASK
        # Keep timestamps inside a short unambiguous half-range window.
        if ((due_us-now)&MASK)>1000000:raise ValueError('due time must be within the next second')
        hi,lo=struct.unpack('>II',payload.ljust(8,b'\0'))
        for off,val in zip(range(0x440,0x458,4),(due_us,(due_us+ttl_us)&MASK,identifier,len(payload),hi,lo)):
            self.write(off,val)
        self.write(0x458,1)
    def receive(self):
        if (self.read(0x41c)>>16)==0:return None
        identifier,flags,hi,lo,stamp=[self.read(x) for x in (0x480,0x484,0x488,0x48c,0x490)]
        frame={'id':identifier,'extended':bool(flags&0x100),'rtr':bool(flags&0x200),
               'data':struct.pack('>II',hi,lo)[:min(flags&15,8)].hex(),'timestamp_us':stamp}
        self.write(0x498,1)
        return frame
    def selftest(self):
        self.configure('internal')
        expected=[(0x1abcde0,bytes(8)),(0x1abcde1,bytes([255])*8),
                  (0x028001fd,bytes.fromhex('123456789abcdef0'))]
        received=[];before=self.read(0x428)
        try:
            for ident,payload in expected:
                self.heartbeat();self.enqueue(ident,payload)
                deadline=time.monotonic()+.2
                while time.monotonic()<deadline:
                    self.heartbeat()
                    frame=self.receive()
                    if frame:
                        assert frame['id']==ident and frame['data']==payload.hex() and frame['extended'] and not frame['rtr'],frame
                        received.append(frame);break
                    time.sleep(.001)
                else:raise TimeoutError('internal CAN self-reception timeout')
            assert self.read(0x428)-before==3
            return {'status':'PASS_INTERNAL_LOOPBACK_ONLY','frames':received,'physical_bus_tested':False,
                    'external_tx_inhibited':True,'registers':self.status()}
        finally:self.stop()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--uio',help='explicit UIO device; signature is still checked')
    sub=ap.add_subparsers(dest='command')
    sub.add_parser('status');sub.add_parser('selftest');sub.add_parser('stop')
    p=sub.add_parser('listen');p.add_argument('--seconds',type=float,default=10)
    p=sub.add_parser('read-position');p.add_argument('--motor',type=int,required=True)
    args=ap.parse_args();dev=CanDevice(args.uio)
    try:
        if args.command in (None,'status'):print(json.dumps(dev.status(),indent=2))
        elif args.command=='selftest':print(json.dumps(dev.selftest(),indent=2))
        elif args.command=='stop':dev.stop();print(json.dumps(dev.status(),indent=2))
        elif args.command=='listen':
            if not 0<args.seconds<=3600:raise ValueError('seconds must be in (0,3600]')
            dev.configure('listen');deadline=time.monotonic()+args.seconds;count=0
            try:
                while time.monotonic()<deadline:
                    frame=dev.receive()
                    if frame:print(json.dumps(frame),flush=True);count+=1
                    else:time.sleep(.001)
            finally:dev.stop()
            print(json.dumps({'received':count,'motion_commands_sent':0,'mode':'listen-only'}))
        elif args.command=='read-position':
            if not 1<=args.motor<=7:raise ValueError('motor ID 1..7 required')
            dev.configure('normal')
            try:
                # Type 17, host FD, index 0x7019 MECH_POS. No enable/write/zero.
                dev.enqueue((17<<24)|(0xfd<<8)|args.motor,bytes.fromhex('1970000000000000'))
                deadline=time.monotonic()+.5;frames=[]
                while time.monotonic()<deadline:
                    dev.heartbeat();frame=dev.receive()
                    if frame:frames.append(frame)
                    else:time.sleep(.001)
                print(json.dumps({'status':'FRAMES_CAPTURED' if frames else 'NO_RESPONSE','frames':frames,
                                  'motion_commands_sent':0,'parameter_requests':1},indent=2))
            finally:dev.stop()
    finally:dev.close()
if __name__=='__main__':main()



_CORRECTION=[]


def _correction_bundle():
    """Whether this bundle's firmware has the correction channel; decided once, from files only (no register access)."""
    if not _CORRECTION:_CORRECTION.append(correction_channel_present())
    return _CORRECTION[0]


def correction_channel_present(directory=None):
    """True only when the deployed bundle's bitstream has the online correction channel (no register is touched)."""
    import hashlib,json
    from pathlib import Path
    # A workcell release ships a copy of this module without the firmware; the loaded firmware is the deployed bundle's.
    candidates=[Path(directory)] if directory else [Path(__file__).resolve().parent,Path.home()/'kv260_can_deploy']
    d=next((c for c in candidates if (c/'correction_channel.json').exists()),None)
    if d is None:return False
    marker=d/'correction_channel.json'
    record=json.loads(marker.read_text());bit=d/record['bitstream']
    return bit.exists() and hashlib.sha256(bit.read_bytes()).hexdigest()==record['sha256']

