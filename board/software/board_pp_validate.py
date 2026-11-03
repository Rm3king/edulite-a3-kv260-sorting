#!/usr/bin/python3
"""Stopped-controller v4 PP bank independence checks. Sends no CAN frames."""
import json
import struct
from canctl import CanDevice


def main():
    dev=CanDevice();saved={};report={'scope':'REGISTER_ONLY_NO_CAN_FRAMES','checks':0}
    try:
        assert dev.read(0x470)==0x40000
        assert dev.read(0x404)==0 and dev.read(0x5a0)==0 and dev.read(0x5a8)==0
        before=dev.read(0x420)
        for mid in range(1,8):
            b=dev.pp_base(mid)
            assert dev.read(b)==0 and dev.read(b+24)==0
            for offset in (4,8,12,16,20):saved[b+offset]=dev.read(b+offset)
        wanted={}
        for mid in range(1,8):
            b=dev.pp_base(mid)
            for offset,value in ((4,-mid*.01),(8,mid*.01),(12,mid*.02),(16,mid*.03),(20,mid*.04)):
                bits=struct.unpack('<I',struct.pack('<f',value))[0]
                dev.write(b+offset,bits);wanted[b+offset]=bits
            # Check all banks already written, detecting address/axis aliasing.
            for address,bits in wanted.items():
                assert dev.read(address)==bits,(address,bits,dev.read(address));report['checks']+=1
        assert dev.read(0x420)==before
        report['submitted_can_frames']=0;report['status']='PASS'
    finally:
        for address,bits in saved.items():dev.write(address,bits)
        report['restored']=all(dev.read(a)==b for a,b in saved.items())
        report['final']=dev.status();dev.close()
    assert report['restored']
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
