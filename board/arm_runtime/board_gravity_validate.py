#!/usr/bin/python3
"""Validate v6 MMIO only; external TX is inhibited and no motor is enabled."""
import argparse,json
from pathlib import Path
from motion_executor import CanDevice


def run(words):
    if len(words)!=220:raise ValueError('220 precomputed staging words required')
    d=CanDevice();report=dict(scope='INTERNAL_ONLY_EXTERNAL_TX_INHIBITED',physical_gravity_tested=False)
    try:
        if d.read(0x470) not in (0x60000,0x60001,0x60002,0x60003):raise RuntimeError('v6 required before new register access')
        if d.read(0x404) or d.read(0x5a8):raise RuntimeError('stopped/disarmed hardware required')
        assert d.read(0xb68)==0x70003
        for mid in range(1,8):
            base=d.pp_base(mid)
            for mode in (0,1,2):
                d.write(base+28,mode);assert d.read(base+28)==mode
            d.write(base+28,1);d.write(base,0)
        report['independent_guard_modes']='PASS'
        d.configure('internal')
        assert d.read(0x404)==15
        d.write(0xb60,1);assert d.read(0xb60)==1 and d.read(0xb40)==0
        for i,w in enumerate(words):
            d.heartbeat();d.write(0xc00+4*i,w);assert d.read(0xc00+4*i)==w&0xffffffff
        d.write(0xb60,1);assert d.read(0xb60)==1 and d.read(0xb40)==2
        d.write(0xb60,1);assert d.read(0xb60)==2
        d.write(0xb60,2);assert d.read(0xb60)==3 and d.read(0xb40)==2 and d.read(0xb48)==0
        report['partial_commit_rejected']='PASS';report['full_commit_and_readback']='PASS'
        report['second_commit_rejected']='PASS';report['start_without_enabled_axes_rejected']='PASS'
        assert d.read(0x5a8)==0 and d.read(0x41c)==0 and d.read(0x418)==0
        report['status']='PASS_V6_INTERNAL_STAGING';report['registers_before_stop']=d.status()
    finally:
        d.stop();d.write(0x5a0,0)
        for mid in range(1,8):d.write(d.pp_base(mid),0)
        report['final']=d.status();d.close()
    return report


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--vectors',required=True);a=ap.parse_args()
    print(json.dumps(run([int(x,16) for x in Path(a.vectors).read_text().split()]),indent=2))
