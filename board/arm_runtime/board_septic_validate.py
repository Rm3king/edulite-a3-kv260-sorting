#!/usr/bin/python3
"""New firmware staging/commit tests in internal mode. No external CAN output.

Physical seven-axis enable is deliberately unavailable in internal mode. This
test validates the real MMIO bank and atomic acceptance without starting HLS;
full evaluation/CAN integration is verified by XSIM and attended motion.
"""
import json,sys
from pathlib import Path
from septic_trajectory import septic_words,movej_c3
from joint_trajectory import JointPlanner
from septic_transport import SepticTransport
from motion_executor import CanDevice

def validate(p):
    # All numerical planning precedes the short hardware lease. On ARM the
    # polynomial validation can exceed 100 ms; it must not run after configure.
    q=[0.,.2,-.2,0.,0.,0.,.3];goal=[.1,.3,-.3,.1,.1,.1,.4]
    segment=movej_c3(p,q,goal)[0];words,_,_,_=septic_words(segment,p)
    d=CanDevice();report=dict(scope='INTERNAL_ONLY_NO_PHYSICAL_CAN_OR_ENABLE',checks=0)
    try:
        if d.read(0x470) not in (0x50001,0x60000,0x60001,0x60002,0x60003):raise RuntimeError('degree-seven firmware required')
        if d.read(0x404) or d.read(0x5a8) or d.read(0x5a0):raise RuntimeError('stopped disarmed board required')
        d.configure('internal');initial=d.status();t=SepticTransport(d,p)
        before=d.read(0x820);d.write(0x820,8)
        assert d.read(0x820)==before+1 and d.read(0x800)==0;report['checks']+=1
        for i,w in enumerate(words):
            d.heartbeat();d.write(0xa00+4*i,w);actual=d.read(0xa00+4*i)
            assert actual==w&0xffffffff,(i,hex(actual),hex(w&0xffffffff),d.status())
            report['checks']+=1
        before=d.read(0x820);d.write(0x820,8)
        assert d.read(0x820)==before and d.read(0x800)==2;report['checks']+=1
        # No enabled motor mask in internal mode: empty-mask start rejected.
        d.write(0x820,2);assert d.read(0x820)==before+1 and d.read(0x800)==2;report['checks']+=1
        assert d.read(0x420)==initial['submitted'] and d.read(0x5a8)==0
        report['status']='PASS';report['submitted_can_frames']=0
    finally:
        d.stop();report['final']=d.status();report['final_trajectory_running']=bool(d.read(0x800)&1)
        d.close()
    return report

if __name__=='__main__':print(json.dumps(validate(JointPlanner(sys.argv[1])),indent=2))
