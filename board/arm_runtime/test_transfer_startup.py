import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from visual_pick_trial import GraspExecutor


class StartupTests(unittest.TestCase):
    def setup_executor(self,unexpected_run=False):
        e=object.__new__(GraspExecutor);e.report={};e.event=Mock()
        e.p=SimpleNamespace(csp_velocity=[.5]*7,acceleration=[.6]*7,current=[.5]*7,to_motor=lambda q:q[:])
        regs={0x470:0x60002};enabled=set();packets=[];feedback={}
        def read(a):
            if a==0x5a8:return sum(1<<(i-1) for i in enabled)
            if a>=0x600 and (a-0x600)%32==24:return 63
            return regs.get(a,0)
        def write(a,v):regs[a]=v
        def send(packet):
            ident,data=packet;kind=ident>>24;mid=ident&255;packets.append(packet)
            if kind==4:raise AssertionError('Unapproved startup reset')
            if kind==3:enabled.add(mid)
            feedback[mid]={'time':time.monotonic(),'position':0.,'mode':2 if mid in enabled or unexpected_run else 0,'fault':0}
        def require(mid,mode):
            f=feedback[mid]
            if f['mode']!=mode:raise RuntimeError('Unexpected motor mode')
            return f
        e.d=SimpleNamespace(read=read,write=write,configure=Mock(),stop=Mock(),pp_base=lambda mid:0x600+32*(mid-1))
        e.s=SimpleNamespace(send=send,pump=Mock(),require=require,read=lambda mid,index:0.,plans={},targets={},
                            feedback=feedback,write_verified=Mock(),active=enabled)
        job=dict(startup_no_reset=True,start_motor_rad=[0.]*7,start_rad=[0.]*7,
                 motor_lower_rad=[-.1]*7,motor_upper_rad=[.1]*7)
        return e,job,packets

    def test_fresh_stopped_feedback_without_any_type4(self):
        e,job,packets=self.setup_executor();e.prepare(job)
        self.assertEqual(sum(p[0]>>24==3 for p in packets),7)
        self.assertFalse(any(p[0]>>24==4 for p in packets))
        self.assertTrue(e.report['startup_reset_skipped']);self.assertEqual(e.state,'HOLDING')

    def test_unexpected_running_mode_rejected_before_enable_or_reconfigure(self):
        e,job,packets=self.setup_executor(True)
        with self.assertRaisesRegex(RuntimeError,'Unexpected motor mode'):e.prepare(job)
        self.assertFalse(any(p[0]>>24 in (3,4) for p in packets))
        e.s.write_verified.assert_not_called()


if __name__=='__main__':unittest.main()
