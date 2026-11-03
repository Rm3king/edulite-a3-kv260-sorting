import unittest
from stopped_connection import setup_stopped_feedback


class Device:
    def __init__(self,active=False):self.control=7 if active else 0;self.writes=[];self.permit=0
    def read(self,address):return self.control if address==0x404 else 0
    def pp_base(self,mid):return 0x700+(mid-1)*32
    def write(self,address,value):
        if address in (0x800,0xb40):raise AssertionError('Read-only hardware register')
        if address==0x5a0 or 0x700<=address<0x800:
            if self.control:raise AssertionError('Live guard configuration')
        self.writes.append((address,value))
        if address==0x5a0:self.permit=value
    def configure(self,mode):self.control=7
    def stop(self):self.control=0


class Session:
    def __init__(self,fail=False):self.frames=[];self.fail=fail;self.required=[]
    def send(self,frame):self.frames.append(frame)
    def pump(self,seconds):
        if self.fail:raise TimeoutError('No motor response')
    def require(self,mid,mode):self.required.append((mid,mode))


class ConnectionTests(unittest.TestCase):
    def test_setup_only_writes_timeout_and_stops_cleanly(self):
        d=Device();s=Session();setup_stopped_feedback(d,s)
        self.assertEqual(s.required,[(i,0) for i in range(1,8)])
        self.assertEqual(len(s.frames),7)
        for ident,data in s.frames:
            self.assertEqual(ident>>24,18)
            self.assertEqual(int.from_bytes(data[:2],'little'),0x7028)
        self.assertEqual((d.control,d.permit),(0,0))
    def test_active_owner_never_touched(self):
        d=Device(True);s=Session()
        with self.assertRaises(RuntimeError):setup_stopped_feedback(d,s)
        self.assertEqual(d.writes,[]);self.assertEqual(s.frames,[])
    def test_failed_feedback_clears_only_controller_permits(self):
        d=Device();s=Session(True)
        with self.assertRaises(TimeoutError):setup_stopped_feedback(d,s)
        self.assertEqual((d.control,d.permit),(0,0))
        self.assertTrue(all(ident>>24==18 for ident,data in s.frames))


if __name__=='__main__':unittest.main()
