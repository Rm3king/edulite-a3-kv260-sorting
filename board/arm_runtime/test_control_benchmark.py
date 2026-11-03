"""Cold-boot initialization cannot send enable or stop another controller."""
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import control_benchmark as b

class Dev:
 def __init__(self,active=False):self.active=active;self.writes=[];self.stops=0;self.closed=False
 def read(self,a):return 7 if a==0x404 and self.active else 0
 def write(self,a,v):self.writes.append((a,v))
 def pp_base(self,m):return 0x680 if m==7 else 0x700+(m-1)*32
 def configure(self,mode):self.active=True
 def stop(self):self.stops+=1;self.active=False
 def close(self):self.closed=True
class Session:
 def __init__(self,d,r):self.d=d;self.r=r;self.modes={i:0 for i in range(1,7)}
 def send(self,packet):self.r['commands'].append(packet)
 def pump(self,t):pass
 def require(self,mid,mode):assert mode==0
 def read(self,mid,index,integer):return self.modes[mid]
 def write_verified(self,mid,index,value,integer):
  assert index==0x7005 and value==5 and integer;self.modes[mid]=value
  self.r['commands'].append((0x1200fd00|mid,b'\x05\x70\0\0\x05\0\0\0'))
class BootTests(unittest.TestCase):
 def test_cold_mode_verified_without_enable(self):
  d=Dev();p=SimpleNamespace(csp_velocity=[.7]*7,acceleration=[.9]*7,current=[1.]*7)
  job={'motor_lower_rad':[-1.]*7,'motor_upper_rad':[1.]*7}
  with patch.object(b,'CanDevice',return_value=d),patch.object(b,'Session',Session):r=b.initialize_disabled_csp(p,job)
  self.assertEqual(set(r['mode_before'].values()),{0});self.assertEqual(set(r['mode_after'].values()),{5})
  self.assertEqual({ident>>24 for ident,data in r['commands']},{4,18});self.assertFalse(r['flash_writes']);self.assertEqual(d.stops,1)
 def test_active_controller_is_never_stopped(self):
  d=Dev(True)
  with patch.object(b,'CanDevice',return_value=d),patch.object(b,'Session',Session):
   with self.assertRaises(RuntimeError):b.initialize_disabled_csp(None,None)
  self.assertEqual(d.writes,[]);self.assertEqual(d.stops,0);self.assertTrue(d.closed)

if __name__=='__main__':unittest.main()
