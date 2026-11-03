import unittest,struct
from types import SimpleNamespace
from unittest.mock import Mock
from continuous_transfer import recover_task_outcome,TransferDevice

class RecoveryTests(unittest.TestCase):
 def fake(self):
  goal=[0]*6+[1.6];branch=dict(segments=[{'plan':'reviewed'}],start_rad=goal,goal_rad=goal);records=SimpleNamespace(recoveries={'screw_0_close':['encoded_review']})
  e=SimpleNamespace(state='HOLDING',d=Mock(),s=Mock(),transport=SimpleNamespace(status=lambda:dict(fault=0),prepared=None,prepared_index=7),event=Mock(),execute_segments=Mock(),settle_arm=Mock(return_value={'ok':True}))
  e.d.read.side_effect=lambda a:{0x418:0,0x5a8:127}[a]
  e.p=SimpleNamespace(to_motor=lambda q:list(q));e.s.require.side_effect=lambda mid,mode:{'position':0. if mid<7 else 1.5}
  return e,dict(recoveries={'screw_0_close':branch}),records
 def test_empty_grasp_uses_reviewed_return_and_marks_supported(self):
  e,j,r=self.fake();report={};recover_task_outcome(e,j,r,'screw_0_close',report)
  self.assertTrue(report['automatic_supported_return_completed']);e.execute_segments.assert_called_once_with(j['recoveries']['screw_0_close']['segments']);self.assertEqual(e.transport.prepared_index,0)
 def test_active_motion_or_fault_never_launches_return(self):
  for kind in ['moving','fault','missing']:
   e,j,r=self.fake()
   if kind=='moving':e.state='MOVING'
   elif kind=='fault':e.d.read.side_effect=lambda a:1
   else:r.recoveries={}
   with self.assertRaises(RuntimeError):recover_task_outcome(e,j,r,'screw_0_close',{})
   e.execute_segments.assert_not_called()
 def test_transfer_cap_and_no_live_parameter_writes(self):
  import ast,inspect,continuous_transfer
  d=TransferDevice(Mock());packet=lambda a:struct.pack('<H',0x7018)+bytes(2)+struct.pack('<f',a)
  d.enqueue(0x1200fd07,packet(.5))
  with self.assertRaises(ValueError):d.enqueue(0x1200fd07,packet(.7))
  tree=ast.parse(inspect.getsource(continuous_transfer.execute))
  for node in ast.walk(tree):
   if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute):
    self.assertNotIn(node.func.attr,('write_verified','send','enqueue'))

 def test_return_rejects_displaced_arm_but_accepts_jaw_contact(self):
  e,j,r=self.fake();e.s.require.side_effect=lambda mid,mode:{'position':.02 if mid==2 else 0.}
  report={}
  with self.assertRaisesRegex(RuntimeError,'Actual arm pose'):recover_task_outcome(e,j,r,'screw_0_close',report)
  e.execute_segments.assert_not_called();self.assertEqual(report['recovery_start_check']['arm_errors_rad'][1],.02)

if __name__=='__main__':unittest.main()
