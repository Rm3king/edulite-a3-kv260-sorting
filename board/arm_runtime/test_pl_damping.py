import copy,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from gravity_trajectory import GravityModel,prepare_stream,hold_segment,validate_records
from motion_runtime import MotionRuntime
from joint_trajectory import JointPlanner
from gravity_transport import GravityTransport
from tuning_worker import guard_job,patch_gains
from autotune import execute

ROOT=Path(__file__).parent
class DampingTests(unittest.TestCase):
 def setUp(self):
  self.p=JointPlanner(ROOT/'profile_autotune.json');self.runtime=MotionRuntime()
  self.q=[-.8,.1,-.2,.3,0.,0.,.35]
 def model(self,value):
  cfg=json.loads((ROOT/'tracking_grasp_diagnostic_baseline.json').read_text());cfg['pl_joint3_damping_nm_s_rad']=value
  with tempfile.NamedTemporaryFile(mode='w',suffix='.json') as f:
   json.dump(cfg,f);f.flush();return GravityModel(self.runtime,self.p,f.name)
 def test_damping_changes_only_reserved_metadata_and_survives_gain_patch(self):
  tasks=[dict(name='hold',segments=[hold_segment(self.p,self.q,.2)],goal_rad=self.q,gravity_alpha=[1.,1.])]
  off=prepare_stream(self.model(0),tasks);m=self.model(2);on=prepare_stream(m,tasks)
  self.assertEqual(off[0]['words'][:216],on[0]['words'][:216]);self.assertEqual(on[0]['words'][216],512)
  self.assertEqual(off[0]['words'][217:],on[0]['words'][217:])
  self.assertEqual(patch_gains(on,[60]*6,[5]*6,m)[0]['words'][216],512)
  validate_records(on,guard_job(self.p,tasks),self.p,m)
  bad=copy.deepcopy(on);w=list(bad[0]['words']);w[216]=513;bad[0]['words']=tuple(w)
  with self.assertRaisesRegex(ValueError,'damping metadata'):validate_records(bad,guard_job(self.p,tasks),self.p,m)
 def test_unknown_firmware_rejects_nonzero_feature_before_mmio_writes(self):
  tasks=[dict(name='hold',segments=[hold_segment(self.p,self.q,.2)],goal_rad=self.q)]
  rec=prepare_stream(self.model(2),tasks);writes=[]
  d=SimpleNamespace(read=lambda a:{0x470:0x60001,0xb68:0x70003}[a],write=lambda *x:writes.append(x))
  with self.assertRaisesRegex(RuntimeError,'v6.2'):GravityTransport(d,self.p,rec)
  self.assertEqual(writes,[])
  t=GravityTransport(d,self.p,[]);t.prepared.extend(rec)
  with self.assertRaisesRegex(RuntimeError,'v6.2'):t.submit(rec[0]['original_plan'])
  self.assertEqual(writes,[])
  d.read=lambda a:{0x470:0x60002,0xb68:0x70003}[a];GravityTransport(d,self.p,rec)
 def test_invalid_gain_rejected(self):
  for x in [-1,2.01,float('nan'),True]:
   with self.assertRaises(ValueError):self.model(x)
 def test_mismatched_experiment_rejected_before_device_open(self):
  with tempfile.TemporaryDirectory() as d,patch('autotune.CanDevice',side_effect=AssertionError('device opened')):
   with self.assertRaisesRegex(ValueError,'identical gains'):
    execute(ROOT/'profile_autotune.json',ROOT/'tracking_grasp_diagnostic_baseline.json',d,full_pick_only=True,compare_pl_damping=True)

if __name__=='__main__':unittest.main()
