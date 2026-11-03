import copy, unittest
from pathlib import Path
import numpy as np
from joint_trajectory import JointPlanner
from motion_runtime import MotionRuntime
from gravity_trajectory import GravityModel,fixed_sample,Q24
from frequency_probe import probe_value,build_bank


class ProbeTests(unittest.TestCase):
    def test_envelope_bounds_zero_ends_and_finite(self):
        for f in [1,2,4,8]:
            t=np.linspace(-.1,6/f+.9,10000)
            values=np.array([probe_value(x,f) for x in t])
            self.assertLessEqual(max(abs(values)),.12000000001)
            self.assertEqual(probe_value(0,f),0)
            self.assertEqual(probe_value(6/f+.8,f),0)

    def test_full_bank_samples_obey_torque_caps_and_return_to_current_start(self):
        root=Path(__file__).parent;p=JointPlanner(root/'profile_autotune.json')
        r=MotionRuntime();m=GravityModel(r,p,root/'tracking_grasp_diagnostic_baseline.json')
        m.config['joint3_frequency_probe']=dict(frequencies_hz=[2,4,6],amplitude_nm=.12)
        q=np.deg2rad([-84,1,-3,32,2,6]).tolist()+[.35]
        capture=dict(status='PASS_READ_ONLY_ALL_7',transactions=[dict(motor=i+1,decoded=dict(valid=True,position_motor_rad=a)) for i,a in enumerate(p.to_motor(q))])
        records,info=build_bank(m,capture,p,r)
        self.assertFalse(info['full_grasp']);np.testing.assert_allclose(records[-1]['original_plan']['goal_rad'],q,atol=1e-9)
        g=m.gravity(info['probe_pose_rad'])[2];offsets={};tested=0
        for rec in records:
            if not rec['task'].startswith('TorqueProbe_'):continue
            f=float(rec['task'][len('TorqueProbe_'):-2]);w=rec['words'];n=w[217]
            offset=offsets.get(f,0)+rec['original_plan']['duration_s']-rec['plan']['duration_s']
            for tick in range(n+1):
                u=tick/n;actual=(fixed_sample(w[128:136],int(u*Q24))/Q24-.5)*28
                self.assertAlmostEqual(actual,g+probe_value(offset+u*rec['plan']['duration_s'],f),delta=9e-6)
                self.assertLess(abs(actual),m.caps[2]);tested+=1
            offsets[f]=offsets.get(f,0)+rec['original_plan']['duration_s']
        self.assertGreater(tested,1500)

if __name__=='__main__':unittest.main()
