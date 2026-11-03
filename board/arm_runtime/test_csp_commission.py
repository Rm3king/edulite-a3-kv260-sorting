import unittest
import json
import tempfile
from pathlib import Path
from csp_commission import plans_for,DELTAS,bind_review,SettlingWindow
from joint_trajectory import JointPlanner

class CommissionPlanTests(unittest.TestCase):
    def test_zero_is_explicit_and_does_not_return_to_start(self):
        p=JointPlanner();start=[1.47,.03,.06,.52,.03,.13,.1]
        review=plans_for(start,p,'visible','zero')
        fresh=[x+.0002 for x in start]
        bound=bind_review(fresh,review,p,'visible','zero')
        self.assertEqual(bound['return_plan']['goal_rad'],[0.]*7)
        self.assertEqual(bound['final_robot_rad'],[0.]*7)
        self.assertNotEqual(bound['start_robot_rad'],bound['final_robot_rad'])
        self.assertEqual(bound['forward_plan']['goal_rad'],bound['return_plan']['start_rad'])
        review['goal_robot_rad']=[0.]*7
        review['forward_plan']=p.movej(review['start_robot_rad'],[0.]*7)
        review['return_plan']=p.movej([0.]*7,[0.]*7,minimum_duration_s=.1)
        direct=bind_review(fresh,review,p,'visible','zero')
        self.assertEqual(direct['return_plan']['duration_s'],.1)
        self.assertEqual(direct['return_plan']['goal_rad'],[0.]*7)
        with self.assertRaises(ValueError):bind_review(fresh,review,p,'visible','start')
        review['final_robot_rad'][6]=.1
        with self.assertRaises(ValueError):bind_review(fresh,review,p,'visible','zero')

    def test_zero_settling_restarts_on_error_or_gap(self):
        w=SettlingWindow()
        for i in range(100):self.assertFalse(w.observe(i*.02,[.001]*7))
        self.assertFalse(w.observe(2.,[.003]+[.001]*6))
        for i in range(100):self.assertFalse(w.observe(2.02+i*.02,[.001]*7))
        self.assertTrue(w.observe(4.04,[.001]*7))
        self.assertFalse(w.observe(4.2,[.001]*7))

    def test_visible_and_small_pin_endpoints_and_limits(self):
        p=JointPlanner();start=[1.47,.0014,.0294,.5223,.0349,.1352,.1055]
        for amplitude in DELTAS:
            r=plans_for(start,p,amplitude)
            self.assertEqual(r['forward_plan']['goal_rad'],r['return_plan']['start_rad'])
            self.assertEqual(r['start_robot_rad'],r['return_plan']['goal_rad'])
            self.assertAlmostEqual(r['start_robot_rad'][0],-start[0])
            if amplitude!='axis3':self.assertGreater(r['goal_robot_rad'][1],r['start_robot_rad'][1])
            else:
                for i in (0,1,3,4,5,6):self.assertEqual(r['goal_robot_rad'][i],r['start_robot_rad'][i])
            self.assertLess(r['goal_robot_rad'][2],r['start_robot_rad'][2])
            self.assertEqual(r['profile_id'],p.profile_id)
    def test_fresh_hold_start_keeps_reviewed_absolute_goal(self):
        p=JointPlanner();start=[1.47,.0014,.0294,.5223,.0349,.1352,.1055]
        original=plans_for(start,p,'visible');fresh=[x+.0005 for x in start]
        bound=bind_review(fresh,original,p,'visible')
        self.assertEqual(bound['goal_robot_rad'],original['goal_robot_rad'])
        self.assertEqual(bound['start_motor_rad'],fresh)
        self.assertEqual(bound['return_plan']['goal_rad'],bound['start_robot_rad'])
        original['forward_plan']=p.movej(original['start_robot_rad'],original['goal_robot_rad'],minimum_duration_s=12.)
        slow=bind_review(fresh,original,p,'visible')
        self.assertGreaterEqual(slow['forward_plan']['duration_s'],12.)
        self.assertEqual(slow['return_plan']['duration_s'],slow['forward_plan']['duration_s'])
        fresh[0]+=.002
        with self.assertRaises(ValueError):bind_review(fresh,original,p,'visible')

    def test_no_clipping_or_automatic_direction_reversal(self):
        p=JointPlanner();start=[1.47,.0014,.0294,.5223,.0349,.1352,1.67]
        for amplitude in ('small','visible'):
            with self.assertRaises(ValueError):plans_for(start,p,amplitude)

    def test_servo_ceiling_does_not_accelerate_pl_trajectory(self):
        p=JointPlanner();start=[1.47,.03,.04,.52,.03,.13,.1]
        old=plans_for(start,p,'axis3');profile=dict(p.profile)
        profile['csp_velocity_limit_rad_s']=list(p.velocity)
        profile['csp_velocity_limit_rad_s'][2]=.06
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'profile.json';path.write_text(json.dumps(profile));new=JointPlanner(path)
            bound=plans_for(start,new,'axis3')
            for k in ('duration_s','coefficients_rad','hls_coefficients_q8_24','peak_velocity_rad_s'):
                self.assertEqual(old['forward_plan'][k],bound['forward_plan'][k])
            self.assertEqual(new.csp_velocity[2],.06)
            self.assertEqual(new.velocity[2],.01)
            profile['csp_velocity_limit_rad_s'][2]=.001
            path.write_text(json.dumps(profile))
            with self.assertRaises(ValueError):JointPlanner(path)

if __name__=='__main__':unittest.main()
