import json
import tempfile
import time
import unittest
from pathlib import Path
import numpy as np
from workcell.tasks import result,validate,bind,Ledger,guard_instruction,control_intent
from workcell.dataset import make
from workcell.geometry import candidate,rigid_fit,plane_point
from screwcell.archives import DeferredArchives


class TaskContractTests(unittest.TestCase):
    def step(self,**extra):return dict(part_class='screw',quantity=2,region='all',destination='B',**extra)
    def test_slots_are_not_motor_commands(self):
        value=result(steps=[self.step()]);bad=dict(value,motor_torque=[1]*7)
        with self.assertRaises(ValueError):validate(bad)
        bad=dict(value,steps=[dict(self.step(),quantity=True)])
        with self.assertRaises(ValueError):validate(bad)
        bad=dict(value,steps=[dict(self.step(),destination='C')])
        with self.assertRaises(ValueError):validate(bad)
    def test_duplicate_keys_are_rejected(self):
        with self.assertRaises(ValueError):validate('{"version":1,"version":2}')
    def test_operator_control_does_not_need_model_or_a_motion_plan(self):
        self.assertEqual(control_intent('接着执行未完成任务')['action'],'resume')
        self.assertEqual(control_intent('  暂停！ ')['action'],'pause')
    def test_negation_or_compound_instruction_is_not_a_control_shortcut(self):
        for text in ['不要暂停','先暂停，再抓两颗螺丝','继续把所有螺丝放B盒。']:
            self.assertIsNone(control_intent(text))
    def test_valid_json_cannot_invent_a_missing_user_quantity(self):
        guessed=result(steps=[self.step()])
        with self.assertRaisesRegex(ValueError,'EXPLICIT_QUANTITY'):guard_instruction(guessed,'麻烦转移螺丝到B盒。')
        self.assertEqual(guard_instruction(guessed,'右侧两枚螺钉送到B盒。'),guessed)
    def test_exclusions_cannot_conflict_with_plan(self):
        with self.assertRaises(ValueError):result(steps=[self.step()],exclude=['screw'])
    def test_sequence_uses_distinct_fresh_ids(self):
        scene=dict(scene_id='fresh_1',image_width_px=800,objects=[dict(object_id=str(i),part_class='screw',center_uv_px=[100+i*180,100]) for i in range(4)])
        plan=result(steps=[self.step(),dict(self.step(),quantity='all')])
        actions=bind(plan,scene,verified=['0'])
        self.assertEqual([x['object_id'] for x in actions],['1','2','3'])
        self.assertTrue(all(x['scene_id']=='fresh_1' and not x['physical_authorized'] for x in actions))
    def test_shortage_is_not_silently_reduced(self):
        scene=dict(scene_id='s',image_width_px=800,objects=[dict(object_id='a',part_class='screw',center_uv_px=[200,100])])
        with self.assertRaisesRegex(ValueError,'INSUFFICIENT'):bind(result(steps=[self.step()]),scene)
    def test_view_regions_are_explicit(self):
        scene=dict(scene_id='s',image_width_px=800,objects=[dict(object_id=str(i),part_class='screw',center_uv_px=[i*400+50,100]) for i in range(2)])
        actions=bind(result(steps=[dict(self.step(),quantity=1,region='right')]),scene)
        self.assertEqual(actions[0]['object_id'],'1')
    def test_motion_done_cannot_count_success(self):
        ledger=Ledger();ledger.begin('x');self.assertFalse(ledger.verified)
        ledger.verify('x','UNKNOWN','occluded_frame');self.assertFalse(ledger.verified)
        ledger.begin('x');ledger.verify('x','SUCCESS','new_frame');self.assertEqual(ledger.verified,{'x'})
        with self.assertRaisesRegex(ValueError,'ALREADY'):ledger.begin('x')
    def test_retry_budget(self):
        ledger=Ledger(1);ledger.begin('x');ledger.begin('x')
        with self.assertRaises(ValueError):ledger.begin('x')
    def test_split_ids_and_wording_are_disjoint(self):
        rows=[make(x) for x in ['train','validation','test']]
        for i in range(3):
            for row in rows[i]:validate(row['expected'])
            for j in range(i):self.assertFalse({x['instruction'] for x in rows[i]}&{x['instruction'] for x in rows[j]})
    def test_expansion_keeps_evaluation_frozen_and_context_ids_unique(self):
        for split in ['validation','test']:self.assertEqual(make(split),make(split,expanded=True))
        rows=make('train',expanded=True)
        self.assertEqual(len(rows),len({x['id'] for x in rows}))
        for row in rows:validate(row['expected'])


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.t=np.diag([1.,-1.,-1.,1.]);self.t[:3,3]=[-.18,0,.78]
        self.k=[[600,0,400],[0,600,270],[0,0,1]]
        self.cal=dict(id='SIM',T_base_camera=self.t.tolist(),K=self.k,plane_normal=[0,0,1],table_height_m=0.,part_axis_height_m=.006,verified=True)
        self.tool=dict(id='SIM_TOOL',T_flange_tcp=np.eye(4).tolist(),verified=True)
    def test_known_plane_and_axis(self):
        c=candidate([400,270],[[400,250],[400,290]],self.cal,self.tool)
        np.testing.assert_allclose(c['position_m'],[-.18,0,.006],atol=1e-10)
        self.assertAlmostEqual(abs(c['shaft_axis_base'][1]),1.)
        self.assertAlmostEqual(abs(c['jaw_closing_axis_base'][0]),1.)
        self.assertAlmostEqual(np.linalg.det(c['rotation']),1.)
        self.assertFalse(c['physical_authorized'])
    def test_tcp_offset_is_composed_not_added_in_world(self):
        self.tool['T_flange_tcp'][2][3]=-.074
        c=candidate([400,270],[[380,270],[420,270]],self.cal,self.tool)
        self.assertAlmostEqual(c['flange_target'][2][3],.080)
    def test_unverified_calibration_or_tool_cannot_be_used(self):
        self.cal['verified']=False
        with self.assertRaisesRegex(ValueError,'CALIBRATION'):candidate([400,270],[[380,270],[420,270]],self.cal,self.tool)
    def test_parallel_and_behind_rays(self):
        with self.assertRaises(ValueError):plane_point([400,270],self.k,self.t,[1,0,0],0.)
        with self.assertRaises(ValueError):plane_point([400,270],self.k,self.t,[0,0,1],1.)
    def test_paired_point_fit_separate_external_validation(self):
        rng=np.random.default_rng(9);a=rng.normal(size=(12,3));r=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
        b=(r@a.T).T+[.1,.2,.3];out=rigid_fit(a,b)
        self.assertLess(out['fit_rmse_mm'],1e-8);self.assertFalse(out['verified']);self.assertIsNone(out['external_validation_error_mm'])
    def test_degenerate_fit_rejected(self):
        a=np.array([[i,0,0] for i in range(4)])
        with self.assertRaises(ValueError):rigid_fit(a,a)


class ArchiveProcessTests(unittest.TestCase):
    def test_post_disable_archive_completes_in_separate_process(self):
        with tempfile.TemporaryDirectory() as d:
            archive=DeferredArchives();archive.submit('task',{'result_summary':{'evidence_pending':True},'data':list(range(10000))},Path(d)/'report.gz')
            completed=[];until=time.monotonic()+5
            while not completed and time.monotonic()<until:completed=archive.poll();time.sleep(.01)
            self.assertEqual(completed[0][0],'task');self.assertTrue(completed[0][1]['evidence_saved'])
    def test_archive_error_is_evidence_error(self):
        with tempfile.TemporaryDirectory() as d:
            archive=DeferredArchives();archive.submit('task',{},Path(d)/'missing/report.gz')
            completed=[];until=time.monotonic()+5
            while not completed and time.monotonic()<until:completed=archive.poll();time.sleep(.01)
            self.assertFalse(completed[0][1]['evidence_saved']);self.assertIn('evidence_error',completed[0][1])

if __name__=='__main__':unittest.main()
