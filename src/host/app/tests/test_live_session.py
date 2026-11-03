import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from workcell.live_session import Session, select, select_reviewed_nut, unchanged, write


CONFIG = dict(source_roi_uv=[[0,0],[100,0],[100,100],[0,100]], excluded_robot_tip_xyxy=[0,0,10,10])


def obj(index, cls='screw', uv=(50,50)):
    return dict(detection_index=index, part_class=cls, center_uv_px=list(uv))


class SelectionTests(unittest.TestCase):
    def test_reviewed_nut_does_not_follow_reordered_detection_id(self):
        scene=dict(objects=[obj(0,'nut',(20,20)),obj(8,'nut',(52,50)),obj(9,'screw',(50,50))])
        self.assertEqual(select_reviewed_nut(scene,CONFIG,[50,50])[0]['detection_index'],8)

    def test_reviewed_nut_rejects_missing_or_ambiguous_target(self):
        for values in [[obj(1,'nut',(57,50))],[obj(1,'nut',(49,50)),obj(2,'nut',(51,50))]]:
            with self.assertRaises(ValueError):select_reviewed_nut(dict(objects=values),CONFIG,[50,50])

    def test_tip_exclusion_and_no_wall_margin(self):
        scene=dict(objects=[obj(0,uv=(5,5)), obj(1,uv=(99,50)),obj(2,'nut')])
        self.assertEqual([o['detection_index'] for o in select(scene, CONFIG,1,0)],[1])

    def test_no_invented_count(self):
        with self.assertRaises(ValueError):select(dict(objects=[obj(1)]),CONFIG,2,0)
        for values in [(True,0),(0,0),(6,0),(-1,2)]:
            with self.assertRaises(ValueError):select(dict(objects=[]),CONFIG,*values)

    def test_permutation_and_empty_class(self):
        self.assertTrue(unchanged([obj(1,uv=(20,20)),obj(2,uv=(50,50))],
                                  [obj(9,uv=(51,50)),obj(0,uv=(20,21))]))

    def test_motion_and_nonselected_change(self):
        a=[obj(1),obj(2,'nut',uv=(20,20))]
        self.assertFalse(unchanged(a,[obj(1),obj(2,'nut',uv=(40,20))]))   # 7 px is detection noise since max_pixels became 10
        self.assertFalse(unchanged(a,[obj(1)]))

    def test_same_center_rotated_screw_is_not_fresh(self):
        a=obj(1);b=obj(1)
        a['pixel_geometry']=dict(axis_unit_uv=[1.,0.])
        b['pixel_geometry']=dict(axis_unit_uv=[0.,1.])
        self.assertFalse(unchanged([a],[b]))
        b['pixel_geometry']['axis_unit_uv']=[-1.,0.]
        self.assertTrue(unchanged([a],[b]))


class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.app=Path(self.temp.name);(self.app/'integration/config').mkdir(parents=True)
        write(self.app/'integration/config/live_workflow.json',CONFIG)
        self.session=Session(self.app,lambda _:None)
        self.out=self.app/'reports/live_task_test';self.out.mkdir(parents=True)

    def test_constructor_does_not_connect(self):
        with patch('subprocess.run',side_effect=AssertionError('No process allowed')):
            Session(self.app)

    def test_declared_override_is_bound_but_silent_edits_are_rejected(self):
        session=Session(self.app,lambda _:None,overrides=dict(joint_time_law='cruise_c3'))
        self.assertEqual(session.config['joint_time_law'],'cruise_c3')
        self.assertEqual(session.file_config(),session.config)
        session.config['joint_time_law']='septic'
        self.assertNotEqual(session.file_config(),session.config)
        session.config['joint_time_law']='cruise_c3'
        write(self.app/'integration/config/live_workflow.json',dict(CONFIG,extra=1))
        self.assertNotEqual(session.file_config(),session.config)

    def test_replay_never_executes(self):
        write(self.out/'prepared.json',dict(replay_only=True))
        with patch.object(self.session,'remote',side_effect=AssertionError('No remote call allowed')):
            with self.assertRaisesRegex(ValueError,'Replay'):self.session.execute(self.out,True)

    def test_execute_requires_attendance(self):
        with self.assertRaises(ValueError):self.session.execute(self.out)

    def test_duplicate_and_expired_before_any_connection(self):
        write(self.out/'prepared.json',dict(replay_only=False,created_monotonic_s=-1000))
        with patch.object(self.session,'remote',side_effect=AssertionError('No remote call allowed')):
            with self.assertRaisesRegex(ValueError,'expired'):self.session.execute(self.out,True)
            write(self.out/'owner.json',dict(pid=123))
            with self.assertRaisesRegex(ValueError,'already launched'):self.session.execute(self.out,True)

    def test_disable_requires_fresh_explicit_text(self):
        for value in [None,'可以','昨天已经同意了']:
            with self.assertRaises(ValueError):self.session.disable_supported(self.out,value)

    def test_camera_stall_does_not_spawn_second_pipeline(self):
        from workcell.live_startup import ensure_camera
        write(self.app/'reports/rgbd_surface_preview_active.json',dict(pid=7,report_dir=str(self.out)))
        write(self.out/'latest.json',dict(host_acquired_monotonic_s=-10000))
        with patch.object(Path,'read_bytes',return_value=b'detect_parts_rgbd_live.py '+str(self.out).encode()), patch('subprocess.Popen',side_effect=AssertionError('No competing camera')):
            with self.assertRaisesRegex(RuntimeError,'stale'):ensure_camera(self.app)

    def test_camera_starting_does_not_spawn_second_pipeline(self):
        from workcell.live_startup import ensure_camera
        write(self.app/'reports/rgbd_surface_preview_active.json',dict(pid=7,report_dir=str(self.out)))
        with patch.object(Path,'read_bytes',return_value=b'detect_parts_rgbd_live.py '+str(self.out).encode()), patch('subprocess.Popen',side_effect=AssertionError('No competing camera')):
            with self.assertRaisesRegex(RuntimeError,'no readable frame'):ensure_camera(self.app)


if __name__=='__main__':unittest.main()
