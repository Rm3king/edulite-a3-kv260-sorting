import copy
import io
import json
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from workcell import reverse_transfer as r
from workcell.live_session import scene_objects
from workcell.order_agent import OrderAgent
from workcell.order_language import fast_parse
from workcell.order_planner import unsupported_reason


class ReverseTransfer(unittest.TestCase):
    def setUp(self):
        folder = Path(__file__).parent/'fixtures/reverse_small_nut'
        self.base = json.loads((folder/'config.json').read_text())
        self.scene = json.loads((folder/'scene.json').read_text())
        self.config = dict(self.base, **r.overrides(self.base))

    def test_filming_sentence_is_an_explicit_reverse_small_nut_task(self):
        task = fast_parse('把左边盘子的小螺母放回右边')
        self.assertTrue(r.requested(task))
        self.assertIsNone(unsupported_reason(task))
        for change in [dict(quantity=2), dict(part_class='nut'), dict(region='all')]:
            bad = copy.deepcopy(task);bad['steps'][0].update(change)
            self.assertFalse(r.requested(bad))
            self.assertIsNotNone(unsupported_reason(bad))

    def test_task_overrides_preserve_default_configuration(self):
        original = copy.deepcopy(self.base)
        config = r.overrides(self.base)
        self.assertEqual(self.base, original)
        self.assertEqual(config['source_roi_uv'], self.base['trays']['B']['roi_uv'])
        self.assertEqual(config['destination_floor'], self.base['trays']['A']['plane'])
        self.assertEqual(config['destinations']['pixels'][r.SLOT], r.INTERIOR_UV)
        self.assertEqual(config['destinations']['pixels'][:4], self.base['trays']['A']['slots_uv'][:4])
        n = self.base['trays']['B']['plane']['normal_base']
        for obj in config['reference_objects']:
            if obj.get('axis_base'):
                self.assertAlmostEqual(sum(a*b for a,b in zip(n,obj['axis_base'])), 0.)

    def test_both_trays_keep_parts_inside_old_robot_image_rectangle(self):
        self.assertEqual(len(r.tray_objects(self.scene,self.config,'B')),4)
        self.assertEqual(len(r.tray_objects(self.scene,self.config,'A')),5)
        self.assertEqual(len(scene_objects(self.scene,self.config,('screw','nut','screw_m6','nut_m6'))),4)

    def test_new_point_is_farther_inside_and_separated_from_existing_parts(self):
        from workcell.live_session import inside
        polygon = self.base['trays']['A']['roi_uv']
        def edge_distance(point):
            import math
            distances = []
            for a, b in zip(polygon, polygon[1:]+polygon[:1]):
                v = [b[i]-a[i] for i in (0, 1)]
                t = max(0., min(1., sum((point[i]-a[i])*v[i] for i in (0, 1))/sum(x*x for x in v)))
                distances.append(math.dist(point, [a[i]+t*v[i] for i in (0, 1)]))
            return min(distances)
        self.assertTrue(inside(r.INTERIOR_UV, polygon))
        self.assertGreater(edge_distance(r.INTERIOR_UV), 3*edge_distance(self.base['trays']['A']['slots_uv'][2]))
        import math
        self.assertGreater(min(math.dist(r.INTERIOR_UV, o['center_uv_px']) for o in r.tray_objects(self.scene,self.config,'A')),160.)

    def marker_scene(self):
        return json.loads((Path(__file__).parent/'fixtures/reverse_small_nut/flat_marker_scene.json').read_text())

    def test_saved_low_confidence_frame_excludes_only_the_known_flat_sticker(self):
        scene = self.marker_scene()
        self.assertEqual(len(r.tray_objects(scene,self.config,'B')),4)
        self.assertEqual(len(r.tray_objects(scene,self.config,'A')),5)
        self.assertEqual(len(scene['fixture_marker_exclusions']),1)
        self.assertEqual(len(scene['objects']),10)  # retained for geometry and evidence

    def test_real_nut_or_changed_calibration_at_marker_is_not_ignored(self):
        for change in ('height','footprint','calibration'):
            with self.subTest(change=change):
                scene = self.marker_scene()
                marker = next(o for o in scene['objects'] if r.fixed_marker(o,scene,self.config))
                if change == 'height':
                    n = self.config['trays']['B']['plane']['normal_camera']
                    marker['camera_xyz_m'] = [x+.006*y for x,y in zip(marker['camera_xyz_m'],n)]
                elif change == 'footprint':marker['box_xyxy_px'] = [324.,576.,361.,613.]
                else:scene['calibration_id'] = 'different_scene'
                with self.assertRaisesRegex(ValueError,'低置信度'):
                    r.tray_objects(scene,self.config)

    def test_other_low_confidence_parts_still_block(self):
        scene = copy.deepcopy(self.scene);scene['objects'][0]['confidence'] = .3
        with self.assertRaisesRegex(ValueError,'低置信度'):
            r.tray_objects(scene,self.config)

    def test_known_robot_region_still_excludes_high_object(self):
        scene=copy.deepcopy(self.scene)
        obj=next(o for o in scene['objects'] if o['part_class']=='screw')
        obj['camera_xyz_m']=[0.,0.,0.2]
        self.assertEqual(len(r.tray_objects(scene,self.config,'B')),3)

    def test_destination_must_be_empty(self):
        r.check_destination(self.scene,self.config)
        scene=copy.deepcopy(self.scene)
        scene['objects'][2]['center_uv_px']=self.config['destinations']['pixels'][r.SLOT]
        with self.assertRaisesRegex(ValueError,'落点已有零件'):
            r.check_destination(scene,self.config)

    def test_preexisting_nut_does_not_prove_a_transfer(self):
        report=dict(parts=[dict(status='PLACED')],placed=1)
        r.verify_counts(report,self.scene,self.scene,self.config)
        self.assertEqual(report['placed'],0)
        self.assertEqual(report['parts'][0]['status'],'UNCONFIRMED_TRANSFER')

    def test_transfer_requires_source_decrease_and_destination_increase(self):
        after=copy.deepcopy(self.scene)
        nut=next(o for o in r.tray_objects(after,self.config,'B') if o['part_class']=='nut_m6')
        nut['center_uv_px']=self.config['destinations']['pixels'][r.SLOT]
        report=r.verify_counts(dict(parts=[dict(status='PLACED')],placed=1),self.scene,after,self.config)
        self.assertEqual(report['placed'],1)
        self.assertEqual(report['transfer_counts'],dict(before=dict(A=2,B=1),after=dict(A=3,B=0)))
        self.assertEqual(report['tray_total_counts'],dict(before=dict(A=5,B=4),after=dict(A=6,B=3)))

    def test_console_plan_selects_reverse_slot_and_next_normal_order_resets_direction(self):
        agent=OrderAgent('/nonexistent',observe=lambda:dict(objects=r.tray_objects(self.scene,self.config,'B'),frame=self.scene),
                         temperatures=lambda:{m:30. for m in range(1,8)})
        agent._session=lambda:SimpleNamespace(config=self.config)
        plan=agent.plan('把左边盘子的小螺母放回右边')
        self.assertTrue(agent.reverse_small_nut)
        self.assertEqual(plan['parts'],['nut_m6'])
        self.assertEqual(plan['batches'][0]['slots'],[r.SLOT])
        agent.plan('把一个大螺母放到左盘')
        self.assertFalse(agent.reverse_small_nut)

    def test_only_reverse_order_adds_the_cli_switch(self):
        for reverse in [False,True]:
            agent=OrderAgent('/nonexistent');agent.reverse_small_nut=reverse
            process=Mock(stdout=io.StringIO(''));process.wait.return_value=0
            with patch('workcell.order_agent.subprocess.Popen',return_value=process) as popen:
                agent.run_batch(['nut_m6'],[r.SLOT])
            self.assertEqual('--reverse-small-nut' in popen.call_args.args[0],reverse)

    def test_cli_refuses_other_reverse_parts_before_any_io(self):
        from workcell.wrist_servo_trial import run
        with self.assertRaisesRegex(ValueError,'one M6 nut'):
            run('/nonexistent','/nonexistent',1,parts=['nut'],reverse_small_nut=True)

    def test_review_uses_left_before_and_right_after(self):
        from workcell.vlm_review import review
        with patch('workcell.vlm_review.color_image',return_value=Path('image.png')), \
             patch('workcell.vlm_review.tray_crop',side_effect=[Path('left.jpg'),Path('right.jpg')]) as crop, \
             patch('workcell.vlm_review.tray_counts',return_value={'nut_m6':3}):
            output=review(None,'reverse',dict(text='完成 1/1 件',done=[('nut_m6',2)],failed=[],pending=[]),[],
                          before='before.json',after='after.json',trays=self.base['trays'],direction='B->A')
        self.assertEqual(crop.call_args_list[0].args[1],self.base['trays']['B']['roi_uv'])
        self.assertEqual(crop.call_args_list[1].args[1],self.base['trays']['A']['roi_uv'])
        self.assertIn('源盘（左盘 B）',output['facts'])
        self.assertIn('目标盘（右盘 A）',output['facts'])


if __name__=='__main__':unittest.main()
