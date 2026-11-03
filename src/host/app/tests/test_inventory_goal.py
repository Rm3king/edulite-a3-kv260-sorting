import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from workcell.inventory_goal import VERSION,validate,inventory,stable_inventory,reconcile,verify,decode_model_output,guard_goal_instruction
from workcell.live_session import Session,write

CONFIG={'trays':{'A':{'roi_uv':[[100,0],[200,0],[200,100],[100,100]]},'B':{'roi_uv':[[0,0],[99,0],[99,100],[0,100]]}}}

def goal(n=3,tray='B'):
    return dict(version=VERSION,action='reconcile',tray=tray,part_class='screw',quantity=n,preserve=['nut'],question='')

def scene(a,b,nuts=True,frame=1):
    objs=[]
    for tray,count,start in [('A',a,110),('B',b,10)]:
        for i in range(count):objs.append(dict(detection_index=len(objs),part_class='screw',confidence=.9,center_uv_px=[start+i*12,25],pixel_geometry={'axis_unit_uv':[1,0]}))
        if nuts:objs.append(dict(detection_index=len(objs),part_class='nut',confidence=.9,center_uv_px=[start,85]))
    return dict(objects=objs,frame_number=frame,synchronized=True)


class GoalTests(unittest.TestCase):
    def test_model_decoder_and_goal_semantics_guard(self):
        wire={'action':'set_count','tray':'B','quantity':3}
        self.assertEqual(decode_model_output(wire),goal())
        for invalid in [dict(wire,quantity=True),dict(wire,source='A'),{'action':'clarify','question':'几颗？','quantity':3}]:
            with self.assertRaises(ValueError):decode_model_output(invalid)
        for text in ['搬三颗螺丝到B盒','让B盒有一些螺丝','让B盒有三颗螺丝而且头朝上','把最左边的三颗螺丝放到B盒']:
            self.assertEqual(guard_goal_instruction(goal(),text)['action'],'clarify')
        self.assertEqual(guard_goal_instruction(goal(),'让B盒恰好有三颗螺丝')['action'],'reconcile')
        self.assertEqual(guard_goal_instruction(goal(2),'控制左盘两个螺丝，多退少补')['action'],'reconcile')
        self.assertEqual(guard_goal_instruction(goal(4,'B'),'请把右边托盘的螺钉数量调整到四颗')['action'],'clarify')
        self.assertEqual(guard_goal_instruction(goal(1,'A'),'结束时，左盘总共要有1个螺丝')['action'],'clarify')

    def test_robot_region_requires_height_not_just_image_mask(self):
        cfg=copy.deepcopy(CONFIG);cfg['excluded_robot_tip_xyxy']=[0,0,99,99]
        cfg['trays']['B']['plane']={'normal_camera':[0,0,-1],'offset_m':1.}
        s=scene(0,1,nuts=False);s['objects'][0]['camera_xyz_m']=[0,0,.99]
        self.assertEqual(inventory(s,cfg)['counts']['B']['screw'],1)
        s['objects'][0]['camera_xyz_m']=[0,0,.8]
        obs=inventory(s,cfg);self.assertEqual(obs['counts']['B']['screw'],0)
        self.assertEqual(len(obs['robot_exclusions']),1)

    def test_short_equal_excess_and_zero(self):
        for a,b,n,status,source,dest,moves in [(4,1,3,'NEEDS_TRANSFER','A','B',2),(4,3,3,'SATISFIED','B','A',0),(1,4,3,'NEEDS_TRANSFER','B','A',1),(0,3,0,'NEEDS_TRANSFER','B','A',3),(1,1,3,'INSUFFICIENT_PARTS','A','B',2)]:
            d=reconcile(goal(n),inventory(scene(a,b),CONFIG))
            self.assertEqual((d['status'],d['source'],d['destination'],d['moves']),(status,source,dest,moves))
        self.assertEqual(reconcile(goal(3,'A'),inventory(scene(1,4),CONFIG))['source'],'B')

    def test_contract_refuses_guess_and_unknown_fields(self):
        for update in [dict(quantity=True),dict(quantity=-1),dict(quantity=6),dict(quantity='3'),dict(tray='C'),dict(preserve=[]),dict(part_class='nut'),dict(extra='code')]:
            with self.assertRaises(ValueError):validate(dict(goal(),**update))
        with self.assertRaises(ValueError):validate('{"version":"x","version":"y"}')
        v=dict(version=VERSION,action='clarify',tray=None,part_class=None,quantity=None,preserve=[],question='最终要几颗？');validate(v)
        with self.assertRaises(ValueError):reconcile(v,inventory(scene(2,1),CONFIG))

    def test_stability_permutations_and_missing_detection(self):
        frames=[scene(4,1,frame=i) for i in range(3)]
        frames[1]['objects'].reverse();stable_inventory(frames,CONFIG)
        with self.assertRaises(ValueError):stable_inventory([frames[0]]*3,CONFIG)
        frames[2]=scene(3,1,frame=2)
        with self.assertRaises(ValueError):stable_inventory(frames,CONFIG)

    def test_inventory_never_silently_discards_unknown_geometry(self):
        s=scene(1,1);del s['objects'][0]['center_uv_px']
        with self.assertRaises(ValueError):inventory(s,CONFIG)
        s=scene(1,1);s['objects'][0]['confidence']=.3
        with self.assertRaises(ValueError):inventory(s,CONFIG)
        s=scene(1,1);s['objects'][0]['center_uv_px'][0]=float('nan')
        with self.assertRaises(ValueError):inventory(s,CONFIG)

    def test_success_requires_conservation_and_preserved_nuts(self):
        before=inventory(scene(4,1),CONFIG);after=inventory(scene(2,3),CONFIG)
        result=verify(goal(),before,after)
        self.assertEqual(result['status'],'VERIFIED_COUNT_GOAL')
        self.assertIs(result['placement_verified'],False)
        self.assertEqual(verify(goal(),before,before)['status'],'GOAL_NOT_REACHED')
        self.assertEqual(verify(goal(),before,inventory(scene(1,3),CONFIG))['status'],'UNKNOWN_COUNT_CHANGED')
        altered=copy.deepcopy(after);next(o for o in altered['trays']['B'] if o['part_class']=='nut')['center_uv_px'][0]+=15
        self.assertEqual(verify(goal(),before,altered)['status'],'PRESERVED_PART_CHANGED')

    def test_satisfied_and_shortage_never_contact_robot(self):
        for a,b,snapshot_frame in [(2,3,10),(1,1,10),(2,3,0),(1,1,0)]:
            with tempfile.TemporaryDirectory() as td:
                app=Path(td);(app/'integration/config').mkdir(parents=True);(app/'reports').mkdir()
                cfg=copy.deepcopy(CONFIG)
                for name in ('workplane','calibration','heights'):
                    p=app/(name+'.json');write(p,{});cfg[name]=str(p)
                cfg.update(profile={},destinations={})
                write(app/'integration/config/live_workflow.json',cfg)
                snap=app/'snapshot.json';write(snap,scene(a,b,frame=snapshot_frame))
                s=Session(app,lambda _:None)
                with patch.object(s,'snapshot',return_value=snap),patch.object(s,'inventory_frames',return_value=[scene(a,b,frame=i) for i in range(3)]),patch.object(s,'capture_joints',side_effect=AssertionError('No robot')),patch.object(s,'remote',side_effect=AssertionError('No network')):
                    d=s.prepare(goal=goal());self.assertEqual(d['motor_commands_sent'],0)
                    self.assertIn(d['status'],('SATISFIED','INSUFFICIENT_PARTS'))

    def test_unvalidated_goal_execution_blocked_before_network(self):
        with tempfile.TemporaryDirectory() as td:
            app=Path(td);(app/'integration/config').mkdir(parents=True)
            write(app/'integration/config/live_workflow.json',CONFIG)
            out=app/'reports/live_task_goal';out.mkdir(parents=True)
            write(out/'prepared.json',dict(replay_only=False,goal=goal()))
            s=Session(app)
            with patch.object(s,'remote',side_effect=AssertionError('No network')):
                with self.assertRaisesRegex(ValueError,'尚待现场验证'):s.execute(out,True)

    def test_reverse_commissioning_remains_scoped_and_fresh(self):
        with tempfile.TemporaryDirectory() as td:
            app=Path(td);(app/'integration/config').mkdir(parents=True)
            write(app/'integration/config/live_workflow.json',CONFIG)
            out=app/'reports/live_task_reverse';out.mkdir(parents=True)
            s=Session(app)
            base=dict(replay_only=False,goal=goal(0),inventory_before=inventory(scene(0,1),CONFIG),
                      goal_direction='B->A',screws=1,nuts=0,created_monotonic_s=-1000)
            with patch.object(s,'remote',side_effect=AssertionError('No network')):
                write(out/'prepared.json',base)
                with self.assertRaisesRegex(ValueError,'尚待现场验证'):s.execute(out,True)
                with self.assertRaisesRegex(ValueError,'expired'):s.execute(out,True,single_reverse_trial=True)
                with self.assertRaisesRegex(ValueError,'attended'):s.execute(out,single_reverse_trial=True)
                for change in [dict(goal=goal(1),inventory_before=inventory(scene(1,0),CONFIG),goal_direction='A->B'),
                               dict(inventory_before=inventory(scene(0,2),CONFIG),screws=2),
                               dict(nuts=1),dict(goal_direction='A->B')]:
                    write(out/'prepared.json',dict(base,**change))
                    with self.assertRaisesRegex(ValueError,'one B->A'):s.execute(out,True,single_reverse_trial=True)
                write(out/'prepared.json',dict(base,replay_only=True))
                with self.assertRaisesRegex(ValueError,'Replay'):s.execute(out,True,single_reverse_trial=True)
            self.assertFalse(s.config.get('inventory_goal_execution_enabled',False))

    def test_goal_commissioning_binds_goal_direction_count_and_lifecycle(self):
        import time
        with tempfile.TemporaryDirectory() as td:
            app=Path(td);(app/'integration/config').mkdir(parents=True)
            write(app/'integration/config/live_workflow.json',CONFIG)
            out=app/'reports/live_task_goal_trial';out.mkdir(parents=True)
            s=Session(app,lambda _:None)
            base=dict(replay_only=False,goal=goal(3),inventory_before=inventory(scene(3,1),CONFIG),
                      goal_direction='A->B',screws=2,nuts=0,created_monotonic_s=time.monotonic(),input_bindings={})
            with patch.object(s,'remote',side_effect=AssertionError('No hardware')):
                write(out/'prepared.json',base)
                with self.assertRaisesRegex(ValueError,'尚待现场验证'):s.execute(out,True)
                with self.assertRaisesRegex(ValueError,'attended'):s.execute(out,commissioning_goal=goal())
                with self.assertRaisesRegex(ValueError,'differs'):s.execute(out,True,commissioning_goal=goal(2))
                for change in [dict(goal_direction='B->A'),dict(screws=3),dict(nuts=1),
                               dict(inventory_before=inventory(scene(1,1),CONFIG))]:
                    write(out/'prepared.json',dict(base,**change))
                    with self.assertRaisesRegex(ValueError,'plan differs'):s.execute(out,True,commissioning_goal=goal())
                for change,message in [(dict(replay_only=True),'Replay'),(dict(created_monotonic_s=-1000),'expired')]:
                    write(out/'prepared.json',dict(base,**change))
                    with self.assertRaisesRegex(ValueError,message):s.execute(out,True,commissioning_goal=goal())
                write(out/'prepared.json',base)
                with patch.object(s,'install_release',side_effect=RuntimeError('VALIDATED_BEFORE_NETWORK')):
                    with self.assertRaisesRegex(RuntimeError,'VALIDATED_BEFORE_NETWORK'):s.execute(out,True,commissioning_goal=goal())
                self.assertEqual(json.loads((out/'commissioning.json').read_text())['goal'],goal())
                write(out/'owner.json',dict(pid=1))
                with self.assertRaisesRegex(ValueError,'already launched'):s.execute(out,True,commissioning_goal=goal())
            self.assertFalse(s.config.get('inventory_goal_execution_enabled',False))


class SlotTests(unittest.TestCase):
    def test_occupied_center_prefers_nearby_free_slot_over_edge(self):
        import numpy as np
        from workcell.tray_geometry import destination_slots
        c={'trays':{'A':{'plane':{'normal_camera':[0,0,-1.],'offset_m':1.},
                        'roi_uv':[[-100,-100],[100,-100],[100,100],[-100,100]],
                        'rotation':np.eye(3).tolist(),
                        'slots_uv':[[-20,0],[20,0],[0,0],[-5,0],[5,0]]}}}
        rows=[dict(tray='A',part_class='nut',axis_base=None,support_normal_base=[0,0,1.],base_surface_xyz_m=[0,0,1.01],surface_height_m=.01)]
        result=destination_slots(rows,c,'A',1,dict(fx=100,fy=100,cx=0,cy=0),np.eye(4))
        self.assertEqual(result['pixels'],[[-5,0]])
        self.assertEqual(result['selection_policy'],'CENTRAL_AVAILABLE_SLOT')

    def test_single_empty_destination_prefers_central_existing_slot(self):
        import numpy as np
        from workcell.tray_geometry import destination_slots
        c={'trays':{'A':{'plane':{'normal_camera':[0,0,-1.],'offset_m':1.},
                        'roi_uv':[[-100,-100],[100,-100],[100,100],[-100,100]],
                        'rotation':np.eye(3).tolist(),
                        'slots_uv':[[-10,-10],[10,-10],[-10,10],[10,10],[0,0]]}}}
        result=destination_slots([],c,'A',1,dict(fx=100,fy=100,cx=0,cy=0),np.eye(4))
        self.assertEqual(result['pixels'],[[0,0]])
        self.assertEqual(c['trays']['A']['slots_uv'][0],[-10,-10])

    def test_each_tray_uses_its_own_surface_plane(self):
        import numpy as np
        from workcell.tray_geometry import surface_objects
        c=copy.deepcopy(CONFIG)
        for name,height in [('A',1.),('B',2.)]:
            c['trays'][name]['plane']={'normal_camera':[0,0,-1.],'offset_m':height}
        rows=surface_objects(scene(1,1,nuts=False),c,dict(fx=100,fy=100,cx=0,cy=0),np.eye(4),{'screw':.01})
        by_tray={r['tray']:r for r in rows}
        self.assertAlmostEqual(by_tray['A']['base_surface_xyz_m'][2],.99)
        self.assertAlmostEqual(by_tray['B']['base_surface_xyz_m'][2],1.99)


    def test_occupied_slots_and_nonparallel_overlap(self):
        import numpy as np
        from workcell.tray_geometry import rectangles_overlap,destination_slots
        R=np.eye(3);self.assertTrue(rectangles_overlap(np.zeros(3),R,[.012,.038],np.zeros(3),R,[.014,.014]))
        self.assertFalse(rectangles_overlap(np.zeros(3),R,[.012,.038],np.array([.10,0,0]),R,[.014,.014]))
        c={'trays':{'B':{'plane':{'normal_camera':[0,0,-1.],'offset_m':1.},'roi_uv':[[-100,-100],[100,-100],[100,100],[-100,100]],'rotation':R.tolist(),'slots_uv':[[0,0],[10,0]]}}}
        rows=[dict(tray='B',part_class='nut',axis_base=None,support_normal_base=[0,0,1.],base_surface_xyz_m=[0,0,1.01],surface_height_m=.01)]
        result=destination_slots(rows,c,'B',1,dict(fx=100,fy=100,cx=0,cy=0),np.eye(4))
        self.assertEqual(result['pixels'],[[10,0]])
        expanded=destination_slots(rows,c,'B',2,dict(fx=100,fy=100,cx=0,cy=0),np.eye(4))
        self.assertEqual(len(expanded['pixels']),2)
        positions=[np.array([uv[0]/100,uv[1]/100,1.]) for uv in expanded['pixels']]
        self.assertFalse(rectangles_overlap(positions[0],R,[.012,.038],positions[1],R,[.012,.038]))
        for p in positions:
            self.assertFalse(rectangles_overlap(p,R,[.012,.038],np.array([0,0,1.]),R,[.014,.014]))
            self.assertLessEqual(np.linalg.norm(p-np.array([.05,0,1.])),.050001)
        c['trays']['B']['roi_uv']=[[-1,-1],[1,-1],[1,1],[-1,1]]
        with self.assertRaisesRegex(ValueError,'未找到'):destination_slots(rows,c,'B',2,dict(fx=100,fy=100,cx=0,cy=0),np.eye(4))

    def test_batch_backtracks_instead_of_blocking_second_placement(self):
        import numpy as np
        from workcell.tray_geometry import destination_slots
        spec={'plane':{'normal_camera':[0,0,-1.],'offset_m':1.},
              'roi_uv':[[-10,-10],[10,-10],[10,10],[-10,10]],
              'rotation':np.eye(3).tolist(),'slots_uv':[[-2.5,0],[2.5,0],[0,0]]}
        r=destination_slots([],{'trays':{'A':spec}},'A',2,dict(fx=100,fy=100,cx=0,cy=0),np.eye(4))
        self.assertEqual({tuple(p) for p in r['pixels']},{(-2.5,0),(2.5,0)})
        self.assertNotEqual(r['selection_policy'],'INTERIOR_BATCH_SEARCH')

    def test_full_part_envelope_stays_inside_even_if_center_is_inside(self):
        import numpy as np
        from workcell.tray_geometry import destination_slots
        spec={'plane':{'normal_camera':[0,0,-1.],'offset_m':1.},
              'roi_uv':[[-10,-5],[10,-5],[10,5],[-10,5]],
              'rotation':np.eye(3).tolist(),'slots_uv':[[0,4],[0,0]]}
        r=destination_slots([],{'trays':{'A':spec}},'A',1,dict(fx=100,fy=100,cx=0,cy=0),np.eye(4))
        self.assertEqual(r['pixels'],[[0,0]])

if __name__=='__main__':unittest.main()
