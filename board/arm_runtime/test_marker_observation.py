import copy,struct,unittest
from types import SimpleNamespace
from unittest.mock import Mock
from continuous_transfer import MARKER_POLICY,MARKER_LIFT_POLICY,contract,path_digest,TransferDevice

class MarkerTests(unittest.TestCase):
    def fixture(self):
        q=[0.]*6+[.91];tilt=q.copy();tilt[4]=-.3;tasks=[]
        previous=q
        for name,goal in [('marker_start_approach',q),('marker_tilt_approach',tilt),('return_supported',q)]:
            rows=[[a,b-a]+[0.]*6 for a,b in zip(previous,goal)]
            tasks.append(dict(name=name,goal_rad=goal.copy(),segments=[dict(coefficients_rad=rows)],return_to_live_start=name=='return_supported'));previous=goal
        j=dict(policy=MARKER_POLICY,startup_no_reset=True,motion_mask=127,profile_id='test',start_rad=q,start_motor_rad=q.copy(),open_rad=.91,marker_capture_seconds=3.,gripper_open_current_a=.5,gripper_close_current_a=.5,tasks=tasks)
        j['geometry_audit']=dict(status='PASS_SAMPLED_WITH_MONOTONIC_EXISTING_START_CONTACT_DEPARTURE',unexpected=[],trajectory_sha256=path_digest(j),finger_extension_m=.035,extended_finger_min_floor_clearance_m=.06)
        return j,SimpleNamespace(profile_id='test',current=[1.,5.5,4.5,2.5,1.,1.,.5],velocity=[.25]*7)
    def test_fixed_observation_allowed(self):
        j,p=self.fixture();contract(j,p)
    def test_rejects_motion_aperture_wait_and_missing_geometry(self):
        base,p=self.fixture()
        for change in ['jaw','arm','overshoot','unbounded','reset','length','clearance','nan_clearance','digest','grasp']:
            j=copy.deepcopy(base)
            if change=='jaw':j['tasks'][1]['goal_rad'][6]+=.01
            if change=='arm':j['tasks'][1]['segments'][0]['coefficients_rad'][1][1]=.01
            if change=='overshoot':j['tasks'][1]['segments'][0]['coefficients_rad'][4][1]=-1.
            if change=='unbounded':j['marker_capture_seconds']=60.
            if change=='reset':j['startup_no_reset']=False
            if change=='length':j['geometry_audit']['finger_extension_m']=0
            if change=='clearance':j['geometry_audit']['extended_finger_min_floor_clearance_m']=.019
            if change=='nan_clearance':j['geometry_audit']['extended_finger_min_floor_clearance_m']=float('nan')
            if change=='grasp':j['targets']=[{}]
            j['geometry_audit']['trajectory_sha256']=path_digest(j) if change!='digest' else 'wrong'
            with self.subTest(change=change),self.assertRaises(ValueError):contract(j,p)
    def lift_fixture(self):
        j,p=self.fixture();j['policy']=MARKER_LIFT_POLICY;j['tasks'][1]['name']='marker_lift_approach';q=j['start_rad'];goal=q.copy();goal[1]=.15;goal[2]=-.21
        previous=q
        for task,g in zip(j['tasks'],[q,goal,q]):
            task['goal_rad']=g.copy();task['segments']=[dict(coefficients_rad=[[a,b-a]+[0.]*6 for a,b in zip(previous,g)])];previous=g
        j['geometry_audit'].update(trajectory_sha256=path_digest(j),maximum_end_displacement_m=.04,end_goal_lift_m=.04,end_goal_rotation_error_rad=0.,minimum_end_height_change_m=0.)
        return j,p
    def test_short_lift_requires_reviewed_upward_sweep(self):
        j,p=self.lift_fixture();contract(j,p)
        for key in ['maximum_end_displacement_m','end_goal_lift_m','end_goal_rotation_error_rad','minimum_end_height_change_m']:
            bad=copy.deepcopy(j);del bad['geometry_audit'][key]
            with self.subTest(key=key),self.assertRaises(ValueError):contract(bad,p)
        for key,value in [('maximum_end_displacement_m',.1),('end_goal_lift_m',-.04),('end_goal_rotation_error_rad',.1),('minimum_end_height_change_m',-.01)]:
            bad=copy.deepcopy(j);bad['geometry_audit'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):contract(bad,p)
    def test_short_lift_cannot_move_jaw(self):
        j,p=self.lift_fixture();j['tasks'][1]['goal_rad'][6]+=.01
        with self.assertRaises(ValueError):contract(j,p)

    def test_driver_exact_hold_only(self):
        underlying=Mock();d=TransferDevice(underlying);d.marker_jaw_hold=.91
        def send(v):d.enqueue(0x1200fd07,struct.pack('<HHf',0x7016,0,v))
        send(.91);self.assertEqual(underlying.enqueue.call_count,1)
        for v in [.90,1.12,float('nan')]:
            with self.assertRaises(ValueError):send(v)
        self.assertEqual(underlying.enqueue.call_count,1)

if __name__=='__main__':unittest.main()
