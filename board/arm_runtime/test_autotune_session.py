"""Exercise the complete session orchestration without hardware or wall delays."""
import json,pickle,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from tuning_search import suggest
from autotune import execute
ROOT=Path(__file__).parent
class SessionTests(unittest.TestCase):
 def test_full_search_repeats_gravity_and_pick_selection_without_hardware(self):
  self.run_session()
 def test_software_recovery_failure_cannot_fall_through_to_reset(self):
  self.run_session(inject_failure=True)
 def test_full_pick_only_skips_axis_search_and_retains_until_supported_return(self):
  self.run_session(full_pick_only=True)
 def test_full_pick_damping_comparison_preserves_position_gains(self):
  self.run_session(full_pick_only=True,candidate_kp=80.,candidate_kd=5.)
 def test_candidate_replay_has_one_complete_path_and_analysis_after_disable(self):
  self.run_session(full_pick_only=True,candidate_kp=80.,candidate_kd=5.,replay_candidate=True)
 def test_refinement_compares_against_previous_damped_candidate(self):
  self.run_session(full_pick_only=True,candidate_kp=60.,candidate_kd=5.,baseline_kp=80.,baseline_kd=5.)
 def test_invalid_baseline_rejected_before_opening_hardware(self):
  with patch('autotune.CanDevice') as device,tempfile.TemporaryDirectory() as out:
   with self.assertRaisesRegex(ValueError,'baseline'):
    execute(ROOT/'profile_autotune.json',ROOT/'tracking_autotune.json',out,full_pick_only=True,baseline_kd=5.1)
   device.assert_not_called()
 def test_friction_abba_uses_same_gains_and_finishes_uncompensated_path_at_support(self):
  self.run_session(full_pick_only=True,candidate_kp=80.,candidate_kd=5.,baseline_kp=80.,baseline_kd=5.,compare_friction=True)
 def test_pl_damping_abba_keeps_all_gains_and_returns_before_reset(self):
  self.run_session(full_pick_only=True,candidate_kp=60.,candidate_kd=5.,baseline_kp=60.,baseline_kd=5.,candidate_axis4_kd=5.,compare_pl_damping=True)
 def test_method_abba_keeps_single_factor_and_builds_before_enable(self):
  for method in ('reference_smoothing','split_damping','inverse_dynamics'):
   self.run_session(full_pick_only=True,candidate_kp=60.,candidate_kd=5.,baseline_kp=60.,baseline_kd=5.,candidate_axis4_kd=5.,method_name=method)
 def test_wrist_damping_comparison_preserves_current_shoulder_candidate(self):
  self.run_session(full_pick_only=True,candidate_kp=60.,candidate_kd=5.,baseline_kp=60.,baseline_kd=5.,candidate_axis4_kd=5.)
 def test_camera_diagnostic_changes_only_joint3_proportional_gain(self):
  self.run_session(full_pick_only=True,candidate_kp=60.,candidate_kd=5.,candidate_axis4_kd=5.,candidate_axis3_kp=40.,replay_candidate=True)
 def test_invalid_joint3_override_rejected_before_opening_hardware(self):
  with patch('autotune.CanDevice') as device,tempfile.TemporaryDirectory() as out:
   with self.assertRaisesRegex(ValueError,'axis3'):
    execute(ROOT/'profile_autotune.json',ROOT/'tracking_autotune.json',out,full_pick_only=True,candidate_axis3_kp=241)
   device.assert_not_called()
 def test_friction_comparison_keeps_preferred_wrist_damping_in_both_groups(self):
  self.run_session(full_pick_only=True,candidate_kp=60.,candidate_kd=5.,baseline_kp=60.,baseline_kd=5.,candidate_axis4_kd=5.,compare_friction=True)
 def run_session(self,inject_failure=False,full_pick_only=False,candidate_kp=50.,candidate_kd=4.,replay_candidate=False,baseline_kp=80.,baseline_kd=4.,compare_friction=False,candidate_axis4_kp=80.,candidate_axis4_kd=4.,candidate_axis3_kp=None,compare_pl_damping=False,method_name=None):
  lifecycle=[];ownership={'released':False,'pl_started':False}
  class Pipe:
   def __init__(s):s.queue=[];s.header={};s.bytes_pending=0
   def poll(s):return bool(s.queue) or bool(s.bytes_pending)
   def send(s,message):
    command,p=message
    if command=='entry':s.queue.append(('ok',{}))
    elif command=='suggest':s.queue.append(('ok',suggest(p['history'],p['axis'])))
    elif command=='records':
     self.assertFalse(ownership['released'] and not ownership['pl_started'],'no numerical wait between host release and first PL start')
     s.queue.append(('records',1));s.bytes_pending=1
    elif command=='calibrate':s.queue.append(('ok',{'scale':[1.]*6}))
    elif command=='analyze':
     if full_pick_only:self.assertEqual(lifecycle,['reset'],'analysis must wait until supported reset')
     s.header=p;s.queue.append(('analysis_ready',None))
    elif command=='end':
     axis=s.header['axis'];score=1+abs(s.header['kp'][axis]-120)/120
     s.queue.append(('ok',dict(score=score,global_score=score,feasible=True,axis=axis+1,joints=[dict(position_rmse_rad=.001)]*6)))
    elif command in ('commands','frames','feedback','pl_samples','segment_submissions','events','quality'):s.queue.append(('chunk_ok',None))
   def recv(s):return s.queue.pop(0)
   def recv_bytes(s,n):s.bytes_pending-=1;return pickle.dumps(dict(original_plan={'goal_rad':[0.]*7},plan={'ticks':1}))
   def close(s):pass
  class Device:
   def read(s,a):return 0
   def status(s):return dict(control=0,fault=0,rx_overflow=0,expired=0)
   def close(s):pass
  class Executor:
   def __init__(s,p,d,r,prepared,m):
    s.p=p;s.d=d;s.report=r;s.submitted=0;s.state='HOLDING'
    s.transport=SimpleNamespace(prepared=[],prepared_index=0,status=lambda:dict(fault=0,holding=True))
    s.s=SimpleNamespace(active=set(),read=lambda mid,index:[1.,.1,.2,.3,.1,.1,.35][mid-1],pump=lambda t:None,require=lambda *args:{})
    def stop():lifecycle.append('reset');r['reset_confirmed']=list(range(1,8));s.s.active.clear()
    s.s.stop_all=stop
   def prepare(s,*args,**kwargs):
    if kwargs.get('before_enable'):
     self.assertFalse(s.s.active);kwargs['before_enable']([1.,.1,.2,.3,.1,.1,.35])
    s.report.update(owned=True,enable_attempted=True,volatile_configuration_changed=True);s.s.active=set(range(1,8))
   def release_host_hold(s):ownership['released']=True
   def pump(s,*args):return s.transport.status()
   def event(s,*args,**kwargs):pass
   def execute_segments(s,plans):
    if inject_failure:raise RuntimeError('injected analysis software exception')
    ownership['pl_started']=True
    s.submitted+=len(plans);s.transport.prepared_index+=len(plans);s.expected=[0.]*7
   def restore_csp_configuration(s):s.report['restored_original_csp_and_torque_limits']=True
  proc=SimpleNamespace(is_alive=lambda:False)
  def supported_hold(e,out,summary):
   self.assertEqual(lifecycle,[])
   lifecycle.append('serviced_hold_until_supported')
   summary['supported_disable_requested']=True
  with tempfile.TemporaryDirectory() as out,patch('autotune.open',side_effect=lambda *a,**kw:Path(out,'mock_controller.lock').open('w'),create=True),patch('autotune.launch_worker',return_value=(Pipe(),proc)),patch('autotune.CanDevice',Device),patch('autotune.TuningExecutor',Executor),patch('autotune.retain_until_supported',side_effect=supported_hold):
   tracking=ROOT/'tracking_autotune.json'
   if compare_friction or compare_pl_damping or method_name:
    config=json.loads(tracking.read_text());config['friction_coulomb_nm']=[0,.1,.05,0,0,0]
    if compare_pl_damping:config.update(friction_coulomb_nm=[0]*6,pl_joint3_damping_nm_s_rad=2.)
    if method_name:
     config.update(friction_coulomb_nm=[0]*6,full_pick_experiment=dict(method=method_name))
     if method_name=='split_damping':
      config['kd'][2]=5.;config['pl_joint3_damping_nm_s_rad']=2.;config['full_pick_experiment']['fast_joint3_kd']=3.
     if method_name=='inverse_dynamics':config['inverse_dynamics_scale']=.35
    config['kp'][3]=candidate_axis4_kp;config['kd'][3]=candidate_axis4_kd
    tracking=Path(out)/'friction.json';tracking.write_text(json.dumps(config))
   result=execute(ROOT/'profile_autotune.json',tracking,out,trials=4,full_pick_only=full_pick_only,candidate_kp=candidate_kp,candidate_kd=candidate_kd,replay_candidate=replay_candidate,baseline_kp=baseline_kp,baseline_kd=baseline_kd,compare_friction=compare_friction,candidate_axis4_kp=candidate_axis4_kp,candidate_axis4_kd=candidate_axis4_kd,candidate_axis3_kp=candidate_axis3_kp,compare_pl_damping=compare_pl_damping,compare_method=bool(method_name))
   if inject_failure:
    self.assertEqual(result['status'],'FAIL')
    self.assertEqual(lifecycle,['serviced_hold_until_supported','reset'])
    self.assertTrue(result['supported_disable_requested'])
    return
   if full_pick_only:
    self.assertEqual(result['status'],'PASS_FULL_PICK_REPLAY_DATA_COLLECTION' if replay_candidate else 'PASS_FULL_PICK_ABBA_DATA_COLLECTION')
    self.assertEqual(len(result['winners']),0)
    self.assertEqual(len(result['experiments']),1 if replay_candidate else 4)
    expected_keys=['pick_uncompensated','pick','pick','pick_uncompensated'] if compare_friction else ['pick']*(1 if replay_candidate else 4)
    if compare_pl_damping:expected_keys=['pick_undamped','pick','pick','pick_undamped']
    if method_name:expected_keys=['pick_control','pick','pick','pick_control']
    self.assertEqual([r['key'] for r in result['experiments']],expected_keys)
    ck=[candidate_kp,candidate_kp if candidate_axis3_kp is None else candidate_axis3_kp]
    self.assertEqual([r['kp'][1:3] for r in result['experiments']],[ck] if replay_candidate else [[baseline_kp]*2,ck,ck,[baseline_kp]*2])
    candidate_pair=[candidate_kd,3. if method_name=='split_damping' else candidate_kd]
    self.assertEqual([r['kd'][1:3] for r in result['experiments']],[candidate_pair] if replay_candidate else [[baseline_kd]*2,candidate_pair,candidate_pair,[baseline_kd]*2])
    fixed=compare_friction or compare_pl_damping or method_name
    self.assertEqual([r['kp'][3] for r in result['experiments']],[candidate_axis4_kp] if replay_candidate else [candidate_axis4_kp if fixed else 80.,candidate_axis4_kp,candidate_axis4_kp,candidate_axis4_kp if fixed else 80.])
    self.assertEqual([r['kd'][3] for r in result['experiments']],[candidate_axis4_kd] if replay_candidate else [candidate_axis4_kd if fixed else 4.,candidate_axis4_kd,candidate_axis4_kd,candidate_axis4_kd if fixed else 4.])
    self.assertEqual(lifecycle,['reset'])
    self.assertFalse(result['candidate_adopted'])
    self.assertTrue(result['analysis_after_supported_disable'])
    self.assertTrue(all(not r['measurement_pending'] for r in result['experiments']))
    self.assertTrue(result['csp_restored'])
    self.assertEqual(json.loads((Path(out)/'summary.json').read_text())['reset_confirmed'],list(range(1,8)))
    return
   self.assertEqual(result['status'],'PASS_TUNING_SEARCH_AND_PICK_ABBA')
   self.assertEqual(len(result['winners']),6);self.assertEqual(len(result['final_pick_trials']),6)
   self.assertEqual(result['full_pick_selection']['selected_group'],'B')
   self.assertEqual(result['selected_kp'],[120.]*6);self.assertTrue(result['csp_restored'])
   self.assertEqual(json.loads((Path(out)/'summary.json').read_text())['reset_confirmed'],list(range(1,8)))
if __name__=='__main__':unittest.main()
