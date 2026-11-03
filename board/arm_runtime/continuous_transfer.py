"""Reviewed attended two-part transfer; snapshots never gate motion.

No hardware on import/default CLI. A SHA-bound reviewed bundle avoids repeating
polynomial root searches on the board. Fresh stopped feedback is still required.
"""
import argparse,fcntl,gzip,hashlib,importlib,json,signal,time,struct,re,math
from pathlib import Path
from calibration_motion import motion_gc_scope
from visual_pick_trial import GraspExecutor
from visual_pick_retry import RetryDevice
from canctl import CanDevice
from joint_trajectory import JointPlanner,Q24
from motion_executor import job_digest,job_and_path_digests
from septic_job import verify_septic_job
from septic_trajectory import extrema
from control_continuity import retain_until_supported,await_disable_approval,WaitStateWriter

POLICY='attended_continuous_two_part_transfer_v1'
BASE_GRASP_HEIGHT_OFFSET_M=.017
# Screws are gripped this much higher than nuts (2026-10-26: the fingertips met the tray floor on descent, the jammed
# motors heated). The job's offset is the distance the carried part hangs below the tool point, so it grows by the same amount.
SCREW_GRASP_RAISE_M=.004


def grasp_raise_m(part_classes):
    """Raise applied to the grasp of a job whose parts are all screws; a nut or a mixed batch keeps the reviewed height."""
    classes=list(part_classes)
    return SCREW_GRASP_RAISE_M if classes and all(c=='screw' for c in classes) else 0.


def target_classes(job):
    return [t.get('part_class') or t['target_id'].split('_')[0] for t in job.get('targets',[])]


def expected_grasp_height_offset_m(part_classes):
    return BASE_GRASP_HEIGHT_OFFSET_M+grasp_raise_m(part_classes)
HOVER_POLICY='attended_noncontact_alignment_hover_v1'
MARKER_POLICY='attended_marker_wrist_observation_v1'
MARKER_LIFT_POLICY='attended_marker_short_lift_observation_v1'
MARKER_POLICIES=(MARKER_POLICY,MARKER_LIFT_POLICY)
# Attended slip trial; a current ceiling is not a measured gripping force.
TRANSFER_GRIPPER_CURRENT_MAX_A=.65
DEPENDENCIES=('continuous_transfer','visual_pick_retry','visual_pick_trial','calibration_motion',
              'motion_executor','septic_job','septic_trajectory','septic_transport','pl_transport',
              'joint_trajectory','arm_commission','canctl','a3_protocol','control_continuity','csp_commission','stopped_connection')


def source_hashes():
    return {n:hashlib.sha256(Path(importlib.import_module(n).__file__).read_bytes()).hexdigest() for n in DEPENDENCIES}


def path_digest(job):
    paths=dict(tasks=job['tasks'],recoveries=job['recoveries']) if job.get('recoveries') else job['tasks']
    return hashlib.sha256(json.dumps(paths,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def chain_of(job):
    """(index, count, supported arm pose) of a job inside one continuous multi-nut session.

    A chain job starts at the previous nut's exit pose, not at the supported pose;
    only the last job returns to support. Every recovery still ends at support.
    """
    chain=job.get('chain')
    if chain is None:return 0,1,job['start_rad'][:6]
    index,count,support=chain['index'],chain['count'],chain['support_rad']
    if (type(index) is not int or type(count) is not int or not 0<=index<count<=5 or len(support)!=7
            or any(not math.isfinite(v) for v in support) or abs(support[6]-job['open_rad'])>1e-9):
        raise ValueError('Invalid continuous chain descriptor')
    return index,count,support[:6]


class PreparedRecords(list):
    pass


class TaskOutcomeError(RuntimeError):
    """A rest-point task outcome failure; eligible for a reviewed return."""


# Return line of the wrist session. The motor driver's own over-temperature fault is 145 C of the thermistor (el_a3_sdk/docs/电机通信协议汇总.md),
# 70 C was a conservative first choice; the operator chose 95 C (2026-10-27), 50 C below that fault; the thermistor sits outside the winding, so the winding is hotter than
# the reading by an amount that has not been measured, which is why this is not raised further without a measurement.
WRIST_RETURN_TEMPERATURE_C=95.


def check_batch_release_temperature(e,job,task):
    """Long batches may return at a reviewed empty-jaw endpoint before overheating."""
    limit=job.get('batch_return_temperature_c')
    if limit is None:return
    temperatures=[e.s.require(mid,2)['temperature'] for mid in range(1,8)]
    if not all(math.isfinite(t) for t in temperatures):raise RuntimeError('Invalid batch temperature feedback')
    e.event('BATCH_RELEASE_TEMPERATURE',task=task,temperature_c=temperatures,return_threshold_c=limit)
    if max(temperatures)>=limit:
        raise TaskOutcomeError('Batch temperature threshold: return after release')


JAW_CEILINGS={1.6:1.62,1.66:1.67}   # reviewed close command -> highest jaw value any later sample may reach


def jaw_ceiling(job):
    """1.62 for the 1.60 close of every part; 1.67 for the M6 nut's 1.66 close (about its 10 mm across flats a 1.60 close only
    touched it: reads 1.574-1.586, two of five empty; the commissioned fully closed jaw is 1.678; user approved 2026-10-30)."""
    close=job.get('close_rad',1.6)
    if close not in JAW_CEILINGS:raise ValueError('Observed grasp settings')
    return JAW_CEILINGS[close]


def initial_jaw_upper(job):
    """Admit the measured fully closed start only for monotone initial opening."""
    if not job.get('initial_open_from_observed_closed'):return jaw_ceiling(job)
    q=job['start_rad'][6];actual=job['start_motor_rad'][6]
    if (not job.get('initial_open_during_departure') or not 1.62<actual<=1.70
            or not math.isfinite(q) or abs(q-actual)>.0001):
        raise ValueError('Fresh measured closed jaw start required')
    return max(jaw_ceiling(job),min(1.70,actual+.002))


def initial_jaw_lower(job):
    """Admit a measured wider aperture only until departure reaches 1.12."""
    if not job.get('initial_normalize_from_observed_wide'):return 1.08
    q=job['start_rad'][6];actual=job['start_motor_rad'][6]
    if (not job.get('initial_open_during_departure') or job.get('initial_open_from_observed_closed')
            or not 0.<=actual<1.08 or not math.isfinite(q) or abs(q-actual)>.0001):
        raise ValueError('Fresh measured wide jaw start required')
    return actual-.002


class TransferDevice(RetryDevice):
    initial_jaw_hold=None
    marker_jaw_hold=None
    jaw_ceiling=1.62      # the owner raises it to jaw_ceiling(job) of its reviewed jobs (1.67 only for an M6 nut close)
    def enqueue(self,identifier,payload,*args,**kwargs):
        if identifier&255==7 and identifier>>24==18 and int.from_bytes(bytes(payload)[:2],'little')==0x7016:
            value=struct.unpack('<f',bytes(payload)[4:])[0]
            if self.marker_jaw_hold is not None:
                if not math.isfinite(value) or abs(value-self.marker_jaw_hold)>.000001:
                    raise ValueError('Marker observation must preserve measured aperture')
                return self.device.enqueue(identifier,payload,*args,**kwargs)
            if value>self.jaw_ceiling+1e-6 or value<1.08:
                hold=self.initial_jaw_hold
                if hold is None or not (0.<=hold<1.08 or 1.62<hold<=1.70) or abs(value-hold)>.0001:
                    raise ValueError('Only the exact measured initial jaw hold is permitted')
                return self.device.enqueue(identifier,payload,*args,**kwargs)
        if identifier&255==7 and identifier>>24==18 and int.from_bytes(bytes(payload)[:2],'little')==0x7018:
            current=struct.unpack('<f',bytes(payload)[4:])[0]
            if not 0<current<=TRANSFER_GRIPPER_CURRENT_MAX_A+1e-6:raise ValueError('Reviewed gripper current envelope')
            return self.device.enqueue(identifier,payload,*args,**kwargs)
        return super().enqueue(identifier,payload,*args,**kwargs)


def marker_contract(job,p):
    """One reviewed wrist-5 tilt and return, fixed measured aperture, bounded capture."""
    q=job['start_rad'];actual=job['start_motor_rad'];jaw=q[6]
    if job.get('startup_no_reset') is not True or job.get('motion_mask')!=127 or job['profile_id']!=p.profile_id:
        raise ValueError('Explicit marker observation and no startup reset required')
    if not .35<=jaw<=1.62 or abs(jaw-actual[6])>1e-7 or job['open_rad']!=jaw:
        raise ValueError('Fresh measured fixed marker aperture required')
    if (job.get('marker_capture_seconds')!=3. or job.get('automatic_task_failure_return') or
            job.get('recoveries') or job.get('targets') or job.get('initial_open_during_departure')):
        raise ValueError('Bounded marker capture only, no jaw action or grasp')
    if p.current[6]>.500001 or any(v>hi for v,hi in zip(p.velocity,[.3]*6+[.25])):
        raise ValueError('Marker observation speed/current envelope')
    if job.get('gripper_open_current_a')!=p.current[6] or job.get('gripper_close_current_a')!=p.current[6]:
        raise ValueError('Fixed configured current required')
    lifting=job.get('policy')==MARKER_LIFT_POLICY
    names=['marker_start_approach','marker_lift_approach' if lifting else 'marker_tilt_approach','return_supported']
    if [t['name'] for t in job['tasks']]!=names:
        raise ValueError('Marker observation has one tilt and supported return')
    tasks=job['tasks']
    if max(abs(a-b) for a,b in zip(tasks[0]['goal_rad'],q))>1e-9 or max(abs(a-b) for a,b in zip(tasks[-1]['goal_rad'],q))>1e-9 or not tasks[-1].get('return_to_live_start'):
        raise ValueError('Marker observation must start and return at reviewed support')
    delta=tasks[1]['goal_rad'][4]-q[4]
    if not lifting and not .20<=abs(delta)<=.35:raise ValueError('Reviewed wrist-5 tilt range required')
    if lifting and max(abs(a-b) for a,b in zip(tasks[1]['goal_rad'][:6],q[:6]))>.25:raise ValueError('Short lift joint excursion exceeded')
    for task in tasks:
        for k in ([6] if lifting else [0,1,2,3,5,6]):
            if abs(task['goal_rad'][k]-q[k])>1e-9:raise ValueError('Only wrist 5 may move')
        for s in task['segments']:
            for k in range(7):
                lo,hi=extrema(s['coefficients_rad'][k])
                if (k==6 if lifting else k!=4) and max(abs(lo-q[k]),abs(hi-q[k]))>1e-8:
                    raise ValueError('Non-wrist or jaw motion forbidden during marker observation')
                if lifting and (lo<min(q[k],tasks[1]['goal_rad'][k])-1e-8 or hi>max(q[k],tasks[1]['goal_rad'][k])+1e-8):
                    raise ValueError('Lift polynomial outside endpoint joint envelope')
                if not lifting and k==4 and (lo<min(q[4],q[4]+delta)-1e-8 or hi>max(q[4],q[4]+delta)+1e-8):
                    raise ValueError('Wrist tilt polynomial outside reviewed interval')
    audit=job['geometry_audit']
    if (audit.get('status')!='PASS_SAMPLED_WITH_MONOTONIC_EXISTING_START_CONTACT_DEPARTURE' or
            audit.get('unexpected') or audit.get('trajectory_sha256')!=path_digest(job) or
            audit.get('finger_extension_m',0)<.035 or
            not math.isfinite(audit.get('extended_finger_min_floor_clearance_m',float('nan'))) or
            audit['extended_finger_min_floor_clearance_m']<.02):
        raise ValueError('Exact extended-finger swept geometry and clearance required')
    if lifting:
        checks=[('maximum_end_displacement_m',0.,.05),('end_goal_lift_m',.039,.041),('end_goal_rotation_error_rad',0.,.001),('minimum_end_height_change_m',-.002,.05)]
        for key,lo,hi in checks:
            value=audit.get(key,float('nan'))
            if not math.isfinite(value) or not lo<=value<=hi:raise ValueError('Reviewed short upward endpoint and sampled sweep required: '+key)


def contract(job,p,path_sha=None):
    if job.get('policy') in MARKER_POLICIES:
        marker_contract(job,p)
        return
    jaw_upper=initial_jaw_upper(job);jaw_lower=initial_jaw_lower(job)
    if job.get('batch_return_temperature_c') is not None:
        if job['batch_return_temperature_c']!=75. or not job.get('automatic_task_failure_return'):
            raise ValueError('Batch temperature exit requires fixed threshold and reviewed return')
    if job.get('startup_no_reset') is not True:raise ValueError('Transfer startup must not issue unapproved reset')
    hover=job.get('policy')==HOVER_POLICY
    if job.get('policy') not in (POLICY,HOVER_POLICY) or job.get('motion_mask')!=127 or job['profile_id']!=p.profile_id:raise ValueError('Reviewed transfer policy')
    jaw_current_limit=.5 if hover else TRANSFER_GRIPPER_CURRENT_MAX_A
    if not 0<p.current[6]<=jaw_current_limit+1e-6 or p.velocity[6]>.250001 or p.csp_velocity[6]>.350001:raise ValueError('Jaw speed/current envelope')
    if any(v>hi for v,hi in zip(p.velocity[:6],[.71875,.43125,.43125,.71875,.71875,.71875])):raise ValueError('Arm speed envelope')
    if job.get('gripper_close_current_a',.3)!=p.current[6] or job.get('gripper_open_current_a',.3)!=p.current[6]:raise ValueError('Configure one fixed current before PL start')
    if job['open_rad']!=1.12:raise ValueError('Observed opening required')
    if not hover and (job['close_rad'] not in JAW_CEILINGS or job['grasp_height_offset_m']!=expected_grasp_height_offset_m(target_classes(job))):raise ValueError('Observed grasp settings')
    names=['departure','screw_approach','screw_close','screw_lift','screw_transfer','screw_place','screw_open','screw_exit',
           'nut_transfer_empty','nut_approach','nut_close','nut_lift','nut_transfer','nut_place','nut_open','nut_exit','return_supported']
    if hover:
        names=['departure','hover_approach','hover_lift','return_supported']
        if (not .06<=job.get('hover_clearance_m',0)<=.15 or
                job.get('hover_capture_seconds')!=2. or job.get('automatic_task_failure_return') or
                job.get('recoveries') or job.get('targets')):
            raise ValueError('Noncontact hover only, bounded capture and no grasp targets')
        audit=job['geometry_audit']
        if (not math.isfinite(audit.get('hover_min_clearance_m',float('nan'))) or
                audit['hover_min_clearance_m']<job['hover_clearance_m']-2e-6):
            raise ValueError('Reviewed hover clearance required')
    elif 'targets' in job:
        targets=job['targets'];ids=[x['target_id'] for x in targets]
        if not 1<=len(ids)<=5 or len(set(ids))!=len(ids) or any(not re.fullmatch(r'(screw|nut)_[0-4]',i) for i in ids):raise ValueError('One to five explicit targets')
        names=[]
        for i,target in enumerate(ids):
            names.append('departure' if i==0 else target+'_transfer_empty')
            names.extend(target+'_'+phase for phase in ['approach','close','lift','transfer','place','open','exit'])
        names.append('return_supported')
    chain_index,chain_count,support=chain_of(job)
    last=chain_index==chain_count-1
    if not last:
        if names[-1]!='return_supported' or hover:raise ValueError('Chain job layout')
        names=names[:-1]
    if [t['name'] for t in job['tasks']]!=names:raise ValueError('Fixed transfer sequence')
    if last:
        return_goal=support+[job['open_rad']]
        if not job['tasks'][-1].get('return_to_live_start') or max(abs(a-b) for a,b in zip(job['tasks'][-1]['goal_rad'],return_goal))>1e-10:raise ValueError('Supported return required')
    elif any(t.get('return_to_live_start') for t in job['tasks']):raise ValueError('Only the last chain job returns')
    if chain_index>0 and (job.get('initial_open_during_departure') or job.get('initial_open_from_observed_closed') or job.get('initial_normalize_from_observed_wide')):
        raise ValueError('Chain jobs start at a working-open exit pose')
    audit=job['geometry_audit']
    if audit.get('status')!='PASS_SAMPLED_WITH_MONOTONIC_EXISTING_START_CONTACT_DEPARTURE' or audit.get('unexpected') or audit['trajectory_sha256']!=(path_sha or path_digest(job)):raise ValueError('Exact swept trajectory review required')
    for task in job['tasks']:
        if hover and abs(task['goal_rad'][6]-job['open_rad'])>1e-9:
            raise ValueError('Hover must keep the jaw open')
        if task['name']=='departure' and job.get('initial_open_during_departure'):
            wide=job.get('initial_normalize_from_observed_wide',False)
            valid=(jaw_lower<=job['start_rad'][6]<1.08<=job['open_rad']) if wide else (1.08<=job['open_rad']<=job['start_rad'][6]<=jaw_upper)
            if not valid or abs(task['goal_rad'][6]-job['open_rad'])>1e-10:raise ValueError('Bounded aperture normalization during departure')
            for s in task['segments']:
                low,peak=extrema([i*v for i,v in enumerate(s['coefficients_rad'][6])][1:])
                if (wide and low < -1e-8) or (not wide and peak>1e-8):raise ValueError('Departure aperture must be monotone toward working opening')
        elif task['name'].endswith(('_close','_open')):
            wanted=job['close_rad'] if task['name'].endswith('_close') else job['open_rad']
            if abs(task['goal_rad'][6]-wanted)>1e-9:raise ValueError('Fixed jaw action goal required')
            for s in task['segments']:
                if max(abs(s['goal_rad'][k]-s['start_rad'][k]) for k in range(6))>1e-10:raise ValueError('Arm stationary during jaw action')
        elif any(any(abs(x)>1e-10 for x in s['coefficients_rad'][6][1:]) for s in task['segments']):raise ValueError('Jaw held during arm movement')
    if job.get('automatic_task_failure_return'):
        expected={t['name'] for t in job['tasks'] if t['name'].endswith(('_close','_open'))}|{'departure'}
        if not last:expected|={t['name'] for t in job['tasks'] if t['name'].endswith('_exit')}
        if set(job.get('recoveries',{}))!=expected:raise ValueError('Every jaw outcome requires reviewed return')
        for t in job['tasks']:
            if t['name'] not in expected:continue
            branch=job['recoveries'][t['name']]
            if max(abs(a-b) for a,b in zip(branch['start_rad'],t['goal_rad']))>1e-9:raise ValueError('Recovery start binding')
            if max(abs(a-b) for a,b in zip(branch['goal_rad'][:6],support))>1e-9:raise ValueError('Recovery must reach supported arm pose')
            for s in branch['segments']:
                if abs(s['start_rad'][6]-t['goal_rad'][6])>1e-9 or any(abs(v)>1e-9 for v in s['coefficients_rad'][6][1:]):raise ValueError('Recovery holds aperture command')


def verify(job,p,path_sha=None):
    contract(job,p,path_sha);records=PreparedRecords(verify_septic_job(job,p,return_prepared=True));records.recoveries={}
    for name,branch in job.get('recoveries',{}).items():
        records.recoveries[name]=verify_septic_job(dict(schema=job['schema'],profile_id=p.profile_id,start_rad=branch['start_rad'],tasks=[branch]),p,return_prepared=True)
    all_records=list(records)+[r for seq in records.recoveries.values() for r in seq]
    departure_ids={id(s) for s in job['tasks'][0]['segments']}
    for i in range(7):
        lo,hi=job['motor_lower_rad'][i],job['motor_upper_rad'][i];legal=sorted((p.lower[i]*p.direction[i]+p.offset[i],p.upper[i]*p.direction[i]+p.offset[i]))
        jaw_bounds=(job['start_rad'][6]-.01,job['start_rad'][6]+.01) if job.get('policy') in MARKER_POLICIES else (initial_jaw_lower(job),initial_jaw_upper(job))
        if not legal[0]<=lo<hi<=legal[1] or i==6 and not jaw_bounds[0]<=lo<hi<=jaw_bounds[1]:raise ValueError('Guard bounds')
        for r in all_records:
            a,b=extrema(r['words'][8*i:8*i+8])
            if a/Q24<lo or b/Q24>hi:raise ValueError('Quantized limits')
            if i==6 and id(r['plan']) not in departure_ids and b/Q24>jaw_ceiling(job)+1e-6:
                raise ValueError('Closed-start allowance applies only to initial opening')
            if i==6 and job.get('initial_normalize_from_observed_wide') and id(r['plan']) not in departure_ids and a/Q24<1.079999:
                raise ValueError('Wide-start allowance applies only to initial normalization')
    return records


def export_bundle(job,p,path):
    digest,path_sha=job_and_path_digests(job)
    records=verify(job,p,path_sha)
    payload=dict(schema='reviewed_transfer_bundle.v1',job_sha256=digest,profile_id=p.profile_id,
                 source_sha256=source_hashes(),records=records,recovery_records=records.recoveries)
    raw=json.dumps(payload,separators=(',',':'),allow_nan=False).encode();Path(path).write_bytes(raw)
    return hashlib.sha256(raw).hexdigest()


def load_bundle(job,p,path,expected_sha):
    raw=Path(path).read_bytes()
    if hashlib.sha256(raw).hexdigest()!=expected_sha:raise ValueError('Reviewed bundle hash mismatch')
    data=json.loads(raw);digest,path_sha=job_and_path_digests(job);contract(job,p,path_sha)
    if data.get('schema')!='reviewed_transfer_bundle.v1' or data['job_sha256']!=digest or data['profile_id']!=p.profile_id or data['source_sha256']!=source_hashes():raise ValueError('Review dependencies changed; regenerate offline')
    plans=[s for t in job['tasks'] for s in t['segments']];records=PreparedRecords(data['records']);records.recoveries=data.get('recovery_records',{})
    if len(plans)!=len(records):raise ValueError('Review record count')
    for plan,record in zip(plans,records):
        if record['plan']!=plan:raise ValueError('Review plan binding')
        record['plan']=plan # Transport requires the exact job object identity.
    if set(records.recoveries)!=set(job.get('recoveries',{})):raise ValueError('Recovery cache binding')
    for name,seq in records.recoveries.items():
        plans=job['recoveries'][name]['segments']
        if len(seq)!=len(plans):raise ValueError('Recovery record count')
        for record,plan in zip(seq,plans):
            if record['plan']!=plan:raise ValueError('Recovery plan binding')
            record['plan']=plan
    return records


def recover_task_outcome(e,job,records,name,report):
    if e.state!='HOLDING' or name not in records.recoveries:raise RuntimeError('No reviewed rest-point return')
    if e.d.read(0x418) or e.d.read(0x5a8)!=127 or e.transport.status()['fault']:raise RuntimeError('Controller cannot execute return')
    branch=job['recoveries'][name]
    expected=e.p.to_motor(branch['start_rad'])
    actual=[e.s.require(mid,2)['position'] for mid in range(1,8)]
    errors=[abs(a-b) for a,b in zip(actual[:6],expected[:6])]
    report['recovery_start_check']=dict(task=name,actual_motor_rad=actual,arm_errors_rad=errors,tolerance_rad=.004)
    # Jaw contact intentionally prevents reaching the close command. Validate
    # the six arm axes, without treating a blocked jaw as an arm pose mismatch.
    if any(not math.isfinite(x) or x>.004 for x in errors):raise RuntimeError('Actual arm pose differs from reviewed return start')
    e.transport.prepared=records.recoveries[name];e.transport.prepared_index=0
    e.event('TASK_FAILED_RETURNING_SUPPORTED',task=name)
    e.execute_segments(branch['segments']);report['recovery_endpoint']=e.settle_arm(branch['goal_rad'])
    report['automatic_supported_return_completed']=True
    e.event('VERIFIED_START_RETURN', status='TASK_FAILED_RETURNED_SUPPORTED', endpoint=report['recovery_endpoint'])


def execute(job,p,out,records):
    if not job.get('hardware_start_verified'):raise ValueError('Fresh physical start required')
    out=Path(out);out.mkdir(exist_ok=False,parents=True)
    report=dict(status='RUNNING',commands=[],frames=[],feedback=[],readbacks={},reset_confirmed=[],stop_errors={},
                job_sha256=job_digest(job),physical_success_verified=False,flash_writes=0,zero_writes=0)
    (out/'job.json').write_text(json.dumps(job,indent=2)+'\n');supported=False
    with motion_gc_scope(),open('/tmp/kv260_gripper_commission.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);d=TransferDevice(CanDevice());e=GraspExecutor(p,d,report,records)
        if job.get('initial_open_from_observed_closed') or job.get('initial_normalize_from_observed_wide'):d.initial_jaw_hold=job['start_motor_rad'][6]
        if job.get('policy') in MARKER_POLICIES:d.marker_jaw_hold=job['start_motor_rad'][6]
        e.wait_state_writer=None
        try:
            # Thread creation can block under load. Do it before enabling any
            # motor, not on entry to the heartbeat-serviced operator wait.
            e.wait_state_writer=WaitStateWriter(out)
            report['wait_writer_prepared_before_enable']=True
            e.prepare(job);e.settle_arm(job['start_rad'],seconds=.15);e.release_host_hold()
            for index,task in enumerate(job['tasks']):
                name=task['name']
                report['active_task']=name
                e.event('TASK',name=name)
                # Driver rejects normal Type18 writes while PL owns targets.
                # Current is configured once in prepare, never between tasks.
                e.execute_segments(task['segments'])
                if name=='departure':d.initial_jaw_hold=None
                report['last_completed_task']=name
                if name.endswith(('_approach','_place')):
                    report.setdefault('pose_checks',[]).append(dict(task=name,**e.settle_arm(task['goal_rad'],seconds=.08)))
                elif name.endswith(('_close','_open')):
                    e.pump(.10);actual=e.s.read(7,0x7019)
                    report.setdefault('jaw_observations',[]).append(dict(task=name,actual_rad=actual,command_rad=task['goal_rad'][6]))
                    e.event('JAW_OUTCOME',task=name,actual_rad=actual,command_rad=task['goal_rad'][6])
                    if name.endswith('_open') and abs(actual-job['open_rad'])>.015:raise TaskOutcomeError('Jaw release not reached')
                    if name.endswith('_close') and job.get('automatic_task_failure_return') and actual>=job['close_rad']-.02:raise TaskOutcomeError('Empty closure: grasp not confirmed')
                    if name.endswith('_open'):check_batch_release_temperature(e,job,name)
                if name.endswith(('_approach','_lift','_open')):
                    # A snapshot is evidence, not an instruction to stop and wait
                    # for a language-model response while holding a component.
                    mark=dict(task=name,index=index,board_monotonic=time.monotonic())
                    if job.get('policy') in MARKER_POLICIES:
                        mark['actual_motor_rad']=report['pose_checks'][-1]['actual_motor_rad']+[e.s.read(7,0x7019)]
                    f=out/f'snapshot_{index}.json';f.write_text(json.dumps(mark))
                    if job.get('policy')==HOVER_POLICY and name=='hover_approach':
                        # Capture is automatic; no unbounded operator wait aloft.
                        e.hold(job['hover_capture_seconds'])
                    if job.get('policy') in MARKER_POLICIES:
                        e.hold(job['marker_capture_seconds'])
                        end=dict(task=name,index=index,capture_end_board_monotonic=time.monotonic(),
                                 actual_motor_rad=[e.s.read(mid,0x7019) for mid in range(1,8)])
                        report.setdefault('marker_capture_ends',[]).append(end)
                        (out/f'capture_end_{index}.json').write_text(json.dumps(end))
            report['return_endpoint']=e.settle_arm(job['tasks'][-1]['goal_rad']);supported=True
            report['status']='COMPLETED_ALIGNMENT_HOVER_RETURN' if job.get('policy')==HOVER_POLICY else 'COMPLETED_TRANSFER_MOTION_VERIFY_IMAGES'
            if job.get('policy') in MARKER_POLICIES:report['status']='COMPLETED_MARKER_OBSERVATION_RETURN'
            e.event('VERIFIED_START_RETURN', status=report['status'], endpoint=report['return_endpoint'])
        except (Exception,KeyboardInterrupt) as error:
            report.update(status='FAIL',error=repr(error));e.event('CONTINUOUS_TRANSFER_FAILED',error=repr(error))
            # Freeze the fault evidence before the long-running hold service
            # bounds its live telemetry buffer. Never discard the trip window.
            context=dict(error=repr(error),task=locals().get('task',{}).get('name'),
                         feedback_tail=report['feedback'][-512:],
                         tracking_diagnostics=report.get('tracking_diagnostics',[])[-128:],
                         velocity_monitor_failures=report.get('velocity_monitor_failures',[])[:],jaw_observations=report.get('jaw_observations',[])[:])
            report['failure_context']=context
            # Evidence I/O must not prevent the recovery/hold path from running.
            try:
                e.d.heartbeat();(out/'failure_context.json').write_text(json.dumps(context));e.d.heartbeat()
            except Exception as evidence_error:report['failure_context_write_error']=repr(evidence_error)
            if report.get('enable_attempted'):
                recovery_name=task['name'] if isinstance(error,TaskOutcomeError) else report.get('last_completed_task')
                recoverable=isinstance(error,TaskOutcomeError) or isinstance(error,ValueError) and recovery_name=='departure'
                if recoverable and job.get('automatic_task_failure_return'):
                    try:recover_task_outcome(e,job,records,recovery_name,report);supported=True
                    except (Exception,KeyboardInterrupt) as recovery_error:
                        report['automatic_return_error']=repr(recovery_error)
                        e.event('AUTOMATIC_RETURN_REJECTED',error=repr(recovery_error),
                                recovery_start_check=report.get('recovery_start_check'))
                if not supported:retain_until_supported(e,out,report);supported=True
        finally:
            if report.get('owned'):
                if report.get('enable_attempted'):
                    if not supported:retain_until_supported(e,out,report)
                    await_disable_approval(e,out,report)
                    try:e.s.stop_all()
                    except (Exception,KeyboardInterrupt) as error:
                        report['stop_errors']['fatal']=repr(error);retain_until_supported(e,out,report)
                    if sorted(report['reset_confirmed'])!=list(range(1,8)) or report['stop_errors']:report['status']='FAIL_DISABLE_CONFIRMATION'
                    elif report.get('error'):report['status']='TASK_FAILED_RETURNED_SUPPORTED' if report.get('automatic_supported_return_completed') else 'FAILED_STOPPED_AFTER_OPERATOR_SUPPORT'
                else:
                    d.stop();d.write(0x5a0,0)
                    for mid in range(1,8):d.write(d.pp_base(mid),0)
            if e.wait_state_writer is not None:e.wait_state_writer.close()
            report['final_controller']=d.status();report['final_motion_mask']=d.read(0x5a0);report['final_active_mask']=d.read(0x5a8);report['heartbeat_max_gap_s']=d.max_gap;d.close()
            summary={k:report.get(k) for k in ('status','error','reset_confirmed','stop_errors','final_controller','final_motion_mask','final_active_mask','heartbeat_max_gap_s','jaw_observations','hold_hardware_fault','motion_status_before_wait_fault','automatic_return_error','recovery_start_check')}
            tmp=out/'runtime_summary.tmp';tmp.write_text(json.dumps(summary));tmp.replace(out/'runtime_summary.json')
            raw=json.dumps(report,indent=2).encode();tmp=out/'result.tmp.gz';tmp.write_bytes(gzip.compress(raw,compresslevel=1));tmp.replace(out/'result.json.gz')
    return report


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for key in ('job','profile','out'):ap.add_argument('--'+key,type=Path,required=True)
    ap.add_argument('--bundle',type=Path);ap.add_argument('--bundle-sha256');ap.add_argument('--execute-attended',action='store_true');a=ap.parse_args()
    job=json.loads(a.job.read_text());p=JointPlanner(a.profile);started=time.monotonic()
    records=load_bundle(job,p,a.bundle,a.bundle_sha256) if a.bundle else verify(job,p)
    print(json.dumps(dict(status='PREPARED',wall_seconds=time.monotonic()-started,segments=len(records),cached=bool(a.bundle))),flush=True)
    if not a.execute_attended:return
    def interrupt(*args):raise KeyboardInterrupt('Retain until supported')
    signal.signal(signal.SIGTERM,interrupt);execute(job,p,a.out,records)


if __name__=='__main__':main()
