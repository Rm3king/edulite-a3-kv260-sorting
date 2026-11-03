#!/usr/bin/python3
"""Current-pose C3 pick with explicit Type1 damping, velocity and dynamics FF.

Default is exhaustive OFFLINE validation. Execution uses the existing v6 HLS
q/v/tau path. One CAN owner holds while an isolated worker rebinds the path;
there is no numerical trajectory or dynamics work after PL starts.
"""
import argparse,fcntl,gc,json,signal,time
from control_continuity import finish_failed_job
from pathlib import Path
import numpy as np
from joint_trajectory import JointPlanner
from motion_runtime import MotionRuntime
from motion_executor import verify_job,job_digest,CanDevice
from gravity_trajectory import GravityModel,prepare_stream,validate_records
from gravity_executor import GravityExecutor
from live_motion import POLICY,start_binding_worker,bind_while_holding
from septic_trajectory import derivative


def validate(job,p,m):
    if job.get('start_policy')!=POLICY:raise ValueError('current-feedback replan job required')
    if job['model_id']!=m.r.model_id:raise ValueError('model mismatch')
    verify_job(job,p)
    for task in job['tasks']:task['gravity_alpha']=[1.,1.]
    records=prepare_stream(m,job['tasks']);validate_records(records,job,p,m)
    mass=[];gravity=[];dynamic=[];feedforward=[]
    for t in job['tasks']:
        for plan in t['segments']:
            for u in np.linspace(0,1,9):
                q=[derivative(row,u) for row in plan['coefficients_rad']]
                v=[derivative(row,u,1)/plan['duration_s'] for row in plan['coefficients_rad']]
                a=[derivative(row,u,2)/plan['duration_s']**2 for row in plan['coefficients_rad']]
                g=m.gravity(q);ff=m.feedforward(q,v,a)
                mass.append(np.diag(m.r.kin.mass_matrix(q[:6]))[:6])
                gravity.append(g);feedforward.append(ff);dynamic.append(ff-g)
    mass=np.array(mass)
    info=dict(status='PASS_OFFLINE_NOT_PHYSICAL',job_sha256=job_digest(job),model_id=m.r.model_id,
        tracking_profile_id=m.id,profile_id=p.profile_id,segments=len(records),kp=m.kp.tolist(),kd=m.kd.tolist(),
        torque_caps_nm=m.caps.tolist(),inverse_dynamics_scale=m.dynamic_scale,
        nominal_seconds=sum(s['duration_s'] for t in job['tasks'] for s in t['segments']),
        task_seconds={t['name']:sum(s['duration_s'] for s in t['segments']) for t in job['tasks']},
        max_sampled_gravity_nm=np.abs(gravity).max(0).tolist(),
        max_sampled_added_dynamic_nm=np.abs(dynamic).max(0).tolist(),
        max_sampled_feedforward_nm=np.abs(feedforward).max(0).tolist(),
        max_fit_error_nm=max(r['sampled_gravity_fit_error_nm'] for r in records),
        mass_diagonal_min=mass.min(0).tolist(),mass_diagonal_max=mass.max(0).tolist(),
        diagonal_damping_ratio_min=(m.kd/(2*np.sqrt(m.kp*mass.max(0)))).tolist(),
        diagonal_damping_ratio_max=(m.kd/(2*np.sqrt(m.kp*mass.min(0)))).tolist(),
        note='Rigid-model diagonal estimates omit motor/gearbox reflected inertia, friction, coupling and payload; not a stability or physical smoothness proof.',
        physical_validation='WAITING',external_tcp_measurement='WAITING')
    return records,info


def execute(job,p,model,profile,tracking_profile,output):
    begin=time.monotonic();prepared,review=validate(job,p,model)
    if not job.get('hardware_start_verified'):raise ValueError('synthetic start cannot execute')
    report=dict(status='RUNNING',commands=[],frames=[],feedback=[],readbacks={},reset_confirmed=[],stop_errors={},
        flash_writes=False,job_sha256=job_digest(job),profile_id=p.profile_id,model_id=model.r.model_id,
        tracking_profile_id=model.id,tracking_config=model.config,offline_review=review,
        prevalidation_seconds=time.monotonic()-begin,started_monotonic=time.monotonic())
    lock=open('/tmp/kv260_gripper_commission.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    (Path(output).parent/'SUPPORTED_DISABLE').unlink(missing_ok=True)
    worker=None;d=None;executor=None;gc_before=gc.isenabled()
    try:
        # Imports/model construction happen in the worker before opening hardware.
        worker=start_binding_worker(job,profile,tracking_profile)
        d=CanDevice();executor=GravityExecutor(p,d,report,prepared,model)
        gc.collect();gc.disable()
        executor.prepare(job,current_tracking=True)
        job=bind_while_holding(executor,job,profile,worker)
        report['bound_job_sha256']=report['binding_job_sha256']
        report['encoded_records']=[dict(task=r['task'],words=r['words'],plan=r['plan']) for r in executor.transport.prepared]
        executor.release_host_hold()
        for task in job['tasks']:
            executor.event('TASK',name=task['name'],kind=task['kind']);start=time.monotonic()
            executor.execute_segments(task['segments'])
            endpoint=executor.settle(task['goal_rad'],task.get('final_hold_s',.25))
            report.setdefault('task_results',[]).append(dict(name=task['name'],kind=task['kind'],begin=start,end=time.monotonic(),**endpoint))
        report['status']='PASS_DAMPED_PICK'
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='FAIL',error=repr(exc))
        if d is not None:report['failure_controller']=d.status()
    finally:
        if executor is not None and report.get('owned'):
            if report.get('enable_attempted'):
                if not report['status'].startswith('PASS'):
                    finish_failed_job(executor,job,Path(output).parent,report)
                try:executor.s.stop_all()
                except Exception as exc:report['stop_errors']['fatal']=repr(exc)
                if sorted(report['reset_confirmed'])!=list(range(1,8)) or report['stop_errors']:report['status']='FAIL_STOP'
            else:
                d.stop();d.write(0x5a0,0)
                for mid in range(1,8):d.write(d.pp_base(mid),0)
            reset_ok=(sorted(report['reset_confirmed'])==list(range(1,8)) and not report['stop_errors']) if report.get('enable_attempted') else report.get('initial_reset_confirmed')==list(range(1,8))
            if report.get('volatile_configuration_changed') and reset_ok:
                try:executor.restore_csp_configuration()
                except Exception as exc:report.update(status='FAIL_RESTORE',restore_error=repr(exc))
            report['final_controller']=d.status();report['final_motion_mask']=d.read(0x5a0)
        stamp=report.get('pl_started_monotonic',float('inf'))
        report['host_targets_after_pl_start']=sum((c['id']>>24==1 or (c['id']>>24==18 and c['data'].startswith('1670'))) and c['time']>=stamp for c in report['commands'])
        report['finished_monotonic']=time.monotonic()
        Path(output).write_text(json.dumps(report,indent=2)+'\n')
        Path(output).with_suffix('.bound_job.json').write_text(json.dumps(job,indent=2)+'\n')
        if d is not None:d.close()
        if worker is not None:
            conn,process=worker
            if process.is_alive():process.terminate()
            process.join(timeout=2);conn.close()
        lock.close()
        if gc_before:gc.enable()
    print(json.dumps({k:report.get(k) for k in ('status','error','reset_confirmed','stop_errors','host_targets_after_pl_start')},indent=2))
    return report


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    for name in ('job','profile','tracking-profile','output'):ap.add_argument('--'+name,required=True)
    ap.add_argument('--execute-attended',action='store_true');a=ap.parse_args()
    job=json.loads(Path(a.job).read_text());p=JointPlanner(a.profile);m=GravityModel(MotionRuntime(),p,a.tracking_profile)
    if a.execute_attended:
        def interrupt(*args):raise KeyboardInterrupt('operator interrupt')
        signal.signal(signal.SIGTERM,interrupt)
        result=execute(job,p,m,a.profile,a.tracking_profile,a.output)
        raise SystemExit(result['status']!='PASS_DAMPED_PICK')
    _,info=validate(job,p,m);Path(a.output).write_text(json.dumps(info,indent=2)+'\n');print(json.dumps(info,indent=2))
