"""Numerical/analysis worker for attended persistent tuning; never opens UIO."""
import copy,gzip,hashlib,json,math,pickle,traceback
from pathlib import Path
import numpy as np
from joint_trajectory import JointPlanner
from motion_runtime import MotionRuntime
from gravity_trajectory import GravityModel,prepare_stream,validate_records,hold_segment
from septic_trajectory import movej_c3
from live_motion import current_reference
from pick_return import prepare as pick_prepare
from tuning_metrics import evaluate,reference,elapsed
from tuning_search import suggest,candidates

ANCHOR=np.deg2rad([-60,40,-100,25,20,-10]).tolist()+[.35]


def bank_key(p,m):
    sources=['joint_trajectory.py','gravity_trajectory.py','septic_trajectory.py','tuning_worker.py']
    raw=''.join(hashlib.sha256(Path(__file__).with_name(n).read_bytes()).hexdigest() for n in sources)
    return hashlib.sha256((p.profile_id+m.id+m.r.model_id+raw).encode()).hexdigest()


def guard_job(p,tasks):
    b=[sorted((a*d+o,c*d+o)) for a,c,d,o in zip(p.lower,p.upper,p.direction,p.offset)]
    return dict(tasks=tasks,motor_lower_rad=[x[0] for x in b],motor_upper_rad=[x[1] for x in b])


def encode(m,tasks):
    records=prepare_stream(m,tasks);validate_records(records,guard_job(m.p,tasks),m.p,m)
    return records


def task(name,plans,goal):return dict(name=name,segments=plans,goal_rad=goal,gravity_alpha=[1.,1.])


def probe(p,axis):
    tasks=[task('gain_settle',[hold_segment(p,ANCHOR,1.)],ANCHOR)]
    q=ANCHOR
    for k,delta in enumerate([math.radians(5),0.,-math.radians(5),0.]):
        goal=ANCHOR.copy();goal[axis]+=delta
        tasks.append(task('probe_'+str(k),movej_c3(p,q,goal,minimum_duration_s=1.5),goal));q=goal
    tasks.append(task('end_settle',[hold_segment(p,ANCHOR,1.)],ANCHOR))
    return tasks


def patch_gains(records,kp,kd,m):
    kp=np.asarray(kp,float);kd=np.asarray(kd,float)
    if kp.shape!=(6,) or kd.shape!=(6,) or not np.isfinite(kp).all() or not np.isfinite(kd).all() or np.any(kp<=0) or np.any(kp>m.guard_kp) or np.any(kd<0) or np.any(kd>m.guard_kd):raise ValueError('candidate gains exceed prepared bounds')
    gain_words=tuple((math.floor(k*65535/500)<<16)|math.floor(d*65535/5) for k,d in zip(kp,kd))
    return [dict(r,words=r['words'][:210]+gain_words+(r['words'][216],)+r['words'][217:]) for r in records]


def pick_tasks(capture,p,r):
    job,_=pick_prepare(capture,p,r,.10,.08,.04)
    for t in job['tasks']:
        t['gravity_alpha']=[1.,1.]
        t['segments']=t['segments']+[hold_segment(p,t['goal_rad'],1. if t['name']=='Release_at_start' else .35)]
    return job['tasks']


def calibration_tasks(p):
    tasks=[];q=ANCHOR
    for n,delta in enumerate((0,15,-15,0)):
        goal=ANCHOR.copy();goal[1]+=math.radians(delta);goal[2]-=math.radians(delta);goal[3]+=math.radians(delta*2/3)
        tasks.append(task('calib_move_'+str(n),movej_c3(p,q,goal,minimum_duration_s=2.),goal))
        tasks.append(task('calib_hold_'+str(n),[hold_segment(p,goal,2.)],goal));q=goal
    return tasks


def calibrate_gravity(report,m):
    from collections import Counter
    records=report['encoded_records'];_,_,_,_,offsets=reference(records);base=report['feedback'][0]['pl_rx_us'];origins=[]
    for s in report['pl_samples']:
        i=s['completed']-report['start_completed']
        if 0<=i<len(records) and not s['holding']:origins.append(elapsed(s['next_due_us'],base)-s['tick']*5000-int(offsets[i])*5000)
    origin=Counter(origins).most_common(1)[0][0];points=[]
    for i,rec in enumerate(records):
        if not rec['task'].startswith('calib_hold_'):continue
        end=(origin+(int(offsets[i])+rec['plan']['ticks'])*5000)*1e-6
        qs=[];taus=[];speeds=[]
        for mid in range(1,7):
            fs=[f for f in report['feedback'] if f['motor']==mid and end-1.<elapsed(f['pl_rx_us'],base)*1e-6<end-.1]
            if len(fs)<100:raise ValueError('insufficient static calibration samples')
            qs.append(float(np.median([f['position'] for f in fs])));taus.append(float(np.median([f['torque'] for f in fs])));speeds.append(float(np.max(np.abs([f['velocity'] for f in fs]))))
        robot=[(a-o)/d for a,o,d in zip(qs,m.p.offset,m.p.direction)]+[.35]
        points.append(dict(q_robot=robot,gravity_motor_nm=m.gravity(robot).tolist(),measured_torque_nm=taus,max_velocity=speeds))
    g=np.array([x['gravity_motor_nm'] for x in points]);y=np.array([x['measured_torque_nm'] for x in points]);scales=[];details=[]
    for i in range(6):
        identifiable=bool(max(abs(g[:,i]))>.15 and np.ptp(g[:,i])>.10)
        alpha=float(g[:,i]@y[:,i]/(g[:,i]@g[:,i])) if identifiable else None
        residual=float(np.sqrt(np.mean((y[:,i]-(alpha or 1)*g[:,i])**2)))
        accepted=identifiable and .8<=alpha<=1.2 and residual<.15
        scales.append(alpha if accepted else 1.)
        details.append(dict(motor=i+1,identifiable=identifiable,raw_scale=alpha,residual_nm=residual,proposed=accepted))
    return dict(scale=scales,details=details,postures=points,scope='Effective gravity scale from motor torque telemetry; NOT independent torque-constant calibration. Static friction and contact can bias it; full pick validation required.')


def run(conn,profile,tracking_profile,out,full_pick_only=False):
    try:
        out=Path(out);out.mkdir(parents=True,exist_ok=True);p=JointPlanner(profile);r=MotionRuntime();m=GravityModel(r,p,tracking_profile)
        key=bank_key(p,m);cache=Path(profile).resolve().parent.parent/'tuning_bank_cache.pkl'
        saved=pickle.loads(cache.read_bytes()) if cache.exists() else {}
        cached=not full_pick_only and saved.get('key')==key
        if full_pick_only:bank={}
        elif cached:bank=saved['bank']
        else:
            bank={str(i):encode(m,probe(p,i)) for i in range(6)}
            bank['calibration']=encode(m,calibration_tasks(p))
        conn.send(('ready',dict(bank_segments={k:len(v) for k,v in bank.items()},model_id=r.model_id,cached=cached)))
        current_records=None
        while True:
            command,payload=conn.recv()
            if command=='quit':break
            if command=='entry':
                ref,_=current_reference(p,payload)
                if not full_pick_only:
                    bank['entry']=encode(m,[task('current_to_anchor',movej_c3(p,ref,ANCHOR),ANCHOR),task('anchor_settle',[hold_segment(p,ANCHOR,1.)],ANCHOR)])
                    bank['exit']=encode(m,[task('return_to_support',movej_c3(p,ANCHOR,ref),ref),task('supported_hold',[hold_segment(p,ref,1.)],ref)])
                capture=dict(status='PASS_READ_ONLY_ALL_7',transactions=[dict(motor=i+1,decoded=dict(valid=True,position_motor_rad=float(q))) for i,q in enumerate(payload)])
                supported_pick_tasks=pick_tasks(capture,p,r)
                if m.config.get('joint3_frequency_probe'):
                    from frequency_probe import build_bank
                    bank['pick'],probe_info=build_bank(m,capture,p,r)
                    (out/'probe_definition.json').write_text(json.dumps(probe_info,indent=2)+'\n')
                elif m.config.get('full_pick_experiment'):
                    from reference_smoothing import build_pair
                    bank['pick_control'],bank['pick'],info=build_pair(m,supported_pick_tasks)
                    (out/'method_definition.json').write_text(json.dumps(info,indent=2)+'\n')
                else:bank['pick']=encode(m,supported_pick_tasks)
                if full_pick_only and m.pl_damping_q8:
                    bank['pick_undamped']=[dict(rec,words=rec['words'][:216]+(0,)+rec['words'][217:]) for rec in bank['pick']]
                if full_pick_only and m.has_friction:
                    uncompensated=copy.copy(m)
                    uncompensated.friction_coulomb=np.zeros(6);uncompensated.has_friction=False
                    bank['pick_uncompensated']=encode(uncompensated,supported_pick_tasks)
                conn.send(('ok',dict(supported_reference_rad=ref)))
            elif command=='suggest':conn.send(('ok',suggest(payload['history'],payload['axis'])))
            elif command=='calibrate':
                raw=json.loads(gzip.decompress((out/'gravity_static_calibration.json.gz').read_bytes()))
                fit=calibrate_gravity(raw,m);m.scale=np.array(fit['scale'])
                bank['pick_calibrated']=encode(m,supported_pick_tasks)
                (out/'gravity_fit.json').write_text(json.dumps(fit,indent=2)+'\n');conn.send(('ok',fit))
            elif command=='records':
                key=payload['key'];current_records=patch_gains(bank[key],payload['kp'],payload['kd'],m)
                # Serialize before announcing readiness. Keep each pipe write
                # below PIPE_BUF, so poll() cannot expose only the header of a
                # large partially written record and block the CAN owner.
                encoded=[]
                for rec in current_records:
                    data=pickle.dumps(rec,protocol=5)
                    if len(data)>65536:raise ValueError('record exceeds bounded IPC size')
                    encoded.append(data)
                conn.send(('records_chunked',[len(data) for data in encoded]))
                for data in encoded:
                    for offset in range(0,len(data),2048):conn.send_bytes(data[offset:offset+2048])
            elif command=='analyze':
                report=dict(payload)
                if full_pick_only:
                    # Live trials are all finished before these messages arrive;
                    # reconstruct each immutable bank with its own logged gains.
                    current_records=patch_gains(bank[report['key']],report['kp'],report['kd'],m)
                # Releasing the previous report can take much longer than a
                # CAN service period. Grant credit only once ready to receive.
                conn.send(('analysis_ready',None))
                while True:
                    key,items=conn.recv()
                    if key=='end':break
                    report.setdefault(key,[]).extend(items)
                    conn.send(('chunk_ok',None))
                report['encoded_records']=[dict(plan=x['plan'],words=x['words'],task=x['task']) for x in current_records]
                try:
                    metrics=evaluate(report['feedback'],report['pl_samples'],current_records,report['start_completed'],report.get('axis',0),m.caps)
                    if full_pick_only:
                        from diagnose_motion_vibration import audit_trial
                        metrics['frequency_audit']=audit_trial(report)[0]
                except Exception as exc:metrics=dict(score=1e9,feasible=False,metric_error=repr(exc))
                if report.get('quality',[{}])[0].get('rx_lost',0):
                    metrics.update(score=1e9,global_score=1e9,feasible=False,measurement_invalid=True)
                report['metrics']=metrics
                raw=json.dumps(report,separators=(',',':'),allow_nan=False).encode()
                (out/(report['label']+'.json.gz')).write_bytes(gzip.compress(raw,compresslevel=3,mtime=0))
                (out/(report['label']+'.metrics.json')).write_text(json.dumps(metrics,indent=2)+'\n')
                with (out/'journal.jsonl').open('a') as journal:
                    journal.write(json.dumps(dict(label=report['label'],kp=report['kp'],kd=report['kd'],metrics=metrics))+'\n')
                conn.send(('ok',metrics))
            else:raise ValueError('unknown numerical worker command')
    except BaseException as exc:
        try:conn.send(('error',dict(error=repr(exc),traceback=traceback.format_exc())))
        except (EOFError,BrokenPipeError):pass
    finally:conn.close()
