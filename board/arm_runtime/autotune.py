#!/usr/bin/python3
"""Attended persistent six-axis MIT tuning, followed by complete pick trials.

The sole hardware owner only transports prevalidated records and drains CAN.
A spawned process handles RNEA, GP suggestions, metrics and gzip evidence.
Between trials PL holds the final q/v/tau, retaining enable. Software failures
return to support or retain a serviced hold. No blind re-enable, fault clearing,
or motor Flash/zero writes. Full-pick-only mode runs four cached captures, then
returns/disables before transferring, analyzing and compressing the evidence.
"""
import argparse,fcntl,gc,json,multiprocessing,pickle,signal,time
from pathlib import Path
from joint_trajectory import JointPlanner
from motion_runtime import MotionRuntime
from gravity_trajectory import GravityModel
from gravity_executor import GravityExecutor,CanDevice
from motion_executor import PathVelocityMonitor
from control_continuity import retain_until_supported,emit
from live_motion import POLICY
from tuning_worker import guard_job,run as worker_run


class TrialEvidence(list):
    """Record trial windows without storing prolonged enabled analysis holds."""
    def __init__(self,gate):super().__init__();self.gate=gate
    def append(self,value):
        if self.gate[0]:super().append(value)


def receive_records(conn,kind,count,pump):
    """Poll between atomic IPC chunks, retaining the live CAN service."""
    if kind not in ('records','records_chunked'):raise ValueError('worker record stream expected')
    received=[]
    for size in count if kind=='records_chunked' else [None]*count:
        if size is not None and not 0<size<=65536:raise ValueError('record IPC size')
        data=bytearray();deadline=time.monotonic()+30
        while size is None or len(data)<size:
            while not conn.poll():
                pump(.002)
                if time.monotonic()>deadline:raise TimeoutError('record IPC')
            data.extend(conn.recv_bytes(2048 if size is not None else 65536));pump(.001)
            if size is None:break
        if size is not None and len(data)!=size:raise ValueError('record IPC length mismatch')
        received.append(pickle.loads(data));pump(.001)
    return received


def upload_evidence(conn,report,header,quality,wait,pump):
    """Credit-controlled IPC: worker stalls must leave the CAN owner pumping."""
    conn.send(('analyze',header))
    if wait()[0]!='analysis_ready':raise ValueError('analysis credit expected')
    for name in ('commands','frames','feedback','pl_samples','segment_submissions','events','drive_telemetry'):
        evidence=report.get(name,[])
        report[name]=TrialEvidence(evidence.gate) if isinstance(evidence,TrialEvidence) else []
        chunk=1 if name=='segment_submissions' else 24
        for offset in range(0,len(evidence),chunk):
            conn.send((name,evidence[offset:offset+chunk]))
            if wait()[0]!='chunk_ok':raise ValueError('evidence credit expected')
            evidence[offset:offset+chunk]=[None]*min(chunk,len(evidence)-offset)
            pump(.001)
    conn.send(('quality',[quality]))
    if wait()[0]!='chunk_ok':raise ValueError('quality credit expected')
    conn.send(('end',None))
    return wait()[1]


SUPPORTED_KEYS=('exit','pick','pick_calibrated','pick_uncompensated','pick_undamped','pick_control')


def return_to_support(e,recovery_exit,current_key,summary):
    """Refresh observations, finish validated records, then return supported."""
    d=e.d
    if not recovery_exit:raise RuntimeError('no cached supported return available')
    if d.read(0x418) or e.transport.status()['fault'] or d.read(0x5a8)!=127:
        raise RuntimeError('controller cannot execute recovery')
    e.s.pump(.03)
    for mid in range(1,8):e.s.require(mid,2)
    e.state='HOLDING'
    rest=e.transport.prepared[e.transport.prepared_index:]
    if rest:e.execute_segments([r['original_plan'] for r in rest])
    else:
        deadline=time.monotonic()+15
        while not e.pump(.002)['holding']:
            if time.monotonic()>deadline:raise TimeoutError('finish committed recovery path')
    if current_key not in SUPPORTED_KEYS:
        e.transport.prepared.extend(recovery_exit)
        e.event('CONTROLLED_RETURN_BEFORE_DISABLE')
        e.execute_segments([r['original_plan'] for r in recovery_exit])
    e.s.pump(.5);summary['controlled_return_before_disable']=True


class HardwareVelocityMonitor(PathVelocityMonitor):
    """Compatibility name; hardware-time handling now applies to all jobs."""



class TuningExecutor(GravityExecutor):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.s.velocity_monitor=HardwareVelocityMonitor(self.p,self.report)

    def pump(self,seconds=.002):
        self.last_snapshot=time.monotonic()
        try:status=super().pump(seconds)
        except RuntimeError as exc:
            initial=self.report.get('stream_initial_counters',{})
            # An observation FIFO overrun is not an actuator fault. Keep the
            # independent PL stream holding/running and invalidate this sample
            # set; never deliberately drop torque for a logging-only loss.
            if str(exc)!='RX overflow or expired target' or self.d.read(0x424)!=initial.get('expired') or self.d.read(0x418) or self.transport.status()['fault']:
                raise
            count=self.d.read(0x42c);delta=count-initial['rx_overflow'];initial['rx_overflow']=count
            self.report.setdefault('observation_gaps',[]).append(dict(time=time.monotonic(),rx_lost=delta))
            status=self.transport.status()
        if self.started and time.monotonic()-getattr(self,'last_phase_capture',0)>.015:
            self.last_phase_capture=time.monotonic()
            # A crossed segment boundary is excluded; the modal epoch estimator
            # also rejects rare snapshots during the HLS pipeline transition.
            before=self.d.read(0xb4c);tick=self.d.read(0xb50);due=self.d.read(0xb58);holding=self.d.read(0xb5c)
            if self.d.read(0xb4c)==before:
                row=dict(time=time.monotonic(),completed=before,tick=tick,next_due_us=due,holding=bool(holding))
                if self.d.read(0x470) in (0x60002,0x60003):
                    row.update(damping_correction_codes=self.d.read(0x6c0),damping_applied_count=self.d.read(0x6c4),damping_clipped_count=self.d.read(0x6c8),damping_stale_count=self.d.read(0x6cc),damping_gain_q8=self.d.read(0x6d0))
                self.report.setdefault('pl_samples',[]).append(row)
        self.diagnostics()
        return status

    def diagnostics(self):
        if not self.report.get('collect_drive_telemetry') or not self.s.active:return
        now=time.monotonic()
        if now-getattr(self,'last_drive_sample',0)<.2:return
        self.last_drive_sample=now
        mid=2+getattr(self,'drive_sample_index',0)%2
        self.drive_sample_index=getattr(self,'drive_sample_index',0)+1
        try:
            iq=self.s.read(mid,0x701a)
            row=dict(motor=mid,time=time.monotonic(),iqf_a=iq,feedback=dict(self.s.feedback.get(mid,{})))
            if mid==3:row['vbus_v']=self.s.read(mid,0x701c)
            self.report.setdefault('drive_telemetry',[]).append(row)
        except TimeoutError as exc:
            # An optional readout timeout does not remove torque or stop a path.
            self.report.setdefault('drive_telemetry_errors',[]).append(dict(time=time.monotonic(),motor=mid,error=repr(exc)))


def launch_worker(profile,tracking,out,full_pick_only=False):
    ctx=multiprocessing.get_context('spawn');a,b=ctx.Pipe();proc=ctx.Process(target=worker_run,args=(b,str(profile),str(tracking),str(out),full_pick_only))
    proc.start();b.close()
    if not a.poll(180):proc.terminate();proc.join();raise TimeoutError('offline bank build')
    kind,info=a.recv()
    if kind!='ready':proc.join();raise RuntimeError(str(info))
    print(json.dumps(dict(event='OFFLINE_BANK_READY',**info)),flush=True)
    return a,proc


def execute(profile,tracking,out,trials=7,full_pick_only=False,candidate_kp=50.,candidate_kd=4.,replay_candidate=False,baseline_kp=80.,baseline_kd=4.,compare_friction=False,candidate_axis4_kp=80.,candidate_axis4_kd=4.,candidate_axis3_kp=None,compare_pl_damping=False,compare_method=False):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);p=JointPlanner(profile);m=GravityModel(MotionRuntime(),p,tracking)
    from reference_smoothing import settings
    method=settings(m.config)
    if bool(method)!=bool(compare_method):raise ValueError('configured full-pick method requires explicit dedicated comparison')
    if compare_method and (not full_pick_only or replay_candidate or compare_friction or compare_pl_damping or candidate_axis3_kp is not None or candidate_kp!=baseline_kp or candidate_kd!=baseline_kd or candidate_axis4_kp!=m.kp[3] or candidate_axis4_kd!=m.kd[3]):
        raise ValueError('method comparison requires dedicated ABBA and identical gains')
    if m.config.get('joint3_frequency_probe') and (not full_pick_only or not replay_candidate or compare_friction):
        raise ValueError('torque response diagnostic requires a single explicit replay, no gain/friction sweep')
    if compare_pl_damping and (not full_pick_only or not m.pl_damping_q8 or compare_friction or m.config.get('joint3_frequency_probe') or candidate_axis3_kp is not None or candidate_kp!=baseline_kp or candidate_kd!=baseline_kd or candidate_axis4_kp!=m.kp[3] or candidate_axis4_kd!=m.kd[3]):
        raise ValueError('PL damping comparison requires identical gains and a dedicated full-pick bank')
    if full_pick_only and not (0<candidate_kp<=min(m.guard_kp[1:3]) and 0<=candidate_kd<=min(m.guard_kd[1:3])):
        raise ValueError('full-pick candidate outside prepared shoulder/elbow gain bounds')
    if full_pick_only and not (0<baseline_kp<=min(m.guard_kp[1:3]) and 0<=baseline_kd<=min(m.guard_kd[1:3])):
        raise ValueError('full-pick baseline outside prepared shoulder/elbow gain bounds')
    if full_pick_only and not (0<candidate_axis4_kp<=m.guard_kp[3] and 0<=candidate_axis4_kd<=m.guard_kd[3]):
        raise ValueError('full-pick axis4 candidate outside prepared gain bounds')
    if candidate_axis3_kp is not None and (not full_pick_only or compare_friction or not 0<candidate_axis3_kp<=m.guard_kp[2]):
        raise ValueError('axis3 Kp override requires a bounded full-pick gain experiment')
    if compare_friction and (not full_pick_only or not m.has_friction or candidate_kp!=baseline_kp or candidate_kd!=baseline_kd or candidate_axis4_kp!=m.kp[3] or candidate_axis4_kd!=m.kd[3]):
        raise ValueError('friction comparison requires full picks, nonzero configured friction and identical A/B gains')
    report=dict(status='RUNNING',commands=[],frames=[],feedback=[],readbacks={},reset_confirmed=[],stop_errors={},flash_writes=False)
    evidence_gate=[not full_pick_only]
    if full_pick_only:
        report.update(frames=TrialEvidence([False]),feedback=TrialEvidence(evidence_gate),
                      pl_samples=TrialEvidence(evidence_gate),raw_can_frames_retained=False,
                      collect_drive_telemetry=True,drive_telemetry=[],
                      evidence_scope='Decoded feedback and PL phase only during trial; full command/configuration records retained')
    summary=dict(status='RUNNING',started_monotonic=time.monotonic(),profile_id=p.profile_id,model_id=m.r.model_id,initial_config=m.config,experiments=[],winners=[],final_pick_trials=[])
    pending_analyses=[]
    lock=open('/tmp/kv260_gripper_commission.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    # Reusing an output directory must not reuse yesterday's operator release.
    (out/'SUPPORTED_DISABLE').unlink(missing_ok=True)
    conn,proc=launch_worker(profile,tracking,out,full_pick_only);d=None;e=None;gc_before=gc.isenabled();recovery_exit=[];at_support=True;current_key=None
    def checkpoint():
        # Large formatting/filesystem work is forbidden in the live CAN owner.
        # The worker journals each completed experiment during the session.
        if e is None or not e.s.active or report.get('reset_confirmed'):
            (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    def wait():
        deadline=time.monotonic()+240
        while not conn.poll():
            if e is not None and not report.get('reset_confirmed'):
                if 'stream_initial_counters' in report:e.pump(.002)
                else:e.s.pump(.002)
            else:time.sleep(.001 if report.get('reset_confirmed') else .02)
            if time.monotonic()>deadline or not proc.is_alive():raise RuntimeError('numerical worker timeout/exit')
        kind,payload=conn.recv()
        if kind=='error':raise RuntimeError(str(payload))
        return kind,payload
    def request(command,payload):conn.send((command,payload));return wait()[1]
    def records(key,kp,kd):
        conn.send(('records',dict(key=key,kp=kp,kd=kd)));kind,count=wait()
        return receive_records(conn,kind,count,e.pump)
    def experiment(label,key,kp,kd,axis=0,retry=0,cached=None):
        nonlocal at_support,current_key
        recs=records(key,kp,kd) if cached is None else cached
        start=e.submitted;e.transport.prepared.extend(recs)
        gap_start=len(report.get('observation_gaps',[]));current_key=key;at_support=False
        evidence_gate[0]=True
        if full_pick_only:e.pump(.01)
        begin=time.monotonic();e.event('EXPERIMENT',label=label,kp=kp,kd=kd)
        e.execute_segments([r['original_plan'] for r in recs])
        at_support=key in SUPPORTED_KEYS
        e.pump(.08)
        if full_pick_only:evidence_gate[0]=False
        header=dict(label=label,axis=axis,kp=kp,kd=kd,key=key,start_completed=start,begin=begin,end=time.monotonic(),
            controller=d.status(),profile_id=p.profile_id,model_id=m.r.model_id,
            raw_can_frames_retained=report.get('raw_can_frames_retained',True),
            evidence_scope=report.get('evidence_scope','Full session evidence between experiment uploads'))
        lost=sum(x['rx_lost'] for x in report.get('observation_gaps',[])[gap_start:])
        if full_pick_only:
            # Four fixed trials fit a bounded in-memory bank. Move ownership of
            # these lists without formatting/IPC, then continue the next path.
            # Numerical analysis and disk compression happen after supported
            # return + verified reset, not while an extended arm carries load.
            bundle={}
            for name in ('commands','frames','feedback','pl_samples','segment_submissions','events','drive_telemetry'):
                evidence=report.get(name,[]);bundle[name]=evidence
                report[name]=TrialEvidence(evidence.gate) if isinstance(evidence,TrialEvidence) else []
            result=dict(label=label,key=key,kp=kp[:],kd=kd[:],axis=axis+1,measurement_pending=True)
            pending_analyses.append((bundle,header,dict(rx_lost=lost),result))
            summary['experiments'].append(result)
            emit(dict(event='TRIAL_CAPTURED_ANALYSIS_AFTER_DISABLE',label=label))
            return result
        metrics=upload_evidence(conn,report,header,dict(rx_lost=lost),wait,e.pump)
        result=dict(metrics,label=label,key=key,kp=kp[:],kd=kd[:],axis=axis+1)
        summary['experiments'].append(result)
        emit(dict(event='TRIAL_RESULT',label=label,score=metrics['score'],feasible=metrics['feasible'],
            position_rmse_deg=[round(x['position_rmse_rad']*57.29578,4) for x in metrics.get('joints',[])]))
        checkpoint()
        if metrics.get('measurement_invalid') and retry<2 and key not in ('entry','exit'):
            return experiment(label+'_retry'+str(retry+1),key,kp,kd,axis,retry+1)
        return result
    try:
        d=CanDevice();e=TuningExecutor(p,d,report,[],m);gc.collect();gc.disable()
        bounds=guard_job(p,[]);bounds['start_policy']=POLICY
        def prebuild(current):
            summary['start_motor_rad']=current
            request('entry',current)
            summary['bank_prepared_while_disabled']=True
        e.prepare(bounds,current_tracking=True,**(dict(before_enable=prebuild) if full_pick_only else {}))
        report['stream_initial_counters']=dict(rx_overflow=d.read(0x42c),expired=d.read(0x424))
        if not full_pick_only:
            current=[e.s.read(mid,0x7019) for mid in range(1,8)]
            summary['start_motor_rad']=current
            request('entry',current)
        kp=[80.]*6;kd=[4.]*6
        if full_pick_only:
            kp=m.kp.tolist();kd=m.kd.tolist()
            kp[1:3]=[baseline_kp]*2;kd[1:3]=[baseline_kd]*2
            ck=kp.copy();cd=kd.copy()
            ck[1:3]=[candidate_kp]*2;cd[1:3]=[candidate_kd]*2
            if candidate_axis3_kp is not None:ck[2]=candidate_axis3_kp
            if compare_method and method['method']=='split_damping':cd[2]=method['fast_joint3_kd']
            ck[3]=candidate_axis4_kp;cd[3]=candidate_axis4_kd
            schedule=[('pick_A1_baseline',kp,kd),
                      ('pick_B1_candidate',ck,cd),
                      ('pick_B2_candidate',ck,cd),
                      ('pick_A2_baseline',kp,kd)]
            if replay_candidate:schedule=[('pick_R1_candidate_replay',schedule[1][1],schedule[1][2])]
            keys=['pick_uncompensated' if compare_friction and label.startswith('pick_A') else 'pick' for label,_,_ in schedule]
            if compare_pl_damping:keys=['pick_undamped' if label.startswith('pick_A') else 'pick' for label,_,_ in schedule]
            if compare_method:keys=['pick_control' if label.startswith('pick_A') else 'pick' for label,_,_ in schedule]
            recovery_exit=records(keys[0],schedule[0][1],schedule[0][2])
            e.release_host_hold()
            for index,(label,ck,cd) in enumerate(schedule):
                # First PL record must start immediately after host release.
                # A second numerical request here leaves neither owner sending
                # targets, even though the CAN/heartbeat loop is pumping.
                result=experiment(label,keys[index],ck,cd,cached=recovery_exit if index==0 else None)
                summary['final_pick_trials'].append(result)
            summary.update(status='PASS_FULL_PICK_REPLAY_DATA_COLLECTION' if replay_candidate else 'PASS_FULL_PICK_ABBA_DATA_COLLECTION',candidate_adopted=False,
                           evaluation_scope='Single candidate replay' if replay_candidate else 'Full pick ABBA; no axis probes, anchor excursion or gravity fitting. Band metrics precede parameter adoption.')
            if m.config.get('joint3_frequency_probe'):
                summary.update(status='PASS_TORQUE_RESPONSE_DIAGNOSTIC',evaluation_scope='Closed-loop joint3 torque probe and supported return; NOT a complete grasp trial')
            return summary
        recovery_exit=records('exit',kp,kd)
        entry=records('entry',kp,kd)
        e.release_host_hold()
        experiment('entry','entry',kp,kd,cached=entry)
        for axis in range(6):
            history=[];base=None
            for n in range(trials):
                k,damping=request('suggest',dict(history=history,axis=axis))
                ck,cd=kp[:],kd[:];ck[axis]=k;cd[axis]=damping
                result=experiment(f'axis{axis+1}_search{n+1}',str(axis),ck,cd,axis)
                history.append(dict(kp=k,kd=damping,score=result['score'],feasible=result['feasible']))
                if n==0:base=result
            best=min(history,key=lambda r:r['score']);ck,cd=kp[:],kd[:];ck[axis]=best['kp'];cd[axis]=best['kd']
            # Matched baseline / selected repeats at the same posture.
            repeat_base=experiment(f'axis{axis+1}_baseline_repeat',str(axis),kp,kd,axis)
            repeat_best=experiment(f'axis{axis+1}_best_repeat',str(axis),ck,cd,axis)
            baseline_score=(base['score']+repeat_base['score'])/2
            selected_score=(best['score']+repeat_best['score'])/2
            accept=repeat_best['feasible'] and selected_score<.95*baseline_score
            if accept:kp,kd=ck,cd
            summary['winners'].append(dict(axis=axis+1,accepted=accept,kp=kp[axis],kd=kd[axis],baseline_score=baseline_score,selected_score=selected_score))
            emit(dict(event='AXIS_SELECTED',**summary['winners'][-1]));checkpoint()
        # Return while enabled; the final assessment uses the full six-step
        # pick-return path, twice for each setting, in ABBA order.
        experiment('gravity_static_calibration','calibration',kp,kd)
        summary['gravity_fit']=request('calibrate',{})
        experiment('return_to_supported_start','exit',kp,kd)
        for label,ck,cd in [('pick_A1_baseline',[80.]*6,[4.]*6),('pick_B1_tuned',kp,kd),('pick_B2_tuned',kp,kd),('pick_A2_baseline',[80.]*6,[4.]*6)]:
            result=experiment(label,'pick',ck,cd)
            summary['final_pick_trials'].append(result);checkpoint()
        for label in ['pick_C1_gravity_fitted','pick_C2_gravity_fitted']:
            result=experiment(label,'pick_calibrated',kp,kd)
            summary['final_pick_trials'].append(result);checkpoint()
        groups={name:[r for r in summary['final_pick_trials'] if r['label'].startswith('pick_'+name)] for name in ['A','B','C']}
        means={name:sum(x.get('global_score',1e9) for x in group)/len(group) for name,group in groups.items()}
        choice='B' if means['B']<.9*means['A'] else 'A'
        if means['C']<.95*means[choice]:choice='C'
        summary['full_pick_selection']=dict(group_mean_scores=means,selected_group=choice,
            kp=kp if choice!='A' else [80.]*6,kd=kd if choice!='A' else [4.]*6,
            gravity_scale=summary['gravity_fit']['scale'] if choice=='C' else [1.]*6)
        summary.update(status='PASS_TUNING_SEARCH_AND_PICK_ABBA',selected_kp=kp,selected_kd=kd)
    except (Exception,KeyboardInterrupt) as exc:
        summary.update(status='FAIL',error=repr(exc))
        if d is not None:summary['failure_controller']=d.status()
        emit(dict(event='SESSION_FAILED',error=repr(exc)))
    finally:
        if e is not None and report.get('owned'):
            # Software/measurement failures get a controlled return, using
            # cached records independent of the numerical worker. Hardware or
            # motor faults cannot be recovered by assuming control still works.
            if report.get('enable_attempted') and not at_support:
                try:
                    return_to_support(e,recovery_exit,current_key,summary)
                except (Exception,KeyboardInterrupt) as exc:
                    summary['controlled_return_error']=repr(exc)
                    retain_until_supported(e,out,summary)
            if report.get('enable_attempted'):
                try:e.s.stop_all()
                except Exception as exc:report['stop_errors']['fatal']=repr(exc)
            else:
                d.stop();d.write(0x5a0,0)
                for mid in range(1,8):d.write(d.pp_base(mid),0)
            reset_ok=sorted(report['reset_confirmed'])==list(range(1,8)) and not report['stop_errors']
            if report.get('volatile_configuration_changed') and reset_ok:
                try:e.restore_csp_configuration()
                except Exception as exc:summary.update(status='FAIL_RESTORE',restore_error=repr(exc))
            summary.update(reset_confirmed=report['reset_confirmed'],stop_errors=report['stop_errors'],csp_restored=report.get('restored_original_csp_and_torque_limits'),final_controller=d.status(),final_motion_mask=d.read(0x5a0))
            if report.get('enable_attempted') and not reset_ok:summary['status']='FAIL_STOP'
        summary['motion_and_stop_finished_monotonic']=time.monotonic()
        if pending_analyses and sorted(report.get('reset_confirmed',[]))==list(range(1,8)) and not report.get('stop_errors'):
            try:
                for bundle,header,quality,result in pending_analyses:
                    metrics=upload_evidence(conn,bundle,header,quality,wait,lambda seconds:time.sleep(.0001))
                    result.update(metrics);result['measurement_pending']=False
                    emit(dict(event='OFFLINE_TRIAL_RESULT',label=result['label'],feasible=metrics.get('feasible')))
                summary['analysis_after_supported_disable']=True
            except (Exception,KeyboardInterrupt) as exc:
                summary.update(status='FAIL_OFFLINE_ANALYSIS_MOTORS_ALREADY_DISABLED',analysis_error=repr(exc))
        summary['finished_monotonic']=time.monotonic();checkpoint()
        # Remaining stop/restore trace is written only after motors are disabled.
        (out/'session_tail.json').write_text(json.dumps(report,indent=2)+'\n')
        if d is not None:d.close()
        if proc.is_alive():
            try:conn.send(('quit',None));proc.join(timeout=2)
            except Exception:pass
            if proc.is_alive():proc.terminate();proc.join()
        conn.close();lock.close()
        if gc_before:gc.enable()
    return summary


if __name__=='__main__':
    a=argparse.ArgumentParser(description=__doc__)
    for name in ('profile','tracking-profile','output-dir'):a.add_argument('--'+name,required=True)
    a.add_argument('--trials-per-axis',type=int,default=7);a.add_argument('--execute-attended',action='store_true')
    a.add_argument('--full-pick-only',action='store_true',help='Matched complete picks: 80/4 baseline vs specified shoulder/elbow gains, ABBA; no axis search')
    a.add_argument('--candidate-kp',type=float,default=50.)
    a.add_argument('--candidate-kd',type=float,default=4.)
    a.add_argument('--baseline-kp',type=float,default=80.,help='Shoulder/elbow baseline Kp for matched full-pick comparison')
    a.add_argument('--baseline-kd',type=float,default=4.,help='Shoulder/elbow baseline Kd for matched full-pick comparison')
    a.add_argument('--compare-friction',action='store_true',help='Matched full-pick ABBA: A friction off, B configured partial friction; identical gains required')
    a.add_argument('--compare-pl-damping',action='store_true',help='Matched full-pick ABBA: A bypass, B filtered PL joint3 velocity-error damping; identical references/gains')
    a.add_argument('--compare-method',action='store_true',help='Dedicated full-pick ABBA: A original, B configured reference smoothing or partial inverse dynamics')
    a.add_argument('--candidate-axis4-kp',type=float,default=80.)
    a.add_argument('--candidate-axis4-kd',type=float,default=4.)
    a.add_argument('--candidate-axis3-kp',type=float,help='Override only joint3 Kp in a full-pick candidate; all other fields remain unchanged')
    a.add_argument('--replay-candidate',action='store_true',help='One complete candidate pick for operator observation; requires full-pick-only')
    x=a.parse_args()
    if not x.execute_attended:a.error('physical tuning requires explicit attended execution flag')
    if x.replay_candidate and not x.full_pick_only:a.error('candidate replay requires full-pick-only')
    if not 4<=x.trials_per_axis<=10:a.error('4..10 bounded search trials per axis')
    def interrupt(*args):raise KeyboardInterrupt('operator interrupt')
    signal.signal(signal.SIGTERM,interrupt)
    result=execute(x.profile,x.tracking_profile,x.output_dir,x.trials_per_axis,x.full_pick_only,x.candidate_kp,x.candidate_kd,x.replay_candidate,x.baseline_kp,x.baseline_kd,x.compare_friction,x.candidate_axis4_kp,x.candidate_axis4_kd,x.candidate_axis3_kp,x.compare_pl_damping,x.compare_method)
    raise SystemExit(not result['status'].startswith('PASS'))
