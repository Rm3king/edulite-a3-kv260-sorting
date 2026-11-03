#!/usr/bin/python3
"""Attended gravity-only A/B execution; no free-drive or Flash writes.

The ARM precomputes all q/v/g polynomial segments before opening hardware.
The FPGA evaluates the three fields and transmits six Type1 + one CSP at 200 Hz.
All live paths are bounded transport, feedback processing, and telemetry only.
"""
import argparse,fcntl,gc,json,math,signal,struct,time
from pathlib import Path
from motion_executor import MoveJExecutor,PathVelocityMonitor,CanDevice,Session,parameter,job_digest
from a3_protocol import enable,disable
from csp_commission import bits
from joint_trajectory import JointPlanner
from motion_runtime import MotionRuntime
from gravity_trajectory import GravityModel,prepare_stream,validate_records,encoded_interval,measured_start_reference
from gravity_transport import GravityTransport
from control_continuity import finish_failed_job


def tracking_packet(mid,q,tau,kp,kd):
    """Same nearest q/v/tau rounding as PL; gains follow SDK truncation."""
    tm=14. if mid<=3 else 6.
    code=lambda x,lo,hi:math.floor((x-lo)*65535/(hi-lo)+.5)
    p,v,t=code(q,-12.57,12.57),32768,code(tau,-tm,tm)
    if not all(0<=x<=65535 for x in (p,v,t)):raise ValueError('MIT static reference outside protocol')
    return 0x1000000+(t<<8)+mid,struct.pack('>HHHH',p,v,math.floor(kp*65535/500),math.floor(kd*65535/5))


class GravitySession(Session):
    def __init__(self,dev,report):
        super().__init__(dev,report);self.target_owner='mit_host';self.static={}

    def send(self,packet):
        ident,data=packet;mid=ident&255
        if ident>>24==1:
            if not 1<=mid<=6 or len(data)!=8:raise ValueError('six-axis MIT packet required')
            self.dev.heartbeat();self.dev.enqueue(ident,data)
            self.report['commands'].append(dict(id=ident,data=data.hex(),time=time.monotonic()))
        else:super().send(packet)

    def pump(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            super().pump(min(.0015,max(0.,end-time.monotonic())))
            if not self.stopping and self.target_owner=='mit_host' and self.active and time.monotonic()-self.last_refresh>=.01:
                self.last_refresh=time.monotonic()
                for mid in sorted(self.active):
                    self.require(mid,2)
                    self.send(self.static[mid] if mid<=6 else parameter(mid,0x7016,self.targets[mid]))


class GravityExecutor(MoveJExecutor):
    def __init__(self,p,dev,report,prepared,model):
        super().__init__(p,dev,report,GravityTransport,prepared)
        self.s=GravitySession(dev,report);self.s.velocity_monitor=PathVelocityMonitor(p,report);self.model=model

    def pump(self,seconds=.003):
        status=super().pump(seconds)
        if self.started:
            r=self.transport.prepared[min(status['completed'],len(self.transport.prepared)-1)]
            u=min(1.,status['tick']/r['plan']['ticks']);now=time.monotonic()
            for i,row in enumerate(r['plan']['motor_coefficients_rad'][:6]):
                target=0.
                for c in reversed(row):target=target*u+c
                error=abs(self.s.require(i+1,2)['position']-target)
                if error>self.model.config['following_envelope_rad']:
                    self.following_since.setdefault(i+1,now)
                    if error>.15 or now-self.following_since[i+1]>.1:raise RuntimeError(f'Axis {i+1}: MIT following error {error}')
                else:self.following_since.pop(i+1,None)
        return status

    def capture_tracking_hold(self,motor):
        """Capture fresh legal q and g for positive-stiffness live entry."""
        from live_motion import current_reference
        reference,adjustment=current_reference(self.p,motor)
        tau=self.model.gravity(reference)
        if any(abs(t)>cap for t,cap in zip(tau,self.model.caps)):raise ValueError('current hold gravity exceeds torque cap')
        for mid,hold in enumerate(self.p.to_motor(reference),1):
            self.s.targets[mid]=hold
            if mid<=6:
                self.s.static[mid]=tracking_packet(mid,hold,tau[mid-1],self.model.kp[mid-1],self.model.kd[mid-1])
            self.s.send(self.s.static[mid] if mid<=6 else parameter(mid,0x7016,hold))
        return reference,adjustment

    def prepare(self,job,live_start=False,current_tracking=False,before_enable=None):
        if before_enable is not None and not current_tracking:raise ValueError('disabled bank preparation requires current tracking')
        if live_start and any(float(x)!=0 for x in self.model.kp):raise ValueError('live hand-guiding start requires zero position stiffness')
        if current_tracking and live_start:raise ValueError('tracking and hand-guiding are distinct entries')
        if current_tracking:
            from live_motion import POLICY
            if job.get('start_policy')!=POLICY:raise ValueError('current tracking requires live replan policy')
        current=live_start or current_tracking
        p,d,s,m=self.p,self.d,self.s,self.model
        if m.pl_damping_q8 and d.read(0x470) not in (0x60002,0x60003):raise RuntimeError('PL damping requires v6.2 before enabling')
        if d.read(0x470) not in (0x60000,0x60001,0x60002,0x60003) or d.read(0x404)!=0 or d.read(0x5a8) or d.read(0x800)&1 or d.read(0xb40)&1:
            raise RuntimeError('v6 stopped controller required')
        d.configure('normal');self.report['owned']=True
        for mid in range(1,8):s.send(disable(mid));s.pump(.02);s.require(mid,0)
        self.report['initial_reset_confirmed']=list(range(1,8))
        actual=[s.read(mid,0x7019) for mid in range(1,8)]
        if not current and max(abs(a-b) for a,b in zip(actual,job['start_motor_rad']))>.002:raise RuntimeError('posture changed since gravity job review')
        self.report['start_motor_rad']=actual
        self.report['original_motor_parameters']={str(mid):dict(mode=s.read(mid,0x7005,True),torque_limit_nm=s.read(mid,0x700b)) for mid in range(1,7)}
        if any(v['mode']!=5 for v in self.report['original_motor_parameters'].values()):raise RuntimeError('CSP baseline required before experimental MIT entry')
        # Initial gravity was calculated offline at the reviewed measured pose.
        if current_tracking:
            from live_motion import current_reference
            reference,_=current_reference(p,actual);initial=m.gravity(reference)
            if any(abs(t)>cap for t,cap in zip(initial,m.caps)):raise ValueError('current hold gravity exceeds torque cap')
        else:initial=self.model.measured_gravity(actual) if live_start else job['initial_gravity_motor_nm']
        d.stop();d.write(0x5a0,127);d.write(0x5a4,100000)
        for mid,(lo,hi) in enumerate(zip(job['motor_lower_rad'],job['motor_upper_rad']),1):
            i=mid-1;base=d.pp_base(mid);margin=p.profile.get('capture_measurement_margin_rad',.02) if current else 0.
            s.plans[mid]=dict(lower=lo-margin,upper=hi+margin)
            mode=2 if mid<=6 else 1
            values=[(0,1),(4,bits(lo)),(8,bits(hi)),(12,bits(p.csp_velocity[i])),
                (16,bits(float(m.caps[i]) if mid<=6 else p.acceleration[i])),(20,bits(p.current[i])),(28,mode)]
            for off,val in values:
                d.write(base+off,val)
                if d.read(base+off)!=val:raise RuntimeError('gravity bank readback mismatch')
            if mid<=6:
                words=encoded_interval(job,p,m,i)
                for k,w in enumerate(words):
                    a=0x600+i*16+k*4;d.write(a,w)
                    if d.read(a)!=w:raise RuntimeError('encoded MIT guard readback mismatch')
        d.configure('normal')
        # Complete configuration for every disabled joint before enabling any.
        # An enabled upstream joint can move a still-disabled downstream joint.
        for mid in range(1,8):
            i=mid-1;fresh=s.read(mid,0x7019);hold=fresh
            if not current and abs(fresh-actual[i])>.002:raise RuntimeError(f'Axis {mid}: disabled pose moved before preparation ({actual[i]} -> {fresh})')
            if mid<=6:
                self.report['volatile_configuration_changed']=True
                for idx,val,integer in [(0x7005,0,True),(0x700b,float(m.caps[i]),False),(0x7028,6000,True)]:s.write_verified(mid,idx,val,integer)
                if current_tracking:
                    reference,_=current_reference(p,actual[:i]+[fresh]+actual[i+1:]);hold=p.to_motor(reference)[i]
                elif live_start:hold=self.model.reference_motor(mid,fresh)
                else:
                    reference,_=measured_start_reference(p,actual[:i]+[fresh]+actual[i+1:])
                    hold=p.to_motor(reference)[i]
                s.static[mid]=tracking_packet(mid,hold,initial[i],m.kp[i],m.kd[i]);s.send(s.static[mid]);s.pump(.004)
            else:
                if current_tracking:
                    reference,_=current_reference(p,actual[:i]+[fresh]+actual[i+1:]);hold=p.to_motor(reference)[i]
                for idx,val,integer in [(0x7005,5,True),(0x7017,p.csp_velocity[i],False),(0x7018,p.current[i],False),(0x7028,6000,True),(0x7016,hold,False)]:s.write_verified(mid,idx,val,integer)
            if d.read(d.pp_base(mid)+24)!=63:raise RuntimeError('MIT/CSP preparation incomplete')
            self.report.setdefault('initial_hold_targets',{})[str(mid)]=hold;s.targets[mid]=hold
        # Type17 replies do not renew the Type2 feedback table. The first axes
        # can age while the remaining disabled axes are prepared. Rewriting the
        # already verified timeout elicits a fresh disabled Type2 without
        # clearing readiness or changing any setting; then refresh its target.
        for mid in range(1,8):
            s.send(parameter(mid,0x7028,6000,True));s.pump(.003);s.require(mid,0)
            s.send(s.static[mid] if mid<=6 else parameter(mid,0x7016,s.targets[mid]))
        s.pump(.008)
        final_capture=[]
        for mid in range(1,8):
            fresh=s.read(mid,0x7019);s.require(mid,0)
            if not current and abs(fresh-actual[mid-1])>.002:raise RuntimeError(f'Axis {mid}: disabled posture changed before grouped enable')
            final_capture.append(fresh)
        if before_enable is not None:
            # Expensive numerical banks are built with every motor disabled.
            # CAN service/heartbeats continue through the caller's worker wait.
            before_enable(final_capture)
            for mid in range(1,8):
                s.send(parameter(mid,0x7028,6000,True));s.pump(.003);s.require(mid,0)
            final_capture=[s.read(mid,0x7019) for mid in range(1,8)]
            self.report['disabled_bank_prepared']=True
            self.event('OFFLINE_BANK_BUILT_WHILE_DISABLED')
        if live_start:
            actual=final_capture;initial=self.model.measured_gravity(actual)
            self.report['live_enable_motor_rad']=actual
            for mid in range(1,8):
                hold=self.model.reference_motor(mid,actual[mid-1]);s.targets[mid]=hold
                if mid<=6:s.static[mid]=tracking_packet(mid,hold,initial[mid-1],m.kp[mid-1],m.kd[mid-1])
                s.send(s.static[mid] if mid<=6 else parameter(mid,0x7016,hold))
            s.pump(.004)
        elif current_tracking:
            actual=final_capture
            self.capture_tracking_hold(actual)
            self.report['live_enable_motor_rad']=actual
            s.pump(.004)
        self.enable_prepared_axes(actual,live_start=current)
        if d.read(0x5a8)!=127:raise RuntimeError('seven enabled axes required')
        s.pump(.1)
        if current_tracking:
            self.state='HOLDING';self.event('CURRENT_MIT_ENABLED_HOLDING',torque_limit_nm=m.caps.tolist())
            return
        s.target_owner='pl'
        deadline=time.monotonic()+.05
        while d.read(0x41c)&65535 or d.read(0x464):
            if time.monotonic()>deadline:raise RuntimeError('MIT preparation queue did not drain')
            s.pump(.001)
        self.report['stream_initial_counters']=dict(rx_overflow=d.read(0x42c),expired=d.read(0x424))
        self.state='HOLDING';self.event('MIT_ENABLED_COMPENSATED_HOLDING',torque_limit_nm=m.caps.tolist())

    def enable_prepared_axes(self,actual,live_start=False):
        d,s=self.d,self.s
        if sorted(s.static)!=list(range(1,7)) or sorted(s.targets)!=list(range(1,8)) or s.active:
            raise RuntimeError('all seven disabled targets must be prepared')
        if any(d.read(d.pp_base(mid)+24)!=63 for mid in range(1,8)):
            raise RuntimeError('all seven readiness guards required before first enable')
        self.report['enable_attempted']=True;stamps={};begin=time.monotonic()
        for mid in range(1,8):
            stamps[mid]=time.monotonic();s.send(enable(mid))
            s.send(s.static[mid] if mid<=6 else parameter(mid,0x7016,s.targets[mid]))
        self.report['enable_batch_submit_seconds']=time.monotonic()-begin
        if self.report['enable_batch_submit_seconds']>.02:raise RuntimeError('grouped enable submission exceeded 20 ms')
        # Drain earlier disabled replies before activating the host mode monitor.
        # Type3 and its target are adjacent in the actual FIFO for each joint.
        s.pump(.02)
        for mid in range(1,8):
            f=s.require(mid,2)
            if f['time']<stamps[mid] or (not live_start and abs(f['position']-actual[mid-1])>.015):
                raise RuntimeError(f'Axis {mid}: unexpected grouped enable displacement')
        s.active.update(range(1,8));s.pump(.1)
        for mid in range(1,8):
            actual_hold=s.read(mid,0x7019)
            self.report.setdefault('initial_hold_actual_motor_rad',{})[str(mid)]=actual_hold
            if not live_start and abs(actual_hold-actual[mid-1])>.025:raise RuntimeError(f'Axis {mid}: initial compensated hold drift')

    def restore_csp_configuration(self):
        """Only after verified reset; restore the exact pre-test torque caps.

        No Type1, setpoint or enable commands are sent during restoration.
        The temporary MIT guard admits only the saved cap while inactive.
        """
        d=self.d;s=Session(d,self.report);before=len(self.report['commands'])
        self.report['readbacks_before_restore']={k:dict(v) for k,v in self.report['readbacks'].items()}
        if d.read(0x404) or d.read(0x5a8):raise RuntimeError('restore requires stopped/disarmed controller')
        try:
            d.write(0x5a0,63)
            for mid in range(1,7):
                base=d.pp_base(mid);d.write(base,1);d.write(base+28,2)
                d.write(base+16,bits(self.report['original_motor_parameters'][str(mid)]['torque_limit_nm']))
            d.configure('normal')
            for mid in range(1,7):s.write_verified(mid,0x700b,self.report['original_motor_parameters'][str(mid)]['torque_limit_nm'])
            d.stop()
            for mid in range(1,7):
                base=d.pp_base(mid);d.write(base+28,1);d.write(base+16,bits(self.p.acceleration[mid-1]))
            d.configure('normal')
            for mid in range(1,7):s.write_verified(mid,0x7005,5,True)
            self.report['restored_original_csp_and_torque_limits']=True
            assert all(c['id']>>24 in (17,18) for c in self.report['commands'][before:])
        finally:
            d.stop();d.write(0x5a0,0)
            for mid in range(1,8):d.write(d.pp_base(mid),0)

    def diagnostics(self):
        now=time.monotonic()
        if now-self.last_diagnostic<.25:return
        self.last_diagnostic=now
        # LOC_REF is a CSP parameter, not the Type1 target. Never use it as a
        # following-error reference in this mode. Compare encoder to PL q offline.
        for mid in (2,3):
            self.report.setdefault('tracking_diagnostics',[]).append(dict(motor=mid,time=time.monotonic(),iqf=self.s.read(mid,0x701a)))

    def settle(self,goal,seconds=.25,tolerance=None):
        tolerance=self.model.config['endpoint_tolerance_rad'] if tolerance is None else tolerance
        return super().settle(goal,seconds,tolerance)


def verify(job,p,m):
    if job.get('schema')!='kv260.reviewed_gravity_job.v1' or job['profile_id']!=p.profile_id or job['model_id']!=m.r.model_id or job['gravity_profile_id']!=m.id:
        raise ValueError('gravity job/model/profile mismatch')
    if not job.get('hardware_start_verified'):raise ValueError('synthetic gravity start forbidden')
    reference,adjustment=measured_start_reference(p,job['start_motor_rad'])
    if max(abs(a-b) for a,b in zip(reference,job['start_rad']))>1e-9 or max(abs(a-b) for a,b in zip(adjustment,job['start_reference_adjustment_motor_rad']))>1e-9:raise ValueError('gravity measured start/reference mapping mismatch')
    if max(abs(a-b) for a,b in zip(m.gravity(job['start_rad']),job['initial_gravity_motor_nm']))>1e-9:raise ValueError('initial gravity mismatch')
    if not job['tasks'] or max(abs(a-b) for a,b in zip(job['tasks'][0]['segments'][0]['start_rad'],job['start_rad']))>2e-6:raise ValueError('gravity first target does not match measured start')
    if len(job['motor_lower_rad'])!=7 or len(job['motor_upper_rad'])!=7:raise ValueError('seven guard intervals required')
    for i,(lo,hi) in enumerate(zip(job['motor_lower_rad'],job['motor_upper_rad'])):
        a,b=sorted((p.lower[i]*p.direction[i]+p.offset[i],p.upper[i]*p.direction[i]+p.offset[i]))
        if not math.isfinite(lo) or not math.isfinite(hi) or not a<=lo<hi<=b:raise ValueError('reviewed guard exceeds joint limits')
    records=prepare_stream(m,job['tasks']);validate_records(records,job,p,m)
    return records


def execute_job(job,p,output):
    started=time.monotonic();m=GravityModel(MotionRuntime(),p);prepared=verify(job,p,m)
    report=dict(status='RUNNING',commands=[],frames=[],feedback=[],readbacks={},reset_confirmed=[],stop_errors={},flash_writes=False,
        job_sha256=job_digest(job),profile_id=p.profile_id,model_id=m.r.model_id,gravity_profile_id=m.id,
        prevalidation_seconds=time.monotonic()-started,precomputed_segments=len(prepared),gravity_config=m.config,
        encoded_records=[dict(task=r['task'],words=r['words'],plan=r['plan']) for r in prepared],started_monotonic=time.monotonic())
    lock=open('/tmp/kv260_gripper_commission.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    (Path(output).parent/'SUPPORTED_DISABLE').unlink(missing_ok=True)
    d=CanDevice();executor=GravityExecutor(p,d,report,prepared,m);gc_before=gc.isenabled();gc.collect();gc.disable()
    try:
        executor.prepare(job)
        for task in job['tasks']:
            executor.event('TASK',name=task['name'],gravity_alpha=task['gravity_alpha']);begin=time.monotonic()
            executor.execute_segments(task['segments']);endpoint=executor.settle(task['goal_rad'])
            report.setdefault('task_results',[]).append(dict(name=task['name'],gravity_alpha=task['gravity_alpha'],begin=begin,end=time.monotonic(),**endpoint))
        report['status']='PASS_GRAVITY_AB_JOB'
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='FAIL',error=repr(exc),failure_controller=d.status(),failure_trajectory=executor.transport.status())
    finally:
        if report.get('owned'):
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
        report['finished_monotonic']=time.monotonic();Path(output).write_text(json.dumps(report,indent=2)+'\n')
        d.close();lock.close()
        if gc_before:gc.enable()
    print(json.dumps({k:report.get(k) for k in ('status','error','reset_confirmed','stop_errors','host_targets_after_pl_start')},indent=2))
    return report


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--job',required=True);ap.add_argument('--profile',required=True);ap.add_argument('--output',required=True)
    ap.add_argument('--supported-attended',action='store_true');a=ap.parse_args()
    if not a.supported_attended:ap.error('explicit attended supported-arm confirmation required')
    def interrupt(*args):raise KeyboardInterrupt('operator interrupt')
    signal.signal(signal.SIGTERM,interrupt)
    result=execute_job(json.loads(Path(a.job).read_text()),JointPlanner(a.profile),a.output)
    raise SystemExit(result['status']!='PASS_GRAVITY_AB_JOB')
