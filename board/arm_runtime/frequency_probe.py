"""Offline, bounded additive joint3 torque excitation; no hardware access.

This measures the closed-loop response about one already used pick posture.
It does not identify a free mechanical mode or override actuator protection.
"""
import math
import numpy as np
from numpy.polynomial import polynomial as poly
from gravity_trajectory import hold_segment, prepare_stream, validate_records
from septic_trajectory import S7, movej_c3, quantize_boundary, Q24


def probe_value(t, frequency, amplitude=.12, cycles=6, ramp=.4):
    duration=cycles/frequency+2*ramp
    if not 0<t<duration:return 0.
    u=min(t/ramp,(duration-t)/ramp,1.)
    return amplitude*poly.polyval(u,S7)*math.sin(2*math.pi*frequency*t)


def build_bank(model,capture,p,r):
    from tuning_worker import pick_tasks, task, guard_job
    from live_motion import current_reference
    cfg=model.config['joint3_frequency_probe']
    frequencies=cfg['frequencies_hz'];amplitude=cfg['amplitude_nm']
    if (not frequencies or any(type(f) not in (int,float) or not 1<=f<=8 for f in frequencies)
            or type(amplitude) not in (int,float) or not 0<amplitude<=.12):
        raise ValueError('probe requires 1..8 Hz and at most 0.12 Nm')
    original=pick_tasks(capture,p,r)
    approach=original[0];q=approach['goal_rad'];tasks=[approach]
    for f in frequencies:
        ticks=math.ceil((6/f+.8)/p.tick)
        # 50ms pieces also preserve polynomial fitting accuracy at 8Hz.
        sizes=[]
        while ticks:
            n=min(10,ticks)
            if 0<ticks-n<3:n-=3
            sizes.append(n);ticks-=n
        plans=[hold_segment(p,q,n*p.tick) for n in sizes]
        tasks.append(task('TorqueProbe_'+str(f)+'Hz',plans,q))
    motor=[x['decoded']['position_motor_rad'] for x in capture['transactions']]
    start,_=current_reference(p,motor)
    tasks.append(task('Return_to_support',movej_c3(p,q,start),start))
    tasks.append(task('Supported_hold',[hold_segment(p,start,1.)],start))
    records=prepare_stream(model,tasks)
    nodes=(1-np.cos(np.linspace(0,np.pi,8)))/2
    offsets={};maximum_error=0.
    for rec in records:
        name=rec['task']
        if not name.startswith('TorqueProbe_'):continue
        f=float(name[len('TorqueProbe_'):-2]);offset=offsets.get(name,0.)
        original=rec['original_plan'];plan=rec['plan']
        first=offset+original['duration_s']-plan['duration_s']
        seconds=plan['duration_s'];w=list(rec['words'])
        # Existing feedforward here is constant gravity at a fixed desired q.
        gravity=model.gravity(q)[2]
        values=[(gravity+probe_value(first+u*seconds,f,amplitude))/28+.5 for u in nodes]
        coefficients=poly.polyfit(nodes,values,7)
        encoded=quantize_boundary(coefficients,-(1<<31),(1<<31)-1)
        for u in np.linspace(0,1,33):
            error=abs((poly.polyval(u,np.array(encoded)/Q24)-.5)*28-gravity-probe_value(first+u*seconds,f,amplitude))
            maximum_error=max(maximum_error,error)
        w[128:136]=encoded;rec['words']=tuple(w)
        offsets[name]=offset+original['duration_s']
    if maximum_error>8e-6:raise ValueError('torque probe quantization/fit error')
    validate_records(records,guard_job(p,tasks),p,model)
    return records,dict(kind='CLOSED_LOOP_JOINT3_TORQUE_RESPONSE',amplitude_nm=amplitude,
                        frequencies_hz=frequencies,probe_pose_rad=q,maximum_torque_fit_error_nm=maximum_error,
                        full_grasp=False,segments=len(records))
