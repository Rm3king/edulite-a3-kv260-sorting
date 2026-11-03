#!/usr/bin/python3
"""Fixed-speed, hardware-time encoder metrics; no hardware access.

Reference epochs use the modal (PL next_due - tick*5000 - prior samples*5000),
excluding endpoint holds. This removes Linux receive-time jitter. No fitted lag
is subtracted from tracking errors. Encoder metrics are not external vibration.
"""
from collections import Counter
import numpy as np


def elapsed(value,base):return ((int(value)-int(base)+(1<<31))%(1<<32))-(1<<31)


def band_rms(x,dt,lo,hi):
    x=np.asarray(x,float);w=np.hanning(len(x));f=np.fft.rfftfreq(len(x),dt)
    power=np.abs(np.fft.rfft((x-x.mean())*w))**2;power[1:-1]*=2
    return float(np.sqrt(power[(f>=lo)&(f<=hi)].sum()/len(x)**2/max(np.mean(w*w),1e-12)))


def smooth(x,dt=.005,hz=10):
    n=min(100,len(x)//3);p=np.pad(x,(n,n),mode='reflect');f=np.fft.rfftfreq(len(p),dt)
    return np.fft.irfft(np.fft.rfft(p)*np.exp(-.5*(f/hz)**8),len(p))[n:-n]


def reference(records):
    q=[];v=[];cmd=[];tau=[];tick_offsets=[];offset=0
    for r in records:
        p=r['plan'];n=p['ticks'];u=np.arange(n+1)/n;w=r['words'];tick_offsets.append(offset)
        q.append(np.column_stack([np.polynomial.polynomial.polyval(u,c) for c in p['motor_coefficients_rad']]))
        v.append(np.column_stack([np.polynomial.polynomial.polyval(u,np.polynomial.polynomial.polyder(c))/p['duration_s'] for c in p['motor_coefficients_rad']]))
        encoded=[];torques=[]
        for i in range(6):
            value=np.polynomial.polynomial.polyval(u,np.array(w[i*8:i*8+8])/16777216)
            encoded.append(np.floor(value*65535+.5)*25.14/65535-12.57)
            value=np.polynomial.polynomial.polyval(u,np.array(w[112+i*8:120+i*8])/16777216)
            torques.append((np.floor(value*65535+.5)/65535-.5)*2*(14 if i<3 else 6))
        cmd.append(np.column_stack(encoded));tau.append(np.column_stack(torques));offset+=n+1
    return np.vstack(q),np.vstack(v),np.vstack(cmd),np.vstack(tau),np.array(tick_offsets)


def evaluate(feedback,snapshots,records,start_completed,axis,caps):
    if not feedback or not snapshots:raise ValueError('missing trial evidence')
    base=feedback[0]['pl_rx_us'];q,v,cmd,tau,offsets=reference(records);origins=[]
    for s in snapshots:
        i=s['completed']-start_completed
        if 0<=i<len(records) and not s['holding']:
            origins.append(elapsed(s['next_due_us'],base)-s['tick']*5000-int(offsets[i])*5000)
    if len(origins)<5:raise ValueError('insufficient hardware phase observations')
    origin,hits=Counter(origins).most_common(1)[0]
    if hits/len(origins)<.6:raise ValueError('ambiguous PL phase epoch')
    times=origin*1e-6+np.arange(len(q))*.005
    # Do not evaluate a partial execution as a valid trial.
    start,end=times[0]+.1,times[-1]-.05;t=np.arange(start,end,.005)
    if len(t)<400:raise ValueError('trial too short for metrics')
    rows=[]
    for i in range(6):
        fs=[f for f in feedback if f['motor']==i+1 and f['mode']==2 and not f['fault']]
        ts=np.array([elapsed(f['pl_rx_us'],base)*1e-6 for f in fs]);actual=np.array([f['position'] for f in fs])
        if len(ts)<len(t)*.8 or ts[0]>start or ts[-1]<end:raise ValueError('incomplete feedback coverage')
        gaps=np.diff(ts[(ts>=start-.02)&(ts<=end+.02)])
        if np.any(gaps<=0) or max(gaps)>.025:raise ValueError('feedback gap/order')
        qa=np.interp(t,ts,actual);va=np.interp(t,ts,[f['velocity'] for f in fs]);ta=np.interp(t,ts,[f['torque'] for f in fs])
        qd=np.interp(t,times,q[:,i]);vd=np.interp(t,times,v[:,i]);qc=np.interp(t,times,cmd[:,i]);e=qa-qd;ve=va-vd
        vf=np.gradient(smooth(qa),.005);dv=vf-vd
        tail=t>times[-1]-.65
        row=dict(motor=i+1,position_rmse_rad=float(np.sqrt(np.mean(e*e))),position_p95_rad=float(np.quantile(np.abs(e),.95)),position_max_rad=float(max(abs(e))),
            command_position_rmse_rad=float(np.sqrt(np.mean((qa-qc)**2))),velocity_rmse_rad_s=float(np.sqrt(np.mean(ve*ve))),
            encoder_velocity_rmse_rad_s=float(np.sqrt(np.mean(dv*dv))),error_0_5_4hz_rms_rad=band_rms(e,.005,.5,4),
            velocity_ripple_4_15hz_rad_s=band_rms(dv,.005,4,15),endpoint_error_rad=float(np.mean(e[tail])),
            torque_rms_nm=float(np.sqrt(np.mean(ta*ta))),torque_peak_nm=float(max(abs(ta))),
            saturation_fraction=float(np.mean(np.abs(ta)>.95*caps[i])),temperature_peak_c=float(max(f['temperature'] for f in fs)),
            maximum_feedback_gap_s=float(max(gaps)))
        rows.append(row)
    # Fixed scales stated before search; position accuracy and dynamic error
    # dominate, so a visibly smooth but inaccurate/slow response cannot win.
    scores=[]
    for i,r in enumerate(rows):
        collateral=max(x['position_rmse_rad'] for x in rows if x is not r)
        scores.append(.40*(r['position_rmse_rad']/.005)**2+.15*(r['velocity_rmse_rad_s']/.04)**2+.20*(r['velocity_ripple_4_15hz_rad_s']/.02)**2+.15*(r['endpoint_error_rad']/.005)**2+.05*(r['torque_rms_nm']/caps[i])**2+.05*(collateral/.01)**2)
    score=scores[axis]
    # Temperature and saturation are observations, not extra drive alarms or
    # reasons to cancel an experiment. Actual motor faults remain in hardware.
    feasible=max(x['position_max_rad'] for x in rows)<.06
    return dict(score=float(score) if feasible else 1e6+float(score),global_score=float(np.mean(scores)) if feasible else 1e6+float(np.mean(scores)),feasible=feasible,axis=axis+1,joints=rows,
        epoch_modal_fraction=hits/len(origins),epoch_samples=len(origins),duration_s=float(times[-1]-times[0]),
        reference_time='PL next_due and phase; no optimized lag removal',external_vibration_measurement=False)
