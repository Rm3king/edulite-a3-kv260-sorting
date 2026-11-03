#!/usr/bin/python3
"""Offline frequency/command audit of paused A/B trials; never opens hardware."""
import argparse,gzip,hashlib,json,struct
from collections import Counter
from pathlib import Path
import numpy as np
from tuning_metrics import reference,elapsed

FS=200.
BANDS=[(.5,2),(2,4),(4,8),(8,15),(15,30),(30,60),(60,95)]


def spectrum(x,n=1024):
    """One-sided linear-detrended Hann Welch PSD (50% overlap)."""
    x=np.asarray(x,float)
    if x.ndim==1:x=x[:,None]
    n=min(n,len(x));w=np.hanning(n);r=np.linspace(-1,1,n)
    blocks=np.array([x[k:k+n] for k in range(0,len(x)-n+1,n//2)])
    blocks-=blocks.mean(axis=1,keepdims=True)
    blocks-=r[None,:,None]*(blocks*r[None,:,None]).sum(axis=1,keepdims=True)/sum(r*r)
    z=np.fft.rfft(blocks*w[None,:,None],axis=1)
    factor=np.ones(n//2+1);factor[1:-1]=2
    power=(abs(z)**2).mean(axis=0)*factor[:,None]/(FS*sum(w*w))
    return np.fft.rfftfreq(n,1/FS),power,z


def reconstruct(raw):
    recs=raw['encoded_records'];q,v,qcmd,tau,offsets=reference(recs)
    base=raw['feedback'][0]['pl_rx_us'];origins=[]
    for s in raw['pl_samples']:
        i=s['completed']-raw['start_completed']
        if 0<=i<len(recs) and not s['holding']:
            origins.append(elapsed(s['next_due_us'],base)-s['tick']*5000-int(offsets[i])*5000)
    origin,hits=Counter(origins).most_common(1)[0];t=np.arange(len(q))/FS
    measured=[];gaps=[]
    for mid in range(1,7):
        fs=[f for f in raw['feedback'] if f['motor']==mid and f['mode']==2 and not f['fault']]
        ts=np.array([elapsed(f['pl_rx_us'],base)*1e-6-origin*1e-6 for f in fs])
        select=(ts>=0)&(ts<=t[-1]);gaps.append(float(max(np.diff(ts[select]))))
        measured.append(np.column_stack([np.interp(t,ts,[f[key] for f in fs]) for key in ('position','velocity','torque')]))
    a=np.stack(measured,axis=1)
    vcmd=[];labels=[]
    for rec in recs:
        n=rec['plan']['ticks'];u=np.arange(n+1)/n
        vcmd.append(np.column_stack([(np.floor(np.polynomial.polynomial.polyval(u,np.array(rec['words'][56+i*8:64+i*8])/16777216)*65535+.5)/65535-.5)*2*(33 if i<3 else 50) for i in range(6)]))
        labels.extend([rec['task']]*(n+1))
    return dict(t=t,q=q[:,:6],v=v[:,:6],qcmd=qcmd,vcmd=np.vstack(vcmd),ff=tau,actual=a,
        epoch_fraction=hits/len(origins),max_hardware_gaps=gaps,labels=np.array(labels))


def audit_trial(raw):
    s=reconstruct(raw);a=s['actual'];verr=a[:,:,1]-s['v'];qerr=a[:,:,0]-s['q']
    f,pv,zv=spectrum(verr);_,pq,_=spectrum(qerr);_,pt,zt=spectrum(a[:,:,2]-s['ff'])
    cross=(zv*np.conj(zt)).mean(axis=0)
    coherence=abs(cross)**2/np.maximum((abs(zv)**2).mean(axis=0)*(abs(zt)**2).mean(axis=0),1e-30)
    df=f[1]-f[0];rows=[]
    for i in range(6):
        band={f'{lo:g}_{hi:g}':float(np.sqrt(pv[(f>=lo)&(f<hi),i].sum()*df)) for lo,hi in BANDS}
        qband={f'{lo:g}_{hi:g}':float(np.rad2deg(np.sqrt(pq[(f>=lo)&(f<hi),i].sum()*df))) for lo,hi in BANDS}
        tband={f'{lo:g}_{hi:g}':float(np.sqrt(pt[(f>=lo)&(f<hi),i].sum()*df)) for lo,hi in BANDS}
        mask=(f>=2)&(f<=30);k=np.flatnonzero(mask)[np.argmax(pv[mask,i])]
        rows.append(dict(motor=i+1,velocity_band_rms=band,dominant_velocity_error_hz=float(f[k]),
            position_error_band_rms_deg=qband,torque_residual_band_rms_nm=tband,
            velocity_torque_residual_coherence_at_peak=float(coherence[k,i]),
            motion_velocity_rmse=float(np.sqrt(np.mean(verr[np.max(abs(s['v']),axis=1)>.02,i]**2))),
            torque_residual_rms=float(np.sqrt(np.mean((a[:,i,2]-s['ff'][:,i])**2)))))
    by_task={}
    for task in dict.fromkeys(s['labels']):
        m=s['labels']==task
        by_task[str(task)]=dict(seconds=float(m.sum()/FS),velocity_rmse=np.sqrt(np.mean(verr[m]**2,axis=0)).tolist(),
            position_rmse_rad=np.sqrt(np.mean(qerr[m]**2,axis=0)).tolist())
    digest=hashlib.sha256()
    for r in raw['encoded_records']:
        for w in r['words'][:210]+r['words'][217:]:digest.update(struct.pack('<I',int(w)&0xffffffff))
    qstep=25.14/65535;kp=raw['kp'];kd=raw['kd']
    result=dict(label=raw['label'],duration_s=float(s['t'][-1]),kp=kp,kd=kd,
        fixed_fields_except_gains_sha256=digest.hexdigest(),hardware_epoch_fraction=s['epoch_fraction'],maximum_hardware_feedback_gap_s=s['max_hardware_gaps'],
        position_protocol_lsb_rad=qstep,position_lsb_times_kp_nm=[float(k*qstep) for k in kp],
        joints=rows,by_task=by_task,
        qualifications=['Velocity is motor telemetry, not external link/TCP motion.','Closed-loop torque/velocity coherence does not prove a plant resonance or causality.','Welch bands use raw Type2 speed; old score used a 10 Hz filtered encoder derivative.'])
    return result,dict(f=f,velocity_psd=pv,position_psd=pq,torque_psd=pt,coherence=coherence,**s)


def run(folder,out):
    folder=Path(folder);out=Path(out);out.mkdir(parents=True,exist_ok=True);results=[];arrays={}
    for prefix in ('pick_A1_','pick_B1_','pick_B2_','pick_A2_'):
        paths=list(folder.glob(prefix+'*.json.gz'))
        if len(paths)!=1:raise ValueError(f'Expected one full trial for {prefix}, got {len(paths)}')
        name=paths[0].name.removesuffix('.json.gz')
        raw=json.loads(gzip.decompress(paths[0].read_bytes()))
        report,data=audit_trial(raw);results.append(report);arrays[name]=data
    (out/'frequency_audit.json').write_text(json.dumps(results,indent=2)+'\n')
    comparison=[]
    for i in range(6):
        row=dict(motor=i+1)
        for metric in ('velocity_band_rms','position_error_band_rms_deg','torque_residual_band_rms_nm'):
            a=float(np.mean([results[k]['joints'][i][metric]['4_8'] for k in (0,3)]))
            b=float(np.mean([results[k]['joints'][i][metric]['4_8'] for k in (1,2)]))
            row[metric]=dict(A=a,B=b,increase_percent=100*(b/a-1) if a else None)
        comparison.append(row)
    matched=len({r['fixed_fields_except_gains_sha256'] for r in results})==1
    (out/'paired_summary.json').write_text(json.dumps(dict(fixed_reference_fields_identical=matched,
        band_hz=[4,8],replicates_per_group=2,joints=comparison),indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(3,2,figsize=(12,9),layout='constrained')
    for name,d in arrays.items():
        group=name.split('_')[1][0];color='#4c78a8' if group=='A' else '#e45756'
        for i,ax in enumerate(axs.flat):
            ax.semilogy(d['f'],d['velocity_psd'][:,i],color=color,alpha=.75,label=name.split('_')[1])
            ax.set(xlim=(.5,30),xlabel='Frequency (Hz)',ylabel='Speed-error PSD [(rad/s)^2/Hz]',title=f'Joint {i+1}')
            ax.grid(alpha=.2);ax.legend()
    fig.suptitle('Matched full-pick speed residuals: A=baseline, B=candidate gains\nRaw Type2 speed; Welch 5.12 s windows; not external vibration')
    fig.savefig(out/'speed_error_psd.png',dpi=170);plt.close(fig)
    fig,axs=plt.subplots(2,1,figsize=(13,7),layout='constrained')
    for name,d in arrays.items():
        color='#4c78a8' if '_A' in name else '#e45756';i=1
        axs[0].plot(d['t'],d['actual'][:,i,1]-d['v'][:,i],lw=.6,color=color,alpha=.7,label=name.split('_')[1])
        axs[1].plot(d['t'],d['actual'][:,i,2]-d['ff'][:,i],lw=.6,color=color,alpha=.7,label=name.split('_')[1])
    axs[0].set(ylabel='J2 speed error (rad/s)');axs[1].set(ylabel='J2 torque - FF (Nm)',xlabel='Reference time (s)')
    for ax in axs:ax.grid(alpha=.2);ax.legend()
    fig.savefig(out/'joint2_time_trace.png',dpi=170);plt.close(fig)
    return results


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder');p.add_argument('--output',required=True);a=p.parse_args()
    r=run(a.folder,a.output);print(json.dumps([dict(label=x['label'],joints=x['joints']) for x in r],indent=2))
