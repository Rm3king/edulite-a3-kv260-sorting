#!/usr/bin/python3
"""Offline summary of measured gain search and matched full-pick repeats."""
import argparse,gzip,json,math
from collections import Counter
from pathlib import Path
import numpy as np


def summarize(folder):
    folder=Path(folder);s=json.loads((folder/'summary.json').read_text())
    groups={g:[x for x in s.get('final_pick_trials',[]) if x['label'].startswith('pick_'+g)] for g in ('A','B','C')}
    stats={}
    for name,trials in groups.items():
        if not trials:continue
        if any(not x.get('feasible') or x.get('measurement_invalid') or len(x.get('joints',[]))!=6 for x in trials):
            stats[name]=dict(valid=False,repeats=len(trials));continue
        rows=[]
        for i in range(6):
            row=dict(motor=i+1)
            for field in ('position_rmse_rad','velocity_rmse_rad_s','velocity_ripple_4_15hz_rad_s','error_0_5_4hz_rms_rad','endpoint_error_rad','torque_rms_nm','temperature_peak_c'):
                values=[x['joints'][i][field] for x in trials]
                row[field]=dict(mean=float(np.mean(values)),min=min(values),max=max(values))
            row['position_mse_rad2_mean']=float(np.mean([x['joints'][i]['position_rmse_rad']**2 for x in trials]))
            rows.append(row)
        scores=[x['global_score'] for x in trials]
        stats[name]=dict(valid=True,repeats=len(trials),global_score_mean=float(np.mean(scores)),global_scores=scores,joints=rows)
    baseline=stats.get('A',{})
    if baseline.get('valid'):
        for g in ('B','C'):
            if stats.get(g,{}).get('valid'):
                stats[g]['score_improvement_percent']=100*(1-stats[g]['global_score_mean']/baseline['global_score_mean'])
    complete=all(stats.get(g,{}).get('valid') and stats[g]['repeats']==2 for g in ('A','B','C'))
    result=dict(status=s['status'] if complete else 'INCOMPLETE_VALID_FULL_PICK_COMPARISON',producer_status=s['status'],groups=stats,winners=s.get('winners',[]),selection=s.get('full_pick_selection'),
        scope='Matched encoder tracking at fixed reference speed; NOT external TCP vibration/accuracy.',
        statistical_limit='Two repeats per group; no statistical-significance or global-optimum claim.',
        original_csp_comparison='Previous CSP trial has different gripper timing and starting pose; no matched overall superiority claim.')
    return result


def plot(result,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fields=[('position_rmse_rad','Position RMSE (deg)',180/math.pi),
            ('velocity_rmse_rad_s','Velocity RMSE (rad/s)',1),
            ('velocity_ripple_4_15hz_rad_s','4–15 Hz velocity residual RMS (rad/s)',1),
            ('error_0_5_4hz_rms_rad','0.5–4 Hz position error RMS (deg)',180/math.pi)]
    fig,axs=plt.subplots(2,2,figsize=(12,7),layout='constrained')
    valid=[(g,d) for g,d in result['groups'].items() if d.get('valid')]
    colors=['#4c78a8','#f58518','#54a24b'];x=np.arange(1,7)
    for ax,(field,label,scale) in zip(axs.flat,fields):
        for n,(g,d) in enumerate(valid):
            vals=[r[field]['mean']*scale for r in d['joints']]
            lo=[(r[field]['mean']-r[field]['min'])*scale for r in d['joints']]
            hi=[(r[field]['max']-r[field]['mean'])*scale for r in d['joints']]
            ax.bar(x+(n-1)*.24,vals,width=.23,color=colors[n],label=g,yerr=[lo,hi],capsize=2)
        ax.set(xlabel='Joint',ylabel=label,xticks=x);ax.grid(axis='y',alpha=.2);ax.legend()
    fig.suptitle('Complete pick-return repeats: A baseline, B gains, C gains + gravity fit\nEncoder metrics; error bars show repeat range, not confidence intervals')
    fig.savefig(out,dpi=170);plt.close(fig)


def plot_trace(folder,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from tuning_metrics import reference,elapsed
    fig,axs=plt.subplots(3,2,figsize=(13,9),layout='constrained')
    for name,color in [('pick_A1_baseline','#4c78a8'),('pick_B1_tuned','#f58518'),('pick_C1_gravity_fitted','#54a24b')]:
        raw=json.loads(gzip.decompress((Path(folder)/(name+'.json.gz')).read_bytes()))
        q,_,_,_,offsets=reference(raw['encoded_records']);base=raw['feedback'][0]['pl_rx_us'];origins=[]
        for sample in raw['pl_samples']:
            i=sample['completed']-raw['start_completed']
            if 0<=i<len(offsets) and not sample['holding']:
                origins.append(elapsed(sample['next_due_us'],base)-sample['tick']*5000-int(offsets[i])*5000)
        origin=Counter(origins).most_common(1)[0][0]*1e-6
        qt=np.arange(len(q))*.005
        for i,ax in enumerate(axs.flat):
            fs=[f for f in raw['feedback'] if f['motor']==i+1 and f['mode']==2 and not f['fault']]
            t=np.array([elapsed(f['pl_rx_us'],base)*1e-6-origin for f in fs]);actual=np.array([f['position'] for f in fs])
            keep=(t>=0)&(t<=qt[-1]);t=t[keep]
            error=(actual[keep]-np.interp(t,qt,q[:,i]))*180/math.pi
            ax.plot(t,error,lw=.65,color=color,label=name.split('_')[1])
            ax.set(xlabel='Reference time (s)',ylabel='Position error (deg)',title=f'Joint {i+1}')
            ax.grid(alpha=.2);ax.legend()
    fig.suptitle('Complete pick-return: encoder tracking error, hardware timestamps, no fitted lag removal')
    fig.savefig(out,dpi=170);plt.close(fig)


if __name__=='__main__':
    a=argparse.ArgumentParser(description=__doc__);a.add_argument('folder');a.add_argument('--output',required=True);a.add_argument('--plot');a.add_argument('--trace-plot');x=a.parse_args()
    result=summarize(x.folder);Path(x.output).write_text(json.dumps(result,indent=2)+'\n')
    if x.plot:plot(result,x.plot)
    if x.trace_plot:plot_trace(x.folder,x.trace_plot)
    print(json.dumps(dict(status=result['status'],selection=result['selection']),indent=2))
