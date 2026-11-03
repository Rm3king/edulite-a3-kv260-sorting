"""Aggregate all retained valid normal/fault trials; never select by winner."""
import argparse,csv,json
from pathlib import Path
import numpy as np

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--directory',required=True);a=p.parse_args();root=Path(a.directory)
 all_trials=[dict(json.loads(f.read_text()),file=f.name) for f in sorted(root.glob('*.metrics.json'))]
 invalid=[dict(file=t['file'],status=t.get('status'),error=t.get('error')) for t in all_trials if 'timing' not in t]
 trials=[t for t in all_trials if 'timing' in t];rows=[]
 for path in ('blend','circle','pick'):
  for injected in (False,True):
   for backend in ('arm','pl'):
    group=[t for t in trials if t['path']==path and t['backend']==backend and bool(t.get('controlled_host_stall'))==injected]
    if not group:continue
    axes=[x for t in group for x in t['timing']['axes']];hist=np.sum([x['histogram'] for x in axes],axis=0);n=int(sum(hist));acc=0;p99=None
    for upper,count in zip(axes[0]['histogram_upper_bounds_us'],hist):
     acc+=int(count)
     if acc>=np.ceil(n*.99):p99=upper;break
    cpu=[t['cpu_one_core_percent'] for t in group]
    rows.append(dict(path=path,scenario='matched_20ms_host_blackout' if injected else 'normal',backend=backend,trials=len(group),
      cpu_one_core_percent_mean=float(np.mean(cpu)),cpu_one_core_percent_min=min(cpu),cpu_one_core_percent_max=max(cpu),
      motion_seconds_mean=float(np.mean([t['motion_wall_seconds'] for t in group])),
      max_abs_command_period_error_us=max(x['max_abs_period_error_us'] for x in axes),p99_abs_command_period_error_upper_bound_us=p99,
      longest_command_interval_us=max(x['max_interval_us'] for x in axes),period_intervals=n,
      intervals_abs_error_gt50_us=sum(x['intervals_abs_error_gt50_us'] for x in axes),period_gaps_gt1_5=sum(x['intervals_above_1_5_period'] for x in axes),
      expired_targets=sum(t['controller_counter_deltas']['expired'] for t in group),rx_overflows=sum(t['controller_counter_deltas']['rx_overflow'] for t in group),
      schedule_rebases=sum(len(t.get('schedule_rebases',[])) for t in group),
      staging_mmio_writes_mean=float(np.mean([t['motor_target_staging_mmio_writes'] for t in group])),
      joint2_rmse_deg=float(np.mean([t['joints'][1]['position_rmse_deg'] for t in group])),
      joint3_rmse_deg=float(np.mean([t['joints'][2]['position_rmse_deg'] for t in group])),
      joint3_nominal_clock_rmse_deg=float(np.mean([t['joints'][2]['position_rmse_vs_nominal_clock_deg'] for t in group])),
      temperature_peak_c=max(x['temperature_peak_c'] for t in group for x in t['joints']),files=[t['file'] for t in group]))
 result=dict(timestamp_scope='CAN_CORE_TRANSMIT_COMMAND_REQUEST_NOT_WIRE_SOF',p99_is_histogram_upper_bound=True,
   arm_baseline='Python scheduling/supervision with optimized native C++ evaluation, packing and MMIO submission',
   pl_backend='PL scheduled evaluation/packing with Python supervision and native bulk coefficient upload',
   dedicated_native_realtime_thread_measured=False,invalid_metrics=invalid,external_tcp_accuracy_verified=False,rows=rows)
 (root/'comparison_summary.json').write_text(json.dumps(result,indent=2)+'\n')
 if rows:
  with (root/'comparison_summary.csv').open('w') as f:
   writer=csv.DictWriter(f,fieldnames=[k for k in rows[0] if k!='files'],lineterminator='\n');writer.writeheader();writer.writerows({k:v for k,v in r.items() if k!='files'} for r in rows)
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 fig,axs=plt.subplots(1,3,figsize=(14,4));names=['blend','circle','pick'];x=np.arange(3)
 for j,backend in enumerate(('arm','pl')):
  group=[next((r for r in rows if r['path']==name and r['scenario']=='normal' and r['backend']==backend),None) for name in names]
  cpu=[r['cpu_one_core_percent_mean'] if r else np.nan for r in group];mmio=[r['staging_mmio_writes_mean'] if r else np.nan for r in group]
  axs[0].bar(x+(j-.5)*.32,cpu,.32,label=backend.upper());axs[1].bar(x+(j-.5)*.32,np.array(mmio)/1000,.32,label=backend.upper())
  r=next((r for r in rows if r['path']=='pick' and r['scenario']=='matched_20ms_host_blackout' and r['backend']==backend),None)
  if r:axs[2].bar(j,r['longest_command_interval_us']/1000,label=backend.upper())
 for ax in axs[:2]:ax.set_xticks(x,names);ax.legend();ax.grid(axis='y',alpha=.2)
 axs[0].set_ylabel('Control process CPU (% of one core)');axs[1].set_ylabel('Target staging MMIO writes (thousands; derived)')
 axs[2].set_xticks([0,1],['ARM','PL']);axs[2].set_ylabel('Longest command-request interval (ms)');axs[2].set_title('Matched 20ms host blackout');axs[2].axhline(5,ls='--',color='gray',lw=1)
 fig.suptitle('Same references and motor gains; shared PL CAN/FIFO; no external TCP measurement');fig.tight_layout();fig.savefig(root/'architecture_comparison.png',dpi=180)
 print(json.dumps(result,indent=2))
