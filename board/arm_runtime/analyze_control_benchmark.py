"""Offline motion evidence; encoder tracking is not external TCP metrology."""
import argparse,gzip,json,pickle
from pathlib import Path
from collections import Counter
import numpy as np
from tuning_metrics import reference,elapsed,band_rms

def analyze(report,bank):
 q,v,cmd,tau,offsets=reference(bank['records']);feedback=report['feedback'];base=feedback[0]['pl_rx_us']
 if report['backend']=='arm':
  origins=[elapsed(s['due_us'],base)-int(offsets[s['record']])*5000 for s in report['native_epochs']]
 else:
  origins=[elapsed(s['next_due_us'],base)-s['tick']*5000-int(offsets[s['completed']])*5000 for s in report['pl_samples']
    if 0<=s['completed']<len(offsets) and not s['holding']]
 origin,hits=Counter(origins).most_common(1)[0]
 if report['backend']=='arm' and report.get('native_due_us'):
  times=np.array([elapsed(x,base)*1e-6 for x in report['native_due_us']])
  if len(times)!=len(q):raise ValueError('native reference timestamps missing')
 else:
  if hits/len(origins)<.6:raise ValueError('ambiguous hardware epoch')
  times=origin*1e-6+np.arange(len(q))*.005
 t=np.arange(times[0]+.1,times[-1]-.05,.005);rows=[]
 for axis in range(6):
  fs=[f for f in feedback if f['motor']==axis+1 and f['mode']==2 and not f['fault']]
  ts=np.array([elapsed(f['pl_rx_us'],base)*1e-6 for f in fs]);actual=np.array([f['position'] for f in fs])
  gaps=np.diff(ts[(ts>=t[0]-.02)&(ts<=t[-1]+.02)])
  allowed_gap=.05 if report.get('controlled_host_stall') else .025
  if len(ts)<len(t)*.8 or ts[0]>t[0] or ts[-1]<t[-1] or not len(gaps) or min(gaps)<=0 or max(gaps)>allowed_gap:raise ValueError('feedback coverage/order/gap')
  measured=np.interp(t,ts,actual);error=measured-np.interp(t,times,q[:,axis]);ve=np.interp(t,ts,[f['velocity'] for f in fs])-np.interp(t,times,v[:,axis])
  nominal_times=times[0]+np.arange(len(q))*.005
  nominal_error=measured-np.interp(t,nominal_times,q[:,axis])
  rows.append(dict(motor=axis+1,position_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(error**2)))),
    position_max_abs_deg=float(np.rad2deg(max(abs(error)))),position_rmse_vs_nominal_clock_deg=float(np.rad2deg(np.sqrt(np.mean(nominal_error**2)))),velocity_rmse_rad_s=float(np.sqrt(np.mean(ve**2))),
    tracking_error_2_15hz_rms_deg=float(np.rad2deg(band_rms(error,.005,2,15))),
    temperature_peak_c=max(f['temperature'] for f in fs),maximum_feedback_gap_ms=float(max(gaps)*1000)))
 return dict(status=report['status'],backend=report['backend'],path=report['path'],reference_sha256=report['reference_sha256'],
  cpu_seconds=report['control_cpu_seconds'],motion_wall_seconds=report['motion_wall_seconds'],cpu_one_core_percent=100*report['control_cpu_seconds']/report['motion_wall_seconds'],
  timing=report['timing'],joints=rows,epoch_consistency_fraction=hits/len(origins),
  motor_target_staging_mmio_writes=(sum(r['words'][217]+1 for r in bank['records'])+15)*49 if report['backend']=='arm' else len(bank['records'])*221,
  staging_write_count_scope='Derived from actually submitted samples/records and fixed ABI; excludes heartbeats/configuration/reads',
  controller_counter_deltas={k:report['controller_after_path'][k]-report['controller_before_path'][k] for k in ('expired','rx_overflow','submitted','received')},
  tracking_reference='Issued reference clock, with nominal-clock RMSE separately retained; no fitted lag subtraction',
  controlled_host_stall=report.get('controlled_host_stall'),schedule_rebases=report.get('native_schedule_rebases',[]),
  uniform_time_law_compliant=not bool(report.get('native_schedule_rebases')),
  external_tcp_accuracy_verified=False,external_vibration_verified=False)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--report',required=True);p.add_argument('--bank',required=True);p.add_argument('--output',required=True);a=p.parse_args()
 with gzip.open(a.report,'rt') as f:r=json.load(f)
 try:out=analyze(r,pickle.loads(Path(a.bank).read_bytes()))
 except Exception as exc:out=dict(status='INVALID_METRICS_PRESERVE_RAW_TRIAL',error=repr(exc),backend=r.get('backend'),path=r.get('path'))
 Path(a.output).write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k not in ('timing','joints')},indent=2))
