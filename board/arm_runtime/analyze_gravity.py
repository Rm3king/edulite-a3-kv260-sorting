#!/usr/bin/python3
"""Encoder-based gravity A/B metrics. No external TCP precision is inferred."""
import argparse,json
from pathlib import Path
import numpy as np
from joint_trajectory import JointPlanner
from motion_runtime import MotionRuntime


def band_rms(values,dt=.005,lower_hz=4.,upper_hz=20.):
    """Hann-windowed band energy, excluding slow commanded motion.

    This is a feedback ripple diagnostic, not an accelerometer measurement or
    an external vibration measurement. Resampling and encoder quantization
    remain in the result.
    """
    x=np.asarray(values,float)
    if len(x)<400 or not np.isfinite(x).all():raise ValueError('insufficient finite ripple samples')
    window=np.hanning(len(x));spectrum=np.fft.rfft((x-x.mean())*window)
    frequency=np.fft.rfftfreq(len(x),dt);power=np.abs(spectrum)**2
    power[1:-1]*=2
    return float(np.sqrt(power[(frequency>=lower_hz)&(frequency<=upper_hz)].sum()/len(x)**2/np.mean(window**2)))


def analyze(report,job,p):
    result=dict(execution_status=report['status'],scope='ENCODER_AND_URDF_ONLY_NOT_EXTERNAL_TCP_METROLOGY',static={},circle={},
        host_targets_after_pl_start=report.get('host_targets_after_pl_start'),reset_confirmed=report.get('reset_confirmed'),stop_errors=report.get('stop_errors'))
    tasks={t['name']:t for t in job['tasks']};runtime=MotionRuntime()
    for tr in report.get('task_results',[]):
        task=tasks[tr['name']];start,end=tr['begin'],tr['end']
        if tr['name'].startswith('static_'):
            target=p.to_motor(task['goal_rad']);rows=[]
            for i in range(6):
                fs=[f for f in report['feedback'] if f['motor']==i+1 and start+1.<f['time']<end-.1 and f['mode']==2 and not f['fault']]
                if len(fs)<10:raise ValueError('insufficient settled static feedback')
                error=np.array([f['position']-target[i] for f in fs])
                rows.append(dict(motor=i+1,samples=len(fs),mean_error_rad=float(np.mean(error)),rms_error_rad=float(np.sqrt(np.mean(error**2))),
                    position_std_rad=float(np.std(error)),mean_torque_nm=float(np.mean([f['torque'] for f in fs]))))
            result['static'][tr['name']]=rows
        if 'circle' in tr['name'].lower() or task.get('kind')=='MoveC':
            # Uniformly interpolate asynchronous seven-motor feedback before FK.
            geom=task['geometry'];until=min(end,start+geom['duration_s']);ts=np.arange(start+.05,until-.05,.005)
            qs=[];ripple=[]
            for i in range(6):
                fs=[f for f in report['feedback'] if f['motor']==i+1 and start-.05<f['time']<until+.05 and f['mode']==2 and not f['fault']]
                if len(fs)<10:raise ValueError('insufficient circle feedback')
                if max(np.diff([f['time'] for f in fs]))>.025:raise ValueError('circle feedback gap')
                motor=np.interp(ts,[f['time'] for f in fs],[f['position'] for f in fs]);qs.append((motor-p.offset[i])/p.direction[i])
                velocity=np.interp(ts,[f['time'] for f in fs],[f['velocity'] for f in fs])
                # Exclude the first/last second, then suppress spectral leakage
                # from the slow 30-second circle with a Hann window.
                core=(ts>start+1.)&(ts<until-1.)
                ripple.append(dict(motor=i+1,feedback_samples=len(fs),maximum_feedback_gap_s=float(max(np.diff([f['time'] for f in fs]))),
                    position_band_rms_deg=float(np.rad2deg(band_rms(motor[core]))),velocity_band_rms_rad_s=band_rms(velocity[core])))
            xyz=np.array([runtime.fk(row).translation for row in np.array(qs).T]);a,b=geom['axes'];other=3-a-b
            center=np.array(geom['origin_xyz_m']);center[a]-=geom['radius_m'];rad=np.linalg.norm(xyz[:,[a,b]]-center[[a,b]],axis=1)
            distance=np.hypot(rad-geom['radius_m'],xyz[:,other]-center[other])
            result['circle'][tr['name']]=dict(samples=len(ts),rms_distance_to_nominal_circle_mm=float(np.sqrt(np.mean(distance**2))*1000),
                max_distance_to_nominal_circle_mm=float(max(distance)*1000),xyz_min_m=xyz.min(axis=0).tolist(),xyz_max_m=xyz.max(axis=0).tolist(),
                feedback_ripple=dict(band_hz=[4.,20.],resample_hz=200.,window='Hann, first/last 1 second excluded',joints=ripple,
                    scope='Encoder/estimated-velocity ripple including quantization and host receive timing; not external structural vibration'))
    off=result['static'].get('static_gravity_off');on=result['static'].get('static_gravity_on_after')
    if off and on:
        result['static_absolute_error_ratio_on_over_off']=[b['rms_error_rad']/max(a['rms_error_rad'],1e-12) for a,b in zip(off,on)]
    result['limits']='Single ordered A/B; gains/mode matched, but not randomized or long-run/thermal/load validation. Static required load torque need not decrease when feedforward is enabled.'
    return result


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--report',required=True);ap.add_argument('--job',required=True);ap.add_argument('--profile',required=True);ap.add_argument('--output',required=True);a=ap.parse_args()
    result=analyze(json.loads(Path(a.report).read_text()),json.loads(Path(a.job).read_text()),JointPlanner(a.profile))
    Path(a.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
