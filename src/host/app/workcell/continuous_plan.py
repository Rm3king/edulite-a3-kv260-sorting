"""Planning side of the continuous wrist correction (experimental): the job's correction map, PL bounds and tube audit.

The board vision loop turns a tool-plane correction (x, y in the hover tool frame, metres) into motor offsets with
A_motor_rad_per_tool_m (6x2), the joint motion that translates the tool by that much at the hover pose without turning it.
The PL bounds each joint offset by limits_rad, which cover the +-max_correction_m box through A with a margin; step_rad bounds
the offset change per 5 ms tick (tool speed `speed_m_s`). Before a job may carry this, its descent/grasp/lift/transfer samples
are audited again with the box corners added (tube audit); one failed corner keeps the part on the stop-and-look servo.
"""
import json,shutil,subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

LIMIT_CEILING_RAD=.25         # PL hard ceiling per joint
STEP_CEILING_RAD=.003
OFFSET_TASK_SUFFIXES=('_approach','_close','_lift','_transfer')   # offsets are frozen through the lift and slew out in the transfer


def jacobian(runtime,q,eps=1e-6):
    """6x6: d(translation, rotation vector)/dq at joint angles q (numerical, same FK the planner uses)."""
    q=np.asarray(q[:6],float);T=runtime.fk(q);p0=np.array(T.translation);R0=np.array(T.rotation);J=np.zeros((6,6))
    for i in range(6):
        dq=q.copy();dq[i]+=eps;Ti=runtime.fk(dq)
        J[:3,i]=(np.array(Ti.translation)-p0)/eps
        J[3:,i]=Rotation.from_matrix(R0.T@np.array(Ti.rotation)).as_rotvec()/eps   # tool-frame rotation; zero is what we ask for
    return J,R0


def correction_map(runtime,planner,q_hover):
    """A_motor (6x2): motor offsets for a unit tool-plane translation (tool x, tool y) at the hover, orientation unchanged."""
    J,R=jacobian(runtime,q_hover)
    rhs=np.zeros((6,2));rhs[:3,:]=R[:,:2]
    A_joint=np.linalg.solve(J,rhs)
    return A_joint*np.asarray(planner.direction[:6],float)[:,None],A_joint


def board_locators_match(bundle=Path(__file__).resolve().parents[4]/'fpga_competition_2026/.cache/kv260_can_deploy'):
    """The deployed board bundle carries byte-identical copies of the host's nut / M6 locators (the board loop runs them)."""
    import hashlib
    here=Path(__file__).resolve().parent
    try:return all(hashlib.sha256((here/n).read_bytes()).digest()==hashlib.sha256((Path(bundle)/n).read_bytes()).digest()
                   for n in ('wrist_nut_localization.py','wrist_m6_localization.py'))
    except OSError:return False


def continuous_config(runtime,planner,job,servo,part_class,max_correction_m=.015,speed_m_s=.03,margin=1.15,part_size='m12',radius_px=None):
    """job['visual_servo']['continuous'] for one part, or ValueError when the bounds would exceed the PL ceilings.

    The board runs the host's own locators (OpenCV on the board since 2026-10-30): on the same frame and seed the board and
    the AI PC give identical centres for all four parts (46 replayed frames, 0.0 px), so `servo` is the part's own servo
    config (target pixel, image Jacobian, seed and search radius) exactly as the stop-and-look servo uses it."""
    if part_class not in ('screw','nut') or part_size not in ('m12','m6'):raise ValueError('continuous correction: unknown part')
    q_hover=job['tasks'][0]['goal_rad']
    A,_=correction_map(runtime,planner,q_hover)
    corners=np.array([[sx,sy] for sx in (-1,1) for sy in (-1,1)],float)*max_correction_m
    reach=np.max(np.abs(corners@A.T),axis=0)
    limits=reach*margin+5e-4
    if np.any(limits>LIMIT_CEILING_RAD):raise ValueError('correction box needs more than the PL ceiling on a joint')
    step=float(np.max(np.abs(A).sum(axis=1))*speed_m_s*.005)
    step=min(max(step,1e-5),STEP_CEILING_RAD)
    screw=part_class=='screw'
    # convergence: a screw is gripped anywhere along its shaft (across error counts, along at 0.3); a nut needs its centre
    # (the stop-and-look board gate is 5 px for a nut)
    vision=dict(part_class=part_class,part_size=part_size,target_px=list(servo['target_px']),J_pixels_per_m=servo['J_pixels_per_m'],A_motor_rad_per_tool_m=A.tolist(),
                coarse_center_px=list(servo.get('coarse_center_px',servo['target_px'])),radius_px=float(radius_px or (220. if screw else (110. if part_size=='m6' else 160.))),
                gain=.5,max_correction_m=max_correction_m,tolerance_px=6. if screw and part_size=='m12' else 4.,
                # an M6 screw is gripped just below its head: correct along the axis too (M12: anywhere on the shank)
                along_weight=1. if screw and part_size=='m6' else .3,
                # position law (board loop): target = offset in force at capture + solve(J, e). run3 (10-30, integral law, gain .4)
                # swung twice; run5 (position, 0.25 s assumed) overshot the target 1.6 mm: a fit of its 11 frames gives the image
                # 0.07 s behind the frame's receipt stamp (4.8 px/mm, J says 4.97), so 0.08 s
                # smooth_s: the board writes the PL targets at 100 Hz through a 0.12 s first-order filter; without it the arm
                # reached each per-frame goal at the PL slew and stood still until the next frame (run5: seen as pauses and shaking)
                control='position',capture_lag_s=.08,settled_m=.001,smooth_s=.12)
    # fade_s: after the lift the owner returns the offsets along a 0.8 s raised cosine; switching the channel off let the PL do
    # it at full step (32 mrad on J1/J6 in 0.13 s), seen as a shake of the lifted screw (run15)
    return dict(limits_rad=limits.tolist(),step_rad=step,lease_us=200000,vision=vision,converge_frames=3,lost_frames=15,timeout_s=6.,
                max_correction_m=max_correction_m,tool_speed_m_s=speed_m_s,fade_s=.8,
                note='experimental: board vision loop + PL bounded offsets; tube audit at the correction box corners')


def transfer_times(job,samples):
    """{sample index: seconds since the transfer task started} for the main-branch transfer samples, or {} when the sample
    layout differs from live_planner's (each segment sampled at max(3, ceil(d/0.04)+1) points over [0, 1])."""
    tasks=job['tasks'];ti=next((i for i,t in enumerate(tasks) if t['name'].endswith('_transfer')),None)
    if ti is None:return {}
    idx=[i for i,r in enumerate(samples) if r['branch']=='main' and r['task']==ti and r['name']==tasks[ti]['name']]
    times=[];start=0.
    for seg in tasks[ti]['segments']:
        d=float(seg['duration_s']);times+=[start+u*d for u in np.linspace(0,1,max(3,int(np.ceil(d/.04))+1))];start+=d
    return dict(zip(idx,times)) if len(idx)==len(times) else {}


def tube_audit(folder,job,cfg,runtime,planner,run_geometry,workdir,lock=None):
    """Geometry audit of the offset-carrying tasks with each correction-box corner added; returns per-corner reports.

    The corner sample sets are built one after the other (FK under `lock`: the runtime is not thread safe), then the four
    geometry audits run in parallel processes."""
    import contextlib
    folder=Path(folder);A=np.asarray(cfg['vision']['A_motor_rad_per_tool_m'],float)
    direction=np.asarray(planner.direction[:6],float);m=cfg['max_correction_m']
    samples=json.loads((folder/'collision_samples.json').read_text())
    # The owner disables the channel when the lift is complete, before the transfer starts; disabled, the PL slews every
    # offset to zero at no less than this job's step per 5 ms tick (a chain's merged step is the largest). So a transfer
    # sample carries at most |dq| - step*(t - 0.05 s)/0.005 (run7: the full 15 mm offset held to the end of the transfer
    # took J6 past its limit, though the real offset is zero there 0.3 s into a 2 s transfer).
    # With fade_s the owner itself takes the offsets down along a raised cosine over fade_s (the PL step may make it slower):
    # a sample carries at most max(|dq| * cosine(t), |dq| - step * t).
    decay=transfer_times(job,samples['samples']);step=float(cfg['step_rad']);fade=float(cfg.get('fade_s',0.))
    corners=[(sx*m,sy*m) for sx in (-1,1) for sy in (-1,1)];outs=[]
    for k,corner in enumerate(corners):
        out=Path(workdir)/f'corner_{k}';shutil.rmtree(out,ignore_errors=True);out.mkdir(parents=True)
        for name in ('job_pending_geometry.json','input.json','destinations.json','workflow_config.json'):
            if (folder/name).exists():shutil.copy(folder/name,out/name)
        shutil.copytree(folder/'target_geometry',out/'target_geometry')
        dq=(A@np.asarray(corner,float))/direction;rows=[]
        with (lock or contextlib.nullcontext()):
            for i,row in enumerate(samples['samples']):
                row=dict(row)
                if row['branch']=='main' and row['name'].endswith(OFFSET_TASK_SUFFIXES):
                    d=dq
                    if i in decay:
                        t=max(0.,decay[i]-.05);left=np.maximum(np.abs(dq)-step*t/.005,0.)
                        if fade>0:left=np.maximum(left,np.abs(dq)*(.5+.5*np.cos(np.pi*min(t/fade,1.))))
                        d=np.sign(dq)*left
                    q=list(np.asarray(row['q_rad'][:6],float)+d)+list(row['q_rad'][6:]);T=runtime.fk(q[:6])
                    row.update(q_rad=q,tcp_m=list(np.array(T.translation)),rotation=np.array(T.rotation).tolist())
                rows.append(row)
        (out/'collision_samples.json').write_text(json.dumps(dict(samples,samples=rows)));outs.append(out)
    def one(k):
        try:run_geometry(outs[k]);rep=json.loads((outs[k]/'route_geometry_report.json').read_text())
        except Exception as error:rep=dict(status='ERROR',error=repr(error)[:300])
        return dict(corner=list(corners[k]),status=rep.get('status'),unexpected=len(rep.get('unexpected',[]) or []))
    with ThreadPoolExecutor(max_workers=4) as pool:
        results=list(pool.map(one,range(len(corners))))
    return dict(passed=all(str(r['status']).startswith('PASS') and not r['unexpected'] for r in results),corners=results)
