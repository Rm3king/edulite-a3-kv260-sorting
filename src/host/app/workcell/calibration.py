"""Offline eye-to-hand, rigid-tip pivot and support-plane fitting, in metres.

T_A_B maps B coordinates into A. All outputs remain unverified. Fitting never
grants physical execution, and no robot/camera drivers are imported here.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from .geometry import transform


def project_rotation(r):
    u,_,vt=np.linalg.svd(r)
    return u@np.diag([1.,1.,np.linalg.det(u@vt)])@vt


def residuals(gs,cs,x,y):
    position=[];angle=[]
    for g,c in zip(gs,cs):
        a,b=g@x,y@c
        position.append(float(np.linalg.norm(a[:3,3]-b[:3,3])*1000))
        angle.append(float(np.degrees(np.arccos(np.clip((np.trace(a[:3,:3].T@b[:3,:3])-1)/2,-1,1)))))
    return dict(position_mm=position,position_rmse_mm=float(np.sqrt(np.mean(np.square(position)))) if position else None,
                rotation_deg=angle,rotation_max_deg=max(angle) if angle else None)


def eye_to_hand(samples,holdout=()):
    if len(samples)<6:raise ValueError('NEED_AT_LEAST_SIX_DIVERSE_POSES')
    gs=[transform(s['T_base_end']) for s in samples];cs=[transform(s['T_camera_board']) for s in samples]
    pairs=[(np.linalg.inv(gs[j])@gs[i],np.linalg.inv(cs[j])@cs[i]) for i in range(len(gs)) for j in range(i)]
    excitation=np.vstack([a[:3,:3]-np.eye(3) for a,b in pairs])
    spectrum=np.linalg.svd(excitation,compute_uv=False)
    if spectrum[-1]<1e-4:raise ValueError('ROTATIONS_NEED_NONPARALLEL_AXES')
    equations=np.vstack([np.kron(np.eye(3),a[:3,:3])-np.kron(b[:3,:3].T,np.eye(3)) for a,b in pairs])
    _,_,vt=np.linalg.svd(equations,full_matrices=False)
    r=vt[-1].reshape(3,3,order='F')
    if np.linalg.det(r)<0:r=-r
    r=project_rotation(r)
    lhs=np.vstack([a[:3,:3]-np.eye(3) for a,b in pairs]);rhs=np.concatenate([r@b[:3,3]-a[:3,3] for a,b in pairs])
    t=np.linalg.lstsq(lhs,rhs,rcond=None)[0]
    x=np.eye(4);x[:3,:3]=r;x[:3,3]=t
    ys=[g@x@np.linalg.inv(c) for g,c in zip(gs,cs)]
    y=np.eye(4);y[:3,:3]=project_rotation(np.mean([v[:3,:3] for v in ys],axis=0));y[:3,3]=np.mean([v[:3,3] for v in ys],axis=0)
    hg=[transform(s['T_base_end']) for s in holdout];hc=[transform(s['T_camera_board']) for s in holdout]
    return dict(T_base_camera=y.tolist(),T_end_board=x.tolist(),training=residuals(gs,cs,x,y),
                held_out_pose_consistency=residuals(hg,hc,x,y),rotation_excitation_singular_values=spectrum.tolist(),
                verified=False,independent_physical_tcp_error_mm=None,
                scope='AX=XB rigid model fit; held-out pose consistency is not physical TCP accuracy')


def tcp_pivot(poses,holdout=()):
    if len(poses)<4:raise ValueError('NEED_FOUR_OR_MORE_TOOL_ORIENTATIONS')
    gs=[transform(g) for g in poses]
    a=np.vstack([np.c_[g[:3,:3],-np.eye(3)] for g in gs]);b=np.concatenate([-g[:3,3] for g in gs])
    x,_,rank,s=np.linalg.lstsq(a,b,rcond=None)
    if rank<6 or s[-1]<1e-4:raise ValueError('DEGENERATE_PIVOT_ORIENTATIONS')
    def errors(values):return [float(np.linalg.norm((transform(g)@np.r_[x[:3],1])[:3]-x[3:])*1000) for g in values]
    return dict(tip_in_end_m=x[:3].tolist(),fixed_point_base_m=x[3:].tolist(),
                training_error_mm=errors(poses),held_out_error_mm=errors(holdout),verified=False,
                scope='Rigid tip translation only; jaw closing axis and actual finger contact centre require separate validation')


def support_plane(points,threshold_m=.002,holdout=()):
    p=np.asarray(points,float)
    if p.ndim!=2 or p.shape[1]!=3 or len(p)<12 or not np.isfinite(p).all() or not 0<threshold_m<.05:
        raise ValueError('PLANE_POINTS_OR_THRESHOLD')
    rng=np.random.default_rng(20261022);best=np.zeros(len(p),bool)
    for _ in range(200):
        a,b,c=p[rng.choice(len(p),3,replace=False)];n=np.cross(b-a,c-a)
        if np.linalg.norm(n)<1e-10:continue
        n/=np.linalg.norm(n);mask=np.abs(p@n-a@n)<threshold_m
        if mask.sum()>best.sum():best=mask
    if best.sum()<max(12,len(p)//2):raise ValueError('NO_DOMINANT_PLANE')
    centre=p[best].mean(0);_,s,vt=np.linalg.svd(p[best]-centre,full_matrices=False)
    if s[1]<1e-5:raise ValueError('COLLINEAR_PLANE_POINTS')
    n=vt[-1]
    if n[2]<0:n=-n
    height=float(n@centre);err=p[best]@n-height
    h=np.asarray(holdout,float)
    if h.size and (h.ndim!=2 or h.shape[1]!=3 or not np.isfinite(h).all()):raise ValueError('HELD_OUT_PLANE_POINTS')
    return dict(plane_normal=n.tolist(),height_m=height,inliers=int(best.sum()),samples=len(p),
                fit_rmse_mm=float(np.sqrt(np.mean(err**2))*1000),held_out_plane_distances_mm=(abs(h@n-height)*1000).tolist() if h.size else [],
                verified=False,scope='Base-frame points; plane fit does not determine bin wall, tool clearance or object grasp height')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('kind',choices=['eye-to-hand','tcp-pivot','plane'])
    p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise SystemExit('PRESERVE_EXISTING_CALIBRATION')
    data=json.loads(a.input.read_text())
    if data.get('units')!='m':raise ValueError('UNITS_M_REQUIRED')
    if a.kind=='eye-to-hand':value=eye_to_hand(data['samples'],data.get('holdout',[]))
    elif a.kind=='tcp-pivot':value=tcp_pivot(data['poses'],data.get('holdout',[]))
    else:value=support_plane(data['points_base_m'],data.get('threshold_m',.002),data.get('holdout',[]))
    value.update(schema='workcell.calibration.fit.v1',kind=a.kind,motor_commands_sent=0)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(value,indent=2)+'\n');print(json.dumps(value,indent=2))


if __name__=='__main__':main()
