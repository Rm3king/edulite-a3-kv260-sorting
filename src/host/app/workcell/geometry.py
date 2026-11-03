"""Eye-to-hand geometry with explicit units and quality diagnostics. No robot IO."""
import math
import numpy as np

def transform(value):
    t=np.asarray(value,float)
    if t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1],atol=1e-9):raise ValueError('TRANSFORM')
    if not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-6) or abs(np.linalg.det(t[:3,:3])-1)>1e-6:raise ValueError('ROTATION')
    return t

def plane_point(uv, intrinsics, base_camera, normal, height):
    """Intersect an undistorted optical ray with n.p=height (metres)."""
    k=np.asarray(intrinsics,float);t=transform(base_camera);n=np.asarray(normal,float)
    if k.shape!=(3,3) or not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0 or not np.allclose(k[2],[0,0,1]):raise ValueError('INTRINSICS')
    if n.shape!=(3,) or not np.isfinite(n).all() or not np.isclose(np.linalg.norm(n),1):raise ValueError('PLANE_NORMAL')
    if not math.isfinite(height):raise ValueError('PLANE_HEIGHT')
    ray=t[:3,:3]@np.linalg.solve(k,[*uv,1.]);den=float(n@ray)
    if abs(den)<1e-8:raise ValueError('RAY_PARALLEL_TO_PLANE')
    depth=(height-float(n@t[:3,3]))/den
    if depth<=0:raise ValueError('PLANE_BEHIND_CAMERA')
    return t[:3,3]+depth*ray

def candidate(center, endpoints, calibration, tool, measured_depth_m=None):
    """Top-down parallel-jaw candidate. Calibration validity never inferred."""
    required=['id','T_base_camera','K','plane_normal','table_height_m','part_axis_height_m','verified']
    if any(k not in calibration for k in required) or calibration['verified'] is not True:raise ValueError('CALIBRATION_NOT_VERIFIED')
    if tool.get('verified') is not True or not tool.get('id'):raise ValueError('TCP_NOT_VERIFIED')
    n=np.asarray(calibration['plane_normal'],float);height=calibration['table_height_m']+calibration['part_axis_height_m']
    def point(uv):return plane_point(uv,calibration['K'],calibration['T_base_camera'],n,height)
    p=point(center);a,b=map(point,endpoints);axis=b-a;axis-=n*(n@axis)
    if np.linalg.norm(axis)<1e-5:raise ValueError('DEGENERATE_AXIS')
    axis/=np.linalg.norm(axis)
    # A3 legacy tool extends along local -Z. Its +Z points away from the table.
    # Tool x is the calibrated jaw-closing direction.
    z=n;x=np.cross(axis,n);x/=np.linalg.norm(x);y=np.cross(z,x);rot=np.column_stack((x,y,z))
    pose=np.eye(4);pose[:3,:3]=rot;pose[:3,3]=p
    cmd=pose@np.linalg.inv(transform(tool['T_flange_tcp']))
    camera=np.linalg.inv(transform(calibration['T_base_camera']))@np.r_[p,1]
    residual=None if measured_depth_m is None else float(measured_depth_m-camera[2])
    return dict(schema='workcell.grasp.v1',frame='robot_base',position_m=p.tolist(),rotation=rot.tolist(),
                flange_target=cmd.tolist(),shaft_axis_base=axis.tolist(),jaw_closing_axis_base=x.tolist(),
                yaw_rad_mod_pi=float(math.atan2(axis[1],axis[0])%math.pi),
                calibration_id=calibration['id'],tool_id=tool['id'],position_source='CALIBRATED_PLANE_AT_PART_AXIS_HEIGHT',
                depth_residual_m=residual,physical_authorized=False,
                quality=dict(external_position_error_mm=None,external_angle_error_deg=None,collision_checked=False))

def rigid_fit(camera_points, base_points):
    """Paired-point Kabsch estimate, report training residual separately."""
    a,b=np.asarray(camera_points,float),np.asarray(base_points,float)
    if a.shape!=b.shape or a.ndim!=2 or a.shape[1]!=3 or len(a)<4 or not np.isfinite(a).all() or not np.isfinite(b).all():raise ValueError('PAIRED_POINTS')
    if np.linalg.matrix_rank(a-a.mean(0),tol=1e-8)<2:raise ValueError('DEGENERATE_CALIBRATION_POINTS')
    u,_,vt=np.linalg.svd((a-a.mean(0)).T@(b-b.mean(0)))
    r=vt.T@np.diag([1.,1.,np.linalg.det(vt.T@u.T)])@u.T
    t=np.eye(4);t[:3,:3]=r;t[:3,3]=b.mean(0)-r@a.mean(0)
    err=np.linalg.norm((r@a.T).T+t[:3,3]-b,axis=1)
    return dict(T_base_camera=t.tolist(),fit_rmse_mm=float(np.sqrt(np.mean(err**2))*1000),
                fit_max_mm=float(err.max()*1000),external_validation_error_mm=None,verified=False)
