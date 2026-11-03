#!/usr/bin/python3
"""Validated seven-channel cubics, bounded lookahead and Cartesian paths.

No driver imports or hardware access. ARM solves IK before execution; PL evaluates
quantized cubic segments at 5 ms. Cartesian error is sampled, not a proof.
"""
import math
import numpy as np
from joint_trajectory import Q24, evaluate, vector
from pl_transport import extrema


def cubic(p, c, duration, kind='CUBIC_CONTINUOUS'):
    c=np.asarray(c,dtype=float)
    if c.shape!=(7,4) or not np.isfinite(c).all():raise ValueError('finite 7x4 cubic required')
    n=round(duration/p.tick)
    if n<1 or abs(n*p.tick-duration)>1e-10 or duration>p.max_duration:raise ValueError('duration/tick mismatch')
    pv=[];pa=[]
    for row in c:
        u=[0.,1.]
        if row[3] and 0 < -row[2]/(3*row[3]) < 1:u.append(-row[2]/(3*row[3]))
        pv.append(max(abs(row[1]+2*row[2]*x+3*row[3]*x*x)/duration for x in u))
        pa.append(max(abs(2*row[2]),abs(2*row[2]+6*row[3]))/duration**2)
    out=p._plan(kind,c.tolist(),duration,pv,pa)
    out.update(start_rad=c[:,0].tolist(),goal_rad=c.sum(axis=1).tolist(),
      start_velocity_rad_s=(c[:,1]/duration).tolist(),
      end_velocity_rad_s=((c[:,1]+2*c[:,2]+3*c[:,3])/duration).tolist())
    cubic_words(out,p)
    return out


def cubic_words(plan,p):
    if plan['kind'] not in ('CUBIC_CONTINUOUS','CONTROLLED_STOP_CANDIDATE') or plan['profile_id']!=p.profile_id:raise ValueError('cubic profile/kind mismatch')
    if plan['tick_s']!=.005 or type(plan['ticks']) is not int or not 1<=plan['ticks']<=120000:raise ValueError('5 ms ticks required')
    duration=plan['ticks']*.005
    c=np.asarray(plan['motor_coefficients_rad'],dtype=float)
    if c.shape!=(7,4) or not np.isfinite(c).all():raise ValueError('invalid coefficients')
    words=[];bounds=[];ends=[];vstart=[];vend=[]
    for i,row in enumerate(c):
        lo,hi=sorted((p.lower[i]*p.direction[i]+p.offset[i],p.upper[i]*p.direction[i]+p.offset[i]))
        l,h=math.ceil(lo*Q24),math.floor(hi*Q24)
        quant=[round(x*Q24) for x in row]
        # Preserve quantized endpoints; only sub-LSB boundary rounding is allowed.
        endpoint=round(sum(row)*Q24)
        first=max(l,min(h,quant[0]));last=max(l,min(h,endpoint))
        if abs(first/Q24-row[0])>1.01/Q24 or abs(last/Q24-sum(row))>1.01/Q24:raise ValueError('cubic endpoint outside legal interval')
        slope0=quant[1];slope1=round((row[1]+2*row[2]+3*row[3])*Q24)
        delta=last-first;quant=[first,slope0,3*delta-2*slope0-slope1,-2*delta+slope0+slope1]
        if any(not -(1<<31)<=v<1<<31 for v in quant):raise ValueError('coefficient overflow')
        mn,mx=extrema(quant)
        if mn<l or mx>h:raise ValueError('quantized cubic outside legal position interval')
        a,b,d,e=quant
        probes=[0.,1.]
        if e and 0 < -d/(3*e)<1:probes.append(-d/(3*e))
        vmax=max(abs(b+2*d*u+3*e*u*u)/Q24/duration for u in probes)
        amax=max(abs(2*d),abs(2*d+6*e))/Q24/duration**2
        if vmax>p.velocity[i]+2e-5 or amax>p.acceleration[i]+1e-4:raise ValueError('quantized cubic derivative bound exceeded')
        words.extend(quant);bounds.append((l,h));ends.append(sum(quant))
        vstart.append(b/Q24/duration);vend.append((b+2*d+3*e)/Q24/duration)
    step,rem=divmod(Q24,plan['ticks'])
    words += [x[0] for x in bounds]+[x[1] for x in bounds]+[plan['ticks'],step,rem]
    return words,ends,vstart,vend


def split(p,plan,max_seconds=.25):
    """Exact polynomial restriction; no stop at internal boundaries."""
    c=np.asarray(plan['coefficients_rad']);n=plan['ticks'];chunk=max(1,int(max_seconds/p.tick))
    parts=[];pieces=math.ceil(n/chunk)
    # Distribute ticks evenly; a 5 ms remainder amplifies coefficient
    # quantization into excessive acceleration even when the parent is legal.
    edges=[round(i*n/pieces) for i in range(pieces+1)]
    for first,end in zip(edges,edges[1:]):
        count=end-first;u=first/n;s=count/n
        rows=np.column_stack((c[:,0]+c[:,1]*u+c[:,2]*u*u+c[:,3]*u**3,
          s*(c[:,1]+2*c[:,2]*u+3*c[:,3]*u*u),s*s*(c[:,2]+3*c[:,3]*u),s**3*c[:,3]))
        parts.append(cubic(p,rows,count*p.tick))
    return parts


def brake_at(p,plan):
    sample=evaluate(plan,plan['duration_s'])
    b=p.brake(sample['q'],sample['v'])
    return cubic(p,b['coefficients_rad'],b['duration_s'],'CONTROLLED_STOP_CANDIDATE')


def hermite(p,q0,q1,v0,v1,seconds):
    q0,q1,v0,v1=map(lambda v:np.asarray(vector(v,'Hermite boundary')), (q0,q1,v0,v1))
    delta=q1-q0
    return cubic(p,np.column_stack((q0,seconds*v0,3*delta-seconds*(2*v0+v1),-2*delta+seconds*(v0+v1))),seconds)


class CartesianPlanner:
    def __init__(self,runtime,planner):self.r,self.p=runtime,planner

    def path(self,start7,kind='line',delta=(.02,0,0),radius=.01,axes=(0,1),tcp_speed=.025):
        from motion_runtime import pose_values,residual
        p=self.p;r=self.r;q0=p.positions(start7)
        T=r.fk(q0[:6]);origin=T.translation.copy();R=T.rotation.copy()
        if not math.isfinite(tcp_speed) or not 0<tcp_speed<=.1:raise ValueError('TCP speed must be in (0,0.1] m/s')
        if kind=='line':
            delta=np.asarray(delta,dtype=float)
            if delta.shape!=(3,) or not np.isfinite(delta).all() or not 0<np.linalg.norm(delta)<=.1:raise ValueError('line displacement invalid')
            length=float(np.linalg.norm(delta))
            def position(s):return origin+s*delta
        elif kind=='circle':
            if axes not in ((0,1),(0,2),(1,2)) or not .002<=radius<=.04:raise ValueError('circle axes/radius invalid')
            length=2*math.pi*radius;axis_a,axis_b=axes
            def position(s):
                v=origin.copy();angle=2*math.pi*s
                v[axis_a]+=radius*(math.cos(angle)-1);v[axis_b]+=radius*math.sin(angle);return v
        else:raise ValueError('line or circle required')
        # Cubic progress gives zero end velocities. Knot spacing <=1.5 mm;
        # solve continuous branch once; jointly stretch timing if limits demand.
        count=max(16,math.ceil(length/.0015));count=min(count,192)
        s=np.linspace(0,1,count+1);qs=[np.asarray(q0[:6])]
        singular=[]
        import pinocchio as pin
        for x in s[1:]:
            target=pin.SE3(R,position(x))
            q=r.ik(pose_values(target),qs[-1])
            if np.max(np.abs(q-qs[-1]))>.12:raise ValueError('IK branch jump')
            qs.append(q)
            full=r.kin._to_pinocchio_q(q.tolist())
            J=pin.computeFrameJacobian(r.kin.model,r.kin.data,full,r.kin._ee_frame_id,pin.LOCAL)[:,:6]
            smallest=float(np.linalg.svd(J,compute_uv=False)[-1]);singular.append(smallest)
            if smallest<.0005:raise ValueError('path too near singularity')
        # Hermite slopes with respect to geometric path progress, then smoothstep
        # time warp. Each knot shares exactly the same dq/dt on both sides.
        qs=np.array(qs);dqds=np.gradient(qs,1/count,axis=0,edge_order=2)
        # Invert smoothstep to put approximately uniform geometric knots in time.
        u=np.array([_inverse_smoothstep(x) for x in s])
        duration=max(1.,1.5*length/tcp_speed)
        for attempt in range(40):
            ticks=np.maximum(1,np.diff(np.round(u*duration/p.tick).astype(int)))
            times=np.r_[0,np.cumsum(ticks)*p.tick];total=times[-1]
            du=times/total;progress_velocity=6*du*(1-du)/total
            velocity=dqds*progress_velocity[:,None];velocity[0]=0;velocity[-1]=0
            plans=[];accepted=True
            try:
                for i in range(count):
                    a=qs[i].tolist()+[q0[6]];b=qs[i+1].tolist()+[q0[6]]
                    plans.append(hermite(p,a,b,velocity[i].tolist()+[0.],velocity[i+1].tolist()+[0.],ticks[i]*p.tick))
            except ValueError:
                accepted=False
            if accepted:break
            duration*=1.15
        else:raise ValueError('Cartesian time parameterization exhausted')
        errors=[];orientation=[]
        for i,plan in enumerate(plans):
            words,_,_,_=cubic_words(plan,p)
            motor=np.array(words[:28],dtype=float).reshape(7,4)/Q24
            coeff=motor*np.array(p.direction)[:,None];coeff[:,0]-=np.array(p.offset)*np.array(p.direction)
            for f in np.linspace(0,1,17):
                q=((coeff[:,3]*f+coeff[:,2])*f+coeff[:,1])*f+coeff[:,0]
                actual=r.fk(q[:6]);elapsed=times[i]+f*plan['duration_s'];a=elapsed/total;desired=a*a*(3-2*a)
                errors.append(float(np.linalg.norm(actual.translation-position(desired))))
                orientation.append(residual(actual,pin.SE3(R,actual.translation))[1])
        if max(errors)>.00025 or max(orientation)>.002:raise ValueError('quantized Cartesian sample error too large')
        return {'kind':kind,'segments':plans,'start_rad':q0,'goal_rad':plans[-1]['goal_rad'],
          'model_id':r.model_id,'profile_id':p.profile_id,'duration_s':float(total),
          'tcp_speed_requested_m_s':tcp_speed,'radius_m':radius if kind=='circle' else None,'axes':axes,
          'delta_m':delta.tolist() if kind=='line' else None,'origin_xyz_m':origin.tolist(),'rotation':R.tolist(),
          'sampled_position_error_m':max(errors),'sampled_orientation_error_rad':max(orientation),
          'minimum_jacobian_singular_value':min(singular),'continuous_error_bound_verified':False}


def _inverse_smoothstep(s):
    lo,hi=0.,1.
    for _ in range(50):
        mid=(lo+hi)/2
        if mid*mid*(3-2*mid)<s:lo=mid
        else:hi=mid
    return (lo+hi)/2


def trim_first_tick(p,plan):
    """Encode after an earlier endpoint: first new target is t=5 ms, not t=0.

    v5 emits both endpoints (n+1 samples). Restricting the next polynomial to
    [5ms,T] removes the duplicated boundary sample without changing FPGA logic.
    The omitted 5ms interval is checked against reviewed velocity/acceleration.
    """
    if plan['ticks']<=1:
        if max(map(abs,plan.get('start_velocity_rad_s',[0.]*7)))>1e-3:raise ValueError('moving one-tick segment cannot be joined')
        return plan
    c=np.asarray(plan['coefficients_rad']);n=plan['ticks'];u=1/n;s=(n-1)/n
    rows=np.column_stack((c[:,0]+c[:,1]*u+c[:,2]*u*u+c[:,3]*u**3,
      s*(c[:,1]+2*c[:,2]*u+3*c[:,3]*u*u),s*s*(c[:,2]+3*c[:,3]*u),s**3*c[:,3]))
    return cubic(p,rows,(n-1)*p.tick)


def clamped_c2_segments(p,positions,ticks):
    """Clamped cubic spline with zero endpoint velocity and shared acceleration.

    Solve all seven channels together. Integer tick intervals are used in the
    linear system, so no post-fit time rounding creates acceleration jumps.
    """
    q=np.asarray(positions,dtype=float);ticks=np.asarray(ticks,dtype=int)
    if q.ndim!=2 or q.shape[1]!=7 or len(ticks)!=len(q)-1 or np.any(ticks<1):raise ValueError('invalid spline knots')
    if not np.isfinite(q).all():raise ValueError('nonfinite spline knots')
    for row in q:p.positions(row.tolist())
    h=ticks*p.tick;n=len(q);A=np.zeros((n,n));rhs=np.zeros((n,7))
    A[0,0]=2*h[0];A[0,1]=h[0];rhs[0]=6*(q[1]-q[0])/h[0]
    A[-1,-2]=h[-1];A[-1,-1]=2*h[-1];rhs[-1]=-6*(q[-1]-q[-2])/h[-1]
    for i in range(1,n-1):
        A[i,i-1]=h[i-1];A[i,i]=2*(h[i-1]+h[i]);A[i,i+1]=h[i]
        rhs[i]=6*((q[i+1]-q[i])/h[i]-(q[i]-q[i-1])/h[i-1])
    M=np.linalg.solve(A,rhs);segments=[]
    for i,duration in enumerate(h):
        delta=q[i+1]-q[i]
        c=np.column_stack((q[i],delta-duration**2*(2*M[i]+M[i+1])/6,
            M[i]*duration**2/2,(M[i+1]-M[i])*duration**2/6))
        segments.append(cubic(p,c,float(duration)))
    return segments


def smooth_cartesian_path(runtime,p,start7,kind='circle',delta=(0,0,.02),radius=.04,axes=(0,1),tcp_speed=.025):
    """Uniform-time IK knots + quintic Cartesian progress + C2 joint splines."""
    from motion_runtime import pose_values,residual
    import pinocchio as pin
    q0=p.positions(start7);T=runtime.fk(q0[:6]);origin=T.translation.copy();R=T.rotation.copy()
    if not math.isfinite(tcp_speed) or not 0<tcp_speed<=.1:raise ValueError('TCP speed invalid')
    if kind=='circle':
        if axes not in ((0,1),(0,2),(1,2)) or not .002<=radius<=.04:raise ValueError('circle axes/radius invalid')
        axis_a,axis_b=axes;length=2*math.pi*radius
        def position(s):
            out=origin.copy();theta=2*math.pi*s;out[axis_a]+=radius*(math.cos(theta)-1);out[axis_b]+=radius*math.sin(theta);return out
    elif kind=='line':
        delta=np.asarray(delta,dtype=float)
        if delta.shape!=(3,) or not np.isfinite(delta).all() or not 0<np.linalg.norm(delta)<=.1:raise ValueError('line displacement invalid')
        length=float(np.linalg.norm(delta))
        def position(s):return origin+s*delta
    else:raise ValueError('line/circle required')
    def progress(u):return u**3*(10+u*(-15+6*u))
    count=max(24,min(192,math.ceil(length/.002)));u=np.linspace(0,1,count+1);q=[q0];singular=[]
    for f in u[1:]:
        target=pin.SE3(R,position(progress(float(f))));sol=runtime.ik(pose_values(target),q[-1][:6])
        if np.max(np.abs(sol-np.array(q[-1][:6])))>.12:raise ValueError('IK branch jump')
        q.append(sol.tolist()+[q0[6]])
        full=runtime.kin._to_pinocchio_q(sol.tolist())
        J=pin.computeFrameJacobian(runtime.kin.model,runtime.kin.data,full,runtime.kin._ee_frame_id,pin.LOCAL)[:,:6]
        sv=float(np.linalg.svd(J,compute_uv=False)[-1]);singular.append(sv)
        if sv<.0005:raise ValueError('near singularity')
    # Equal integer intervals mean that normalized IK knots exactly match time.
    total=max(1.,1.875*length/tcp_speed)
    for attempt in range(40):
        ticks=max(2,math.ceil(total/count/p.tick));total=ticks*count*p.tick
        try:segments=clamped_c2_segments(p,q,[ticks]*count);break
        except ValueError:total*=1.12
    else:raise ValueError('C2 time parameterization exhausted')
    errors=[];orient=[];jumps=[];jerks=[];previous=None
    for i,plan in enumerate(segments):
        c=np.asarray(plan['coefficients_rad']);dt=plan['duration_s'];a0=2*c[:,2]/dt**2;ae=(2*c[:,2]+6*c[:,3])/dt**2
        if previous is not None:jumps.append(abs(a0-previous))
        previous=ae;jerks.append(abs(6*c[:,3]/dt**3))
        words,_,_,_=cubic_words(plan,p);mc=np.array(words[:28]).reshape(7,4)/(1<<24)
        for f in np.linspace(0,1,17):
            motor=((mc[:,3]*f+mc[:,2])*f+mc[:,1])*f+mc[:,0];angles=(motor-np.array(p.offset))*np.array(p.direction)
            actual=runtime.fk(angles[:6]);desired=pin.SE3(R,position(progress((i+f)/count)))
            a,b=residual(actual,desired);errors.append(a);orient.append(b)
    if max(errors)>.00025 or max(orient)>.002:raise ValueError('C2 Cartesian sampled error exceeded')
    return dict(kind=kind,segments=segments,start_rad=q0,goal_rad=segments[-1]['goal_rad'],model_id=runtime.model_id,profile_id=p.profile_id,
      duration_s=total,tcp_speed_requested_m_s=tcp_speed,radius_m=radius if kind=='circle' else None,axes=axes,
      delta_m=delta.tolist() if kind=='line' else None,origin_xyz_m=origin.tolist(),rotation=R.tolist(),
      sampled_position_error_m=max(errors),sampled_orientation_error_rad=max(orient),minimum_jacobian_singular_value=min(singular),
      continuous_error_bound_verified=False,time_law='QUINTIC_PROGRESS',joint_continuity='C2',
      maximum_acceleration_jump_rad_s2=np.max(jumps,axis=0).tolist(),maximum_jerk_rad_s3=np.max(jerks,axis=0).tolist())
