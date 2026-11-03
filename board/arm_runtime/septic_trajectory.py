#!/usr/bin/python3
"""Degree-seven C3 references and checked Q8.24 encoding. No hardware access.

Shared position/velocity/acceleration/jerk boundaries give mathematical C3.
Fixed-point derivative residuals are measured separately; sampled motor motion
is not asserted to be C3. Integer Bernstein hulls certify common bounds;
inconclusive hulls retain the original polynomial-root extrema checks.
"""
import math
import numpy as np
from numpy.polynomial import polynomial as poly
from joint_trajectory import Q24,vector

KIND='C3_SEPTIC_CONTINUOUS'
S7=np.array([0.,0.,0.,0.,35.,-84.,70.,-20.])
BOUND_STATS={'hull_checks':0,'root_fallbacks':0}
# Integer-scaled power-to-Bernstein conversion. The polynomial lies in the
# convex hull of its Bernstein coefficients on [0,1]. No sample-only shortcut.
BERNSTEIN=[]
for degree in range(8):
    denominator=math.lcm(*(math.comb(degree,i) for i in range(degree+1)))
    matrix=tuple(tuple(math.comb(k,i)*(denominator//math.comb(degree,i)) for i in range(k+1)) for k in range(degree+1))
    BERNSTEIN.append((denominator,matrix))


def bernstein_integer_bounds(row,order=0):
    c=list(map(int,row))
    for _ in range(order):c=[(i+1)*v for i,v in enumerate(c[1:])]
    denominator,matrix=BERNSTEIN[len(c)-1]
    coefficients=[sum(weight*c[i] for i,weight in enumerate(weights)) for weights in matrix]
    return min(coefficients),max(coefficients),denominator


def bernstein_bounds(row,order=0,duration=1.):
    c=list(map(float,row))
    for _ in range(order):c=[(i+1)*v for i,v in enumerate(c[1:])]
    denominator,matrix=BERNSTEIN[len(c)-1]
    values=[math.fsum(weight*c[i] for i,weight in enumerate(weights))/denominator for weights in matrix]
    # This floating variant supplies conservative metadata only. The command
    # bounds below use integer hull numerators with exact position comparisons.
    padding=1e-13*max(1.,sum(map(abs,c)))
    return (min(values)-padding)/duration**order,(max(values)+padding)/duration**order


def derivative(row,u,order=0,duration=1.):
    # This tiny degree<=7 operation used to allocate several NumPy arrays on
    # every boundary check. Preserve the same multiplication/Horner order with
    # scalar doubles; the complete prior transmitted stream is compared below.
    c=list(map(float,row))
    for _ in range(order):c=[(i+1)*x for i,x in enumerate(c[1:])]
    value=0.
    if c:
        value=c[-1]
        for x in reversed(c[:-1]):value=x+value*u
    return value/duration**order


def extrema(row,order=0,duration=1.):
    # Bounds repeatedly inspect identical constant/trimmed rows. Cache the
    # arithmetic result only; every caller still checks its own physical bound.
    return _cached_extrema(tuple(map(float,row)),order,float(duration))


from functools import lru_cache


@lru_cache(maxsize=16384)
def _cached_extrema(row,order=0,duration=1.):
    c=poly.polyder(np.asarray(row,dtype=float),order)
    roots=poly.polyroots(poly.polyder(c)) if len(c)>1 else []
    probes=[0.,1.]+[float(z.real) for z in roots if abs(z.imag)<1e-8 and 0<z.real<1]
    values=[float(poly.polyval(x,np.asarray(c,dtype=np.longdouble)))/duration**order for x in probes]
    return min(values),max(values)


def restrict(rows,start,scale):
    """Exact polynomial restriction p(start + scale*u), before quantization."""
    c=np.asarray(rows,dtype=float)
    return np.array([[sum(c[i,j]*math.comb(j,k)*start**(j-k)*scale**k for j in range(k,8))
                      for k in range(8)] for i in range(7)])


def boundary(plan,order=0,end=False,motor=False):
    c=plan['motor_coefficients_rad' if motor else 'coefficients_rad']
    return [derivative(row,1. if end else 0.,order,plan['duration_s']) for row in c]


def septic(p,rows,duration,_validate=True):
    c=np.asarray(rows,dtype=float);n=round(duration/p.tick)
    if c.shape!=(7,8) or not np.isfinite(c).all():raise ValueError('finite 7x8 polynomial required')
    if n<2 or abs(n*p.tick-duration)>1e-10 or duration>p.max_duration:raise ValueError('septic tick duration invalid')
    peaks=[]
    for order in (1,2):peaks.append([max(map(abs,bernstein_bounds(row,order,duration))) for row in c])
    out=p._plan(KIND,c.tolist(),duration,*peaks)
    out.update(start_rad=c[:,0].tolist(),goal_rad=c.sum(axis=1).tolist(),
        joint_continuity='C3_REFERENCE',pending=['physical tracking, external TCP metrology, fixed-point derivative residuals'])
    for order,name in [(1,'velocity_rad_s'),(2,'acceleration_rad_s2'),(3,'jerk_rad_s3')]:
        out['start_'+name]=boundary(out,order);out['end_'+name]=boundary(out,order,True)
    if _validate:septic_words(out,p)
    return out


def quantize_boundary(row,lo,hi):
    """Integer Hermite construction preserves endpoint q and scaled v/a/j.

    Quantize Taylor coefficients at both ends, then solve the remaining four
    coefficients using integer arithmetic. No hidden saturation/clipping.
    """
    first,last=round(row[0]*Q24),round(sum(row)*Q24)
    for value,raw in [(first,row[0]),(last,sum(row))]:
        if not lo<=value<=hi and abs(max(lo,min(hi,value))/Q24-raw)>1.01/Q24:
            raise ValueError('septic endpoint outside legal interval')
    first=max(lo,min(hi,first));last=max(lo,min(hi,last))
    a,b,c,d=[first]+[round(row[i]*Q24) for i in (1,2,3)]
    end=[round(derivative(row,1.,i)*Q24/math.factorial(i)) for i in (1,2,3)]
    r0=last-a-b-c-d;r1=end[0]-b-2*c-3*d
    r2=2*end[1]-2*c-6*d;r3=6*end[2]-6*d
    return [a,b,c,d,35*r0-15*r1+5*r2//2-r3//6,
        -84*r0+39*r1-7*r2+r3//2,70*r0-34*r1+13*r2//2-r3//2,
        -20*r0+10*r1-2*r2+r3//6]


def septic_words(plan,p):
    if plan['kind']!=KIND or plan['profile_id']!=p.profile_id:raise ValueError('septic profile/kind mismatch')
    if plan['tick_s']!=.005 or type(plan['ticks']) is not int or not 2<=plan['ticks']<=120000:raise ValueError('septic requires 5 ms ticks')
    duration=plan['ticks']*.005
    if abs(plan['duration_s']-duration)>1e-10:raise ValueError('septic duration mismatch')
    robot=np.asarray(plan['coefficients_rad'],dtype=float);motor=np.asarray(plan['motor_coefficients_rad'],dtype=float)
    if motor.shape!=(7,8) or robot.shape!=(7,8) or not np.isfinite(motor).all() or not np.isfinite(robot).all():raise ValueError('invalid septic coefficients')
    expected=robot*np.array(p.direction)[:,None];expected[:,0]+=p.offset
    if np.max(np.abs(expected-motor))>1e-12:raise ValueError('motor/robot polynomial mismatch')
    limits=tuple(tuple(sorted((p.lower[i]*p.direction[i]+p.offset[i],p.upper[i]*p.direction[i]+p.offset[i]))) for i in range(7))
    w,e,sv,ev=_cached_septic_words(motor.astype('<f8',copy=False).tobytes(),plan['ticks'],limits,tuple(p.velocity),tuple(p.acceleration))
    # Fresh containers: transport/report callers may annotate their copy.
    return list(w),list(e),[list(x) for x in sv],[list(x) for x in ev]


@lru_cache(maxsize=2048)
def _cached_septic_words(motor_bytes,n,limits,velocity,acceleration):
    motor=np.frombuffer(motor_bytes,dtype='<f8').reshape(7,8);duration=n*.005
    words=[];bounds=[];ends=[];derivatives=[]
    for i,row in enumerate(motor):
        low,high=limits[i]
        lo,hi=math.ceil(low*Q24),math.floor(high*Q24)
        q=quantize_boundary(row,lo,hi)
        if any(not -(1<<31)<=v<1<<31 for v in q):raise ValueError('septic Q8.24 coefficient overflow')
        mn,mx,den=bernstein_integer_bounds(q)
        BOUND_STATS['hull_checks']+=1
        if mn<lo*den or mx>hi*den:
            BOUND_STATS['root_fallbacks']+=1;mn,mx=extrema(q)
        else:mn,mx=mn/den,mx/den
        # This tolerance is <1e-12 rad and only covers floating root evaluation.
        if mn<lo-1e-5 or mx>hi+1e-5:raise ValueError('quantized septic outside position limits')
        for order,limit,roundoff in [(1,velocity[i],2e-5),(2,acceleration[i],1e-4)]:
            mn,mx,den=bernstein_integer_bounds(q,order)
            BOUND_STATS['hull_checks']+=1
            upper=max(abs(mn),abs(mx))/(den*duration**order)/Q24
            # A tiny arithmetic margin makes acceptance conservative; ambiguous
            # cases keep the exact prior root-based rejection behavior.
            if upper+1e-12>limit+roundoff:
                BOUND_STATS['root_fallbacks']+=1
                upper=max(map(abs,extrema(q,order,duration)))/Q24
            if upper>limit+roundoff:
                raise ValueError('quantized septic derivative bound exceeded')
        words.extend(q);bounds.append((lo,hi));ends.append(sum(q))
        derivatives.append([[derivative(q,u,k,duration)/Q24 for k in (1,2,3)] for u in (0.,1.)])
    step,rem=divmod(Q24,n)
    words += [b[0] for b in bounds]+[b[1] for b in bounds]+[n,step,rem]
    return tuple(words),tuple(ends),tuple(tuple(x) for x in np.array(derivatives)[:,0,:]),tuple(tuple(x) for x in np.array(derivatives)[:,1,:])


def split_rows(p,rows,ticks,max_seconds=.25):
    c=np.asarray(rows,dtype=float);pieces=math.ceil(ticks/max(2,int(max_seconds/p.tick)))
    edges=[round(i*ticks/pieces) for i in range(pieces+1)]
    return [septic(p,restrict(c,a/ticks,(b-a)/ticks),(b-a)*p.tick) for a,b in zip(edges,edges[1:])]


def movej_c3(p,start,goal,minimum_duration_s=.1):
    start,goal=p.positions(start),p.positions(goal);delta=np.array(goal)-start
    vmax=max(map(abs,extrema(S7,1)));amax=max(map(abs,extrema(S7,2)))
    duration=max(minimum_duration_s,max(vmax*abs(d)/v for d,v in zip(delta,p.velocity)),
        max(math.sqrt(amax*abs(d)/a) for d,a in zip(delta,p.acceleration)))
    ticks,_=p._duration(float(duration)*1.001)
    c=delta[:,None]*S7;c[:,0]=start
    # Split before fixed-point conversion: large global septic coefficients
    # may exceed Q8.24 even though all local motion is well within joint limits.
    return split_rows(p,c,ticks)


MIN_CRUISE_TICKS=12        # 60 ms at the 5 ms tick
MIN_INTERIOR_SEGMENT_S=.04  # shortest segment the executor can chain; first and last segments are exempt (rest at one end)


def segment_floor_violations(segments,minimum_s=MIN_INTERIOR_SEGMENT_S):
    """Indices of interior segments shorter than `minimum_s` (a path of fewer than three segments has none)."""
    d=[x['duration_s'] for x in segments]
    return [i for i in range(1,len(d)-1) if d[i]<minimum_s-1e-9]


def movej_cruise_c3(p,start,goal,minimum_duration_s=.1):
    """Synchronized C3 ramps + cruise within the SAME velocity/acceleration caps.

    Quintic velocity ramps have zero acceleration/jerk at both ends. Unlike a
    single S7 position law, long moves can cruise instead of accelerating and
    decelerating for their entire duration. Every quantized segment is checked.
    """
    start,goal=p.positions(start),p.positions(goal)
    delta=np.array(goal)-start
    moving=np.flatnonzero(np.abs(delta)>1e-12)
    if not len(moving):return movej_c3(p,start,goal,minimum_duration_s)
    velocity=min(p.velocity[i]/abs(delta[i]) for i in moving)
    acceleration=min(p.acceleration[i]/abs(delta[i]) for i in moving)
    velocity=min(velocity,math.sqrt(acceleration/1.875))*.999
    ramp=1.875*velocity/acceleration
    cruise=max(0.,1/velocity-ramp)
    stretch=max(1.,minimum_duration_s/(2*ramp+cruise))
    ramp_ticks=max(3,math.ceil(ramp*stretch/p.tick))
    cruise_ticks=math.ceil(cruise*stretch/p.tick)
    if 0<cruise_ticks<MIN_CRUISE_TICKS:
        # A cruise segment of a few ticks is shorter than the executor loop can service (it submits one segment per pass with a
        # two-deep PL queue): the queue ran dry at non-zero velocity and the task failed (2026-10-28, 15 ms segment between
        # two 250 ms ramps). Accel-limited moves have a ~2 ms cruise, which the old `<3` rule turned into exactly that 15 ms
        # segment. Lengthen each ramp by the whole cruise time instead: the distance is 1/(ramp+cruise) * (ramp+cruise) as before, the
        # peak velocity is unchanged and the acceleration is lower, so every cap still holds (half the cruise per ramp would
        # raise the peak velocity and break velocity-limited moves).
        ramp_ticks+=cruise_ticks;cruise_ticks=0
    ramp=ramp_ticks*p.tick;cruise=cruise_ticks*p.tick
    velocity=1/(ramp+cruise)
    up=np.array([0.,0.,0.,0.,2.5,-3.,1.,0.])*velocity*ramp
    down=-restrict(up[None,:].repeat(7,axis=0),1.,-1.)[0]
    down[0]+=1.
    laws=[(up,ramp_ticks)]
    if cruise_ticks:
        laws.append((np.array([up.sum(),velocity*cruise,0.,0.,0.,0.,0.,0.]),cruise_ticks))
    laws.append((down,ramp_ticks))
    result=[]
    for progress,ticks in laws:
        rows=delta[:,None]*progress;rows[:,0]+=start
        result.extend(split_rows(p,rows,ticks))
    if segment_floor_violations(result):raise ValueError('cruise path has a segment too short for the executor to chain')
    return result


def hermite_rows(q0,q1,v0,v1,a0,a1,j0,j1,seconds):
    q0,q1,v0,v1,a0,a1,j0,j1=map(lambda x:np.asarray(vector(np.asarray(x).tolist(),'C3 boundary')),(q0,q1,v0,v1,a0,a1,j0,j1))
    low=np.column_stack((q0,v0*seconds,a0*seconds**2/2,j0*seconds**3/6))
    rhs=np.array([q1-low.sum(axis=1),v1*seconds-(low[:,1]+2*low[:,2]+3*low[:,3]),
        a1*seconds**2-(2*low[:,2]+6*low[:,3]),j1*seconds**3-6*low[:,3]])
    A=np.array([[1,1,1,1],[4,5,6,7],[12,20,30,42],[24,60,120,210]],dtype=float)
    return np.column_stack((low,np.linalg.solve(A,rhs).T))


def continuity_metrics(segments,p,quantized=False):
    maximum=np.zeros(4);endpoint=np.zeros(3);previous=None
    for plan in segments:
        if quantized:
            words,_,_,_=septic_words(plan,p);c=np.array(words[:56]).reshape(7,8)/Q24
        else:c=np.asarray(plan['motor_coefficients_rad'])
        begin=np.array([[derivative(row,0.,k,plan['duration_s']) for row in c] for k in range(4)])
        end=np.array([[derivative(row,1.,k,plan['duration_s']) for row in c] for k in range(4)])
        if previous is not None:maximum=np.maximum(maximum,np.max(np.abs(begin-previous),axis=1))
        else:endpoint=np.max(np.abs(begin[1:]),axis=1)
        previous=end
    endpoint=np.maximum(endpoint,np.max(np.abs(previous[1:]),axis=1))
    return dict(maximum_boundary_jumps=maximum.tolist(),rest_endpoint_v_a_j=endpoint.tolist(),
        coordinates='motor radians, seconds',quantized=quantized)


def smooth_cartesian_c3(runtime,p,start7,kind='circle',delta=(0,0,.02),radius=.04,axes=(0,2),tcp_speed=.025):
    """Uniform IK nodes with S7 progress, shared q/v/a/j Hermite segments."""
    from continuous_trajectory import clamped_c2_segments
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
    count=max(24,min(192,math.ceil(length/.002)));progress=lambda u:float(poly.polyval(u,S7))
    qs=[q0];singular=[]
    for u in np.linspace(0,1,count+1)[1:]:
        sol=runtime.ik(pose_values(pin.SE3(R,position(progress(u)))),qs[-1][:6])
        if np.max(np.abs(sol-np.array(qs[-1][:6])))>.12:raise ValueError('IK branch jump')
        qs.append(sol.tolist()+[q0[6]])
        full=runtime.kin._to_pinocchio_q(sol.tolist())
        J=pin.computeFrameJacobian(runtime.kin.model,runtime.kin.data,full,runtime.kin._ee_frame_id,pin.LOCAL)[:,:6]
        smallest=float(np.linalg.svd(J,compute_uv=False)[-1]);singular.append(smallest)
        if smallest<.0005:raise ValueError('path too near singularity')
    duration=max(1.,2.1875*length/tcp_speed);q=np.asarray(qs)
    for attempt in range(40):
        ticks=math.ceil(duration/count/p.tick);h=ticks*p.tick;total=h*count
        try:
            base=clamped_c2_segments(p,qs,[ticks]*count)
            v=np.array([base[0]['start_velocity_rad_s']]+[s['end_velocity_rad_s'] for s in base])
            a=np.array([[derivative(base[0]['coefficients_rad'][i],0.,2,h) for i in range(7)]]+
                [[derivative(s['coefficients_rad'][i],1.,2,h) for i in range(7)] for s in base])
            j=np.array([[derivative(s['coefficients_rad'][i],0.,3,h) for i in range(7)] for s in base])
            shared_j=np.vstack((np.zeros(7),(j[:-1]+j[1:])/2,np.zeros(7)))
            a[0]=a[-1]=0;v[0]=v[-1]=0
            plans=[septic(p,hermite_rows(q[i],q[i+1],v[i],v[i+1],a[i],a[i+1],shared_j[i],shared_j[i+1],h),h) for i in range(count)]
            plans=[x for s in plans for x in split_rows(p,s['coefficients_rad'],s['ticks'])]
            break
        except ValueError as exc:last_error=exc;duration*=1.1
    else:raise ValueError('C3 time parameterization exhausted: '+str(last_error)) from last_error
    errors=[];orient=[];elapsed=0.
    for plan in plans:
        words,_,_,_=septic_words(plan,p);c=np.array(words[:56]).reshape(7,8)/Q24
        c*=np.array(p.direction)[:,None];c[:,0]-=np.array(p.offset)*p.direction
        for f in np.linspace(0,1,17):
            actual=runtime.fk([derivative(row,f) for row in c][:6])
            target=position(progress((elapsed+f*plan['duration_s'])/total))
            errors.append(float(np.linalg.norm(actual.translation-target)))
            orient.append(residual(actual,pin.SE3(R,actual.translation))[1])
        elapsed+=plan['duration_s']
    if max(errors)>.00025 or max(orient)>.002:raise ValueError('quantized C3 Cartesian sample error too large')
    return dict(kind=kind,segments=plans,start_rad=q0,goal_rad=plans[-1]['goal_rad'],
        model_id=runtime.model_id,profile_id=p.profile_id,duration_s=total,
        tcp_speed_requested_m_s=tcp_speed,radius_m=radius if kind=='circle' else None,axes=axes,
        delta_m=delta.tolist() if kind=='line' else None,origin_xyz_m=origin.tolist(),rotation=R.tolist(),
        sampled_position_error_m=max(errors),sampled_orientation_error_rad=max(orient),
        minimum_jacobian_singular_value=min(singular),continuous_error_bound_verified=False,
        time_law='SEPTIC_PROGRESS',joint_continuity='C3_REFERENCE',
        reference_continuity=continuity_metrics(plans,p),quantized_continuity=continuity_metrics(plans,p,True))


def trim_first_tick(p,plan,_validate=True):
    n=plan['ticks']
    if n<3:raise ValueError('C3 segment too short for endpoint deduplication')
    return septic(p,restrict(plan['coefficients_rad'],1/n,(n-1)/n),(n-1)*p.tick,_validate)
