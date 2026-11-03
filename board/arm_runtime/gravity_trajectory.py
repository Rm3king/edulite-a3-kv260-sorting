#!/usr/bin/python3
"""Precompute normalized q/v/gravity polynomials for FPGA Type1 output.

First six channels use normalized protocol fields; gripper q remains raw motor
radians for CSP. Every polynomial is checked before a hardware lease is opened.
Gravity is evaluated on the desired continuous path, like the vendor's normal
tracking branch, without adding a new EMA delay to the existing C3 reference.
"""
import hashlib,json,math
from pathlib import Path
import numpy as np
from numpy.polynomial import polynomial as poly
from joint_trajectory import Q24
from septic_trajectory import derivative,extrema,septic_words,trim_first_tick,septic,S7,boundary,quantize_boundary,bernstein_bounds,bernstein_integer_bounds


def measured_start_reference(p,motor):
    """Explicit measurement adapter, never a widened command interval.

    A disabled joint may read a fraction of one Type2 position code past its
    calibrated zero. Preserve that measurement; put only the reference at the
    legal endpoint. Larger excursions still require physical correction.
    """
    tolerance=25.14/65535
    robot=[(a-o)/d for a,o,d in zip(motor,p.offset,p.direction)]
    reference=[]
    for q,lo,hi in zip(robot,p.lower,p.upper):
        target=min(hi,max(lo,q))
        if not math.isfinite(q) or abs(target-q)>tolerance:raise ValueError('measured start exceeds one protocol position code outside joint interval')
        reference.append(target)
    p.positions(reference)
    return reference,[a-b for a,b in zip(p.to_motor(reference),motor)]


def encoded_interval(job,p,model,i):
    def interval(lo,hi,scale,offset=0.):
        a=math.ceil((lo-offset)*scale);b=math.floor((hi-offset)*scale)
        if not 0<=a<=b<=65535:raise ValueError('invalid encoded physical interval')
        return a|(b<<16)
    q=interval(job['motor_lower_rad'][i]+12.57,job['motor_upper_rad'][i]+12.57,65535/25.14)
    vmax=model.vmax[i];tmax=model.tmax[i]
    v=interval(vmax-p.velocity[i],vmax+p.velocity[i],65535/(2*vmax))
    t=interval(tmax-model.caps[i],tmax+model.caps[i],65535/(2*tmax))
    gains=math.floor(model.guard_kp[i]*65535/500)|(math.floor(model.guard_kd[i]*65535/5)<<16)
    return q,v,t,gains


def rounded_div(value,denominator):
    q,r=divmod(value,denominator)
    return q+int(2*r>denominator or (2*r==denominator and q&1))


def fixed_sample(row,phase):
    """Independent integer reference for HLS 40-bit fractional Horner."""
    y=int(row[-1])<<16
    for c in reversed(row[:-1]):y=rounded_div(y*phase,Q24)+(int(c)<<16)
    return rounded_div(y,1<<16)


def validate_records(records,job,p,model,phase_validator=None):
    if not records:raise ValueError('empty gravity stream')
    bounds=[encoded_interval(job,p,model,i) for i in range(6)]
    previous=None;previous_alpha=1.
    for task in job['tasks']:
        plans=task['segments']
        alpha=task.get('gravity_alpha',[1.,1.])
        if len(alpha)!=2 or abs(alpha[0]-previous_alpha)>1e-12:raise ValueError('gravity ratio discontinuity')
        previous_alpha=alpha[1]
        if not plans or max(abs(a-b) for a,b in zip(plans[-1]['goal_rad'],task['goal_rad']))>2e-6:raise ValueError('gravity task goal mismatch')
        if any(abs(x)>1e-7 for k in (1,2,3) for x in boundary(plans[0],k)+boundary(plans[-1],k,True)):
            raise ValueError('gravity tasks must start/end at rest through jerk')
        for plan in plans:
            if max(abs(a-b) for a,b in zip(boundary(plan),plan['start_rad']))>1e-9 or max(abs(a-b) for a,b in zip(boundary(plan,end=True),plan['goal_rad']))>1e-9:
                raise ValueError('gravity endpoint metadata mismatch')
            if previous is not None:
                for order,tol in ((0,2e-6),(1,1e-7),(2,1e-7),(3,1e-7)):
                    if max(abs(a-b) for a,b in zip(boundary(previous,order,True),boundary(plan,order)))>tol:raise ValueError('gravity reference discontinuity')
            previous=plan
    if phase_validator is not None:
        return phase_validator(records,job,p,model)
    for r in records:
        w=r['words'];n=w[217]
        if len(w)!=220:raise ValueError('gravity staging word count')
        if not 0<=w[216]<=512:raise ValueError('PL damping metadata exceeds reviewed Q8 gain')
        # Exhaust the actual 200 Hz fixed-point phases, not just floating roots.
        for tick in range(n+1):
            phase=(tick*Q24)//n if tick<n else Q24
            for field in range(3):
                for i in range(7):
                    start=field*56+i*8;value=fixed_sample(w[start:start+8],phase)
                    if not w[168+field*14+i]<=value<=w[175+field*14+i]:raise ValueError('HLS sample outside atomic bounds')
                    if i<6:
                        code=(value*65535+(1<<23))>>24;lo=bounds[i][field]&65535;hi=bounds[i][field]>>16
                        if not lo<=code<=hi:raise ValueError(f'encoded field {field} axis {i+1} exceeds independent guard')
                    elif field==0 and not job['motor_lower_rad'][6]<=value/Q24<=job['motor_upper_rad'][6]:raise ValueError('gripper sample outside reviewed guard')
        if max(map(abs,r['end_velocity']))<1e-7:
            if any(((fixed_sample(w[56+i*8:64+i*8],Q24)*65535+(1<<23))>>24)!=32768 for i in range(6)):
                raise ValueError('final normalized velocity not rest code')
    return True

class GravityModel:
    def __init__(self,runtime,p,path=None):
        self.r,self.p=runtime,p
        raw=Path(path or Path(__file__).with_name('gravity_profile.json')).read_bytes()
        self.config=json.loads(raw);self.id=hashlib.sha256(raw).hexdigest()
        self.kp=np.array(self.config['kp'],float);self.kd=np.array(self.config['kd'],float)
        self.scale=np.array(self.config['gravity_joint_scale'],float)
        self.caps=np.array(self.config['motor_torque_limit_nm'],float)
        if any(x.shape!=(6,) or not np.isfinite(x).all() for x in (self.kp,self.kd,self.scale,self.caps)):raise ValueError('six finite gravity parameters required')
        if np.any(self.kp<=0) or np.any(self.kp>500) or np.any(self.kd<0) or np.any(self.kd>5):raise ValueError('gravity tracking gains invalid')
        self.guard_kp=np.array(self.config.get('guard_kp',self.kp),float)
        self.guard_kd=np.array(self.config.get('guard_kd',self.kd),float)
        if self.guard_kp.shape!=(6,) or self.guard_kd.shape!=(6,) or not np.isfinite(self.guard_kp).all() or not np.isfinite(self.guard_kd).all() or np.any(self.guard_kp<self.kp) or np.any(self.guard_kp>500) or np.any(self.guard_kd<self.kd) or np.any(self.guard_kd>5):
            raise ValueError('gain guard must cover configured gains within protocol')
        if np.any(self.caps<=0) or np.any(self.caps>[14,14,14,6,6,6]):raise ValueError('motor torque cap invalid')
        self.vmax=np.array([33,33,33,50,50,50.]);self.tmax=np.array([14,14,14,6,6,6.])
        self.dynamic_scale=self.config.get('inverse_dynamics_scale',0.)
        if type(self.dynamic_scale) not in (int,float) or not math.isfinite(self.dynamic_scale) or not 0<=self.dynamic_scale<=1:
            raise ValueError('inverse dynamics scale must be finite in [0,1]')
        self.friction_coulomb=np.asarray(self.config.get('friction_coulomb_nm',[0.]*6),float)
        self.friction_speed=self.config.get('friction_velocity_scale_rad_s',.05)
        if self.friction_coulomb.shape!=(6,) or not np.isfinite(self.friction_coulomb).all() or np.any(self.friction_coulomb<0) or np.any(self.friction_coulomb>self.caps):
            raise ValueError('six nonnegative finite friction coefficients within torque caps required')
        if type(self.friction_speed) not in (int,float) or not math.isfinite(self.friction_speed) or self.friction_speed<=0:
            raise ValueError('friction velocity scale must be positive and finite')
        self.has_friction=bool(np.any(self.friction_coulomb))
        damping=self.config.get('pl_joint3_damping_nm_s_rad',0.)
        if type(damping) not in (int,float) or not math.isfinite(damping) or not 0<=damping<=2:
            raise ValueError('PL joint3 damping must be finite in [0,2] Nm*s/rad')
        self.pl_damping_q8=round(damping*256)

    def gravity(self,robot7):
        q=self.p.positions(list(map(float,robot7)))
        return np.array(self.r.kin.compute_gravity(q[:6])[:6])*np.array(self.p.direction[:6])*self.scale

    def feedforward(self,robot7,velocity7,acceleration7):
        """Desired-state gravity/dynamics and optional smooth partial friction.

        Model joint 7 is fixed at zero by the six-axis SDK adapter. A changing
        gripper opening must never be interpreted as that fictitious joint.
        Directions apply to torque after robot-coordinate RNEA, preserving power.
        """
        q=self.p.positions(list(map(float,robot7)))
        v=np.asarray(velocity7,dtype=float);a=np.asarray(acceleration7,dtype=float)
        if v.shape!=(7,) or a.shape!=(7,) or not np.isfinite(v).all() or not np.isfinite(a).all():
            raise ValueError('seven finite velocity/acceleration values required')
        g=self.gravity(q)
        if self.dynamic_scale:
            gv=np.array(self.r.kin.compute_gravity(q[:6])[:6])
            total=np.array(self.r.kin.inverse_dynamics(q[:6],v[:6].tolist()+[0.],a[:6].tolist()+[0.])[:6])
            g=g+self.dynamic_scale*(total-gv)*np.array(self.p.direction[:6])
        if self.has_friction:
            # Desired motor velocity, not noisy measured velocity; smooth and
            # exactly zero at rest. This is partial compensation, not a Kt fit.
            g=g+self.friction_coulomb*np.tanh(v[:6]*self.p.direction[:6]/self.friction_speed)
        return g

    def encode(self,plan,alpha=(1.,1.),alpha_start=0.,batch_evaluation=True):
        septic_words(plan,self.p)
        if len(alpha)!=2 or not all(type(x) in (int,float) and math.isfinite(x) and 0<=x<=1 for x in alpha):raise ValueError('gravity ratio in [0,1] required')
        robot=np.array(plan['coefficients_rad']);motor=np.array(plan['motor_coefficients_rad']);duration=plan['duration_s']
        if not 0<=alpha_start<1:raise ValueError('invalid ramp phase restriction')
        q=motor.copy();v=np.zeros((7,8));tau=np.zeros((7,8));ratio=np.zeros(8);ratio[0]=alpha[0];ratio+=(alpha[1]-alpha[0])*S7
        ratio=np.array([sum(ratio[j]*math.comb(j,k)*alpha_start**(j-k)*(1-alpha_start)**k for j in range(k,8)) for k in range(8)])
        # Gauss-Lobatto nodes include both endpoints; degree-seven fit error
        # is checked independently at 33 additional uniform phase values.
        nodes=(1-np.cos(np.linspace(0,np.pi,8)))/2
        phases=np.r_[nodes,np.linspace(0,1,33)]
        angle_samples={}
        if batch_evaluation:
            # Same scalar-double Horner order, vectorized across all joints and
            # sample phases. No reassociated BLAS dot product or changed fit.
            values=np.broadcast_to(robot[:,-1,None],(7,len(phases))).copy()
            for k in range(6,-1,-1):values=robot[:,k,None]+values*phases
            angle_samples={float(u):values[:,i].tolist() for i,u in enumerate(phases)}
        def desired_torque(u):
            angles=angle_samples[float(u)] if batch_evaluation else [derivative(row,u) for row in robot]
            # alpha scales gravity only, keeping the legacy gravity A/B meaning.
            g=self.gravity(angles)
            if self.dynamic_scale or self.has_friction:
                velocity=[derivative(row,u,1)/duration for row in robot]
                acceleration=[derivative(row,u,2)/duration**2 for row in robot]
                ff=self.feedforward(angles,velocity,acceleration)
            else:ff=g
            return ff-g+g*poly.polyval(u,ratio)
        desired=[]
        for u in nodes:
            desired.append(desired_torque(u))
        torque_coeff=poly.polyfit(nodes,np.array(desired),7).T
        error=[]
        for u in np.linspace(0,1,33):
            expected=desired_torque(u)
            got=np.array([derivative(row,u) for row in torque_coeff]);error.append(float(np.max(np.abs(got-expected))))
        if max(error)>1e-4:raise ValueError('gravity fit error exceeds 0.1 mNm')
        for i in range(6):
            hull=bernstein_bounds(torque_coeff[i])
            peak=max(map(abs,hull))
            if peak>self.caps[i]:peak=max(map(abs,extrema(torque_coeff[i])))
            if peak>self.caps[i]:raise ValueError('gravity exceeds torque cap')
            q[i]/=25.14;q[i,0]+=12.57/25.14
            v[i,:7]=poly.polyder(motor[i])/duration/(2*self.vmax[i]);v[i,0]+=.5
            tau[i]=torque_coeff[i]/(2*self.tmax[i]);tau[i,0]+=.5
        v[6,0]=tau[6,0]=.5
        fields=[q,v,tau];words=[];bound_words=[]
        for field,c in enumerate(fields):
            # Preserve both endpoint values in integer arithmetic. Independent
            # coefficient rounding can make a rest velocity 0.5 - one Q24 LSB,
            # which rounds to protocol code 32767 instead of the rest code 32768.
            quant=np.array([quantize_boundary(row,-(1<<31),(1<<31)-1) for row in c],dtype=np.int64)
            if np.any(quant<-(1<<31)) or np.any(quant>=(1<<31)):raise ValueError('normalized coefficient overflow')
            for i,row in enumerate(quant):
                if field==0 and i==6:
                    lower,upper=self.p.lower[6],self.p.upper[6]
                elif field==0:
                    lower,upper=sorted(((self.p.lower[i]*self.p.direction[i]+self.p.offset[i]+12.57)/25.14,
                                        (self.p.upper[i]*self.p.direction[i]+self.p.offset[i]+12.57)/25.14))
                elif field==1:lower,upper=(.5-self.p.velocity[i]/(2*self.vmax[i]),.5+self.p.velocity[i]/(2*self.vmax[i])) if i<6 else (0.,1.)
                else:lower,upper=(.5-self.caps[i]/(2*self.tmax[i]),.5+self.caps[i]/(2*self.tmax[i])) if i<6 else (0.,1.)
                l,h=math.ceil(lower*Q24),math.floor(upper*Q24)
                # Exact integer convex-hull certificate. Only ambiguous rows
                # need roots; the physical interval and tolerance are unchanged.
                mn,mx,den=bernstein_integer_bounds(row)
                if mn>=(l-1)*den and mx<=(h+1)*den:continue
                lo,hi=extrema(row)
                if lo<l-1 or hi>h+1:raise ValueError('normalized polynomial exceeds physical interval')
            words.extend(int(x) for x in quant.ravel())
            # HLS evaluates protocol-normalized fields atomically in [0,1].
            # Physical encoded bounds are repeated independently by can_engine.
            bounds=[(0,Q24)]*7
            if field==0:bounds[6]=(math.ceil(self.p.lower[6]*Q24),math.floor(self.p.upper[6]*Q24))
            bound_words += [b[0] for b in bounds]+[b[1] for b in bounds]
        words+=bound_words
        words += [(math.floor(k*65535/500)<<16)|math.floor(d*65535/5) for k,d in zip(self.kp,self.kd)]+[self.pl_damping_q8]
        n=plan['ticks'];step,rem=divmod(Q24,n);words += [n,step,rem]
        if len(words)!=220:raise AssertionError('MIT staging layout')
        return dict(plan=plan,words=tuple(words),gravity_alpha=list(alpha),sampled_gravity_fit_error_nm=max(error),inverse_dynamics_scale=self.dynamic_scale)


def prepare_stream(model,tasks,preceding_plan=None):
    """No live planning. Exact next sample at +5 ms, without duplicate knots."""
    records=[];previous=preceding_plan['goal_rad'] if preceding_plan is not None else None
    for task in tasks:
        alpha=task.get('gravity_alpha',[1.,1.])
        for plan in task['segments']:
            # Ramp tests use one constant-pose segment. Moving tasks use fixed
            # gravity ratio, so truncation never changes the intended ramp.
            if alpha[0]!=alpha[1] and len(task['segments'])!=1:raise ValueError('ramp must be a single reviewed constant-pose segment')
            if alpha[0]!=alpha[1] and np.max(np.abs(np.array(plan['coefficients_rad'])[:,1:]))>1e-12:raise ValueError('gravity ramp requires fixed pose')
            if previous is not None and np.max(np.abs(np.array(plan['start_rad'])-previous))>2e-6:raise ValueError('MIT path discontinuity')
            joined=bool(records) or preceding_plan is not None
            encoded=trim_first_tick(model.p,plan) if joined else plan
            record=model.encode(encoded,alpha,1/plan['ticks'] if joined else 0.)
            record.update(original_plan=plan,task=task['name'],
                ends=tuple(round(x*Q24) for x in model.p.to_motor(plan['goal_rad'])),
                end_velocity=tuple(boundary(plan,1,True,True)))
            records.append(record);previous=plan['goal_rad']
    return records


def hold_segment(p,q,seconds=2.):
    c=np.zeros((7,8));c[:,0]=q;return septic(p,c,round(seconds/p.tick)*p.tick)
