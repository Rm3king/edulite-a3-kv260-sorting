"""Offline positive FIR command smoothing; no motor I/O or modal claims.

All joints share delays/weights. MoveL geometry is measured again after shaping;
joint-space convolution is not assumed to preserve Cartesian straight lines.
"""
import bisect,copy,math
import numpy as np
from septic_trajectory import boundary,derivative,hermite_rows,septic


def settings(config):
    cfg=config.get('full_pick_experiment')
    if cfg is None:return None
    if not isinstance(cfg,dict) or cfg.get('method') not in ('reference_smoothing','inverse_dynamics','split_damping'):
        raise ValueError('unknown full-pick method')
    if cfg['method']=='reference_smoothing':
        ticks=cfg.get('spacing_ticks',25)
        if type(ticks) is not int or not 10<=ticks<=40:raise ValueError('FIR spacing requires 10..40 ticks')
    elif cfg['method']=='split_damping':
        fast=cfg.get('fast_joint3_kd');slow=config.get('pl_joint3_damping_nm_s_rad',0)
        if type(fast) not in (int,float) or not math.isfinite(fast) or not 0<fast<5 or not 0<slow<=2 or abs(fast+slow-config['kd'][2])>1e-9 or config.get('inverse_dynamics_scale',0):
            raise ValueError('split damping must preserve nominal low-frequency Kd without dynamics')
    elif not 0<config.get('inverse_dynamics_scale',0)<=.6:
        raise ValueError('dynamics comparison requires scale in (0,.6]')
    if (cfg['method']!='split_damping' and config.get('pl_joint3_damping_nm_s_rad',0)) or any(config.get('friction_coulomb_nm',[0]*6)):
        raise ValueError('method comparison must not combine friction/PL damping')
    return cfg


def smooth_task(p,task,spacing_ticks=25):
    plans=task['segments'];edges=np.cumsum([0]+[x['ticks'] for x in plans]).tolist()
    start=boundary(plans[0]);end=boundary(plans[-1],end=True)
    def source(tick,order):
        if tick<=0:return np.array(start) if order==0 else np.zeros(7)
        if tick>=edges[-1]:return np.array(end) if order==0 else np.zeros(7)
        k=bisect.bisect_right(edges,tick)-1;s=plans[k]
        u=(tick-edges[k])/s['ticks']
        return np.array([derivative(row,u,order,s['duration_s']) for row in s['coefficients_rad']])
    def state(tick,order):
        return sum(weight*source(tick-delay,order) for weight,delay in ((.25,0),(.5,spacing_ticks),(.25,2*spacing_ticks)))
    total=edges[-1]+2*spacing_ticks
    for maximum in (40,20,10,5,2):
        count=math.ceil(total/maximum);knots=[round(i*total/count) for i in range(count+1)]
        out=[];error=np.zeros(3)
        for a,b in zip(knots,knots[1:]):
            seconds=(b-a)*p.tick
            rows=hermite_rows(state(a,0),state(b,0),state(a,1),state(b,1),state(a,2),state(b,2),state(a,3),state(b,3),seconds)
            plan=septic(p,rows,seconds)
            for tick in range(a,b+1):
                u=(tick-a)/(b-a)
                for order in range(3):
                    actual=np.array([derivative(row,u,order,seconds) for row in rows])
                    error[order]=max(error[order],float(np.max(abs(actual-state(tick,order)))))
            out.append(plan)
        if np.all(error<np.array([1e-6,1e-5,5e-4])):break
    else:raise ValueError('FIR Hermite approximation failed independent 200Hz check')
    result=copy.deepcopy(task);result['segments']=out
    result['reference_smoothing']=dict(weights=[.25,.5,.25],spacing_ticks=spacing_ticks,added_seconds=2*spacing_ticks*p.tick,maximum_fit_error_q_v_a=error.tolist())
    return result


def geometry_audit(runtime,p,original,shaped):
    worst_position=0.;worst_orientation=0.
    from motion_runtime import residual
    from septic_trajectory import septic_words,Q24
    for old,new in zip(original,shaped):
        if old.get('kind')!='MoveL':continue
        T0=runtime.fk(old['segments'][0]['start_rad'][:6]);T1=runtime.fk(old['goal_rad'][:6])
        line=T1.translation-T0.translation;length2=float(line@line)
        for plan in new['segments']:
            w=septic_words(plan,p)[0];c=np.array(w[:56]).reshape(7,8)/Q24
            c*=np.array(p.direction)[:,None];c[:,0]-=np.array(p.offset)*p.direction
            for tick in range(plan['ticks']+1):
                q=[derivative(row,tick/plan['ticks']) for row in c]
                q=[max(a,min(b,x)) for x,a,b in zip(q,p.lower,p.upper)]
                T=runtime.fk(q[:6]);u=float((T.translation-T0.translation)@line/length2)
                distance=np.linalg.norm(T.translation-(T0.translation+np.clip(u,0,1)*line))
                worst_position=max(worst_position,float(distance));worst_orientation=max(worst_orientation,residual(T,T0)[1])
    if worst_position>.00025 or worst_orientation>.002:raise ValueError('FIR MoveL geometry exceeded 0.25mm/2mrad')
    return dict(sampled_moveL_line_error_m=worst_position,sampled_moveL_orientation_error_rad=worst_orientation,all_5ms_phases_checked=True,continuous_bound_verified=False)


def build_pair(model,tasks):
    from tuning_worker import encode
    cfg=settings(model.config)
    if cfg is None:raise ValueError('method configuration required')
    if cfg['method']=='reference_smoothing':
        if model.dynamic_scale:raise ValueError('FIR comparison requires dynamics off')
        candidate=[smooth_task(model.p,t,cfg.get('spacing_ticks',25)) for t in tasks]
        audit=geometry_audit(model.r,model.p,tasks,candidate)
        audit.update(method=cfg['method'],tasks=[t['reference_smoothing'] for t in candidate])
        return encode(model,tasks),encode(model,candidate),audit
    if cfg['method']=='split_damping':
        base=copy.copy(model);base.pl_damping_q8=0
        return encode(base,tasks),encode(model,tasks),dict(method=cfg['method'],fast_joint3_kd=cfg['fast_joint3_kd'],slow_joint3_damping_nm_s_rad=model.pl_damping_q8/256,nominal_low_frequency_damping=model.kd[2],correction_cap_nm=.30,filtered_damping_is_delayed_and_capped=True)
    base=copy.copy(model);base.dynamic_scale=0.
    return encode(base,tasks),encode(model,tasks),dict(method=cfg['method'],candidate_inverse_dynamics_scale=model.dynamic_scale)
