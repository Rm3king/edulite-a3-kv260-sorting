"""Offline C3 speed ramps on an existing path. No device access on import.

Braking and restarting change path time, preserving the geometric reference.
All polynomial fitting, bounds and normalized q/v/gravity encoding finish with
motors disabled. The live owner only chooses among these immutable records.
"""
import bisect
import math


def restart_entries(checkpoint):
    """Stable main-path indices survive serialization/fork/spawn boundaries."""
    suffix=checkpoint['suffix'];indices=checkpoint['suffix_original_indices']
    if len(indices)!=len(suffix):raise ValueError('restart suffix index count')
    known=[i for i in indices if i is not None]
    if any(type(i) is not int or i<0 for i in known) or known!=sorted(set(known)):
        raise ValueError('restart suffix indices must increase')
    return [(r,None) for r in checkpoint['resume']]+list(zip(suffix,indices))


def validate_restart_indices(bank,checkpoint):
    for record,index in restart_entries(checkpoint):
        if index is not None and (index>=len(bank['records']) or record['words']!=bank['records'][index]['words']):
            raise ValueError('restart suffix does not match original segment index')


class Path:
    def __init__(self, plans):
        import numpy as np
        self.plans = plans
        self.derivatives=[]
        self.edges = [0.]
        for p in plans:
            self.edges.append(self.edges[-1]+p['duration_s'])
            rows=np.asarray(p['coefficients_rad'],dtype=float)
            derived=[]
            for order in range(4):
                c=rows.copy()
                for _ in range(order):c=c[:,1:]*np.arange(1,c.shape[1])
                derived.append(c)
            self.derivatives.append(derived)

    def at(self, seconds,position_only=False):
        import numpy as np
        i = min(len(self.plans)-1, max(0, bisect.bisect_right(self.edges, seconds+1e-10)-1))
        p = self.plans[i]
        u = min(1., max(0., (seconds-self.edges[i])/p['duration_s']))
        out=[]
        for order,c in enumerate(self.derivatives[i][:1] if position_only else self.derivatives[i]):
            value=c[:,-1].copy()
            for k in range(c.shape[1]-2,-1,-1):value=c[:,k]+value*u
            out.append(value/p['duration_s']**order)
        return np.asarray(out)

    def suffix(self, p, seconds):
        from septic_trajectory import restrict, septic
        i = min(len(self.plans)-1, max(0, bisect.bisect_right(self.edges, seconds+1e-9)-1))
        remaining = self.edges[i+1]-seconds
        result = []
        ticks = round(remaining/p.tick)
        if ticks >= 2:
            start = (seconds-self.edges[i])/self.plans[i]['duration_s']
            result.append(septic(p, restrict(self.plans[i]['coefficients_rad'], start, 1-start), ticks*p.tick))
        elif ticks == 1:
            raise ValueError('one-tick suffix; choose another ramp duration')
        return result+self.plans[i+1:]


def ramp_state(path, start, seconds, duration, restart=False,position_only=False):
    """Chain rule through jerk for a quintic velocity ramp."""
    u = min(1., max(0., seconds/duration))
    smooth = 10*u**3-15*u**4+6*u**5
    integral = 2.5*u**4-3*u**5+u**6
    w = smooth if restart else 1-smooth
    dw = (30*u**2-60*u**3+30*u**4)/duration
    ddw = (60*u-180*u**2+120*u**3)/duration**2
    if not restart:
        dw, ddw = -dw, -ddw
    s = start+duration*(integral if restart else u-integral)
    if position_only:return path.at(s,True)[0]
    q, v, a, j = path.at(s)
    return q, v*w, a*w*w+v*dw, j*w**3+3*a*w*dw+v*ddw


def ramp(path, p, start, duration, restart=False):
    from septic_trajectory import hermite_rows, septic, derivative
    import numpy as np
    count = math.ceil(duration/.1)
    edges = [round(i*duration/p.tick/count)*p.tick for i in range(count+1)]
    result = []
    fit_error = 0.
    for a, b in zip(edges, edges[1:]):
        x, y = ramp_state(path, start, a, duration, restart), ramp_state(path, start, b, duration, restart)
        c = hermite_rows(x[0], y[0], x[1], y[1], x[2], y[2], x[3], y[3], b-a)
        part = septic(p, c, b-a)
        for u in (.2, .4, .6, .8):
            wanted = ramp_state(path, start, a+(b-a)*u, duration, restart,True)
            fit_error = max(fit_error, float(np.max(np.abs(wanted-[derivative(row, u) for row in c]))))
        result.append(part)
    if fit_error > 2e-6:
        raise ValueError('path time-warp approximation exceeds 2 microradians')
    return result, fit_error


_fork_data=None


def _compile_forked(interval):
    bank,p,model,spacing,validator=_fork_data
    before=getattr(validator,'checked_phases',0)
    hits,misses=(model.hits,model.misses) if hasattr(model,'absorb') else (0,0)
    branches,_=prepare_branches(bank,p,model,spacing,validator,index_interval=interval)
    cache_delta=(model.hits-hits,model.misses-misses) if hasattr(model,'absorb') else (0,0)
    return branches,getattr(validator,'checked_phases',0)-before,cache_delta


def prepare_branches(bank, p, model, spacing=.4, validator=None,workers=1,index_interval=None):
    from gravity_trajectory import prepare_stream, validate_records
    from septic_trajectory import boundary
    check=validator or validate_records
    plans = [r['original_plan'] for r in bank['records']]
    path = Path(plans)
    if workers>1:
        # This branch is used only during disabled, CPU-only preparation.
        # Fork inherits the immutable Pinocchio model; children never receive a
        # device handle or call MMIO. Finish/join the pool before execution.
        import multiprocessing as mp
        from concurrent.futures import ProcessPoolExecutor
        global _fork_data
        # Natural zero-v/a/j endpoints reset spacing selection. Compile whole
        # intervals between them, preserving rejected-ramp selection exactly.
        intervals=[];first=0
        for i,end in enumerate(path.edges[1:]):
            rest=max(abs(x) for k in (1,2,3) for x in boundary(plans[i],k,True))<1e-7
            if rest:intervals.append((first,i+1));first=i+1
        if first<len(plans):raise ValueError('missing final rest checkpoint')
        if len(intervals)<2:return prepare_branches(bank,p,model,spacing,validator)
        _fork_data=(bank,p,model,spacing,validator)
        checkpoints={}
        try:
            with ProcessPoolExecutor(max_workers=min(4,int(workers)),mp_context=mp.get_context('fork')) as pool:
                for branches,phases,cache_delta in pool.map(_compile_forked,intervals,chunksize=1):
                    checkpoints.update(branches)
                    if validator is not None:validator.checked_phases+=phases
                    if hasattr(model,'absorb'):
                        model.hits+=cache_delta[0];model.misses+=cache_delta[1]
                        for b in branches.values():
                            for name in ('brake','resume','suffix'):model.absorb(b[name])
        finally:_fork_data=None
        if len(plans)-1 not in checkpoints:raise ValueError('missing final rest checkpoint')
        return dict(sorted(checkpoints.items())),path.edges
    checkpoints = {}
    last = path.edges[index_interval[0]] if index_interval is not None and index_interval[0]>0 else -1.
    for i, end in enumerate(path.edges[1:]):
        if index_interval is not None and not index_interval[0]<=i<index_interval[1]:continue
        rest = max(abs(x) for k in (1, 2, 3) for x in boundary(plans[i], k, True)) < 1e-7
        if end-last < spacing and not rest:
            continue
        if rest:
            checkpoints[i] = dict(brake=[], resume=[], suffix=bank['records'][i+1:],suffix_original_indices=list(range(i+1,len(plans))),
                                  end_s=end, stop_s=end, ramp_s=0., fit_error_rad=0.)
            last = end
            continue
        for duration in (1., 1.2, 1.5, 2.):
            if end+duration >= path.edges[-1]-.02:
                continue
            try:
                braking, err1 = ramp(path, p, end, duration)
                restart, err2 = ramp(path, p, end+duration/2, duration, True)
                # The first record follows the already transmitted knot, so
                # use prepare_stream's existing +5ms knot de-duplication.
                brake_task = dict(name='C3_brake', segments=braking, goal_rad=braking[-1]['goal_rad'])
                resume_task = dict(name='C3_restart', segments=restart, goal_rad=restart[-1]['goal_rad'])
                suffix = path.suffix(p, end+duration)
                # The prefix is already encoded/validated in the main bank.
                # Supply its boundary without compiling and discarding it again.
                records = prepare_stream(model, [brake_task,resume_task],preceding_plan=plans[i])
                # Validate q/v/tau at every actual HLS phase, including both
                # branches. Task endpoints need not be at rest at the prefix.
                proof = dict(bank['job'], tasks=[dict(name='branch',segments=braking+restart,
                                                    goal_rad=restart[-1]['goal_rad'])])
                # Structural C3 entry/exit are checked below; the existing
                # validator's task-rest rule is inapplicable to a moving join.
                check(records, dict(proof, tasks=[]), p, model)
                for left, right in zip([plans[i]]+braking+restart, braking+restart+suffix[:1]):
                    for order in range(4):
                        if max(abs(a-b) for a,b in zip(boundary(left,order,True),boundary(right,order))) > 2e-6:
                            raise ValueError('C3 branch continuity')
                suffix_records = [];suffix_indices=[]
                if suffix:
                    # Restriction only affects the first suffix polynomial;
                    # following original records can be reused unchanged.
                    tail_task = dict(name='resume_suffix',segments=[suffix[0]],goal_rad=suffix[0]['goal_rad'])
                    suffix_records = prepare_stream(model,[tail_task],preceding_plan=restart[-1])
                    idx = bisect.bisect_right(path.edges,end+duration+1e-9)-1
                    suffix_records += bank['records'][idx+1:]
                    suffix_indices=[None]+list(range(idx+1,len(plans)))
                    check(suffix_records[:1],dict(proof,tasks=[]),p,model)
                checkpoints[i] = dict(brake=records[:len(braking)],
                    resume=records[len(braking):], suffix=suffix_records,suffix_original_indices=suffix_indices,
                    end_s=end,stop_s=end+duration/2,ramp_s=duration,fit_error_rad=max(err1,err2))
                last = end
                break
            except ValueError:
                continue
    # Every pause request can at least reach the final zero-speed endpoint.
    if index_interval is None and len(plans)-1 not in checkpoints:
        raise ValueError('missing final rest checkpoint')
    return checkpoints, path.edges
