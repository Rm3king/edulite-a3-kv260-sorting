"""Bounded per-joint GP lower-confidence search; deterministic, no hardware."""
import math
import numpy as np


def candidates(axis):
    kp=[40,60,80,100,120,160,200,240] if axis<3 else [30,45,60,80,100,120,150,180]
    kd=[.6,1,1.5,2,3,4,5] if axis<3 else [.3,.5,.8,1.2,2,3,4,5]
    # Exclude strongly underdamped corners of the rigid-body estimate before
    # physical exploration. Reflected inertia/servo dynamics remain unmodeled.
    mass=[.066,.117,.082,.0075,.0026,.00065][axis]
    return [(float(k),float(d)) for k in kp for d in kd if d>=.4*2*math.sqrt(k*mass)]


def suggest(history,axis):
    pool=candidates(axis);seen={(x['kp'],x['kd']) for x in history};pool=[p for p in pool if p not in seen]
    if not pool:raise ValueError('search pool exhausted')
    for pair in [(80.,4.),(120.,4.),(160. if axis<3 else 120.,5.),(80.,2.)]:
        if pair not in seen and pair in pool:return pair
    # GP on log parameters and log objective; fixed, documented noise floor.
    x=np.log(np.array([[r['kp'],r['kd']] for r in history]));z=np.log(np.array(pool))
    low=np.log(np.min(np.array(candidates(axis)),axis=0));high=np.log(np.max(np.array(candidates(axis)),axis=0));x=(x-low)/(high-low);z=(z-low)/(high-low)
    y=np.log(np.maximum([r['score'] for r in history],1e-8));mean=float(y.mean());std=max(.2,float(y.std()));y=(y-mean)/std
    best=None
    for length in (.25,.5,1.):
        def kernel(a,b):
            d=np.sqrt(np.sum((a[:,None,:]-b[None,:,:])**2,axis=2))/length
            return (1+np.sqrt(5)*d+5*d*d/3)*np.exp(-np.sqrt(5)*d)
        k=kernel(x,x)+np.eye(len(x))*.04**2
        try:l=np.linalg.cholesky(k)
        except np.linalg.LinAlgError:continue
        alpha=np.linalg.solve(l.T,np.linalg.solve(l,y));nll=.5*y@alpha+np.log(np.diag(l)).sum()
        cross=kernel(z,x);mu=cross@alpha;variance=np.maximum(0,1-np.sum(np.linalg.solve(l,cross.T)**2,axis=0));acquisition=mu-1.0*np.sqrt(variance)
        if best is None or nll<best[0]:best=(nll,pool[int(np.argmin(acquisition))])
    if best is None:raise ValueError('GP factorization failed')
    return best[1]
