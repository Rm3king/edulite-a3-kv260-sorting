"""Bounded exact-input numerical cache. No device, request or execution cache.

Only polynomial encoding is reusable. Fresh path construction, metadata/C3
checks and exhaustive independent integer phase validation still run. Entries
contain no enable/session/plan authorization. Files are JSON, never pickle.
"""
from collections import OrderedDict
import gzip
import hashlib
import json
import math
import os
import sys
import platform
import zlib
from pathlib import Path


def canonical(value):return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()

def implementation_id(runtime):
    root=Path(runtime)
    names=['gravity_trajectory.py','septic_trajectory.py','joint_trajectory.py','motion_runtime.py']
    sources={n:hashlib.sha256((root/n).read_bytes()).hexdigest() for n in names}
    sources['preparation_cache.py']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    return hashlib.sha256(canonical(sources)).hexdigest()


class EncodingCache:
    def __init__(self,model,limit=4096):
        if type(model).__name__!='GravityModel':raise ValueError('CACHE_REQUIRES_BASE_GRAVITY_MODEL')
        self.model=model;self.limit=limit;self.entries=OrderedDict();self.hits=0;self.misses=0;self.load_status='NOT_LOADED'
        import gravity_trajectory
        self.implementation=implementation_id(Path(gravity_trajectory.__file__).parent)
    def __getattr__(self,name):
        # Avoid recursion if multiprocessing inspects a partly restored object.
        if name=='model':raise AttributeError(name)
        return getattr(self.model,name)
    def namespace(self):
        m=self.model;p=m.p
        import numpy as np
        import pinocchio as pin
        return dict(implementation=self.implementation,robot_model=m.r.model_id,tracking=m.id,
            numeric_backend=dict(python=list(sys.version_info[:2]),numpy=np.__version__,pinocchio=pin.__version__,byteorder=sys.byteorder,machine=platform.machine()),
            motion=p.profile_id,parameters={name:list(map(float,getattr(m,name))) for name in ['kp','kd','scale','caps','vmax','tmax','friction_coulomb']},
            dynamic_scale=m.dynamic_scale,friction_speed=m.friction_speed,has_friction=m.has_friction,damping=m.pl_damping_q8,
            planner={name:list(map(float,getattr(p,name))) for name in ['lower','upper','velocity','acceleration','direction','offset']},tick=p.tick,max_duration=p.max_duration)
    def key(self,plan,alpha,alpha_start):
        # Exact JSON float representation: no angle rounding or fuzzy origin.
        return hashlib.sha256(canonical(dict(namespace=self.namespace(),plan=plan,alpha=list(alpha),alpha_start=alpha_start))).hexdigest()
    def keep(self,key,value):
        self.entries[key]=value;self.entries.move_to_end(key)
        while len(self.entries)>self.limit:self.entries.popitem(last=False)
    def encode(self,plan,alpha=(1.,1.),alpha_start=0.,batch_evaluation=True):
        if not batch_evaluation:return self.model.encode(plan,alpha,alpha_start,False)
        key=self.key(plan,alpha,alpha_start)
        value=self.entries.get(key)
        if value is None:
            self.misses+=1;record=self.model.encode(plan,alpha,alpha_start)
            value=dict(words=list(record['words']),sampled_gravity_fit_error_nm=record['sampled_gravity_fit_error_nm'],inverse_dynamics_scale=record['inverse_dynamics_scale'])
            self.keep(key,value)
        else:self.hits+=1;self.entries.move_to_end(key)
        return dict(plan=plan,words=tuple(value['words']),gravity_alpha=list(alpha),encoding_cache_key=key,
            sampled_gravity_fit_error_nm=value['sampled_gravity_fit_error_nm'],inverse_dynamics_scale=value['inverse_dynamics_scale'])
    def absorb(self,records):
        # Fork workers return actual freshly validated records. Retain their
        # arithmetic artifacts in the parent's bounded cache for the next task.
        for r in records:
            if 'encoding_cache_key' in r:
                self.keep(r['encoding_cache_key'],dict(words=list(r['words']),sampled_gravity_fit_error_nm=r['sampled_gravity_fit_error_nm'],inverse_dynamics_scale=r['inverse_dynamics_scale']))
    def save(self,path):
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        payload=dict(schema='screwcell.encoding_cache.v1',implementation=self.implementation,entries=list(self.entries.items()))
        raw=canonical(payload);envelope=canonical(dict(sha256=hashlib.sha256(raw).hexdigest(),payload=payload));temporary=path.with_name(path.name+'.tmp')
        with temporary.open('wb') as f:f.write(gzip.compress(envelope,compresslevel=1,mtime=0));f.flush();os.fsync(f.fileno())
        os.replace(temporary,path)
    def load(self,path):
        try:
            path=Path(path)
            if path.stat().st_size>16*1024*1024:raise ValueError('CACHE_TOO_LARGE')
            with gzip.open(path,'rb') as f:raw=f.read(32*1024*1024+1)
            if len(raw)>32*1024*1024:raise ValueError('CACHE_EXPANSION_LIMIT')
            envelope=json.loads(raw);payload=envelope['payload']
            if hashlib.sha256(canonical(payload)).hexdigest()!=envelope['sha256']:raise ValueError('CACHE_DIGEST')
            if payload['schema']!='screwcell.encoding_cache.v1' or payload['implementation']!=self.implementation:raise ValueError('CACHE_IMPLEMENTATION_CHANGED')
            pending=[]
            for key,value in payload['entries']:
                if type(key)is not str or len(key)!=64 or any(x not in '0123456789abcdef' for x in key):raise ValueError('CACHE_KEY')
                w=value['words']
                if type(w)is not list or len(w)!=220 or any(type(x)is not int or not -(1<<31)<=x<1<<32 for x in w):raise ValueError('CACHE_WORDS')
                if not math.isfinite(value['sampled_gravity_fit_error_nm']) or not 0<=value['sampled_gravity_fit_error_nm']<=1e-4:raise ValueError('CACHE_FIT')
                if not 0<=value['inverse_dynamics_scale']<=1:raise ValueError('CACHE_DYNAMICS')
                pending.append((key,value))
            for key,value in pending[-self.limit:]:self.keep(key,value)
            self.load_status='LOADED'
        except FileNotFoundError:self.load_status='COLD_START'
        except (OSError,ValueError,KeyError,TypeError,EOFError,zlib.error) as e:self.load_status='MISS_'+type(e).__name__
        return self.load_status
    def stats(self):return dict(hits=self.hits,misses=self.misses,entries=len(self.entries),load_status=self.load_status)
