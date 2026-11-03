"""Offline compile benchmark from a recorded capture; no hardware IO.

Includes main/stop/restart/suffix encoding and native field checks. Excludes
live feedback capture and the PL FK audit; never label this as full RPC time.
"""
import argparse
import gzip
import hashlib
import importlib.util
import json
import sys
import time
from pathlib import Path

def run(a):
    sys.path.insert(0,str(Path(a.runtime).resolve()))
    if a.oracle:sys.path.insert(0,str(Path(a.oracle).resolve()))
    from control_benchmark import build
    from joint_trajectory import JointPlanner
    from gravity_trajectory import GravityModel
    from motion_runtime import MotionRuntime
    from screwcell.offline_fields import FieldValidator
    if a.oracle:
        spec=importlib.util.spec_from_file_location('original_motion_paths',Path(a.oracle)/'motion_paths.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);prepare=module.prepare_branches
    else:
        from screwcell.motion_paths import prepare_branches as prepare
    capture=json.loads(Path(a.capture).read_text());p=JointPlanner(a.profile);m=GravityModel(MotionRuntime(),p,a.tracking)
    cache_load_s=cache_save_s=0.
    if a.encoding_cache:
        from screwcell.preparation_cache import EncodingCache
        m=EncodingCache(m);started=time.perf_counter();m.load(a.encoding_cache);cache_load_s=time.perf_counter()-started
    validator=FieldValidator(a.native);begin=time.perf_counter()
    bank=build(capture,a.profile,a.tracking,'pick',phase_validator=validator,**({'encoding_model':m} if a.encoding_cache else {}));main=time.perf_counter()-begin
    start=time.perf_counter();branches,_=prepare(bank,p,m,validator=validator,**({'workers':a.workers} if not a.oracle else {}));branch=time.perf_counter()-start
    words=dict(main=[list(r['words']) for r in bank['records']],branches={str(i):{k:[list(r['words']) for r in b[k]] for k in ('brake','resume','suffix')} for i,b in branches.items()})
    raw=json.dumps(words,sort_keys=True,separators=(',',':')).encode()
    if a.encoding_cache:
        started=time.perf_counter();m.save(a.encoding_cache);cache_save_s=time.perf_counter()-started
    out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    with gzip.open(out/'words.json.gz','wb') as f:f.write(raw)
    report=dict(status='PASS_OFFLINE_ENCODED_PHASE_CHECKS',main_s=main,branches_s=branch,total_s=main+branch,
                checkpoints=len(branches),main_segments=len(bank['records']),checked_phases=validator.checked_phases,workers=a.workers,
                word_digest=hashlib.sha256(raw).hexdigest(),motor_commands_sent=0,hardware_access=False,
                max_time_warp_sample_error_rad=max(b['fit_error_rad'] for b in branches.values()),
                validation_implementation=validator.implementation,encoding_cache=m.stats() if a.encoding_cache else None,
                cache_load_s=cache_load_s,cache_save_s=cache_save_s,offline_service_total_s=main+branch+cache_load_s+cache_save_s,
                scope='Recorded capture, full offline reference compilation; excludes runtime startup, live capture, PL FK audit and RPC')
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser()
    for n in ['runtime','capture','profile','tracking','native','output']:p.add_argument('--'+n,required=True)
    p.add_argument('--oracle');p.add_argument('--workers',type=int,default=1);p.add_argument('--encoding-cache');run(p.parse_args())
