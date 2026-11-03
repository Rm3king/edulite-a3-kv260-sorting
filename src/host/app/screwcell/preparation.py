"""Numerical preparation worker, isolated from the actuator owner process.

Imports existing runtime arithmetic; never creates a device or executes motion.
Model startup/cache IO are included in worker_total_s. PL audit is a later,
explicit board-side phase and cannot be simulated as hardware acceptance.
"""
import multiprocessing as mp
import queue
import sys
import threading
import time
from pathlib import Path


def compile_numeric(runtime,profile,tracking,capture,cache_path,workers=4,notify=lambda **x:None):
    begun=time.perf_counter();notify(stage='IMPORT_RUNTIME')
    sys.path.insert(0,str(runtime));sys.path.append(str(Path(runtime).parents[1]/'kv260_can_deploy'))
    from joint_trajectory import JointPlanner
    from motion_runtime import MotionRuntime
    from gravity_trajectory import GravityModel
    from control_benchmark import build
    from .offline_fields import FieldValidator
    from .motion_paths import prepare_branches,validate_restart_indices
    from .preparation_cache import EncodingCache
    notify(stage='MODEL_AND_CACHE')
    p=JointPlanner(profile);model=EncodingCache(GravityModel(MotionRuntime(),p,tracking));model.load(cache_path)
    load_s=time.perf_counter()-begun;validator=FieldValidator(Path(runtime)/'libnative_reference.so')
    notify(stage='MAIN_PATH',cache=model.stats());start=time.perf_counter()
    bank=build(capture,profile,tracking,'pick',phase_validator=validator,encoding_model=model);main_s=time.perf_counter()-start
    notify(stage='STOP_RESUME_BRANCHES',main_s=main_s,segments=len(bank['records']));start=time.perf_counter()
    branches,edges=prepare_branches(bank,p,model,validator=validator,workers=workers);branches_s=time.perf_counter()-start
    for branch in branches.values():validate_restart_indices(bank,branch)
    notify(stage='CACHE_SAVE',branches_s=branches_s,checkpoints=len(branches));start=time.perf_counter()
    cache_error=None
    try:model.save(cache_path)
    except OSError as e:cache_error=repr(e) # optional numerical cache IO cannot reset an actuator
    save_s=time.perf_counter()-start
    from .wire import digest
    prepared_digest=digest(dict(reference=bank['reference_sha256'],
        branches={str(i):{k:[list(r['words']) for r in b[k]] for k in ('brake','resume')} for i,b in branches.items()}))
    metrics=dict(model_and_cache_load_s=load_s,main_s=main_s,branches_s=branches_s,cache_save_s=save_s,
        worker_total_s=time.perf_counter()-begun,encoding_cache=model.stats(),cache_save_error=cache_error,
        checked_phases=validator.checked_phases,validation_implementation=validator.implementation,workers=workers,
        hardware_access=False,motor_commands_sent=0)
    notify(stage='NUMERICAL_READY',metrics=metrics)
    return dict(bank=bank,branches=branches,edges=edges,metrics=metrics,prepared_digest=prepared_digest)


def _run(results,kwargs):
    def notify(**event):results.put(dict(progress=event))
    try:results.put(dict(result=compile_numeric(**kwargs,notify=notify)))
    except Exception as e:
        import traceback
        results.put(dict(error=repr(e),traceback=traceback.format_exc()))


class NumericalPreparation:
    """The parent polls between STATUS/HEARTBEAT commands; no planning thread."""
    def __init__(self,**kwargs):
        ctx=mp.get_context('spawn');self.results=ctx.Queue();self.process=ctx.Process(target=_run,args=(self.results,kwargs));self.process.start();self.done=False
        # Queue.get also unpickles the large bank. Keep that pure CPU/IPC work
        # off the sole hardware owner's polling path, including child teardown.
        self.received=queue.Queue();self.reader=threading.Thread(target=self._receive,daemon=True);self.reader.start()
    def _receive(self):
        try:
            while True:
                try:value=self.results.get(timeout=.1)
                except queue.Empty:
                    if not self.process.is_alive():raise RuntimeError('NUMERICAL_PREPARATION_PROCESS_EXITED')
                    continue
                if 'result'in value or 'error'in value:
                    self.process.join(5);self.results.close();self.received.put(value);return
                self.received.put(value)
        except Exception as exc:
            import traceback
            self.received.put(dict(error=repr(exc),traceback=traceback.format_exc()))
    def poll(self,timeout=0.):
        try:value=self.received.get(timeout=timeout)
        except queue.Empty:return None
        if 'result'in value or 'error'in value:
            self.done=True
        return value
