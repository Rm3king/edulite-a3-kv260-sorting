"""Warm planning workers for the wrist-servo host (no hardware, no network).

A worker keeps one interpreter with its models loaded and runs the very functions the one-shot planning
commands run, one request at a time, so a request no longer pays interpreter start, imports and model loads.
  sdk worker: plan_move, plan_grasp, bundle      (pinocchio environment)
  sim worker: geometry_live, geometry_wrist      (MuJoCo environment)
Requests and replies are JSON lines. Anything the functions print goes to stderr, never to the reply channel.
"""
import json,subprocess,sys,threading,traceback
from pathlib import Path


def sdk_ops():
    from .wrist_servo_plan import plan,plan_grasp
    def plan_move(root,requestfile,out):
        data=json.loads(Path(requestfile).read_text());plan(root,data['request'],data['delta'],out);return True
    def grasp(root,requestfile,out):
        data=json.loads(Path(requestfile).read_text());plan_grasp(root,data['request'],out);return True
    def bundle(directory):
        from continuous_transfer import export_bundle
        from joint_trajectory import JointPlanner
        p=Path(directory);return export_bundle(json.loads((p/'job.json').read_text()),JointPlanner(p/'profile.json'),p/'prepared_bundle.json')
    return dict(plan_move=plan_move,plan_grasp=grasp,bundle=bundle)


def sim_ops():
    from .live_geometry import check as live_check
    from .wrist_handeye_geometry import check as wrist_check
    def load(config):return json.loads(Path(config).read_text())
    def live(out,config):
        result=live_check(out,load(config))
        if result['unexpected']:raise ValueError('Geometry audit found unexpected contacts')
        return True
    def wrist(out,config):wrist_check(out,load(config));return True
    return dict(geometry_live=live,geometry_wrist=wrist)


def serve(kind):
    channel=sys.stdout;sys.stdout=sys.stderr  # protect the reply channel from prints in planning code
    ops=sdk_ops() if kind=='sdk' else sim_ops()
    channel.write(json.dumps(dict(ready=kind))+'\n');channel.flush()
    for line in sys.stdin:
        request=json.loads(line)
        try:reply=dict(id=request['id'],ok=True,result=ops[request['op']](*request['args']))
        except SystemExit as error:reply=dict(id=request['id'],ok=False,error='exit '+str(error.code))
        except BaseException as error:reply=dict(id=request['id'],ok=False,error=repr(error),trace=traceback.format_exc()[-600:])
        channel.write(json.dumps(reply)+'\n');channel.flush()


class ServiceError(RuntimeError):
    pass


class PlanService:
    """Client for one worker process; one call at a time, a dead or stuck worker is replaced on the next call."""
    def __init__(self,kind,launcher,paths,command=None):
        self.kind,self.launcher,self.paths,self.command=kind,launcher,paths,command;self.lock=threading.Lock();self.process=None;self.counter=0
        self.start()

    def start(self):
        bootstrap='import sys,runpy;sys.path[:0]='+repr(self.paths)+';sys.argv=["plan_service",'+repr(self.kind)+'];runpy.run_module("workcell.plan_service",run_name="__main__")'
        command=self.command or [self.launcher,self.kind,'python','-u','-c',bootstrap]
        self.process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True)
        self.ready=threading.Event();threading.Thread(target=self._wait_ready,daemon=True).start()

    def _wait_ready(self):
        try:json.loads(self.process.stdout.readline());self.ready.set()
        except (ValueError,OSError):pass

    def kill(self):
        process=self.process
        if process is not None and process.poll() is None:
            process.kill()
            try:process.wait(2.)  # so the next call sees a dead worker, not one that is merely not yet reaped
            except subprocess.TimeoutExpired:pass

    def call(self,op,args,timeout):
        with self.lock:
            if self.process.poll() is not None:self.start()
            if not self.ready.wait(timeout):self.kill();raise ServiceError(self.kind+' worker did not become ready')
            self.counter+=1;request=dict(id=self.counter,op=op,args=[str(a) for a in args]);box={}
            def exchange():
                try:
                    self.process.stdin.write(json.dumps(request)+'\n');self.process.stdin.flush()
                    box['reply']=json.loads(self.process.stdout.readline())
                except (OSError,ValueError) as error:box['error']=repr(error)
            thread=threading.Thread(target=exchange,daemon=True);thread.start();thread.join(timeout)
            if thread.is_alive():self.kill();raise ServiceError(f'{self.kind}.{op} timed out')
            reply=box.get('reply')
            if reply is None or reply.get('id')!=request['id']:
                self.kill();raise ServiceError(f'{self.kind}.{op} failed: {box.get("error","no reply")}')  # request/reply order is lost: replace the worker
            if not reply['ok']:raise ServiceError(f'{self.kind}.{op}: {reply["error"]}')
            return reply['result']

    def close(self):
        try:self.process.stdin.close()
        except (OSError,AttributeError):pass
        self.kill()


class PlanPool:
    """A few sdk+sim worker pairs: speculation uses one, a cancelled speculation may still finish in another."""
    def __init__(self,launcher,paths,pairs=3):
        self.free=[(PlanService('sdk',launcher,paths),PlanService('sim',launcher,paths)) for _ in range(pairs)]
        self.condition=threading.Condition();self.all=list(self.free)

    def acquire(self,timeout=10.):
        with self.condition:
            if not self.condition.wait_for(lambda:self.free,timeout):raise ServiceError('no free planning worker')
            return self.free.pop()

    def release(self,pair):
        with self.condition:self.free.append(pair);self.condition.notify()

    def close(self):
        for pair in self.all:
            for service in pair:service.close()


if __name__=='__main__':serve(sys.argv[1])
