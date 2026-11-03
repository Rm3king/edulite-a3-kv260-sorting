#!/usr/bin/python3
"""Robot service contract: planning JSON lines and explicit attended job execution.

Implements Status, PlanMoveJ, PlanStop and simulated lifecycle for integration
tests. Raw physical commands remain closed; --execute-job selects the reviewed continuous
MoveJ/MoveL/MoveC backend. Default mode and imports never touch hardware.
"""
import argparse
import json
import signal
import sys
from joint_trajectory import JointPlanner, MotionContract


class ArmService:
    def __init__(self, profile=None, execution_backend=None):
        self.planner = JointPlanner(profile) if profile else JointPlanner()
        self.execution_backend = execution_backend
        self.session = MotionContract(self.planner)
        self.last_sequence = -1

    def handle(self, request):
        if not isinstance(request, dict):
            raise ValueError('request must be an object')
        sequence = request.get('sequence')
        if type(sequence) is not int or not 0 <= sequence <= 0xffffffff or sequence <= self.last_sequence:
            raise ValueError('invalid, replayed or reordered sequence')
        # Consume even rejected requests. Never replay a queued operation.
        self.last_sequence = sequence
        cmd = request.get('command')
        if cmd == 'Status':
            result = {'state': self.session.state, 'profile_id': self.planner.profile_id,
                      'execution_backend': 'REVIEWED_JOB' if self.execution_backend else 'OFFLINE_ONLY',
                      'hardware_motion_ready': self.execution_backend is not None}
        elif cmd == 'PlanMoveJ':
            result = self.planner.movej(request['start_rad'], request['goal_rad'],
                                       request.get('speed_scale', 1.), request.get('minimum_duration_s', .1))
        elif cmd in ('PlanMoveL','PlanMoveC'):
            from motion_runtime import MotionRuntime
            from continuous_trajectory import CartesianPlanner
            geometry=CartesianPlanner(MotionRuntime(),self.planner)
            result=geometry.path(request['start_rad'], 'line' if cmd=='PlanMoveL' else 'circle',
                delta=request.get('delta_m',[0,0,.02]),radius=request.get('radius_m',.01),
                axes=tuple(request.get('axes',[0,1])),tcp_speed=request.get('tcp_speed_m_s',.025))
        elif cmd in ('ExecuteReviewedJob','MoveJ','MoveL','MoveC'):
            if self.execution_backend is None:raise ValueError('Physical execution backend not commissioned; no command sent')
            job=request['job']
            if cmd!='ExecuteReviewedJob':
                kinds=[task['kind'] for task in job['tasks']]
                if cmd not in kinds or any(kind not in ('MoveJ',cmd) for kind in kinds):
                    raise ValueError('reviewed job does not match requested motion command')
            result=self.execution_backend(job)
            stopped=sorted(result.get('reset_confirmed',[]))==list(range(1,8)) and not result.get('stop_errors',{})
            return {'sequence':sequence,'ok':result['status'].startswith('PASS'),'state':'DISABLED' if stopped else 'FAULT',
                'host_loc_ref_writes_sent':sum(c['id']>>24==18 and c['data'].startswith('1670') for c in result['commands']),
                'result':{k:result.get(k) for k in ('status','error','task_results','reset_confirmed')}}
        elif cmd == 'PlanStop':
            result = self.planner.brake(request['position_rad'], request['velocity_rad_s'])
        elif cmd == 'SimEnable':
            self.session.enable_simulated(request['position_rad']); result = {}
        elif cmd == 'SimMoveJ':
            result = self.session.movej(request['goal_rad'])
        elif cmd == 'SimAdvance':
            self.session.advance(request['seconds']); result = {}
        elif cmd == 'SimPause':
            result = self.session.pause() or {}
        elif cmd == 'SimHold':
            result = {'position_rad': self.session.hold()}
        elif cmd == 'SimFault':
            self.session.fault(); result = {}
        elif cmd == 'SimDisable':
            self.session.disable_simulated(); result = {}
        elif cmd in ('Enable', 'MoveJ', 'MoveL', 'Pause', 'Hold', 'Home', 'Disable'):
            raise ValueError('Physical execution backend not commissioned; no command sent')
        else:
            raise ValueError('unsupported command')
        return {'sequence': sequence, 'ok': True, 'state': self.session.state,
                'motion_commands_sent': 0, 'result': result}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile')
    parser.add_argument('--execute-job', help='explicit attended execution of immutable reviewed job file')
    parser.add_argument('--output', default='motion_job_result.json')
    parser.add_argument('--request', help='one JSON request; otherwise read JSON lines on stdin')
    args = parser.parse_args()
    if args.execute_job:
        def interrupted(*args):raise KeyboardInterrupt('operator/process interrupt')
        signal.signal(signal.SIGTERM,interrupted)
        from motion_executor import execute_job
        p=JointPlanner(args.profile) if args.profile else JointPlanner()
        service=ArmService(args.profile,lambda job:execute_job(job,p,args.output,args.profile))
        request={'sequence':0,'command':'ExecuteReviewedJob','job':json.loads(open(args.execute_job).read())}
        result=service.handle(request);print(json.dumps(result,allow_nan=False));return 0 if result['ok'] else 1
    service = ArmService(args.profile)
    inputs = [args.request] if args.request is not None else sys.stdin
    failed = False
    for line in inputs:
        try:
            if len(line) > 65536:
                raise ValueError('request exceeds 64 KiB')
            response = service.handle(json.loads(line))
        except (ValueError, KeyError, TypeError) as exc:
            response = {'ok': False, 'error': str(exc), 'motion_commands_sent': 0}
            failed = True
        print(json.dumps(response, allow_nan=False), flush=True)
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
