"""Taught named poses: record the arm pose while it is hand-guided (freedrive), store it under a name, list / delete.

Only reads: the live freedrive state file on the board and the stored JSON. Nothing here moves or enables the arm.
A stored pose keeps the motor angles as measured (the gravity/guard frame) and the joint angles the planner uses.
"""
import json,time
from pathlib import Path
import numpy as np

STORE=Path(__file__).resolve().parents[2]/'configs/named_poses.json'
LIVE_STATE='/dev/shm/fd_live.json'
READ_LIVE='''import json,time
from pathlib import Path
rows=[]
for _ in range(int(json.loads(__import__("sys").stdin.read())["samples"])):
    try:rows.append(json.loads(Path("%s").read_text()))
    except Exception:pass
    time.sleep(.3)
print(json.dumps(rows))'''%LIVE_STATE


def load(path=STORE):
    path=Path(path)
    return json.loads(path.read_text()) if path.exists() else dict(schema='aipc.named_poses.v1',poses={})


def save(data,path=STORE):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,ensure_ascii=False,indent=1)+'\n');tmp.replace(path)


def read_live_pose(session,samples=5,max_spread_rad=.01):
    """Latest hand-guided pose from the board; refuses a pose that is still moving (spread over ~1.2 s above `max_spread_rad`)."""
    rows=session.remote(READ_LIVE,dict(samples=samples),timeout=20)
    motor=np.array([r['actual_motor_rad'] for r in rows if 'actual_motor_rad' in r],float)
    if len(motor)<samples-1:raise RuntimeError('No live freedrive state on the board (is hand-guiding running?)')
    spread=float(np.max(np.ptp(motor[:,:6],axis=0)))
    if spread>max_spread_rad:raise ValueError(f'Arm is still moving (spread {spread*1000:.1f} mrad over {samples*.3:.1f} s): hold it still')
    return motor[-1],spread,rows[-1]


def record(name,session,profile,store=STORE,overwrite=False):
    """Store the current hand-guided pose as `name`; returns the stored entry."""
    from joint_trajectory import JointPlanner
    from motion_runtime import MotionRuntime
    name=str(name).strip()
    if not name or any(c in name for c in '/\\\n'):raise ValueError('pose name must be a short plain label')
    data=load(store)
    if name in data['poses'] and not overwrite:raise ValueError(f'pose {name!r} exists; pass overwrite to replace it')
    p=JointPlanner(profile);motor,spread,live=read_live_pose(session)
    joint=[(float(m)-o)/d for m,o,d in zip(motor,p.offset,p.direction)]
    tf=MotionRuntime().fk(joint[:6]);tcp=[float(v) for v in tf.translation]
    for i in range(6):
        lo,hi=sorted((p.lower[i]*p.direction[i]+p.offset[i],p.upper[i]*p.direction[i]+p.offset[i]))
        if not lo<=motor[i]<=hi:raise ValueError(f'joint {i+1} outside the installed range')
    entry=dict(motor_rad=[float(v) for v in motor],joint_rad=joint,tcp_base_m=tcp,spread_rad=spread,recorded=time.strftime('%Y-%m-%d %H:%M:%S'),
               source='freedrive live state',saturated=live.get('torque_saturated'),profile_id=getattr(p,'profile_id',None))
    data['poses'][name]=entry;save(data,store);return entry


def delete(name,store=STORE):
    data=load(store);removed=data['poses'].pop(name)
    save(data,store);return removed
