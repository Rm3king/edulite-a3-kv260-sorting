"""Empty-jaw opening diagnostic; six arm axes stay disabled."""
import argparse, math, json
from gripper_aperture_probe import run


def validate(start, target, current):
    if not all(math.isfinite(x) for x in (start,target,current)):
        raise ValueError('finite settings required')
    if not 1.08 <= target < start <= 1.62 or start-target > .5:
        raise ValueError('inward opening in observed aperture range only')
    if current not in (.3,.5):
        raise ValueError('reviewed 0.3/0.5 A empty opening only')
    return dict(observed_start_rad=start,initial_target_rad=start,target_rad=target,
                lower=target-.02,upper=min(1.62,start+.02),speed=.08,
                acceleration=.2,current=current,width_mm=None)


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--start',type=float,required=True)
    ap.add_argument('--target',type=float,default=1.12)
    ap.add_argument('--current',type=float,default=.5)
    ap.add_argument('--out',required=True)
    ap.add_argument('--execute-attended',action='store_true')
    args=ap.parse_args();plan=validate(args.start,args.target,args.current)
    if args.execute_attended:
        r=run(plan,args.out)
        print(json.dumps({k:r.get(k) for k in ['status','error','final_motor_rad','disabled_confirmed','active_mask','motion_mask']}))
    else:print(json.dumps(dict(status='VALIDATED_NO_HARDWARE',plan=plan)))
