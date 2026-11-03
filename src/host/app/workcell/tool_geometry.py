"""CAD-only gripper contract, shared by physics and viewer. Not calibration.

The legacy SDK end frame is not the grasp centre. Original finger meshes
extend to end.z=-.104 m and have open inner faces at x=+/- .0467 m.
"""
from pathlib import Path
import numpy as np

TOOL_ID='A3_CAD_GRASP_20261023_UNCALIBRATED'
TCP_IN_END=np.array([0.,0.,-.098])
OPEN_INNER_HALF=.0467
MAX_SLIDE=.0457
OPEN_WIDTH=.040
CLOSED_WIDTH=.010
MOVING_PARTS={
    'hand_r.stl':1,'rack__30_teeth.stl':1,'manifold_solid_brep_39_1__2.stl':1,
    'part_15.stl':-1,'rack__30_teeth__2.stl':-1,'manifold_solid_brep_39_1.stl':-1,
}


def moving_side(filename):return MOVING_PARTS.get(Path(filename).name,0)


def width_to_slide(width):
    if not .002<=width<=2*OPEN_INNER_HALF:raise ValueError('GRIPPER_WIDTH_OUTSIDE_CAD_STROKE')
    return (width-2*OPEN_INNER_HALF)/2


def grasp_transform():
    t=np.eye(4);t[:3,3]=TCP_IN_END;return t


def visual_transform(visual,poses,jaw_slide):
    t=visual['local'].copy();side=moving_side(visual['path'].name)
    if side and visual['link']=='l5_l6_urdf_asm':
        t[0,3]+=side*jaw_slide['right' if side==1 else 'left']
    return poses[visual['link']]@t


def parallel(c):
    c=np.asarray(c,float);return np.eye(3)*(c@c)-np.outer(c,c)


def split_wrist_inertia(mass,com,inertia):
    """Keep the vendor aggregate inertia at the CAD-open reference exactly.

The extra locked L7 is .05 kg, COM=(0,0,-.074) in wrist, I=.0001 I.
Distribute .025 kg to each moving CAD jaw, subtract it from the fixed
aggregate. Individual finger COM/inertia are geometric estimates, unmeasured.
"""
    gm=.05;gc=np.array([0.,0.,-.074]);gi=np.eye(3)*.0001
    total_mass=mass+gm;first=mass*np.asarray(com)+gm*gc
    total_origin=inertia+mass*parallel(com)+gi+gm*parallel(gc)
    fingers=[]
    for sign in [1,-1]:
        centre=np.array([sign*.0662,.0165,-.14825])
        # AABB geometric inertia, not a newly measured physical distribution.
        dims=np.array([.039,.075,.0595]);local=np.diag(gm/2/12*(np.sum(dims*dims)-dims*dims))
        fingers.append((gm/2,centre,local))
    fixed_mass=total_mass-sum(f[0] for f in fingers)
    fixed_com=(first-sum(f[0]*f[1] for f in fingers))/fixed_mass
    fixed_inertia=total_origin-sum(f[2]+f[0]*parallel(f[1]) for f in fingers)-fixed_mass*parallel(fixed_com)
    eigen=np.linalg.eigvalsh(fixed_inertia)
    if eigen[0]<=0 or eigen[-1]>=eigen[0]+eigen[1]:raise ValueError('INVALID_REMAINING_WRIST_INERTIA')
    return fixed_mass,fixed_com,fixed_inertia,fingers
