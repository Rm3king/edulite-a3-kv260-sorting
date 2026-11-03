"""Offline per-tray surfaces and occupied-slot planning for inventory goals."""
import json
import sys
from pathlib import Path
import numpy as np
from .inventory_goal import inventory, point_in_polygon


def floor_point(uv, plane, intrinsics, transform, height=0.):
    n=np.asarray(plane['normal_camera'],float)
    if not np.isclose(np.linalg.norm(n),1.,atol=1e-6):raise ValueError('Nonunit support plane')
    ray=np.array([(uv[0]-intrinsics['cx'])/intrinsics['fx'],(uv[1]-intrinsics['cy'])/intrinsics['fy'],1.])
    denom=float(n@ray)
    if abs(denom)<1e-8:raise ValueError('Ray parallel to plane')
    depth=(height-plane['offset_m'])/denom
    if depth<=0:raise ValueError('Surface behind camera')
    pc=depth*ray
    return transform[:3,:3]@pc+transform[:3,3]


def rectangles_overlap(c1,r1,size1,c2,r2,size2,margin=.006):
    # Separating axes in the common near-planar table surface. Dimensions are
    # conservative part envelopes; full arm/path audit is performed separately.
    for axis in [r1[:,0],r1[:,1],r2[:,0],r2[:,1]]:
        reach=sum(abs(axis@r1[:,i])*size1[i]+abs(axis@r2[:,i])*size2[i] for i in (0,1))
        if abs(axis@(c2-c1))>reach+margin:return False
    return True


def surface_objects(snapshot, config, intrinsics, transform, heights):
    inv=inventory(snapshot,config);rows=[]
    for tray,objects in inv['trays'].items():
        plane=config['trays'][tray]['plane'];n=transform[:3,:3]@np.asarray(plane['normal_camera'])
        for obj in objects:
            uv=np.array(obj['center_uv_px']);height=heights[obj['part_class']]
            pos=floor_point(uv,plane,intrinsics,transform,height)
            axis_uv=(obj.get('pixel_geometry') or {}).get('axis_unit_uv')
            axis=None
            if obj['part_class']=='screw':
                if axis_uv is None:raise ValueError('螺丝杆轴未定位，不能准备抓取')
                a=np.array(axis_uv,float)
                if a.shape!=(2,) or not np.isfinite(a).all() or np.linalg.norm(a)<1e-8:raise ValueError('Invalid shaft axis')
                a/=np.linalg.norm(a)
                axis=floor_point(uv+10*a,plane,intrinsics,transform,height)-floor_point(uv-10*a,plane,intrinsics,transform,height)
                axis/=np.linalg.norm(axis)
            rows.append(dict(detection_index=obj['detection_index'],part_class=obj['part_class'],tray=tray,
                confidence=obj['confidence'],center_uv_px=uv.tolist(),base_surface_xyz_m=pos.tolist(),
                support_normal_base=n.tolist(),surface_height_m=height,axis_base=None if axis is None else axis.tolist(),
                inside_support_fit_patch=point_in_polygon(uv,config['trays'][tray]['roi_uv']),
                robot_grasp_allowed=False,metric_accuracy_verified=False,status='EMPIRICAL_TRAY_SURFACE'))
    return rows


def destination_slots(rows, config, destination, count, intrinsics, transform):
    if type(count) is not int or not 1 <= count <= 5:
        raise ValueError('Placement count must be 1..5')
    spec=config['trays'][destination];R=np.array(spec['rotation']);n=R[:,2];occupied=[];chosen=[]
    for obj in rows:
        if obj['tray']!=destination:continue
        axis=np.array(obj['axis_base']) if obj['axis_base'] is not None else R[:,1]
        normal=np.array(obj['support_normal_base']);rotation=np.column_stack([-np.cross(normal,axis),axis,normal])
        occupied.append((np.array(obj['base_surface_xyz_m'])-obj['surface_height_m']*normal,rotation,
                         [.012,.038] if obj['part_class']=='screw' else [.014,.014]))
    candidates=spec['slots_uv']
    central_single = not occupied and count == 1 and bool(candidates)
    if candidates:
        # Prefer interior candidates even when another part occupies the center.
        # Occupancy filtering below still applies to every selected placement.
        # This ranks existing slots; it is not a measured wall-clearance proof.
        center=np.mean(np.asarray(candidates,float),axis=0)
        candidates=sorted(candidates,key=lambda uv:float(np.sum((np.asarray(uv)-center)**2)))
    if not candidates:
        raise ValueError(f'{destination}盒没有配置候选区域；未准备运动')
    origin=floor_point(center,spec['plane'],intrinsics,transform)
    polygon=[(R.T@(floor_point(uv,spec['plane'],intrinsics,transform)-origin))[:2].tolist()
             for uv in spec['roi_uv']]
    def feasible(uv):
        pos=floor_point(uv,spec['plane'],intrinsics,transform)
        local=R.T@(pos-origin)
        # Placement envelope only, not an arm/gripper wall collision gate.
        # Keep the entire 24x76mm screw envelope plus 8mm inset in the tray ROI.
        if not all(point_in_polygon([local[0]+dx,local[1]+dy],polygon)
                   for dx in (-.020,.020) for dy in (-.046,.046)):
            return None
        if any(rectangles_overlap(pos,R,[.012,.038],c,r,size) for c,r,size in occupied):
            return None
        return pos

    searched=0
    def select_batch(options):
        nonlocal searched
        positions=[feasible(uv) for uv in options]
        valid=[i for i,p in enumerate(positions) if p is not None]
        conflicts={}
        def conflict(i,j):
            key=(min(i,j),max(i,j))
            if key not in conflicts:
                conflicts[key]=rectangles_overlap(positions[i],R,[.012,.038],positions[j],R,[.012,.038])
            return conflicts[key]
        def search(remaining, selected):
            nonlocal searched
            searched+=1
            if searched>50000:
                raise ValueError('放置组合搜索达到上限；未证明盒内无空间，未准备运动')
            if len(selected)==count:return selected
            if len(remaining)<count-len(selected):return None
            for offset,i in enumerate(remaining):
                rest=[j for j in remaining[offset+1:] if not conflict(i,j)]
                result=search(rest,selected+[i])
                if result is not None:return result
            return None
        result=search(valid,[])
        return None if result is None else [options[i] for i in result]

    chosen=select_batch(candidates)
    dense=False
    if chosen is None:
        # Add samples within the old candidates' maximum distance from their
        # center. This extends the finite grid, never the tray ROI/reach radius.
        radius=max(np.linalg.norm((R.T@(floor_point(uv,spec['plane'],intrinsics,transform)-origin))[:2]) for uv in candidates)
        step=max(.010,radius/10)
        limit=int(np.ceil(radius/step));expanded=list(candidates)
        inverse=np.linalg.inv(transform)
        for ix in range(-limit,limit+1):
            for iy in range(-limit,limit+1):
                x,y=ix*step,iy*step
                if x*x+y*y>radius*radius+1e-12:continue
                pc=inverse[:3,:3]@(origin+R[:,0]*x+R[:,1]*y)+inverse[:3,3]
                if pc[2]<=0:continue
                uv=[float(intrinsics['fx']*pc[0]/pc[2]+intrinsics['cx']),float(intrinsics['fy']*pc[1]/pc[2]+intrinsics['cy'])]
                if all(np.linalg.norm(np.asarray(uv)-old)>.1 for old in expanded):expanded.append(uv)
        expanded.sort(key=lambda uv:float(np.linalg.norm(floor_point(uv,spec['plane'],intrinsics,transform)-origin)))
        chosen=select_batch(expanded);dense=True
    if chosen is None:
        raise ValueError(f'{destination}盒未找到可同时放下{count}颗的候选组合；现有零件位置影响落点，未准备运动')
    return dict(pixels=chosen,rotation=R.tolist(),destination_tray=destination,
                selection_policy='INTERIOR_BATCH_SEARCH' if dense else 'CENTRAL_SINGLE_EMPTY_TRAY' if central_single else 'CENTRAL_AVAILABLE_SLOT',
                search_nodes=searched,placement_envelope_inset_m=.008,
                occupancy_checked=True,scope='Finite candidate slots, existing nuts/screws and selected placements; full swept check follows')


def prepare(out, config):
    out=Path(out);snapshot_path=Path(json.loads((out/'input.json').read_text())['points'])
    snapshot=json.loads(snapshot_path.read_text());session=json.loads((snapshot_path.parent/'report.json').read_text())
    work=json.loads(Path(config['workplane']).read_text());cal=json.loads(Path(config['calibration']).read_text())
    if (not snapshot['synchronized'] or work['intrinsics']!=session['intrinsics'] or
        work['camera_serial']!=session['camera_serial'] or snapshot['calibration_id']!=work['calibration_id'] or
        cal['calibration_id']!=work['calibration_id']):raise ValueError('Camera/calibration mismatch')
    import hashlib
    if session['calibration_sha256']!=hashlib.sha256(Path(config['calibration']).read_bytes()).hexdigest():raise ValueError('Camera transform changed')
    T=np.array(cal['T_base_camera']);K=work['intrinsics']
    heights=json.loads(Path(config['heights']).read_text())['surface_heights_m']
    rows=surface_objects(snapshot,config,K,T,heights)
    goal=json.loads((out/'goal_plan.json').read_text());chosen=destination_slots(rows,config,goal['destination'],goal['moves'],K,T)
    folder=out/'target_geometry';folder.mkdir()
    (folder/'surface_points.json').write_text(json.dumps(dict(objects=rows,camera_opened=False,robot_commands_sent=0),indent=2))
    (out/'destinations.json').write_text(json.dumps(chosen,indent=2))


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--config',required=True);a=p.parse_args()
    prepare(a.out,json.loads(Path(a.config).read_text()))
