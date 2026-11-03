"""Attended nut trial: online wrist XY followed immediately by a reviewed grasp.

Default is disconnected. This commissioning CLI does not change the GUI's
production workflow or permit a raised grasp height.
"""
import argparse,base64,gzip,hashlib,json,os,re,subprocess,sys,threading,time
from pathlib import Path


def write(path,value):Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2))

DEPLOY='''import sys,json,gzip,base64,hashlib,subprocess,os
from pathlib import Path
p=json.load(sys.stdin);r=Path(p['directory']);r.mkdir(parents=True,exist_ok=False)
for n,b in p['files'].items():
 raw=gzip.decompress(base64.b64decode(b));assert hashlib.sha256(raw).hexdigest()==p['sha256'][n];(r/n).write_bytes(raw)
env=dict(os.environ,PYTHONPATH=p['release']+':/home/ubuntu/kv260_motion_runtime/src:/home/ubuntu/kv260_can_deploy')
jobs=p['jobs']
a=['/home/ubuntu/kv260_motion_runtime/.venv/bin/python',str(r/'wrist_servo_motion.py'),'--job',str(r/jobs[0]['job']),'--profile',str(r/'profile.json'),'--out',str(r/'execution'),'--reviewed-bundle',str(r/jobs[0]['bundle']),'--bundle-sha256',jobs[0]['sha']]
if len(jobs)>1:
 (r/'chain.json').write_text(json.dumps([dict(job=str(r/j['job']),reviewed_bundle=str(r/j['bundle']),bundle_sha256=j['sha']) for j in jobs[1:]]));a+=['--chain',str(r/'chain.json')]
if p.get('validate',True):
 x=subprocess.run(a,env=env,capture_output=True,text=True,timeout=p.get('validate_timeout',45));(r/'validation.log').write_text(x.stdout+x.stderr)
else:
 class x:stdout='';stderr='';returncode=0
print(json.dumps(dict(directory=str(r),returncode=x.returncode,validation=x.stdout,error=x.stderr,release=p['release'],args=a,sha256=p['sha256'])))'''
LAUNCH='''import sys,json,os,subprocess,hashlib
from pathlib import Path
p=json.load(sys.stdin);r=Path(p['directory']);assert not (r/'execution').exists()
for n,h in p['sha256'].items():assert hashlib.sha256((r/n).read_bytes()).hexdigest()==h
env=dict(os.environ,PYTHONPATH=p['release']+':/home/ubuntu/kv260_motion_runtime/src:/home/ubuntu/kv260_can_deploy',PYTHONUNBUFFERED='1')
with (r/'execution.log').open('xb') as log:q=subprocess.Popen(p['args']+['--execute-attended']+(['--prestart-vision'] if p.get('prestart_vision') else []),env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
o=dict(pid=q.pid,directory=str(r));(r/'owner.json').write_text(json.dumps(o));print(json.dumps(o))'''
POLL='''import sys,json,fcntl
from pathlib import Path
p=json.load(sys.stdin);r=Path(p['directory']);out=r/'execution';log=(r/'execution.log').read_text();f=out/'runtime_summary.json';summary=None;alive=Path('/proc/%d'%p['pid']).exists()
if f.exists():
 with open('/tmp/kv260_gripper_commission.lock','a') as lock:
  try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);summary=json.loads(f.read_text())
  except BlockingIOError:pass
print(json.dumps(dict(summary=summary,alive=alive,tail=log[-600:],failed='VISUAL_SERVO_FAILED' in log,events=[x for x in log.splitlines() if any(k in x for k in ['VISUAL_SERVO','ONLINE_GRASP_TASK','JAW_OUTCOME','CHAIN_NEXT_NUT','REBASE','PART_LOST','OPERATOR_SUPPORT','HARDWARE_','RETAINING'])][-1:])))'''
# Read-only, after the run: every part's wrist alignment from the owner's full event log (the polled tail misses most events).
ALIGNMENTS='''import sys,json
from pathlib import Path
p=json.load(sys.stdin);out=[]
for line in (Path(p['directory'])/'execution.log').read_text().splitlines():
 try:e=json.loads(line)
 except ValueError:continue
 if e.get('event')=='CONTINUOUS_SERVO_END':out.append(dict(part=e.get('nut'),mode='continuous',status=e.get('status'),seconds=e.get('seconds'),error_px=e.get('error_px')))
 elif e.get('event')=='VISUAL_SERVO_REPLY' and e.get('action')=='GRASP':out.append(dict(part=e.get('nut'),mode='stop_and_look',rounds=e.get('iteration',0)+1,error_px=e.get('pixel_error')))
print(json.dumps(out))'''
OPEN='''import sys,json,base64,hashlib,importlib.util
from pathlib import Path
p=json.load(sys.stdin);sys.path[:0]=[p['release'],'/home/ubuntu/kv260_motion_runtime/src','/home/ubuntu/kv260_can_deploy'];f=Path('/home/ubuntu/aipc_wrist_servo')/p['filename'];raw=base64.b64decode(p['source']);assert hashlib.sha256(raw).hexdigest()==p['sha'];f.write_bytes(raw)
spec=importlib.util.spec_from_file_location('working_open',str(f));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);r=m.run(m.validate_working_open(p['start']),f.with_suffix(''));print(json.dumps({k:r.get(k) for k in ('status','error','final_motor_rad','disabled_confirmed','active_mask')}))'''


def pick_parts(objects,parts):
    """One scene object per requested class, in the requested order; the nearest-to-robot (largest image v) of each class first."""
    chosen=[];used=set()
    for cls in parts:
        pool=sorted((o for o in objects if o['part_class']==cls and o['detection_index'] not in used),key=lambda o:-o['center_uv_px'][1])
        if not pool:raise ValueError(f'Only {sum(1 for o in objects if o["part_class"]==cls)} {cls} in the source tray for {parts.count(cls)} requested')
        chosen.append(pool[0]);used.add(pool[0]['detection_index'])
    return chosen


def occupied_slots(frame,config,radius_px=70.):
    """Destination slots that already hold a screw/nut in the head-camera frame (same matching radius as the placement check)."""
    import math
    from .live_session import inside
    parts=[o['center_uv_px'] for o in frame['objects'] if str(o.get('part_class','')).startswith(('screw','nut')) and o.get('center_uv_px') and not inside(o['center_uv_px'],config['source_roi_uv'])]
    return {i for i,px in enumerate(config['destinations']['pixels']) if any(math.dist(px,p)<=radius_px for p in parts)}


def run(app,central,count,slots=None,offline_check=False,legacy_motion=False,speculative_grasp=True,prepare_only=False,parts=None,cooling_wait_s=600.,thermal_gate=True,size='m12',replan=False,continuous=False,track_only_s=0.,reverse_small_nut=False):
    """Pick `parts` (default `count` nuts) in ONE enabled session: part k+1 starts at part k's exit pose, only the last returns to support."""
    t0=time.monotonic();marks=[]
    if reverse_small_nut and (parts != ['nut_m6'] or replan):
        raise ValueError('Reverse transfer is limited to one M6 nut')
    # a service left from an earlier run on one of our ports would make that part fail mid-run: refuse before anything moves
    from .proc_guard import require_free_ports,end_group
    require_free_ports('192.168.26.1',[18768+i for i in range(len(parts) if parts else count)])
    def mark(name):
        marks.append(dict(name=name,t=round(time.monotonic()-t0,3)));print('TIMELINE',name,marks[-1]['t'],flush=True)
    names=list(parts) if parts else ['nut']*count;count=len(names)
    if any(c not in ('screw','nut','screw_m6','nut_m6') for c in names):raise ValueError('parts must be screw, nut, screw_m6 or nut_m6')
    # screw_m6 / nut_m6 pick the M6 part of that kind; next to M12 parts (plain screw / nut) they make ONE mixed chain, no return between sizes
    sizes=['m6' if n.endswith('_m6') else size for n in names];parts=[n.split('_')[0] for n in names]
    mixed=len(set(sizes))>1
    if not mixed:size=sizes[0];names=list(parts)
    pick_names=names if mixed else parts
    import numpy as np
    from .live_session import Session,scene_objects,unchanged
    from joint_trajectory import JointPlanner
    from motion_executor import job_digest
    from .wrist_servo_plan import plan_grasp
    from continuous_transfer import export_bundle
    from wrist_servo_motion import validate_grasp_reply
    app=Path(app).resolve();central=Path(central).resolve();runtime=central/'contest/arm_runtime'
    camera=str(app/'.venv-camera/bin/python');launcher=str(Path.home()/'.local/share/aipc-dev/aipc.sh')
    paths=[str(app/'integration'),str(runtime),str(central/'contest/kv260_can/software')]
    slots_given=slots is not None
    if slots is None:
        if set(parts)=={'nut'}:slots=[1,3,0,4][:count]
        else:
            # Slots 2 and 3 are 267/271 mm from the base, 0 and 1 are 330/333 mm: the farther ones held J2 at 5.2 Nm for the
            # place, open and exit legs (40 % of the heat of a screw, run 43). Screws take the near slots first, a nut the centre slot.
            from .order_planner import SLOT_POOLS as pools;slots=[]
            for cls in parts:slots.append(next(x for x in pools[cls] if x not in slots))
    if len(slots)!=count or len(set(slots))!=count or any(x not in range(5) for x in slots):raise ValueError('One distinct destination slot required per nut')
    def scene(fr):return scene_objects(fr,s.config,('screw','nut','screw_m6','nut_m6') if mixed else ('screw','nut'))
    def size_view(points_path,index):
        # mixed chain: an M6 part is planned on the M6 view of the same frame (M6 as screw/nut, M12 kept as obstacles); an M12 part on the raw frame
        if not mixed or sizes[index]!='m6':return points_path
        view=points_path.with_name(points_path.name.replace('_points.json','_m6view_points.json'))
        if not view.exists():view.write_text(points_path.read_text());s.map_m6(view,json.loads(points_path.read_text()))
        return view
    overrides=dict(joint_time_law='septic' if legacy_motion else 'cruise_c3',part_size='m12' if mixed else size)
    if reverse_small_nut:
        from .reverse_transfer import overrides as reverse_overrides, check_destination, SLOT, tray_objects
        overrides.update(reverse_overrides(json.loads((app/'integration/config/live_workflow.json').read_text())))
        slots=[SLOT];slots_given=True
    s=Session(app,overrides=overrides);initial=s.snapshot();frame=json.loads(initial.read_text());targets=pick_parts(scene(frame),pick_names);mark('snapshot')
    if reverse_small_nut:
        if len([o for o in scene(frame) if o['part_class']=='nut']) != 1:raise ValueError('Left tray must contain exactly one M6 nut')
        check_destination(frame,s.config)
    if not slots_given:
        # A part already lying in a destination slot (an earlier run, or placed by hand) keeps that slot: take the next free one in the same order.
        from .order_planner import SLOT_POOLS as pools
        taken=occupied_slots(frame,s.config)
        if taken:
            slots=[];order={'screw':pools['screw'],'nut':[1,3,0,4] if set(parts)=={'nut'} else pools['nut']}
            for cls in parts:
                free=[x for x in order[cls] if x not in slots and x not in taken]
                if not free:raise ValueError('No free destination slot: occupied '+str(sorted(taken)))
                slots.append(free[0])
            print('DESTINATION_SLOTS_OCCUPIED',sorted(taken),'-> using',slots,flush=True)
    root=app/'reports'/('four_integrated_'+time.strftime('%Y%m%d_%H%M%S'));root.mkdir();write(app/'reports/wrist_servo_trial_active.json',dict(directory=str(root),status='PREPARING',host_pid=os.getpid()))
    write(root/'initial.json',dict(points=str(initial),targets=targets[:count]));reference=json.loads((app/'reports/wrist_online_20261025_160952/servo_config.json').read_text());results=[]
    sources={n:(runtime/n).read_bytes() for n in ['wrist_servo_motion.py','wrist_handeye_motion.py','wrist_continuous.py']}
    def geometry(module,out):
        bootstrap='import sys,runpy;sys.path[:0]='+repr(paths)+';runpy.run_module('+repr(module)+',run_name="__main__")'
        r=subprocess.run([launcher,'sim','python','-c',bootstrap,'--out',str(out),'--config',str(out/'workflow_config.json')],capture_output=True,text=True,timeout=20)
        (out/'geometry.log').write_text(r.stdout+r.stderr);r.check_returncode()
    def camera_clearance(candidate,out):
        rows=json.loads((out/'collision_samples.json').read_text())['samples'];plane=s.config['trays']['A']['plane'];n=np.array(plane['normal_base'])
        v=min(float(n@(np.array(x['tcp_m'])+np.array(x['rotation'])[:,2]*.098)+plane['offset_base_m']-.075) for x in rows)
        if reverse_small_nut:
            plane=s.config['trays']['B']['plane'];n=np.array(plane['normal_base'])
            v=min(v,min(float(n@(np.array(x['tcp_m'])+np.array(x['rotation'])[:,2]*.098)+plane['offset_base_m']-.075) for x in rows))
        if v<.005:raise ValueError('Camera housing clearance')
        candidate['geometry_audit'].update(wrist_camera_proxy_checked=True,camera_proxy_min_floor_clearance_m=v)
    servers=[];stop=threading.Event();thread=None
    try:
        # ---- stopped-arm checks, once for the whole session
        print('BEGIN_SESSION',count,flush=True);jointdir=root/'joints_1';jointdir.mkdir();s.capture_joints(jointdir);mark('joints_read')
        feedback=json.loads((jointdir/'connection_setup.json').read_text())['feedback']
        latest={f['motor']:f for f in feedback}
        if len(latest)!=7:raise ValueError('Missing stopped temperature feedback')
        temperatures={i:f['temperature'] for i,f in latest.items()}
        cooling_deadline=time.monotonic()+cooling_wait_s  # 0 = operator chose to start now: same path as the timeout, the board's 70 C return still applies
        cooling_round=0
        # A chained session heats the shoulder (J2) about 11 C and J3 about 6 C per nut (2026-10-26 runs); start only
        # when the whole chain is predicted to stay under the 70 C return line, else cool, else do fewer nuts.
        # Measured per part (2026-10-26): a nut adds about J2 +11 / J3 +6 C; a screw (4 hover corrections, long dwell
        # with the arm extended) added J2 +24 / J3 +12 C in one run and reached the 70 C return line.
        from .order_planner import predicted_peak,START_LIMIT_C,PEAK_LIMIT_C
        def peak(temps,n):return predicted_peak(temps,parts[:n])  # per-part rises live in order_planner.RISE
        if not thermal_gate and max(temperatures.values())>=START_LIMIT_C:raise TimeoutError('Too warm to start even without the thermal gate')
        while thermal_gate and (max(temperatures.values())>=START_LIMIT_C or peak(temperatures,count)>PEAK_LIMIT_C) and not prepare_only:
            write(root/'thermal_pause.json',dict(next_target=1,temperature_c=temperatures,predicted_peak_c=peak(temperatures,count),nuts=count,motor_commands_enabled=0))
            print('COOLING_SUPPORTED',temperatures,'predicted peak',round(peak(temperatures,count)),'for',count,'nuts',flush=True)
            if time.monotonic()>cooling_deadline:
                if max(temperatures.values())>=START_LIMIT_C:raise TimeoutError('Still warm while disabled; no enable sent')
                fit=max((n for n in range(1,count+1) if peak(temperatures,n)<=PEAK_LIMIT_C),default=1)
                print('COOLING_TIMEOUT_REDUCING_NUTS',count,'->',fit,flush=True);count=fit;slots=slots[:count];break
            time.sleep(30);cooling_round+=1
            jointdir=root/f'joints_1_cooling_{cooling_round}';jointdir.mkdir();s.capture_joints(jointdir)
            feedback=json.loads((jointdir/'connection_setup.json').read_text())['feedback'];latest={f['motor']:f for f in feedback}
            if len(latest)!=7 or any(f['mode']!=0 or f['fault'] for f in latest.values()):raise ValueError('Stopped feedback changed during cooling')
            temperatures={i:f['temperature'] for i,f in latest.items()}
        mark('thermal_gate');cap=json.loads((jointdir/'board_capture.json').read_text());jaw=[x['decoded']['position_motor_rad'] for x in cap['transactions'] if x['motor']==7][-1]
        if abs(jaw-1.12)>.002:
            if prepare_only:raise ValueError('Prepare-only requires working-open jaw; no motor command sent')
            raw=(runtime/'gripper_aperture_probe.py').read_bytes();r=s.remote(OPEN,dict(source=base64.b64encode(raw).decode(),sha=hashlib.sha256(raw).hexdigest(),filename='working_open_'+time.strftime('%H%M%S')+'.py',release=s.release,start=jaw),timeout=25);write(jointdir/'jaw_open.json',r)
            assert r['disabled_confirmed'] and r['active_mask']==0 and r['status'].startswith('PASS')
            # the job's start pose must carry the opened jaw (1.12), not the pre-open reading
            jointdir=root/'joints_1_after_open';jointdir.mkdir();s.capture_joints(jointdir)
            cap=json.loads((jointdir/'board_capture.json').read_text());jaw=[x['decoded']['position_motor_rad'] for x in cap['transactions'] if x['motor']==7][-1]
            if abs(jaw-1.12)>.01:raise ValueError(f'Jaw not at working open after restore: {jaw:.4f}')
        # ---- plan every nut up front from one fresh snapshot; nut k+1 starts at nut k's exit pose
        # The scene was captured before any cooling wait; pick the nuts again from a fresh frame so a
        # long wait (or a hand re-placing parts) cannot invalidate the earlier correspondence.
        points=s.snapshot();mark('snapshot_2');folders=[];jobs=[];planners=[]
        targets=pick_parts(scene(json.loads(points.read_text())),pick_names)
        if reverse_small_nut:check_destination(json.loads(points.read_text()),s.config)
        write(root/'initial.json',dict(points=str(points),targets=targets[:count],note='re-selected after cooling'))
        # Part k+1 is planned from part k's exit pose, which exists once part k is planned. Its geometry audit, bundle export and file
        # edits (about 4.5 s) therefore run in a worker thread while the next part is planned, instead of one part after the other.
        from concurrent.futures import ThreadPoolExecutor
        pool=ThreadPoolExecutor(max_workers=3);futures=[]
        def finalize(index,folder,finish,cls,within=None):
            finish()
            meta=json.loads((folder/'prepared.json').read_text());meta.update(status='INLINE_SERVO_ONLY',replay_only=True);write(folder/'prepared.json',meta)
            job=json.loads((folder/'job.json').read_text());planner=JointPlanner(folder/'profile.json')
            if index>0:job['hardware_start_verified']=True # replaced by the board's exit-pose handover check
            for i in range(6):
                lo,hi=sorted((planner.lower[i]*planner.direction[i]+planner.offset[i],planner.upper[i]*planner.direction[i]+planner.offset[i]));job['motor_lower_rad'][i]=max(lo,job['motor_lower_rad'][i]-.22);job['motor_upper_rad'][i]=min(hi,job['motor_upper_rad'][i]+.22)
            if within is not None:
                # a re-planned job must stay inside the guards the board installed at enable (the board checks its records too)
                job['motor_lower_rad']=[max(a,b) for a,b in zip(job['motor_lower_rad'],within[0])];job['motor_upper_rad']=[min(a,b) for a,b in zip(job['motor_upper_rad'],within[1])]
            job['visual_servo']=dict(endpoint=f'http://192.168.26.1:{18768+index}/',integrated_grasp=True,noncontact_only=False,max_correction_m=.035,normal_return_disable_authorized=True,**({'rebase':True} if replan and index>0 else {}))
            # M6 screw stays on stop-and-look: the head camera aims at the screw's middle, the grip point is under its head
            # 10-17 mm along the axis, beyond the 15 mm correction box (run16: saturated at 15 mm, 15 px short, timeout)
            if continuous and (cls,sizes[index])==('screw','m6'):print('CONTINUOUS_SKIPPED',index,'screw_m6 uses stop-and-look',flush=True)
            if continuous and (cls,sizes[index])!=('screw','m6'):
                # Experimental continuous wrist correction for this part: bounds from the hover pose, then the tube audit at the
                # correction-box corners. Any refusal leaves the part on the stop-and-look servo.
                from .continuous_plan import continuous_config,tube_audit
                # seed and search radius exactly as this part's servo_config.json below gives them to the stop-and-look servo
                servo_ref=dict(reference);radius=None
                if cls=='screw':servo_ref['coarse_center_px']=reference['target_px'];radius=220.
                else:
                    if any(job.get('wrist_hover_bias_tool_xy_m',[0.,0.])):servo_ref['coarse_center_px']=s.config.get('wrist_hover_expected_center_px',reference['target_px'])
                    if sizes[index]=='m6':radius=110.
                try:
                    from .continuous_plan import board_locators_match
                    if (cls=='nut' or sizes[index]=='m6') and not board_locators_match():raise ValueError('board bundle locators differ from the host ones: redeploy')
                    # a smaller correction box when the 15 mm corners take a joint past its limit at this hover (run6 M12 nut)
                    for box in (.015,.010,.006):
                        try:
                            with s.runtime_lock:cfg=continuous_config(s.runtime,planner,job,servo_ref,cls,part_size=sizes[index],radius_px=radius,max_correction_m=box)
                            if track_only_s:cfg['track_only_s']=float(track_only_s)
                            audit=tube_audit(folder,job,cfg,s.runtime,planner,lambda out:geometry('workcell.live_geometry',out),folder/'tube_audit',lock=s.runtime_lock)
                            break
                        except ValueError as error:
                            if box==.006 or not any(w in str(error) for w in ('limit','ceiling')):raise
                            print('CONTINUOUS_BOX_SHRINK',index,box,repr(error),flush=True)
                    write(folder/'tube_audit.json',dict(cfg,audit=audit))
                    if audit['passed']:
                        cfg['tube_audit']=dict(passed=True,corners=[dict(corner=c['corner'],status=c['status']) for c in audit['corners']])
                        job['visual_servo']['continuous']=cfg;print('CONTINUOUS_ENABLED',index,cls,'limits_mrad',[round(v*1e3,1) for v in cfg['limits_rad']],flush=True)
                    else:print('CONTINUOUS_REFUSED_TUBE',index,json.dumps(audit['corners'])[:300],flush=True)
                except ValueError as error:print('CONTINUOUS_REFUSED',index,repr(error),flush=True)
            camera_clearance(job,folder);write(folder/'job.json',job)
            initial_bundle_sha=export_bundle(job,planner,folder/'servo_initial_bundle.json')
            with s.runtime_lock:tf=s.runtime.fk(job['tasks'][0]['goal_rad'][:6])
            source=job['targets'][0]['source'];rotation=job['targets'][0]['source_rotation']
            write(folder/'probe_plan.json',dict(rotation=rotation,source=source,views=[dict(name='departure',source=source,correction_base_m=[0.,0.,0.],T_base_end=tf.homogeneous.tolist())]))
            hover_bias=job.get('wrist_hover_bias_tool_xy_m',[0.,0.])
            # With the measured offset applied the nut appears near the target pixel, not at the old coarse seed.
            # screw_length_px 328 = full screw incl. head seen uncut at the hover height (2026-10-26 run28 frame, about 69 mm)
            write(folder/'servo_config.json',dict(reference,**({'coarse_center_px':s.config.get('wrist_hover_expected_center_px',reference['target_px'])} if any(hover_bias) else {}),part_class=cls,part_size=sizes[index],**({'coarse_center_px':reference['target_px'],'initial_search_radius_px':220.,'screw_length_px':328. if sizes[index]=='m12' else None} if cls=='screw' else ({'initial_search_radius_px':110.} if sizes[index]=='m6' else {})),hover_bias_tool_xy_m=hover_bias,warm_services=True,job_sha256=job_digest(job),python_paths=paths,launcher=launcher,max_correction_m=.035,integrated_grasp=True,speculative_grasp=speculative_grasp))
            if index==0 and offline_check:
                request=dict(q_rad=job['tasks'][0]['goal_rad'],cumulative_tool_xy_m=[0.,0.],executed_prefix=job['tasks'][0]['segments'],pixel_error=3.)
                checkdir=folder/'offline_continuation_test';plan_grasp(folder,request,checkdir);geometry('workcell.live_geometry',checkdir)
                candidate=json.loads((checkdir/'job.json').read_text());camera_clearance(candidate,checkdir)
                reply=dict(action='GRASP',candidate=candidate,pixel_error=3.,cumulative_tool_xy_m=[0.,0.]);checked=validate_grasp_reply(reply,request,job,planner)
                import pickle,copy
                restored=pickle.loads(pickle.dumps(checked));assert restored['records'][0]['plan'] is restored['candidate']['tasks'][1]['segments'][0]
                wrong=copy.deepcopy(reply);wrong['candidate']['grasp_height_offset_m']=.030
                try:validate_grasp_reply(wrong,request,job,planner)
                except ValueError:pass
                else:raise AssertionError('Raised height accepted')
                write(checkdir/'result.json',dict(status='PASS_OFFLINE_ONLY',synthetic_pixel_error=True,remaining_segments=len(checked['records']),identity_preserved=True,raised_height_rejected=True));print('INLINE_OFFLINE_PASS',flush=True)
            return folder,job,planner,initial_bundle_sha
        previous=None;first_pending=None
        for index,original in enumerate(targets[:count]):
            cls=parts[index];same=[o for o in scene(json.loads(points.read_text())) if o['part_class']==pick_names[index]];selected=min(same,key=lambda o:np.linalg.norm(np.array(o['center_uv_px'])-original['center_uv_px']))
            if np.linalg.norm(np.array(selected['center_uv_px'])-original['center_uv_px'])>=30:raise ValueError(f'Original {cls} moved beyond correspondence window')
            chain=dict(index=index,count=count)
            if index>0:chain.update(support_rad=first_pending['chain']['support_rad'],start_rad=json.loads((previous/'job_pending_geometry.json').read_text())['tasks'][-1]['goal_rad'])
            prepared=s.prepare(screws=int(cls=='screw'),nuts=int(cls=='nut'),source_nut_uv_px=selected['center_uv_px'],source_part_class=cls,destination_slot_index=slots[index],chain=chain,replay=folders[0] if index>0 else None,replay_points=size_view(points,index) if index>0 else None,captured=(size_view(points,index),jointdir) if index==0 else None,defer_geometry=True)
            folder=Path(prepared['directory']);(root/f'nut_{index+1}.txt').write_text(str(folder));folders.append(folder);previous=folder
            if index==0:first_pending=json.loads((folder/'job_pending_geometry.json').read_text())
            futures.append(pool.submit(finalize,index,folder,prepared['finish'],cls));mark(f'part_{index}_planned')
        for index,future in enumerate(futures):
            folder,job,planner,initial_bundle_sha=future.result();jobs.append(job);planners.append((planner,initial_bundle_sha));mark(f'part_{index}_prepared')
            if index>0 and max(abs(a-b) for a,b in zip(job['start_rad'],jobs[index-1]['tasks'][-1]['goal_rad']))>1e-9:raise ValueError('Chain start differs from the previous exit pose')
        pool.shutdown()
        REBASE_PX=float(s.config.get('rebase_moved_px',40.))   # head-camera px (~0.34 mm/px): smaller moves stay with the wrist correction (<=35 mm)
        def installed_guards():
            # what the board installs at enable: the job's own bounds for one job, the union over a chain
            return ([min(j['motor_lower_rad'][k] for j in jobs) for k in range(7)],[max(j['motor_upper_rad'][k] for j in jobs) for k in range(7)]) if len(jobs)>1 else (jobs[0]['motor_lower_rad'],jobs[0]['motor_upper_rad'])
        def rebase(i,threshold=None):
            # Part i-1 is being placed: look at the source tray again; if part i was moved, plan it afresh from part i-1's exit pose.
            decision=dict(action='KEEP')
            try:
                orig=np.array(targets[i]['center_uv_px'],float)
                for attempt in range(5):
                    # a hand that just moved the part may still cover it: look again for a few seconds before giving up
                    pts=s.snapshot();fr=json.loads(pts.read_text())
                    same=[o for o in scene(fr) if o['part_class']==pick_names[i]]
                    if same:break
                    time.sleep(1.2)
                if not same:decision=dict(action='KEEP',reason='part not seen by the head camera')
                else:
                    sel=min(same,key=lambda o:np.linalg.norm(np.array(o['center_uv_px'])-orig));moved=float(np.linalg.norm(np.array(sel['center_uv_px'])-orig))
                    decision=dict(action='KEEP',moved_px=moved,reason='moved less than %.0f px'%REBASE_PX)
                    # Until the head-camera (D435i) stand is in the collision model, a re-plan stays near the region the reviewed plans
                    # already use: 2026-10-29 a part moved 340 px (~11 cm) gave a departure that struck the camera stand.
                    if moved>float(s.config.get('rebase_max_moved_px',120.)) and threshold is None:
                        decision=dict(action='KEEP',moved_px=moved,reason='moved beyond the re-plan limit (camera stand not modelled)')
                    elif moved>=(REBASE_PX if threshold is None else threshold):
                        cls=parts[i];chain=dict(index=i,count=count,support_rad=jobs[i]['chain']['support_rad'],start_rad=jobs[i]['start_rad'])
                        prepared=s.prepare(screws=int(cls=='screw'),nuts=int(cls=='nut'),source_nut_uv_px=sel['center_uv_px'],source_part_class=cls,destination_slot_index=slots[i],chain=chain,replay=folders[0],replay_points=size_view(pts,i),defer_geometry=True)
                        folder,job,_,sha=finalize(i,Path(prepared['directory']),prepared['finish'],cls,within=installed_guards())
                        if max(abs(x-y) for x,y in zip(job['tasks'][-1]['goal_rad'],jobs[i]['tasks'][-1]['goal_rad']))>1e-6:decision=dict(action='KEEP',moved_px=moved,reason='re-planned exit pose differs')
                        else:decision=dict(action='REPLAN',folder=str(folder),job_sha256=job_digest(job),bundle_sha256=sha,moved_px=moved)
            except Exception as error:decision=dict(action='KEEP',error=repr(error)[:300])
            write(folders[i]/'rebase_decision.json',decision);print('REBASE_DECISION',i,json.dumps(decision)[:300],flush=True)
        # ---- one deployment, one owner on the board, one vision worker per nut
        files=dict(sources);entries=[]
        files['profile.json']=(folders[0]/'profile.json').read_bytes()
        for index,folder in enumerate(folders):
            prefix='' if index==0 else f'nut{index}_'
            files[prefix+'job.json']=(folder/'job.json').read_bytes();files[prefix+'servo_initial_bundle.json']=(folder/'servo_initial_bundle.json').read_bytes()
            entries.append(dict(job=prefix+'job.json',bundle=prefix+'servo_initial_bundle.json',sha=planners[index][1]))
        # The vision workers boot and the scene-check frame is taken while the files travel to the board.
        env=dict(os.environ,PYTHONPATH=str(app/'integration'))
        for index,folder in enumerate(folders):
            hostlog=(folder/'host.log').open('w');servers.append((subprocess.Popen([camera,'-u','-m','workcell.wrist_servo_host',str(folder),str(18768+index)],env=env,stdout=hostlog,stderr=subprocess.STDOUT,start_new_session=True),hostlog))
        record=folders[0]/'wrist_sequence';record.mkdir()
        def record_camera():
            from urllib.request import urlopen
            i=0
            while not stop.is_set():
                try:
                    with urlopen('http://127.0.0.1:18766/frame.jpg',timeout=1) as response:
                        (record/f'{i:04d}.jpg').write_bytes(response.read());write(record/f'{i:04d}.json',dict(headers=dict(response.headers),host_monotonic=time.monotonic()));i+=1
                except Exception as error:(record/'last_error.txt').write_text(str(error))
                stop.wait(.15)
        thread=threading.Thread(target=record_camera,daemon=True);thread.start()
        recheck=ThreadPoolExecutor(max_workers=1).submit(lambda:json.loads(s.snapshot().read_text()))
        extra=30*(count-1)
        deployed=s.remote(DEPLOY,dict(directory='/home/ubuntu/aipc_wrist_servo/'+root.name,release=s.release,jobs=entries,validate=prepare_only,validate_timeout=45+extra,files={n:base64.b64encode(gzip.compress(b)).decode() for n,b in files.items()},sha256={n:hashlib.sha256(b).hexdigest() for n,b in files.items()}),timeout=55+extra);write(folders[0]/'servo_deployment.json',deployed)
        mark('deployed')
        if deployed['returncode']:raise ValueError(deployed)
        if prepare_only and replan and count>1:
            # Offline self-test of the re-plan path: force a re-plan of part 2 and run the board's own rebase validation here.
            from wrist_servo_motion import validate_rebase
            rebase(1,threshold=0.);decision=json.loads((folders[1]/'rebase_decision.json').read_text())
            if decision['action']!='REPLAN':raise ValueError('Rebase self-test did not re-plan: '+json.dumps(decision))
            nf=Path(decision['folder']);old=json.loads((folders[1]/'job.json').read_text())
            reply=dict(nonce='selftest',action='REPLAN',job_sha256=decision['job_sha256'],bundle_sha256=decision['bundle_sha256'],
                       job_gz_b64=base64.b64encode(gzip.compress((nf/'job.json').read_bytes())).decode(),bundle_gz_b64=base64.b64encode(gzip.compress((nf/'servo_initial_bundle.json').read_bytes())).decode())
            lower=[min(j['motor_lower_rad'][k] for j in jobs) for k in range(7)];upper=[max(j['motor_upper_rad'][k] for j in jobs) for k in range(7)]
            checked=validate_rebase(reply,dict(nonce='selftest',q_rad=old['start_rad'],guards=(lower,upper)),old,JointPlanner(folders[0]/'profile.json'))
            print('REBASE_SELFTEST_PASS',dict(records=len(checked['validated'][0]),digest=checked['digest'][:12],seconds_note='see REBASE_DECISION'),flush=True)
        if prepare_only:
            print('PREPARED_ONLY',json.dumps(deployed['validation']),flush=True);write(root/'completed.json',dict(prepared=[str(f) for f in folders],validation=deployed['validation']));return
        deadline=time.monotonic()+4+2*count
        while any('READY' not in (f/'host.log').read_text() for f in folders):
            if any(server.poll() is not None for server,_ in servers) or time.monotonic()>deadline:raise RuntimeError('Vision worker did not become ready')
            time.sleep(.05)
        mark('vision_workers_ready')
        old=json.loads(Path(json.loads((folders[0]/'input.json').read_text())['points']).read_text());fresh=recheck.result()
        if not unchanged(scene_objects(old,s.config),scene_objects(fresh,s.config)):raise ValueError('Scene changed before enable')
        if reverse_small_nut:
            check_destination(fresh,s.config)
            if not unchanged(tray_objects(old,s.config),tray_objects(fresh,s.config)):raise ValueError('Scene changed before enable')
        mark('scene_checked')
        rebase_pool=ThreadPoolExecutor(max_workers=1);placed=0;holding=False
        owner=s.remote(LAUNCH,dict(deployed,prestart_vision=any((j.get('visual_servo') or {}).get('continuous') for j in jobs)),timeout=15);mark('owner_launched');write(folders[0]/'servo_owner.json',owner);deadline=time.monotonic()+120+90*count;last='';failed=None;next_alarm=0.;poll_errors=0
        # A fault leaves the owner holding the arm until someone supports it and confirms. This loop must therefore
        # outlive the fault: keep watching, keep the vision workers, and keep saying so. It never decides to remove torque.
        while True:
            try:poll=s.remote(POLL,owner,timeout=8);poll_errors=0
            except Exception as error:
                poll_errors+=1;print('\aLINK_POLL_ERROR',poll_errors,repr(error)[:160],flush=True)
                if poll_errors in (3,30) or poll_errors%120==0:print('\aALARM_BOARD_LINK_LOST: arm may be held or released; check the arm and the board now',flush=True)
                time.sleep(1.);continue
            event=str(poll['events'])
            if event!=last:
                print('SESSION',event,flush=True);last=event
                # part k is placed once any of its place/open/exit events follows its close (polling can miss a single event)
                if replan and '_close"' in event:holding=True
                elif replan and holding and any(k in event for k in ('_place"','_open"','_exit"')):
                    holding=False;placed+=1
                    if placed<count:rebase_pool.submit(rebase,placed)
                named=re.search(r'"(?:name|event)": "([^"]+)"',event);mark('board:'+(named.group(1) if named else '?'))
            if not poll.get('alive',True) and not poll['summary'] and failed is None:
                raise RuntimeError('Board executor exited before finishing (nothing was enabled if no EXECUTION event): '+poll.get('tail','')[-300:])
            if poll['summary']:
                summary=poll['summary'];write(folders[0]/'servo_runtime_summary.json',summary);assert summary['final_active_mask']==0
                break
            if failed is None and (poll.get('failed') or 'VISUAL_SERVO_FAILED' in event):  # the last matching line is overwritten by the 30 s support reminders
                failed=time.monotonic();write(root/'FAULT_ARM_HELD.json',dict(event=event,detected=time.strftime('%H:%M:%S')))
            if failed is not None and time.monotonic()>=next_alarm:
                next_alarm=time.monotonic()+30.
                print('\aALARM_ARM_HELD_NEEDS_SUPPORT waiting %ds; support the arm, then create SUPPORTED_DISABLE on the board; last=%s'%(time.monotonic()-failed,event[:200]),flush=True)
            if failed is None and time.monotonic()>deadline:raise TimeoutError('Motion monitor deadline; do not force reset')
            time.sleep(.6)
        rebase_pool.shutdown(wait=True)   # a late re-plan must not share the camera with the acceptance snapshot
        try:after=s.snapshot();(folders[0]/'after_snapshot_path.txt').write_text(str(after))
        except (TimeoutError,ValueError,RuntimeError) as error:after=None;print('AFTER_SNAPSHOT_FAILED',repr(error),flush=True)  # the motion is already over and verified; do not lose its results
        statuses=summary.get('nut_statuses') or []
        if after is not None:
            # Acceptance from the head camera: every part that completed should now sit in its slot.
            from .placement_report import verify_placement
            done=[dict(part_class=pick_names[i],slot=slots[i]) for i in range(count) if i<len(statuses) and str(statuses[i]).startswith('COMPLETED')]
            # all five slots: the run folder only holds the slots of that part
            try:report=verify_placement(json.loads(after.read_text()),s.config,done,s.config['destinations']['pixels'],before=frame);write(root/'placement_report.json',report);print('PLACEMENT_REPORT',report['summary'],flush=True)
            except (OSError,ValueError,KeyError) as error:print('PLACEMENT_REPORT_FAILED',repr(error),flush=True)
        try:
            alignments=s.remote(ALIGNMENTS,owner,timeout=15);write(root/'alignments.json',alignments)
            for a in alignments:print('ALIGNMENT',json.dumps(a),flush=True)
        except Exception as error:print('ALIGNMENTS_FAILED',repr(error)[:160],flush=True)
        for index,folder in enumerate(folders):
            statuses=summary.get('nut_statuses') or []
            results.append(dict(number=index+1,directory=str(folder),status=statuses[index] if index<len(statuses) else 'NOT_ATTEMPTED'))
        results.append(dict(summary=summary,after_snapshot=str(after)));write(root/'results.json',results)
        mark('end');print('END_SESSION',summary['status'],summary.get('nut_statuses'),flush=True)
        write(root/'completed.json',dict(results=results));print('BATCH_FINISHED',root,flush=True)
    except BaseException as error:write(root/'failure.json',dict(error=repr(error),results=results));raise
    finally:
        write(root/'timeline.json',marks);stop.set()
        if not prepare_only:
            try:
                from .trial_log import append
                append(root,parts)
            except Exception as error:print('TRIAL_LOG_FAILED',repr(error),flush=True)
        if thread is not None:thread.join(2)
        for server,hostlog in servers:
            end_group(server)   # the service and its planning workers
            hostlog.close()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--app-root',type=Path,default=Path(__file__).resolve().parents[2]);p.add_argument('--central-root',type=Path,default=Path('/opt/edulite/project'));p.add_argument('--count',type=int,choices=range(1,5),default=4);p.add_argument('--size',choices=('m12','m6'),default='m12',help='m6: pick the bright M6 screws/nuts (detector classes screw_m6/nut_m6) with the same procedure');p.add_argument('--destination-slots',type=int,nargs='+');p.add_argument('--offline-check',action='store_true');p.add_argument('--legacy-motion',action='store_true',help='Use previous all-ramp joint timing');p.add_argument('--no-speculative-grasp',action='store_true',help='Plan the grasp only after convergence (previous behaviour)');p.add_argument('--parts',nargs='+',choices=['screw','nut','screw_m6','nut_m6'],help='Parts to pick in this order, e.g. screw screw nut (default: --count nuts)');p.add_argument('--skip-thermal-gate',action='store_true',help='Operator order: start now with all parts; the board return line is the protection (start still refused above the start limit)');p.add_argument('--no-cooling-wait',action='store_true',help='Do not wait for the start temperature gate; run the timeout path at once (at least 1 part). The board 70 C return stays.');p.add_argument('--prepare-only',action='store_true',help='Plan, deploy and validate on the board, then stop before enabling anything');p.add_argument('--execute-attended',action='store_true');p.add_argument('--continuous',action='store_true',help='Experimental: board-side continuous wrist correction through the PL correction channel (all four parts)');p.add_argument('--track-only',type=float,default=0.,help='With --continuous: only follow the part for this many seconds at the hover, then return without grasping (commissioning)');p.add_argument('--replan-moved',action='store_true',help='Before each next part, look again with the head camera and re-plan it from the exit pose if it was moved');p.add_argument('--reverse-small-nut',action='store_true',help='One M6 nut from left tray B to the interior of right tray A');a=p.parse_args()
    if not a.execute_attended and not a.prepare_only:print(json.dumps(dict(status='DRY_RUN',motor_commands_sent=0,joint_time_law='septic' if a.legacy_motion else 'cruise_c3')));return
    sys.path[:0]=[str(a.central_root/'contest/arm_runtime'),str(a.central_root/'contest/kv260_can/software')]
    run(a.app_root,a.central_root,a.count,a.destination_slots,a.offline_check,a.legacy_motion,not a.no_speculative_grasp,a.prepare_only,a.parts,0. if a.no_cooling_wait else 600.,not a.skip_thermal_gate,a.size,a.replan_moved,a.continuous,a.track_only,a.reverse_small_nut)

if __name__=='__main__':main()
