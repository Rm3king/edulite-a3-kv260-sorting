"""Single-session image-bound wrist correction service; no direct hardware access."""
import base64,hashlib,json,subprocess,sys,threading,time,gzip,copy
from http.server import BaseHTTPRequestHandler,HTTPServer
from pathlib import Path
import cv2,numpy as np
from .wrist_nut_localization import locate,locate_near,track_template,locate_clipped_body,track_consensus
from .plan_service import PlanPool
from .wrist_screw_localization import locate_screw_near,axis_difference_deg,across_axis_error,visible_tip
from .wrist_m6_localization import locate_m6_near,SCREW_OPEN_PX
from .wrist_m6_shadow import locate_m6_screw_view
M6_SCREW_LENGTH_PX=108.   # full M6 screw body in the wrist image at hover height (10 frames of run 003951: 106-110 px)


def locate_m6_screw(gray,expected,radius):
    """locate_m6_near on the opened mask and on the plain mask; the candidate nearer the expected position wins.

    locate_m6_near alone returns whatever the opened mask yields first. The opening (19 px) can split a thin screw into two pieces that
    both fail the size limits, and then a different object that passes (2026-11-02 run 023344: a tray-edge blob cut by the image border,
    205 px from the expected position) wins, and the arm is steered to it. The plain mask sees the whole screw."""
    found,last=[],None
    for open_px in (SCREW_OPEN_PX,0):
        try:found.append(locate_m6_screw_view(gray,expected,radius,open_px))
        except ValueError as error:last=error
    if not found:raise last
    return min(found,key=lambda c:(c['gap'],c['distance']))


def m6_screw_consensus(frames, expected, radius, agree=15.):
    """At least two independently observed grip points must agree; retain rejection causes."""
    observations, failures = [], []
    for i, gray in enumerate(frames):
        try:observations.append(dict(locate_m6_screw(gray, expected, radius), frame_index=i))
        except ValueError as error:failures.append(f'frame {i}: {error}')
    group = max(([o for o in observations if np.linalg.norm(np.subtract(o['center_px'], c['center_px'])) <= agree]
                 for c in observations), key=len, default=[])
    if len(group) < 2:
        details = '; '.join(failures) or 'grip points disagree'
        raise ValueError(f'No two M6 screw frames agree (detected {len(observations)}/{len(frames)}; {details})')
    return group


class Servo:
    def __init__(self,root):
        self.root=Path(root);self.config=json.loads((self.root/'servo_config.json').read_text())
        self.template=None;self.offset=None;self.last_center=None;self.last_delta=np.zeros(2);self.next_iteration=0
        self.session=None;self.spec=None
        # Warm sdk/sim workers (same functions as the one-shot commands); off unless the config asks for them.
        self.pool=PlanPool(self.config['launcher'],self.config['python_paths']) if self.config.get('warm_services') else None
    def run_stage(self,name,*args,**kwargs):
        started=time.monotonic()
        try:return subprocess.run(*args,**kwargs)
        finally:self.timings[name]=time.monotonic()-started
    def stage(self,timings,name,args,timeout,spec=None):
        """Run one planning subprocess; a speculative run can be cancelled and killed."""
        started=time.monotonic();proc=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        if spec is not None:spec.procs.append(proc)
        try:output,error=proc.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            proc.kill();proc.communicate();raise
        finally:timings[name]=time.monotonic()-started
        if spec is not None and spec.cancelled.is_set():raise RuntimeError('Speculation cancelled')
        if proc.returncode:raise subprocess.CalledProcessError(proc.returncode,args,output,error)
        return output
    def stage_call(self,pair,kind,op,args,oneshot,name,timeout,timings,spec=None):
        """One planning stage: on a warm worker when there is one, else as the original one-shot subprocess."""
        if pair is None:return self.stage(timings,name,oneshot,timeout,spec)
        started=time.monotonic()
        try:result=pair[0 if kind=='sdk' else 1].call(op,args,timeout)
        finally:timings[name]=time.monotonic()-started
        if spec is not None and spec.cancelled.is_set():raise RuntimeError('Speculation cancelled')
        return result
    def build_grasp(self,request,folder,timings,spec=None):
        pair=self.pool.acquire() if self.pool is not None else None
        try:return self.build_grasp_on(pair,request,folder,timings,spec)
        finally:
            if pair is not None:self.pool.release(pair)
    def build_grasp_on(self,pair,request,folder,timings,spec=None):
        """Plan, camera-clearance check and review-bundle the grasp continuation for this exact q/prefix/correction."""
        folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
        requestfile=folder/'planning_input.json';requestfile.write_text(json.dumps(dict(request={k:v for k,v in request.items() if k!='images'})))
        out=folder/'grasp_plan';paths=self.config['python_paths'];launcher=self.config['launcher']
        bootstrap='import sys,runpy;sys.path[:0]='+repr(paths)+';runpy.run_module("workcell.wrist_servo_plan",run_name="__main__")'
        self.stage_call(pair,'sdk','plan_grasp',(self.root,requestfile,out),[launcher,'sdk','python','-c',bootstrap,str(self.root),str(requestfile),str(out),'--grasp'],'planning_s',5.,timings,spec)
        bootstrap='import sys,runpy;sys.path[:0]='+repr(paths)+';runpy.run_module("workcell.live_geometry",run_name="__main__")'
        self.stage_call(pair,'sim','geometry_live',(out,out/'workflow_config.json'),[launcher,'sim','python','-c',bootstrap,'--out',str(out),'--config',str(out/'workflow_config.json')],'geometry_s',8.,timings,spec)
        candidate=json.loads((out/'job.json').read_text());rows=json.loads((out/'collision_samples.json').read_text())['samples'];config=json.loads((self.root/'workflow_config.json').read_text());plane=config['trays']['A']['plane'];normal=np.array(plane['normal_base'])
        clearance=min(float(normal@(np.array(row['tcp_m'])+np.array(row['rotation'])[:,2]*.098)+plane['offset_base_m']-.075) for row in rows)
        if clearance<.005:raise ValueError('Camera housing floor clearance')
        candidate['geometry_audit'].update(wrist_camera_proxy_checked=True,camera_proxy_min_floor_clearance_m=clearance)
        (out/'job.json').write_text(json.dumps(candidate))
        bootstrap='import sys;sys.path[:0]='+repr(paths)+';from continuous_transfer import export_bundle;from joint_trajectory import JointPlanner;import json;from pathlib import Path;p=Path(sys.argv[1]);print(export_bundle(json.loads((p/"job.json").read_text()),JointPlanner(p/"profile.json"),p/"prepared_bundle.json"))'
        printed=self.stage_call(pair,'sdk','bundle',(out,),[launcher,'sdk','python','-c',bootstrap,str(out)],'bundle_s',3.,timings,spec)
        raw=(out/'prepared_bundle.json').read_bytes();sha=hashlib.sha256(raw).hexdigest()
        if printed.strip()!=sha:raise ValueError('Bundle digest mismatch')
        return dict(candidate=candidate,raw=raw,sha=sha)
    def start_speculation(self,request,move,delta,folder):
        """After a MOVE reply the next q/prefix/correction are already determined; plan the grasp from there while the arm moves.

        The evidence for this plan records the 5 px gate bound, not a measurement. The measured error is still
        required <=5 by the host action choice and by the board's validate_grasp_reply.
        """
        self.speculate(dict(q_rad=move['goal_rad'],cumulative_tool_xy_m=list(delta),executed_prefix=list(request['executed_prefix'])+list(move['segments'])),folder)
    def start_initial_speculation(self):
        """The first hover, its history and a zero correction are known before the arm moves: plan that grasp during the departure."""
        task=json.loads((self.root/'job.json').read_text())['tasks'][0]
        self.speculate(dict(q_rad=task['goal_rad'],cumulative_tool_xy_m=[0.,0.],executed_prefix=list(task['segments'])),self.root/'speculative_initial')
    def speculate(self,inputs,folder):
        if not (self.config.get('integrated_grasp') and self.config.get('speculative_grasp')):return
        spec=type('Speculation',(),{})();spec.inputs=inputs;spec.cancelled=threading.Event();spec.procs=[];spec.result=None;spec.error=None;spec.timings={}
        planning=dict(inputs,pixel_error=5.,pixel_error_basis='GATE_BOUND_SPECULATIVE_NOT_A_MEASUREMENT')
        def work():
            try:spec.result=self.build_grasp(planning,Path(folder)/'speculative',spec.timings,spec)
            except BaseException as error:spec.error=repr(error)
        spec.thread=threading.Thread(target=work,daemon=True);self.spec=spec;spec.thread.start()
    def cancel_speculation(self):
        spec,self.spec=self.spec,None
        if spec is not None:
            spec.cancelled.set()
            for proc in list(spec.procs):
                if proc.poll() is None:proc.kill()
    def take_speculation(self,request):
        """Return the precomputed grasp only for bit-identical q/prefix/correction; otherwise discard it."""
        spec=self.spec
        if spec is None:return None,'none'
        if any(request[k]!=spec.inputs[k] for k in spec.inputs):
            self.cancel_speculation();return None,'inputs_differ'
        spec.thread.join(timeout=12.)
        if spec.thread.is_alive() or spec.result is None:
            reason='unfinished' if spec.thread.is_alive() else 'failed:'+str(spec.error);self.cancel_speculation();return None,reason
        self.spec=None;return dict(spec.result,timings=dict(spec.timings)),'used'
    def respond(self,request):
        started=time.monotonic();self.timings={}
        keys=('nonce','job_sha256','frame_sha256','iteration')
        reply={k:request[k] for k in keys}
        if request['job_sha256']!=self.config['job_sha256'] or request['iteration']!=self.next_iteration:
            raise ValueError('Unexpected episode/iteration')
        if self.session is not None and self.session!=request['camera_session']:raise ValueError('Camera restarted')
        self.session=request['camera_session']
        frames=[]
        for raw,sha in zip(request['images'],request['frame_sha256']):
            data=base64.b64decode(raw,validate=True)
            if hashlib.sha256(data).hexdigest()!=sha:raise ValueError('Image digest mismatch')
            gray=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_GRAYSCALE)
            if gray is None or gray.shape!=(400,640):raise ValueError('Invalid wrist frame')
            frames.append(gray)
        if len(frames)!=3:raise ValueError('Three fresh frames required')
        raw_dir=self.root/'raw_frames'/f"it{request['iteration']:02d}_{time.monotonic_ns()}";raw_dir.mkdir(parents=True,exist_ok=True)
        for k,g in enumerate(frames):cv2.imwrite(str(raw_dir/f'frame_{k}.png'),g)
        cumulative=np.array(request['cumulative_tool_xy_m']);J=np.array(self.config['J_pixels_per_m'])
        coarse=False
        predicted=(np.array(self.config['coarse_center_px']) if self.last_center is None
                   else self.last_center+J@(cumulative-self.last_delta))
        screw=self.config.get('part_class','nut')=='screw'
        m6=self.config.get('part_size','m12')=='m6';m6nut=m6 and not screw   # bright M6 parts: PL-twin locator instead of the dark-body / template ones
        if self.template is None and not screw and not m6nut:
            detection=None;seed_frame=1
            for seed_frame in (1,0,2):
                try:detection=locate_near(frames[seed_frame],predicted);break
                except ValueError:pass
            if detection is None:
                observations=[locate_clipped_body(g,predicted) for g in frames]
                coarse=True
            if not coarse:
                center=np.array(detection['center_px'])
                x,y,w,h=detection['bbox'];margin=min(8,x-1,y-1,639-x-w,399-y-h);x-=margin;y-=margin;w+=2*margin;h+=2*margin
                if min(x,y)<=0 or x+w>=640 or y+h>=400:raise ValueError('Template clipped')
                self.template=frames[seed_frame][y:y+h,x:x+w].copy();self.offset=center-[x,y];predicted=center
                cv2.imwrite(str(self.root/'live_template.png'),self.template)
        if screw:
            # A screw is segmented afresh in every frame (no rotation-bound template); its centre is the middle of its extent.
            locate=locate_m6_screw if m6 else locate_screw_near
            if m6:
                # One frame can flip the head end (a glint fattens the thread end) or lose the screw: keep the frames whose grip points agree, at least two.
                try:
                    observations=m6_screw_consensus(frames,predicted,self.config.get('initial_search_radius_px',220.),
                                                   float(self.config.get('m6_screw_agree_px',15.)))
                except ValueError as error:
                    (raw_dir/'vision_rejection.json').write_text(json.dumps(dict(reason=str(error),predicted_px=np.asarray(predicted,float).tolist())))
                    raise
            else:observations=[dict(locate(g,predicted,self.config.get('initial_search_radius_px',220.)),frame_index=i) for i,g in enumerate(frames)]
            if max(axis_difference_deg(observations[0]['axis_deg'],o['axis_deg']) for o in observations)>(float(self.config.get('m6_axis_tol_deg',8.)) if m6 else 4.):raise ValueError('Screw axis unstable')  # M6 outline is short (about 100 px): a glint on it moves the principal axis by up to 6 degrees
        elif m6nut:
            observations=[]
            for i,g in enumerate(frames):
                try:observations.append(dict(locate_m6_near(g,predicted,'nut_m6',self.config.get('initial_search_radius_px',220.)),frame_index=i))
                except ValueError:pass
            # Tray glints flicker and merge with the nut in some frames (outline wider, centre 3-4 px off): keep the largest group of frames whose
            # centres agree within 2.5 px, the more compact group (higher fill) on a tie; at least two frames.
            agree=float(self.config.get('m6_nut_agree_px',2.5))
            groups=[[v for v in observations if np.linalg.norm(np.subtract(v['center_px'],c['center_px']))<=agree] for c in observations]
            observations=max(groups,key=lambda g:(len(g),np.mean([v['score'] for v in g])),default=[])
            if len(observations)<2:raise ValueError('Nut image unstable')
        elif not coarse:
            observations=track_consensus(frames,self.template,self.offset,predicted)
        centers=np.array([v['center_px'] for v in observations]);center=np.median(centers,axis=0)
        if screw:
            # A cut body is measured across its axis only; the along-axis position stays with the scene camera.
            screw_error,center,spread=across_axis_error(self.config['target_px'],observations,self.config.get('screw_length_px') or (M6_SCREW_LENGTH_PX if m6 else None))
            # The screw is accepted within 18 px across its axis (jaws and screw are forgiving), so 4 px of frame-to-frame scatter is tolerable; a nut keeps 1.5 px.
            if spread>float(self.config.get('m6_screw_stability_px',8.) if m6 else self.config.get('screw_stability_px',4.)):raise ValueError('Screw image unstable')  # M6: the grip point sits behind the head end, its along-axis scatter is far inside the 60 px along tolerance
        elif np.max(np.linalg.norm(centers-center,axis=1))>(float(self.config.get('m6_nut_stability_px',3.)) if m6nut else 1.5):raise ValueError('Nut image unstable')
        if not coarse and not screw and not m6nut:
            if max(v['score'] for v in observations)<.85:raise ValueError('No strong template support')
            best_observation=max(observations,key=lambda v:v['score']);best=best_observation['frame_index']
            matched=np.array(best_observation['center_px']);x,y=np.rint(matched-self.offset).astype(int);h,w=self.template.shape
            if x<=0 or y<=0 or x+w>=640 or y+h>=400:raise ValueError('Updated template clipped')
            self.template=frames[best][y:y+h,x:x+w].copy();self.offset=matched-[x,y]
        goal=np.array(self.config['target_px']);error=screw_error if screw else goal-center;norm=float(np.linalg.norm(error))
        screw_tolerance=None
        if screw:
            # A screw is gripped anywhere along its shaft, so it needs far less than a nut: the jaws need the shaft between
            # them (across) and the grip somewhere central (along). The board's <=5 gate stays untouched for every part;
            # the reported number is the error in units of this tolerance scaled so that 5 means "inside it".
            axis=np.deg2rad(float(np.median([o['axis_deg'] for o in observations])));along_dir=np.array([np.cos(axis),np.sin(axis)])
            across_tol=float(self.config.get('screw_across_tol_px',18.));along_tol=float(self.config.get('screw_along_tol_px',60.))
            across_px=abs(float(error@np.array([-along_dir[1],along_dir[0]])));along_px=abs(float(error@along_dir))
            screw_tolerance=dict(across_px=across_px,along_px=along_px,across_tol_px=across_tol,along_tol_px=along_tol,raw_error_px=norm)
            norm=5.*max(across_px/across_tol,along_px/along_tol)
            if m6 and any(o['cut'] for o in observations) and any(visible_tip(o) is None for o in observations):
                # A cut M6 screw whose end is not in the image gives only its across-axis offset; the along-axis error was removed, so
                # the pose looks converged while the jaws may be a screw length away (run 023344: empty close, 26 mm along the axis).
                # Never grasp on that: keep moving; the iteration limit then returns the arm safely.
                norm=max(norm,5.5)
        self.last_center=center;self.last_delta=cumulative.copy();self.next_iteration+=1
        reply.update(center_px=center.tolist(),pixel_error=norm,observations=observations,target_px=goal.tolist(),target_is_provisional=True,coarse_only=coarse)
        if screw:reply.update(pixel_error_basis='SCREW_TOLERANCE_NORMALIZED',screw_tolerance=screw_tolerance,part_class='screw',axis_deg=float(np.median([o['axis_deg'] for o in observations])),body_cut=any(o['cut'] for o in observations))
        if not (norm<=5 and not coarse):self.cancel_speculation()
        index=request['iteration'];folder=self.root/f'iteration_{index:02d}'
        if folder.exists():folder=self.root/f'iteration_{index:02d}_retry_{time.monotonic_ns()}'
        folder.mkdir(exist_ok=False)
        for k,g in enumerate(frames):cv2.imwrite(str(folder/f'frame_{k}.png'),g)
        (folder/'request.json').write_text(json.dumps({k:v for k,v in request.items() if k!='images'}))
        # The search centre this iteration used: lets the board-side (PL) locator be replayed on the same frames.
        (folder/'vision_input.json').write_text(json.dumps(dict(predicted_px=np.asarray(predicted,float).tolist(),screw=screw,
            radius_px=float(self.config.get('initial_search_radius_px',220.)))))
        self.timings['vision_and_evidence_s']=time.monotonic()-started
        if norm<=5 and not coarse:
            reply['action']='CONVERGED'
            if self.config.get('integrated_grasp'):
                built,why=self.take_speculation(request)
                if built is None:built=self.build_grasp(dict(request,pixel_error=norm),folder,self.timings)
                else:self.timings.update(('speculative_'+k,v) for k,v in built['timings'].items())
                reply.update(action='GRASP',candidate=built['candidate'],cumulative_tool_xy_m=cumulative.tolist(),reviewed_bundle_b64=base64.b64encode(gzip.compress(built['raw'])).decode(),reviewed_bundle_sha256=built['sha'],speculation=why)
        elif index>=4:reply.update(action='RETURN',reason='Iteration limit')
        else:
            step=np.linalg.solve(J,error);step*=min(1.,.012/max(np.linalg.norm(step),1e-12))
            delta=cumulative+step
            if np.linalg.norm(delta+np.array(self.config.get('hover_bias_tool_xy_m',[0.,0.])))>self.config.get('max_correction_m',.025):reply.update(action='RETURN',reason='Correction envelope exceeded')
            else:
                requestfile=folder/'planning_input.json';requestfile.write_text(json.dumps(dict(request=request,delta=delta.tolist())))
                out=folder/'plan'
                paths=self.config['python_paths'];bootstrap='import sys,runpy;sys.path[:0]='+repr(paths)+';runpy.run_module("workcell.wrist_servo_plan",run_name="__main__")'
                pair=self.pool.acquire() if self.pool is not None else None
                try:
                    self.stage_call(pair,'sdk','plan_move',(self.root,requestfile,out),[self.config['launcher'],'sdk','python','-c',bootstrap,str(self.root),str(requestfile),str(out)],'planning_s',3.,self.timings)
                    # The next hover is fixed as soon as the step is planned: start the grasp planning now and let it
                    # run through this step's geometry review, the board's validation and the arm's own motion.
                    # job.json only exists after the geometry stage; the planned tasks are the same text in the pending file.
                    self.start_speculation(request,json.loads((out/'job_pending_geometry.json').read_text())['tasks'][0],delta.tolist(),folder)
                    try:
                        bootstrap='import sys,runpy;sys.path[:0]='+repr(paths)+';runpy.run_module("workcell.wrist_handeye_geometry",run_name="__main__")'
                        self.stage_call(pair,'sim','geometry_wrist',(out,self.root/'workflow_config.json'),[self.config['launcher'],'sim','python','-c',bootstrap,'--out',str(out),'--config',str(self.root/'workflow_config.json')],'geometry_s',3.,self.timings)
                        reply.update(action='MOVE',candidate=json.loads((out/'job.json').read_text()),cumulative_tool_xy_m=delta.tolist())
                    except BaseException:
                        self.cancel_speculation();raise
                finally:
                    if pair is not None:self.pool.release(pair)
        reply['host_timing_s']=dict(self.timings,total=time.monotonic()-started)
        (folder/'reply.json').write_text(json.dumps(reply));return reply


def rebase_reply(root,request,servos,wait_s=38.):
    """Answer the board at the previous exit pose: KEEP, or REPLAN with the job the trial re-planned after the part was moved.
    The trial writes `rebase_decision.json` into this part's folder; a new Servo for the re-planned folder is registered by its
    job digest, so visual requests bound to either job (the board may still refuse the new one) are answered consistently."""
    path=Path(root)/'rebase_decision.json';deadline=time.monotonic()+wait_s
    while not path.exists():
        if time.monotonic()>deadline:return dict(nonce=request['nonce'],action='KEEP',reason='no decision in time')
        time.sleep(.05)
    decision=json.loads(path.read_text())
    if decision.get('action')!='REPLAN':return dict(nonce=request['nonce'],action='KEEP',reason=decision.get('reason') or decision.get('error'))
    folder=Path(decision['folder'])
    servo=Servo(folder);servo.start_initial_speculation();servos[decision['job_sha256']]=servo
    return dict(nonce=request['nonce'],action='REPLAN',job_sha256=decision['job_sha256'],bundle_sha256=decision['bundle_sha256'],moved_px=decision.get('moved_px'),
                job_gz_b64=base64.b64encode(gzip.compress((folder/'job.json').read_bytes())).decode(),
                bundle_gz_b64=base64.b64encode(gzip.compress((folder/'servo_initial_bundle.json').read_bytes())).decode())


def serve(root,address='192.168.26.1',port=18768):
    first=Servo(root);first.start_initial_speculation();servos={first.config['job_sha256']:first}
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            started=time.monotonic()
            size=int(self.headers.get('Content-Length','0'))
            if self.path.rstrip('/').endswith('rebase'):
                try:
                    if not 0<size<10000:raise ValueError('Request size')
                    reply=rebase_reply(root,json.loads(self.rfile.read(size)),servos);data=json.dumps(reply).encode();self.send_response(200)
                    print(json.dumps(dict(rebase=reply['action'],moved_px=reply.get('moved_px'),reason=reply.get('reason'),seconds=time.monotonic()-started)),flush=True)
                except Exception as error:
                    data=json.dumps(dict(error=repr(error))).encode();self.send_response(422);print(data.decode(),flush=True)
                self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
                return
            body=self.rfile.read(size) if 0<size<600000 else b''
            try:servo=servos.get(json.loads(body).get('job_sha256'),first)
            except Exception:servo=first
            state={k:copy.deepcopy(getattr(servo,k)) for k in ('template','offset','last_center','last_delta','next_iteration','session')}
            try:
                if not 0<size<600000:raise ValueError('Request size')
                reply=servo.respond(json.loads(body));data=json.dumps(reply).encode();self.send_response(200)
                print(json.dumps(dict(iteration=reply['iteration'],action=reply['action'],error=reply['pixel_error'],seconds=time.monotonic()-started)),flush=True)
            except Exception as error:
                for k,v in state.items():setattr(servo,k,v)
                data=json.dumps(dict(error=repr(error))).encode();self.send_response(422);print(data.decode(),flush=True)
            self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
        def log_message(self,*unused):pass
    server=HTTPServer((address,port),Handler);print('READY',flush=True)
    try:server.serve_forever()
    finally:
        for servo in servos.values():
            if servo.pool is not None:servo.pool.close()

if __name__=='__main__':
    from .proc_guard import exit_with_parent
    exit_with_parent()   # never outlive the trial that started this service (with its workers: one process group)
    serve(sys.argv[1],port=int(sys.argv[2]) if len(sys.argv)>2 else 18768)
