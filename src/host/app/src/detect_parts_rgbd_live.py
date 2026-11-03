"""Live RGB-D screw/nut surface XYZ preview. Default dry-run; no robot API."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import time
import cv2
import numpy as np
from detect_parts_live import Detector, NpuDetector, check_model, configure_window
from live_part_geometry import load_transform, locate_parts
from workcell import instance_masks

ROOT = Path(__file__).resolve().parents[2]
CALIBRATION = ROOT/'aipc_app/reports/q18_wide_20261023_165239/camera_to_base_candidate.json'
TCP = ROOT/'aipc_app/configs/grasp_tcp_provisional_20261023.json'
WINDOW = 'D435i RGB-D | screw + nut | base XYZ (mm)'


def atomic_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    temporary.replace(path)


def draw(image, objects, infer_ms, age_ms):
    canvas = cv2.resize(image, (960, 540)); sx=960/image.shape[1]; sy=540/image.shape[0]
    panel = np.full((540, 400, 3), 245, np.uint8)
    for i, obj in enumerate(objects):
        color={'screw':(30,220,30),'nut':(0,150,255),'screw_m6':(255,200,40),'nut_m6':(255,60,200)}[obj['part_class']]
        x1,y1,x2,y2=obj['box_xyxy_px']; a=(int(x1*sx),int(y1*sy));b=(int(x2*sx),int(y2*sy))
        cv2.rectangle(canvas,a,b,color,2)
        label=f'{i+1} {obj["part_class"]} {obj["confidence"]:.2f}'
        cv2.putText(canvas,label,(a[0],max(16,a[1]-4)),cv2.FONT_HERSHEY_SIMPLEX,.45,color,1,cv2.LINE_AA)
        if obj.get('center_uv_px'):
            u,v=obj['center_uv_px'];cv2.circle(canvas,(int(u*sx),int(v*sy)),4,color,-1)
        if i<8:
            y=28+i*62
            cv2.putText(panel,label,(8,y),cv2.FONT_HERSHEY_SIMPLEX,.5,(20,20,20),1,cv2.LINE_AA)
            if obj['geometry_valid']:
                x,yb,z=np.array(obj['base_xyz_m'])*1000;line=f'X {x:+.1f}  Y {yb:+.1f}  Z {z:+.1f} mm'
            else:line='No valid surface XYZ'
            cv2.putText(panel,line,(8,y+24),cv2.FONT_HERSHEY_SIMPLEX,.46,(20,20,20),1,cv2.LINE_AA)
    view=cv2.copyMakeBorder(np.hstack([canvas,panel]),82,0,0,0,cv2.BORDER_CONSTANT,value=(245,245,245))
    lines=[f'Base-frame surface estimates | {len(objects)} detections | inference {infer_ms:.0f} ms | processing age {age_ms:.0f} ms',
           'Preview only | camera extrinsics: candidate | TCP: CAD 98 mm, unmeasured',
           'Q / Esc: close | S: save current RGB-D snapshot | labels renumber each frame']
    for i,line in enumerate(lines):cv2.putText(view,line,(10,22+24*i),cv2.FONT_HERSHEY_SIMPLEX,.48,(25,25,25),1,cv2.LINE_AA)
    return view


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',action='store_true');ap.add_argument('--headless',action='store_true')
    ap.add_argument('--seconds',type=float,default=0);ap.add_argument('--device',choices=['gpu','cpu'],default='gpu')
    # npu: the AMD Quark INT8 export of the same checkpoint on the Ryzen AI NPU (GPU left to the local VLM); the GPU detector
    # stays loaded and takes over on any NPU failure (reports/npu_parts_v2_20261031: 270/270 grasp targets, p95 1.1 mm)
    ap.add_argument('--backend',choices=['gpu','npu'],default='gpu')
    # The D435i's IR projector pattern (a dense dot field) is seen by the wrist camera and disturbs the M6 locators.
    # 'off' switches the projector off for this run (depth then comes from the scene's own texture).
    ap.add_argument('--emitter',choices=['default','off'],default='default')
    ap.add_argument('--calibration',type=Path,default=CALIBRATION)
    ap.add_argument('--model-plan',type=Path,help='Explicit versioned model/hash/confidence JSON; default preserves the old model.')
    ap.add_argument('--out',type=Path)
    ap.add_argument('--archive-frames',type=int,default=0,help='Save up to 120 diagnostic RGB-D frames; default off.')
    ap.add_argument('--archive-interval',type=float,default=0.5)
    a=ap.parse_args()
    if not np.isfinite(a.seconds) or a.seconds<0:ap.error('seconds must be nonnegative')
    if a.headless and a.seconds<=0:ap.error('headless check requires finite --seconds')
    if not 0<=a.archive_frames<=120:ap.error('archive-frames must be between 0 and 120')
    if not np.isfinite(a.archive_interval) or a.archive_interval<0.2:ap.error('archive-interval must be at least 0.2 seconds')
    calibration,matrix=load_transform(a.calibration);checkpoint,plan=check_model(a.model_plan)
    tcp=json.loads(TCP.read_text());policy=json.loads((ROOT/'aipc_app/configs/parts_depth_draft_v1.json').read_text())
    if not a.run:
        print(json.dumps(dict(status='DRY_RUN_NO_CAMERA_NO_ROBOT',model_sha256=plan['checkpoint_sha256'],calibration_id=calibration['calibration_id'],tcp=tcp)));return
    if not a.headless:configure_window()
    out=(a.out or ROOT/'aipc_app/reports'/('rgbd_parts_'+datetime.now().strftime('%Y%m%d_%H%M%S'))).resolve()
    out.mkdir(parents=True,exist_ok=False)
    for source in [Path(__file__),Path(__file__).with_name('live_part_geometry.py'),Path(instance_masks.__file__),a.calibration,TCP]:
        (out/source.name).write_bytes(source.read_bytes())
    atomic_json(out/'model_plan.json',plan)
    report=dict(status='STARTING',robot_commands_sent=0,robot_connection_opened=False,calibration_verified=False,
                calibration_id=calibration['calibration_id'],calibration_sha256=hashlib.sha256(a.calibration.read_bytes()).hexdigest(),
                detector_sha256=plan['checkpoint_sha256'],tcp=tcp,frames_processed=0,camera_stream_stopped=True,
                depth_alignment='color',objects_are_surface_estimates=True,
                archive_frames_requested=a.archive_frames,archive_frames_saved=0,archive_interval_s=a.archive_interval,
                static_scene_confirmed=False)
    atomic_json(out/'report.json',report)
    detector=Detector(checkpoint,plan,a.device)
    if a.backend=='npu':
        if plan['checkpoint_sha256']!='bbe3f4000909d5ea22fd8c0017db7e004d95f5ba8b687a56cb0da7f787e25c88':
            raise ValueError('The NPU model was quantized from parts_v2 interim1 only; this plan names another checkpoint')
        detector=NpuDetector(fallback=detector)
    # Warm up without owning the camera; weights and environment stay unchanged.
    detector.predict(np.zeros((1080,1920,3),np.uint8))
    report['environment']=detector.environment
    import pyrealsense2 as rs
    context=rs.context();devices=context.query_devices()
    if len(devices)!=1:raise RuntimeError('Expected one D435i')
    serial=devices[0].get_info(rs.camera_info.serial_number)
    # Both sensors stamp frames with the device's own clock: the host-mapped global time of the colour sensor drifted by tens of
    # seconds within 35-90 min (2026-10-29), which made every frame "unsynchronized" and the detector report nothing.
    report['global_time_disabled']=[]
    for sensor in devices[0].query_sensors():
        if sensor.supports(rs.option.global_time_enabled):
            sensor.set_option(rs.option.global_time_enabled,0);report['global_time_disabled'].append(sensor.get_info(rs.camera_info.name))
    config=rs.config();config.enable_device(serial)
    config.enable_stream(rs.stream.color,1920,1080,rs.format.bgr8,30)
    config.enable_stream(rs.stream.depth,848,480,rs.format.z16,30)
    pipe=rs.pipeline(context);queue=rs.frame_queue(1);align=rs.align(rs.stream.color)
    started=False;stop=[False];last_save=0.;first_snapshot=False;last_log=0.;last_archive=0.;snapshot_requested=[False]
    def request_stop(*unused):stop[0]=True
    def request_snapshot(*unused):snapshot_requested[0]=True
    signal.signal(signal.SIGTERM,request_stop);signal.signal(signal.SIGINT,request_stop)
    signal.signal(signal.SIGUSR1,request_snapshot)
    report['snapshot_signal']='SIGUSR1'
    try:
        profile=pipe.start(config,queue);started=True;report['camera_stream_stopped']=False
        if a.emitter=='off':
            depth_sensor=profile.get_device().first_depth_sensor()
            depth_sensor.set_option(rs.option.emitter_enabled,0)
            report['emitter_enabled']=depth_sensor.get_option(rs.option.emitter_enabled)
        else:report['emitter_enabled']='device default'
        for sensor in profile.get_device().query_sensors():   # again after start: the pipeline may restore the default
            if sensor.supports(rs.option.global_time_enabled):sensor.set_option(rs.option.global_time_enabled,0)
        intr=profile.get_stream(rs.stream.color).as_video_stream_profile().intrinsics
        intrinsics=dict(width=intr.width,height=intr.height,fx=intr.fx,fy=intr.fy,cx=intr.ppx,cy=intr.ppy,coeffs=list(intr.coeffs),distortion=str(intr.model))
        original=calibration['camera_intrinsics']
        if any(not np.isclose(intrinsics[k],original[k],rtol=0,atol=1e-4) for k in ['width','height','fx','fy','cx','cy']) or intrinsics['coeffs']!=original['coeffs']:
            raise RuntimeError('Live color intrinsics differ from calibration capture')
        scale=profile.get_device().first_depth_sensor().get_depth_scale()
        report.update(camera_serial=serial,usb=profile.get_device().get_info(rs.camera_info.usb_type_descriptor),intrinsics=intrinsics,depth_scale_m=scale)
        for _ in range(30):queue.wait_for_frame(5000)
        begin=time.monotonic();report['status']='RUNNING_RGBD_COORDINATE_PREVIEW';atomic_json(out/'report.json',report)
        if not a.headless:cv2.namedWindow(WINDOW,cv2.WINDOW_AUTOSIZE)
        window_open=[not a.headless]
        print(json.dumps(dict(event='RGBD_PREVIEW_STARTED',output=str(out),usb=report['usb'],robot_connected=False)),flush=True)
        color_seen=[None,time.monotonic()]
        while not stop[0] and (not a.seconds or time.monotonic()-begin<a.seconds):
            frames=align.process(queue.wait_for_frame(5000).as_frameset());color=frames.get_color_frame();depth_frame=frames.get_depth_frame()
            if not color or not depth_frame:continue
            # The colour stream of this D435i stops now and then (2026-10-29, three times in 4 h) while depth goes on; the syncer then hands out
            # the last colour frame forever and every frame is "unsynchronized". A colour frame number that stays still for 3 s restarts the pipeline.
            if color.get_frame_number()!=color_seen[0]:color_seen[:]=[color.get_frame_number(),time.monotonic()]
            elif time.monotonic()-color_seen[1]>3.:
                report['color_stream_restarts']=report.get('color_stream_restarts',0)+1
                print(json.dumps(dict(event='COLOR_STREAM_STALLED_RESTART',frame_number=color_seen[0],restarts=report['color_stream_restarts'])),flush=True)
                pipe.stop();time.sleep(1.);profile=pipe.start(config,queue)
                for sensor in profile.get_device().query_sensors():
                    if sensor.supports(rs.option.global_time_enabled):sensor.set_option(rs.option.global_time_enabled,0)
                for _ in range(10):queue.wait_for_frame(5000)
                color_seen[:]=[None,time.monotonic()];continue
            acquired=time.monotonic();bgr=np.asanyarray(color.get_data()).copy();depth=np.asanyarray(depth_frame.get_data()).copy()
            domain_match=color.get_frame_timestamp_domain()==depth_frame.get_frame_timestamp_domain()
            skew=abs(color.get_timestamp()-depth_frame.get_timestamp())
            prediction,infer_ms=detector.predict(bgr)
            objects=locate_parts(prediction,depth,scale,intrinsics,policy,matrix) if domain_match and skew<=50 else []
            now=time.monotonic();age_ms=(now-acquired)*1000
            snapshot=dict(schema='aipc.live_surface_points.v1',frame_number=color.get_frame_number(),color_timestamp_ms=color.get_timestamp(),
                          depth_timestamp_ms=depth_frame.get_timestamp(),timestamp_domain=str(color.get_frame_timestamp_domain()),rgb_depth_skew_ms=skew,
                          synchronized=domain_match and skew<=50,host_acquired_monotonic_s=acquired,host_processed_monotonic_s=now,
                          processing_age_ms=age_ms,inference_ms=infer_ms,inference_backend=getattr(detector,'active','gpu'),calibration_id=calibration['calibration_id'],calibration_verified=False,
                          tcp_physically_calibrated=False,robot_grasp_allowed=False,objects=objects)
            report['frames_processed']+=1
            report['last_detection_counts']={name:sum(o['part_class']==name for o in objects) for name in ['screw','nut','screw_m6','nut_m6']}
            report['last_valid_xyz_counts']={name:sum(o['part_class']==name and o['geometry_valid'] for o in objects) for name in ['screw','nut','screw_m6','nut_m6']}
            report['last_inference_ms']=infer_ms
            view=draw(bgr,objects,infer_ms,age_ms)
            key=-1
            if window_open[0]:
                # Esc / q still stop the detector. A preview window that disappears (closed by hand, 2026-11-01 02:51: cv2.error NULL guiReceiver)
                # no longer takes the detector down with it: the run needs the detections, not the picture.
                try:
                    cv2.imshow(WINDOW,view);key=cv2.waitKey(1)&255
                    if key in [27,ord('q'),ord('Q')]:stop[0]=True
                    elif cv2.getWindowProperty(WINDOW,cv2.WND_PROP_VISIBLE)<1:raise cv2.error('preview window closed')
                except cv2.error as error:
                    window_open[0]=False;report['preview_window_lost']=repr(error)[:160]
                    print(json.dumps(dict(event='PREVIEW_WINDOW_LOST',detector_continues_headless=True)),flush=True)
            if now-last_save>=1 or not first_snapshot:
                atomic_json(out/'latest.json',snapshot);atomic_json(out/'report.json',report);cv2.imwrite(str(out/'latest_preview.jpg'),view);last_save=now
            if not first_snapshot or key in [ord('s'),ord('S')] or snapshot_requested[0]:
                prefix='initial' if not first_snapshot else f'frame_{color.get_frame_number()}'
                if not cv2.imwrite(str(out/(prefix+'_color.png')),bgr):raise OSError('Failed to save RGB snapshot')
                np.save(out/(prefix+'_depth.npy'),depth)
                np.savez_compressed(out/(prefix+'_prediction.npz'),**prediction)
                atomic_json(out/(prefix+'_points.json'),snapshot);first_snapshot=True;snapshot_requested[0]=False
            if report['archive_frames_saved']<a.archive_frames and now-last_archive>=a.archive_interval:
                prefix=f"sample_{report['archive_frames_saved']+1:03d}"
                if not cv2.imwrite(str(out/(prefix+'_color.png')),bgr):raise OSError('Failed to archive RGB frame')
                np.save(out/(prefix+'_depth.npy'),depth)
                np.savez_compressed(out/(prefix+'_prediction.npz'),**prediction)
                atomic_json(out/(prefix+'_points.json'),snapshot)
                report['archive_frames_saved']+=1;last_archive=now
                atomic_json(out/'report.json',report)
            if now-last_log>=5:
                print(json.dumps(dict(event='RGBD_POINTS',frames=report['frames_processed'],counts=report['last_detection_counts'],valid_xyz=report['last_valid_xyz_counts'],inference_ms=round(infer_ms,1))),flush=True);last_log=now
        report['status']='RGBD_PREVIEW_STOPPED'
    except BaseException as error:
        report.update(status='FAILED',error=repr(error));raise
    finally:
        if started:pipe.stop()
        if hasattr(detector,'close'):detector.close()
        cv2.destroyAllWindows();report['camera_stream_stopped']=True;atomic_json(out/'report.json',report)
    print(json.dumps(report,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
