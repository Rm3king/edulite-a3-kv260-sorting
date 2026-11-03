"""Construct reproducible motion banks and collect a shared PL timing observer.

Bank construction, analysis, and serialization run only with motors disabled.
Both backends use exactly the same 220-word fixed-point references and gains.
"""
import argparse,copy,ctypes,gc,gzip,hashlib,json,math,os,pickle,signal,time
from pathlib import Path
import numpy as np
from joint_trajectory import JointPlanner
from motion_runtime import MotionRuntime
from motion_executor import Session
from a3_protocol import disable
from csp_commission import bits
from gravity_trajectory import GravityModel,prepare_stream,validate_records,hold_segment
from gravity_executor import GravityExecutor,CanDevice
from live_motion import current_reference,POLICY
from septic_trajectory import movej_c3,smooth_cartesian_c3
from blended_motion import blended_movej
from tuning_worker import guard_job,task
from gravity_transport import GravityTransport

BINS=[1,2,5,10,25,50,100,250,500,1000,2000,5000,10000,20000,50000,None]

def build(capture,profile,tracking,path,phase_validator=None,encoding_model=None):
 p=JointPlanner(profile);r=MotionRuntime();m=encoding_model or GravityModel(r,p,tracking)
 if m.p.profile_id!=p.profile_id:raise ValueError('encoding cache motion profile mismatch')
 if encoding_model is not None:
  if m.r.model_id!=r.model_id or m.id!=hashlib.sha256(Path(tracking).read_bytes()).hexdigest():raise ValueError('encoding cache robot/tracking profile mismatch')
 by={i:[x['decoded']['position_motor_rad'] for x in capture['transactions'] if x['motor']==i and x.get('decoded',{}).get('valid')] for i in range(1,8)}
 start,_=current_reference(p,[by[i][-1] for i in range(1,8)])
 anchor=np.deg2rad([-60,10,-15,15,20,-10]).tolist()+[1.6]
 if path=='pick':
  from pick_return import prepare
  job,audit=prepare(capture,p,r,.10,.08,.046)
  tasks=job['tasks']
  for t in tasks:t['gravity_alpha']=[1.,1.]
  # Stops here are necessary approach/empty close/lift/open task transitions.
 else:
  tasks=[task('Entry_current_to_anchor',movej_c3(p,start,anchor),anchor)]
  if path=='blend':
   pts=[anchor,np.deg2rad([-50,18,-26,22,27,-4]).tolist()+[1.6],np.deg2rad([-65,25,-35,10,34,-15]).tolist()+[1.6],anchor]
   plans,audit=blended_movej(p,pts);tasks.append(task('Continuous_joint_waypoints',plans,anchor))
  elif path=='circle':
   circle=smooth_cartesian_c3(r,p,anchor,radius=.04,axes=(0,2),tcp_speed=.03)
   audit={k:v for k,v in circle.items() if k!='segments'}
   tasks.append(task('XZ_circle_80mm',circle['segments'],circle['goal_rad']))
  else:raise ValueError('unknown path')
  tasks.append(task('Return_to_current_supported_start',movej_c3(p,tasks[-1]['goal_rad'],start),start))
  job=guard_job(p,tasks)
  job.update(start_policy=POLICY,start_rad=start,start_motor_rad=[by[i][-1] for i in range(1,8)],profile_id=p.profile_id,model_id=r.model_id)
 final=tasks[-1]['goal_rad'];tasks.append(task('Supported_endpoint_hold',[hold_segment(p,final,.3)],final))
 records=prepare_stream(m,tasks);validate_records(records,job,p,m,phase_validator=phase_validator)
 sha=hashlib.sha256(np.asarray([x['words'] for x in records],dtype='<i8').tobytes()).hexdigest()
 return dict(job=job,records=records,audit=audit,path=path,reference_sha256=sha,
    duration_s=sum((x['words'][217]+1)*.005 for x in records),profile_id=p.profile_id,model_id=r.model_id,tracking_profile_id=m.id)

def timing_read(d):
 if d.read(0x100)!=0x54494d31:raise RuntimeError('timing observer absent')
 d.write(0x104,0);axes=[]
 for axis in range(7):
  d.write(0x108,axis);n=d.read(0x114)
  total=d.read(0x124)|(d.read(0x128)<<32);square=d.read(0x12c)|(d.read(0x130)<<32)
  hist=[d.read(0x180+4*k) for k in range(16)]
  def quantile(frac):
   count=0
   for lim,h in zip(BINS,hist):
    count+=h
    if count>=math.ceil(n*frac):return lim
  axes.append(dict(motor=axis+1,frames=d.read(0x110),intervals=n,min_interval_us=d.read(0x118),max_interval_us=d.read(0x11c),
    max_abs_period_error_us=d.read(0x120),mean_abs_period_error_us=total/n if n else None,rms_period_error_us=math.sqrt(square/n) if n else None,
    p99_abs_period_error_upper_bound_us=quantile(.99) if n else None,histogram=hist,histogram_upper_bounds_us=BINS,
    max_due_to_command_request_us=d.read(0x134),intervals_above_1_5_period=d.read(0x13c),intervals_abs_error_gt50_us=d.read(0x140)))
 return dict(timestamp_event='CAN_CORE_TRANSMIT_COMMAND_REQUEST_NOT_WIRE_SOF',period_us=d.read(0x10c),axes=axes)

def native_library():
 lib=ctypes.CDLL(str(Path(__file__).with_name('libnative_reference.so')))
 lib.native_submit.argtypes=[ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32),ctypes.c_uint32,ctypes.c_uint32]
 lib.native_packets.argtypes=[ctypes.POINTER(ctypes.c_int32),ctypes.c_uint32,ctypes.POINTER(ctypes.c_uint32)]
 lib.native_upload.argtypes=[ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
 return lib

class NativeSegmentTransport(GravityTransport):
 def __init__(self,d,p,records):
  super().__init__(d,p,records);self.lib=native_library()
  self.arrays=[(ctypes.c_int32*220)(*r['words']) for r in records]
 def submit(self,plan):
  i=self.prepared_index
  if i>=len(self.prepared) or self.prepared[i]['original_plan'] is not plan:raise ValueError('unreviewed segment order')
  rc=self.lib.native_upload(self.dev.base,self.arrays[i])
  if rc:raise RuntimeError('native segment upload '+str(rc))
  r=self.prepared[i];self.tail=r['ends'];self.tail_velocity=r['end_velocity'];self.last_encoded_plan=r['plan'];self.prepared_index+=1

def following_guard(e,record,tick):
 """Same existing following envelope for BOTH producers, at 50Hz."""
 now=time.monotonic()
 if now-getattr(e,'last_common_guard',0)<.02:return
 e.last_common_guard=now;u=min(1.,tick/record['plan']['ticks'])
 for axis,row in enumerate(record['plan']['motor_coefficients_rad'][:6]):
  target=0.
  for c in reversed(row):target=target*u+c
  error=abs(e.s.require(axis+1,2)['position']-target)
  if error>e.model.config['following_envelope_rad']:
   e.following_since.setdefault(axis+1,now)
   if error>.15 or now-e.following_since[axis+1]>.1:raise RuntimeError('sustained benchmark following error')
  else:e.following_since.pop(axis+1,None)

def pl_run(e,bank,report):
 """Service CAN continuously; observe trajectory phase only at 50Hz."""
 d=e.d;records=bank['records'];e.release_host_hold();d.write(0x104,3);report['controller_before_path']=d.status()
 begin=time.monotonic();cpu=time.process_time();phase_last=0.;submitted=0
 e.state='MOVING';e.event('PATH_START',segments=len(records),phase_observation_hz=50)
 deadline=begin+bank['duration_s']+10.
 while time.monotonic()<deadline:
  e.inject_stall_if_due();e.s.pump(.0015);flags=d.read(0xb40)
  if not flags&2 and submitted<len(records):
   r=records[submitted];e.transport.submit(r['original_plan']);submitted+=1;e.submitted+=1
   report.setdefault('segment_submissions',[]).append(dict(ordinal=submitted,plan=r['original_plan'],encoded_plan=r['plan'],time=time.monotonic()))
   if not e.started:e.transport.start();e.started=True
  now=time.monotonic()
  if now-phase_last>=.02:
   phase_last=now;s=e.transport.status();due=d.read(0xb58)
   if d.read(0xb4c)==s['completed']:
    report.setdefault('pl_samples',[]).append(dict(time=now,**s,next_due_us=due))
    i=min(s['completed'],len(records)-1);following_guard(e,records[i],s['tick'])
   if s['holding'] and s['completed']>=len(records):break
 else:raise TimeoutError('benchmark PL trajectory')
 e.state='HOLDING';e.expected=[x/(1<<24) for x in e.transport.tail]
 e.s.pump(.075);report['control_cpu_seconds']=time.process_time()-cpu;report['motion_wall_seconds']=time.monotonic()-begin

def initialize_disabled_csp(p,job):
 """Normalize a power-on factory mode before the existing MIT entry.

 No target, enable, zero, or Flash command. All mode writes are verified while
 drives and controller are inactive; the normal executor still owns enabling.
 """
 d=CanDevice();report=dict(commands=[],frames=[],feedback=[],readbacks={},reset_confirmed=[],stop_errors={})
 s=Session(d,report);owned=False
 try:
  if d.read(0x404) or d.read(0x5a8):raise RuntimeError('boot-mode initialization requires inactive controller')
  owned=True
  d.write(0x5a0,127);d.write(0x5a4,100000)
  for mid in range(1,8):
   i=mid-1;b=d.pp_base(mid)
   for off,value in [(0,1),(4,bits(job['motor_lower_rad'][i])),(8,bits(job['motor_upper_rad'][i])),
     (12,bits(p.csp_velocity[i])),(16,bits(p.acceleration[i])),(20,bits(p.current[i])),(28,1)]:d.write(b+off,value)
  d.configure('normal')
  for mid in range(1,8):s.send(disable(mid));s.pump(.01);s.require(mid,0)
  report['mode_before']={str(mid):s.read(mid,0x7005,True) for mid in range(1,7)}
  for mid in range(1,7):
   if report['mode_before'][str(mid)]!=5:s.write_verified(mid,0x7005,5,True)
  report['mode_after']={str(mid):s.read(mid,0x7005,True) for mid in range(1,7)}
  if any(v!=5 for v in report['mode_after'].values()):raise RuntimeError('CSP boot initialization readback')
  report['enable_commands_sent']=0;report['flash_writes']=False
 finally:
  if owned:
   d.stop();d.write(0x5a0,0)
   for mid in range(1,8):d.write(d.pp_base(mid),0)
  d.close()
 return report

def native_run(e,bank,report):
 lib=native_library();arrays=[(ctypes.c_int32*220)(*r['words']) for r in bank['records']]
 e.s.target_owner='native_reference';e.started=False
 # Drain preparation/holding traffic before starting the measured epoch.
 deadline=time.monotonic()+.05
 while e.d.read(0x41c)&65535 or e.d.read(0x464):
  e.s.pump(.0005)
  if time.monotonic()>deadline:raise TimeoutError('preparation queue drain')
 e.d.write(0x104,3);report['controller_before_path']=e.d.status();due=(e.d.read(0x408)+3000)&0xffffffff
 stamps=[];report['native_epochs']=[];report['native_due_us']=[];cpu=time.process_time();begin=time.monotonic()
 for ri,(record,arr) in enumerate(zip(bank['records'],arrays)):
  for tick in range(record['words'][217]+1):
   e.inject_stall_if_due()
   while True:
    now=e.d.read(0x408);left=((due-now+(1<<31))%(1<<32))-(1<<31)
    if left<=1000:break
    e.s.pump(min(.0005,(left-500)*1e-6))
   # Preserve every reference point on an exceptional Linux scheduling stall.
   # A rebase is explicitly logged, never counted as deadline compliance.
   if left< -1000:
    report.setdefault('native_schedule_rebases',[]).append(dict(record=ri,tick=tick,lateness_us=-left))
    due=(now+1000)&0xffffffff
   started=time.monotonic_ns();rc=lib.native_submit(e.d.base,arr,tick,due)
   if rc==-4:
    e.s.pump(.0005);rc=lib.native_submit(e.d.base,arr,tick,due)
   if rc:raise RuntimeError(f'native reference submission {rc}, record={ri},tick={tick}')
   stamps.append((time.monotonic_ns()-started)/1000.)
   report['native_last_cursor']=(ri,tick)
   if tick==0:report['native_epochs'].append(dict(record=ri,due_us=due))
   report['native_due_us'].append(due)
   due=(due+5000)&0xffffffff;e.s.pump(.0002);following_guard(e,record,tick)
 # Continue the same constant endpoint, retaining torque through observation.
 last=bank['records'][-1];arr=arrays[-1]
 for _ in range(15):
  while ((due-e.d.read(0x408)+(1<<31))%(1<<32))-(1<<31)>1000:e.s.pump(.0005)
  rc=lib.native_submit(e.d.base,arr,last['words'][217],due)
  if rc:raise RuntimeError('native supported hold submission '+str(rc))
  due=(due+5000)&0xffffffff;e.s.pump(.0002)
 report['control_cpu_seconds']=time.process_time()-cpu;report['motion_wall_seconds']=time.monotonic()-begin
 report['native_compute_guard_enqueue_samples_us']=stamps

def execute(bank,p,m,backend,output,host_stall_ms=0):
 import fcntl
 output=Path(output);report=dict(status='RUNNING',backend=backend,path=bank['path'],reference_sha256=bank['reference_sha256'],
  commands=[],frames=[],feedback=[],readbacks={},reset_confirmed=[],stop_errors={},flash_writes=False,events=[])
 report['host_environment']=dict(uname=list(os.uname()),cpu_affinity=sorted(os.sched_getaffinity(0)),
   cpufreq={str(p):p.read_text().strip() for p in Path('/sys/devices/system/cpu/cpufreq').glob('policy*/scaling_*') if p.name in ('scaling_governor','scaling_cur_freq','scaling_max_freq')})
 lock=open('/tmp/kv260_gripper_commission.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 from autotune import TuningExecutor
 class BenchmarkExecutor(TuningExecutor):
  def inject_stall_if_due(self):
   if getattr(self,'stall_begin',None) is not None and host_stall_ms and not getattr(self,'stall_injected',False) and time.monotonic()-self.stall_begin>=5.:
    self.stall_injected=True;begin=time.monotonic();time.sleep(host_stall_ms/1000.)
    report['controlled_host_stall']=dict(requested_ms=host_stall_ms,actual_ms=(time.monotonic()-begin)*1000,scope='EXPLICIT_MATCHED_FAULT_INJECTION_NOT_NORMAL_WORKLOAD')
  def pump(self,seconds=.002):
   self.inject_stall_if_due();return super().pump(seconds)
 report['disabled_boot_mode_initialization']=initialize_disabled_csp(p,bank['job'])
 d=CanDevice();e=BenchmarkExecutor(p,d,report,bank['records'],m)
 if backend=='pl':e.transport=NativeSegmentTransport(d,p,bank['records'])
 before_gc=gc.isenabled();gc.collect();gc.disable()
 # Do not save thousands of raw frames twice. Decoded feedback retains device
 # timestamps, mode/fault/temperature, angle/velocity/torque for each response.
 class Discard(list):
  def append(self,value):pass
 report['frames']=Discard()
 try:
  if d.read(0x100)!=0x54494d31:raise RuntimeError('timing observer must be verified before enabling')
  e.prepare(bank['job'],current_tracking=True)
  e.event('BENCHMARK_PATH_START',backend=backend,path=bank['path'],reference_sha256=bank['reference_sha256'])
  e.stall_begin=time.monotonic()
  if backend=='pl':
   pl_run(e,bank,report)
  elif backend=='arm':native_run(e,bank,report)
  else:raise ValueError('unknown backend')
  report['timing']=timing_read(d);report['controller_after_path']=d.status()
  report['supported_return_actual']={str(mid):dict(e.s.require(mid,2)) for mid in range(1,8)}
  error=max(abs(report['supported_return_actual'][str(i+1)]['position']-p.to_motor(bank['job']['start_rad'])[i]) for i in range(7))
  report['supported_return_max_error_rad']=error
  if error>m.config['endpoint_tolerance_rad']:raise RuntimeError('supported return endpoint outside tracking tolerance')
  report['status']='PASS_RETURNED_TO_SUPPORTED_START'
 except (Exception,KeyboardInterrupt) as exc:
  report.update(status='FAIL',error=repr(exc),failure_controller=d.status())
  if report.get('enable_attempted'):
   from control_continuity import retain_until_supported
   if backend=='pl':
    from control_continuity import finish_failed_job
    for t in bank['job']['tasks']:
     if t['name'].startswith('Return_'):t['return_to_live_start']=True
    finish_failed_job(e,bank['job'],output.parent,report)
   else:
    # Retain the most recently commanded posture while diagnosing a software
    # failure. Never fall through to reset an unsupported arm.
    import struct
    cursor=report.get('native_last_cursor')
    if cursor is not None:
     ri,tick=cursor;pack=(ctypes.c_uint32*21)();lib=native_library()
     lib.native_packets((ctypes.c_int32*220)(*bank['records'][ri]['words']),tick,pack)
     for axis in range(6):e.s.static[axis+1]=(pack[3*axis],struct.pack('>II',pack[3*axis+1],pack[3*axis+2]))
     e.s.targets[7]=struct.unpack('<f',struct.pack('>I',pack[20]))[0]
    e.s.target_owner='mit_host';retain_until_supported(e,output.parent,report)
 finally:
  if report.get('owned'):
   if report.get('enable_attempted'):
    try:e.s.stop_all()
    except Exception as exc:report['stop_errors']['fatal']=repr(exc)
   else:d.stop();d.write(0x5a0,0)
   if sorted(report['reset_confirmed'])==list(range(1,8)) and not report['stop_errors']:
    try:e.restore_csp_configuration()
    except Exception as exc:report.update(status='FAIL_RESTORE',restore_error=repr(exc))
   report['final_controller']=d.status()
  d.close();lock.close()
  if before_gc:gc.enable()
 # Evidence/analysis after supported return and reset only.
 stamps=report.pop('native_compute_guard_enqueue_samples_us',None)
 if stamps:report['native_compute_guard_enqueue_us']={k:float(np.percentile(stamps,q)) for k,q in [('p50',50),('p99',99),('max',100)]}
 with gzip.open(output,'wt') as f:json.dump(report,f)
 print(json.dumps({k:report.get(k) for k in ('status','error','backend','path','reset_confirmed','control_cpu_seconds','motion_wall_seconds','native_compute_guard_enqueue_us','supported_return_max_error_rad')},indent=2),flush=True)
 return report

if __name__=='__main__':
 a=argparse.ArgumentParser(description=__doc__);a.add_argument('command',choices=['build','execute']);a.add_argument('--capture');a.add_argument('--profile',required=True);a.add_argument('--tracking',required=True);a.add_argument('--path',choices=['pick','blend','circle']);a.add_argument('--backend',choices=['arm','pl']);a.add_argument('--bank',required=True);a.add_argument('--output');a.add_argument('--host-stall-ms',type=int,choices=[0,20],default=0);a.add_argument('--execute-attended',action='store_true');args=a.parse_args()
 if args.command=='build':
  b=build(json.loads(Path(args.capture).read_text()),args.profile,args.tracking,args.path)
  Path(args.bank).write_bytes(pickle.dumps(b,protocol=5));Path(args.bank).with_suffix('.review.json').write_text(json.dumps({k:v for k,v in b.items() if k not in ('job','records')},indent=2)+'\n')
  print(json.dumps(dict(path=b['path'],segments=len(b['records']),duration_s=b['duration_s'],reference_sha256=b['reference_sha256']),indent=2))
 else:
  if not args.execute_attended:a.error('explicit attended execution required')
  signal.signal(signal.SIGTERM,lambda *x:(_ for _ in ()).throw(KeyboardInterrupt('operator interrupt')))
  p=JointPlanner(args.profile);m=GravityModel(MotionRuntime(),p,args.tracking);bank=pickle.loads(Path(args.bank).read_bytes())
  if bank['profile_id']!=p.profile_id or bank['model_id']!=m.r.model_id or bank['tracking_profile_id']!=m.id:raise ValueError('bank/profile/model mismatch')
  report=execute(bank,p,m,args.backend,args.output,args.host_stall_ms);raise SystemExit(report['status']!='PASS_RETURNED_TO_SUPPORTED_START')
