#!/usr/bin/python3
"""Internal-only expiry and autonomous scheduling checks for firmware v2/v3/v4."""
import json,time
from canctl import CanDevice
from joint_safety import guard_words
from a3_protocol import motion,enable

x=CanDevice();r={'external_tx':'INHIBITED_INTERNAL_MODE','motion_sent_to_arm':False}
try:
 if x.read(0x470) not in (0x20000,0x30000,0x40000):raise RuntimeError('Expected guard firmware v2/v3/v4')
 x.stop()
 if x.read(0x5a0)!=0:raise RuntimeError('Expected default disabled motion mask')
 for mid in range(1,8):
  for packet in (motion(mid,0,0,0,0),enable(mid)):
   try:x.check_motion_guard(*packet)
   except ValueError:pass
   else:raise AssertionError('Default axis permission accepted by host preflight')
 r['default_motion_permission']={'host_preflight_rejections':14,'scope':'Actual PL mask read; no rejected MMIO write or motor command'}
 words=guard_words(1,-.1,.1,.1,.2,5,.5,33,14)
 saved=[x.read(0x600+i*4) for i in range(4)]
 try:
  for i,word in enumerate(words):x.write(0x600+i*4,word)
  if [x.read(0x600+i*4) for i in range(4)]!=words:raise AssertionError('Guard register readback')
  r['guard_configuration_readback']={'status':'PASS','motor':1,'words':words,'motion_permitted':False}
 finally:
  for i,word in enumerate(saved):x.write(0x600+i*4,word)
 x.configure('internal');before=x.read(0x460)
 x.heartbeat();x.enqueue(0x1100fd01,bytes.fromhex('1970000000000000'),ttl_us=10)
 time.sleep(.005)
 s=x.status()
 if x.read(0x418)!=4 or x.read(0x460)!=before+1 or x.read(0x404)!=0 or x.read(0x464)!=0:
  raise AssertionError(f'In-flight deadline failed: {s}')
 r['inflight_ttl_abort']='PASS'
 x.configure('internal')
 start=x.status();due=(x.read(0x408)+5000)&0xffffffff
 for i in range(8):x.enqueue(0x1200000+i,bytes([i])*8,(due+i*2000)&0xffffffff,1000)
 # No polling or heartbeat for 50 ms: PL must execute the preloaded 19 ms schedule.
 host_start=time.monotonic();time.sleep(.05);host_elapsed=time.monotonic()-host_start
 frames=[]
 while True:
  f=x.receive()
  if f is None:break
  frames.append(f)
 if len(frames)!=8 or x.read(0x420)-start['submitted']!=8:raise AssertionError('Autonomous queue did not complete')
 for i,f in enumerate(frames):
  if f['id']!=0x1200000+i or f['data']!=(bytes([i])*8).hex():raise AssertionError(f)
  if not 0<=((f['timestamp_us']-(due+i*2000))&0xffffffff)<1000:raise AssertionError('Late internal reception')
 r['host_stall_autonomy']={'status':'PASS','host_sleep_seconds':host_elapsed,'frames':8,'frames_raw':frames,
                           'planned_interval_us':2000,'comparison_to_USB_CAN':'NOT_MEASURED',
                           'internal_rx_lag_us':[((f['timestamp_us']-(due+i*2000))&0xffffffff) for i,f in enumerate(frames)],
                           'internal_rx_intervals_us':[((b['timestamp_us']-a['timestamp_us'])&0xffffffff) for a,b in zip(frames,frames[1:])],
                           'timestamp_scope':'PL polling observes internal RX, not physical CAN SOF'}
 x.stop();r['final']=x.status();r['motion_mask']=x.read(0x5a0)
 if r['final']['control']!=0 or r['motion_mask']!=0:raise AssertionError('Unsafe final state')
 r['status']='PASS'
finally:x.stop();x.close()
print(json.dumps(r,indent=2))
