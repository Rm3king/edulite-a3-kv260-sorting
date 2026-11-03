import asyncio
import json
import math
import sqlite3
import struct
import tempfile
import time
import unittest
from pathlib import Path
from screwcell import wire
from screwcell.client import Client
from screwcell.owner import Owner
from screwcell.service import Service,Store
from screwcell.tasks import Workflow,parse_intent,planning_fixture,select_objects

class WireTests(unittest.TestCase):
    def test_golden_bytes_and_unicode(self):
        msg=wire.envelope('HELLO',{'client_id':'ai','role':'control'},request_id='golden');msg.update(source_monotonic_ns=1,sequence=1)
        raw=wire.encode(msg)
        self.assertEqual(struct.unpack('!I',raw[:4])[0],len(raw)-4)
        self.assertEqual(wire.decode(raw[4:]),msg)
    def test_duplicate_nan_and_wrong_types(self):
        for raw in [b'{"a":1,"a":2}',b'NaN',b'[]',b'\xff']:
            with self.assertRaises(wire.ProtocolError):wire.decode(raw)
        for change in [dict(schema_version=True),dict(type=[]),dict(sequence=True),dict(payload={'x':float('nan')}),dict(extra=1)]:
            m=wire.envelope('HELLO');m.update(change)
            with self.assertRaises(wire.ProtocolError):wire.encode(m)
    def test_size_and_depth(self):
        with self.assertRaises(wire.ProtocolError):wire.encode(wire.envelope('HELLO',{'large':'x'*65536}))
        d={};leaf=d
        for _ in range(20):leaf['next']={};leaf=leaf['next']
        with self.assertRaises(wire.ProtocolError):wire.encode(wire.envelope('HELLO',d))
    def test_quaternion_and_numeric_pose(self):
        for pose in [dict(position_m=[0,0,0],quaternion_xyzw=[0,0,0,0]),dict(position_m=[True,0,0],quaternion_xyzw=[0,0,0,1])]:
            with self.assertRaises(wire.ProtocolError):wire.pose(pose)

class TaskTests(unittest.TestCase):
    def test_model_output_contract(self):
        good='{"action":"sort","part_class":"screw","count":2,"region":"right","destination":"B"}'
        self.assertEqual(parse_intent('```json\n'+good+'\n```')['count'],2)
        for raw in [good+' trailing',good.replace('"count":2','"count":true'),good.replace('"B"','"C"')]:
            with self.assertRaises(wire.ProtocolError):parse_intent(raw)
    def test_result_not_counted_at_motion_done_and_bounded_retries(self):
        w=Workflow(maximum_retries=1);w.start('obj');w.motion_done();self.assertEqual(len(w.verified),0)
        self.assertEqual(w.verify('UNKNOWN','img1'),'RETRY');w.start('obj');w.motion_done();self.assertEqual(w.verify('FAIL','img2'),'FAILED')
        w=Workflow();w.start('obj');w.motion_done();w.verify('SUCCESS','img3');self.assertEqual(w.verified,{'obj'})
        with self.assertRaises(wire.ProtocolError):w.start('obj')
    def test_object_selection_from_explicit_image_frame(self):
        i=parse_intent('{"action":"sort","part_class":"screw","count":1,"region":"right","destination":"B"}')
        scene=dict(image_width_px=640,objects=[dict(object_id='a',part_class='screw',center_uv_px=[500,10],geometry_valid=True),dict(object_id='b',part_class='nut',center_uv_px=[550,10],geometry_valid=True)])
        self.assertEqual(select_objects(i,scene)[0]['object_id'],'a')
        i['count']=2
        with self.assertRaises(wire.ProtocolError):select_objects(i,scene)

class IntegrationTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp=tempfile.TemporaryDirectory();self.db=Path(self.temp.name)/'state.db'
        self.owner=Owner();self.service=Service(self.owner,self.db)
        self.server=await asyncio.start_server(self.service.control,'127.0.0.1',0)
        self.port=self.server.sockets[0].getsockname()[1]
        self.client=Client(port=self.port,client_id='test');await self.client.connect();await self.client.sync(2)
    async def asyncTearDown(self):
        await self.client.close();self.server.close();await self.server.wait_closed();await self.service.close();self.temp.cleanup()
    async def prepare(self):
        await self.client.call('SCENE',dict(scene_id='scene',source_kind='synthetic',calibration_id='UNSET',objects=[dict(object_id='screw_01',part_class='screw')]))
        msg=self.client.message('PREPARE_PICK_PLACE',planning_fixture('scene',deadline=self.client.deadline()))
        reply=await self.client.send_message(msg)
        self.assertEqual(reply['payload']['status'],'PLANNED',reply)
        return msg,reply
    async def test_fragmented_coalesced_and_oversize_frames(self):
        r,w=await asyncio.open_connection('127.0.0.1',self.port)
        raw=wire.encode(wire.envelope('HELLO',dict(client_id='viewer',role='observer')))
        for byte in raw:w.write(bytes([byte]));await w.drain()
        reply=await wire.read(r);self.assertEqual(reply['payload']['status'],'HELLO')
        msgs=[wire.envelope('STATUS',session=self.service.session,sequence=i) for i in [1,2]]
        w.write(b''.join(map(wire.encode,msgs)));await w.drain()
        for msg in msgs:self.assertEqual((await wire.read(r))['request_id'],msg['request_id'])
        w.write(struct.pack('!I',65537));await w.drain();self.assertEqual(await asyncio.wait_for(r.read(1),2),b'')
        w.close();await w.wait_closed()
    async def test_duplicate_conflict_and_observer(self):
        msg,reply=await self.prepare();self.assertEqual(await self.client.send_message(msg),reply)
        conflict=json.loads(json.dumps(msg));conflict['payload']['object_id']='different'
        self.assertEqual((await self.client.send_message(conflict))['payload']['code'],'REQUEST_ID_CONFLICT')
        other=Client(port=self.port,client_id='viewer',role='observer');await other.connect()
        try:self.assertEqual((await other.call('HEARTBEAT'))['payload']['code'],'READ_ONLY_CLIENT')
        finally:await other.close()
    async def test_duplicate_execution_single_effect_and_verify(self):
        _,reply=await self.prepare();p=reply['payload']
        stop=asyncio.Event();hb=asyncio.create_task(self.client.heartbeat(stop))
        try:
            msg=self.client.message('EXECUTE_PLAN',dict(plan_id=p['plan_id'],plan_digest=p['plan_digest']))
            a=await self.client.send_message(msg);self.assertEqual(a['payload']['status'],'STARTED')
            self.assertEqual(await self.client.send_message(msg),a)
            again=await self.client.call('EXECUTE_PLAN',msg['payload']);self.assertEqual(again['payload']['status'],'REJECTED')
            await asyncio.sleep(.8)
            q=await self.client.call('QUERY_TASK',dict(task_id='pick_001'));self.assertEqual(q['payload']['state'],'MOTION_DONE')
            status=await self.client.call('STATUS');self.assertEqual(status['payload']['owner']['executions'],1)
            result=await self.client.call('VERIFY_RESULT',dict(task_id='pick_001',result='SUCCESS',evidence_id='synthetic'))
            self.assertEqual(result['payload']['status'],'VERIFIED')
        finally:stop.set();await hb
    async def test_link_loss_holds_and_never_auto_resumes(self):
        _,reply=await self.prepare();p=reply['payload']
        await self.client.call('EXECUTE_PLAN',dict(plan_id=p['plan_id'],plan_digest=p['plan_digest']))
        await asyncio.sleep(.7)
        status=await self.client.call('STATUS');self.assertEqual(status['payload']['owner']['state'],'HOLDING')
        await self.client.call('HEARTBEAT');await asyncio.sleep(.15)
        status=await self.client.call('STATUS');self.assertEqual(status['payload']['owner']['state'],'HOLDING')
        result=await self.client.call('RETURN_SUPPORTED',dict(task_id='pick_001'));self.assertEqual(result['payload']['status'],'ACCEPTED')
    async def test_expired_bad_frame_and_old_session(self):
        _,reply=await self.prepare()
        msg=self.client.message('STATUS');msg['session_id']='old';self.assertEqual((await self.client.send_message(msg))['payload']['code'],'SESSION_MISMATCH')
        bad=planning_fixture('scene',task_id='other',deadline=1)
        r=await self.client.call('PREPARE_PICK_PLACE',bad);self.assertEqual(r['payload']['code'],'TARGET_EXPIRED_OR_HORIZON_TOO_LONG')
        bad.update(valid_until_kv_monotonic_ns=self.client.deadline(),frame='camera')
        r=await self.client.call('PREPARE_PICK_PLACE',bad);self.assertEqual(r['payload']['code'],'FRAME_MISMATCH')
    async def test_reconnect_query_without_reexecution(self):
        msg,reply=await self.prepare();await self.client.close();await self.client.connect()
        self.assertEqual(await self.client.send_message(msg),reply)
        q=await self.client.call('QUERY_TASK',dict(task_id='pick_001'));self.assertEqual(q['payload']['state'],'PLANNED')
    async def test_old_calibration_and_payload_unknown_field(self):
        await self.prepare();bad=planning_fixture('scene',task_id='other',deadline=self.client.deadline());bad['calibration_id']='old'
        self.assertEqual((await self.client.call('PREPARE_PICK_PLACE',bad))['payload']['code'],'CALIBRATION_MISMATCH')
        self.assertEqual((await self.client.call('PAUSE',dict(task_id='x',reset=True)))['payload']['status'],'REJECTED')

    async def test_plan_only_backend_cannot_execute(self):
        _,reply=await self.prepare();p=reply['payload'];self.service.mode='kv260-plan-only'
        result=await self.client.call('EXECUTE_PLAN',dict(plan_id=p['plan_id'],plan_digest=p['plan_digest']))
        self.assertEqual(result['payload']['code'],'PHYSICAL_EXECUTION_UNAVAILABLE_IN_PLAN_ONLY_RELEASE')
        status=await self.owner.call('STATUS');self.assertEqual(status['executions'],0)
    async def test_independent_mock_owner_detects_blocked_network_loop(self):
        _,reply=await self.prepare();p=reply['payload']
        await self.client.call('EXECUTE_PLAN',dict(plan_id=p['plan_id'],plan_digest=p['plan_digest']))
        time.sleep(.7)  # Deliberate test-only network event-loop blackout.
        result=await self.owner.call('STATUS');self.assertEqual(result['state'],'HOLDING')
    async def test_telemetry_does_not_block_control(self):
        server=await asyncio.start_server(self.service.telemetry,'127.0.0.1',0)
        port=server.sockets[0].getsockname()[1]
        r,w=await asyncio.open_connection('127.0.0.1',port)
        try:
            await wire.write(w,wire.envelope('HELLO',dict(client_id='viewer',role='observer')))
            frame=await wire.read(r);self.assertEqual(frame['type'],'TELEMETRY')
            # Stop reading telemetry while control remains responsive.
            for _ in range(5):self.assertEqual((await self.client.call('STATUS'))['payload']['status'],'STATUS')
        finally:
            w.close();await w.wait_closed();server.close();await server.wait_closed()

class StoreTests(unittest.TestCase):
    def test_restart_marks_uncertain_execution(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)/'db';s=Store(p);s.db.execute('INSERT INTO plans VALUES (?,?,?,?,?)',('p','old','t','RUNNING','{}'));s.db.commit();s.close()
            s=Store(p);self.assertEqual(s.db.execute('SELECT state FROM plans').fetchone()[0],'RECONCILIATION_REQUIRED');s.close()


class InstructionTests(unittest.TestCase):
    def test_rejects_guessed_count_and_conflicting_destination(self):
        from screwcell.tasks import validate_instruction
        raw='{"action":"sort","part_class":"screw","count":4,"region":"all","destination":"B"}'
        with self.assertRaisesRegex(wire.ProtocolError,'CLARIFICATION_REQUIRED'):validate_instruction(raw,'把螺丝放到B盒')
        with self.assertRaisesRegex(wire.ProtocolError,'INVALID_OR_MISSING'):validate_instruction(raw,'把四颗螺丝放到C盒')
        with self.assertRaisesRegex(wire.ProtocolError,'DISAGREES'):validate_instruction(raw,'把两颗螺丝放到B盒')
        self.assertEqual(validate_instruction(raw,'把四颗螺丝放到B盒')['count'],4)
        for text in ['不要把四颗螺丝放到B盒','把至少四颗螺丝放到B盒','把四颗螺丝放到B盒然后搬开盒子']:
            with self.assertRaises(wire.ProtocolError):validate_instruction(raw,text)



class ClockTests(unittest.TestCase):
    def test_wrap_extends_with_bracket(self):
        from screwcell.clocks import PLClock
        c=PLClock();c.sample((1<<32)-10,0,2000)
        r=c.sample(10,20000,22000)
        self.assertEqual(r['extended_tick_us'],(1<<32)+10)
        self.assertTrue(r['continuity_verified']);self.assertEqual(r['bracket_uncertainty_ns'],2000)
    def test_reset_and_large_gap_remap(self):
        from screwcell.clocks import PLClock
        c=PLClock();c.sample(1_000_000,1_000_000_000,1_000_001_000)
        r=c.sample(1,1_000_100_000,1_000_101_000)
        self.assertEqual(r['software_mapping_epoch'],1);self.assertFalse(r['continuity_verified'])
        r=c.sample(50,70_000_100_000,70_000_101_000);self.assertEqual(r['software_mapping_epoch'],2)
    def test_invalid_bracket(self):
        from screwcell.clocks import PLClock
        for args in [(1,2,1),(-1,0,1)]:
            with self.assertRaises(ValueError):PLClock().sample(*args)



class ReconnectTests(unittest.IsolatedAsyncioTestCase):
    async def test_hello_retries_service_startup_without_replaying_commands(self):
        import socket
        sock=socket.socket();sock.bind(('127.0.0.1',0));port=sock.getsockname()[1];sock.close()
        received=[]
        async def handler(r,w):
            msg=await wire.read(r);received.append(msg['type'])
            await wire.write(w,wire.envelope('RESULT',dict(status='HELLO'),session='fresh',request_id=msg['request_id']))
            w.close();await w.wait_closed()
        async def later():
            await asyncio.sleep(.15)
            return await asyncio.start_server(handler,'127.0.0.1',port)
        task=asyncio.create_task(later());c=Client(port=port)
        try:
            await c.connect();self.assertEqual(c.session,'fresh');self.assertEqual(received,['HELLO'])
        finally:
            await c.close();server=await task;server.close();await server.wait_closed()
    async def test_close_never_masks_primary_connection_error(self):
        from unittest.mock import AsyncMock,Mock
        c=Client();c.writer=Mock();c.writer.wait_closed=AsyncMock(side_effect=ConnectionResetError('reset'))
        await c.close()



class DefaultSlotTests(unittest.TestCase):
    def test_omitted_region_uses_application_default_without_guessing_count(self):
        from screwcell.tasks import validate_instruction
        raw='{"action":"sort","part_class":"screw","count":2,"region":"left","destination":"B"}'
        self.assertEqual(validate_instruction(raw,'把两颗螺丝放到B盒')['region'],'all')
        with self.assertRaises(wire.ProtocolError):validate_instruction(raw,'把右侧两颗螺丝放到B盒')
        with self.assertRaises(wire.ProtocolError):validate_instruction(raw,'把螺丝放到B盒')

if __name__=='__main__':unittest.main()
