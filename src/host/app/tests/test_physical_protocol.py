"""Physical protocol semantics tested with a nonactuating owner double."""
import asyncio
import tempfile
import time
import unittest
from pathlib import Path
from screwcell import wire
from screwcell.client import Client
from screwcell.service import Service


class OwnerDouble:
    def __init__(self):
        self.events=asyncio.Queue();self.state='IDLE';self.executions=0;self.task_id=None;self.fail=False
    async def call(self,op,p=None):
        if op=='PREPARE_SKILL':
            self.task_id=p['task_id'];return dict(prepared_digest='encoded_digest',physical_execution_supported=True)
        if op=='EXECUTE':
            self.executions+=1
            if self.fail:raise RuntimeError('injected ambiguous owner failure')
            self.state='RUNNING'
        if op=='STATUS':return dict(state=self.state,task_id=self.task_id,executions=self.executions)
        if op=='PAUSE':self.state='HOLDING'
        if op=='RESUME':self.state='RUNNING'
        return {}


class PhysicalProtocolTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.owner=OwnerDouble()
        self.service=Service(self.owner,Path(self.tmp.name)/'db','kv260-motion')
        self.service.configured_model_id='MODEL'
        self.server=await asyncio.start_server(self.service.control,'127.0.0.1',0)
        self.client=Client(port=self.server.sockets[0].getsockname()[1],client_id='controller')
        self.caps=await self.client.connect()
    async def asyncTearDown(self):
        await self.client.close();self.server.close();await self.server.wait_closed()
        self.service.event_task.cancel();await asyncio.gather(self.service.event_task,return_exceptions=True)
        self.service.store.close();self.tmp.cleanup()
    def payload(self,task='t'):
        return dict(task_id=task,skill_id='empty_pick_return_v1',model_id='MODEL',valid_until_kv_monotonic_ns=time.monotonic_ns()+300_000_000_000)
    async def prepare(self):
        result=await self.client.call('PREPARE_SKILL',self.payload())
        self.assertEqual(result['payload']['status'],'PLANNED',result)
        return result['payload']
    async def execute(self,p):
        await self.client.call('HEARTBEAT')
        return self.client.message('EXECUTE_PLAN',dict(plan_id=p['plan_id'],plan_digest=p['plan_digest']))
    async def test_capability_and_versioned_skill(self):
        self.assertTrue(self.caps['physical_execution_supported']);self.assertEqual(self.caps['protocol_revision'],'2.2')
        p=await self.prepare();self.assertEqual(p['report']['prepared_digest'],'encoded_digest')
    async def test_bad_model_skill_expiry_and_extra_fields(self):
        for change,code in [({'model_id':'BAD'},'MODEL_MISMATCH'),({'skill_id':'arbitrary_torque'},'UNKNOWN_PHYSICAL_SKILL'),
                            ({'valid_until_kv_monotonic_ns':1},'SKILL_AUTHORIZATION_EXPIRED'),({'enable':True},'invalid payload fields')]:
            p=self.payload();p.update(change)
            self.assertEqual((await self.client.call('PREPARE_SKILL',p))['payload']['code'],code)
        self.assertEqual(self.owner.executions,0)
    async def test_digest_is_checked_before_owner_and_execute_is_single_effect(self):
        p=await self.prepare();bad=await self.execute(p);bad['payload']['plan_digest']='wrong'
        self.assertEqual((await self.client.send_message(bad))['payload']['code'],'PLAN_DIGEST_MISMATCH')
        self.assertEqual(self.owner.executions,0)
        msg=await self.execute(p);reply=await self.client.send_message(msg)
        self.assertFalse(reply['payload']['simulated']);self.assertEqual(await self.client.send_message(msg),reply)
        await self.client.close();await self.client.connect()
        self.assertEqual(await self.client.send_message(msg),reply)
        self.assertEqual((await self.client.call('EXECUTE_PLAN',msg['payload']))['payload']['code'],'PLAN_ALREADY_USED_OR_INVALID')
        self.assertEqual(self.owner.executions,1)
    async def test_ambiguous_execute_is_not_rearmed(self):
        p=await self.prepare();self.owner.fail=True;msg=await self.execute(p)
        first=await self.client.send_message(msg);self.assertEqual(first['payload']['status'],'REJECTED')
        self.assertEqual(await self.client.send_message(msg),first)
        q=await self.client.call('QUERY_TASK',dict(task_id='t'));self.assertEqual(q['payload']['state'],'RECONCILIATION_REQUIRED')
        self.assertEqual(self.owner.executions,1)
    async def test_no_synthetic_success_or_uncalibrated_visual_motion(self):
        self.assertEqual((await self.client.call('VERIFY_RESULT',dict(task_id='t',result='SUCCESS',evidence_id='fake')))['payload']['code'],'VISUAL_VERIFICATION_UNAVAILABLE_FOR_EMPTY_SKILL')
        self.assertEqual((await self.client.call('PREPARE_PICK_PLACE',{}))['payload']['code'],'VISUAL_PHYSICAL_SKILL_NOT_CALIBRATED')
    async def test_shutdown_requires_disabled_completed_owner(self):
        self.assertTrue(await self.service.shutdown_allowed())
        for state in ('RUNNING','BRAKING','HOLDING','RETURNING','HOLDING_FAULT'):
            self.owner.state=state;self.assertFalse(await self.service.shutdown_allowed())
        self.owner.state='MOTION_DONE';self.assertTrue(await self.service.shutdown_allowed())
    async def test_cannot_take_over_holding_controller(self):
        self.owner.state='HOLDING';self.service.client_seen=0
        other=Client(port=self.server.sockets[0].getsockname()[1],client_id='other')
        try:
            caps=await other.connect();self.assertEqual(caps['status'],'REJECTED')
            self.assertEqual(caps['code'],'CONTROL_OWNER_EXISTS')
        except wire.ProtocolError:
            pass  # Client correctly refuses a rejected HELLO.
        finally:await other.close()


class FrameIdleTests(unittest.IsolatedAsyncioTestCase):
    async def test_idle_wait_and_partial_frame_have_separate_deadlines(self):
        reader=asyncio.StreamReader()
        msg=wire.envelope('HELLO',dict(client_id='a',role='observer'))
        async def feed():
            await asyncio.sleep(.05);reader.feed_data(wire.encode(msg))
        task=asyncio.create_task(feed())
        self.assertEqual(await wire.read(reader,timeout=.02,idle_timeout=.2),msg)
        await task
        reader=asyncio.StreamReader();reader.feed_data(b'\x00')
        with self.assertRaises(asyncio.TimeoutError):await wire.read(reader,timeout=.02,idle_timeout=.2)


class BoundedTelemetryTests(unittest.TestCase):
    def test_long_disconnection_keeps_bounded_evidence_and_explicit_loss_count(self):
        from screwcell.physical_owner import BoundedEvidence
        import json
        data=BoundedEvidence(1000,64)
        for i in range(100000):data.append(dict(sequence=i))
        self.assertLessEqual(len(data),1000)
        self.assertEqual(data[-1]['sequence'],99999)
        self.assertEqual(data.dropped+len(data),100000)
        self.assertEqual(len(json.loads(json.dumps(data))),len(data))
    def test_archive_io_failure_is_explicit_evidence_error(self):
        from screwcell.archives import archive_disabled
        with tempfile.TemporaryDirectory() as root:
            result=archive_disabled({'motion':'already_returned_and_disabled'},Path(root)/'missing'/'report.gz')
            self.assertFalse(result['evidence_saved']);self.assertIn('FileNotFoundError',result['evidence_error'])
