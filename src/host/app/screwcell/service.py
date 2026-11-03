"""Task TCP server: explicit mock, MuJoCo, KV plan-only or registered motion."""
import argparse
import asyncio
import contextlib
import json
import math
import signal
import sqlite3
import time
import uuid
from pathlib import Path
from . import wire
from .owner import Owner

class Store:
    def __init__(self, path):
        self.db = sqlite3.connect(path)
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('CREATE TABLE IF NOT EXISTS requests (session TEXT,id TEXT,hash TEXT,response TEXT,PRIMARY KEY(session,id))')
        self.db.execute('CREATE TABLE IF NOT EXISTS plans (id TEXT PRIMARY KEY,session TEXT,task TEXT,state TEXT,data TEXT)')
        self.db.execute('CREATE TABLE IF NOT EXISTS events (number INTEGER PRIMARY KEY AUTOINCREMENT,task TEXT,data TEXT)')
        self.db.execute("UPDATE plans SET state='RECONCILIATION_REQUIRED' WHERE state IN ('RUNNING','HOLDING','BRAKING','RETURNING','HOLDING_FAULT')")
        self.db.commit()
    def event(self, task, data):
        self.db.execute('INSERT INTO events(task,data) VALUES (?,?)', (task, json.dumps(data, ensure_ascii=False)))
        self.db.commit()
    def close(self):
        self.db.close()

class Service:
    def __init__(self, owner, database, mode='mock'):
        self.owner = owner; self.store = Store(database); self.mode = mode
        self.session = uuid.uuid4().hex; self.client = None; self.client_seen = 0
        self.sequences = {}; self.inflight = {}; self.scenes = {}; self.planning = False
        self.event_task = asyncio.create_task(self.collect_events())
    async def collect_events(self):
        while True:
            event = await self.owner.events.get()
            state = event['event']; task = event['task_id']
            self.store.event(task, event)
            if state in ('MOTION_DONE', 'HOLDING', 'STARTED', 'RETURNED_SUPPORTED', 'BRAKING', 'RETURNING', 'HOLDING_FAULT', 'RESUMED'):
                value = {'STARTED': 'RUNNING', 'RESUMED': 'RUNNING', 'RETURNED_SUPPORTED': 'RETURNED_SUPPORTED'}.get(state, state)
                self.store.db.execute('UPDATE plans SET state=? WHERE task=? AND session=?', (value, task, self.session))
                self.store.db.commit()
    def reply(self, request, status, **data):
        return wire.envelope('RESULT', dict(status=status, **data), self.session,
                             request['sequence'], request['request_id'])
    async def handle(self, request, connection):
        wire.validate(request)
        kind = request['type']; p = request['payload']
        if kind == 'HELLO':
            wire.payload_fields(p, ['client_id', 'role'])
            wire.identifier(p['client_id']); wire.require(p['role'] in ('control', 'observer'), 'invalid client role')
            if p['role'] == 'control':
                if self.client not in (None, p['client_id']):
                    status = await self.owner.call('STATUS')
                    if self.planning or time.monotonic()-self.client_seen < .5 or status['state'] not in ('IDLE', 'MOTION_DONE'):
                        return self.reply(request, 'REJECTED', code='CONTROL_OWNER_EXISTS')
                self.client = p['client_id']; self.client_seen = time.monotonic()
            connection.update(client=p['client_id'], role=p['role'])
            return self.reply(request, 'HELLO', backend=self.mode, physical_execution_supported=self.mode=='kv260-motion',
                              protocol_version=2, event_channel='QUERY_TASK_CURSOR', telemetry_hz=20,
                              protocol_revision='2.2', registered_skills=['empty_pick_return_v1'] if self.mode=='kv260-motion' else [],
                              model_id=self.model_id(), boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                              gripper_units='preset_id_not_metres', motor_commands_sent=0)
        if request['session_id'] != self.session:
            return self.reply(request, 'REJECTED', code='SESSION_MISMATCH')
        if not connection.get('client'):
            return self.reply(request, 'REJECTED', code='HELLO_REQUIRED')
        if kind == 'SYNC':
            wire.payload_fields(p, ['t1_ns']); wire.integer(p['t1_ns'])
            t2 = connection['received_ns']
            return self.reply(request, 'SYNC', t1_ns=p['t1_ns'], t2_ns=t2, t3_ns=time.monotonic_ns(),
                              clock_domain='kv_monotonic', pl_tick_mapping_channel='STATUS.owner.pl_clock_mapping')
        if kind in ('STATUS', 'QUERY_TASK'):
            if kind == 'STATUS':
                wire.payload_fields(p, [])
                return self.reply(request, 'STATUS', owner=await self.owner.call('STATUS'), backend=self.mode)
            wire.payload_fields(p, ['task_id'], ['after_event'])
            wire.identifier(p['task_id']); cursor = wire.integer(p.get('after_event', 0))
            row = self.store.db.execute('SELECT id,state FROM plans WHERE task=? ORDER BY rowid DESC LIMIT 1', (p['task_id'],)).fetchone()
            events = self.store.db.execute('SELECT number,data FROM events WHERE task=? AND number>? ORDER BY number LIMIT 128', (p['task_id'], cursor)).fetchall()
            return self.reply(request, 'TASK', plan_id=row[0] if row else None, state=row[1] if row else 'UNKNOWN',
                              events=[dict(event_number=n, **json.loads(data)) for n, data in events])
        if connection['role'] != 'control' or connection['client'] != self.client:
            return self.reply(request, 'REJECTED', code='READ_ONLY_CLIENT')
        if kind == 'HEARTBEAT':
            wire.payload_fields(p, [])
            self.client_seen = time.monotonic()
            await self.owner.call('HEARTBEAT')
            return self.reply(request, 'ALIVE')
        key = (self.session, request['request_id'])
        fingerprint = wire.digest(dict(type=kind, payload=p, client=connection['client']))
        old = self.store.db.execute('SELECT hash,response FROM requests WHERE session=? AND id=?', key).fetchone()
        if old:
            if old[0] != fingerprint:
                return self.reply(request, 'REJECTED', code='REQUEST_ID_CONFLICT')
            return json.loads(old[1])
        if key in self.inflight:
            old_hash, job = self.inflight[key]
            if old_hash != fingerprint:
                return self.reply(request, 'REJECTED', code='REQUEST_ID_CONFLICT')
            return await asyncio.shield(job)
        if request['sequence'] <= self.sequences.get(connection['client'], -1):
            return self.reply(request, 'REJECTED', code='OLD_SEQUENCE')
        self.sequences[connection['client']] = request['sequence']
        async def run():
            try:
                result = await self.dispatch(request)
            except (wire.ProtocolError, ValueError, RuntimeError, asyncio.TimeoutError) as exc:
                result = self.reply(request, 'REJECTED', code=str(exc))
            self.store.db.execute('INSERT INTO requests VALUES (?,?,?,?)', (*key, fingerprint, json.dumps(result)))
            self.store.db.commit()
            return result
        job = asyncio.create_task(run()); self.inflight[key] = (fingerprint, job)
        try:
            return await asyncio.shield(job)
        finally:
            if job.done():
                self.inflight.pop(key, None)
    def model_id(self):
        return getattr(self, 'configured_model_id', 'MOCK_MODEL')
    async def shutdown_allowed(self):
        return self.mode!='kv260-motion' or (await self.owner.call('STATUS'))['state'] in ('IDLE','MOTION_DONE')
    async def dispatch(self, request):
        p = request['payload']; kind = request['type']
        if kind == 'PREPARE_SKILL':
            wire.require(self.mode=='kv260-motion', 'PHYSICAL_SKILL_BACKEND_NOT_ENABLED')
            wire.payload_fields(p,['task_id','skill_id','model_id','valid_until_kv_monotonic_ns'])
            for k in ('task_id','skill_id','model_id'):wire.identifier(p[k])
            wire.require(p['skill_id']=='empty_pick_return_v1','UNKNOWN_PHYSICAL_SKILL')
            wire.require(p['model_id']==self.model_id(),'MODEL_MISMATCH')
            wire.integer(p['valid_until_kv_monotonic_ns'])
            wire.require(time.monotonic_ns()<p['valid_until_kv_monotonic_ns']<=time.monotonic_ns()+600_000_000_000,'SKILL_AUTHORIZATION_EXPIRED')
            wire.require(not self.planning,'PLANNING_BUSY')
            wire.require(self.store.db.execute('SELECT id FROM plans WHERE task=?',(p['task_id'],)).fetchone() is None,'TASK_ALREADY_HAS_PLAN')
            self.planning=True
            try:report=await self.owner.call('PREPARE_SKILL',p)
            finally:self.planning=False
            plan_id=uuid.uuid4().hex;plan_digest=wire.digest(dict(payload=p,prepared_digest=report['prepared_digest']))
            data=dict(payload=p,report=report,digest=plan_digest,owner_client_id=self.client)
            self.store.db.execute("UPDATE plans SET state='SUPERSEDED' WHERE session=? AND state='PLANNED'",(self.session,))
            self.store.db.execute('INSERT INTO plans VALUES (?,?,?,?,?)',(plan_id,self.session,p['task_id'],'PLANNED',json.dumps(data)))
            self.store.db.commit()
            self.store.event(p['task_id'],dict(event='PLANNED',plan_id=plan_id,time_ns=time.monotonic_ns(),backend=self.mode))
            return self.reply(request,'PLANNED',plan_id=plan_id,plan_digest=plan_digest,report=report)
        if kind == 'SCENE':
            wire.payload_fields(p, ['scene_id', 'source_kind', 'objects'], ['calibration_id'])
            wire.identifier(p['scene_id'])
            wire.require(p['source_kind'] in ('recorded', 'live', 'synthetic'), 'invalid source kind')
            wire.require(type(p['objects']) is list and len(p['objects']) <= 64, 'invalid objects')
            ids = set()
            for obj in p['objects']:
                wire.require(type(obj) is dict, 'invalid scene object')
                wire.identifier(obj.get('object_id')); wire.require(obj['object_id'] not in ids, 'duplicate object ID'); ids.add(obj['object_id'])
                wire.require(obj.get('part_class') in ('screw', 'nut'), 'invalid part class')
            previous = self.scenes.get(p['scene_id'])
            wire.require(previous is None or wire.digest(previous) == wire.digest(p), 'SCENE_ID_CONFLICT')
            wire.require(previous is not None or len(self.scenes) < 64, 'SCENE_STORE_FULL')
            self.scenes[p['scene_id']] = p
            return self.reply(request, 'RECEIVED', scene_id=p['scene_id'])
        if kind == 'PREPARE_PICK_PLACE':
            wire.require(self.mode!='kv260-motion','VISUAL_PHYSICAL_SKILL_NOT_CALIBRATED')
            required = ['task_id', 'scene_id', 'object_id', 'model_id', 'calibration_id', 'layout_id', 'tool_id',
                        'frame', 'grasp_tcp', 'place_tcp', 'approach_height_m', 'lift_height_m',
                        'gripper_preset_id', 'part_class', 'valid_until_kv_monotonic_ns', 'planning_start_joint_rad']
            wire.payload_fields(p, required)
            for k in ['task_id', 'scene_id', 'object_id', 'model_id', 'calibration_id', 'layout_id', 'tool_id']:
                wire.identifier(p[k])
            wire.require(p['frame'] == 'robot_base', 'FRAME_MISMATCH')
            wire.require(p['model_id'] == self.model_id(), 'MODEL_MISMATCH')
            wire.require(p['gripper_preset_id'] == 'release_20261019_empty', 'UNKNOWN_GRIPPER_PRESET')
            wire.pose(p['grasp_tcp']); wire.pose(p['place_tcp'])
            for name in ('approach_height_m', 'lift_height_m'):
                wire.require(type(p[name]) in (int, float) and .01 <= p[name] <= .15, 'invalid approach/lift height')
            wire.require(type(p['planning_start_joint_rad']) is list and len(p['planning_start_joint_rad']) == 7
                         and all(type(x) in (int, float) for x in p['planning_start_joint_rad']), 'invalid planning joint fixture')
            scene = self.scenes.get(p['scene_id'])
            wire.require(scene is not None, 'SCENE_UNKNOWN')
            objects = [x for x in scene['objects'] if x['object_id'] == p['object_id']]
            wire.require(len(objects) == 1 and objects[0]['part_class'] == p['part_class'], 'OBJECT_MISMATCH')
            wire.require(scene.get('calibration_id', 'UNSET') == p['calibration_id'], 'CALIBRATION_MISMATCH')
            wire.integer(p['valid_until_kv_monotonic_ns'])
            wire.require(time.monotonic_ns() < p['valid_until_kv_monotonic_ns'] <= time.monotonic_ns()+60_000_000_000, 'TARGET_EXPIRED_OR_HORIZON_TOO_LONG')
            wire.require(not self.planning, 'PLANNING_BUSY')
            prior = self.store.db.execute('SELECT id FROM plans WHERE task=? AND session=?', (p['task_id'], self.session)).fetchone()
            wire.require(prior is None, 'TASK_ALREADY_HAS_PLAN')
            self.planning = True
            try:
                report = await self.owner.call('PREPARE', p)
            finally:
                self.planning = False
            plan_id = uuid.uuid4().hex; plan_digest = wire.digest(p)
            data = dict(payload=p, report=report, digest=plan_digest, owner_client_id=self.client)
            self.store.db.execute('INSERT INTO plans VALUES (?,?,?,?,?)', (plan_id, self.session, p['task_id'], 'PLANNED', json.dumps(data)))
            self.store.db.commit()
            self.store.event(p['task_id'], dict(event='PLANNED', plan_id=plan_id, time_ns=time.monotonic_ns(), backend=self.mode))
            return self.reply(request, 'PLANNED', plan_id=plan_id, plan_digest=plan_digest, report=report)
        if kind == 'EXECUTE_PLAN':
            wire.payload_fields(p, ['plan_id', 'plan_digest'])
            wire.identifier(p['plan_id'])
            row = self.store.db.execute('SELECT session,task,state,data FROM plans WHERE id=?', (p['plan_id'],)).fetchone()
            wire.require(row is not None and row[0] == self.session, 'PLAN_SESSION_MISMATCH')
            data = json.loads(row[3])
            wire.require(data.get('owner_client_id') == self.client, 'PLAN_OWNER_MISMATCH')
            wire.require(p['plan_digest'] == data['digest'], 'PLAN_DIGEST_MISMATCH')
            wire.require(row[2] == 'PLANNED', 'PLAN_ALREADY_USED_OR_INVALID')
            wire.require(time.monotonic_ns() < data['payload']['valid_until_kv_monotonic_ns'], 'PLAN_EXPIRED')
            wire.require(self.mode in ('mock','kv260-motion','mujoco'), 'PHYSICAL_EXECUTION_UNAVAILABLE_IN_PLAN_ONLY_RELEASE')
            wire.require(time.monotonic()-self.client_seen < .5, 'HEARTBEAT_REQUIRED')
            self.store.db.execute("UPDATE plans SET state='RUNNING' WHERE id=?", (p['plan_id'],)); self.store.db.commit()
            try:
                execution=await self.owner.call('EXECUTE', dict(task_id=row[1],prepared_digest=data['report'].get('prepared_digest')))
            except Exception:
                self.store.db.execute("UPDATE plans SET state=? WHERE id=?", ('RECONCILIATION_REQUIRED' if self.mode=='kv260-motion' else 'REJECTED',p['plan_id'])); self.store.db.commit(); raise
            return self.reply(request, 'STARTED', simulated=self.mode in ('mock','mujoco'), task_id=row[1],execution=execution)
        if kind in ('PAUSE', 'RESUME', 'RETURN_SUPPORTED'):
            wire.payload_fields(p, ['task_id'])
            current = await self.owner.call('STATUS')
            wire.require(current['task_id'] == p['task_id'], 'TASK_NOT_ACTIVE')
            wire.require(self.mode in ('mock','kv260-motion','mujoco'), 'PHYSICAL_EXECUTION_UNAVAILABLE_IN_PLAN_ONLY_RELEASE')
            if kind == 'RESUME':
                wire.require(time.monotonic()-self.client_seen < .5, 'HEARTBEAT_REQUIRED')
            await self.owner.call(kind, p)
            return self.reply(request, 'ACCEPTED', simulated=self.mode in ('mock','mujoco'))
        if kind == 'VERIFY_RESULT':
            wire.require(self.mode!='kv260-motion','VISUAL_VERIFICATION_UNAVAILABLE_FOR_EMPTY_SKILL')
            wire.payload_fields(p, ['task_id', 'result', 'evidence_id'])
            wire.require(p['result'] in ('SUCCESS', 'FAIL', 'UNKNOWN'), 'invalid verification')
            wire.identifier(p['evidence_id'])
            row = self.store.db.execute('SELECT id,state FROM plans WHERE task=? AND session=?', (p['task_id'], self.session)).fetchone()
            wire.require(row is not None and row[1] in ('MOTION_DONE', 'VERIFY_FAIL', 'VERIFY_UNKNOWN'), 'MOTION_NOT_DONE')
            self.store.db.execute('UPDATE plans SET state=? WHERE id=?', ('VERIFY_'+p['result'], row[0])); self.store.db.commit()
            self.store.event(p['task_id'], dict(event='VERIFY_'+p['result'], evidence_id=p['evidence_id'], time_ns=time.monotonic_ns()))
            return self.reply(request, 'VERIFIED', result=p['result'])
        raise wire.ProtocolError('UNSUPPORTED_MESSAGE')
    async def control(self, reader, writer):
        connection = {}
        try:
            while True:
                request = await wire.read(reader,idle_timeout=900)
                connection['received_ns'] = time.monotonic_ns()
                result = await self.handle(request, connection)
                await wire.write(writer, result)
        except (wire.ProtocolError, asyncio.IncompleteReadError, ConnectionError, asyncio.TimeoutError):
            pass
        finally:
            writer.close()
            with contextlib.suppress(ConnectionError,OSError):
                await writer.wait_closed()
    async def telemetry(self, reader, writer):
        try:
            hello = await wire.read(reader, 3)
            wire.require(hello['type'] == 'HELLO' and hello['payload'].get('role') == 'observer', 'telemetry observer hello required')
            while True:
                status = await self.owner.call('STATUS')
                await wire.write(writer, wire.envelope('TELEMETRY', dict(owner=status, backend=self.mode), self.session), .2)
                await asyncio.sleep(.05)
        except (wire.ProtocolError, ConnectionError, asyncio.TimeoutError, asyncio.IncompleteReadError):
            pass
        finally:
            writer.close()
            with contextlib.suppress(ConnectionError,OSError):
                await writer.wait_closed()
    async def close(self):
        self.event_task.cancel(); await asyncio.gather(self.event_task, return_exceptions=True)
        jobs = [job for _, job in self.inflight.values()]
        if jobs:
            await asyncio.gather(*jobs, return_exceptions=True)
        await self.owner.close(); self.store.close()

async def serve(args):
    if args.backend=='kv260-motion' and not (args.enable_physical_backend and args.tracking and args.evidence_root):
        raise ValueError('Physical backend requires explicit enable, tracking profile and evidence directory')
    owner = Owner(args.backend, args.runtime_root, args.profile,args.tracking,args.evidence_root)
    service = Service(owner, args.database, args.backend)
    service.configured_model_id = args.model_id
    control = await asyncio.start_server(service.control, args.bind, args.port, limit=wire.MAX_FRAME+4)
    telemetry = await asyncio.start_server(service.telemetry, args.bind, args.port+1, limit=wire.MAX_FRAME+4)
    print(json.dumps(dict(event='READY', backend=args.backend, bind=args.bind, port=args.port,
                         session_id=service.session, physical_execution_supported=args.backend=='kv260-motion')), flush=True)
    done = asyncio.Event()
    for signum in (signal.SIGTERM, signal.SIGINT):
        asyncio.get_running_loop().add_signal_handler(signum, done.set)
    try:
        while True:
            await done.wait()
            if await service.shutdown_allowed():break
            # Keep the endpoints AND the sole hardware owner alive. An
            # operator can still resume/return through the same protocol.
            print(json.dumps(dict(event='SHUTDOWN_DEFERRED_ACTIVE_OWNER')),flush=True)
            done.clear()
    finally:
        control.close(); telemetry.close(); await control.wait_closed(); await telemetry.wait_closed(); await service.close()

if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--bind', default='127.0.0.1'); p.add_argument('--port', type=int, default=46000)
    p.add_argument('--backend', choices=['mock', 'kv260-plan-only','kv260-motion','mujoco'], default='mock'); p.add_argument('--database', required=True)
    p.add_argument('--enable-physical-backend',action='store_true');p.add_argument('--tracking',default='');p.add_argument('--evidence-root',default='')
    p.add_argument('--runtime-root', default=''); p.add_argument('--profile', default=''); p.add_argument('--model-id', default='MOCK_MODEL')
    asyncio.run(serve(p.parse_args()))
