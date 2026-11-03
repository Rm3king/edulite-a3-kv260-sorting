"""Bounded v2 TCP framing and strict JSON envelope, standard library only."""
import asyncio
import hashlib
import json
import math
import re
import struct
import time
import uuid

MAX_FRAME = 65536
TYPES = {'HELLO', 'SYNC', 'HEARTBEAT', 'SCENE', 'PREPARE_PICK_PLACE', 'PREPARE_SKILL',
         'EXECUTE_PLAN', 'QUERY_TASK', 'PAUSE', 'RESUME', 'RETURN_SUPPORTED',
         'VERIFY_RESULT', 'STATUS', 'RESULT', 'TELEMETRY'}
FIELDS = {'schema_version', 'type', 'request_id', 'session_id', 'sequence',
          'source_monotonic_ns', 'payload'}

class ProtocolError(ValueError):
    pass

def require(condition, message):
    if not condition:
        raise ProtocolError(message)

def identifier(value, allow_empty=False):
    require(isinstance(value, str) and ((allow_empty and value == '') or
            re.fullmatch(r'[A-Za-z0-9_.:-]{1,96}', value)), 'invalid identifier')
    return value

def integer(value, low=0, high=(1 << 63)-1):
    require(type(value) is int and low <= value <= high, 'invalid integer')
    return value

def finite(value, depth=0):
    require(depth <= 16, 'JSON nesting exceeds 16')
    if isinstance(value, dict):
        require(all(isinstance(k, str) for k in value), 'non-string JSON key')
        for v in value.values():
            finite(v, depth+1)
    elif isinstance(value, list):
        for v in value:
            finite(v, depth+1)
    elif type(value) is float:
        require(math.isfinite(value), 'non-finite float')
    else:
        require(value is None or type(value) in (str, int, bool), 'invalid JSON value')

def validate(message):
    require(type(message) is dict and set(message) == FIELDS, 'invalid envelope fields')
    require(type(message['schema_version']) is int and message['schema_version'] == 2, 'unsupported schema version')
    require(type(message['type']) is str and message['type'] in TYPES, 'unsupported message type')
    identifier(message['request_id'])
    identifier(message['session_id'], allow_empty=message['type'] == 'HELLO')
    integer(message['sequence'])
    integer(message['source_monotonic_ns'])
    require(type(message['payload']) is dict, 'payload must be an object')
    finite(message)
    return message

def unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, 'duplicate JSON key')
        result[key] = value
    return result

def canonical(value):
    finite(value)
    return json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True,
                      separators=(',', ':')).encode('utf-8')

def digest(value):
    return hashlib.sha256(canonical(value)).hexdigest()

def encode(message):
    raw = canonical(validate(message))
    require(0 < len(raw) <= MAX_FRAME, 'frame size exceeds limit')
    return struct.pack('!I', len(raw)) + raw

def decode(raw):
    try:
        return validate(json.loads(raw.decode('utf-8'), object_pairs_hook=unique_object,
                                  parse_constant=lambda x: (_ for _ in ()).throw(ProtocolError('invalid JSON constant'))))
    except (UnicodeError, json.JSONDecodeError, RecursionError, TypeError, OverflowError) as exc:
        raise ProtocolError('invalid UTF-8 JSON envelope') from exc

async def read(reader, timeout=15, idle_timeout=None):
    first = b''
    if idle_timeout is not None:
        # An idle observer may wait through offline planning. Once the first
        # byte arrives, a partial frame still has the short assembly deadline.
        first = await asyncio.wait_for(reader.readexactly(1),idle_timeout)
    async def frame():
        head = first+await reader.readexactly(4-len(first))
        size, = struct.unpack('!I', head)
        require(0 < size <= MAX_FRAME, 'invalid frame length')
        return decode(await reader.readexactly(size))
    return await asyncio.wait_for(frame(), timeout)

async def write(writer, message, timeout=3):
    writer.write(encode(message))
    await asyncio.wait_for(writer.drain(), timeout)

def envelope(kind, payload=None, session='', sequence=0, request_id=None):
    return dict(schema_version=2, type=kind, request_id=request_id or uuid.uuid4().hex,
                session_id=session, sequence=sequence, source_monotonic_ns=time.monotonic_ns(),
                payload=payload or {})

def payload_fields(payload, required, optional=()):
    require(set(required) <= set(payload) <= set(required) | set(optional), 'invalid payload fields')

def pose(value):
    require(type(value) is dict and set(value) == {'position_m', 'quaternion_xyzw'}, 'invalid TCP pose')
    for name, n in [('position_m', 3), ('quaternion_xyzw', 4)]:
        row = value[name]
        require(type(row) is list and len(row) == n and all(type(x) in (int, float) and math.isfinite(x) for x in row), 'invalid pose numbers')
    require(abs(sum(x*x for x in value['quaternion_xyzw'])-1) < 1e-5, 'quaternion must be normalized')
