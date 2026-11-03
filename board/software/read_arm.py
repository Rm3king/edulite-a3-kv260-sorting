#!/usr/bin/python3
"""Read-only A3 CAN discovery using Type17 MECH_POS; never enables motors."""
import argparse
import json
import math
import struct
import time
from pathlib import Path
from a3_protocol import read_position
from canctl import CanDevice
from joint_safety import motor_to_joint, LIMITS, GripperRange


def decode_position(frame, motor):
    identifier = frame['id']
    data = bytes.fromhex(frame['data'])
    if not (frame['extended'] and not frame['rtr'] and len(data) == 8
            and identifier >> 24 == 17 and (identifier >> 8) & 255 == motor
            and identifier & 255 == 0xfd and struct.unpack('<H', data[:2])[0] == 0x7019):
        return None
    status = (identifier >> 16) & 255
    value = struct.unpack('<f', data[4:])[0]
    return {'motor': motor, 'parameter_status': status, 'position_motor_rad': value if math.isfinite(value) else None,
            'valid': status == 0 and math.isfinite(value)}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--rounds', type=int, default=1)
    args = ap.parse_args()
    if not 1 <= args.rounds <= 20:
        ap.error('rounds must be 1..20')
    report = {'motion_commands_sent': 0, 'enable_commands_sent': 0, 'parameter_writes_sent': 0,
              'requested_parameter': 'MECH_POS_0x7019', 'transactions': [], 'unmatched_frames': []}
    dev = CanDevice()
    try:
        dev.configure('normal')
        report['start'] = dev.status()
        failed_bus = False
        for turn in range(args.rounds):
            for motor in range(1, 8):
                dev.heartbeat()
                ident, data = read_position(motor)
                started = time.monotonic_ns()
                dev.enqueue(ident, data)
                transaction = {'round': turn+1, 'motor': motor, 'request_id': ident, 'request_data': data.hex(), 'response': None}
                deadline = time.monotonic() + .05
                while time.monotonic() < deadline:
                    dev.heartbeat()
                    frame = dev.receive()
                    if frame:
                        decoded = decode_position(frame, motor)
                        if decoded:
                            transaction['response'] = frame
                            transaction['decoded'] = decoded
                            transaction['host_elapsed_us'] = (time.monotonic_ns()-started)/1000
                            break
                        if len(report['unmatched_frames']) < 100:
                            report['unmatched_frames'].append(frame)
                    if dev.read(0x418):
                        failed_bus = True
                        break
                    time.sleep(.0002)
                transaction['can_errors'] = {'rx': dev.read(14*4), 'tx': dev.read(15*4),
                                               'error_capture': dev.read(12*4)}
                transaction['core_status'] = dev.read(2*4)
                report['transactions'].append(transaction)
                # Stop promptly on errors/unacknowledged request; no continuous retries.
                if transaction['can_errors']['tx'] or transaction['can_errors']['rx'] or failed_bus:
                    failed_bus = True
                    break
            if failed_bus:
                break
        report['before_stop'] = dev.status()
        good = [t for t in report['transactions'] if t.get('decoded', {}).get('valid')]
        report['responding_motors'] = sorted({t['motor'] for t in good})
        latest = {t['motor']: t['decoded']['position_motor_rad'] for t in good}
        report['joint_reference_latest'] = [
            {'motor': mid, 'position_joint_rad': motor_to_joint(mid, position),
             'sdk_limits_rad': LIMITS[mid],
             'within_sdk_limits': LIMITS[mid][0] <= motor_to_joint(mid, position) <= LIMITS[mid][1]}
            for mid, position in sorted(latest.items()) if mid<=6]
        report['joint_reference_note'] = 'SDK directions and zero offsets only; physical calibration not verified'
        if 7 in latest:
            configuration=Path(__file__).with_name('a3_commissioning.json')
            if not configuration.exists():configuration=Path(__file__).parents[1]/'config/a3_commissioning.json'
            reference=json.loads(configuration.read_text()) if configuration.exists() else {}
            if reference.get('zero_reference')=='EXISTING_MOTOR_FLASH_USER_CONFIRMED_CALIBRATED_DO_NOT_REZERO':
                report['joint_reference_note']='SDK arm directions and software offsets; existing motor Flash zero confirmed by user, unchanged by Agent'
            gripper=reference.get('gripper',{})
            report['gripper_reference']={'position_motor_rad':latest[7],
                'closed_motor_rad':gripper.get('closed_motor_rad'),'open_motor_rad':gripper.get('open_motor_rad'),
                'within_measured_limits':None,'status':'ENDPOINTS_INCOMPLETE',
                'zero_reference':reference.get('zero_reference','UNKNOWN')}
            try:bounds=GripperRange(gripper.get('closed_motor_rad'),gripper.get('open_motor_rad'))
            except ValueError:pass
            else:
                report['gripper_reference'].update(status='MEASURED_ENDPOINTS_REFERENCE_ONLY',
                    within_measured_limits=bounds.lower<=latest[7]<=bounds.upper,
                    motor_limits_rad=[bounds.lower,bounds.upper])
        report['status'] = ('PASS_READ_ONLY_ALL_7' if len(good) == 7*args.rounds and not failed_bus
                            else 'CAN_ERROR' if failed_bus else 'PARTIAL_OR_NO_RESPONSE')
    finally:
        dev.stop()
        report['final'] = dev.status()
        dev.close()
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report['status'] == 'PASS_READ_ONLY_ALL_7' else 2


if __name__ == '__main__':
    raise SystemExit(main())
