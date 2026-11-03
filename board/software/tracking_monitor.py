#!/usr/bin/python3
"""Board-side access to the PL tracking-error / torque-saturation monitor (registers 0x200..0x2FF, observation only).

    python3 tracking_monitor.py status      # read-only
    python3 tracking_monitor.py validate    # internal loopback with synthetic frames; the external CAN TX stays inhibited, no motor is touched

The monitor never gates motion: it latches the time (us, same counter as register 0x408) at which a tracking error or a torque
saturation has persisted for its hold time. validate checks that on real hardware and reports how precisely the PL trip follows the
configured hold; the comparison with the ARM's 0.25 s Python polling is done by the caller with these same time stamps.
"""
import argparse
from pathlib import Path
import json
import struct
import time

from canctl import CanDevice

BASE = 0x200
REG = dict(magic=0x200, control=0x204, select=0x208, err_hold=0x20c, tq_hold=0x210, trip_mask=0x214, first_trip_us=0x218,
           first_trip_axis=0x21c, err_thr=0x220, tq_thr=0x224, cmd_pos=0x228, act_pos=0x22c, cur_err=0x230, peak_err=0x234,
           err_first_over_us=0x238, err_trip_us=0x23c, act_tq=0x240, peak_tq=0x244, tq_first_over_us=0x248, tq_trip_us=0x24c,
           samples=0x250)
CODE_RAD = 25.14 / 65535
MAGIC = 0x54524b31


class TrackingMonitor:
    def __init__(self, dev):
        self.dev = dev
        if dev.read(REG['magic']) != MAGIC:
            raise RuntimeError('tracking monitor not present in this bitstream (signature 0x%08x)' % dev.read(REG['magic']))

    def select(self, axis):
        self.dev.write(REG['select'], axis)

    def axis(self, axis):
        self.select(axis)
        return {name: self.dev.read(REG[name]) for name in ('err_thr', 'tq_thr', 'cmd_pos', 'act_pos', 'cur_err', 'peak_err',
                'err_first_over_us', 'err_trip_us', 'act_tq', 'peak_tq', 'tq_first_over_us', 'tq_trip_us', 'samples')}

    def status(self):
        d = self.dev
        return dict(enabled=d.read(REG['control']) & 1, any_trip=(d.read(REG['control']) >> 1) & 1, err_hold_us=d.read(REG['err_hold']),
                    tq_hold_us=d.read(REG['tq_hold']), err_trip_mask=d.read(REG['trip_mask']) & 0x7f,
                    tq_trip_mask=(d.read(REG['trip_mask']) >> 8) & 0x7f, first_trip_us=d.read(REG['first_trip_us']),
                    first_trip_axis=d.read(REG['first_trip_axis']), now_us=d.read(0x408))

    def configure(self, err_codes=None, tq_codes=None, err_hold_us=None, tq_hold_us=None):
        """Only while disabled. err_codes / tq_codes: {axis: code}; 0 disables that check."""
        self.dev.write(REG['control'], 0)
        for axis, code in (err_codes or {}).items():
            self.select(axis); self.dev.write(REG['err_thr'], code)
        for axis, code in (tq_codes or {}).items():
            self.select(axis); self.dev.write(REG['tq_thr'], code)
        if err_hold_us:
            self.dev.write(REG['err_hold'], err_hold_us)
        if tq_hold_us:
            self.dev.write(REG['tq_hold'], tq_hold_us)

    def start(self):
        self.dev.write(REG['control'], 3)   # clear latched state, then enable
        self.dev.write(REG['control'], 1)

    def stop(self):
        self.dev.write(REG['control'], 0)


def command_frame(motor, position):
    return (1 << 24) | (32768 << 8) | motor, struct.pack('>HHHH', position, 32768, 0, 0)


def feedback_frame(motor, position, torque):
    return (2 << 24) | (motor << 8) | 0xfd, struct.pack('>HHHH', position, 32768, torque, 250)


def pump(dev, ident, payload):
    dev.heartbeat()
    dev.enqueue(ident, payload)
    deadline = time.monotonic() + .05
    while time.monotonic() < deadline:
        dev.heartbeat()
        if dev.receive():
            return
        time.sleep(.0005)
    raise TimeoutError('loopback frame not received')


def validate():
    dev = CanDevice()
    mon = TrackingMonitor(dev)
    result = dict(checks=[])

    def check(name, ok, **detail):
        result['checks'].append(dict(name=name, ok=bool(ok), **detail))
        if not ok:
            raise AssertionError(name + ' ' + json.dumps(detail))
    try:
        dev.configure('internal')
        mon.configure(err_codes={0: 105, 1: 105}, tq_codes={2: 3000}, err_hold_us=100000, tq_hold_us=100000)
        mon.start()
        start_us = dev.read(0x408)
        # axis 1: tracked (error 20 codes); axis 2 (motor 2): lags by 150 codes (57 mrad); axis 3 (motor 3): torque at +3500 codes
        pump(dev, *command_frame(1, 20000)); pump(dev, *command_frame(2, 30000)); pump(dev, *command_frame(3, 5000))
        for _ in range(60):
            pump(dev, *feedback_frame(1, 20020, 32768 + 100))
            pump(dev, *feedback_frame(2, 29850, 32768))
            pump(dev, *feedback_frame(3, 5002, 32768 + 3500))
            time.sleep(.004)
        trip = mon.status()
        a0, a1, a2 = mon.axis(0), mon.axis(1), mon.axis(2)
        check('tracked axis does not trip', not trip['err_trip_mask'] & 1 and a0['peak_err'] == 20, peak_err_codes=a0['peak_err'])
        check('lagging axis trips', trip['err_trip_mask'] & 2, status=trip)
        hold_error_us = (a1['err_trip_us'] - a1['err_first_over_us']) - 100000
        check('PL trip follows the 100 ms hold within one feedback interval', 0 <= hold_error_us < 20000, hold_error_us=hold_error_us)
        check('torque saturation trips its own flag', trip['tq_trip_mask'] & 4 and not trip['err_trip_mask'] & 4,
              peak_tq=a2['peak_tq'], status=trip)
        check('first trip recorded', trip['any_trip'] == 1, first_trip_us=trip['first_trip_us'], axis=trip['first_trip_axis'])
        result.update(status='PASS_INTERNAL_LOOPBACK_ONLY', physical_bus_tested=False, external_tx_inhibited=True,
                      monitor_registers=trip, axis_registers=dict(axis0=a0, axis1=a1, axis2=a2), elapsed_us=dev.read(0x408) - start_us,
                      code_to_mrad=CODE_RAD * 1000)
        return result
    finally:
        mon.stop()
        dev.stop()


def watch(out, err_codes=105, err_hold_us=300000):
    """Monitor-only observer for a real run: arm the PL monitor, poll until SIGTERM/SIGINT, write per-axis peaks to `out`.

    It only reads PL registers, never touches the motion chain's CAN submission; stop it after the run finishes."""
    import signal, time
    stop = []
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stop.append(1))
    dev = CanDevice()
    mon = TrackingMonitor(dev)
    mon.configure(err_codes={a: err_codes for a in range(7)}, tq_codes={a: 0 for a in range(7)}, err_hold_us=err_hold_us)
    mon.start()
    begin = dev.read(0x408)
    while not stop:
        time.sleep(.05)
    result = dict(started_us=begin, status=mon.status(), axes=[mon.axis(a) for a in range(7)],
                  err_thr_codes=err_codes, err_hold_us=err_hold_us, code_to_mrad=CODE_RAD * 1000)
    mon.stop()
    Path(out).write_text(json.dumps(result, indent=1))
    print(json.dumps(result['status']))


def wait_alive(dev, seconds):
    """Sleep while keeping the 100 ms software lease alive (an expired lease disables the CAN engine, by design)."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        dev.heartbeat()
        time.sleep(min(.01, max(0., end - time.monotonic())))


def latency(trials=20, hold_us=300000, arm_period_s=.15):
    """Detection latency of a sustained 150-code (57 mrad) lag, internal loopback: the PL monitor against the ARM rule.

    The ARM rule is the executor's diagnostics(): sample at most every 0.15 s, trip when the error stayed above 0.04 rad for
    more than 0.3 s. Here it is applied to register snapshots at that period, so the comparison isolates the sampling, not
    the extra CAN parameter reads the real ARM guard needs. Onset = first lagging feedback frame (PL time base).
    """
    import random
    dev = CanDevice()
    mon = TrackingMonitor(dev)
    rows = []
    try:
        dev.configure('internal')
        for trial in range(trials):
            mon.configure(err_codes={1: 105}, tq_codes={a: 0 for a in range(7)}, err_hold_us=hold_us)
            mon.start()
            pump(dev, *command_frame(2, 30000))
            for _ in range(5):
                pump(dev, *feedback_frame(2, 30000, 32768)); wait_alive(dev, .004)
            # the ARM samples run on their own clock: first sample a random fraction of a period after the onset
            onset_us, arm_since, arm_trip_us, next_sample = None, None, None, time.monotonic() + random.uniform(0, arm_period_s)
            deadline = time.monotonic() + (hold_us / 1e6) + 3 * arm_period_s + .2
            while time.monotonic() < deadline and arm_trip_us is None:
                pump(dev, *feedback_frame(2, 29850, 32768))
                if onset_us is None:
                    onset_us = mon.axis(1)['err_first_over_us']
                if time.monotonic() >= next_sample:
                    next_sample += arm_period_s
                    a = mon.axis(1)
                    now_us = dev.read(0x408)
                    if abs(a['cmd_pos'] - a['act_pos']) > 105:
                        arm_since = now_us if arm_since is None else arm_since
                        if now_us - arm_since > hold_us:
                            arm_trip_us = now_us
                    else:
                        arm_since = None
                wait_alive(dev, .004)
            a = mon.axis(1)
            rows.append(dict(pl_ms=(a['err_trip_us'] - onset_us) / 1e3 if a['err_trip_us'] else None,
                             arm_ms=(arm_trip_us - onset_us) / 1e3 if arm_trip_us else None))
            mon.stop()
    finally:
        mon.stop()
        dev.stop()
    pl = [r['pl_ms'] for r in rows if r['pl_ms'] is not None]
    arm = [r['arm_ms'] for r in rows if r['arm_ms'] is not None]
    q = lambda v: dict(min=round(min(v), 1), p50=round(sorted(v)[len(v) // 2], 1), max=round(max(v), 1)) if v else None
    return dict(status='INTERNAL_LOOPBACK_ONLY', trials=trials, hold_ms=hold_us / 1e3, arm_period_s=arm_period_s,
                pl_detect_ms=q(pl), arm_rule_detect_ms=q(arm), pl_detected=len(pl), arm_detected=len(arm), rows=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['status', 'validate', 'watch', 'latency'])
    parser.add_argument('--out', default='tracking_watch.json')
    args = parser.parse_args()
    if args.command == 'watch':
        return watch(args.out)
    if args.command == 'latency':
        return print(json.dumps(latency(), indent=1))
    if args.command == 'status':
        mon = TrackingMonitor(CanDevice())
        print(json.dumps(dict(monitor=mon.status(), axes=[mon.axis(a) for a in range(7)]), indent=2))
    else:
        print(json.dumps(validate(), indent=2))


if __name__ == '__main__':
    main()
