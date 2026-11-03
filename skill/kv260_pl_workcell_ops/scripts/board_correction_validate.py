#!/usr/bin/python3
"""Online correction channel (840..8b4) on the real board, internal CAN mode: no external CAN output, no enable.

Internal mode cannot start the trajectory stream (no motor feedback, so no motion mask), so the per-tick slew of the
offsets is covered by tb_trajectory_correction (179 ticks) and the attended motion test. This checks what only the
silicon can: the channel is in the loaded bitstream, its registers read back, every refused write is ignored and counted
without an AXI error (SLVERR can abort the A53), clamping, disable-to-zero targets and the reset when CAN stops.
"""
import json
from canctl import CanDevice, correction_channel_present

Q24 = 1 << 24


def signed(v):
    return ((v & 0xffffffff) ^ 0x80000000) - 0x80000000


def validate():
    report = dict(scope='INTERNAL_ONLY_NO_PHYSICAL_CAN_OR_ENABLE', checks=[])

    def check(name, ok, **detail):
        report['checks'].append(dict(name=name, ok=bool(ok), **detail))
        if not ok:
            raise AssertionError(name + ' ' + json.dumps(detail))
    if not correction_channel_present():
        raise RuntimeError('deployment bundle has no correction channel marker for its bitstream; nothing touched')
    d = CanDevice()
    try:
        if d.read(0x404) or d.read(0x5a8) or d.read(0x5a0):
            raise RuntimeError('stopped disarmed board required')
        d.configure('internal')
        check('channel id', d.read(0x894) == 0x43520001, id=hex(d.read(0x894)))
        check('septic capability unchanged', d.read(0x828) == 0x00070008)
        rejects = lambda: d.read(0x898)
        r0 = rejects()
        d.write(0x840, 1)
        check('enable without a configuration is refused, no bus error', d.read(0x840) & 1 == 0 and rejects() == r0 + 1)
        limits = [int(.12 * Q24), int(.18 * Q24), int(.2 * Q24), int(.06 * Q24), int(.01 * Q24), int(.12 * Q24)]
        for j, v in enumerate(limits):
            d.write(0x8a0 + 4 * j, v)
        check('limits read back', [d.read(0x8a0 + 4 * j) for j in range(6)] == limits)
        r0 = rejects()
        d.write(0x8a0, int(.25 * Q24) + 1); d.write(0x848, 50333); d.write(0x84c, 0); d.write(0x84c, 500001)
        check('values above the ceilings are refused and counted', rejects() == r0 + 4 and d.read(0x8a0) == limits[0], rejects=rejects() - r0)
        d.write(0x848, int(.0015 * Q24)); d.write(0x84c, 200000)
        check('step and lease read back', d.read(0x848) == int(.0015 * Q24) and d.read(0x84c) == 200000)
        r0 = rejects()
        d.write(0x850, int(.01 * Q24))
        check('a target while disabled is refused', rejects() == r0 + 1 and d.read(0x850) == 0)
        d.write(0x840, 1)
        check('enable after configuration', d.read(0x840) & 1 == 1)
        writes = d.read(0x888)
        d.write(0x850, int(.01 * Q24)); d.write(0x854, -int(.02 * Q24))
        check('targets inside the limits read back', signed(d.read(0x850)) == int(.01 * Q24) and signed(d.read(0x854)) == -int(.02 * Q24)
              and d.read(0x888) == writes + 2)
        clamps = d.read(0x88c)
        d.write(0x860, int(.2 * Q24)); d.write(0x864, -int(.2 * Q24))
        check('targets beyond a joint limit are clamped and counted', signed(d.read(0x860)) == limits[4] and signed(d.read(0x864)) == -limits[5]
              and d.read(0x88c) == clamps + 2)
        check('lease age counts', 0 <= d.read(0x890) < 1000000, age_us=d.read(0x890))
        check('no offset without a running stream', all(d.read(0x870 + 4 * j) == 0 for j in range(6)))
        d.write(0x840, 0)
        check('disable clears every target', d.read(0x840) & 1 == 0 and all(d.read(0x850 + 4 * j) == 0 for j in range(6)))
        check('no CAN frame submitted', d.read(0x5a8) == 0)
        report['status'] = 'PASS_INTERNAL_REGISTER_BANK'
    finally:
        d.stop()
        report['after_stop'] = dict(ctrl=d.read(0x840), limit0=d.read(0x8a0), step=d.read(0x848), rejects=d.read(0x898))
        # ctrl bit1 is the read-only stale flag (lease 0 after the reset is always expired); bit0 is the enable
        a = report['after_stop']
        report['reset_on_stop'] = a['ctrl'] & 1 == 0 and a['limit0'] == a['step'] == a['rejects'] == 0
        d.close()
    return report


if __name__ == '__main__':
    print(json.dumps(validate(), indent=2))
