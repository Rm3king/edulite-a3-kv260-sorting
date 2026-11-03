"""Bounded stopped-only feedback setup. Never enables or disables motors."""
from arm_commission import parameter


def setup_stopped_feedback(device, session):
    state={hex(a):device.read(a) for a in (0x404,0x418,0x5a0,0x5a8,0x800,0xb40)}
    if state['0x404'] or state['0x5a8']:
        raise RuntimeError('Controller still active: '+str(state))
    if state['0x800'] or state['0xb40']:
        raise RuntimeError('Stopped stream state not cleared')
    # Stream status is read-only; RTL clears streams while control[0] is zero.
    try:
        device.write(0x418,0)
        device.write(0x5a0,127)
        for mid in range(1,8):device.write(device.pp_base(mid),1)
        device.configure('normal')
        for mid in range(1,8):
            session.send(parameter(mid,0x7028,6000,True))
            session.pump(.012)
            session.require(mid,0)
    finally:
        device.stop()
        device.write(0x5a0,0)
        for mid in range(1,8):device.write(device.pp_base(mid),0)
    return state
