#!/usr/bin/python3
"""Pure joint/motor transforms and conservative encoded limit generation."""
import math

DIRECTIONS = {1:-1, 2:1, 3:-1, 4:1, 5:-1, 6:1, 7:1}
OFFSETS = dict.fromkeys(range(1,8), 0.0)
LIMITS = {1:(-2.79253,2.79253),2:(0.,3.66519),3:(-4.01426,0.),
          4:(-1.5708,1.5708),5:(-1.5708,1.5708),6:(-1.5708,1.5708),7:(-1.5708,1.5708)}
# LIMITS preserves SDK reference values. Actual gripper PP travel is separate
# and uses measured motor-coordinate endpoints, without changing motor zero.


def finite(value):
    if not math.isfinite(value):raise ValueError('Non-finite joint value')
    return value


def joint_to_motor(mid, q, offsets=OFFSETS):
    return finite(q)*DIRECTIONS[mid]+finite(offsets[mid])


def motor_to_joint(mid, q, offsets=OFFSETS):
    return (finite(q)-finite(offsets[mid]))*DIRECTIONS[mid]


def check_joint(mid, q, margin=0.):
    finite(q);finite(margin)
    lo,hi=LIMITS[mid]
    if margin<0 or lo+margin>hi-margin or not lo+margin<=q<=hi-margin:
        raise ValueError(f'Joint {mid} outside configured limits')
    return q


def encoded_interval(lo, hi, protocol_lo, protocol_hi):
    if not all(math.isfinite(x) for x in (lo,hi,protocol_lo,protocol_hi)) or not protocol_lo<=lo<=hi<=protocol_hi:
        raise ValueError('Invalid encoded interval')
    # Bound the decoded quantized command, not only the unquantized input.
    lower=math.ceil((lo-protocol_lo)*65535/(protocol_hi-protocol_lo))
    upper=math.floor((hi-protocol_lo)*65535/(protocol_hi-protocol_lo))
    if lower>upper:raise ValueError('Interval smaller than protocol quantization')
    return lower | (upper<<16)


def guard_words(mid, joint_lower, joint_upper, max_velocity, max_torque, max_kp, max_kd,
                vmax, tmax, offsets=OFFSETS):
    check_joint(mid,joint_lower);check_joint(mid,joint_upper)
    if joint_lower>joint_upper or min(max_velocity,max_torque,max_kp,max_kd)<0:
        raise ValueError('Invalid commissioning limits')
    p=sorted((joint_to_motor(mid,joint_lower,offsets),joint_to_motor(mid,joint_upper,offsets)))
    if not 0<=max_kp<=500 or not 0<=max_kd<=5:raise ValueError('Invalid gains')
    return [encoded_interval(*p,-12.57,12.57),encoded_interval(-max_velocity,max_velocity,-vmax,vmax),
            encoded_interval(-max_torque,max_torque,-tmax,tmax),
            math.floor(max_kp*65535/500) | (math.floor(max_kd*65535/5)<<16)]


class GripperRange:
    """Measured PP endpoints; unknown open endpoint keeps command generation closed."""
    def __init__(self, closed_motor_rad, open_motor_rad):
        if any(type(v) not in (int,float) or not math.isfinite(v) for v in (closed_motor_rad,open_motor_rad)):
            raise ValueError('Both measured gripper endpoints are required')
        if closed_motor_rad==open_motor_rad:raise ValueError('Gripper travel must be nonzero')
        self.closed=closed_motor_rad;self.open=open_motor_rad
        self.lower,self.upper=sorted((self.closed,self.open))
    def target(self, open_fraction):
        finite(open_fraction)
        if not 0<=open_fraction<=1:raise ValueError('Opening fraction outside 0..1')
        return self.closed+(self.open-self.closed)*open_fraction
    def check(self, motor_position):
        finite(motor_position)
        if not self.lower<=motor_position<=self.upper:raise ValueError('Outside measured gripper travel')
        return motor_position


class SessionGuard:
    """ARM-side contract, latched until explicit recovery; not a motor stop."""
    def __init__(self, timeout_us=100000):
        if type(timeout_us) is not int or not 0<timeout_us<1000000:raise ValueError('Invalid timeout')
        self.timeout_us=timeout_us;self.sequence=-1;self.fault=None
    def accept(self, sequence, now_us, issued_us, expires_us, feedback_ages_us, calibrated):
        reason=None
        if self.fault:raise ValueError(self.fault)
        if type(sequence) is not int or not 0<=sequence<=0xffffffff or any(type(t) is not int or t<0 for t in (now_us,issued_us,expires_us)):reason='INVALID_HEADER'
        elif calibrated is not True:reason='CALIBRATION_UNVERIFIED'
        elif sequence<=self.sequence:reason='REPLAY_OR_REORDER'
        elif not issued_us<=now_us<expires_us or now_us-issued_us>self.timeout_us or expires_us-issued_us>self.timeout_us:reason='STALE_COMMAND'
        elif not isinstance(feedback_ages_us,dict) or set(feedback_ages_us)!=set(range(1,8)):reason='MISSING_JOINT'
        elif any(type(v) not in (int,float) or not math.isfinite(v) or not 0<=v<self.timeout_us for v in feedback_ages_us.values()):reason='STALE_FEEDBACK'
        if reason:self.fault=reason;raise ValueError(reason)
        self.sequence=sequence
    def recover(self):
        self.fault=None # Sequence remains monotonic within this session.
