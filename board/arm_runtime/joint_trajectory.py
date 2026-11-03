#!/usr/bin/python3
"""Seven-channel joint-space planning contract; no CAN or MMIO imports.

Channels 1..6 are robot joint radians, channel 7 is measured gripper motor
radians (not a seventh kinematic joint). Motor coordinates are emitted
separately for the HLS boundary. Plans are NOT released for motor execution.
"""
import hashlib
import json
import math
from pathlib import Path

PROFILE = Path(__file__).with_name('motion_profile.json')
Q24 = 1 << 24


def vector(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 7:
        raise ValueError(name + ': seven numeric values required')
    if any(type(x) not in (int, float) or not math.isfinite(x) for x in value):
        raise ValueError(name + ': finite numbers required, no bool/string')
    return list(map(float, value))


def positive(value, name):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(name + ': positive finite number required')
    return float(value)


class JointPlanner:
    def __init__(self, path=PROFILE):
        raw = Path(path).read_bytes()
        self.profile_id = hashlib.sha256(raw).hexdigest()
        self.profile = p = json.loads(raw)
        self.velocity = vector(p['velocity_rad_s'], 'velocity')
        # The motor's CSP servo ceiling is distinct from the PL path derivative.
        # Legacy profiles retain their previous behavior until explicitly tuned.
        self.csp_velocity = vector(p.get('csp_velocity_limit_rad_s', self.velocity), 'CSP velocity limit')
        if any(c < v or c <= 0 for c, v in zip(self.csp_velocity, self.velocity)):
            raise ValueError('CSP servo ceiling must cover the planned velocity')
        self.acceleration = vector(p['acceleration_rad_s2'], 'acceleration')
        self.direction = vector(p['motor_directions'], 'directions')
        self.offset = vector(p['motor_offsets_rad'], 'offsets')
        self.current = vector(p['current_caps_a'], 'current')
        if any(x <= 0 for x in self.velocity + self.acceleration + self.current):
            raise ValueError('positive per-axis limits required')
        if any(x not in (-1, 1) for x in self.direction):
            raise ValueError('directions must be +/-1')
        self.lower = vector(p['position_lower_rad'], 'lower')
        self.upper = vector(p['position_upper_rad'], 'upper')
        if any(l >= h for l, h in zip(self.lower, self.upper)):
            raise ValueError('invalid position interval')
        self.tick = positive(p['tick_s'], 'tick')
        self.max_duration = positive(p['maximum_segment_s'], 'maximum segment')

    def positions(self, q):
        q = vector(q, 'position')
        if any(not l <= x <= h for x, l, h in zip(q, self.lower, self.upper)):
            raise ValueError('position outside configured limits; no automatic clipping')
        return q

    def to_motor(self, q):
        return [x*d+o for x, d, o in zip(self.positions(q), self.direction, self.offset)]

    def _duration(self, seconds):
        # Integer tick count rounds upward, so limits are not weakened.
        ticks = max(1, math.ceil(positive(seconds, 'duration') / self.tick))
        if ticks*self.tick > self.max_duration:
            raise ValueError('segment duration budget exceeded')
        return ticks, ticks*self.tick

    def _plan(self, kind, coefficients, duration, peaks_v, peaks_a):
        ticks = round(duration/self.tick)
        if ticks < 1 or not math.isclose(ticks*self.tick, duration, rel_tol=0, abs_tol=1e-12):
            raise ValueError('internal duration must already be quantized to ticks')
        motor = [[self.direction[i]*x + (self.offset[i] if j == 0 else 0)
                  for j, x in enumerate(row)] for i, row in enumerate(coefficients)]
        # Reject before a fixed-point cast; ap_fixed AP_SAT would hide overflow.
        words = [[round(x*Q24) for x in row] for row in motor]
        if any(not -(1 << 31) <= x < (1 << 31) for row in words for x in row):
            raise ValueError('HLS coefficient Q8.24 overflow')
        return {'schema': 'kv260.joint_segment.dry_run.v1', 'kind': kind,
                'profile_id': self.profile_id, 'coordinates': 'robot_rad_1_to_6_gripper_motor_rad_7',
                'coefficients_rad': coefficients, 'motor_coefficients_rad': motor,
                'hls_coefficients_q8_24': words, 'phase_convention': 'u=t/duration, 0<=u<=1',
                'ticks': ticks, 'tick_s': self.tick, 'duration_s': duration,
                'peak_velocity_rad_s': peaks_v, 'peak_acceleration_rad_s2': peaks_a,
                'executable': False, 'motion_commands_sent': 0,
                'pending': ['quantized segment extrema and HLS transport validation',
                            'CSP guard and physical servo validation',
                            'collision/clearance validation', 'jerk limits']}

    def movej(self, start, goal, speed_scale=1., minimum_duration_s=.1):
        start, goal = self.positions(start), self.positions(goal)
        scale = positive(speed_scale, 'speed scale')
        if scale > 1:
            raise ValueError('speed scale cannot exceed commissioned profile')
        duration = positive(minimum_duration_s, 'minimum duration')
        delta = [b-a for a, b in zip(start, goal)]
        for d, v, a in zip(delta, self.velocity, self.acceleration):
            duration = max(duration, 1.5*abs(d)/(v*scale), math.sqrt(6*abs(d)/(a*scale)))
        _, duration = self._duration(duration)
        c = [[q, 0., 3*d, -2*d] for q, d in zip(start, delta)]
        result = self._plan('MOVEJ_REST_TO_REST', c, duration,
                            [1.5*abs(d)/duration for d in delta],
                            [6*abs(d)/duration**2 for d in delta])
        result.update(start_rad=start, goal_rad=goal, speed_scale=scale,
                      ends_at_rest=True, tcp_straight_line=False)
        return result

    def brake(self, position, velocity):
        """Synchronized constant-deceleration candidate, analytic position check.

        Continuous q/v, bounded a, but acceleration can step at boundaries.
        Velocity must already be validated; raw Type2 spikes are unsuitable.
        This is a planned stop, not a claim that PP retargeting brakes this way.
        """
        q, v = self.positions(position), vector(velocity, 'velocity')
        if any(abs(x) > lim for x, lim in zip(v, self.velocity)):
            raise ValueError('velocity outside planning profile')
        _, duration = self._duration(max(self.tick, max(abs(x)/a for x, a in zip(v, self.acceleration))))
        stop = self.positions([x+.5*speed*duration for x, speed in zip(q, v)])
        c = [[x, speed*duration, -.5*speed*duration, 0.] for x, speed in zip(q, v)]
        out = self._plan('CONTROLLED_STOP_CANDIDATE', c, duration,
                         list(map(abs, v)), [abs(x)/duration for x in v])
        out.update(start_rad=q, goal_rad=stop, ends_at_rest=True,
                   velocity_source_requirement='VALIDATED_ESTIMATE_NOT_RAW_TYPE2',
                   physically_verified=False)
        return out


def evaluate(plan, elapsed_s):
    if type(elapsed_s) not in (float, int) or not math.isfinite(elapsed_s) or not 0 <= elapsed_s <= plan['duration_s']:
        raise ValueError('time outside segment')
    t = plan['duration_s']; u = elapsed_s/t
    c = plan['coefficients_rad']
    return {'q': [((x[3]*u+x[2])*u+x[1])*u+x[0] for x in c],
            'v': [(3*x[3]*u*u+2*x[2]*u+x[1])/t for x in c],
            'a': [(6*x[3]*u+2*x[2])/t**2 for x in c]}


class MotionContract:
    """Offline state-machine reference. No driver and no background holding.

    Hold retains the current planned setpoint only when stopped. Pause creates
    a deceleration candidate; only completed braking transitions into HOLDING.
    A missing execution/feedback path cannot promise to maintain holding torque.
    """
    def __init__(self, planner):
        self.planner = planner
        self.state = 'DISABLED'
        self.plan = None
        self.q = None
        self.elapsed = 0.

    def enable_simulated(self, q):
        if self.state != 'DISABLED':
            raise ValueError('already enabled or faulted')
        self.q = self.planner.positions(q)
        self.state = 'HOLDING'

    def movej(self, goal, **kwargs):
        if self.state != 'HOLDING':
            raise ValueError('MoveJ requires HOLDING; no queue overwrite')
        plan = self.planner.movej(self.q, goal, **kwargs)
        self.plan, self.elapsed, self.state = plan, 0., 'MOVING'
        return plan

    def advance(self, seconds):
        dt = positive(seconds, 'advance')
        if self.state not in ('MOVING', 'BRAKING'):
            return
        self.elapsed = min(self.elapsed+dt, self.plan['duration_s'])
        self.q = evaluate(self.plan, self.elapsed)['q']
        if self.elapsed == self.plan['duration_s']:
            self.q = list(self.plan['goal_rad'])
            self.plan, self.state = None, 'HOLDING'

    def pause(self):
        if self.state == 'HOLDING':
            return None
        if self.state != 'MOVING':
            raise ValueError('Pause requires MOVING or HOLDING')
        sample = evaluate(self.plan, self.elapsed)
        plan = self.planner.brake(sample['q'], sample['v'])
        self.plan, self.elapsed, self.state = plan, 0., 'BRAKING'
        return plan

    def hold(self):
        if self.state != 'HOLDING':
            raise ValueError('Hold is not an instantaneous stop; request Pause')
        return list(self.q)

    def fault(self):
        self.plan, self.state = None, 'FAULT'
        # No automatic re-enable/replay. Real fault stop is backend-specific.

    def disable_simulated(self):
        self.plan, self.state = None, 'DISABLED'
