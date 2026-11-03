"""Order -> executable batches: object choice, destination slots and heat-aware batching. No hardware, camera or model I/O.

The language model only produces a `workcell.task.v1` order; everything below is deterministic and explainable, so every
decision has a Chinese reason that the operator interface and the video can show.
"""
from .tasks import validate

# J2/J3 temperature rise per part measured on the arm (2026-10-26/27 runs). The nut figures come from two-nut chains, the
# screw figure from a single run into the near slot (J2 33 -> 49 C); a screw+nut chain then rose J2 35 -> 59 C (+24, predicted +28).
RISE = {'nut': {2: 11., 3: 6.}, 'screw': {2: 17., 3: 12.}}
# M6 parts: no separate stop-and-look measurements; taken as their M12 kind (a lighter part, so an upper bound). 2026-10-31
RISE.update(screw_m6=dict(RISE['screw']), nut_m6=dict(RISE['nut']))
DEFAULT_RISE_C = 2.
START_LIMIT_C = 85.  # do not start a batch with a joint this warm
PEAK_LIMIT_C = 93.  # the board returns the arm at WRIST_RETURN_TEMPERATURE_C = 95 C (motor driver fault: 145 C)
# Cooling while disabled: J2 fell from 46 C to 35 C in about 2.5 minutes between two runs. Two observations, an estimate only.
COOL_C_PER_MIN = 2.5
# Slots 2/3 are 267/271 mm from the base, 0/1 are 330/333 mm and hold J2 at about 5 Nm instead of 4 Nm during place/open/exit.
SLOT_POOLS = dict(screw=[2, 3, 0, 1], nut=[4, 1, 0, 3])
SLOT_POOLS.update(screw_m6=SLOT_POOLS['screw'], nut_m6=SLOT_POOLS['nut'])   # as wrist_servo_trial assigns a mixed chain (by kind)
MAX_BATCH = 5
WRIST_LINE_C = 95.  # board-side return temperature (continuous_transfer.WRIST_RETURN_TEMPERATURE_C)
DESTINATIONS = {'A': 'right', 'B': 'left'}  # tray A is the source (right), tray B the left tray
ZH = {'screw': '大螺丝', 'nut': '大螺母', 'screw_m6': '小螺丝', 'nut_m6': '小螺母'}


def predicted_peak(temperatures, parts):
    """Highest joint temperature after `parts` (list of class names) run in one chained session."""
    return max(t + sum(RISE[c].get(m, DEFAULT_RISE_C) for c in parts) for m, t in temperatures.items())


def expand(task, objects):
    """Part classes in execution order from a validated order and the objects now visible in the source tray."""
    task = validate(task)
    if task['action'] != 'plan':
        raise ValueError('Only a plan can be expanded')
    parts, reasons = [], []
    for step in task['steps']:
        pool = [o for o in objects if o['part_class'] == step['part_class']]
        n = len(pool) if step['quantity'] == 'all' else step['quantity']
        if not n or len(pool) < n:
            raise ValueError(f"需要 {n} 个{step['part_class']}，盘里只有 {len(pool)} 个")
        parts += [step['part_class']] * n
    return parts


def assign_slots(parts, occupied=()):
    used, slots = list(occupied), []
    for cls in parts:
        free = [s for s in SLOT_POOLS[cls] if s not in used]
        if not free:
            raise ValueError('目标盘里没有足够的空位')
        slots.append(free[0]);used.append(free[0])
    return slots


AMBIENT_C = 25.


def cooling_needed_min(temperatures, parts, cool_c_per_min):
    """Minutes of cooling before `parts` may start as one chain: below the start limit and under the peak limit afterwards.

    Infinite when even a fully cooled (ambient) joint would pass the limit.
    """
    wait = 0.
    for motor, temp in temperatures.items():
        rise = sum(RISE[c].get(motor, DEFAULT_RISE_C) for c in parts)
        need = max(temp - (START_LIMIT_C - 1.), temp + rise - PEAK_LIMIT_C, 0.)
        if temp - need < AMBIENT_C - 1e-9 and need > 0 and AMBIENT_C + rise > PEAK_LIMIT_C:
            return float('inf')
        wait = max(wait, need / cool_c_per_min)
    return wait


MAX_WAIT_MIN = 3.  # wait this long for a bigger chain rather than split it (each session costs a prepare and a restart)


def batches(temperatures, parts, cool_c_per_min=COOL_C_PER_MIN, occupied=(), skip_cooling=False):
    """Split `parts` into chained sessions that stay under the peak limit, with the estimated cooling between them.

    Returns [dict(parts, slots, start_temperatures, predicted_peak_c, cool_before_min, reason)]. The longest chain whose
    cooling wait is at most MAX_WAIT_MIN is taken; if none is, the next single part waits as long as it must.
    `skip_cooling` (the operator's order) never waits: one chain of up to MAX_BATCH parts, the board's return line is the protection.
    """
    slots, out, temps, i = assign_slots(parts, occupied), [], dict(temperatures), 0
    if skip_cooling:
        if max(temps.values()) >= START_LIMIT_C:
            raise ValueError(f'机械臂已经 {max(temps.values()):.0f}°C，超过起步上限 {START_LIMIT_C:.0f}°C，不能跳过降温')
        chunk = parts[:MAX_BATCH];peak = predicted_peak(temps, chunk)
        note = '' if peak <= PEAK_LIMIT_C else f'，预测超过 {PEAK_LIMIT_C:.0f}°C，到 {WRIST_LINE_C:.0f}°C 板端会提前返回'
        return [dict(parts=chunk, slots=slots[:len(chunk)], start_temperatures=dict(temps), predicted_peak_c=round(peak, 1), cool_before_min=0.,
                     reason=f"{'、'.join(ZH[c] for c in chunk)}：按指令不等降温，预测峰值 {peak:.0f}°C{note}")]
    while i < len(parts):
        room = min(MAX_BATCH, len(parts) - i)
        waits = {k: cooling_needed_min(temps, parts[i:i + k], cool_c_per_min) for k in range(1, room + 1)}
        fit = [k for k, w in waits.items() if w <= MAX_WAIT_MIN]
        n = max(fit) if fit else 1
        wait = round(waits[n], 1)
        if wait:
            temps = {m: max(t - cool_c_per_min * wait, AMBIENT_C) for m, t in temps.items()}
        chunk = parts[i:i + n];peak = predicted_peak(temps, chunk)
        out.append(dict(parts=chunk, slots=slots[i:i + n], start_temperatures=dict(temps), predicted_peak_c=round(peak, 1), cool_before_min=wait,
                        reason=f"{'、'.join(ZH[c] for c in chunk)}：预测 J2/J3 峰值 {peak:.0f}°C，" + ('不需要等待' if not wait else f'先冷却约 {wait:.1f} 分钟')))
        temps = {m: t + sum(RISE[c].get(m, DEFAULT_RISE_C) for c in chunk) for m, t in temps.items()}
        i += n
    return out


def unsupported_reason(task):
    """The executor picks from the right tray (A) and places into the left tray (B). Anything else is refused whole, in words."""
    if task['action'] != 'plan':
        return None
    from .reverse_transfer import requested
    if requested(task):
        return None
    for index, step in enumerate(task['steps'], 1):
        if step['destination'] != 'B' or step['region'] == 'left':
            where = '左盘（B）' if step['region'] == 'left' else '右盘（A）'
            return (f"第 {index} 步是把{ZH[step['part_class']]}从{where}放到{'左' if step['destination'] == 'B' else '右'}盘，"
                    '目前支持右盘（A）到左盘（B），反方向仅支持左盘的一颗小螺母放回右盘。这条指令我没有执行。')
    return None
