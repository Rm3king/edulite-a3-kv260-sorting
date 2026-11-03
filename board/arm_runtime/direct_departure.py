"""Synchronized C3 departure/return; no single-joint clearance preamble.

All moving joints share one progress law. Geometry review must use the actual
encoded path; the caller cannot infer Cartesian clearance from endpoint poses.
"""
from septic_trajectory import movej_c3, movej_cruise_c3, continuity_metrics


def direct_departure(planner, start, goal, open_during_departure=False, observed_closed_start=None, observed_wide_start=None, cruise=False):
    start = planner.positions(start)
    goal = planner.positions(goal)
    upper=1.62
    if observed_closed_start is not None:
        if not open_during_departure or not 1.62<observed_closed_start<=1.70 or abs(start[6]-observed_closed_start)>.0001:
            raise ValueError('Observed closed start only permits initial opening')
        upper=observed_closed_start+.0001
    if observed_wide_start is not None:
        if (observed_closed_start is not None or not open_during_departure or
                not 0.<=observed_wide_start<1.08 or abs(start[6]-observed_wide_start)>.0001 or goal[6]!=1.12):
            raise ValueError('Measured wide start only permits normalization to working opening')
    elif open_during_departure and not 1.08 <= goal[6] <= start[6] <= upper:
        raise ValueError('Concurrent aperture action must only open within bounds')
    if not open_during_departure and abs(start[6] - goal[6]) > 1e-10:
        raise ValueError('Hold aperture during synchronized arm travel')
    segments = (movej_cruise_c3 if cruise else movej_c3)(planner, start, goal)
    return segments, dict(
        mode='SYNCHRONIZED_DIRECT',
        duration_s=sum(s['duration_s'] for s in segments),
        single_joint_preamble=False,
        interior_rest_waypoints=0,
        quantized_continuity=continuity_metrics(segments, planner, True),
        geometry_review_required=True,
    )
