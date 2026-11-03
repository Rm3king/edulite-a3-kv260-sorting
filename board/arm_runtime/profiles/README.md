# 2026-10-19 tested motion baseline

Use `release_20261019_motion.json` and `release_20261019_tracking.json` explicitly.
These are exact copies of the attended three-path ABBA profiles; global service
and historical commissioning defaults are unchanged. No motor Flash/zero writes.
The profiles are validated only for the tested unloaded paths and supported return,
not certified for the entire workspace or arbitrary payloads.

Planning caps: joints [0.575,0.345,0.345,0.575,0.575,0.575] rad/s, gripper1.2;
Kp [80,60,60,80,80,80], Kd [4,5,5,5,4,4]; gravity1, dynamics/friction0.
Current caps A and torque caps Nm are separate fields with different units.
Generate the approach from fresh pose feedback. Do not reuse an old benchmark
bank as a general current-start trajectory. Retain semantic pick/place stops;
blend interior travel waypoints with shared C3 derivatives where validated.

Firmware SHA256: e9f59c9763dd9f967de45d888a42e25b148ef5607cb6bb5d81367c9a69d39390.
Require TIM1 signature at0x100 for the new hardware timing metrics; firmware ABI
0x60002 alone cannot distinguish this extension. See
[actual evidence and limits](../../../reports/control_convergence_20261019/REPORT.md).
