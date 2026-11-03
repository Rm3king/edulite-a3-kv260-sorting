# Seven-channel degree-seven trajectory evaluator

`source/trajectory_wide.cpp` evaluates seven degree-seven polynomials with
normalized phase u in [0,1], then checks all seven motor-coordinate bounds.
Inputs/outputs use Q8.24, the phase uses unsigned Q1.24, intermediate Horner
arithmetic has 40 fractional bits. A nonzero rejection mask suppresses every
axis in the downstream stream. Overflowing coefficients must be rejected by
the ARM encoder before casting.

The corresponding ARM reference is `contest/arm_runtime/septic_trajectory.py`.
Degree-seven Hermite segments share position, velocity, acceleration and jerk
at knots. Rest-to-rest MoveJ and Cartesian progress use
`35*u^4 - 84*u^5 + 70*u^6 - 20*u^7`; IK stays on KV260 ARM.
Original and transmitted, endpoint-deduplicated coefficients are checked
separately. Mathematical C3 is not a claim about continuous physical motion
from sampled CSP targets or externally measured TCP accuracy.

Rebuild from repository root:

```bash
./contest/trajectory_wide/run.sh csim
./contest/trajectory_wide/run.sh csynth
./contest/trajectory_wide/run.sh cosim
/usr/bin/python3 scripts/sync_trajectory_wide_rtl.py
source env/xilinx_2025_2.sh
/usr/bin/python3 contest/kv260_can/run_tests.py --simulator xsim
```

`export_vectors.py` binds a reviewed C3 job to HLS vectors, including all actual
5 ms phases for original and transmitted segments. Keep its matching job as
evidence. `hls_wide_import.json` binds generated integration RTL to HLS source.

Independent HLS C simulation and RTL co-simulation passed 118701 checks, with
334 job segments and maximum arithmetic difference 2.98023223877e-8 rad from
the quantized-coefficient double reference. The synthesized operator has
latency 7 cycles and II=1 at a 100 MHz target; HLS estimates 189 DSP, 9854 FF,
21974 LUT, no BRAM/URAM. Full PS/AXI/ILA implementation metrics and attended
hardware evidence are recorded separately in `reports/c3_vertical_20261017/`.

Firmware 0x00050001 preserves legacy cubic staging at 0x900..0x9b0 / commit 1.
Degree-seven staging is 73 words at 0xa00..0xb20 / commit 8: 56 coefficients,
7 lower bounds, 7 upper bounds, ticks, phase quotient, phase remainder.
Capability register 0x828 reads 0x00070008. Check firmware version before that
read: old FPGA images do not map the new register. One active and one pending
segment, fixed 5 ms output, guard/feedback checks, hold and fault behavior remain
part of the complete CAN system. No live parameter/gain/torque mode changes are
introduced by this evaluator.
