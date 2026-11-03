# A3 batch FK and sampled TCP-region audit

This accelerator accepts six model-joint septic polynomials, samples the whole
segment, computes the legacy URDF `end_effector` transform, and returns XYZ
extrema, violation count/first index/axis mask, and the endpoint3x4 transform.
It shares one function between planning batches and single-pose observation.
It never sends CAN, changes motor permits, resets a drive, or changes feedback.

The input is **URDF joint radians**, not signed raw motor radians. Directions and
zero offsets are applied once on ARM. Coefficients/bounds/output are signedQ8.24,
metres for translation; rotations are matrix entries. Six revolute origins and
one fixed TCP origin are generated from the exact legacy URDF, including its
rounded1.5708RPY values. This frame is not a calibrated physical tool/table frame.

Run `./contest/fk_batch/run.sh csim`, `csynth`, then `cosim`.
`/usr/bin/python3 scripts/validate_fk_batch_ip.py --write` binds the generated IP
and source hashes after C/RTL PASS. Full system `scripts/build_can.sh bitstream`
instantiates isolated AXI-Lite IP at0xA0010000, alongside CAN at0xA0000000.
The project UIO map is128KiB; the original CAN API still maps its first4KiB.
HLS estimates and full routed system metrics are separate evidence.

HLS control:0x00ap_ctrl,0x10sample count(1..4097),0x10048 coefficient words
(axis-major ascending powers),0x200six bounds(xmin,xmax,ymin,ymax,zmin,zmax),
0x30032 result words. Result0=FKB1 signature;1invalid-config/domain flags;
2processed;3violations;4first violation(-1if none);5XYZ outside bits plus
bit6domain invalid;6..11interleavedXYZ minima/maxima;12..23row-major3x4
endpoint;24URDF SHA prefix;25sampled-only flag. Remaining words reserved0.
HLS control interrupts are unused; polling never touches CAN state.

`client.BatchFK.audit()` is planning-only; `observe()` accepts six current model
joint angles. `audit_bank.py` reports all sampled points from trusted project
banks and marks <=10um boundary margin for software refinement. It does not
prove continuous clearance, whole-arm collision freedom, or accurate physical
TCP. In particular, it does not add an automatic-disable condition.
`observe_capture.py` converts a previously captured read-only motor sample with
the explicit motion profile; Type17 reads are sequential, not time-synchronous.

Build native library on the target with:
`g++ -O3 -Wall -Wextra -Werror -shared -fPIC -I source float_baseline.cpp native_transport.cpp -o libfk_batch.so`.
The native implementation uses double arithmetic and is independently checked
against Pinocchio. HLS uses bounded fixed arithmetic and28-step CORDIC. Horner
intermediates cannot exceed8*128rad for Q8.24 coefficients and phase[0,1], so
40-bitQ12.28 wrap arithmetic has no valid-input overflow. Valid FK rotation/
translation intermediates fit the Q4.28 range; joint domain is separately
reported. Conditional sign changes avoid unnecessary32-bit multipliers.

`benchmark.py` validates random poses and every segment of the three tested
banks, then compares native C++ and actual PL end-to-end including all register
I/O. It explicitly reports both native callable timings and public Python API
whole-path costs. Busy polling is used for latency tests; a CPU utilization
reduction or maximum theoretical throughput is not inferred from these times.
Small workloads may favor native ARM. Original negative area experiments and
all benchmark results remain in[the report directory](../../reports/fk_batch_20261019).


The full application comparison also accepts the original double planner
coefficients: native C++ evaluates them directly without PL quantization;
PL includes quantization, all register I/O and decode. Those results are
separate from pre-encoded interface timings. Both ordinary and8x dense
planning grids are reported. The native baseline is a direct-O3 rigid-chain
implementation, not a claim of the best possible cached/vectorized ARM code.
`submit()`/`poll_result()` expose asynchronous completion so other ARM work can
run between calls; the functional overlap test does not infer CPU percentages.


Validated2026-10-19: the final full design uses74820LUT/451DSP/3.5BRAM at100MHz,
WNS+0.050ns. Full original-double application speedup on8x dense blend/circle/
pick audits is1.76x/1.57x/1.64x; original grids only1.07x/1.02x/1.03x. Single
pose PL latency is higher, so the default single-pose/IK backend remains ARM.
The first Python-marshalling implementation was slower and is preserved.
Native finite/range checks and float quantization precede all PL register writes;
this preserves validation while avoiding repeated NumPy dispatch overhead.

For dense planning reports use `audit_bank.py --backend pl --sampling-multiplier 8`
with explicit bank/library/output paths. The default region is only a broad
model-space test region; supply a calibrated task region for actual planning.
See[completed evidence](../../reports/fk_batch_20261019/REPORT.md).
