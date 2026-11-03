#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
source "$root/env/xilinx_2025_2.sh"
cd "$root/contest/trajectory"
mkdir -p result
stage="${1:-csim}"
log="result/${stage}_$(date +%Y%m%d_%H%M%S).log"
case "$stage" in
 csim) cmd=(vitis-run --mode hls --config hls_config.cfg --work_dir build/trajectory_eval --csim);;
 csynth) cmd=(v++ --compile --mode hls --config hls_config.cfg --work_dir build/trajectory_eval);;
 cosim) [[ -d build/trajectory_eval/hls/syn ]] || exit 2; cmd=(vitis-run --mode hls --config hls_config.cfg --work_dir build/trajectory_eval --cosim);;
 *) exit 2;;
esac
"${cmd[@]}" > "$log" 2>&1 || { tail -n 40 "$log"; exit 1; }
tail -n 15 "$log"
