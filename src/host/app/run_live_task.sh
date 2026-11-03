#!/usr/bin/env bash
set -euo pipefail
app_root="${AIPC_APP_ROOT:-$HOME/Wenq_WorkSpace/EDULITE_A3_0211abadec527575/aipc_app}"
fpga_root="${AIPC_FPGA_ROOT:-$HOME/Wenq_WorkSpace/fpga_competition_2026}"
cd "$app_root/integration"
exec "$HOME/.local/share/aipc-dev/aipc.sh" sdk env \
  PYTHONPATH="$PWD:$fpga_root/contest/arm_runtime:$fpga_root/contest/kv260_can/software" \
  python -u -m workcell.live_session --app-root "$app_root" "$@"
