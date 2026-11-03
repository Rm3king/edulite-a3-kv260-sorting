#!/usr/bin/env bash
set -euo pipefail
app_root="${AIPC_APP_ROOT:-$HOME/Wenq_WorkSpace/EDULITE_A3_0211abadec527575/aipc_app}"
cd "$app_root/integration"
exec 9>"$app_root/reports/live_workbench.lock"
if ! flock -n 9; then
  echo '实物工作站已打开，请使用现有窗口。'
  exit 0
fi
export QT_IM_MODULE="${QT_IM_MODULE:-ibus}"
export XMODIFIERS="${XMODIFIERS:-@im=ibus}"
python3 -m workcell.live_startup --app-root "$app_root" --board
# Reuse running PL. Only a cold boot without the device loads the saved package;
# neither startup nor model loading enables a motor.
exec "$HOME/.local/share/aipc-dev/aipc.sh" sdk python -m workcell.desktop \
  --app-root "$app_root" --live --prewarm-model "$@"
