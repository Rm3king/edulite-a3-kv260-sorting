#!/usr/bin/env bash
# One-sentence operator console: starts the local server (and the local Qwen worker) once, then opens it as a full-screen app window.
set -euo pipefail
app_root="${AIPC_APP_ROOT:-$HOME/Wenq_WorkSpace/EDULITE_A3_0211abadec527575/aipc_app}"
port="${CONSOLE_PORT:-8780}"
cd "$app_root/integration"
if ! curl -fs "http://127.0.0.1:$port/api/state" >/dev/null 2>&1; then
  PYTHONPATH=. nohup "$HOME/.local/share/aipc-dev/aipc.sh" sdk python -u -m workcell.console --port "$port" >"$app_root/reports/console.log" 2>&1 &
  echo $! >"$app_root/reports/console.pid"
  for _ in $(seq 1 60); do curl -fs "http://127.0.0.1:$port/api/state" >/dev/null 2>&1 && break; sleep 1; done
fi
echo "控制台已启动：http://127.0.0.1:$port/"
# A running Firefox ignores --kiosk and only adds a tab to its existing window (2026-10-31: the console seemed not to open):
# then open a new window instead. --kiosk hides the browser frame (Alt+F4 closes the window; the server keeps running).
if pgrep -x firefox >/dev/null 2>&1 || pgrep -f '/snap/firefox/.*/firefox$' >/dev/null 2>&1; then
  exec firefox --new-window "http://127.0.0.1:$port/"
fi
exec firefox --kiosk "http://127.0.0.1:$port/"
