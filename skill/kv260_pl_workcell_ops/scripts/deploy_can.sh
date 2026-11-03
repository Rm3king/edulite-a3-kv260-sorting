#!/usr/bin/env bash
# Explicit board deployment only. No fixed host, private SSH configuration or motor enable.
set -euo pipefail
if [[ $# -lt 2 || $# -gt 3 || "$1" == --help ]]; then
    echo 'Usage: bash deploy_can.sh USER@HOST BUNDLE_DIR [SSH_CONFIG]'
    echo 'BUNDLE_DIR is the .cache/kv260_can_deploy produced by package_can.py.'
    [[ "${1:-}" != --help ]] && exit 2 || exit 0
fi
target="$1"
bundle="$(cd "$2" && pwd)"
[[ "$target" =~ ^[a-zA-Z0-9_.-]+@[a-zA-Z0-9_.-]+$ ]] || { echo 'Expected USER@HOST (SSH hostname or IPv4).' >&2; exit 2; }
[[ -f "$bundle/install_board.sh" && -f "$bundle/SHA256SUMS" ]] || { echo 'Incomplete bundle; run package_can.py first.' >&2; exit 2; }
(cd "$bundle" && sha256sum --check SHA256SUMS)
ssh_args=()
[[ $# != 3 ]] || ssh_args=(-F "$3")
ssh "${ssh_args[@]}" "$target" 'mkdir -p ~/kv260_can_deploy'
scp "${ssh_args[@]}" "$bundle/"* "$target:kv260_can_deploy/"
ssh -t "${ssh_args[@]}" "$target" 'cd ~/kv260_can_deploy && sudo bash install_board.sh'
