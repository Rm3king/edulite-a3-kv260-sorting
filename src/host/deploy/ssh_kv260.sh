#!/usr/bin/env bash
# Explicit maintenance entry; uses already-installed AI PC credentials only.
set -euo pipefail
handoff_key="$HOME/.ssh/kv260_project_ed25519"
handoff_hosts="$HOME/.ssh/kv260_project_known_hosts"
test -r "$handoff_key" && test -r "$handoff_hosts"
exec ssh -i "$handoff_key" -o IdentitiesOnly=yes -o BatchMode=yes \
  -o StrictHostKeyChecking=yes -o "UserKnownHostsFile=$handoff_hosts" \
  -o ConnectTimeout=8 ubuntu@192.168.26.2 "$@"
