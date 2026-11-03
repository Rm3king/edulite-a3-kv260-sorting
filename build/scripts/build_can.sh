#!/usr/bin/env bash
set -euo pipefail
entry="$(cd "$(dirname "$0")" && pwd)"
while [[ "$entry" != / ]]; do
    for relative in build/scripts/rebuild.py fpga/build/scripts/rebuild.py; do
        if [[ -f "$entry/$relative" ]]; then
            exec python3 "$entry/$relative" build "$@"
        fi
    done
    entry="$(dirname "$entry")"
done
echo 'Keep this script inside the complete submission package' >&2
exit 2
