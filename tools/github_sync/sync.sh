#!/usr/bin/env bash
set -euo pipefail
sync_tool_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$sync_tool_dir/sync.py" "$@"
