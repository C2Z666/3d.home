#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/env_live.sh"
exec python3 "$ROOT/scripts/walk_reset.py" "$@"
