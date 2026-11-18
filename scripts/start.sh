#!/usr/bin/env bash
# Start services only. A navigation goal still requires web confirmation.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
source "$ROOT/scripts/env_live.sh"
exec python3 "$ROOT/scripts/start_stack.py" "$@"
