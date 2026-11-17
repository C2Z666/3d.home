#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/env_live.sh"
mkdir -p "$ROOT/work"
exec 9>"$ROOT/work/planner_preview.lock"
flock -n 9 || { echo 'Planner preview already running'; exit 1; }
exec ros2 launch "$ROOT/scripts/planner_preview.launch.py" "$@"
