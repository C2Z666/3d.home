#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$(dirname "$0")/.."
source "$ROOT/scripts/env_live.sh"
colcon build --packages-select d1_host_control --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
