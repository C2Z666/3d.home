#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/env_live.sh"
source /home/robot/d1_airy_offline_ws/install/setup.bash
mkdir -p "$ROOT/work"
cd "$ROOT/work"
exec 9>fastlio.lock
flock -n 9 || { echo 'FAST-LIO 已运行，请检查 work/fastlio.log'; exit 1; }
exec /home/robot/d1_airy_offline_ws/install/fast_lio/lib/fast_lio/fastlio_mapping --ros-args --params-file "$ROOT/config/fastlio_live.yaml" -p use_sim_time:=false
