#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [[ $# -ne 1 ]]; then echo "Usage: bash scripts/run_nav_bridge.sh ROBOT_IP" >&2; exit 2; fi
if pgrep -x d1_bridge >/dev/null; then echo '已有 d1_bridge，请先结束原控制会话。' >&2; exit 1; fi
source "$ROOT/scripts/env_live.sh"
source "$ROOT/install/setup.bash"
exec ros2 run d1_host_control d1_bridge --ros-args -r __ns:=/d1/sdk \
 -r cmd_vel:=supervised_cmd_vel -p "robot_ip:=$1" \
 -p max_lin_vel:=0.15 -p max_ang_vel:=0.4 \
 -p max_lin_accel:=0.5 -p max_ang_accel:=1.0 -p watchdog_timeout:=0.3 -p trace_control:=true
