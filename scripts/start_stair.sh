#!/usr/bin/env bash
set -eo pipefail
echo '请沿用实测的双终端流程：'
echo '终端1: bash scripts/run_stair_bridge.sh 192.168.168.168 0.5'
echo '终端2: python3 scripts/stair_start.py --duration 本次最大前进秒数'
echo '不要同时运行 stair_start.py 和 nav_to_stair.py。'
exit 2
