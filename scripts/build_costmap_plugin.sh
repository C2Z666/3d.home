#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
cd "$ROOT"
colcon --log-base work/costmap_build_log build --base-paths src/d1_temporal_costmap \
  --packages-select d1_temporal_costmap --build-base work/costmap_build --install-base install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
