#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
# Only the costmap plugin is added; no SDK bridge overlay is replaced.
D1_COSTMAP_PREFIX="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/install/d1_temporal_costmap"
if [ -d "$D1_COSTMAP_PREFIX/share/ament_index" ]; then
  export AMENT_PREFIX_PATH="$D1_COSTMAP_PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$D1_COSTMAP_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
unset D1_COSTMAP_PREFIX
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export PYTHONNOUSERSITE=1
export OPENBLAS_NUM_THREADS=1
export PYTHONUTF8=1
export PYTHONIOENCODING=utf-8
