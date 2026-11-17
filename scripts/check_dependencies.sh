#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/env_live.sh"
python3 -c 'import rclpy,cv_bridge,numpy,scipy,yaml,cv2; assert hasattr(cv2,"SIFT_create"); print("Python/ROS dependencies OK")'
for package in nav2_map_server nav2_controller nav2_planner nav2_bt_navigator nav2_lifecycle_manager nav2_navfn_planner dwb_core dwb_plugins dwb_critics nav2_velocity_smoother; do
 ros2 pkg prefix "$package"
done
