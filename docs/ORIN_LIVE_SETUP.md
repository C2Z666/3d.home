# 实机接通记录
日期：2026-11-14（北京时间）

已通过 rk3588 跳板连接 orin。
FAST-LIO 使用 /home/robot/zhangzhang/fastlio_live/d1_airy_live.yaml。
工厂 /ota/calibration_results.yaml 的 lidar_front_to_imu_front 与原离线标定完全一致。
旋转矩阵按原 fastlio_video.py 执行 SVD 正交化；不使用单位外参。
ROS_DOMAIN_ID=24；RMW_IMPLEMENTATION=rmw_zenoh_cpp；实时系统时间。

10 秒实测：101 个 Odometry 样本，约 10.014 Hz。
时间戳严格递增，位姿数值有效，消息延迟 0.040–0.085 秒。
里程计：/Odometry，d1_lio_init -> d1_imu，参考点为前雷达 IMU。
图像：/front_camera/image，1920x1080 rgb8。
原始点云：/front_lidar，rslidar_head；约 10 Hz。
IMU：/front_lidar/imu，约 199 Hz。
扫描：/d1/nav/scan；状态 /d1/nav/scan_status ready=true。

本次狗不在原建图场景，视觉初始定位 ready=false 是当前实测结果。
未保存入口；未标记地图/扫描人工验收；未启动 Nav2、任务执行器或 SDK 控制桥。
已有上楼项目 d1_stair_nav_pck 未修改。

默认 Python 用户目录中的 NumPy 2.2.6 与 ROS Humble/OpenCV 不兼容。
第三阶段 env_live.sh 设置 PYTHONNOUSERSITE=1，使用已有系统 NumPy 1.21.5。
没有修改全局依赖。OPENBLAS_NUM_THREADS=1 限制小矩阵处理的线程开销。

在 Orin 上使用：
cd /home/robot/zhangzhang/d1_stair_nav_phase3_orin_scan_axisfix_20261116
python3 scripts/sensor_stack.py status
python3 scripts/sensor_stack.py start
python3 scripts/sensor_stack.py stop

start 仅启动 FAST-LIO、视觉定位与点云扫描；不会启动导航或运动控制。

查看 FAST-LIO 日志：
tail -n 80 /home/robot/zhangzhang/fastlio_live/work/fastlio.log

查看第三阶段日志：
tail -n 80 /home/robot/zhangzhang/d1_stair_nav_phase3_orin_scan_axisfix_20261116/work/localization.log
tail -n 80 /home/robot/zhangzhang/d1_stair_nav_phase3_orin_scan_axisfix_20261116/work/cloud_scan.log

回到原建图下层平台后：
source /home/robot/zhangzhang/d1_stair_nav_phase3_orin_scan_axisfix_20261116/scripts/env_live.sh
ros2 topic echo /d1/localization/status --once

只有 tracking_valid=true、ready=true 且图像场景和地图位置核对正确后，
再由操作者将狗停到楼梯前的平整预备位置、对正方向，
然后执行：
python3 scripts/capture_entry.py --confirm-lower-entry

该命令会保存当前地图坐标；不要在错误场景、未定位或尚未到预备点时执行。
地图与扫描在 RViz 中核对、人工确认之前，不要运行 validate_site.py。
