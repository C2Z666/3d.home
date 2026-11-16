# 后续导航文件说明

这份导出使用完整原始 bag `d1_stairs_3d_20261110_150243` 和项目的 `script/01_mapping.sh` 重新生成。重建处理了 **961 帧雷达扫描**，得到 **97 个优化关键帧、75,861 个地图点、84 个视觉关键帧及 24,458 个带深度的视觉特征**。重建前后的关键帧 ID、时间戳和位置一致。两张平台栅格从重建数据库重新导出，图像及 YAML 与原平台文件逐字节一致。

| 文件 | 用途 |
| --- | --- |
| [output/demo/visual_map.npz](output/demo/visual_map.npz) | SIFT 视觉重定位库，含图像特征和 `map` 坐标系下的三维点 |
| [config/calibration.yaml](config/calibration.yaml) | 同次建库的相机内参、畸变和外参。与视觉库及建图记录中的 SHA-256 一致 |
| [output/demo/map.db](output/demo/map.db)、[map.ply](output/demo/map.ply)、[mapping.json](output/demo/mapping.json) | 正式 RTAB-Map 雷达数据库、同一坐标系中的点云以及建图记录。数据库已通过 SQLite 完整性检查 |
| [下平台 YAML](output/platform_maps/lower.yaml)、[PGM](output/platform_maps/lower.pgm) | 第一次上楼前的下平台，地面高度约 `z=-0.47 m` |
| [上平台 YAML](output/platform_maps/upper.yaml)、[PGM](output/platform_maps/upper.pgm) | 第一次上楼后的上平台，地面高度约 `z=1.80 m` |
| [平台参数](output/platform_maps/platforms.yaml)、[栅格预览](output/platform_maps/preview.png) | 两张图都用 `map` 的 XY 坐标，分辨率 0.05 m。地面高度单独记录，地图 YAML 的 `origin` 第三个值是偏航角 |
| [fastlio/jt16.yaml](fastlio/jt16.yaml)、[replay.yaml](fastlio/replay.yaml)、[源码版本](fastlio/源码版本.txt) | 上游 JT16 模板、本次项目留存的 FAST-LIO 回放参数和固定源码版本 |
| [SHA256SUMS](SHA256SUMS) | 在本目录执行 `sha256sum -c SHA256SUMS` 可校验文件 |

ROS 2 Humble 的 Nav2 Map Server 已成功加载上下平台两张图。视觉库从同一个 bag 中另取 16 张图做加载与几何检查，12 张通过。这个结果只用于确认文件能正常读取和定位算法能运行。新增“仅图片”数据此前实测为 0/51，跨次采集的定位可靠性仍待验证。

## FAST-LIO 接口

- 留存回放配置订阅 `/front_lidar` 和 `/front_lidar/imu`，使用 JT16（`lidar_type: 1`）、16 线、逐点时间单位秒（`timestamp_unit: 0`）、陀螺仪单位 rad/s。回放按 bag 接收时间调度，并保留消息自身的 `header.stamp`。
- 固定版源码发布 `/Odometry`，`header.frame_id=camera_init`，`child_frame_id=body`。该版输出位姿参考点是 **IMU**。回放脚本用外参换算成机身 `base_link`，再对齐到三维 `map`。
- `ROS_DOMAIN_ID=95`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`ROS_LOCALHOST_ONLY=1` 来自本项目的离线回放脚本。机载运行环境的 Domain ID、RMW、DDS 发现范围、传感器时间基准及实机的消息 frame ID 需要现场确认。回放配置可用于复现，实机参数须按驱动输出核对。

## 导航还缺什么

- **在线定位启动和 TF**：需要把实时图像重定位、FAST-LIO 里程计和地图校正接成在线节点，持续发布 `odom → base_link`，由全局定位维护 `map → odom`，并提供传感器静态 TF、首次定位和失效状态。当前绿色校正轨迹由离线脚本生成。FAST-LIO 实机输出频率、延迟和断档仍需验证。
- **可上机的 Nav2 配置**：需要目标机器人的 launch、控制器/规划器和全局/局部 costmap 参数、实际机身 footprint、实时障碍输入，以及 `cmd_vel` 到底层的适配。两张已导出的栅格是平台局部草图，尚需现场查漏、核对平台边缘和未知区。
- **楼梯交接数据与控制**：需要标定入口预备点的 `x、y、yaw`、楼梯口禁行区、上平台整机到达确认区，以及地图切换、步态切换、控制权交接和异常处理配置。
- **独立验证数据**：需要另一次带同步雷达、IMU 和 RGB 的采集，检查跨光照重定位和回放断档。原始 bag 留在项目数据目录，可用于重建，不需要跟着地图包复制到导航机器。

## 需求截图

![导航导出需求](需求截图.png)
