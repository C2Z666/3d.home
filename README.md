# D1 定位与点到点导航

本分支以 Orin scan axisfix 版本为基线，运行流程为传感器 → 融合定位 → Nav2 → 到点停稳结束。目标坐标通过命令行传入，单位为米，yaw 为弧度。地图、标定、场景配置、视觉库与建图数据均由 Git 管理。

在项目根目录执行以下步骤。网页、Nav2、SDK 桥接和导航任务分别占用一个终端。

```bash
cd /home/robot/zhangzhang/d1_stair_nav_phase3_orin_scan_axisfix_20261116
source scripts/env_live.sh
bash scripts/build.sh
python3 scripts/sensor_stack.py start
python3 scripts/sensor_stack.py status
bash scripts/run_web.sh --host 0.0.0.0 --port 8767
```

网页显示定位、扫描、路径及原配置中的参考点。实际目标使用下面的 `--x --y --yaw` 参数。初次导航前，完成所选场景地图与扫描的现场核对，记录已有的检查结果。

```bash
python3 scripts/validate_site.py --confirm-map-and-scan
bash scripts/run_nav2.sh scene:=old floor:=lower
```

在另一终端连接 SDK。将 ROBOT_IP 替换为机器人控制器的实际地址。

```bash
bash scripts/run_nav_bridge.sh ROBOT_IP
```

在另一终端发送目标。下面坐标仅演示参数格式，运行时填写当前地图内已核对的目标。`--z` 是当前平面上的机体 map 高度，用于定位高度检查。

```bash
bash scripts/run_mission.sh --live --x 1.0 --y 0.0 --yaw 0.0 --z 0.0
```

任务在定位和扫描有效、SDK 控制可用且 Walk 姿态确认后发送 Nav2 目标。到达目标并确认停稳后输出 `DONE`，退出码为 0。定位丢失、扫描过期、控制权变化或中断时撤销目标并发送零速度。前进速度上限为 0.30 m/s，角速度上限为 0.25 rad/s。

选择其他场景时，传感器进程的 `D1_SCENE`、Nav2 的 `scene:=...` 和任务的 `--scene ...` 必须一致。切换场景后重新核对地图与扫描。定位和导航都在同一楼层平面内工作。

```bash
python3 scripts/run_stage.py --scene front_A nav2 floor:=lower
python3 scripts/run_stage.py --scene front_A mission --live --x 1 --y 0 --yaw 0 --z 0
```

软件验证

```bash
bash scripts/test_software.sh
source scripts/env_live.sh
mkdir -p work/nav_simple_test
python3 tests/ros_nav2_mission.py work/nav_simple_test
```

Nav2 集成测试使用独立 ROS 域和模拟状态。历史操作文档保留作基线记录，本分支启动与目标导航以本文件为准。
