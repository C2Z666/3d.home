# D1 定位与点到点导航

## 楼层与 Stair 任务（czc_nav_mutil）

楼道大厅楼层连接配置为 `config/floor_connections.yaml`。`floor1` 对应
`lower` 地图，`floor2` 对应 `upper` 平台地图。查看地图、机器人实际楼层和
任务目标楼层分别显示；在其他楼层看到的是明确标注的虚拟 XY 投影。
查看楼层不会修改定位、地图执行状态或触发步态。未知实际楼层时禁止普通导航。

星标预备点为 `(-1.582, 13.577, yaw=1.7518 rad)`。点击星标后“规划预览”
只导航到该点；“规划经此楼梯上楼”才建立跨层任务，仍需点击“导航”确认。
也可在 floor2 选最终目标，先预览到入口，到层后按实际位置计算后续平地路线。
未标定出口时显示连接关系而不虚构出口坐标或后续可执行路径。

Stair 使用独立许可/速度通道，与平地速度互斥。默认直行巡航 0.4 m/s，
水平平台候选阶段 0.15 m/s，不转向。上述为待现场验证的配置值。
旧 `pck` 的 SDK 模式确认和许可看门狗已接入当前桥接，未替换现有平地控制。
停止按钮取消整个跨层任务，超时和异常不会被当成到达，不自动倒退或趴下。

`exit: null` 表示未标定，完全不参与结束坐标判定。此时须先观察到爬升和
前向进度，再持续水平、相对高度稳定且有足够平台前进距离，最后发零速复核停稳。
此判据仍需真机验证，不能区分所有中间平台。标定后 `exit` 必须包含
`floor_id: floor2` 和 `x/y/yaw`，可补充机身地图 `z`；默认 `combine: all`
要求平台坐标与姿态条件同时满足。结束坐标只属于相应连接的目标楼层，
不能在 floor1 因 XY 相同而结束。新增楼梯/楼层以独立 ID 添加；没有隐式下楼边。

手动初值：停稳并结束导航，在所查看地图选点/拖动朝向或输入 X/Y/yaw，
填写当前机身地图 Z，打开“手动标定当前位置”并确认。它更新 map 对齐、
递增定位 epoch、废弃旧路线，保留 FAST-LIO 本身。上层高度不明时不得沿用
一层 Z。恢复图像匹配后仍按原有策略渐进视觉纠偏。

软件验证：`python3 -m unittest discover -s tests -p test_multifloor.py -v`；
`node tests/test_navigation_ui.cjs`；隔离 ROS/假 SDK 验证为
`python3 tests/ros_stair_bridge.py`（仅假 SDK、localhost、独立 ROS 域）。
不应使用真实机器人来替代上述软件测试。

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


## 2026-11-18 平地导航与临时障碍修复

- 平地导航不再用地图 Z 相对起点变化或机身倾斜角作为退出条件；原始重力姿态和高度仅写入诊断。定位/TF/传感器时效、SDK 控制权、急停以及真实障碍检查继续生效。上楼任务的爬升与恢复水平平台判据不变。
- 实时障碍使用项目内的 `d1_temporal_costmap::TemporalVoxelLayer`：保留最近 **10 秒**的有效点云观测，并重建这一动态层。持续检测到的障碍不断刷新；连续 10 秒未再次检测到的动态点退出，原位置及其膨胀区一并更新。静态底图不被清空。源时间戳过期、点云断流或 TF 无法更新时冻结动态层并标记不就绪；恢复新鲜数据稳定 0.3 秒后再重建。此机制不依赖机器人停稳，也不识别人或物体类别；离开视野的临时物体可能过期，返回后由激光重新检测。
- 执行前及 FollowPath 过程中约每秒复核剩余路线，按配置的完整矩形机身检查真实障碍和未知格。中心不能进入 253 膨胀阻塞格；机身边缘不把膨胀代价重复计算成第二层实体尺寸。机身尺寸、0.4 m 膨胀半径和 0.3 m/s 平地速度保持原值。
- 路线受阻先零速并取消旧动作，确认动作结束和停稳；障碍消失可恢复，持续阻挡等 1.5 秒后申请新路线。重规划间隔至少 3 秒，每任务最多 4 次，总运行时限不重置。没有合格路线时停车退出。
- 地图预备点仅绘制一个星标；去掉名称尾部重复星号。目标细框仍表示代价栅格，不是机身范围。

验证包含 Python 单元测试、独立 ROS 域的真实 Nav2 集成测试：高度漂移、平地倾斜诊断、人工取消、动态阻挡/重规划；原射线清除测试已补充下述时间记忆测试，静态墙保持不变。这些测试不连接机器人硬件，也不代替现场行走验收。


## 动态记忆与实际转向范围

首次拉取本版本或改变 C++ 动态层后，先执行 `bash scripts/build_costmap_plugin.sh`，再执行 `bash scripts/start.sh`。插件使用现有 Humble 依赖，不改变 SDK bridge；`env_live.sh` 只加载该插件的安装路径。

记忆时长位于 `config/nav2.yaml` 的全局、局部 `obstacle_layer.obstacles.observation_persistence`，两处均为 **10.0 秒**；网页预览规划器继承全局设置。这里结合动态层每次重建实现实际格子过期，单独修改 Nav2 原有 persistence 并不能删除已经写入的格子。时限基于采集时间，地图更新和网页刷新会带来少量显示延迟。静态地图未知格仍然禁止通行。

起步及终点转向按当前矩形机身到目标朝向的最短有向转角检查，同时涵盖实测角速度的 0.3 秒滑行量；不再要求整个外接圆空闲。检测使用格子方形与机身矩形的相交判定，并为离散角度采样保留至多约四分之一格的几何误差余量。终点小幅前进同样检查矩形机身的短距离扫过范围。局部地图占用值 100 等于 Nav2 原始实体障碍代价 254，99 等于原始膨胀阻塞代价 253；报错会明确标明这两种标度。

验证命令：

```bash
source scripts/env_live.sh
python3 -m unittest discover -s tests -p 'test_*.py'
D1_TEMPORAL_TEST=1 D1_TEST_CASES=approved_turn,turn_obstacle,turn_inflation,dynamic_clear python3 tests/ros_nav2_mission.py work/temporal_nav2
```

时间记忆测试覆盖持续观测超过 10 秒仍占用、消失 5 秒仍保留、消失超过 10 秒同时清除障碍和膨胀、重新出现立即标记、传感器断流超过 10 秒仍保留以及恢复后的更新；静态墙在各阶段保持不变。几何测试覆盖完整圆内但实际转角外的障碍、中途碰撞、格子角点、未知区域、地图旋转及滑行量。


A 楼梯 floor1→floor2 的出口已标定为 `x=-3.282, y=18.891, yaw=1.9173 rad`，`exit.floor_id=floor2`。可选的 `preview_exit` 是仅供预览的端点，目前使用同一坐标。正常浏览只显示入口预备点；已规划经过该楼梯时，目标层显示 `preview_exit`（未设置则显示 `exit`）和楼梯虚线连接。预览端点不参与结束判定，实际结束仍使用 `exit`、目标楼层身份及平台姿态条件。

障碍标记点云已在 `nav_base` 中表达，标记源使用同一坐标系，避免额外传感器原点 TF 阻塞局部地图更新；实际清除射线仍使用 `d1_nav_lidar_origin` 的真实原点。

### 矩形机身规划与软离墙代价（2026-11-18）

网页平地预览及已确认路线的运行中重规划使用 scripts/body_planner.py。
搜索状态包含 XY 与朝向（8 方向、约 10cm 格点），移动边与旋转边都检查矩形机身。
连续段、起步转向、转角和终点转向采用同一套扫掠检查；短路径化后重新认证，
再生成 DWB 路径。执行前使用最新全局代价地图复核，运行中继续检测变化。
障碍地图、膨胀半径 0.4m、机身尺寸与速度限制未改变。

硬约束：原始 Nav2 代价 >=253 禁止中心进入；>=254（含未知255）禁止机身覆盖。
软代价：距障碍 d<0.8m 时增加 2*((0.8-d)/0.8)^2 的单位路长代价，
另保留原代价地图的软惩罚，鼓励有空间时居中走；0.8m 不是禁行半径。
窄通道仍按矩形机身通行性判断。规划失败会搜索替代位置和朝向，不重复原 Navfn 路径。
搜索上限8秒，结果含 body_certificate；证书仅对应当时地图、尺寸和理想沿线姿态，
不能替代实际跟踪偏差、新障碍和定位误差的运行时保护。
未到达楼层的静态参考线仍仅供显示，到层后按上述规则重新规划。

回归：tests/test_body_planner.py 包含绕障、独立多边形扫掠验证、70cm窄通道、
终点转向、特殊代价类别和旋转地图。隔离 Nav2 测试不得连接硬件。
