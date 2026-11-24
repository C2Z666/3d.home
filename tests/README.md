# 测试方法

批次 B：在 Linux / Humble、已编译代价地图插件的独立测试副本中运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
mkdir -p work
python3 -m unittest discover -s tests
D1_TEST_DOMAIN_ID=197 D1_TEST_CASES=local_bypass,local_tight,blocked_corridor,approved_path,curved_path,tf_transient,motion_retry,approved_turn,turn_obstacle,tf_transient_turn \
  python3 tests/ros_nav2_mission.py work/batch_b_validation
```

该 ROS 测试创建单独的 fixture 地图及参数；默认使用
`tests/fastdds_localhost.xml`，没有 SDKClient 或真实机器人 socket。
`local_bypass` 验证转弯参考路线可由 DWB 连续通过，不允许停车或重规划；
`local_tight` 保留直线近障碍场景，验证当前局部/转向约束无法通过时只搜索一次，
随后安全等待至时限，报告明确标注该限制，不将它计为到达成功。
`blocked_corridor` 验证整条通路被堵时的同样兜底。
每个场景记录是否在 NAVIGATE 阶段请求全局认证以及局部检查耗时。

纯逻辑与几何：

    /usr/bin/python3 -B -m unittest discover -s tests -p 'test_core.py' -v

真实 ROS 进程 + 假 SDK（仅 Linux Humble）。假 SDK 完整替换 SDKClient 方法，不创建机器人网络 socket；测试域 179，localhost only。

先构建桥接。默认 tests/ros_fake_stair.py 的第一个参数是测试输出目录，其父目录需要有 install/：

    source /opt/ros/humble/setup.bash
    colcon --log-base work/log build --build-base work/build --install-base work/install --packages-select d1_host_control
    source work/install/setup.bash
    /usr/bin/python3 tests/ros_fake_stair.py work/fake_stair

测试生成可删除的日志和假 SDK .so 到指定 work 目录，不需要真实机器狗。测试断言：无自动站立/清软急停、y 前零非零指令、普通 cmd_vel 在 Stair 不生效、y 后有前进、时限结束零速、输入关闭取消、控制权丢失结束。

上游 src/d1_host_control/test 中旧脚本依赖原工作区路径和启动行为，保留作来源资料；本交付的回归入口是 tests/ros_fake_stair.py，不直接运行旧脚本声称其适配本版。

在线视觉 ROS 合成测试：

    source /opt/ros/humble/setup.bash
    /usr/bin/python3 tests/ros_synthetic_localization.py work/synthetic_localization

它用生成的图像、已知平面三维特征和零畸变测试标定验证 PnP、在线位姿输出、视觉超时撤销 ready、里程计复位撤销 ready。合成场景不代表真实楼梯的识别率或定位精度。
