# 测试方法

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
