# Orin 使用说明（UTF-8 / Linux 版）

适用：NVIDIA Orin，ARM64（aarch64），Ubuntu 22.04，ROS 2 Humble。
版本目录：`/home/robot/zhangzhang/d1_stair_nav_phase3_orin_scan_axisfix_20261116`。

本版恢复原始雷达轴顺序，并修正 FAST-LIO Y-up 到导航 Z-up 的扫描投影；地图、标定、视觉库、楼梯预备点和导航/上楼控制参数不变。网页中文及被乱码破坏的引号、标签已恢复。文本统一 UTF-8（无 BOM）和 LF 换行；启动脚本使用本版本自己的 env_live.sh。归档不带旧 build/install/log/work 数据或 Python 缓存，首次使用需要编译。

## 解压与编译

将 tar.gz 包上传至 `/home/robot/zhangzhang` 后，在 Orin 执行：

```bash
cd /home/robot/zhangzhang
tar -xzf d1_stair_nav_phase3_orin_scan_axisfix_20261116.tar.gz
cd d1_stair_nav_phase3_orin_scan_axisfix_20261116
bash scripts/check_dependencies.sh
bash scripts/build.sh
source install/setup.bash
```

目录名可以改变，脚本会按自己的实际位置加载环境。不要把旧版本的 build、install、log 或 work 复制进来。原有 FAST-LIO 工作区 `/home/robot/zhangzhang/fastlio_live` 仍是外部依赖；本包不会复制或改写该工作区。

## 启动定位、雷达和网页

先结束旧版本的定位、雷达和网页进程；在机器狗停稳且导航/上楼任务已经结束时切换版本。随后在新目录执行：

```bash
source scripts/env_live.sh
python3 scripts/sensor_stack.py start
python3 scripts/sensor_stack.py status
bash scripts/run_web.sh --host 0.0.0.0 --port 8767
```

网页在前台运行，保持该终端开启。网页仅观察，不发送运动指令。ROS 的 NumPy/OpenCV 需要使用系统依赖；env_live.sh 已设置 PYTHONNOUSERSITE=1，避免用户目录中的不兼容 NumPy 覆盖 ROS 依赖。

## 电脑访问网页

在电脑 PowerShell 另开窗口，通过 RK3588 转发网页：

```powershell
ssh -N -L 127.0.0.1:18767:192.168.168.100:8767 -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 robot@10.196.164.98
```

完成 RK3588 登录并保持窗口开启。浏览器访问 `http://127.0.0.1:18767/`。若要另外登录 Orin，可使用：

```powershell
ssh -J robot@10.196.164.98 robot@192.168.168.100
```

## 停止

网页所在终端按 Ctrl+C。然后在 Orin 的新版本目录执行：

```bash
python3 scripts/sensor_stack.py stop
python3 scripts/sensor_stack.py status
```

三项均应为 STOPPED。电脑上的隧道窗口也按 Ctrl+C。

## 软件检查

```bash
source scripts/env_live.sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -p 'test_*.py' -v
```

界面和软件测试不能代替现场检查雷达方向、地图对齐与导航效果。详细定位/导航/上楼操作仍见 `docs/阶段三_网页与融合定位操作说明.md`。
