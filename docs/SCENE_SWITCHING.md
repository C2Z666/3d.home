# 新旧场景切换

本版本默认保持旧场景行为。新增场景为 `front_A`，对应 `front_A_20261113_205139` 的视觉库、二维 lower/upper 地图和同一份相机标定。

## 启动定位

旧场景：

```bash
cd /home/robot/zhangzhang/d1_stair_nav_phase3_orin_scan_axisfix_20261116
D1_SCENE=old bash scripts/run_localization.sh 2>&1 | tee work/localization_old.log
```

新场景：

```bash
cd /home/robot/zhangzhang/d1_stair_nav_phase3_orin_scan_axisfix_20261116
D1_SCENE=front_A bash scripts/run_localization.sh 2>&1 | tee work/localization_front_A.log
```

## 启动网页

旧场景网页：

```bash
D1_SCENE=old bash scripts/run_web.sh --host 0.0.0.0 --port 8767
```

新场景网页：

```bash
D1_SCENE=front_A bash scripts/run_web.sh --host 0.0.0.0 --port 8767
```

网页左上角的“场景”下拉框可以切换旧场景和新场景的地图、参考图与场景身份。若网页选择的场景和正在运行的定位进程不同，页面会提示，需要先停止定位进程，再用对应的 `D1_SCENE` 重新启动。

新场景当前没有写入未经现场核验的楼梯入口点，因此它可以用于视觉初定位、地图查看和数据验证；旧场景的入口与既有导航功能保持不变。
