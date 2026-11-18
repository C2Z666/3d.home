#!/usr/bin/env python3
"""跑任务前的位姿/地图自检。

用法：
    source scripts/env_live.sh
    python3 scripts/precheck_pose.py

返回码 0 = 可以跑；非 0 = 不要跑，按提示处理。
不发送任何运动指令。
"""
import json
import math
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def map_bounds():
    """按当前场景的 lower 地图算出覆盖范围（米）。"""
    scenes = json.loads((ROOT / "config/scenes.json").read_text(encoding="utf-8"))
    scene = scenes["scenes"]["old"]
    yaml_rel = scene["maps"]["lower"]["yaml"]
    meta = yaml.safe_load((ROOT / yaml_rel).read_text(encoding="utf-8"))
    res = float(meta["resolution"])
    ox, oy, _ = meta["origin"]
    pgm = (ROOT / yaml_rel).parent / meta["image"]
    with open(pgm, "rb") as f:
        data = f.read()
    head = []
    idx = 0
    while len(head) < 3:
        end = data.index(b"\n", idx)
        line = data[idx:end]
        idx = end + 1
        if line.startswith(b"#"):
            continue
        head.append(line)
    w, h = map(int, head[1].split())
    return ox, ox + w * res, oy, oy + h * res, w, h, res


def main():
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String

    x0, x1, y0, y1, w, h, res = map_bounds()

    rclpy.init()
    node = rclpy.create_node("d1_precheck_pose")
    loc = {}
    pose = [None]
    node.create_subscription(
        String, "/d1/localization/status", lambda m: loc.update(json.loads(m.data)), 1
    )
    node.create_subscription(
        PoseStamped, "/d1/localization/pose6d", lambda m: pose.__setitem__(0, m), 1
    )
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    site = yaml.safe_load((ROOT / "config/site.yaml").read_text(encoding="utf-8"))
    entry = site.get("entry") or {}

    ok = True
    print("=" * 58)
    print("地图   %dx%d px @ %.3f m" % (w, h, res))
    print("覆盖   x: %7.2f ~ %7.2f   y: %7.2f ~ %7.2f" % (x0, x1, y0, y1))
    if entry:
        print(
            "入口   x: %7.2f   y: %7.2f   yaw: %6.1f deg"
            % (entry.get("x", 0), entry.get("y", 0), math.degrees(entry.get("yaw", 0)))
        )
    print("-" * 58)
    print("定位   ready=%s  mode=%s  tracking_valid=%s" % (
        loc.get("ready"), loc.get("mode"), loc.get("tracking_valid")))
    print("       visual_age=%s  reason=%s" % (
        loc.get("visual_age"), loc.get("reason")))

    if loc.get("ready") is not True:
        ok = False
        print("  [FAIL] 定位未 ready —— 先 relocalize")
    if loc.get("mode") != "FUSED":
        ok = False
        print("  [FAIL] mode 不是 FUSED（当前 %s）" % loc.get("mode"))
    va = loc.get("visual_age")
    if not isinstance(va, (int, float)) or va > 5:
        ok = False
        print("  [FAIL] 视觉超过 5 秒没更新 —— 视觉已失效")

    if pose[0] is None:
        ok = False
        print("  [FAIL] 没有收到 /d1/localization/pose6d")
    else:
        p = pose[0].pose.position
        q = pose[0].pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        inside = x0 <= p.x <= x1 and y0 <= p.y <= y1
        print("-" * 58)
        print("位姿   x: %7.3f   y: %7.3f   yaw: %6.1f deg" % (p.x, p.y, math.degrees(yaw)))
        if inside:
            print("  [OK]   位姿在地图范围内")
        else:
            ok = False
            print("  [FAIL] 位姿在地图外！Nav2 一定失败（out of bounds）")
            print("         x 超出 %.2f m   y 超出 %.2f m" % (
                max(0, x0 - p.x, p.x - x1), max(0, y0 - p.y, p.y - y1)))
        if entry:
            d = math.hypot(p.x - entry.get("x", 0), p.y - entry.get("y", 0))
            print("       距入口 %.2f m" % d)

    print("=" * 58)
    if ok:
        print("结论：可以跑任务。")
        print("提醒：跑之前再确认 nav2_ready=True（ros2 action list | grep navigate_to_pose）")
    else:
        print("结论：不要跑任务。先解决上面 [FAIL] 的项。")
        print("典型处理：把狗推回地图范围内 -> 静止 -> ")
        print("  python3 scripts/relocalize.py --confirm-stopped")
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
