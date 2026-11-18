#!/bin/bash
# clean_nav_env.sh —— 清理导航环境里的重复/残留进程
#
# 用法：
#   bash scripts/clean_nav_env.sh check    只报告，不动手（默认）
#   bash scripts/clean_nav_env.sh clean    真的杀
#
# 保留规则：命令行里带本包目录（$ROOT）的进程一律保留；
#           其余路径的定位/扫描、Nav2 残留、重复桥全部清掉。

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MODE="${1:-check}"

red(){ printf '\033[31m%s\033[0m\n' "$*"; }
grn(){ printf '\033[32m%s\033[0m\n' "$*"; }
ylw(){ printf '\033[33m%s\033[0m\n' "$*"; }

declare -A seen
pids=()
reasons=()

add(){
  local p="$1" why="$2"
  if [ -n "${seen[$p]:-}" ]; then return 0; fi
  seen[$p]=1
  pids+=("$p")
  reasons+=("$why")
}

echo "本包目录: $ROOT"
echo "模式: $MODE"
echo

# ---------- 1) 非本包路径的进程 ----------
for p in $(pgrep -f "/home/robot/lmy/" 2>/dev/null); do
  add "$p" "非本包路径 /home/robot/lmy"
done

# ---------- 2) 重复的定位 / 扫描 / rtabmap ----------
for p in $(pgrep -f "localization_live\.py|cloud_scan\.py|rtabmap" 2>/dev/null); do
  cmd="$(ps -o cmd= -p "$p" 2>/dev/null)"
  if [ -z "$cmd" ]; then continue; fi
  case "$cmd" in
    *"$ROOT"*) : ;;
    *) add "$p" "重复的定位/扫描进程" ;;
  esac
done

# ---------- 3) Nav2 残留 ----------
# 注意：run_nav2.sh 最后 exec 的是 run_stage.py，所以三种形态都要认
nav2_alive=0
if pgrep -f "run_nav2\.sh" >/dev/null 2>&1; then nav2_alive=1; fi
if pgrep -f "run_stage\.py.*nav2" >/dev/null 2>&1; then nav2_alive=1; fi
if pgrep -f "nav2_bringup\.launch\.py" >/dev/null 2>&1; then nav2_alive=1; fi

if [ "$nav2_alive" -eq 1 ]; then
  grn "检测到 Nav2 正在运行，跳过 Nav2 残留清理。"
else
  for p in $(pgrep -f "nav2_map_server/map_server" 2>/dev/null); do
    add "$p" "Nav2 残留 map_server"
  done
  for p in $(pgrep -f "nav2_lifecycle_manager/lifecycle_manager" 2>/dev/null); do
    add "$p" "Nav2 残留 lifecycle_manager"
  done
fi

# ---------- 4) 重复的 d1_bridge ----------
bridges=($(pgrep -x d1_bridge 2>/dev/null || true))
if [ "${#bridges[@]}" -gt 1 ]; then
  for p in "${bridges[@]:1}"; do
    add "$p" "重复的 d1_bridge"
  done
fi

# ---------- 报告 ----------
if [ "${#pids[@]}" -eq 0 ]; then
  grn "进程层没有需要清理的东西。"
else
  ylw "发现 ${#pids[@]} 个待清理进程："
  i=0
  while [ "$i" -lt "${#pids[@]}" ]; do
    p="${pids[$i]}"
    printf '  PID %-7s %-30s %s\n' "$p" "${reasons[$i]}" "$(ps -o cmd= -p "$p" 2>/dev/null | cut -c1-75)"
    i=$((i+1))
  done
fi

# ---------- 执行 ----------
if [ "$MODE" = "clean" ] && [ "${#pids[@]}" -gt 0 ]; then
  echo
  ylw "发送 SIGTERM ..."
  for p in "${pids[@]}"; do kill -TERM "$p" 2>/dev/null; done
  sleep 3
  for p in "${pids[@]}"; do
    if kill -0 "$p" 2>/dev/null; then
      red "  $p 未退出，改 SIGKILL"
      kill -KILL "$p" 2>/dev/null
    fi
  done
  sleep 1
fi

# ---------- 验证 ----------
echo
echo "================ 数据源检查 ================"
source "$ROOT/scripts/env_live.sh" >/dev/null 2>&1 || true

bad=0

check_topic(){
  local topic="$1" want="$2" out cnt
  out="$(timeout 10 ros2 topic info "$topic" 2>&1)"
  cnt="$(printf '%s\n' "$out" | awk '/Publisher count/ {print $3; exit}')"
  if [ -z "$cnt" ]; then
    red "  $topic : 读不到发布者数量"
    printf '%s\n' "$out" | head -3 | sed 's/^/    /'
    return 1
  fi
  if [ "$cnt" -eq "$want" ]; then
    grn "  $topic : 发布者 $cnt ✅"
    return 0
  fi
  red "  $topic : 发布者 $cnt（应为 $want）❌"
  echo "    发布它的节点："
  timeout 10 ros2 topic info "$topic" --verbose 2>&1 \
    | grep -E "Node name|Node namespace" | sed 's/^/    /' | head -12
  return 1
}

check_topic /d1/localization/status 1 || bad=1
check_topic /d1/nav/scan 1 || bad=1
check_topic /d1/localization/pose6d 1 || bad=1

echo
echo "================ 进程残留检查 ================"
if pgrep -f "/home/robot/lmy/" >/dev/null 2>&1; then
  red "  仍有 /home/robot/lmy 的进程："
  ps -eo pid,cmd | grep "/home/robot/lmy/" | grep -v grep | cut -c1-95 | sed 's/^/    /'
  bad=1
else
  grn "  无 lmy 残留 ✅"
fi

echo
echo "================ 桥 / Nav2 ================"
if pgrep -x d1_bridge >/dev/null 2>&1; then
  grn "  d1_bridge 在跑 ✅"
else
  ylw "  d1_bridge 没在跑（要用就自己起）"
fi
if timeout 10 ros2 action list 2>/dev/null | grep -q navigate_to_pose; then
  grn "  /navigate_to_pose 在 ✅"
else
  ylw "  /navigate_to_pose 不在（Nav2 没起）"
fi

echo
if [ "$bad" -eq 0 ]; then
  grn "结论：数据源唯一，可以继续。"
  exit 0
fi
red "结论：仍有冲突。"
echo "  - 进程已经没了但发布者还是 2 → ROS 图缓存，执行："
echo "      ros2 daemon stop && sleep 2 && ros2 daemon start"
echo "  - 节点还在 → 用上面的节点名配合下面的命令找 PID："
echo "      ros2 node info /<节点名>   # 看它是不是你认识的那个"
echo "      ps -eo pid,cmd | grep <关键词>"
exit 1
