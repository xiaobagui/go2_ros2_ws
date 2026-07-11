#!/usr/bin/env bash
set -eo pipefail

MAP_YAML="${1:-$HOME/go2_maps/go2_map.yaml}"
POSE_FILE="${POSE_FILE:-$HOME/go2_maps/go2_initial_pose.yaml}"
USE_RVIZ="${USE_RVIZ:-true}"
VIDEO_ENABLE="${VIDEO_ENABLE:-false}"

source_if_exists() {
  local setup_file="$1"
  if [ -f "$setup_file" ]; then
    # ROS 2 Foxy setup scripts may read unset variables such as
    # AMENT_TRACE_SETUP_FILES, so keep nounset disabled while sourcing them.
    set +u
    source "$setup_file"
    set -u
  fi
}

source_if_exists /opt/ros/foxy/setup.bash
source_if_exists "$HOME/go2_ros2_ws/install/setup.bash"

if [ ! -f "$MAP_YAML" ]; then
  echo "找不到地图文件: $MAP_YAML"
  exit 1
fi

if [ ! -f "$POSE_FILE" ]; then
  echo "找不到固定起点文件: $POSE_FILE"
  echo "请先运行 go2_slam_record_session.sh 并输入 save。"
  exit 1
fi

cleanup() {
  if [ -n "${NAV_PID:-}" ] && kill -0 "$NAV_PID" >/dev/null 2>&1; then
    echo
    echo "正在停止导航 launch..."
    kill "$NAV_PID" >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT

echo "启动已有地图定位导航:"
echo "  map : $MAP_YAML"
echo "  pose: $POSE_FILE"

ros2 launch go2_navigation go2_localization_nav.launch.py \
  map:="$MAP_YAML" \
  auto_localize:=false \
  use_rviz:="$USE_RVIZ" \
  video_enable:="$VIDEO_ENABLE" &
NAV_PID="$!"

echo "等待 AMCL 激活..."
for _ in $(seq 1 60); do
  if ros2 lifecycle get /amcl 2>/dev/null | grep -q "active"; then
    break
  fi
  sleep 1
done

if ! ros2 lifecycle get /amcl 2>/dev/null | grep -q "active"; then
  echo "AMCL 没有在 60 秒内 active，请检查 launch 终端日志。"
  wait "$NAV_PID"
  exit 1
fi

echo "AMCL 已激活，发布固定初始位姿..."
ros2 run go2_navigation publish_initial_pose.py --pose-file "$POSE_FILE" --repeat 5 --period 0.5

echo
echo "已发布固定起点。请在 RViz 中确认 /scan 和地图重合，然后使用 Nav2 Goal 点目标。"
echo "按 Ctrl+C 结束导航。"

wait "$NAV_PID"
