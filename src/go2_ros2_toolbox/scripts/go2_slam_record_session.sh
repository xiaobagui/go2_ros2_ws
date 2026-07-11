#!/usr/bin/env bash
set -eo pipefail

MAP_PREFIX="${1:-$HOME/go2_maps/go2_map}"
MAP_DIR="$(dirname "$MAP_PREFIX")"
USE_RVIZ="${USE_RVIZ:-auto}"
VIDEO_ENABLE="${VIDEO_ENABLE:-false}"

mkdir -p "$MAP_DIR"

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

if ! command -v ros2 >/dev/null 2>&1; then
  echo "没有找到 ros2 命令，请先 source ROS 2 Foxy 和工作空间。"
  exit 1
fi

PIDS=()

start_bg() {
  local name="$1"
  shift
  echo "启动 $name ..."
  "$@" &
  PIDS+=("$!")
  sleep 1
}

cleanup() {
  echo
  echo "正在停止本脚本启动的进程..."
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
}

trap cleanup EXIT

start_bg "go2_base" ros2 launch go2_core go2_base.launch.py video_enable:="$VIDEO_ENABLE"
start_bg "go2_pointcloud_process" ros2 launch go2_perception go2_pointcloud_process.launch.py
start_bg "slam_toolbox" ros2 launch go2_slam go2_slamtoolbox.launch.py

if [ "$USE_RVIZ" = "true" ] || { [ "$USE_RVIZ" = "auto" ] && [ -n "${DISPLAY:-}" ]; }; then
  RVIZ_CONFIG="$(ros2 pkg prefix go2_core)/share/go2_core/config/default.rviz"
  if [ -f "$RVIZ_CONFIG" ]; then
    start_bg "rviz2" rviz2 -d "$RVIZ_CONFIG"
  else
    start_bg "rviz2" rviz2
  fi
fi

cat <<EOF

SLAM 建图会话已启动。

请现在遥控机器狗完成建图。建图结束后，把机器狗停在“下次已有地图导航要复用的固定起点”，并保持狗头朝向一致。

可输入:
  save  保存地图和当前 map->base_link 位姿
  quit  退出，不保存

当前输出前缀:
  $MAP_PREFIX

EOF

while true; do
  read -r -p "go2-slam> " cmd
  case "$cmd" in
    save)
      echo "保存前请确认机器狗已经停稳，SLAM 地图没有明显跳动。"
      ros2 run go2_navigation save_map_and_pose.py \
        --map-prefix "$MAP_PREFIX" \
        --pose-file "$MAP_DIR/go2_initial_pose.yaml" \
        --initialpose-script "$MAP_DIR/set_go2_initial_pose.sh"
      echo
      echo "保存完成。下次已有地图导航前，把狗放回这个位置和朝向，然后运行:"
      echo "  $MAP_DIR/set_go2_initial_pose.sh"
      break
      ;;
    quit|exit)
      echo "退出，不保存。"
      break
      ;;
    "")
      ;;
    *)
      echo "未知指令: $cmd"
      echo "请输入 save 或 quit"
      ;;
  esac
done
