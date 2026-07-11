#!/usr/bin/env python3
import argparse
import json
import math
import os
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp_occupied(value):
    if value < 0:
        return 205
    if value >= 65:
        return 0
    if value <= 25:
        return 254
    return 205


def save_pgm(map_msg, pgm_path):
    width = map_msg.info.width
    height = map_msg.info.height
    data = map_msg.data

    with pgm_path.open('wb') as f:
        f.write(b'P5\n')
        f.write(f'# CREATOR: go2 save_map_and_pose.py {map_msg.info.resolution:.6f} m/pix\n'.encode())
        f.write(f'{width} {height}\n255\n'.encode())

        # ROS occupancy grids start at the bottom-left. PGM files are written
        # top row first, matching nav2_map_server/map_saver behavior.
        for y in range(height - 1, -1, -1):
            row_start = y * width
            row = bytearray(clamp_occupied(v) for v in data[row_start:row_start + width])
            f.write(row)


def save_map_yaml(map_msg, yaml_path, image_name):
    origin = map_msg.info.origin
    yaw = yaw_from_quaternion(origin.orientation)

    text = (
        f'image: {image_name}\n'
        'mode: trinary\n'
        f'resolution: {map_msg.info.resolution:.10g}\n'
        f'origin: [{origin.position.x:.10g}, {origin.position.y:.10g}, {yaw:.10g}]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n'
    )
    yaml_path.write_text(text)


def pose_dict_from_transform(transform):
    t = transform.transform.translation
    q = transform.transform.rotation
    return {
        'frame_id': transform.header.frame_id,
        'child_frame_id': transform.child_frame_id,
        'position': {
            'x': float(t.x),
            'y': float(t.y),
            'z': 0.0,
        },
        'orientation': {
            'x': 0.0,
            'y': 0.0,
            'z': float(q.z),
            'w': float(q.w),
        },
        'covariance': {
            'x': 0.25,
            'y': 0.25,
            'yaw': 0.0685,
        },
    }


def save_pose_yaml(pose, pose_path):
    text = (
        f'frame_id: {pose["frame_id"]}\n'
        f'child_frame_id: {pose["child_frame_id"]}\n'
        'position:\n'
        f'  x: {pose["position"]["x"]:.10g}\n'
        f'  y: {pose["position"]["y"]:.10g}\n'
        f'  z: {pose["position"]["z"]:.10g}\n'
        'orientation:\n'
        f'  x: {pose["orientation"]["x"]:.10g}\n'
        f'  y: {pose["orientation"]["y"]:.10g}\n'
        f'  z: {pose["orientation"]["z"]:.10g}\n'
        f'  w: {pose["orientation"]["w"]:.10g}\n'
        'covariance:\n'
        f'  x: {pose["covariance"]["x"]:.10g}\n'
        f'  y: {pose["covariance"]["y"]:.10g}\n'
        f'  yaw: {pose["covariance"]["yaw"]:.10g}\n'
    )
    pose_path.write_text(text)


def initialpose_command(pose):
    return f'''ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  frame_id: {pose["frame_id"]}
pose:
  pose:
    position:
      x: {pose["position"]["x"]:.10g}
      y: {pose["position"]["y"]:.10g}
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: {pose["orientation"]["z"]:.10g}
      w: {pose["orientation"]["w"]:.10g}
  covariance:
  - {pose["covariance"]["x"]:.10g}
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - {pose["covariance"]["y"]:.10g}
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - {pose["covariance"]["yaw"]:.10g}
"'''


def save_initialpose_script(pose, script_path, pose_path):
    text = f'''#!/usr/bin/env bash
set -e

if [ -f /opt/ros/foxy/setup.bash ]; then
  source /opt/ros/foxy/setup.bash
fi

if [ -f "$HOME/go2_ros2_ws/install/setup.bash" ]; then
  source "$HOME/go2_ros2_ws/install/setup.bash"
fi

if command -v ros2 >/dev/null 2>&1 && ros2 pkg executables go2_navigation 2>/dev/null | grep -q publish_initial_pose.py; then
  ros2 run go2_navigation publish_initial_pose.py --pose-file "{pose_path}" --repeat 5 --period 0.5
else
  {initialpose_command(pose)}
fi
'''
    script_path.write_text(text)
    script_path.chmod(0o755)


class MapPoseSaver:
    def __init__(self, args):
        self.args = args
        self.map_msg = None
        self.node = rclpy.create_node('go2_save_map_and_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.node.create_subscription(OccupancyGrid, args.map_topic, self.map_callback, volatile_qos)
        self.node.create_subscription(OccupancyGrid, args.map_topic, self.map_callback, transient_qos)

    def map_callback(self, msg):
        self.map_msg = msg

    def wait_for_map(self):
        deadline = self.node.get_clock().now() + Duration(seconds=self.args.timeout)
        while rclpy.ok() and self.node.get_clock().now() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.map_msg is not None:
                return self.map_msg
        raise RuntimeError(f'timed out waiting for map topic {self.args.map_topic}')

    def wait_for_transform(self):
        deadline = self.node.get_clock().now() + Duration(seconds=self.args.timeout)
        last_error = None
        while rclpy.ok() and self.node.get_clock().now() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                if self.tf_buffer.can_transform(
                    self.args.map_frame,
                    self.args.base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                ):
                    return self.tf_buffer.lookup_transform(
                        self.args.map_frame,
                        self.args.base_frame,
                        rclpy.time.Time(),
                    )
            except TransformException as exc:
                last_error = exc
        detail = f': {last_error}' if last_error else ''
        raise RuntimeError(
            f'timed out waiting for TF {self.args.map_frame} -> {self.args.base_frame}{detail}'
        )

    def destroy(self):
        self.node.destroy_node()


def parse_args():
    parser = argparse.ArgumentParser(
        description='Save the current /map and map->base_link pose for fixed-start AMCL navigation.'
    )
    default_prefix = os.path.join(os.path.expanduser('~'), 'go2_maps', 'go2_map')
    parser.add_argument('--map-prefix', default=default_prefix, help='output map prefix, without extension')
    parser.add_argument('--pose-file', default=None, help='output initial pose YAML path')
    parser.add_argument('--initialpose-script', default=None, help='output helper shell script path')
    parser.add_argument('--map-topic', default='/map')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--timeout', type=float, default=15.0)
    return parser.parse_args()


def main():
    args = parse_args()
    map_prefix = Path(os.path.expanduser(args.map_prefix))
    map_prefix.parent.mkdir(parents=True, exist_ok=True)
    yaml_path = map_prefix.with_suffix('.yaml')
    pgm_path = map_prefix.with_suffix('.pgm')
    pose_path = Path(os.path.expanduser(args.pose_file)) if args.pose_file else map_prefix.parent / 'go2_initial_pose.yaml'
    json_path = pose_path.with_suffix('.json')
    script_path = (
        Path(os.path.expanduser(args.initialpose_script))
        if args.initialpose_script
        else map_prefix.parent / 'set_go2_initial_pose.sh'
    )

    rclpy.init()
    saver = MapPoseSaver(args)
    try:
        map_msg = saver.wait_for_map()
        transform = saver.wait_for_transform()
        pose = pose_dict_from_transform(transform)

        save_pgm(map_msg, pgm_path)
        save_map_yaml(map_msg, yaml_path, pgm_path.name)
        save_pose_yaml(pose, pose_path)
        json_path.write_text(json.dumps(pose, indent=2))
        save_initialpose_script(pose, script_path, pose_path)

        print('保存完成:')
        print(f'  地图 YAML: {yaml_path}')
        print(f'  地图 PGM : {pgm_path}')
        print(f'  起点 YAML: {pose_path}')
        print(f'  起点 JSON: {json_path}')
        print(f'  发布脚本 : {script_path}')
        print()
        print('记录到的固定起点:')
        print(f'  x = {pose["position"]["x"]:.6f}')
        print(f'  y = {pose["position"]["y"]:.6f}')
        print(f'  z = {pose["orientation"]["z"]:.6f}')
        print(f'  w = {pose["orientation"]["w"]:.6f}')
    finally:
        saver.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
