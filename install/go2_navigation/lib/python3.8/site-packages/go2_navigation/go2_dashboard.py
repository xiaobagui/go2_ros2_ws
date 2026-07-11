#!/usr/bin/env python3
import argparse
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from go2_navigation.publish_initial_pose import build_msg, parse_pose_file_strict
    from go2_navigation.save_map_and_pose import (
        pose_dict_from_transform,
        save_initialpose_script,
        save_map_yaml,
        save_pgm,
        save_pose_yaml,
    )
except ImportError:
    from publish_initial_pose import build_msg, parse_pose_file_strict
    from save_map_and_pose import (
        pose_dict_from_transform,
        save_initialpose_script,
        save_map_yaml,
        save_pgm,
        save_pose_yaml,
    )


HTML = r'''<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Go2 ROS2 Dashboard</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #eef2f5;
      --panel: #ffffff;
      --text: #17202a;
      --muted: #687383;
      --line: #d9e0e7;
      --ok: #18864b;
      --warn: #b7791f;
      --bad: #c53030;
      --blue: #2563eb;
      --blue-dark: #1d4ed8;
      --ink: #0f172a;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: var(--bg);
      color: var(--text);
    }
    header {
      background: #16202a;
      color: white;
      padding: 14px 18px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
    }
    h1 { font-size: 20px; line-height: 1.2; margin: 0; font-weight: 700; }
    .sub { color: #b8c2cc; font-size: 13px; }
    main {
      max-width: 1180px;
      margin: 0 auto;
      padding: 18px;
      display: grid;
      grid-template-columns: repeat(12, 1fr);
      gap: 12px;
    }
    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 14px;
    }
    .span-4 { grid-column: span 4; }
    .span-6 { grid-column: span 6; }
    .span-8 { grid-column: span 8; }
    .span-12 { grid-column: span 12; }
    h2 {
      margin: 0 0 12px;
      font-size: 15px;
      line-height: 1.2;
      color: var(--ink);
    }
    .big {
      font-size: 28px;
      font-weight: 750;
      line-height: 1.15;
      margin: 2px 0 8px;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px 12px;
    }
    .item {
      display: flex;
      justify-content: space-between;
      gap: 8px;
      border-bottom: 1px solid #edf1f5;
      padding: 6px 0;
      min-height: 34px;
      align-items: center;
    }
    .label { color: var(--muted); font-size: 13px; }
    .value {
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      font-size: 13px;
      text-align: right;
      overflow-wrap: anywhere;
    }
    .pill {
      display: inline-flex;
      align-items: center;
      min-height: 24px;
      padding: 2px 8px;
      border-radius: 999px;
      font-size: 12px;
      font-weight: 700;
      color: white;
      background: var(--muted);
    }
    .ok { background: var(--ok); }
    .warn { background: var(--warn); }
    .bad { background: var(--bad); }
    .idle { background: #64748b; }
    .buttons {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
    }
    button {
      appearance: none;
      border: 1px solid transparent;
      border-radius: 8px;
      padding: 10px 13px;
      font-weight: 700;
      cursor: pointer;
      color: white;
      background: var(--blue);
      min-height: 42px;
    }
    button:hover { background: var(--blue-dark); }
    button.secondary { background: #475569; }
    button.secondary:hover { background: #334155; }
    button.danger { background: #c53030; }
    button.danger:hover { background: #9b2c2c; }
    pre {
      margin: 0;
      white-space: pre-wrap;
      word-break: break-word;
      max-height: 240px;
      overflow: auto;
      background: #0f172a;
      color: #d6e2ff;
      border-radius: 8px;
      padding: 12px;
      font-size: 12px;
      line-height: 1.45;
    }
    @media (max-width: 860px) {
      main { grid-template-columns: 1fr; padding: 12px; }
      .span-4, .span-6, .span-8, .span-12 { grid-column: span 1; }
      .grid { grid-template-columns: 1fr; }
      header { align-items: flex-start; flex-direction: column; }
    }
  </style>
</head>
<body>
  <header>
    <div>
      <h1>Go2 ROS2 Dashboard</h1>
      <div class="sub">状态监控、SLAM 保存地图、固定起点发布</div>
    </div>
    <div id="heartbeat" class="sub">连接中...</div>
  </header>

  <main>
    <section class="span-4">
      <h2>当前状态</h2>
      <div id="mode" class="big">-</div>
      <div id="motion" class="pill idle">-</div>
      <div class="item"><span class="label">linear.x</span><span id="linear" class="value">-</span></div>
      <div class="item"><span class="label">angular.z</span><span id="angular" class="value">-</span></div>
    </section>

    <section class="span-8">
      <h2>操作</h2>
      <div class="buttons">
        <button onclick="postAction('/api/save')">Save Map + Initial Pose</button>
        <button class="secondary" onclick="postAction('/api/publish_initial_pose')">Publish Saved Initial Pose</button>
        <button class="danger" onclick="postAction('/api/stop')">Stop Robot</button>
      </div>
      <div style="height:12px"></div>
      <div class="grid">
        <div class="item"><span class="label">地图输出</span><span id="mapPrefix" class="value">-</span></div>
        <div class="item"><span class="label">起点文件</span><span id="poseFile" class="value">-</span></div>
      </div>
    </section>

    <section class="span-6">
      <h2>SLAM / 感知</h2>
      <div class="grid">
        <div class="item"><span class="label">/map</span><span id="mapStatus" class="value">-</span></div>
        <div class="item"><span class="label">/scan</span><span id="scanStatus" class="value">-</span></div>
        <div class="item"><span class="label">map -> base_link</span><span id="tfStatus" class="value">-</span></div>
        <div class="item"><span class="label">/plan</span><span id="planStatus" class="value">-</span></div>
      </div>
    </section>

    <section class="span-6">
      <h2>节点</h2>
      <div class="grid">
        <div class="item"><span class="label">slam_toolbox</span><span id="nodeSlam" class="value">-</span></div>
        <div class="item"><span class="label">map_server</span><span id="nodeMapServer" class="value">-</span></div>
        <div class="item"><span class="label">amcl</span><span id="nodeAmcl" class="value">-</span></div>
        <div class="item"><span class="label">bt_navigator</span><span id="nodeBt" class="value">-</span></div>
        <div class="item"><span class="label">planner_server</span><span id="nodePlanner" class="value">-</span></div>
        <div class="item"><span class="label">controller_server</span><span id="nodeController" class="value">-</span></div>
      </div>
    </section>

    <section class="span-12">
      <h2>最近消息</h2>
      <pre id="messages">-</pre>
    </section>
  </main>

  <script>
    function fmt(v) {
      if (v === null || v === undefined) return '-';
      if (typeof v === 'number') return v.toFixed(3);
      return String(v);
    }
    function okText(v) { return v ? 'OK' : '-'; }
    function setText(id, text) { document.getElementById(id).textContent = text; }
    async function postAction(path) {
      setText('messages', '执行中: ' + path + '\n' + document.getElementById('messages').textContent);
      try {
        const res = await fetch(path, {method: 'POST'});
        const data = await res.json();
        if (!res.ok || !data.ok) {
          throw new Error(data.error || ('HTTP ' + res.status));
        }
        setText('messages', data.message + '\n\n' + (data.log || ''));
      } catch (err) {
        setText('messages', '操作失败: ' + err.message + '\n\n' + document.getElementById('messages').textContent);
      }
      refresh();
    }
    async function refresh() {
      try {
        const res = await fetch('/api/status');
        const s = await res.json();
        setText('heartbeat', '更新时间: ' + s.time);
        setText('mode', s.mode);
        const motion = document.getElementById('motion');
        motion.textContent = s.motion.label;
        motion.className = 'pill ' + (s.motion.active ? 'ok' : 'idle');
        setText('linear', fmt(s.cmd_vel.linear_x));
        setText('angular', fmt(s.cmd_vel.angular_z));
        setText('mapPrefix', s.config.map_prefix);
        setText('poseFile', s.config.pose_file);
        setText('mapStatus', s.map.received ? `${s.map.width}x${s.map.height}, ${s.map.age_sec.toFixed(1)}s` : '-');
        setText('scanStatus', s.scan.received ? `${s.scan.count} beams, finite ${s.scan.finite_count}, ${s.scan.age_sec.toFixed(1)}s` : '-');
        setText('tfStatus', s.tf.ok ? `x=${fmt(s.tf.x)}, y=${fmt(s.tf.y)}, yaw=${fmt(s.tf.yaw_deg)}°` : s.tf.message);
        setText('planStatus', s.plan.received ? `${s.plan.poses} poses, ${s.plan.age_sec.toFixed(1)}s` : '-');
        setText('nodeSlam', okText(s.nodes.slam_toolbox));
        setText('nodeMapServer', okText(s.nodes.map_server));
        setText('nodeAmcl', okText(s.nodes.amcl));
        setText('nodeBt', okText(s.nodes.bt_navigator));
        setText('nodePlanner', okText(s.nodes.planner_server));
        setText('nodeController', okText(s.nodes.controller_server));
        setText('messages', s.messages.join('\n'));
      } catch (err) {
        setText('heartbeat', '连接失败: ' + err.message);
      }
    }
    refresh();
    setInterval(refresh, 1000);
  </script>
</body>
</html>
'''


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class DashboardNode:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node('go2_dashboard')
        self.lock = threading.Lock()
        self.last_cmd_vel = None
        self.last_cmd_time = None
        self.last_scan = None
        self.last_scan_time = None
        self.last_map = None
        self.last_map_time = None
        self.last_plan = None
        self.last_plan_time = None
        self.messages = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.cmd_pub = self.node.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.initialpose_pub = self.node.create_publisher(PoseWithCovarianceStamped, args.initialpose_topic, 10)

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(Twist, args.cmd_vel_topic, self.cmd_callback, 10)
        self.node.create_subscription(LaserScan, args.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.node.create_subscription(OccupancyGrid, args.map_topic, self.map_callback, transient_qos)
        self.node.create_subscription(NavPath, args.plan_topic, self.plan_callback, 10)
        self.log('dashboard started')

    def now_sec(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    def log(self, message):
        stamp = time.strftime('%H:%M:%S')
        with self.lock:
            self.messages.insert(0, f'{stamp} {message}')
            self.messages = self.messages[:60]

    def cmd_callback(self, msg):
        with self.lock:
            self.last_cmd_vel = msg
            self.last_cmd_time = self.now_sec()

    def scan_callback(self, msg):
        finite_count = sum(1 for r in msg.ranges if math.isfinite(r))
        with self.lock:
            self.last_scan = {
                'count': len(msg.ranges),
                'finite_count': finite_count,
                'frame_id': msg.header.frame_id,
            }
            self.last_scan_time = self.now_sec()

    def map_callback(self, msg):
        with self.lock:
            self.last_map = msg
            self.last_map_time = self.now_sec()

    def plan_callback(self, msg):
        with self.lock:
            self.last_plan = {'poses': len(msg.poses), 'frame_id': msg.header.frame_id}
            self.last_plan_time = self.now_sec()

    def motion_status(self, msg, age):
        if msg is None or age is None or age > 2.0:
            return {'label': '停止/无新命令', 'active': False}
        linear = msg.linear.x
        angular = msg.angular.z
        moving = abs(linear) > 0.05
        turning = abs(angular) > 0.10
        if not moving and not turning:
            return {'label': '停止', 'active': False}
        parts = []
        if linear > 0.05:
            parts.append('前进')
        elif linear < -0.05:
            parts.append('后退')
        if angular > 0.10:
            parts.append('左转')
        elif angular < -0.10:
            parts.append('右转')
        return {'label': ' + '.join(parts), 'active': True}

    def node_flags(self):
        names = {name for name, _namespace in self.node.get_node_names_and_namespaces()}
        return {
            'slam_toolbox': 'slam_toolbox' in names,
            'map_server': 'map_server' in names,
            'amcl': 'amcl' in names,
            'bt_navigator': 'bt_navigator' in names,
            'planner_server': 'planner_server' in names,
            'controller_server': 'controller_server' in names,
        }

    def mode_from_nodes(self, nodes):
        if nodes['slam_toolbox']:
            return 'SLAM 建图'
        if nodes['map_server'] and nodes['amcl']:
            return '已有地图定位导航'
        if nodes['bt_navigator'] or nodes['planner_server']:
            return 'Navigation2'
        return '空闲/未识别'

    def tf_status(self):
        try:
            if not self.tf_buffer.can_transform(
                self.args.map_frame,
                self.args.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            ):
                return {'ok': False, 'message': '无 TF'}
            transform = self.tf_buffer.lookup_transform(
                self.args.map_frame,
                self.args.base_frame,
                rclpy.time.Time(),
            )
            p = transform.transform.translation
            q = transform.transform.rotation
            yaw_deg = math.degrees(yaw_from_quaternion(q))
            return {
                'ok': True,
                'message': 'OK',
                'x': p.x,
                'y': p.y,
                'yaw_deg': yaw_deg,
            }
        except TransformException as exc:
            return {'ok': False, 'message': str(exc)}

    def get_status(self):
        now = self.now_sec()
        with self.lock:
            cmd = self.last_cmd_vel
            cmd_age = None if self.last_cmd_time is None else now - self.last_cmd_time
            scan = dict(self.last_scan) if self.last_scan else None
            scan_age = None if self.last_scan_time is None else now - self.last_scan_time
            map_msg = self.last_map
            map_age = None if self.last_map_time is None else now - self.last_map_time
            plan = dict(self.last_plan) if self.last_plan else None
            plan_age = None if self.last_plan_time is None else now - self.last_plan_time
            messages = list(self.messages)

        nodes = self.node_flags()
        status = {
            'time': time.strftime('%H:%M:%S'),
            'mode': self.mode_from_nodes(nodes),
            'nodes': nodes,
            'config': {
                'map_prefix': self.args.map_prefix,
                'pose_file': self.args.pose_file,
            },
            'cmd_vel': {
                'linear_x': cmd.linear.x if cmd else None,
                'angular_z': cmd.angular.z if cmd else None,
                'age_sec': cmd_age,
            },
            'motion': self.motion_status(cmd, cmd_age),
            'scan': {
                'received': scan is not None,
                'age_sec': scan_age,
                'count': scan['count'] if scan else 0,
                'finite_count': scan['finite_count'] if scan else 0,
                'frame_id': scan['frame_id'] if scan else '',
            },
            'map': {
                'received': map_msg is not None,
                'age_sec': map_age,
                'width': map_msg.info.width if map_msg else 0,
                'height': map_msg.info.height if map_msg else 0,
                'resolution': map_msg.info.resolution if map_msg else 0.0,
            },
            'plan': {
                'received': plan is not None,
                'age_sec': plan_age,
                'poses': plan['poses'] if plan else 0,
                'frame_id': plan['frame_id'] if plan else '',
            },
            'tf': self.tf_status(),
            'messages': messages,
        }
        return status

    def stop_robot(self):
        self.cmd_pub.publish(Twist())
        self.log('published zero /cmd_vel')
        return {'ok': True, 'message': '已发布零速度 Stop Robot'}

    def save_map_and_pose(self):
        map_prefix = Path(os.path.expanduser(self.args.map_prefix))
        map_prefix.parent.mkdir(parents=True, exist_ok=True)
        yaml_path = map_prefix.with_suffix('.yaml')
        pgm_path = map_prefix.with_suffix('.pgm')
        pose_path = Path(os.path.expanduser(self.args.pose_file))
        json_path = pose_path.with_suffix('.json')
        script_path = Path(os.path.expanduser(self.args.initialpose_script))

        with self.lock:
            map_msg = self.last_map
        if map_msg is None:
            raise RuntimeError(f'没有收到地图话题 {self.args.map_topic}')

        if not self.tf_buffer.can_transform(
            self.args.map_frame,
            self.args.base_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=1.0),
        ):
            raise RuntimeError(f'没有 {self.args.map_frame} -> {self.args.base_frame} TF')
        transform = self.tf_buffer.lookup_transform(
            self.args.map_frame,
            self.args.base_frame,
            rclpy.time.Time(),
        )
        pose = pose_dict_from_transform(transform)

        save_pgm(map_msg, pgm_path)
        save_map_yaml(map_msg, yaml_path, pgm_path.name)
        save_pose_yaml(pose, pose_path)
        json_path.write_text(json.dumps(pose, indent=2))
        save_initialpose_script(pose, script_path, pose_path)

        message = (
            '保存完成\n'
            f'地图 YAML: {yaml_path}\n'
            f'地图 PGM : {pgm_path}\n'
            f'起点 YAML: {pose_path}\n'
            f'起点 JSON: {json_path}\n'
            f'发布脚本 : {script_path}\n'
            f'x={pose["position"]["x"]:.6f}, y={pose["position"]["y"]:.6f}, '
            f'z={pose["orientation"]["z"]:.6f}, w={pose["orientation"]["w"]:.6f}'
        )
        self.log(message.replace('\n', ' | '))
        return {'ok': True, 'message': message}

    def publish_initial_pose(self):
        pose_file = os.path.expanduser(self.args.pose_file)
        pose = parse_pose_file_strict(pose_file)
        for i in range(max(1, self.args.initialpose_repeat)):
            msg = build_msg(self.node, pose)
            self.initialpose_pub.publish(msg)
            time.sleep(self.args.initialpose_period)
        message = (
            f'已发布固定初始位姿 {max(1, self.args.initialpose_repeat)} 次\n'
            f'pose_file: {pose_file}\n'
            f'x={pose["position"]["x"]:.6f}, y={pose["position"]["y"]:.6f}, '
            f'z={pose["orientation"]["z"]:.6f}, w={pose["orientation"]["w"]:.6f}'
        )
        self.log(message.replace('\n', ' | '))
        return {'ok': True, 'message': message}

    def destroy(self):
        self.node.destroy_node()


def make_handler(dashboard):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def send_json(self, data, status=200):
            payload = json.dumps(data, ensure_ascii=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            path = urlparse(self.path).path
            if path == '/':
                payload = HTML.encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
            elif path == '/api/status':
                self.send_json(dashboard.get_status())
            else:
                self.send_json({'ok': False, 'error': 'not found'}, status=404)

        def do_POST(self):
            path = urlparse(self.path).path
            try:
                if path == '/api/save':
                    self.send_json(dashboard.save_map_and_pose())
                elif path == '/api/stop':
                    self.send_json(dashboard.stop_robot())
                elif path == '/api/publish_initial_pose':
                    self.send_json(dashboard.publish_initial_pose())
                else:
                    self.send_json({'ok': False, 'error': 'not found'}, status=404)
            except Exception as exc:
                dashboard.log(f'operation failed: {exc}')
                self.send_json({'ok': False, 'error': str(exc)}, status=500)

    return Handler


def parse_args():
    default_map_prefix = os.path.join(os.path.expanduser('~'), 'go2_maps', 'go2_map')
    default_pose_file = os.path.join(os.path.expanduser('~'), 'go2_maps', 'go2_initial_pose.yaml')
    default_script = os.path.join(os.path.expanduser('~'), 'go2_maps', 'set_go2_initial_pose.sh')
    parser = argparse.ArgumentParser(description='Go2 ROS2 web dashboard')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--map-prefix', default=default_map_prefix)
    parser.add_argument('--pose-file', default=default_pose_file)
    parser.add_argument('--initialpose-script', default=default_script)
    parser.add_argument('--map-topic', default='/map')
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--plan-topic', default='/plan')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    parser.add_argument('--initialpose-topic', default='/initialpose')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--initialpose-repeat', type=int, default=5)
    parser.add_argument('--initialpose-period', type=float, default=0.5)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    dashboard = DashboardNode(args)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(dashboard))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f'Go2 Dashboard: http://{args.host}:{args.port}')
    print('如果在电脑浏览器访问板载机，请打开: http://<板载机IP>:8080')
    try:
        rclpy.spin(dashboard.node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        dashboard.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
