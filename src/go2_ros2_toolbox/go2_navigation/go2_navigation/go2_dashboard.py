#!/usr/bin/env python3
import argparse
import errno
import json
import math
import os
import signal
import shlex
import shutil
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatus
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
    button.small {
      min-height: 30px;
      padding: 5px 8px;
      border-radius: 6px;
      font-size: 12px;
    }
    .waypoint-list {
      display: grid;
      gap: 8px;
      margin-top: 12px;
    }
    .waypoint-row {
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto;
      align-items: center;
      gap: 10px;
      border: 1px solid #e4e9ef;
      border-radius: 8px;
      padding: 9px 10px;
      background: #f8fafc;
    }
    .waypoint-main {
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      font-size: 13px;
      overflow-wrap: anywhere;
    }
    .waypoint-actions {
      display: flex;
      flex-wrap: wrap;
      justify-content: flex-end;
      gap: 6px;
    }
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
        <button onclick="postAction('/api/save')">保存建图和初始点</button>
        <button onclick="postAction('/api/save_initial_pose')">保存当前点为初始点</button>
        <button class="secondary" onclick="postAction('/api/publish_initial_pose')">发布已保存初始点</button>
        <button class="danger" onclick="postAction('/api/stop')">停止机器狗</button>
      </div>
      <div style="height:12px"></div>
      <div class="grid">
        <div class="item"><span class="label">地图输出</span><span id="mapPrefix" class="value">-</span></div>
        <div class="item"><span class="label">起点文件</span><span id="poseFile" class="value">-</span></div>
      </div>
    </section>

    <section class="span-12">
      <h2>网页模式</h2>
      <div class="buttons">
        <button onclick="postAction('/api/mode/mapping')">建图模式</button>
        <button onclick="postAction('/api/mode/calibrate_patrol')">重新标定巡航点</button>
        <button onclick="postAction('/api/mode/patrol')">巡逻模式</button>
        <button onclick="postAction('/api/mode/cruise')">巡航模式</button>
        <button onclick="postAction('/api/mode/cruise_no_return')">巡航不回原点模式</button>
        <button class="danger" onclick="postAction('/api/mode/stop')">停止当前模式</button>
      </div>
      <div style="height:12px"></div>
      <div class="grid">
        <div class="item"><span class="label">网页启动模式</span><span id="managedMode" class="value">-</span></div>
        <div class="item"><span class="label">后台进程</span><span id="managedProcesses" class="value">-</span></div>
      </div>
    </section>

    <section class="span-12">
      <h2>巡航点</h2>
      <div class="buttons">
        <button onclick="postAction('/api/patrol/save_point')">保存当前点为巡航点</button>
        <button class="secondary" onclick="postAction('/api/patrol/overwrite_last')">覆盖最后一个巡航点</button>
        <button class="secondary" onclick="postAction('/api/patrol/delete_last')">删除最后一个巡航点</button>
        <button onclick="postAction('/api/patrol/start')">开始巡航</button>
        <button class="secondary" onclick="postAction('/api/patrol/stop')">停止巡航</button>
        <button class="danger" onclick="postAction('/api/patrol/clear')">清空巡航点</button>
      </div>
      <div style="height:12px"></div>
      <div class="grid">
        <div class="item"><span class="label">巡航点文件</span><span id="patrolFile" class="value">-</span></div>
        <div class="item"><span class="label">巡航状态</span><span id="patrolStatus" class="value">-</span></div>
        <div class="item"><span class="label">巡航点数量</span><span id="patrolCount" class="value">-</span></div>
        <div class="item"><span class="label">当前点</span><span id="patrolCurrent" class="value">-</span></div>
      </div>
      <div id="patrolList" class="waypoint-list">-</div>
    </section>

    <section class="span-6">
      <h2>SLAM / 感知</h2>
      <div class="grid">
        <div class="item"><span class="label">/map</span><span id="mapStatus" class="value">-</span></div>
        <div class="item"><span class="label">/scan</span><span id="scanStatus" class="value">-</span></div>
        <div class="item"><span class="label">/odom</span><span id="odomStatus" class="value">-</span></div>
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
    function renderPatrolPoints(points) {
      const el = document.getElementById('patrolList');
      el.innerHTML = '';
      if (!points || !points.length) {
        el.textContent = '还没有巡航点';
        return;
      }
      points.forEach((p) => {
        const row = document.createElement('div');
        row.className = 'waypoint-row';

        const main = document.createElement('div');
        main.className = 'waypoint-main';
        main.textContent = `${p.index}. ${p.name}  x=${fmt(p.x)}  y=${fmt(p.y)}  yaw=${fmt(p.yaw_deg)}°`;

        const actions = document.createElement('div');
        actions.className = 'waypoint-actions';

        const overwrite = document.createElement('button');
        overwrite.className = 'small secondary';
        overwrite.textContent = '覆盖';
        overwrite.onclick = () => postAction('/api/patrol/overwrite?index=' + encodeURIComponent(p.index - 1));

        const del = document.createElement('button');
        del.className = 'small danger';
        del.textContent = '删除';
        del.onclick = () => postAction('/api/patrol/delete?index=' + encodeURIComponent(p.index - 1));

        actions.appendChild(overwrite);
        actions.appendChild(del);
        row.appendChild(main);
        row.appendChild(actions);
        el.appendChild(row);
      });
    }
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
        setText('managedMode', s.managed.mode || '-');
        setText('managedProcesses', s.managed.processes.length ? s.managed.processes.join(', ') : '-');
        setText('patrolFile', s.config.patrol_file);
        setText('patrolStatus', s.patrol.active ? '巡航中' : '停止');
        setText('patrolCount', s.patrol.count);
        setText('patrolCurrent', s.patrol.current || '-');
        renderPatrolPoints(s.patrol.points);
        setText('mapStatus', s.map.received ? `${s.map.width}x${s.map.height}, ${s.map.age_sec.toFixed(1)}s` : '-');
        setText('scanStatus', s.scan.received ? `${s.scan.count} beams, finite ${s.scan.finite_count}, ${s.scan.age_sec.toFixed(1)}s` : '-');
        setText('odomStatus', s.odom.received ? `${s.odom.age_sec.toFixed(1)}s` : '-');
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


def quaternion_from_yaw(yaw):
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(yaw * 0.5),
        'w': math.cos(yaw * 0.5),
    }


def average_yaw(yaws):
    sin_sum = sum(math.sin(yaw) for yaw in yaws)
    cos_sum = sum(math.cos(yaw) for yaw in yaws)
    return math.atan2(sin_sum, cos_sum)


def yaw_from_pose_dict(pose):
    q = pose['orientation']
    class Quaternion:
        pass
    quat = Quaternion()
    quat.x = float(q.get('x', 0.0))
    quat.y = float(q.get('y', 0.0))
    quat.z = float(q['z'])
    quat.w = float(q['w'])
    return yaw_from_quaternion(quat)


def read_patrol_waypoints(path):
    path = Path(os.path.expanduser(path))
    if not path.exists():
        return []

    waypoints = []
    current = None
    in_waypoints = False
    for raw_line in path.read_text().splitlines():
        line = raw_line.rstrip()
        stripped = line.strip()
        if not stripped or stripped.startswith('#'):
            continue
        if stripped == 'waypoints:':
            in_waypoints = True
            continue
        if not in_waypoints:
            continue
        if stripped.startswith('- '):
            if current:
                waypoints.append(current)
            current = {}
            item = stripped[2:].strip()
            if item and ':' in item:
                key, value = item.split(':', 1)
                current[key.strip()] = value.strip()
            continue
        if current is not None and ':' in stripped:
            key, value = stripped.split(':', 1)
            key = key.strip()
            value = value.strip()
            if key in ('x', 'y', 'yaw'):
                current[key] = float(value)
            else:
                current[key] = value
    if current:
        waypoints.append(current)

    for index, waypoint in enumerate(waypoints, start=1):
        waypoint.setdefault('name', f'point_{index:03d}')
        waypoint.setdefault('frame_id', 'map')
        waypoint.setdefault('yaw', 0.0)
    return waypoints


def write_patrol_waypoints(path, waypoints):
    path = Path(os.path.expanduser(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = ['waypoints:']
    for index, waypoint in enumerate(waypoints, start=1):
        lines.extend([
            f'  - name: {waypoint.get("name", f"point_{index:03d}")}',
            f'    frame_id: {waypoint.get("frame_id", "map")}',
            f'    x: {float(waypoint["x"]):.10g}',
            f'    y: {float(waypoint["y"]):.10g}',
            f'    yaw: {float(waypoint["yaw"]):.10g}',
        ])
    path.write_text('\n'.join(lines) + '\n')


def renumber_default_patrol_names(waypoints):
    for index, waypoint in enumerate(waypoints, start=1):
        name = str(waypoint.get('name', ''))
        if not name or name.startswith('point_'):
            waypoint['name'] = f'point_{index:03d}'


class DashboardNode:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node('go2_dashboard')
        self.lock = threading.Lock()
        self.last_cmd_vel = None
        self.last_cmd_time = None
        self.last_scan = None
        self.last_scan_time = None
        self.last_odom = None
        self.last_odom_time = None
        self.last_map = None
        self.last_map_time = None
        self.last_plan = None
        self.last_plan_time = None
        self.messages = []
        self.patrol_lock = threading.Lock()
        self.patrol_thread = None
        self.patrol_stop_event = threading.Event()
        self.patrol_active = False
        self.patrol_current = ''
        self.current_goal_handle = None
        self.autostart_timer = None
        self.process_lock = threading.Lock()
        self.managed_processes = {}
        self.managed_mode = ''
        self.mode_thread = None
        self.mode_stop_event = threading.Event()
        self.log_dir = Path(os.path.expanduser('~/go2_logs'))
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.cmd_pub = self.node.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.initialpose_pub = self.node.create_publisher(PoseWithCovarianceStamped, args.initialpose_topic, 10)
        self.nav_to_pose_client = ActionClient(self.node, NavigateToPose, args.navigate_action)

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(Twist, args.cmd_vel_topic, self.cmd_callback, 10)
        self.node.create_subscription(LaserScan, args.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, args.odom_topic, self.odom_callback, 10)
        self.node.create_subscription(OccupancyGrid, args.map_topic, self.map_callback, transient_qos)
        self.node.create_subscription(NavPath, args.plan_topic, self.plan_callback, 10)
        self.log('dashboard started')
        if args.patrol_autostart:
            self.autostart_timer = self.node.create_timer(
                args.patrol_autostart_delay,
                self.patrol_autostart_callback,
            )

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

    def odom_callback(self, msg):
        with self.lock:
            self.last_odom = {
                'frame_id': msg.header.frame_id,
                'child_frame_id': msg.child_frame_id,
            }
            self.last_odom_time = self.now_sec()

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
            odom = dict(self.last_odom) if self.last_odom else None
            odom_age = None if self.last_odom_time is None else now - self.last_odom_time
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
                'patrol_file': self.args.patrol_file,
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
            'odom': {
                'received': odom is not None,
                'age_sec': odom_age,
                'frame_id': odom['frame_id'] if odom else '',
                'child_frame_id': odom['child_frame_id'] if odom else '',
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
            'patrol': self.patrol_status(),
            'managed': self.managed_status(),
            'messages': messages,
        }
        return status

    def managed_status(self):
        with self.process_lock:
            processes = []
            for name, proc in list(self.managed_processes.items()):
                if proc.poll() is None:
                    processes.append(name)
            return {
                'mode': self.managed_mode,
                'processes': processes,
            }

    def patrol_status(self):
        with self.patrol_lock:
            active = self.patrol_active
            current = self.patrol_current
        try:
            waypoints = read_patrol_waypoints(self.args.patrol_file)
        except Exception:
            waypoints = []
        points = []
        for index, waypoint in enumerate(waypoints, start=1):
            points.append({
                'index': index,
                'name': waypoint.get('name', f'point_{index:03d}'),
                'frame_id': waypoint.get('frame_id', self.args.map_frame),
                'x': float(waypoint.get('x', 0.0)),
                'y': float(waypoint.get('y', 0.0)),
                'yaw': float(waypoint.get('yaw', 0.0)),
                'yaw_deg': math.degrees(float(waypoint.get('yaw', 0.0))),
            })
        return {
            'active': active,
            'current': current,
            'count': len(waypoints),
            'points': points,
        }

    def current_map_pose(self):
        if not self.tf_buffer.can_transform(
            self.args.map_frame,
            self.args.base_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=1.0),
        ):
            with self.lock:
                odom_missing = self.last_odom is None
                odom_age = None if self.last_odom_time is None else self.now_sec() - self.last_odom_time
            if odom_missing or odom_age is None or odom_age > 2.0:
                raise RuntimeError(
                    f'没有 {self.args.map_frame} -> {self.args.base_frame} TF，'
                    f'而且 {self.args.odom_topic} 没有新数据。请先检查 go2_base 是否运行、'
                    '/utlidar/robot_pose 是否有数据'
                )
            raise RuntimeError(
                f'没有 {self.args.map_frame} -> {self.args.base_frame} TF；'
                f'{self.args.odom_topic} 有数据，但 AMCL 还没有发布 map -> odom。'
                '请确认 /amcl 已 active，并在 RViz 用 2D Pose Estimate 或重新发布初始点'
            )
        transform = self.tf_buffer.lookup_transform(
            self.args.map_frame,
            self.args.base_frame,
            rclpy.time.Time(),
        )
        t = transform.transform.translation
        q = transform.transform.rotation
        return {
            'frame_id': self.args.map_frame,
            'x': float(t.x),
            'y': float(t.y),
            'yaw': float(yaw_from_quaternion(q)),
        }

    def current_map_pose_average(self, sample_count=None, sample_period=None):
        sample_count = max(1, int(sample_count or self.args.waypoint_samples))
        sample_period = max(0.0, float(
            self.args.waypoint_sample_period if sample_period is None else sample_period
        ))
        poses = []
        for index in range(sample_count):
            poses.append(self.current_map_pose())
            if index + 1 < sample_count and sample_period > 0.0:
                time.sleep(sample_period)
        return {
            'frame_id': self.args.map_frame,
            'x': sum(pose['x'] for pose in poses) / len(poses),
            'y': sum(pose['y'] for pose in poses) / len(poses),
            'yaw': average_yaw([pose['yaw'] for pose in poses]),
            'sample_count': len(poses),
        }

    def stop_robot(self):
        self.cmd_pub.publish(Twist())
        self.log('published zero /cmd_vel')
        return {'ok': True, 'message': '已发布零速度 Stop Robot'}

    def map_yaml_path(self):
        return str(Path(os.path.expanduser(self.args.map_prefix)).with_suffix('.yaml'))

    def process_env(self):
        env = os.environ.copy()
        display = self.args.display or env.get('DISPLAY') or ':0'
        if display:
            env['DISPLAY'] = display
        if not env.get('DBUS_SESSION_BUS_ADDRESS'):
            uid = os.getuid()
            dbus_path = f'/run/user/{uid}/bus'
            if os.path.exists(dbus_path):
                env['DBUS_SESSION_BUS_ADDRESS'] = f'unix:path={dbus_path}'
        if env.get('DISPLAY') and not env.get('XAUTHORITY'):
            xauthority = os.path.join(os.path.expanduser('~'), '.Xauthority')
            if os.path.exists(xauthority):
                env['XAUTHORITY'] = xauthority
        return env

    def terminal_command(self, name, log_path, watched_pid=None):
        if not self.args.open_terminal:
            return None
        env = self.process_env()

        title = f'Go2 {name}'
        script = (
            f'echo "{title}"\n'
            f'echo "日志: {log_path}"\n'
            'echo "这个窗口只显示日志；停止/切换模式请用网页按钮。launch 结束后本窗口会自动关闭。"\n'
            f'touch {shlex.quote(str(log_path))}\n'
        )
        if watched_pid is not None:
            script += (
                f'if tail --help 2>/dev/null | grep -q -- "--pid"; then\n'
                f'  exec tail -n +1 -f --pid={int(watched_pid)} {shlex.quote(str(log_path))}\n'
                f'fi\n'
            )
        script += f'exec tail -n +1 -f {shlex.quote(str(log_path))}\n'

        if shutil.which('xterm'):
            return ['xterm', '-display', env.get('DISPLAY', ':0'), '-T', title, '-e', 'bash', '-lc', script]
        if shutil.which('gnome-terminal'):
            return ['gnome-terminal', '--display', env.get('DISPLAY', ':0'), '--title', title, '--wait', '--', 'bash', '-lc', script]
        if shutil.which('xfce4-terminal'):
            return ['xfce4-terminal', '--title', title, '--command', f'bash -lc {shlex.quote(script)}']
        if shutil.which('x-terminal-emulator'):
            return ['x-terminal-emulator', '-T', title, '-e', 'bash', '-lc', script]

        self.log('没有找到 gnome-terminal/xterm/xfce4-terminal，改为后台启动')
        return None

    def start_process(self, name, command):
        with self.process_lock:
            existing = self.managed_processes.get(name)
            if existing is not None and existing.poll() is None:
                return
            log_path = self.log_dir / f'{name}.log'
            log_file = log_path.open('w')
            log_file.write(f'$ {shlex.join(command)}\n\n')
            log_file.flush()
            env = self.process_env()
            self.log(f'启动后台进程 {name}: {shlex.join(command)}')
            self.log(f'图形环境 DISPLAY={env.get("DISPLAY", "-")}, XAUTHORITY={env.get("XAUTHORITY", "-")}')
            try:
                proc = subprocess.Popen(
                    command,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    env=env,
                )
                self.managed_processes[name] = proc
            finally:
                log_file.close()

            terminal = self.terminal_command(name, log_path, self.managed_processes[name].pid)
            if terminal:
                try:
                    self.log(f'打开 {name} 日志终端: {shlex.join(terminal)}')
                    terminal_proc = subprocess.Popen(
                        terminal,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                        env=env,
                    )
                    self.managed_processes[f'{name}_terminal'] = terminal_proc
                    time.sleep(0.3)
                    if terminal_proc.poll() is not None:
                        self.log(f'{name} 日志终端启动后立即退出，退出码: {terminal_proc.returncode}')
                    else:
                        self.log(f'已打开 {name} 日志终端: {log_path}')
                except Exception as exc:
                    self.log(f'打开日志终端失败，launch 仍在后台运行: {exc}')

    def stop_managed_processes(self):
        with self.process_lock:
            items = list(self.managed_processes.items())
            self.managed_processes = {}

        for name, proc in items:
            if proc.poll() is not None:
                continue
            self.log(f'停止后台进程 {name}')
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=8.0)
                continue
            except Exception:
                pass
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=5.0)
                continue
            except Exception:
                pass
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass

    def set_managed_mode(self, mode):
        with self.process_lock:
            self.managed_mode = mode

    def stop_mode(self):
        self.mode_stop_event.set()
        self.stop_patrol()
        self.stop_managed_processes()
        self.set_managed_mode('')
        self.cmd_pub.publish(Twist())
        message = '已停止网页启动的模式和巡航，并发布零速度'
        self.log(message)
        return {'ok': True, 'message': message}

    def start_mapping_mode(self):
        self.stop_mode()
        self.mode_stop_event.clear()
        self.start_process(
            'mapping',
            [
                'ros2',
                'launch',
                'go2_core',
                'go2_startup.launch.py',
                'use_rviz:=true',
                'video_enable:=false',
            ],
        )
        self.set_managed_mode('建图模式')
        message = (
            '建图模式已启动\n'
            '现在可以遥控机器狗建图，也可以在网页中连续保存多个巡航点。\n'
            '保存巡航点后可以直接点击“开始巡航”，建图和巡航会继续使用当前 SLAM 地图。\n'
            '完成建图后再点击“保存建图和初始点”，用于后续保存地图巡逻。'
        )
        self.log(message.replace('\n', ' | '))
        return {'ok': True, 'message': message}

    def wait_for_lifecycle_active(self, node_name, timeout_sec):
        start = time.monotonic()
        while time.monotonic() - start < timeout_sec:
            if self.mode_stop_event.is_set():
                raise RuntimeError('模式启动已停止')
            try:
                result = subprocess.run(
                    ['ros2', 'lifecycle', 'get', node_name],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    timeout=3.0,
                )
                if result.returncode == 0 and 'active' in result.stdout:
                    return
            except Exception:
                pass
            time.sleep(1.0)
        raise RuntimeError(f'{node_name} 未在 {timeout_sec:.0f} 秒内进入 active 状态')

    def reset_nav_observations(self):
        with self.lock:
            self.last_map = None
            self.last_map_time = None
            self.last_scan = None
            self.last_scan_time = None
            self.last_plan = None
            self.last_plan_time = None

    def wait_for_recent_observation(self, name, attr_name, timeout_sec):
        start = time.monotonic()
        while time.monotonic() - start < timeout_sec:
            if self.mode_stop_event.is_set():
                raise RuntimeError('模式启动已停止')
            with self.lock:
                value = getattr(self, attr_name)
            if value is not None:
                self.log(f'已收到{name}')
                return
            time.sleep(0.2)
        raise RuntimeError(f'{timeout_sec:.0f} 秒内没有收到{name}')

    def wait_for_initialpose_subscriber(self, timeout_sec):
        start = time.monotonic()
        while time.monotonic() - start < timeout_sec:
            if self.mode_stop_event.is_set():
                raise RuntimeError('模式启动已停止')
            count = self.initialpose_pub.get_subscription_count()
            if count > 0:
                self.log(f'/initialpose 已连接订阅者: {count}')
                return
            time.sleep(0.2)
        raise RuntimeError('/initialpose 没有订阅者，AMCL 可能还没有启动完成')

    def transform_stamp_sec(self, transform):
        stamp = transform.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) / 1e9

    def wait_for_localization_tf(self, timeout_sec):
        start = time.monotonic()
        last_publish = 0.0
        last_wait_log = 0.0
        publish_count = 0
        pose = parse_pose_file_strict(os.path.expanduser(self.args.pose_file))
        first_publish_ros_sec = None
        while time.monotonic() - start < timeout_sec:
            if self.mode_stop_event.is_set():
                raise RuntimeError('模式启动已停止')

            now = time.monotonic()
            if now - last_publish >= self.args.initialpose_period:
                msg = build_msg(self.node, pose)
                self.initialpose_pub.publish(msg)
                if first_publish_ros_sec is None:
                    first_publish_ros_sec = (
                        float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
                    )
                    self.log(
                        '开始发布保存的初始点: '
                        f'x={pose["position"]["x"]:.3f}, y={pose["position"]["y"]:.3f}, '
                        f'z={pose["orientation"]["z"]:.3f}, w={pose["orientation"]["w"]:.3f}'
                    )
                publish_count += 1
                last_publish = now
            if now - last_wait_log >= 5.0:
                self.log(
                    f'等待定位 TF {self.args.map_frame} -> {self.args.base_frame}，'
                    f'已发布初始点 {publish_count} 次'
                )
                last_wait_log = now

            if self.tf_buffer.can_transform(
                self.args.map_frame,
                self.args.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            ):
                transform = self.tf_buffer.lookup_transform(
                    self.args.map_frame,
                    self.args.base_frame,
                    rclpy.time.Time(),
                )
                tf_stamp = self.transform_stamp_sec(transform)
                current_ros_sec = self.now_sec()
                is_fresh = (
                    first_publish_ros_sec is not None
                    and tf_stamp >= first_publish_ros_sec - 0.5
                    and current_ros_sec - tf_stamp <= self.args.mode_tf_max_age
                )
                if is_fresh:
                    t = transform.transform.translation
                    q = transform.transform.rotation
                    self.log(
                        f'保存初始点已生效，定位 TF 新鲜: {self.args.map_frame} -> {self.args.base_frame}, '
                        f'x={t.x:.3f}, y={t.y:.3f}, yaw={math.degrees(yaw_from_quaternion(q)):.1f}°, '
                        f'发布 {publish_count} 次'
                    )
                    return

            time.sleep(0.1)
        raise RuntimeError(
            f'发布初始点后仍没有 {self.args.map_frame} -> {self.args.base_frame} TF，'
            '请检查初始点是否属于当前保存地图、/scan 是否正常、AMCL 是否 active'
        )

    def initial_pose_waypoint(self):
        pose = parse_pose_file_strict(os.path.expanduser(self.args.pose_file))
        return {
            'name': 'initial_pose',
            'frame_id': pose.get('frame_id', self.args.map_frame),
            'x': pose['position']['x'],
            'y': pose['position']['y'],
            'yaw': yaw_from_pose_dict(pose),
        }

    def build_mode_waypoints(self, return_to_start):
        waypoints = read_patrol_waypoints(self.args.patrol_file)
        if not waypoints:
            raise RuntimeError(f'没有巡航点，请先在“重新标定巡航点”模式保存巡航点: {self.args.patrol_file}')
        if return_to_start:
            waypoints = list(waypoints)
            waypoints.append(self.initial_pose_waypoint())
        return waypoints

    def start_saved_map_sequence(self, mode_label, wait_duration, return_to_start, start_patrol_after=True):
        nav_launch_started = False
        try:
            map_yaml = self.map_yaml_path()
            pose_file = os.path.expanduser(self.args.pose_file)
            if not Path(map_yaml).exists():
                raise RuntimeError(f'地图不存在，请先保存建图: {map_yaml}')
            if not Path(pose_file).exists():
                raise RuntimeError(f'初始点不存在，请先保存初始点: {self.args.pose_file}')
            waypoints = self.build_mode_waypoints(return_to_start) if start_patrol_after else []

            self.set_managed_mode(f'{mode_label}启动中')
            self.reset_nav_observations()
            self.log(f'{mode_label}加载地图: {map_yaml}')
            display_env = self.process_env()
            self.log(
                f'{mode_label}将打开 RViz 和日志终端: '
                f'DISPLAY={display_env.get("DISPLAY", "-")}, '
                f'XAUTHORITY={display_env.get("XAUTHORITY", "-")}'
            )
            command = [
                'ros2',
                'launch',
                'go2_navigation',
                'go2_localization_nav.launch.py',
                f'map:={map_yaml}',
                'auto_localize:=false',
                'use_rviz:=true',
                'video_enable:=false',
                # go2_localization_nav.launch.py starts AMCL/Nav2 only. The
                # dashboard publishes the saved initial pose after AMCL connects.
            ]
            self.start_process('saved_map_nav', command)
            nav_launch_started = True
            self.log(f'{mode_label}将使用保存的初始点: {pose_file}')
            self.log('等待 AMCL / Nav2 进入 active')
            self.wait_for_lifecycle_active('/amcl', self.args.mode_start_timeout)
            self.wait_for_lifecycle_active('/bt_navigator', self.args.mode_start_timeout)
            self.wait_for_lifecycle_active('/planner_server', self.args.mode_start_timeout)
            self.wait_for_lifecycle_active('/controller_server', self.args.mode_start_timeout)
            self.wait_for_recent_observation('/map 地图', 'last_map', self.args.mode_start_timeout)
            self.wait_for_recent_observation('/scan 激光', 'last_scan', self.args.mode_start_timeout)

            if self.mode_stop_event.is_set():
                raise RuntimeError('模式启动已停止')
            self.wait_for_initialpose_subscriber(self.args.mode_start_timeout)
            self.wait_for_localization_tf(self.args.mode_localization_timeout)
            time.sleep(self.args.mode_initialpose_settle)
            if start_patrol_after:
                self.start_patrol(
                    waypoints=waypoints,
                    wait_duration=wait_duration,
                    loop=False,
                    mode_label=mode_label,
                )
            else:
                self.log(
                    f'{mode_label}已就绪: 请确认 RViz 中 /scan 与地图重合，'
                    '机器狗停稳后保存/覆盖巡航点'
                )
            self.set_managed_mode(mode_label)
        except Exception as exc:
            if not self.mode_stop_event.is_set() and not nav_launch_started:
                self.stop_managed_processes()
            if nav_launch_started and not self.mode_stop_event.is_set():
                self.set_managed_mode(f'{mode_label}等待人工定位')
                if start_patrol_after:
                    self.log(
                        f'{mode_label}未能自动开始，但保存地图导航和 RViz 已保留: {exc} | '
                        '可以在 RViz 里用 2D Pose Estimate 设置初始位姿，'
                        '定位成功后再点“开始巡航”或重新点巡逻模式'
                    )
                else:
                    self.log(
                        f'{mode_label}自动定位未确认，但保存地图导航和 RViz 已保留: {exc} | '
                        '可以在 RViz 里用 2D Pose Estimate 设置初始位姿，'
                        '确认 /scan 与地图重合后再保存巡航点'
                    )
            else:
                self.set_managed_mode('')
                self.log(f'{mode_label}启动失败: {exc}')

    def start_saved_map_mode(self, mode_label, wait_duration, return_to_start):
        self.stop_mode()
        self.mode_stop_event.clear()
        self.mode_thread = threading.Thread(
            target=self.start_saved_map_sequence,
            args=(mode_label, wait_duration, return_to_start, True),
            daemon=True,
        )
        self.mode_thread.start()
        message = (
            f'{mode_label}正在启动\n'
            f'点间等待: {wait_duration:.1f} 秒\n'
            f'结束回到初始点: {"是" if return_to_start else "否"}'
        )
        self.log(message.replace('\n', ' | '))
        return {'ok': True, 'message': message}

    def start_calibrate_patrol_mode(self):
        self.stop_mode()
        self.mode_stop_event.clear()
        self.mode_thread = threading.Thread(
            target=self.start_saved_map_sequence,
            args=('重新标定巡航点', 0.0, False, False),
            daemon=True,
        )
        self.mode_thread.start()
        message = (
            '重新标定巡航点模式正在启动\n'
            '会加载已保存地图、发布已保存初始点并打开 RViz，但不会自动巡逻。\n'
            '等 RViz 中 /scan 与地图重合、机器狗停稳后，可以保存当前点为初始点，'
            '也可以保存/覆盖/删除巡航点。'
        )
        self.log(message.replace('\n', ' | '))
        return {'ok': True, 'message': message}

    def pose_dict_from_averaged_map_pose(self, averaged_pose):
        q = quaternion_from_yaw(float(averaged_pose['yaw']))
        return {
            'frame_id': self.args.map_frame,
            'child_frame_id': self.args.base_frame,
            'position': {
                'x': float(averaged_pose['x']),
                'y': float(averaged_pose['y']),
                'z': 0.0,
            },
            'orientation': q,
            'covariance': {
                'x': 0.25,
                'y': 0.25,
                'yaw': 0.0685,
            },
        }

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

    def save_initial_pose_only(self):
        pose_path = Path(os.path.expanduser(self.args.pose_file))
        pose_path.parent.mkdir(parents=True, exist_ok=True)
        json_path = pose_path.with_suffix('.json')
        script_path = Path(os.path.expanduser(self.args.initialpose_script))

        averaged_pose = self.current_map_pose_average()
        sample_count = averaged_pose.pop('sample_count')
        pose = self.pose_dict_from_averaged_map_pose(averaged_pose)

        save_pose_yaml(pose, pose_path)
        json_path.write_text(json.dumps(pose, indent=2))
        save_initialpose_script(pose, script_path, pose_path)

        message = (
            '已保存当前点为初始点\n'
            f'TF 平均样本: {sample_count} 次\n'
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

    def save_patrol_point(self):
        if self.is_patrol_active():
            raise RuntimeError('巡航正在运行，请先停止巡航再保存巡航点')
        waypoints = read_patrol_waypoints(self.args.patrol_file)
        pose = self.current_map_pose_average()
        sample_count = pose.pop('sample_count')
        pose['name'] = f'point_{len(waypoints) + 1:03d}'
        waypoints.append(pose)
        write_patrol_waypoints(self.args.patrol_file, waypoints)

        message = (
            f'已保存巡航点 {pose["name"]}\n'
            f'TF 平均样本: {sample_count} 次\n'
            f'文件: {os.path.expanduser(self.args.patrol_file)}\n'
            f'x={pose["x"]:.6f}, y={pose["y"]:.6f}, yaw={math.degrees(pose["yaw"]):.2f}°\n'
            f'当前巡航点数量: {len(waypoints)}'
        )
        self.log(message.replace('\n', ' | '))
        return {'ok': True, 'message': message}

    def overwrite_patrol_point(self, index=None):
        if self.is_patrol_active():
            raise RuntimeError('巡航正在运行，请先停止巡航再覆盖巡航点')
        waypoints = read_patrol_waypoints(self.args.patrol_file)
        if not waypoints:
            raise RuntimeError('还没有巡航点，请先保存一个巡航点')
        if index is None:
            index = len(waypoints) - 1
        if index < 0 or index >= len(waypoints):
            raise RuntimeError(f'巡航点序号超出范围: {index + 1}')

        old_name = waypoints[index].get('name', f'point_{index + 1:03d}')
        pose = self.current_map_pose_average()
        sample_count = pose.pop('sample_count')
        pose['name'] = old_name
        waypoints[index] = pose
        write_patrol_waypoints(self.args.patrol_file, waypoints)

        message = (
            f'已覆盖巡航点 {index + 1}: {old_name}\n'
            f'TF 平均样本: {sample_count} 次\n'
            f'文件: {os.path.expanduser(self.args.patrol_file)}\n'
            f'x={pose["x"]:.6f}, y={pose["y"]:.6f}, yaw={math.degrees(pose["yaw"]):.2f}°'
        )
        self.log(message.replace('\n', ' | '))
        return {'ok': True, 'message': message}

    def delete_patrol_point(self, index=None):
        if self.is_patrol_active():
            raise RuntimeError('巡航正在运行，请先停止巡航再删除巡航点')
        waypoints = read_patrol_waypoints(self.args.patrol_file)
        if not waypoints:
            raise RuntimeError('还没有巡航点可删除')
        if index is None:
            index = len(waypoints) - 1
        if index < 0 or index >= len(waypoints):
            raise RuntimeError(f'巡航点序号超出范围: {index + 1}')

        removed = waypoints.pop(index)
        renumber_default_patrol_names(waypoints)
        write_patrol_waypoints(self.args.patrol_file, waypoints)
        message = (
            f'已删除巡航点 {index + 1}: {removed.get("name", "-")}\n'
            f'当前巡航点数量: {len(waypoints)}'
        )
        self.log(message.replace('\n', ' | '))
        return {'ok': True, 'message': message}

    def save_all(self):
        map_result = self.save_map_and_pose()
        message = (
            '保存建图和初始点完成\n\n'
            f'{map_result["message"]}\n\n'
            '建图模式中已经保存的巡航点会保留，可继续点击“开始巡航”。\n'
            '后续启动保存地图巡逻前，请确认巡航点和初始点已经标定完成。'
        )
        return {'ok': True, 'message': message}

    def clear_patrol_points(self):
        if self.is_patrol_active():
            raise RuntimeError('巡航正在运行，请先停止巡航')
        write_patrol_waypoints(self.args.patrol_file, [])
        message = f'已清空巡航点: {os.path.expanduser(self.args.patrol_file)}'
        self.log(message)
        return {'ok': True, 'message': message}

    def is_patrol_active(self):
        with self.patrol_lock:
            return self.patrol_active

    def set_patrol_state(self, active=None, current=None, goal_handle_marker=False, goal_handle=None):
        with self.patrol_lock:
            if active is not None:
                self.patrol_active = active
            if current is not None:
                self.patrol_current = current
            if goal_handle_marker:
                self.current_goal_handle = goal_handle

    def wait_for_future(self, future, stop_event, timeout_sec=None):
        start = time.monotonic()
        while rclpy.ok() and not stop_event.is_set():
            if future.done():
                return future.result()
            if timeout_sec is not None and time.monotonic() - start > timeout_sec:
                raise RuntimeError('等待 Nav2 action 超时')
            time.sleep(0.05)
        raise RuntimeError('巡航已停止')

    def build_nav_goal(self, waypoint):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = waypoint.get('frame_id', self.args.map_frame)
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(waypoint['x'])
        goal.pose.pose.position.y = float(waypoint['y'])
        goal.pose.pose.position.z = 0.0
        q = quaternion_from_yaw(float(waypoint.get('yaw', 0.0)))
        goal.pose.pose.orientation.x = q['x']
        goal.pose.pose.orientation.y = q['y']
        goal.pose.pose.orientation.z = q['z']
        goal.pose.pose.orientation.w = q['w']
        return goal

    def patrol_worker(self, waypoints, wait_duration=None, loop=None, mode_label='巡航'):
        if wait_duration is None:
            wait_duration = self.args.patrol_wait
        if loop is None:
            loop = self.args.patrol_loop
        try:
            self.log(f'{mode_label}开始，共 {len(waypoints)} 个点')
            if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
                raise RuntimeError(f'Nav2 action 不可用: {self.args.navigate_action}')

            cycles = 0
            while rclpy.ok() and not self.patrol_stop_event.is_set():
                cycles += 1
                for index, waypoint in enumerate(waypoints, start=1):
                    if self.patrol_stop_event.is_set():
                        break
                    name = waypoint.get('name', f'point_{index:03d}')
                    label = f'{index}/{len(waypoints)} {name}'
                    self.set_patrol_state(current=label)
                    self.log(
                        f'发送巡航点 {label}: '
                        f'x={float(waypoint["x"]):.3f}, y={float(waypoint["y"]):.3f}, '
                        f'yaw={math.degrees(float(waypoint.get("yaw", 0.0))):.1f}°'
                    )

                    send_future = self.nav_to_pose_client.send_goal_async(self.build_nav_goal(waypoint))
                    goal_handle = self.wait_for_future(send_future, self.patrol_stop_event, timeout_sec=10.0)
                    if not goal_handle.accepted:
                        raise RuntimeError(f'巡航点 {label} 被 Nav2 拒绝')
                    self.set_patrol_state(goal_handle_marker=True, goal_handle=goal_handle)

                    result_future = goal_handle.get_result_async()
                    result = self.wait_for_future(result_future, self.patrol_stop_event)
                    self.set_patrol_state(goal_handle_marker=True, goal_handle=None)
                    if result.status != GoalStatus.STATUS_SUCCEEDED:
                        if not self.args.patrol_continue_on_failure:
                            raise RuntimeError(f'巡航点 {label} 导航失败，状态码 {result.status}')
                        self.log(f'巡航点 {label} 导航失败，状态码 {result.status}，继续下一个点')
                    else:
                        self.log(f'已到达巡航点 {label}')
                    has_next = index < len(waypoints) or loop
                    if wait_duration > 0 and has_next and not self.patrol_stop_event.is_set():
                        time.sleep(wait_duration)

                if not loop:
                    break
                self.log(f'巡航第 {cycles} 圈完成，继续循环')

            if self.patrol_stop_event.is_set():
                self.log('巡航已停止')
            else:
                self.log('巡航完成')
        except Exception as exc:
            self.log(f'巡航失败: {exc}')
        finally:
            self.set_patrol_state(active=False, current='', goal_handle_marker=True, goal_handle=None)

    def start_patrol(self, waypoints=None, wait_duration=None, loop=None, mode_label='巡航'):
        if self.is_patrol_active():
            raise RuntimeError('巡航已经在运行')
        if waypoints is None:
            waypoints = read_patrol_waypoints(self.args.patrol_file)
        if not waypoints:
            raise RuntimeError(f'没有巡航点，请先保存巡航点: {self.args.patrol_file}')
        if wait_duration is None:
            wait_duration = self.args.patrol_wait
        if loop is None:
            loop = self.args.patrol_loop

        self.patrol_stop_event.clear()
        self.set_patrol_state(active=True, current='准备启动', goal_handle_marker=True, goal_handle=None)
        self.patrol_thread = threading.Thread(
            target=self.patrol_worker,
            args=(waypoints, wait_duration, loop, mode_label),
            daemon=True,
        )
        self.patrol_thread.start()
        message = (
            f'{mode_label}已启动，共 {len(waypoints)} 个点\n'
            f'文件: {os.path.expanduser(self.args.patrol_file)}\n'
            f'点间等待: {wait_duration:.1f} 秒\n'
            f'循环: {"是" if loop else "否"}\n'
            f'失败后继续: {"是" if self.args.patrol_continue_on_failure else "否"}'
        )
        self.log(message.replace('\n', ' | '))
        return {'ok': True, 'message': message}

    def stop_patrol(self):
        self.patrol_stop_event.set()
        with self.patrol_lock:
            goal_handle = self.current_goal_handle
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.log(f'取消当前巡航目标失败: {exc}')
        self.cmd_pub.publish(Twist())
        message = '已请求停止巡航，并发布零速度'
        self.log(message)
        return {'ok': True, 'message': message}

    def patrol_autostart_callback(self):
        if self.autostart_timer is not None:
            self.autostart_timer.cancel()
            self.autostart_timer = None
        try:
            result = self.start_patrol()
            self.log(result['message'].replace('\n', ' | '))
        except Exception as exc:
            self.log(f'自动巡航启动失败: {exc}')

    def destroy(self):
        self.patrol_stop_event.set()
        self.mode_stop_event.set()
        self.stop_managed_processes()
        self.node.destroy_node()


def make_handler(dashboard):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def send_json(self, data, status=200):
            payload = json.dumps(data, ensure_ascii=False).encode('utf-8')
            try:
                self.send_response(status)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                return False
            except OSError as exc:
                if exc.errno in (errno.EPIPE, errno.ECONNRESET):
                    return False
                raise
            return True

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
            parsed = urlparse(self.path)
            path = parsed.path
            query = parse_qs(parsed.query)
            try:
                index = None
                if 'index' in query and query['index']:
                    index = int(query['index'][0])
                if path == '/api/save':
                    self.send_json(dashboard.save_map_and_pose())
                elif path == '/api/save_initial_pose':
                    self.send_json(dashboard.save_initial_pose_only())
                elif path == '/api/save_all':
                    self.send_json(dashboard.save_all())
                elif path == '/api/stop':
                    self.send_json(dashboard.stop_robot())
                elif path == '/api/publish_initial_pose':
                    self.send_json(dashboard.publish_initial_pose())
                elif path == '/api/patrol/save_point':
                    self.send_json(dashboard.save_patrol_point())
                elif path == '/api/patrol/overwrite':
                    self.send_json(dashboard.overwrite_patrol_point(index))
                elif path == '/api/patrol/overwrite_last':
                    self.send_json(dashboard.overwrite_patrol_point())
                elif path == '/api/patrol/delete':
                    self.send_json(dashboard.delete_patrol_point(index))
                elif path == '/api/patrol/delete_last':
                    self.send_json(dashboard.delete_patrol_point())
                elif path == '/api/patrol/start':
                    self.send_json(dashboard.start_patrol())
                elif path == '/api/patrol/stop':
                    self.send_json(dashboard.stop_patrol())
                elif path == '/api/patrol/clear':
                    self.send_json(dashboard.clear_patrol_points())
                elif path == '/api/mode/mapping':
                    self.send_json(dashboard.start_mapping_mode())
                elif path == '/api/mode/calibrate_patrol':
                    self.send_json(dashboard.start_calibrate_patrol_mode())
                elif path == '/api/mode/patrol':
                    self.send_json(dashboard.start_saved_map_mode('巡逻模式', 0.0, True))
                elif path == '/api/mode/cruise':
                    self.send_json(dashboard.start_saved_map_mode('巡航模式', 3.0, True))
                elif path == '/api/mode/cruise_no_return':
                    self.send_json(dashboard.start_saved_map_mode('巡航不回原点模式', 3.0, False))
                elif path == '/api/mode/stop':
                    self.send_json(dashboard.stop_mode())
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
    default_patrol_file = os.path.join(os.path.expanduser('~'), 'go2_maps', 'patrol_waypoints.yaml')
    parser = argparse.ArgumentParser(description='Go2 ROS2 web dashboard')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--map-prefix', default=default_map_prefix)
    parser.add_argument('--pose-file', default=default_pose_file)
    parser.add_argument('--initialpose-script', default=default_script)
    parser.add_argument('--patrol-file', default=default_patrol_file)
    parser.add_argument('--map-topic', default='/map')
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--plan-topic', default='/plan')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    parser.add_argument('--initialpose-topic', default='/initialpose')
    parser.add_argument('--navigate-action', default='/navigate_to_pose')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--initialpose-repeat', type=int, default=5)
    parser.add_argument('--initialpose-period', type=float, default=0.5)
    parser.add_argument('--patrol-loop', action='store_true')
    parser.add_argument('--patrol-wait', type=float, default=3.0)
    parser.add_argument('--patrol-continue-on-failure', action='store_true')
    parser.add_argument('--patrol-autostart', action='store_true')
    parser.add_argument('--patrol-autostart-delay', type=float, default=5.0)
    parser.add_argument('--waypoint-samples', type=int, default=10)
    parser.add_argument('--waypoint-sample-period', type=float, default=0.1)
    parser.add_argument('--mode-start-timeout', type=float, default=90.0)
    parser.add_argument('--mode-initialpose-settle', type=float, default=5.0)
    parser.add_argument('--mode-localization-timeout', type=float, default=90.0)
    parser.add_argument('--mode-tf-max-age', type=float, default=2.0)
    parser.add_argument('--display', default='')
    parser.add_argument('--open-terminal', dest='open_terminal', action='store_true', default=True)
    parser.add_argument('--no-open-terminal', dest='open_terminal', action='store_false')
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
