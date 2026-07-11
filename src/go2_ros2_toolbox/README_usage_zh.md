# Go2 ROS2 Toolbox 使用指南

本项目是面向 Unitree Go2 EDU 机器人的 ROS 2 工具箱，用于将 Go2 自带的位姿、雷达点云、相机和运动控制接口接入 ROS 2 生态，并集成 SLAM Toolbox 与 Navigation2，实现建图、自主导航和可视化调试。

项目主要完成以下工作：

- 将 Go2 原生位姿 `/utlidar/robot_pose` 转换为 ROS 导航需要的 `/odom` 和 TF。
- 将 Go2 原生雷达点云 `/utlidar/cloud_deskewed` 处理为 `/scan`。
- 使用 SLAM Toolbox 基于 `/scan` 建图。
- 使用 Navigation2 进行路径规划、避障和目标点导航。
- 将 Navigation2 输出的 `/cmd_vel` 转换为 Unitree 运动 API 请求 `/api/sport/request`。
- 可选启动 Go2 前视相机视频流，并发布为 ROS Image 话题。

> 注意：本项目主要面向 Unitree Go2 EDU 扩展坞自带计算机环境。其他部署方式，例如外部 PC 直连机器人，可能需要额外网络、DDS、中间件和话题配置。

## 1. 项目结构

```text
go2_ros2_toolbox/
├── go2_core/              # Go2 底盘桥接、里程计、TF、视频流
├── go2_perception/        # 点云累积、点云转 LaserScan
├── go2_slam/              # SLAM Toolbox 启动与参数
├── go2_navigation/        # Navigation2 启动与参数
├── unitree_pkgs/          # Unitree 消息定义与运动 API 封装
├── asset/                 # 演示资源
├── README.md
└── README_zh.md
```

各包功能如下：

| 包名 | 作用 |
| --- | --- |
| `go2_core` | 启动底盘桥接节点，将 Go2 位姿发布为 `/odom`，广播 `odom -> base_link`，并将 `/cmd_vel` 转换为 Unitree 运动请求 |
| `go2_perception` | 处理 Go2 雷达点云，将 3D 点云累积、过滤并转换为 2D `/scan` |
| `go2_slam` | 启动 SLAM Toolbox，用 `/scan` 和 `/odom` 在线建图 |
| `go2_navigation` | 启动 Navigation2，实现路径规划、局部避障和目标点导航 |
| `unitree_pkgs/unitree_go` | Unitree Go2 相关 ROS 消息定义 |
| `unitree_pkgs/unitree_api` | Unitree API 请求/响应消息定义 |
| `unitree_pkgs/go2_sport_api` | Unitree Sport API 的 ROS 2 封装 |
| `unitree_pkgs/go2_h264_repub` | H264 视频流转 ROS Image 的备用实现 |

## 2. 环境要求

推荐环境：

| 项目 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 20.04 |
| ROS 2 | Foxy |
| 机器人 | Unitree Go2 EDU |
| 固件 | v1.1.7 已测试 |
| 部署位置 | Go2 EDU 扩展坞自带计算机 |

需要提前安装 Unitree 官方 ROS 2 包：

```bash
# 参考 Unitree 官方 ROS 2 仓库
# https://github.com/unitreerobotics/unitree_ros2
```

## 3. 安装依赖

安装 ROS 2 依赖：

```bash
sudo apt update

sudo apt install -y \
  ros-foxy-navigation2 \
  ros-foxy-nav2-bringup \
  ros-foxy-pcl-ros \
  ros-foxy-tf-transformations \
  ros-foxy-slam-toolbox \
  ros-foxy-cv-bridge \
  ros-foxy-tf2-ros \
  ros-foxy-tf2-sensor-msgs \
  ros-foxy-message-filters \
  ros-foxy-laser-geometry
```

安装 Python 依赖：

```bash
pip3 install transforms3d
```

如果需要使用视频流，还需要确保系统支持 OpenCV 与 GStreamer：

```bash
sudo apt install -y \
  python3-opencv \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav
```

## 4. 创建工作空间并编译

创建 ROS 2 工作空间：

```bash
mkdir -p ~/go2_ros2_ws/src
cd ~/go2_ros2_ws/src
```

克隆本项目：

```bash
git clone https://github.com/andy-zhuo-02/go2_ros2_toolbox.git
```

返回工作空间根目录并编译：

```bash
cd ~/go2_ros2_ws
colcon build
```

加载环境：

```bash
source install/setup.bash
```

建议将环境加载命令加入 `~/.bashrc`：

```bash
echo "source ~/go2_ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 5. 快速启动

完整启动机器人、点云处理、SLAM、Navigation2 和 RViz：

```bash
ros2 launch go2_core go2_startup.launch.py
```

该命令会启动以下模块：

```text
go2_base
video_stream_node
cloud_accumulation
pointcloud_to_laserscan_node
slam_toolbox
navigation2
rviz2
```

启动后，可以在 RViz 中查看机器人位姿、点云、LaserScan、地图和导航状态。

## 6. 分模块启动

如果你想单独调试各模块，可以按下面方式逐个启动。

### 6.1 启动底盘桥接

```bash
ros2 launch go2_core go2_base.launch.py
```

该 launch 默认启动：

- `go2_base`
- 可选视频节点 `video_stream_node.py`

`go2_base` 的主要作用：

```text
订阅：
/utlidar/robot_pose
/cmd_vel

发布：
/odom
/api/sport/request

广播 TF：
odom -> base_link
```

验证底盘位姿：

```bash
ros2 topic echo /odom
```

验证 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

手动发送速度测试：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
"
```

停止机器人：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
"
```

### 6.2 启动点云处理

```bash
ros2 launch go2_perception go2_pointcloud_process.launch.py
```

该模块会启动：

- `cloud_accumulation`
- `pointcloud_to_laserscan_node`

数据流如下：

```text
/utlidar/cloud_deskewed
        |
        v
cloud_accumulation
        |
        v
/trans_cloud
        |
        v
pointcloud_to_laserscan_node
        |
        v
/scan
```

检查原始点云：

```bash
ros2 topic echo /utlidar/cloud_deskewed
```

检查处理后的点云：

```bash
ros2 topic echo /trans_cloud
```

检查 LaserScan：

```bash
ros2 topic echo /scan
```

在 RViz 中可以添加：

- `PointCloud2`，话题选择 `/utlidar/cloud_deskewed`
- `PointCloud2`，话题选择 `/trans_cloud`
- `LaserScan`，话题选择 `/scan`

### 6.3 启动 SLAM 建图

先确保底盘桥接和点云处理已经启动：

```bash
ros2 launch go2_core go2_base.launch.py
ros2 launch go2_perception go2_pointcloud_process.launch.py
```

然后启动 SLAM Toolbox：

```bash
ros2 launch go2_slam go2_slamtoolbox.launch.py
```

SLAM 使用的主要配置：

```text
map_frame:  map
odom_frame: odom
base_frame: base_link
scan_topic: /scan
mode:       mapping
```

SLAM 启动后，TF 树应为：

```text
map -> odom -> base_link
```

查看 TF：

```bash
ros2 run tf2_tools view_frames
```

或实时查看：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

保存地图：

```bash
mkdir -p ~/go2_maps

ros2 run nav2_map_server map_saver_cli -f ~/go2_maps/go2_map
```

保存后会生成：

```text
~/go2_maps/go2_map.yaml
~/go2_maps/go2_map.pgm
```

#### 6.3.1 一键建图、保存地图和固定起点

如果希望把“建图最后停下的位置”作为下一次使用已有地图导航的固定起点，可以使用项目提供的交互脚本。

运行：

```bash
cd ~/go2_ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash

ros2 run go2_navigation go2_slam_record_session.sh
```

脚本会自动启动：

```text
go2_base
go2_pointcloud_process
slam_toolbox
rviz2
```

> 如果板载机没有图形界面，或不想启动 RViz，可以这样运行：
>
> ```bash
> USE_RVIZ=false ros2 run go2_navigation go2_slam_record_session.sh
> ```

建图完成后，把机器狗停在以后要作为固定起点的位置和朝向，然后在脚本终端输入：

```text
save
```

脚本会保存：

```text
~/go2_maps/go2_map.yaml
~/go2_maps/go2_map.pgm
~/go2_maps/go2_initial_pose.yaml
~/go2_maps/go2_initial_pose.json
~/go2_maps/set_go2_initial_pose.sh
```

其中：

```text
go2_map.yaml / go2_map.pgm
```

是地图文件。

```text
go2_initial_pose.yaml
```

是建图结束时的 `map -> base_link` 位姿，下一次已有地图定位导航时会作为 AMCL 初始位姿。

```text
set_go2_initial_pose.sh
```

是发布固定初始位姿的快捷脚本。

这个保存器不依赖 `map_saver_cli`，而是直接订阅 `/map` 并写出 `.pgm/.yaml`，可以绕过部分板载机上 ImageMagick `SIGSEGV` 导致地图保存失败的问题。

也可以指定地图输出前缀：

```bash
ros2 run go2_navigation go2_slam_record_session.sh ~/go2_maps/lab_map
```

这会生成：

```text
~/go2_maps/lab_map.yaml
~/go2_maps/lab_map.pgm
~/go2_maps/go2_initial_pose.yaml
~/go2_maps/set_go2_initial_pose.sh
```

### 6.4 建图时启动 Navigation2

先启动底盘、感知和 SLAM：

```bash
ros2 launch go2_core go2_base.launch.py
ros2 launch go2_perception go2_pointcloud_process.launch.py
ros2 launch go2_slam go2_slamtoolbox.launch.py
```

然后启动 Nav2：

```bash
ros2 launch go2_navigation go2_nav2.launch.py
```

Nav2 的输入：

```text
/scan
/odom
/map
TF: map -> odom -> base_link
```

Nav2 的输出：

```text
/cmd_vel
```

`go2_base` 会接收 `/cmd_vel`，并转换成 Unitree 运动请求：

```text
/cmd_vel -> /api/sport/request
```

在 RViz 中使用导航：

1. 确认 Fixed Frame 设置为 `map`。
2. 确认能看到地图、机器人位姿、LaserScan、Global Costmap 和 Local Costmap。
3. 点击 RViz 顶部的 `Navigation2 Goal`。
4. 在地图中点击目标位置。
5. 拖动鼠标设置目标朝向。
6. 机器人开始规划并执行导航。

### 6.5 使用已保存地图进行定位导航

如果已经保存了地图：

```text
~/go2_maps/go2_map.yaml
~/go2_maps/go2_map.pgm
```

后续导航时不需要再启动 `go2_slam` 建图。应使用已有地图启动 `map_server + AMCL + Navigation2`，由 AMCL 根据 `/scan` 在地图中定位，并发布：

```text
map -> odom
```

底盘节点仍然负责发布：

```text
odom -> base_link
```

项目提供了定位导航启动文件：

```bash
ros2 launch go2_navigation go2_localization_nav.launch.py
```

默认会加载：

```text
~/go2_maps/go2_map.yaml
```

如果地图路径不同，可以通过 `map` 参数指定：

```bash
ros2 launch go2_navigation go2_localization_nav.launch.py map:=/home/unitree/go2_maps/go2_map.yaml
```

该 launch 会启动：

```text
go2_base
cloud_accumulation
pointcloud_to_laserscan_node
map_server
amcl
navigation2
rviz2
```

启动后，在 RViz 中执行定位和导航：

1. 确认 Fixed Frame 为 `map`。
2. 确认地图已经显示。
3. 点击 `2D Pose Estimate`，在地图上给机器人一个初始位姿。
4. 观察 LaserScan 是否和地图轮廓对齐。
5. 如果未对齐，重新设置初始位姿，或原地小幅旋转让 AMCL 收敛。
6. 点击 `Navigation2 Goal`，在地图上设置目标点。

定位导航时的关键检查命令：

```bash
timeout 3 ros2 topic echo /map
timeout 3 ros2 topic echo /scan
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

如果 `/map` 有数据、`/scan` 有数据，并且 `map -> odom -> base_link` 都存在，就可以进行基于已有地图的导航。

#### 自动全局定位

如果不想每次手动点击 `2D Pose Estimate`，可以让 AMCL 先执行全局定位，然后让机器人原地慢速旋转一段时间，利用多方向 `/scan` 和已有地图自动收敛。

启动时添加：

```bash
ros2 launch go2_navigation go2_localization_nav.launch.py \
  map:=/home/unitree/go2_maps/go2_map.yaml \
  auto_localize:=true
```

默认流程为：

```text
等待 8 秒，让 map_server、AMCL、Nav2、点云处理和 TF 启动
调用 /reinitialize_global_localization
以 0.2 rad/s 原地旋转 18 秒
发布零速度停车
```

可调参数：

```bash
ros2 launch go2_navigation go2_localization_nav.launch.py \
  map:=/home/unitree/go2_maps/go2_map.yaml \
  auto_localize:=true \
  auto_localize_start_delay:=8.0 \
  auto_localize_spin_duration:=18.0 \
  auto_localize_angular_speed:=0.2
```

自动定位完成后，观察 RViz：

- `/scan` 是否和地图边界基本重合。
- 机器人位姿是否在真实位置附近。
- `map -> odom -> base_link` 是否稳定。

检查 TF：

```bash
ros2 run tf2_ros tf2_echo map odom
```

如果自动定位后 `/scan` 和地图没有对齐，说明当前环境不适合纯 AMCL 全局定位，或者地图特征不够明显。这时仍建议使用 `2D Pose Estimate` 手动给初始位姿，或采用固定起点、AprilTag、UWB 等辅助定位方式。

> 注意：自动定位会让机器人原地旋转。测试前请确保机器人周围有足够空间，并保持急停手段可用。

> 注意：不要同时启动 `go2_slamtoolbox.launch.py` 和 `go2_localization_nav.launch.py`。SLAM 和 AMCL 都会尝试维护 `map -> odom`，同时运行会导致 TF 冲突。

#### 固定起点启动已有地图导航

如果已经通过 `go2_slam_record_session.sh` 保存了固定起点，下一次启动时按下面做：

1. 把机器狗放回建图结束时保存固定起点的位置。
2. 狗头朝向保持和当时一致。
3. 启动已有地图导航并自动发布初始位姿。

一键启动：

```bash
cd ~/go2_ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash

ros2 run go2_navigation go2_start_saved_map_nav.sh
```

默认使用：

```text
~/go2_maps/go2_map.yaml
~/go2_maps/go2_initial_pose.yaml
```

如果地图文件名不同，可以指定地图：

```bash
ros2 run go2_navigation go2_start_saved_map_nav.sh /home/unitree/go2_maps/lab_map.yaml
```

如果只想手动启动定位导航，也可以先启动：

```bash
ros2 launch go2_navigation go2_localization_nav.launch.py \
  map:=/home/unitree/go2_maps/go2_map.yaml \
  auto_localize:=false
```

等 AMCL 启动后，另开终端发布固定起点：

```bash
~/go2_maps/set_go2_initial_pose.sh
```

或者直接运行：

```bash
ros2 run go2_navigation publish_initial_pose.py \
  --pose-file ~/go2_maps/go2_initial_pose.yaml \
  --repeat 5 \
  --period 0.5
```

发布后在 RViz 中检查：

- `/scan` 是否和地图边界基本重合。
- `ros2 run tf2_ros tf2_echo map base_link` 是否有连续输出。
- 先用 `Nav2 Goal` 点前方 0.5 到 1 米的空地测试。

固定起点法要求每次启动前机器狗的位置和朝向尽量一致。如果位置差太远，或朝向差几十度以上，AMCL 仍可能定位错误。

### 6.6 Web 图形化状态面板

项目提供了一个轻量 Web Dashboard，可以在浏览器里查看当前状态，并执行常用操作。

启动：

```bash
cd ~/go2_ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash

ros2 run go2_navigation go2_dashboard.py
```

默认监听：

```text
0.0.0.0:8080
```

如果在板载机本机浏览器访问：

```text
http://localhost:8080
```

如果在电脑浏览器访问板载机：

```text
http://192.168.123.18:8080
```

其中 `192.168.123.18` 换成你的板载机 IP。

Dashboard 会显示：

```text
当前模式：SLAM 建图 / 已有地图定位导航 / 空闲
当前动作：停止 / 前进 / 后退 / 左转 / 右转 / 前进 + 左转
/cmd_vel linear.x 和 angular.z
/map 状态
/scan 状态
/plan 状态
map -> base_link TF 状态
slam_toolbox / map_server / amcl / planner_server / controller_server / bt_navigator 节点状态
```

Dashboard 上有三个按钮：

```text
Save Map + Initial Pose
Publish Saved Initial Pose
Stop Robot
```

`Save Map + Initial Pose` 用于 SLAM 建图模式。建图完成后，把机器狗停在下次要复用的固定起点，点击这个按钮，会保存：

```text
~/go2_maps/go2_map.yaml
~/go2_maps/go2_map.pgm
~/go2_maps/go2_initial_pose.yaml
~/go2_maps/go2_initial_pose.json
~/go2_maps/set_go2_initial_pose.sh
```

这个按钮和 `go2_slam_record_session.sh` 输入 `save` 的效果一致。

`Publish Saved Initial Pose` 用于已有地图定位导航模式。启动 AMCL 后点击它，会读取：

```text
~/go2_maps/go2_initial_pose.yaml
```

并向 `/initialpose` 连续发布固定初始位姿。

`Stop Robot` 会向 `/cmd_vel` 发布零速度，用于测试时立即停下运动命令。

如果端口 8080 被占用，可以换端口：

```bash
ros2 run go2_navigation go2_dashboard.py --port 8090
```

如果地图或起点文件想用其他路径：

```bash
ros2 run go2_navigation go2_dashboard.py \
  --map-prefix ~/go2_maps/lab_map \
  --pose-file ~/go2_maps/lab_initial_pose.yaml \
  --initialpose-script ~/go2_maps/set_lab_initial_pose.sh
```

Dashboard 不依赖 RViz，也不需要 OpenGL；如果 RViz 在远程桌面里出现 GLSL/OpenGL 报错，仍然可以使用这个网页面板保存地图和固定起点。

## 7. 一键完整运行流程

推荐真机测试流程：

```bash
cd ~/go2_ros2_ws
source install/setup.bash
ros2 launch go2_core go2_startup.launch.py
```

启动后检查关键话题：

```bash
ros2 topic list
```

应该能看到类似话题：

```text
/utlidar/robot_pose
/utlidar/cloud_deskewed
/trans_cloud
/scan
/odom
/cmd_vel
/api/sport/request
/map
```

检查关键 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

如果 `/scan`、`/odom` 和 TF 都正常，通常就可以在 RViz 中进行建图和导航。

## 8. 视频流使用

默认完整启动 `go2_startup.launch.py` 时会启用视频节点。

视频节点读取 Go2 前视相机的 H264 UDP 多播流，并发布 ROS Image：

```text
/camera/image_raw
```

单独启动底盘和视频：

```bash
ros2 launch go2_core go2_base.launch.py video_enable:=true
```

可选参数：

```bash
ros2 launch go2_core go2_base.launch.py \
  video_enable:=true \
  image_topic:=/camera/image_raw \
  tcp_enable:=true \
  tcp_host:=127.0.0.1 \
  tcp_port:=5432 \
  target_fps:=30
```

查看图像话题：

```bash
ros2 topic echo /camera/image_raw
```

使用 `rqt_image_view` 查看图像：

```bash
ros2 run rqt_image_view rqt_image_view
```

然后选择：

```text
/camera/image_raw
```

## 9. 主要话题说明

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/utlidar/robot_pose` | `geometry_msgs/PoseStamped` | Go2 原生机器人位姿 |
| `/utlidar/cloud_deskewed` | `sensor_msgs/PointCloud2` | Go2 原生去畸变点云 |
| `/trans_cloud` | `sensor_msgs/PointCloud2` | 累积和高度过滤后的点云 |
| `/scan` | `sensor_msgs/LaserScan` | 由点云转换得到的 2D 激光扫描 |
| `/odom` | `nav_msgs/Odometry` | 机器人里程计 |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 或手动控制输出的速度命令 |
| `/api/sport/request` | `unitree_api/Request` | 发送给 Unitree Sport API 的运动请求 |
| `/camera/image_raw` | `sensor_msgs/Image` | 前视相机图像 |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM 生成的地图 |

## 10. 坐标系说明

| 坐标系 | 来源 | 说明 |
| --- | --- | --- |
| `map` | SLAM Toolbox | 全局地图坐标系 |
| `odom` | Go2 位姿桥接 | 局部里程计坐标系 |
| `base_link` | Go2 底盘桥接 | 机器人本体坐标系 |

完整导航 TF 链路：

```text
map -> odom -> base_link
```

其中：

- `odom -> base_link` 由 `go2_base` 发布。
- `map -> odom` 由 `slam_toolbox` 发布。

## 11. 常用调试命令

查看所有话题：

```bash
ros2 topic list
```

查看所有节点：

```bash
ros2 node list
```

查看话题频率：

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /utlidar/cloud_deskewed
```

查看话题类型：

```bash
ros2 topic info /scan
ros2 topic info /cmd_vel
ros2 topic info /api/sport/request
```

查看 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

生成 TF 树 PDF：

```bash
ros2 run tf2_tools view_frames
```

查看 Nav2 action：

```bash
ros2 action list
ros2 action info /navigate_to_pose
```

查看当前生命周期节点状态：

```bash
ros2 lifecycle nodes
```

## 12. RViz 配置建议

RViz 的 Fixed Frame 建议设置为：

```text
map
```

建议添加以下显示项：

| RViz Display | 话题或配置 |
| --- | --- |
| `TF` | 显示 TF 树 |
| `Map` | `/map` |
| `LaserScan` | `/scan` |
| `PointCloud2` | `/trans_cloud` |
| `Odometry` | `/odom` |
| `Path` | Nav2 global path |
| `Costmap` | local/global costmap |
| `Image` | `/camera/image_raw` |

导航时使用 RViz 顶部工具栏中的：

```text
Navigation2 Goal
```

## 13. 参数调节建议

### 13.1 点云高度过滤

点云累积节点会保留一定高度范围内的点。当前源码中主要保留：

```text
0.2m <= z <= 1.0m
```

如果环境中低矮障碍物较多，可以适当降低过滤下限。

### 13.2 LaserScan 参数

`go2_perception/launch/go2_pointcloud_process.launch.py` 中可调：

```yaml
min_height: -0.1
max_height: 1.0
range_min: 0.10
range_max: 20.0
angle_min: -3.14
angle_max: 3.14
angle_increment: 0.0087
```

如果 `/scan` 过密导致计算压力较大，可以增大 `angle_increment`。

### 13.3 Nav2 速度参数

`go2_navigation/config/nav2_params.yaml` 中 DWB 控制器速度参数较激进：

```yaml
max_vel_x: 3.0
max_vel_theta: 1.5
acc_lim_x: 10.0
acc_lim_theta: 5.0
```

室内初次测试建议降低，例如：

```yaml
max_vel_x: 0.5
max_vel_theta: 0.8
acc_lim_x: 1.0
acc_lim_theta: 1.5
```

确认建图、避障、局部规划稳定后，再逐步提高速度。

## 14. 常见问题

### 14.1 看不到 `/scan`

检查原始点云是否存在：

```bash
ros2 topic echo /utlidar/cloud_deskewed
```

检查点云处理节点是否启动：

```bash
ros2 node list
```

检查 `/trans_cloud` 是否存在：

```bash
ros2 topic echo /trans_cloud
```

如果 `/utlidar/cloud_deskewed` 不存在，说明 Go2 原生雷达数据没有进入 ROS 2，需要优先检查 Unitree 官方 ROS 2 环境和 DDS 配置。

### 14.2 RViz 中机器人位置不动

检查 `/utlidar/robot_pose`：

```bash
ros2 topic echo /utlidar/robot_pose
```

检查 `/odom`：

```bash
ros2 topic echo /odom
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

如果 `/utlidar/robot_pose` 正常但 `/odom` 没有输出，说明 `go2_base` 节点没有正常运行。

### 14.3 Nav2 不执行目标点

检查 Nav2 action 是否存在：

```bash
ros2 action list | grep navigate
```

检查 `/cmd_vel` 是否有输出：

```bash
ros2 topic echo /cmd_vel
```

如果 `/cmd_vel` 没有输出，通常是 Nav2 没有成功激活、TF 不完整、地图不可用或 costmap 报错。

如果 `/cmd_vel` 有输出但机器人不动，检查 Unitree 请求是否发出：

```bash
ros2 topic echo /api/sport/request
```

### 14.4 地图漂移或建图效果差

建议检查：

- `/scan` 是否稳定。
- `odom -> base_link` 是否连续。
- 机器人运动速度是否过快。
- 点云累积帧数是否导致拖影。
- 环境中是否存在大量动态物体。
- SLAM Toolbox 参数是否适合当前场地。

初次建图建议慢速移动机器人，避免急转和快速平移。

### 14.5 视频流打不开

检查网络接口是否为 `eth0`。视频节点默认使用：

```text
multicast-iface=eth0
```

如果实际网卡不是 `eth0`，需要修改视频节点中的 GStreamer 字符串。

检查 GStreamer 是否能工作：

```bash
gst-inspect-1.0
```

检查 OpenCV 是否支持 GStreamer：

```bash
python3 - <<EOF
import cv2
print(cv2.getBuildInformation())
EOF
```

### 14.6 `apt update` 出现 ROS key 过期或 404

如果在 Go2 板载机安装依赖时看到类似错误：

```text
EXPKEYSIG F42ED6FBAB17C654 Open Robotics
404 Not Found ros-foxy-xxx
NO_PUBKEY FB0B24895113F120
```

通常不是依赖包名称写错，而是 apt 源或签名 key 出了问题：

- `EXPKEYSIG F42ED6FBAB17C654`：ROS/ROS2 apt 签名 key 过期或本机 key 太旧。
- `404 Not Found`：当前镜像源的 apt 索引和实际包文件不同步，常见于镜像站同步滞后。
- `NO_PUBKEY FB0B24895113F120`：RealSense apt 源缺少公钥。本项目不依赖 RealSense，可以先禁用该源。

建议按下面顺序处理。

先安装基础工具：

```bash
sudo apt install -y curl gnupg ca-certificates lsb-release
```

查找并临时禁用 RealSense 源：

```bash
grep -R "librealsense\\|realsense" -n /etc/apt/sources.list /etc/apt/sources.list.d || true
```

如果输出中有类似 `/etc/apt/sources.list.d/librealsense.list`，执行：

```bash
sudo mv /etc/apt/sources.list.d/librealsense.list /etc/apt/sources.list.d/librealsense.list.disabled
```

如果文件名不同，请按实际文件名修改。

更新 ROS2 apt key：

```bash
sudo mkdir -p /usr/share/keyrings

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
```

将 ROS2 源切到官方源：

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros2.list
```

如果不需要 ROS1，可以临时禁用 ROS1 源，避免 ROS1 源继续报签名错误：

```bash
grep -R "ros/ubuntu" -n /etc/apt/sources.list /etc/apt/sources.list.d || true
```

如果输出中有类似 `/etc/apt/sources.list.d/ros-latest.list`，执行：

```bash
sudo mv /etc/apt/sources.list.d/ros-latest.list /etc/apt/sources.list.d/ros-latest.list.disabled
```

清空旧缓存并重新更新：

```bash
sudo apt clean
sudo rm -rf /var/lib/apt/lists/*
sudo apt update
```

如果 `sudo apt update` 不再报签名错误和 404，再重新安装依赖：

```bash
sudo apt install -y \
  ros-foxy-navigation2 \
  ros-foxy-nav2-bringup \
  ros-foxy-pcl-ros \
  ros-foxy-tf-transformations \
  ros-foxy-slam-toolbox \
  ros-foxy-cv-bridge \
  ros-foxy-tf2-ros \
  ros-foxy-tf2-sensor-msgs \
  ros-foxy-message-filters \
  ros-foxy-laser-geometry
```

## 15. 推荐测试顺序

第一次部署时，不建议直接完整启动导航。推荐按以下顺序验证：

1. 验证 Unitree 官方 ROS 2 环境。
2. 验证 `/utlidar/robot_pose` 和 `/utlidar/cloud_deskewed`。
3. 启动 `go2_base`，检查 `/odom` 和 `odom -> base_link`。
4. 启动点云处理，检查 `/trans_cloud` 和 `/scan`。
5. 启动 SLAM，检查 `/map` 和 `map -> odom`。
6. 在 RViz 中确认地图和机器人位置正常。
7. 启动 Nav2，先设置很近的目标点。
8. 确认 `/cmd_vel` 和 `/api/sport/request` 正常。
9. 低速、小范围测试导航。
10. 调整 Nav2 速度、costmap、SLAM 参数。

## 16. 安全注意事项

- 首次测试请抬高机器人或保证周围有足够空间。
- 室内初次导航时建议降低 Nav2 最大速度。
- 保持急停手段可用。
- 不要在人员密集、障碍复杂或地面不平环境中直接高速导航。
- 如果 `/cmd_vel` 异常持续输出，请立即停止相关节点或发送零速度命令。
- 建图和导航前，确认 TF 树稳定且无明显跳变。

## 17. 核心数据流总结

完整数据流如下：

```text
Go2 原生位姿
/utlidar/robot_pose
        |
        v
go2_base
        |
        +----> /odom
        +----> TF: odom -> base_link


Go2 原生点云
/utlidar/cloud_deskewed
        |
        v
cloud_accumulation
        |
        v
/trans_cloud
        |
        v
pointcloud_to_laserscan_node
        |
        v
/scan
        |
        +----> slam_toolbox
        |          |
        |          +----> /map
        |          +----> TF: map -> odom
        |
        +----> Navigation2 costmap


Navigation2
        |
        v
/cmd_vel
        |
        v
go2_base
        |
        v
/api/sport/request
        |
        v
Unitree Go2 运动控制
```

## 18. 许可证

本项目采用 MIT License。详情请查看仓库中的 `LICENSE` 文件。

## 19. 致谢

感谢：

- Unitree Robotics 提供 Go2 EDU 平台。
- ROS 2 社区提供 Navigation2、SLAM Toolbox、TF2 等基础组件。
- 本项目作者和贡献者对 Go2 ROS 2 自主导航工具链的整理与适配。
