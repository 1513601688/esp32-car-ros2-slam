# ESP32 全向轮小车 ROS 2 SLAM 与导航系统

本仓库汇总了一套经过实车验证的四轮全向小车工程，包括：

- ESP32-S3 底盘固件、Web 前端和图形化烧录工具。
- 开发板端底盘控制、轮式里程计、IMU、EKF、TF 和 LD14 雷达驱动。
- VMware 虚拟机端 Slam Toolbox、AMCL、Nav2 和 RViz。

系统已经可以完成全向移动、二维激光建图、保存和继续扩建地图、静态地图定位及
Nav2 自动导航。

## 一、系统架构

```text
Windows 电脑
└─ ESP-IDF 固件、Web 前端、烧录 UI
          │ USB 烧录
          ▼
ESP32-S3
├─ 电机与编码器
├─ JY901 IMU
├─ 全向底盘运动学
└─ Wi-Fi Web 页面
          │ UART：速度命令、WHEEL、IMU
          ▼
车载开发板（ROS 2 Rolling）
├─ ESP32 串口桥
├─ /wheel/odom、/imu/data
├─ robot_localization → /odom
├─ 车体与传感器 TF
└─ LD14 → /scan
          │ ROS 2 DDS 局域网
          ▼
VMware Ubuntu 24.04（ROS 2 Jazzy）
├─ /scan → /scan_slam
├─ Slam Toolbox 建图
├─ Map Server + AMCL 定位
└─ Nav2 + RViz 导航与显示
```

ESP32 不直接运行 ROS 2。它通过串口连接开发板；ESP32 的 Wi-Fi 只用于访问本机
Web 页面。真正的 ROS 2 分布式通信发生在开发板和虚拟机之间。

ROS 2 基于 DDS，不需要 ROS Master。只要开发板与虚拟机满足以下条件，就能自动
发现并共享话题：

- 位于同一局域网，VMware 使用桥接网络。
- 两端 `ROS_DOMAIN_ID` 相同，默认均为 `0`。
- 防火墙和路由器没有阻止 DDS 组播。
- 两端系统时间保持同步。

开发板使用 Rolling、虚拟机使用 Jazzy。跨机只传输标准 ROS 2 消息和 TF，并已
实测互通；SLAM、AMCL 和 Nav2 全部在 Jazzy 虚拟机本机运行。

## 二、数据与控制链路

### 传感器链路

```text
ESP32 WHEEL ──→ /wheel/odom ─┐
                             ├─ EKF ─→ /odom + odom→base_footprint
ESP32 IMU   ──→ /imu/data ───┘

LD14 ──→ /scan ──→ 固定360点 /scan_slam ──→ SLAM / AMCL / Nav2
```

EKF 当前只融合：

- 已校准的轮式 `vx、vy`。
- IMU 的 Z 轴角速度 `gz`。

轮式 `wz`、JY901 绝对 Yaw 和 IMU 线加速度暂不参与融合，这是根据实车标定结果
做出的选择。

### 导航控制链路

```text
Nav2 Goal
  → NavFn A* 全局规划
  → Rotation Shim + Omni MPPI 局部控制
  → 低速死区补偿
  → 速度平滑
  → 碰撞监控
  → /cmd_vel
  → 开发板
  → ESP32
```

当前导航配置：

| 功能 | 实现 |
|---|---|
| 定位 | AMCL OmniMotionModel |
| 全局规划 | NavFn，启用 A* |
| 局部控制 | Rotation Shim + Omni MPPI |
| 障碍处理 | Static、Obstacle、Inflation Layer |
| 安全控制 | Collision Monitor + `/cmd_vel` 超时停车 |

MPPI 使用 `Omni` 模型，可以同时输出 `vx、vy、wz`，支持全向轮横移和斜向移动。

## 三、TF 与车体尺寸

```text
map
└── odom
    └── base_footprint          # 车体中心在地面的投影
        └── base_link           # 中间层中心，离地 0.11 m
            ├── imu_link        # 右侧 5 cm，绝对高度 0.17 m
            └── laser_frame     # 中心上方，绝对高度 0.18 m，Yaw=180°
```

坐标约定：

- X 指向车头。
- Y 指向左侧。
- Z 垂直向上。
- 绕 Z 轴逆时针为正。

车体为 `0.30 m × 0.30 m` 的正方形，Nav2 footprint 以
`base_footprint` 为中心，padding 为 `0.02 m`，障碍物膨胀半径为 `0.30 m`。

主要 ROS 2 话题：

| 话题 | 用途 |
|---|---|
| `/cmd_vel` | 最终底盘速度命令 |
| `/wheel/odom` | ESP32 轮式速度 |
| `/imu/data` | ESP32 IMU 数据 |
| `/odom` | EKF 融合里程计 |
| `/scan` | LD14 原始扫描 |
| `/scan_slam` | 固定 360 点扫描 |
| `/map` | SLAM 或 Map Server 地图 |
| `/tf`、`/tf_static` | 动态和静态坐标变换 |

## 四、仓库目录

```text
.
├── esp32_firmware/                 # ESP32 固件、Web 前端和烧录 UI
├── board_control/                  # 开发板 ROS 2 控制与 LD14 驱动
├── vm_slam_nav2/                   # 虚拟机 SLAM、AMCL、Nav2 和地图
├── SOURCE_MANIFEST.md              # 三端源码来源
└── README.md
```

常用入口：

| 功能 | 位置 |
|---|---|
| 图形化烧录 | `esp32_firmware/flash_gui.bat` |
| ESP32 串口协议 | `esp32_firmware/docs/ros串口协议.md` |
| 开发板底盘节点 | `board_control/control_node/src/esp32_car_node.cpp` |
| EKF 参数 | `board_control/control_node/config/ekf.yaml` |
| SLAM 参数 | `vm_slam_nav2/src/esp32_car_slam/config/slam_toolbox.yaml` |
| Nav2 参数 | `vm_slam_nav2/src/esp32_car_slam/config/nav2_params.yaml` |
| 虚拟机详细指南 | `vm_slam_nav2/SLAM启动指南.md` |
| 完整实测记录 | `board_control/control_node/PROJECT_PROGRESS.md` |

当前地图：

```text
vm_slam_nav2/maps/aaaaccc.yaml       # AMCL/Nav2 地图配置
vm_slam_nav2/maps/aaaaccc.pgm        # 静态栅格地图
vm_slam_nav2/maps/aaaaccc.posegraph  # 可继续建图的位姿图
vm_slam_nav2/maps/aaaaccc.data       # 位姿图扫描数据
```

## 五、操作流程

### 1. 烧录 ESP32

Windows 运行：

```text
esp32_firmware/flash_gui.bat
```

在 UI 中选择 COM 口并填写 Wi-Fi 名称和密码。烧录后前端地址为：

```text
http://esp32car.local/
```

如果 mDNS 不可用，使用烧录 UI 或串口日志显示的实际 IP：

```text
http://<ESP32实际IP>/
```

### 2. 启动开发板基础节点

终端 1：底盘、轮式里程计、IMU 和 EKF。

```bash
source /opt/ros/rolling/setup.bash
source ~/esp32_car_project/ldlidar_ros2_ws/install/setup.bash
source ~/esp32_car_project/control_node/install/setup.bash
ros2 launch esp32_car_control localization.launch.py
```

终端 2：LD14 雷达。

```bash
source /opt/ros/rolling/setup.bash
source ~/esp32_car_project/ldlidar_ros2_ws/install/setup.bash
source ~/esp32_car_project/control_node/install/setup.bash
ros2 launch esp32_car_control lidar.launch.py
```

检查频率：

```bash
ros2 topic hz /odom    # 约 50 Hz
ros2 topic hz /scan    # 约 6 Hz
```

### 3. 新建地图

开发板启动本地键盘遥控：

```bash
source /opt/ros/rolling/setup.bash
source ~/esp32_car_project/control_node/install/setup.bash
ros2 run esp32_car_control mapping_teleop_node
```

虚拟机启动建图：

```bash
cd ~/桌面/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch esp32_car_slam mapping.launch.py use_rviz:=true
```

建图时低速行驶并尽量形成闭合路线。实时雷达点应与已生成的墙线基本重合。

### 4. 保存地图

停车等待约 3 秒，不要先关闭建图节点。另开虚拟机终端执行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/桌面/ros2_ws/install/setup.bash

MAP_NAME=room_$(date +%Y%m%d_%H%M)

ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$HOME/桌面/ros2_ws/maps/${MAP_NAME}'}"

ros2 run esp32_car_slam save_map.py \
  --output "$HOME/桌面/ros2_ws/maps/${MAP_NAME}"
```

应生成 `.yaml`、`.pgm`、`.posegraph` 和 `.data` 四个文件。

### 5. 继续扩建旧地图

先启动 `mapping.launch.py`，车辆保持静止，然后加载旧位姿图：

```bash
source /opt/ros/jazzy/setup.bash
source ~/桌面/ros2_ws/install/setup.bash

ros2 service call /slam_toolbox/deserialize_map \
  slam_toolbox/srv/DeserializePoseGraph \
  "{filename: '/home/luohjj/桌面/ros2_ws/maps/aaaaccc', match_type: 1, initial_pose: {x: 0.0, y: 0.0, theta: 0.0}}"
```

旧地图出现且雷达点重合后再动车，完成后另存为新名称。

### 6. 启动 Nav2 导航

先停止建图遥控和 Slam Toolbox，避免多个 `/cmd_vel` 或 `map -> odom` 发布者：

```bash
pkill -TERM -x mapping_teleop_node
```

虚拟机启动：

```bash
cd ~/桌面/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch esp32_car_slam nav2.launch.py \
  map:=$HOME/桌面/ros2_ws/maps/aaaaccc.yaml \
  start_rviz:=true
```

RViz 中依次操作：

1. 使用 `2D Pose Estimate` 设置车辆真实位置和车头方向。
2. 确认雷达点与静态地图基本重合。
3. 在 Navigation 2 面板点击 `Startup`。
4. 等待 Localization 和 Navigation 均为 `active`。
5. 使用 `Nav2 Goal` 设置目标位置和最终朝向。

人工搬动车辆或定位错位后，必须重新设置 `2D Pose Estimate`。首次测试应选择空旷
区域和短距离目标，并保留紧急断电手段。
