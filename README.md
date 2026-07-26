# ESP32 全向轮小车 ROS 2 SLAM 与导航系统

本仓库是一套已经通过实车验证的四轮全向移动机器人工程，覆盖 ESP32-S3
下位机固件、开发板端硬件驱动与状态估计，以及 VMware Ubuntu 虚拟机端的
二维激光 SLAM、AMCL 定位和 Nav2 自主导航。

项目采用分层、分布式架构：ESP32-S3 负责接近硬件的实时任务，车载开发板负责
把专用串口协议转换为标准 ROS 2 接口，虚拟机负责计算量较大的建图、定位、
规划、控制和 RViz 可视化。三部分源码在同一仓库中保持版本对应，便于部署、
备份、回滚和继续开发。

## 项目能力

- 四轮全向底盘前进、后退、左右平移、斜向移动和原地旋转。
- ESP32-S3 独立上报轮式速度与 JY901 IMU 数据。
- `robot_localization` 融合已校准的轮式 `vx/vy` 和 IMU `gz`。
- 发布标准 `/odom`、`/scan`、`/imu/data`、`/wheel/odom` 和完整 TF。
- LDROBOT LD14 二维激光雷达采集与固定 360 点扫描归一化。
- Slam Toolbox 新建地图、保存地图和基于 PoseGraph 继续扩建旧地图。
- Map Server + AMCL 在静态地图中定位。
- Nav2 全局路径规划、全向局部控制、速度平滑和碰撞监控。
- Windows 图形化烧录工具，可选择 Wi-Fi、密码和 COM 口并显示烧录进度。
- ESP32 Web 页面实时显示底盘、电机、IMU 和里程计状态。

## 一、系统总体架构

```text
┌─────────────────────────────────────────────────────────────────────┐
│ Windows 开发电脑                                                     │
│ ESP-IDF 源码 + Web 前端 + 图形化烧录工具                             │
└───────────────────────────┬─────────────────────────────────────────┘
                            │ USB/JTAG/UART（编译与烧录）
                            ▼
┌─────────────────────────────────────────────────────────────────────┐
│ ESP32-S3 下位机                                                      │
│ 电机控制 │ 编码器解算 │ JY901 采集 │ Web Server │ Wi-Fi/mDNS         │
└───────────────┬─────────────────────────────────────┬───────────────┘
                │ UART 专用串口协议                    │ HTTP/WebSocket
                │ WHEEL / IMU / cmd_vel                │ 仅用于 Web 页面
                ▼                                      ▼
┌───────────────────────────────────────┐      手机或电脑浏览器
│ 车载开发板：ROS 2 Rolling             │
│ 串口桥接 │ 传感器发布 │ EKF │ TF      │
│ LD14 驱动 │ 建图期间的本地键盘遥控     │
└──────────────────┬────────────────────┘
                   │ ROS 2 DDS（局域网）
                   │ /odom /scan /tf /cmd_vel ...
                   ▼
┌─────────────────────────────────────────────────────────────────────┐
│ VMware Ubuntu 24.04：ROS 2 Jazzy                                    │
│ 扫描归一化 │ Slam Toolbox │ Map Server │ AMCL │ Nav2 │ RViz          │
└─────────────────────────────────────────────────────────────────────┘
```

### 1. ESP32-S3：实时硬件层

ESP32-S3 不直接加入 ROS 2 DDS 网络，而是通过串口与车载开发板通信。这样可以
让电机时序、编码器轮询和 IMU 采集保持在下位机侧，不受虚拟机负载或无线网络
抖动影响。

ESP32-S3 的主要职责：

- 接收开发板转换后的 `vx/vy/wz` 速度控制帧。
- 完成四轮全向底盘运动学映射并控制电机。
- 读取编码器并计算已经过实车校准的底盘 `vx/vy/wz`。
- 以独立数据帧发送轮式速度和 IMU 原始测量。
- 连接用户在烧录 UI 中设置的 Wi-Fi。
- 提供 HTTP/WebSocket 前端，支持 `http://esp32car.local` 和实际 DHCP IP。

ESP32 的 Wi-Fi 页面是监控与调试入口，不是本项目 ROS 2 数据传输链路的一部分。
即使浏览器没有打开，只要 ESP32 与开发板串口正常，ROS 2 控制与传感器上报仍可
独立工作。

### 2. 车载开发板：ROS 2 硬件抽象层

车载开发板运行 ROS 2 Rolling，靠近底盘和传感器，是整车 ROS 2 图中的硬件
入口。它把 ESP32 专用串口数据转换为标准 ROS 2 消息，并向局域网中的其他
ROS 2 主机发布。

开发板端的主要职责：

- `esp32_car_node`：管理 ESP32 串口、订阅 `/cmd_vel`、下发速度帧和执行
  失联停车看门狗。
- `sensor_publisher`：发布 `/wheel/odom`、`/imu/data` 和诊断话题。
- `robot_localization`：融合轮式 `vx/vy` 与 IMU `gz`，输出 `/odom` 和
  `odom -> base_footprint`。
- 静态 TF：描述 `base_footprint`、`base_link`、`imu_link` 与
  `laser_frame` 的真实安装关系。
- LD14 驱动：通过 USB 转串口读取雷达并发布 `/scan`。
- `mapping_teleop_node`：仅在建图时从开发板 SSH 终端读取键盘并发布
  `/cmd_vel`，避免控制操作依赖虚拟机显示延迟。

### 3. VMware 虚拟机：SLAM、定位与导航层

虚拟机运行 Ubuntu 24.04 和 ROS 2 Jazzy，不直接访问 ESP32 或雷达串口。它通过
ROS 2 DDS 订阅开发板发布的标准话题，因此虚拟机端工程与具体串口设备路径解耦。

虚拟机端的主要职责：

- 将 LD14 每圈约 390～393 个不等长点重采样为固定 360 点的 `/scan_slam`。
- 使用 Slam Toolbox 消费 `/scan_slam`、`/odom` 和 TF 完成二维建图。
- 使用 Map Server 加载 `.yaml/.pgm` 静态栅格地图。
- 使用适配全向底盘的 AMCL 运动模型估计地图坐标中的车辆位姿。
- 使用 Nav2 生成全局路径、全向局部速度并执行安全过滤。
- 运行 RViz，显示静态地图、实时雷达、TF、代价地图、规划路径与机器人轮廓。

## 二、ROS 2 分布式架构

### 1. 为什么开发板和虚拟机可以直接共享话题

ROS 2 基于 DDS，无需像 ROS 1 一样单独运行 ROS Master。开发板和虚拟机位于
同一局域网、使用相同 `ROS_DOMAIN_ID` 且网络允许 DDS 发现与数据包通过时，
双方节点会自动发现彼此：

```text
开发板发布 /scan、/odom、/tf
              │
              │ DDS 自动发现和点对点传输
              ▼
虚拟机直接订阅，无需 SSH 转发、串口映射或中间服务器
```

当前实车部署中，开发板使用 ROS 2 Rolling，虚拟机使用 ROS 2 Jazzy。项目跨机
共享的是 `sensor_msgs`、`nav_msgs`、`geometry_msgs` 和 TF 等标准接口，已经
实测互通。跨发行版并不是所有 ROS 包都保证二进制兼容，因此 Slam Toolbox、
AMCL 和 Nav2 均只在 Jazzy 虚拟机本机运行；开发板只提供稳定的标准数据接口。

### 2. 网络要求

- VMware 网卡使用桥接模式，使虚拟机获得与开发板同网段的独立 IP。
- 两端的 `ROS_DOMAIN_ID` 必须相同；未设置时都使用默认域 `0`。
- 防火墙、AP 隔离和路由器访客网络不能阻断 DDS 组播发现或节点间通信。
- 开发板与虚拟机都应同步系统时间，避免雷达消息因时间戳异常被 TF 过滤器丢弃。
- ROS 2 话题发现异常时，应同时检查网络、ROS daemon 和实际话题数据，不能只看
  `ros2 node list`。

### 3. 计算任务为什么这样分配

| 层级 | 优先目标 | 放置的任务 |
|---|---|---|
| ESP32-S3 | 实时性和硬件确定性 | 电机、编码器、IMU、串口协议、Web 服务 |
| 车载开发板 | 硬件常驻和低延迟控制 | ROS 串口桥、雷达驱动、EKF、TF、建图遥控 |
| VMware 虚拟机 | 算力和可视化 | SLAM、AMCL、Nav2、代价地图、RViz |

建图时由开发板本地键盘控制车辆，虚拟机只做建图和显示。导航时由虚拟机 Nav2
发布 `/cmd_vel`，开发板负责把命令可靠地送到 ESP32。两种模式不能同时存在多个
控制发布者。

## 三、数据链路

### 1. 传感器反馈链

```text
编码器
  └─ ESP32: WHEEL,<stamp_ms>,<vx>,<vy>,<wz>
       └─ 开发板: /wheel/odom
            ┐
            ├─ robot_localization EKF
            │      ├─ /odom（约 50 Hz）
            │      └─ odom -> base_footprint
            │
JY901 IMU   │
  └─ ESP32: IMU,<stamp_ms>,<ax>,...,<gz>,...,<yaw>
       └─ 开发板: /imu/data
            ┘

LD14（USB 转串口）
  └─ 开发板: /scan（约 6 Hz，每圈点数可变）
       └─ 虚拟机 scan_normalizer
            └─ /scan_slam（固定 360 点）
                 ├─ Slam Toolbox
                 ├─ AMCL
                 └─ Nav2 obstacle layer / collision monitor
```

当前 EKF 只融合：

- `/wheel/odom` 中的 `vx`、`vy`。
- `/imu/data` 中绕 Z 轴的角速度 `gz`。

轮式 `wz`、JY901 绝对 Yaw 和 IMU 线加速度目前不参与融合。这是根据实车标定结果
做出的选择：轮式旋转速度不可靠，而绝对 Yaw 容易受车体电磁环境影响。

### 2. 导航控制链

```text
RViz Nav2 Goal
  └─ BT Navigator
       ├─ NavFn 全局规划器（A*）
       ├─ Simple Smoother
       └─ Rotation Shim + MPPI 局部控制器（Omni）
            └─ /cmd_vel_nav_raw
                 └─ 全向轮低速死区补偿
                      └─ /cmd_vel_nav
                           └─ Velocity Smoother
                                └─ /cmd_vel_smoothed
                                     └─ Collision Monitor
                                          └─ /cmd_vel
                                               └─ 开发板串口桥
                                                    └─ ESP32
```

MPPI 使用 `Omni` 运动模型，可同时产生 `vx`、`vy` 和 `wz`，因此能够利用全向轮
底盘的横移能力。Rotation Shim 负责起步和终点附近的朝向处理，避免车辆到达目标
点后持续绕圈。

## 四、ROS 2 接口

| 名称 | 类型 | 主要发布端 | 主要用途 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 建图遥控或 Nav2 碰撞监控 | 底盘最终速度命令 |
| `/wheel/odom` | `nav_msgs/Odometry` | 开发板串口桥 | 轮式 `vx/vy/wz` 测量 |
| `/imu/data` | `sensor_msgs/Imu` | 开发板串口桥 | IMU 角速度与加速度 |
| `/odom` | `nav_msgs/Odometry` | 开发板 EKF | 融合后的局部里程计 |
| `/scan` | `sensor_msgs/LaserScan` | 开发板 LD14 驱动 | 原始二维扫描 |
| `/scan_slam` | `sensor_msgs/LaserScan` | 虚拟机归一化节点 | 固定 360 点扫描 |
| `/map` | `nav_msgs/OccupancyGrid` | Slam Toolbox 或 Map Server | 建图或静态地图 |
| `/tf` | `tf2_msgs/TFMessage` | EKF、SLAM 或 AMCL | 动态坐标变换 |
| `/tf_static` | `tf2_msgs/TFMessage` | 开发板静态 TF 节点 | 传感器安装关系 |
| `/esp32_car/velocity` | `geometry_msgs/TwistStamped` | 开发板串口桥 | 底盘速度诊断 |
| `/esp32_car/yaw` | `std_msgs/Float32` | 开发板串口桥 | 原始 Yaw 诊断，不参与 EKF |

建图时 `/map` 和 `map -> odom` 由 Slam Toolbox 发布；静态地图定位与导航时，
`/map` 由 Map Server 发布，`map -> odom` 由 AMCL 发布。两套系统不能同时运行。

## 五、TF 坐标树与实车尺寸

项目遵循 ROS REP-103：

- X 轴指向车头。
- Y 轴指向车体左侧。
- Z 轴垂直向上。
- 绕 Z 轴逆时针旋转为正。

```text
map                                # Slam Toolbox 或 AMCL
└── odom                           # robot_localization 的局部世界坐标系
    └── base_footprint             # 车体几何中心在地面的投影
        └── base_link              # 第二层中心，离地 0.11 m
            ├── imu_link           # (0, -0.05, 0.06 m)
            └── laser_frame        # (0, 0, 0.07 m), yaw = π
```

实测安装关系：

| 变换 | 平移 | 旋转/说明 |
|---|---|---|
| `base_footprint -> base_link` | `(0, 0, 0.11 m)` | 第二层中心 |
| `base_link -> imu_link` | `(0, -0.05, 0.06 m)` | IMU 在右侧 5 cm，绝对高度 17 cm |
| `base_link -> laser_frame` | `(0, 0, 0.07 m)` | 雷达绝对高度 18 cm，Yaw 为 180° |

车体平面尺寸为 `0.30 m × 0.30 m`，`base_footprint` 位于正方形几何中心。Nav2
局部与全局代价地图使用相同的方形 footprint，并增加 `0.02 m` padding；障碍物
膨胀半径当前为 `0.30 m`。

## 六、SLAM、定位与导航设计

### 1. 建图

Slam Toolbox 运行在线异步建图，使用：

- `/scan_slam`：固定 360 点的 LD14 扫描。
- `/odom`：轮式速度与 IMU 角速度融合后的局部里程计。
- `odom -> base_footprint -> base_link -> laser_frame`：开发板提供的完整 TF。
- `0.025 m/cell`：当前地图分辨率。

`.yaml + .pgm` 是静态栅格地图，供 Map Server、AMCL 和 Nav2 使用；
`.posegraph + .data` 保存 Slam Toolbox 的位姿图和扫描数据，供后续在原地图基础上
继续建图。仅有 PGM/YAML 不能恢复可编辑的建图状态。

### 2. 定位

AMCL 使用 `nav2_amcl::OmniMotionModel`，能够理解全向底盘的 X/Y 平移和旋转。
每次重新启动或人工搬动车辆后，都需要通过 RViz 的 `2D Pose Estimate` 告诉 AMCL
车辆在静态地图中的大致位置和车头方向。

### 3. 导航

当前导航组件：

| 功能 | 实现 |
|---|---|
| 全局规划 | NavFn Planner，启用 A* |
| 路径平滑 | Simple Smoother |
| 局部控制 | Rotation Shim 包装 Omni MPPI |
| 定位 | AMCL OmniMotionModel |
| 障碍建模 | Static Layer + Obstacle Layer + Inflation Layer |
| 速度处理 | 死区补偿 + Velocity Smoother |
| 安全保护 | Collision Monitor + 开发板 `/cmd_vel` 超时停车 |

导航生命周期故意设置为不自动激活。必须先完成 AMCL 初始定位并确认雷达点与地图
重合，再在 RViz Navigation 2 面板点击 `Startup`，以防 `map -> odom` 尚未建立时
代价地图和规划器提前启动。

## 七、仓库结构

```text
.
├── esp32_firmware/
│   ├── flash_gui.bat                    # Windows 图形化烧录入口
│   ├── flash_gui.ps1                    # UI 主程序
│   ├── docs/                            # 协议和排查文档
│   └── modules/firmware/esp32_car/      # ESP-IDF 固件、Web 前端和配置
├── board_control/
│   ├── README.md
│   ├── control_node/                    # esp32_car_control ROS 2 包
│   └── ldlidar_ros2_ws/                 # LD14 ROS 2 驱动工作区
├── vm_slam_nav2/
│   ├── SLAM启动指南.md                   # 虚拟机详细操作和故障排查
│   ├── maps/                            # 当前地图及可续建位姿图
│   └── src/esp32_car_slam/              # SLAM、AMCL、Nav2 和 RViz 配置
├── SOURCE_MANIFEST.md                   # 三端源码来源与导出说明
└── README.md
```

关键实现位置：

| 功能 | 文件或目录 |
|---|---|
| ESP32 运动学、标定参数 | `esp32_firmware/modules/firmware/esp32_car/main/car_config.h` |
| ESP32 ROS 串口 | `esp32_firmware/modules/firmware/esp32_car/main/ros_serial.*` |
| ESP32 Web 页面与遥测 | `esp32_firmware/modules/firmware/esp32_car/main/web_server.*` |
| ROS 串口协议 | `esp32_firmware/docs/ros串口协议.md` |
| 开发板底盘节点 | `board_control/control_node/src/esp32_car_node.cpp` |
| 串口解析 | `board_control/control_node/src/serial_protocol.cpp` |
| 标准传感器消息发布 | `board_control/control_node/src/sensor_publisher.cpp` |
| EKF 参数 | `board_control/control_node/config/ekf.yaml` |
| 开发板静态 TF | `board_control/control_node/launch/esp32_car.launch.py` |
| LD14 启动与雷达 TF | `board_control/control_node/launch/lidar.launch.py` |
| 扫描归一化 | `vm_slam_nav2/src/esp32_car_slam/scripts/scan_normalizer.py` |
| Slam Toolbox 参数 | `vm_slam_nav2/src/esp32_car_slam/config/slam_toolbox.yaml` |
| AMCL 参数 | `vm_slam_nav2/src/esp32_car_slam/config/amcl.yaml` |
| Nav2 参数 | `vm_slam_nav2/src/esp32_car_slam/config/nav2_params.yaml` |
| 一键导航入口 | `vm_slam_nav2/src/esp32_car_slam/launch/nav2.launch.py` |
| 完整实测和历史进度 | `board_control/control_node/PROJECT_PROGRESS.md` |

## 八、地图文件

仓库当前保留的最新地图为：

```text
vm_slam_nav2/maps/aaaaccc.yaml
vm_slam_nav2/maps/aaaaccc.pgm
vm_slam_nav2/maps/aaaaccc.posegraph
vm_slam_nav2/maps/aaaaccc.data
```

| 文件 | 用途 |
|---|---|
| `aaaaccc.yaml` | 地图元数据、分辨率、原点和图像路径 |
| `aaaaccc.pgm` | Nav2/AMCL 使用的静态占据栅格 |
| `aaaaccc.posegraph` | Slam Toolbox 位姿图 |
| `aaaaccc.data` | 与位姿图对应的序列化扫描数据 |

## 九、安全约束

- 建图遥控和 Nav2 不得同时运行，避免多个节点同时发布 `/cmd_vel`。
- `mapping.launch.py`、`localization.launch.py` 和 `nav2.launch.py` 不得同时运行，
  否则可能有多个节点竞争发布 `map -> odom`。
- 每次启动导航或人工搬动车辆后，必须重新设置 `2D Pose Estimate`。
- 雷达点与静态地图未基本重合前，不要激活 Navigation，也不要发送导航目标。
- 首次测试新参数时使用空旷区域和短距离目标，并保留紧急断电手段。
- 开发板底盘节点具有 `/cmd_vel` 超时停车保护，但它不能替代物理急停。

## 十、操作流程

下面的命令按实际部署顺序排列。更完整的虚拟机故障排查见
[`vm_slam_nav2/SLAM启动指南.md`](vm_slam_nav2/SLAM启动指南.md)。

### 1. 烧录 ESP32-S3

在 Windows 中运行：

```text
esp32_firmware/flash_gui.bat
```

在 UI 中选择 ESP32 的 COM 口，填写 Wi-Fi 名称和密码，然后点击烧录。界面会显示
环境检查、编译、烧录和完成进度，并在设备联网后显示实际访问地址。

前端优先尝试：

```text
http://esp32car.local/
```

如果当前电脑或路由器不支持 mDNS，则使用烧录 UI 或串口日志中显示的实际 IP：

```text
http://<ESP32实际IP>/
```

真实 Wi-Fi 凭据不会提交到本仓库；`sdkconfig` 被忽略，
`sdkconfig.defaults` 只保留空值。

### 2. 首次构建开发板工程

先构建 LD14 驱动：

```bash
cd ~/esp32_car_project/ldlidar_ros2_ws
source /opt/ros/rolling/setup.bash
colcon build --symlink-install
```

再构建底盘控制工程：

```bash
cd ~/esp32_car_project/control_node
source /opt/ros/rolling/setup.bash
source ~/esp32_car_project/ldlidar_ros2_ws/install/setup.bash
colcon build --packages-select esp32_car_control --symlink-install
```

### 3. 每次启动开发板基础节点

开发板终端 1：启动 ESP32 串口桥、轮式里程计、IMU、EKF 和车体 TF。

```bash
source /opt/ros/rolling/setup.bash
source ~/esp32_car_project/ldlidar_ros2_ws/install/setup.bash
source ~/esp32_car_project/control_node/install/setup.bash
ros2 launch esp32_car_control localization.launch.py
```

开发板终端 2：启动 LD14。

```bash
source /opt/ros/rolling/setup.bash
source ~/esp32_car_project/ldlidar_ros2_ws/install/setup.bash
source ~/esp32_car_project/control_node/install/setup.bash
ros2 launch esp32_car_control lidar.launch.py
```

确认数据：

```bash
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic echo /imu/data --once
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link laser_frame
```

预期 `/odom` 约 50 Hz，`/scan` 约 6 Hz。

### 4. 首次构建虚拟机工程

```bash
cd ~/桌面/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select esp32_car_slam --symlink-install
source install/setup.bash
```

每个新虚拟机终端都需要加载：

```bash
cd ~/桌面/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

建图或导航前，先在虚拟机验证可以收到开发板数据：

```bash
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link laser_frame
```

### 5. 新建地图

开发板终端 3 启动建图专用键盘遥控：

```bash
source /opt/ros/rolling/setup.bash
source ~/esp32_car_project/control_node/install/setup.bash
ros2 run esp32_car_control mapping_teleop_node
```

虚拟机启动 Slam Toolbox 和建图 RViz：

```bash
cd ~/桌面/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch esp32_car_slam mapping.launch.py use_rviz:=true
```

开始时让车辆静止数秒，再从开发板 SSH 终端低速控制车辆。优先行驶闭合路线并回到
已经扫描过的区域，观察实时雷达点、旧墙线和新墙线是否重合。新建图阶段不使用
`2D Pose Estimate`，也不点击 Navigation 2 的 `Startup`。

### 6. 保存地图

停车并等待约 3 秒，不要关闭正在运行的 `mapping.launch.py`。在新的虚拟机终端
执行：

```bash
cd ~/桌面/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

MAP_NAME=room_$(date +%Y%m%d_%H%M)
echo "本次地图名称：${MAP_NAME}"

ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$HOME/桌面/ros2_ws/maps/${MAP_NAME}'}"

ros2 run esp32_car_slam save_map.py \
  --output "$HOME/桌面/ros2_ws/maps/${MAP_NAME}"

ls -lh "$HOME/桌面/ros2_ws/maps/${MAP_NAME}".*
```

确认生成 `.yaml`、`.pgm`、`.posegraph` 和 `.data` 后，再退出 Slam Toolbox。

### 7. 在旧地图基础上继续建图

先按“新建地图”步骤启动 `mapping.launch.py`，但不要动车。车辆位于原建图起点且
朝向一致时，在另一虚拟机终端加载旧位姿图：

```bash
source /opt/ros/jazzy/setup.bash
source ~/桌面/ros2_ws/install/setup.bash

ros2 service call /slam_toolbox/deserialize_map \
  slam_toolbox/srv/DeserializePoseGraph \
  "{filename: '/home/luohjj/桌面/ros2_ws/maps/aaaaccc', match_type: 1, initial_pose: {x: 0.0, y: 0.0, theta: 0.0}}"
```

等待旧地图完整出现并确认雷达点与地图重合，然后开始扩建。完成后按“保存地图”
步骤使用新名称保存，不要直接覆盖原地图。

如果车辆不在原建图起点，需要先获得其在旧地图中的 `x/y/theta`，并使用
`match_type: 2`。

### 8. 使用静态地图启动 Nav2

先停止开发板上的建图遥控节点，并确认没有其他 `/cmd_vel` 发布者：

```bash
pkill -TERM -x mapping_teleop_node
ros2 topic info /cmd_vel
```

确认 `mapping.launch.py` 已退出，然后在虚拟机启动完整定位与导航：

```bash
cd ~/桌面/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch esp32_car_slam nav2.launch.py \
  map:=$HOME/桌面/ros2_ws/maps/aaaaccc.yaml \
  start_rviz:=true
```

在 RViz 中严格按以下顺序操作：

1. 点击 `2D Pose Estimate`，在地图上设置车辆实际位置和车头方向。
2. 等待 AMCL 收敛，确认实时雷达点与静态地图障碍物基本重合。
3. 在 Navigation 2 面板点击 `Startup`。
4. 等待 `Localization: active` 和 `Navigation: active`。
5. 点击 `Nav2 Goal`，拖出目标位置和最终车头方向。

如果车辆被人工搬动、雷达点明显错位或定位发散，应立即取消导航，重新设置
`2D Pose Estimate`，确认重新重合后再发送目标。

### 9. 只启动静态地图定位

如果只检查地图、雷达和 AMCL，不需要 Nav2 控制：

```bash
cd ~/桌面/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch esp32_car_slam localization.launch.py \
  map:=$HOME/桌面/ros2_ws/maps/aaaaccc.yaml \
  use_rviz:=true
```

设置 `2D Pose Estimate` 后，车辆静止且 AMCL 未立即刷新时可以请求一次无运动更新：

```bash
ros2 service call /request_nomotion_update std_srvs/srv/Empty '{}'
```
