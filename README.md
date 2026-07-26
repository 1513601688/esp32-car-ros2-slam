# ESP32 麦克纳姆小车 ROS 2 SLAM 与导航系统

本仓库汇总了当前小车系统的三个配套工程：ESP32-S3 下位机固件、开发板端
ROS 2 控制与传感器节点，以及虚拟机端 SLAM/Nav2 工程。三个模块在同一次提交
中保持版本对应，便于部署、回滚和继续开发。

## 仓库结构

```text
.
├── esp32_firmware/       # Windows 电脑：ESP32-S3 固件、Web 前端和带 UI 的烧录工具
├── board_control/        # 开发板：底盘串口、轮式里程计、IMU、EKF、TF 和 LD14 驱动
└── vm_slam_nav2/         # Ubuntu 虚拟机：建图、AMCL 定位、Nav2 和最新地图
```

## 系统数据链路

```text
ESP32-S3
  ├─ 接收 /cmd_vel 对应的串口控制帧
  └─ 上报轮速、IMU 和底盘状态
          ↓
开发板 ROS 2
  ├─ /wheel/odom
  ├─ /imu/data
  ├─ EKF → /odom
  ├─ TF: odom → base_footprint → base_link
  └─ LD14 → /scan
          ↓
虚拟机 ROS 2
  ├─ Slam Toolbox 建图
  ├─ Map Server + AMCL 定位
  └─ Nav2: NavFn A* + Omni MPPI + Rotation Shim
```

## 1. ESP32-S3 固件

源码位于：

```text
esp32_firmware/modules/firmware/esp32_car
```

Windows 下可直接运行：

```text
esp32_firmware/flash_gui.bat
```

在界面中选择 COM 口并填写 Wi-Fi 名称和密码后烧录。真实 Wi-Fi 凭据不会存入
本仓库；`sdkconfig` 被忽略，`sdkconfig.defaults` 中只保留空值。

## 2. 开发板端控制

开发板使用 ROS 2 Rolling。先构建 LD14 驱动：

```bash
cd ~/esp32_car_project/ldlidar_ros2_ws
source /opt/ros/rolling/setup.bash
colcon build --symlink-install
```

再构建控制工程：

```bash
cd ~/esp32_car_project/control_node
source /opt/ros/rolling/setup.bash
source ~/esp32_car_project/ldlidar_ros2_ws/install/setup.bash
colcon build --packages-select esp32_car_control --symlink-install
```

启动底盘、里程计、IMU 和 EKF：

```bash
source /opt/ros/rolling/setup.bash
source ~/esp32_car_project/ldlidar_ros2_ws/install/setup.bash
source ~/esp32_car_project/control_node/install/setup.bash
ros2 launch esp32_car_control localization.launch.py
```

另开终端启动 LD14：

```bash
source /opt/ros/rolling/setup.bash
source ~/esp32_car_project/ldlidar_ros2_ws/install/setup.bash
source ~/esp32_car_project/control_node/install/setup.bash
ros2 launch esp32_car_control lidar.launch.py
```

更完整的协议、标定和历史进度见
`board_control/control_node/PROJECT_PROGRESS.md`。

## 3. 虚拟机端 SLAM 与 Nav2

虚拟机使用 Ubuntu 24.04 和 ROS 2 Jazzy。将 `vm_slam_nav2/src/esp32_car_slam`
放入 ROS 2 工作区的 `src` 后构建：

```bash
cd ~/桌面/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select esp32_car_slam --symlink-install
source install/setup.bash
```

最新地图为：

```text
vm_slam_nav2/maps/aaaaccc.yaml
vm_slam_nav2/maps/aaaaccc.pgm
vm_slam_nav2/maps/aaaaccc.posegraph
vm_slam_nav2/maps/aaaaccc.data
```

其中 YAML/PGM 用于 AMCL 和 Nav2；PoseGraph/Data 用于继续原地图建图。

虚拟机端详细启动步骤见 `vm_slam_nav2/SLAM启动指南.md`。

## 安全说明

- Nav2 运行期间不得同时启动开发板键盘遥控节点，避免多个 `/cmd_vel` 发布者冲突。
- 每次启动导航都应先用 `2D Pose Estimate` 设置真实初始位姿。
- 雷达点与静态地图基本重合后，再在 Navigation 2 面板点击 `Startup`。
- 首次实车测试应选择空旷区域和短距离目标，并保留紧急断电手段。
