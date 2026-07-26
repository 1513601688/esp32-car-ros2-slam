# esp32_car_control

ROS 2 C++ 串口桥接与底盘控制包。节点向 ESP32 下发速度指令，并把 ESP32
独立上报的轮速和 IMU 数据转换为标准 ROS 2 消息，供后续定位与 SLAM 使用。

## 构建与启动

```bash
cd ~/esp32_car_project/control_node
source /opt/ros/rolling/setup.bash
colcon build --symlink-install --packages-select esp32_car_control
source install/setup.bash
ros2 launch esp32_car_control esp32_car.launch.py serial_port:=/dev/ttyUSB0
```

默认参数位于 `config/esp32_car.yaml`。设备变化时优先通过 launch 参数覆盖串口，
其他话题名、坐标系和协方差统一在 YAML 中维护。

## 模块职责

- `src/esp32_car_node.cpp`：底盘串口收发、`/cmd_vel`、安全看门狗和 Service
- `src/mapping_teleop_node.cpp`：唯一的交互式键盘控制程序
- `src/sensor_publisher.cpp`：轮式里程计与 IMU 的 ROS 2 消息构造和发布
- `include/esp32_car_control/sensor_publisher.hpp`：传感器发布接口
- `src/serial_protocol.cpp`：ESP32 上行协议解析

控制节点只向 `SensorPublisher` 传递解析后的结构体，不直接创建或发布传感器话题。

## ROS 接口

- 订阅 `/cmd_vel` (`geometry_msgs/msg/Twist`)
- 发布 `/wheel/odom` (`nav_msgs/msg/Odometry`)，只填写底盘速度，不发布 TF
- 发布 `/imu/data` (`sensor_msgs/msg/Imu`)
- 兼容发布 `/esp32_car/velocity`、`/esp32_car/yaw` 和 `/esp32_car/command`

`odom -> base_footprint` 的位姿和 TF 应由 `robot_localization` 等融合节点唯一发布，
避免原始轮速节点和融合节点同时发布同一 TF。

当前车体坐标系按实体安装高度划分：

```text
odom -> base_footprint -> base_link -> imu_link
                                └──> laser_frame
```

- `base_footprint`：车体几何中心在地面的投影，`z=0`
- `base_link`：第二层几何中心，离地 `0.11 m`
- `imu_link`：相对 `base_link` 为 `(0, -0.05, 0.06 m)`，离地 `0.17 m`
- `laser_frame`：相对 `base_link` 为 `(0, 0, 0.07 m)`，离地 `0.18 m`，Yaw `180°`

## 二维 EKF 里程计

安装融合节点：

```bash
sudo apt update
sudo apt install ros-rolling-robot-localization
```

启动底盘串口桥、IMU 静态 TF 和 EKF：

```bash
source /opt/ros/rolling/setup.bash
source ~/esp32_car_project/control_node/install/setup.bash
ros2 launch esp32_car_control localization.launch.py
```

第一版 EKF 只融合 `/wheel/odom` 的 `vx/vy` 和 `/imu/data` 的 `gz`，输出
`/odom` 并发布 `odom -> base_footprint`。JY901 的地磁融合 Yaw 不参与融合。

## 开发板本地建图遥控

建图时由开发板本地发布 `/cmd_vel`，虚拟机只运行 Slam Toolbox 和 RViz，
不再承担键盘控制。先在开发板的一个 SSH 终端启动底盘、EKF 和雷达，
再在另一个交互式 SSH 终端运行：

```bash
source /opt/ros/rolling/setup.bash
source ~/esp32_car_project/control_node/install/setup.bash
ros2 run esp32_car_control mapping_teleop_node
```

按键：

```text
W / S       前进 / 后退
A / D       左移 / 右移
Q / E       逆时针 / 顺时针旋转
U / O       左前 / 右前斜向平移
M / .       左后 / 右后斜向平移
1 / 2 / 3   低速 / 中速 / 高速
+ / -       逐级加速 / 减速
K / X / 空格 立即停车
Ctrl-C      停车并退出
```

默认使用中速档：线速度 `0.20 m/s`、角速度 `0.40 rad/s`。低速档为
`0.10/0.20`，高速档为 `0.35/0.70`；单位分别为 m/s 和 rad/s。
`+/-` 每次调整 `0.05 m/s` 和 `0.10 rad/s`，软件限制为最高
`0.60 m/s` 和 `1.20 rad/s`。调速前会主动停车，下一次运动使用新速度。

由于 SSH 终端不提供
按键松开事件，程序会识别操作系统产生的连续按键重复：按住进入连续运动后，
松手约 `0.12 s` 自动发布零速度。首次按键仍使用 `0.70 s` 超时跨过键盘首次
重复延迟，避免按住时走走停停。运动期间以 `50 Hz` 持续发布；若遥控进程
异常退出，底盘节点的 `/cmd_vel` 0.5 秒看门狗会再次兜底停车。

需要临时调整速度时可通过 ROS 参数启动：

```bash
ros2 run esp32_car_control mapping_teleop_node --ros-args \
  -p linear_speed:=0.25 -p angular_speed:=0.50
```

同一时间只能运行一个 `/cmd_vel` 控制源。建图期间不要在虚拟机启动旧的
`mapping_teleop.py`，后续运行 Nav2 时也应先退出本地遥控节点。

SSH 终端会把多个按键串行发送，无法可靠提供多个键的按下/松开状态，因此不使用
`W+A` 等多键组合。每个方向键都会生成一条完整的新速度命令并把其他轴归零；
斜向平移统一使用 `U/O/M/.`，避免频繁切换时残留上一方向的速度分量。

## ESP32 上行协议

```text
WHEEL,<stamp_ms>,<vx>,<vy>,<wz>
IMU,<stamp_ms>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<roll>,<pitch>,<yaw>
```

单位分别为 ms、m/s、rad/s、m/s^2 和 rad。协议解析代码集中在
`serial_protocol.hpp/.cpp`，并由 `serial_protocol_test` 覆盖正常帧和错误帧。
