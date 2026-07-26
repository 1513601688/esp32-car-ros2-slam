# ESP32 Car ROS 2 项目进度说明

> 最后更新：2026-07-26  
> 上位机：`luohjj@dg-pi-rb5.local`  
> 项目总目录：`/home/luohjj/esp32_car_project`  
> 底盘 ROS 2 工作目录：`/home/luohjj/esp32_car_project/control_node`  
> 雷达 ROS 2 工作目录：`/home/luohjj/esp32_car_project/ldlidar_ros2_ws`  
> ESP32 固件目录（开发电脑）：`C:\Users\clearlove\Desktop\ESP32_project\esp32_car\modules\firmware\esp32_car`

## 1. 文档用途

本文档用于记录 ESP32 小车从下位机串口上报、ROS 2 标准话题发布、轮式里程计与 IMU 融合，到后续激光雷达 SLAM 的项目进度。

重新接手项目时建议先阅读本文档，不需要重新探索整个工程。文档中的“实测结果”是截至最后更新时间的真实测试数据；“待办任务”按推荐执行顺序排列。

## 2. 当前结论

底盘侧的 ROS 2 里程计链路已经基本完成：

- ESP32 独立发送轮速和 IMU 数据。
- 上位机使用 C++ 独立解析串口协议并发布标准 ROS 2 话题。
- `robot_localization` 已融合轮式 `vx、vy` 和 IMU `gz`。
- 已发布可供 SLAM 使用的 `/odom` 和 `odom -> base_footprint` TF。
- 前进、后退、左移、右移和原地旋转方向均已实车验证。
- LDROBOT LD14 已通过 USB 转串口接入，并稳定发布 `/scan`。
- 已根据实测高度和数据手册方向建立 `base_link -> laser_frame` 静态 TF。
- 已确定采用分布式架构：开发板发布标准话题，电脑虚拟机运行 `slam_toolbox`、RViz 和 Nav2。
- 虚拟机已安装并激活 `slam_toolbox`，已创建独立工程和 `mapping.launch.py`，初始 `/map` 与 `map -> odom` 已验证。
- 虚拟机已安装 Nav2 Jazzy，并新增静态地图 + AMCL 的独立定位入口。

当前阶段可以概括为：**开发板发布端、跨机 DDS、开发板本地全向遥控、SLAM 建图和地图保存均已打通；虚拟机只负责 SLAM/RViz/Nav2。静态地图重载、AMCL 定位、全向路径跟踪和终点原地转向均已完成实车验证。**

## 3. 工程结构

ROS 2 工程主要文件：

```text
/home/luohjj/esp32_car_project/control_node/
├── CMakeLists.txt
├── package.xml
├── README.md
├── PROJECT_PROGRESS.md
├── config/
│   ├── esp32_car.yaml
│   ├── ekf.yaml
│   └── ld14.yaml
├── include/esp32_car_control/
│   ├── serial_protocol.hpp
│   └── sensor_publisher.hpp
├── launch/
│   ├── esp32_car.launch.py
│   ├── lidar.launch.py
│   └── localization.launch.py
├── src/
│   ├── esp32_car_node.cpp
│   ├── mapping_teleop_node.cpp
│   ├── serial_protocol.cpp
│   └── sensor_publisher.cpp
├── srv/
│   └── TimedVelocity.srv
└── test/
    └── serial_protocol_test.cpp
```

模块职责：

- `esp32_car_node.cpp`：串口管理、`/cmd_vel`、安全看门狗和定时定速服务，不再读取键盘。
- `mapping_teleop_node.cpp`：开发板上唯一的交互式键盘控制程序，独立发布 `/cmd_vel`。
- `serial_protocol.cpp`：独立解析 ESP32 串口协议。
- `sensor_publisher.cpp`：将解析后的轮速和 IMU 数据转换成标准 ROS 2 消息。
- `esp32_car.yaml`：串口、话题、坐标系、轮速比例和传感器协方差。
- `ekf.yaml`：`robot_localization` 二维 EKF 配置。
- `ld14.yaml`：LD14 型号、稳定串口路径、话题、坐标帧和扫描方向配置。
- `lidar.launch.py`：启动 LD14 驱动，并发布已确认的 `base_link -> laser_frame` 静态 TF。
- `localization.launch.py`：统一启动底盘节点、静态 IMU TF 和 EKF。

LD14 官方驱动位于独立工作区：

```text
/home/luohjj/esp32_car_project/ldlidar_ros2_ws/src/ldlidar_sl_ros2
```

## 4. ESP32 下位机数据协议

ESP32 当前独立发送：

```text
WHEEL,<stamp_ms>,<vx>,<vy>,<wz>
IMU,<stamp_ms>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<roll>,<pitch>,<yaw>
```

实测频率：

- `WHEEL`：约 `24.6 Hz`
- `IMU`：约 `50 Hz`

固件侧已经完成的主要修改：

- 轮速和 IMU 独立发送，不再塞入同一帧。
- 电机轮速查询周期提高。
- 电机查询按实际 8 字节回复读取，消除了不必要的串口等待。
- 保留原有已校准的底盘 `vx、vy` 计算逻辑。

## 5. 当前 ROS 2 接口

标准接口：

| 名称 | 类型/用途 | 状态 |
|---|---|---|
| `/cmd_vel` | 底盘速度控制 | 已完成 |
| `/wheel/odom` | `nav_msgs/Odometry`，当前有效字段为 twist | 已完成 |
| `/imu/data` | `sensor_msgs/Imu` | 已完成 |
| `/odom` | EKF 融合后的 `nav_msgs/Odometry` | 已完成 |
| `/tf` | 动态 TF | 已完成 |
| `/tf_static` | 静态 TF | 已完成，残留进程已清理 |
| `/scan` | LD14 `sensor_msgs/LaserScan` | 已完成 |
| `/pointcloud2d` | LD14 二维点云辅助话题 | 已完成 |

辅助接口：

- `/esp32_car/command`
- `/esp32_car/velocity`
- `/esp32_car/yaw`
- `/diagnostics`
- `/esp32_car/timed_velocity` 服务

定时定速服务定义：

```text
float64 vx
float64 vy
float64 wz
float64 duration
---
bool success
string message
```

## 6. 坐标系约定与 TF

项目遵循 ROS REP-103：

- X：车头前方
- Y：车体左侧
- Z：垂直向上
- 绕 Z 轴逆时针：正方向

`base_footprint` 是车体几何中心在地面的投影。`base_link` 位于第二层几何
中心，离地 `0.11 m`。IMU 安装方向与车体轴一致，安装位置相对
`base_link` 为：

```text
x = 0.00 m
y = -0.05 m
z = 0.06 m
roll = 0
pitch = 0
yaw = 0
```

即 IMU 位于车体中心右侧 5 cm、第二层上方 6 cm，绝对离地高度为 17 cm。
雷达扫描平面相对 `base_link` 高 7 cm，绝对离地高度为 18 cm；车头对应
雷达 180°，因此 `laser_frame` 相对 `base_link` 的 Yaw 为 `pi rad`。

当前目标 TF 结构：

```text
map                              # SLAM 或 AMCL 发布 map -> odom
└── odom                         # EKF 的世界坐标系
    └── base_footprint           # EKF 发布，地面投影
        └── base_link            # 静态 TF，z = 0.11 m
            ├── imu_link          # (0, -0.05, 0.06 m)
            └── laser_frame       # (0, 0, 0.07 m), yaw = pi
```

## 7. IMU 实测结论

静止测试结果表明 IMU 加速度噪声较小，`gz` 在固件死区处理后静止时为零。

旋转测试发现：

- JY901 的地磁融合绝对 Yaw 受车体电磁环境干扰。
- 实际旋转约 90° 时，地磁融合 Yaw 曾只变化约 63°。
- 同一测试中 `gz` 积分约 91°，明显更可信。

因此当前处理策略为：

- `/imu/data.orientation` 不参与融合。
- `orientation_covariance[0] = -1`，明确表示姿态不可用。
- 原始地磁 Yaw 只保留在 `/esp32_car/yaw` 供诊断。
- EKF 只融合 IMU 的 Z 轴角速度 `gz`。
- 暂不融合 IMU 线加速度。

**后续不要直接把 JY901 的绝对 Yaw 加入 EKF，除非重新完成磁力计标定并验证车体电磁干扰。**

## 8. EKF 融合方案

使用软件包：

```text
ros-rolling-robot-localization
```

当前 `config/ekf.yaml` 核心设置：

```yaml
frequency: 50.0
two_d_mode: true
publish_tf: true
world_frame: odom
```

融合输入：

- `/wheel/odom`：只融合 `vx、vy`
- `/imu/data`：只融合 `vyaw`，即 IMU `gz`

不融合：

- 轮式 `wz`
- JY901 绝对 Yaw
- IMU 线加速度
- `/wheel/odom` 中无效的 pose

输出：

- `/odom`，实测约 `50 Hz`
- `odom -> base_link` TF

轮式 `wz` 在电机驱动旋转测试中方向和幅值均不可靠：理论逆时针约 90° 时，轮式积分约为 `-43°`。因此当前不融合轮式 `wz` 是有意设计，不是遗漏。

## 9. 当前轮速校准参数

`config/esp32_car.yaml` 当前参数：

```yaml
wheel_vx_scale: -1.0
wheel_vy_scale: -0.97
```

解释：

- `wheel_vx_scale: -1.0`：修正底盘原始 `vx` 与 ROS X 正方向相反的问题。
- `wheel_vy_scale: -0.97`：修正底盘原始 `vy` 的方向，并根据左右移动实测取折中尺度。
- 这些参数只影响轮式里程计发布，不改变 `/cmd_vel` 控制方向。

注意：最终的 `wheel_vy_scale: -0.97` 是根据左移和右移结果计算的折中值，**方向已经验证，但该最终数值尚未再次进行实车复测**。

## 10. 实车测试记录

### 10.1 前进

命令：

```text
vx = +0.1 m/s
duration = 2 s
```

结果：

- 尺子实测：约 `18.5 cm`
- `/odom`：约 `18.83 cm`
- 相对误差：约 `+1.8%`
- 方向：ROS 正 X，正确

### 10.2 后退

命令：

```text
vx = -0.1 m/s
duration = 2 s
```

结果：

- 尺子实测：约 `18.5 cm`
- `/wheel/odom` 积分：约 `18.80 cm`
- `/odom`：约 `18.90 cm`
- 相对误差：约 `+2.2%`
- 横向偏移：约 `0.98 mm`
- 方向：ROS 负 X，正确

结论：`vx` 正反方向一致，`wheel_vx_scale: -1.0` 无需继续调整。

### 10.3 左移

命令：

```text
vy = +0.1 m/s
duration = 2 s
```

原始数据曾把实际左移报告为负 Y，已修正符号。最准确一次尺子测量约 `17.5 cm`，用于计算横向比例。

### 10.4 右移

命令：

```text
vy = -0.1 m/s
duration = 2 s
```

在 `wheel_vy_scale: -0.95` 时重复测试：

- 尺子实测：约 `18.0 cm`
- `/wheel/odom` 积分：约 `17.48 cm`
- `/odom`：约 `17.46 cm`
- 前后串扰：约 `0.10 mm`
- 方向：ROS 负 Y，正确

左移和右移建议的比例分别约为 `0.95` 和 `0.98`，说明横向正反运动存在轻微不对称。当前最终取折中值 `0.97`。

### 10.5 原地逆时针旋转

命令：

```text
wz = +0.5 rad/s
duration = pi s
理论目标约 90°
```

结果：

- 肉眼实测：距离 90° 还差几度
- IMU `gz` 积分：`86.729°`
- `/odom` 航向变化：`86.733°`
- IMU 与 `/odom` 差异：约 `0.004°`
- 原地旋转位置漂移：约 `3.6 mm`
- 轮式 `wz` 积分：约 `-43°`，不可靠

结论：IMU `gz` 和融合航向可信，底盘执行本身没有完全转满理论 90°。

## 11. 编译与启动

进入工作区后编译：

```bash
cd /home/luohjj/esp32_car_project/control_node
source /opt/ros/rolling/setup.bash
colcon build --packages-select esp32_car_control --symlink-install
source install/setup.bash
```

运行协议测试：

```bash
cd /home/luohjj/esp32_car_project/control_node
source /opt/ros/rolling/setup.bash
source install/setup.bash
colcon test --packages-select esp32_car_control
colcon test-result --verbose
```

启动底盘和 EKF：

```bash
cd /home/luohjj/esp32_car_project/control_node
source /opt/ros/rolling/setup.bash
source install/setup.bash
ros2 launch esp32_car_control localization.launch.py
```

指定其他串口：

```bash
ros2 launch esp32_car_control localization.launch.py serial_port:=/dev/ttyUSB1
```

## 12. 快速验证命令

查看核心话题：

```bash
ros2 topic list
ros2 topic hz /wheel/odom
ros2 topic hz /imu/data
ros2 topic hz /odom
```

检查停车状态：

```bash
ros2 topic echo /esp32_car/command --once
```

检查融合里程计：

```bash
ros2 topic echo /odom --once
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link imu_link
```

检查诊断：

```bash
ros2 topic echo /diagnostics
```

## 13. 已知问题与注意事项

### 13.1 残留静态 TF 进程（已解决）

此前多次后台重启 Launch 后，系统中曾残留数个孤立的 `static_transform_publisher`，它们都发布相同的 `base_link -> imu_link`。

开发板在 2026-07-20 重启后，旧孤立进程已全部清除。目前只保留当前 `localization.launch.py` 管理的一只 IMU 静态 TF 发布器。

先检查：

```bash
ps -eo pid,ppid,args | grep static_transform_publisher | grep -v grep
```

清理时应先确认 PID，避免结束当前 Launch 所属进程。

### 13.2 系统升级等待交互（已解决）

此前系统中有一次 `apt upgrade` 在等待 Docker 配置问题：

```text
Automatically restart Docker daemon?
```

开发板重启后该等待进程已消失，当前未发现 APT 锁占用。

### 13.3 ROS Rolling 平台警告

当前环境是 Ubuntu 24.04 + ROS 2 Rolling。系统持续提示 Rolling 已迁移到 Ubuntu 26.04，Ubuntu 24.04 不再接收最新的 Rolling 系统包。

目前 `robot_localization` 已在系统升级后正常运行，但未来安装 SLAM/Nav2 软件包时仍需留意 ABI 或版本不匹配问题。

### 13.4 ROS 图中节点名称偶尔不可见

曾出现 `ros2 node list` 为空、但话题和进程正常工作的情况，可能与 Fast DDS/ROS daemon 图缓存有关。可尝试：

```bash
ros2 daemon stop
ros2 daemon start
```

不要仅根据 `ros2 node list` 判断底盘节点是否已经退出，同时检查进程和话题。

## 14. 激光雷达 SLAM 总体步骤

- [x] ESP32 独立上报轮速与 IMU
- [x] 上位机 C++ 串口协议解析
- [x] 发布 `/wheel/odom`
- [x] 发布 `/imu/data`
- [x] 建立 `base_link -> imu_link`
- [x] 校准轮式 X/Y 方向
- [x] 验证前进、后退、左移、右移
- [x] 验证 IMU `gz` 旋转积分
- [x] 配置 `robot_localization`
- [x] 发布 `/odom` 与 `odom -> base_link`
- [ ] 最终复测 `wheel_vy_scale: -0.97`
- [x] 清理残留静态 TF 进程
- [x] 完成等待中的系统升级
- [x] 确认激光雷达型号和连接方式
- [x] 安装并启动雷达驱动
- [x] 确认稳定发布 `/scan`
- [x] 测量雷达相对 `base_link` 的安装位姿
- [x] 添加 `base_link -> laser_frame`
- [x] 验证完整 TF 和雷达时间戳
- [x] 在虚拟机安装并配置 `slam_toolbox`
- [x] 在虚拟机创建 `mapping.launch.py`
- [x] 使用 RViz 低速实车建图
- [ ] 调整 SLAM 和传感器协方差参数
- [x] 完成绕房间一圈的闭合路线测试
- [x] 保存并重新加载地图（`room_0025.yaml` 已通过 Nav2 Map Server 验证）
- [x] 安装 Nav2 Jazzy
- [x] 完成 AMCL 实车初始定位和运动收敛验证
- [x] 配置 Nav2 路径规划、全向控制、代价地图和碰撞监控
- [ ] 完成 Nav2 首次短距离目标点实车测试

## 15. 下一次继续项目时的推荐顺序

1. 阅读本文档的“已知问题”和“待办任务”。
2. 启动 `localization.launch.py` 和 `lidar.launch.py`。
3. 验证 `/wheel/odom`、`/imu/data`、`/odom`、`/scan` 和现有 TF。
4. 如需要，最后复测一次 `wheel_vy_scale: -0.97`。
5. 确保虚拟机使用桥接网络并能看到开发板 ROS 2 话题。
6. 确认 ESP32 复位后 `/wheel/odom` 和 `/imu/data` 恢复实时发布。
7. 建图时启动 `mapping.launch.py`；使用保存地图时启动 `localization.launch.py`，两者不能同时运行。
8. 在 RViz 使用 `2D Pose Estimate` 设置初始位姿，验证 AMCL 粒子和雷达点收敛。

## 16. 后续接入激光雷达时需要提供的信息

已确认信息：

- 型号：LDROBOT LD14 二维三角测距雷达
- 接口：USB 转串口
- 设备节点：`/dev/ttyACM0`
- 稳定路径：`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6C087033-if00`
- 波特率：`115200`
- ROS 2 驱动：官方 `ldlidar_sl_ros2`
- `/scan`：已正常发布

安装位姿已确认：

```text
base_link -> laser_frame
x = 0.00 m
y = 0.00 m
z = 0.15 m
roll = 0
pitch = 0
yaw = pi rad (180°)
```

雷达水平安装在底盘正中心上方 15 cm。数据手册表明雷达 `180°` 指向车头，因此雷达 `0°/X轴` 指向车尾，需要相对 `base_link` 绕 Z 轴旋转 180°。

后续在 RViz 建图时仍需观察车体自身遮挡；只有确认存在固定遮挡后才启用角度裁剪。

## 17. LD14 接入与验证记录（2026-07-20）

官方驱动来源：

```text
https://github.com/ldrobotSensorTeam/ldlidar_sl_ros2.git
commit: d70802ac5d46e4e02c8318b2769f508f0f86172e
```

当前 ROS 2 Rolling 已移除旧的 `ament_target_dependencies()` CMake helper，因此对官方驱动做了最小兼容补丁，改为直接链接：

```cmake
rclcpp::rclcpp
${sensor_msgs_TARGETS}
${geometry_msgs_TARGETS}
```

驱动编译成功，仅有一个旧源码中的枚举与浮点运算弃用警告，不影响运行。

启动雷达：

```bash
source /opt/ros/rolling/setup.bash
source /home/luohjj/esp32_car_project/ldlidar_ros2_ws/install/setup.bash
source /home/luohjj/esp32_car_project/control_node/install/setup.bash
ros2 launch esp32_car_control lidar.launch.py
```

100 帧 `/scan` 实测：

- 帧：`laser_frame`
- 频率：`ros2 topic hz /scan` 稳定为 `5.999 Hz`；两次含统计计算的 100 帧验证约为 `5.88 Hz`、`5.94 Hz`
- 角度覆盖：`0–360°`
- 角度增量：约 `0.923°`
- 每圈点数：`390–393`
- 平均有效点数：约 `359`
- 平均有效点比例：约 `91.7%`
- 消息量程字段：`0.02–12.0 m`
- 现场观测距离：`0.333–2.482 m`
- 时间戳：严格单调
- 驱动日志：`ldlidar communication is normal`
- 持续采样期间：未发现通信错误

最终端口占用互不冲突：

```text
/dev/ttyUSB0  -> esp32_car_node
/dev/ttyACM0  -> ldlidar_sl_ros2_node
```

`/pointcloud2d` 同时正常发布，消息类型为 `sensor_msgs/PointCloud`。底盘协议单元测试 `serial_protocol_test` 为 `1/1` 通过。

注意：在当前“包根目录同时也是 colcon 工作区”的目录布局中直接运行全量 lint，会把 `build/` 和 `install/` 中的 ROS 自动生成文件也纳入扫描并产生大量格式失败；这不代表功能测试失败。协议单元测试应使用：

```bash
ctest --test-dir build/esp32_car_control -R serial_protocol_test --output-on-failure
```

当前 `/scan.header.frame_id` 为 `laser_frame`，正式 `lidar.launch.py` 同时发布实测静态 TF：平移 `(0, 0, 0.15 m)`，Yaw 为 `pi rad`。该参数来自实际安装高度和 LD14 数据手册方向，没有照抄官方示例的 `z=0.18 m`。

`tf2_echo base_link laser_frame` 已验证：

```text
Translation: [0.000, 0.000, 0.150]
Quaternion (xyzw): [0.000, 0.000, 1.000, 0.000]
RPY degree: [0.000, 0.000, 180.000]
```

加入 TF 并由正式 Launch 重启后，`/scan` 仍稳定为 `5.999 Hz`，消息帧为 `laser_frame`，日志无通信错误。

## 18. 备份记录

上位机现有重要备份：

```text
/home/luohjj/.codex_backups/control_node_20260716
/home/luohjj/.codex_backups/control_node_before_publisher_split_20260716
/home/luohjj/.codex_backups/control_node_before_imu_yaw_disable_20260718
/home/luohjj/.codex_backups/control_node_before_vx_sign_fix_20260718
/home/luohjj/.codex_backups/control_node_before_vy_calibration_20260718
/home/luohjj/.codex_backups/control_node_before_ld14_integration_20260720
/home/luohjj/.codex_backups/control_node_before_laser_tf_20260720
/home/luohjj/.codex_backups/control_node_before_cmd_vel_watchdog_20260720
/home/luohjj/.codex_backups/control_node_before_board_teleop_20260721
/home/luohjj/.codex_backups/control_node_before_remove_legacy_keyboard_20260721
/home/luohjj/.codex_backups/control_node_before_adaptive_teleop_timeout_20260721
/home/luohjj/.codex_backups/control_node_before_diagonal_teleop_20260721
/home/luohjj/.codex_backups/control_node_before_direction_residue_fix_20260721
/home/luohjj/.codex_backups/control_node_before_runtime_speed_control_20260721
```

虚拟机端遥控清理前备份：

```text
/home/luohjj/.codex_backups/esp32_car_slam_before_remove_vm_teleop_20260721
/home/luohjj/.codex_backups/esp32_car_slam_before_resolution_0025_20260721
```

恢复前应先对当前工作区另做备份，不要直接覆盖仍有价值的新修改。

## 19. 开发板与虚拟机分布式架构（2026-07-20）

已确定不在开发板运行 SLAM 和导航。职责划分如下：

```text
开发板 dg-pi-rb5（当前 DHCP 地址 192.168.31.78）
├── ESP32 串口桥接与 /cmd_vel 控制
├── /wheel/odom
├── /imu/data
├── robot_localization -> /odom + odom -> base_link
├── LD14 -> /scan + /pointcloud2d
├── base_link -> imu_link
└── base_link -> laser_frame

电脑虚拟机
├── slam_toolbox -> /map + map -> odom
├── RViz2 建图显示
├── 地图保存与加载
└── Nav2 定位、规划、控制和可视化
```

开发板 ROS 2 网络状态：

```text
Wi-Fi: 192.168.31.78/24（曾为 192.168.31.28，重启后 DHCP 已变化）
ROS_DOMAIN_ID: 0（默认）
RMW: rmw_fastrtps_cpp（默认）
ROS_LOCALHOST_ONLY: 0（未限制为本机）
NTP: synchronized
```

开发板已提供虚拟机需要的标准话题：

```text
/scan
/odom
/tf
/tf_static
/cmd_vel
/wheel/odom
/imu/data
```

虚拟机要求：

- 虚拟网卡使用桥接模式，获得 `192.168.31.0/24` 网段地址；NAT 模式可能阻断 DDS 组播发现。
- 与开发板使用相同 `ROS_DOMAIN_ID=0`。
- 建议使用相同 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`。
- `ROS_LOCALHOST_ONLY=0`。
- 系统时间必须通过 NTP 同步。
- 防火墙应允许 DDS 使用的 UDP 组播与动态端口。
- 建图前必须先在虚拟机确认能够看到 `/scan`、`/odom`、`/tf` 和 `/tf_static`。

开发板上曾开始下载 `slam_toolbox` 源码，但在确定分布式架构后已停止并清理，开发板没有安装或运行 SLAM 组件。

## 20. 虚拟机跨机通信与 SLAM 初步验证（2026-07-20）

虚拟机信息：

```text
主机名: luohjj-VMware-Virtual-Platform
系统: Ubuntu 24.04.4 LTS
地址: 192.168.31.68/24（VMware 桥接网络）
ROS 2: Jazzy
ROS_DOMAIN_ID: 0（默认）
RMW: rmw_fastrtps_cpp（默认）
NTP: synchronized
```

虚拟机已经能直接发现开发板节点，并能接收 `/scan`、`/odom`、`/tf` 和 `/tf_static`。跨机实测 `/scan` 为 `5.997 Hz`，`base_link -> laser_frame` 仍为 `(0, 0, 0.15 m)` 和 `yaw=180°`，说明 DDS、时间同步和静态 TF 跨机传输正常。Rolling 开发板与 Jazzy 虚拟机之间目前使用的均为标准消息和 TF，实测可以互通。

虚拟机已安装：

```text
ros-jazzy-slam-toolbox 2.8.5
```

新建的虚拟机工程：

```text
/home/luohjj/esp32_car_slam_ws/src/esp32_car_slam/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/slam_toolbox.yaml
├── launch/mapping.launch.py
└── rviz/mapping.rviz
```

该工程只在虚拟机运行。`mapping.launch.py` 启动异步 `slam_toolbox`，可通过 `use_rviz` 参数选择是否同时启动 RViz，不会重复启动开发板上的底盘、EKF 或雷达节点。针对当前小车和约 6 Hz 的 LD14，关键参数已调整为：

```text
odom_frame: odom
map_frame: map
base_frame: base_link
scan_topic: /scan_slam
resolution: 0.05 m
min_laser_range: 0.05 m
max_laser_range: 8.0 m
minimum_time_interval: 0.15 s
minimum_travel_distance: 0.05 m
minimum_travel_heading: 0.05 rad
```

无界面验证命令：

```bash
source /opt/ros/jazzy/setup.bash
source ~/esp32_car_slam_ws/install/setup.bash
ros2 launch esp32_car_slam mapping.launch.py use_rviz:=false
```

验证结果：

- `/slam_toolbox` 生命周期状态为 `active`。
- 已注册 `laser_frame` 雷达并接收扫描。
- 已发布 `/map`，初始地图分辨率 `0.05 m`，当时大小为 `81 x 55` 栅格。
- 已发布 `map -> odom`，车辆静止时初始变换为零。
- `/slam_toolbox/save_map`、`serialize_map` 等保存服务已存在。
- 长时间运行时约每 3 秒出现一次异步扫描队列满，约丢弃 1/18 帧；SLAM 仍稳定处理约 5.6 Hz 的有效扫描，未出现 TF 链断开，首次运动匹配正常。后续完整建图时继续观察，只有出现地图重影或跟踪丢失才需要进一步限频或调整时序。

虚拟机桌面建图启动命令：

```bash
source /opt/ros/jazzy/setup.bash
source ~/esp32_car_slam_ws/install/setup.bash
ros2 launch esp32_car_slam mapping.launch.py use_rviz:=true
```

### 20.1 ESP32 未启动时的诊断记录（已解决）

在 SLAM 初步验证过程中曾检测到 `/wheel/odom` 与 `/imu/data` 停止产生新消息。开发板串口节点仍在发送零速度命令，但接收计数停在：

```text
WHEEL=53006
IMU=105009
```

最终确认原因只是当时 ESP32 尚未启动，并非硬件或软件故障。ESP32 上电后，开发板和虚拟机同时使用轻量订阅器连续测量 10 秒，结果如下：

```text
开发板:
/scan       5.994 Hz
/wheel/odom 24.777 Hz
/imu/data   49.854 Hz
/odom       49.554 Hz

虚拟机跨机接收:
/scan       5.898 Hz
/wheel/odom 24.492 Hz
/imu/data   49.684 Hz
/odom       49.284 Hz
```

由此确认 ESP32 串口、ROS 发布、跨机 DDS 和 EKF 均恢复正常。随后重新启动了虚拟机 `slam_toolbox`，成功发布初始 `/map` 和 `map -> odom`。短时静止测试没有错误；长时间运行会偶尔丢弃单帧扫描，但未阻断建图或 TF。

RViz 已在 VMware 桌面实际启动并检查：`Global Status: Ok`，`Map`、`LD14 Scan` 和 `TF` 均正常显示，渲染约 `31 FPS`。日志中出现过一次 VMware/Mesa 的 GLSL sampler 警告，但实际地图层已经显示，因此属于非致命警告。当前正式进入低速实车建图阶段。

## 21. SLAM 首次直线运动验证（2026-07-20）

开发板重启后 DHCP 地址由 `192.168.31.28` 变为 `192.168.31.78`。ROS 节点当前未配置为系统开机自启，因此重启后手动重新启动了：

```text
localization.launch.py
lidar.launch.py
```

启动后重新实测：

```text
开发板:
/scan       5.896 Hz
/wheel/odom 24.784 Hz
/imu/data   49.668 Hz
/odom       49.268 Hz

虚拟机跨机接收:
/scan       5.898 Hz
/wheel/odom 24.292 Hz
/imu/data   47.985 Hz
/odom       47.585 Hz
```

随后通过 `/esp32_car/timed_velocity` 执行：

```text
vx = 0.10 m/s
vy = 0
wz = 0
duration = 2.0 s
目标位移约 0.20 m
```

结果：

```text
融合 /odom:
x = 0.18818 m
y = 0.00024 m
yaw ≈ 0.227°

SLAM map -> base_link:
x = 0.198 m
y = 0.000 m
yaw ≈ 0.227°

最终 /esp32_car/command:
vx = 0, vy = 0, wz = 0
```

用户用实际运动确认小车确实前进约 `20 cm`。结论：前进方向、停车逻辑、里程计积分、雷达扫描匹配和 `map -> odom -> base_link` 坐标链均正确。SLAM 将轮式/IMU 里程计的 `0.188 m` 修正到约 `0.198 m`，接近目标 `0.20 m`。RViz 实际画面中 `Global Status: Ok`，实时激光轮廓与栅格边界对齐，没有发现坐标反向或明显重影。地图宽高仍为 `81 x 55` 是因为 0.2 m 运动仍在初始地图边界内，并不代表地图未更新。

### 21.1 原地旋转验证

在直线测试终点执行逆时针旋转：

```text
wz = +0.20 rad/s
duration = 2.62 s
目标相对旋转约 30°
```

测试结果：

```text
旋转前 SLAM yaw: 0.227°
旋转后 EKF yaw: 31.76°
EKF 相对变化: 约 31.53°

旋转后 SLAM yaw: 30.357°
SLAM 相对变化: 约 30.13°

旋转后 SLAM 位置:
x = 0.198 m
y = -0.001 m
```

最终速度命令已自动归零。SLAM 航向与目标 `30°` 的误差约 `0.13°`，原地旋转时平移漂移约 `1 mm`。RViz 中旋转后的实时扫描仍与已有栅格边界贴合，没有出现双墙、扇形重影或方向反转。由此确认 IMU `gz` 正方向、雷达 `yaw=180°` 安装变换、EKF 航向与 SLAM 扫描匹配一致。

## 22. `/cmd_vel` 安全看门狗与建图遥控（2026-07-20）

在进入连续遥控建图前，为底盘节点加入了 `/cmd_vel` 超时停车保护：

```yaml
cmd_vel_timeout_sec: 0.5
```

行为如下：

- 收到 `/cmd_vel` 后记录最后更新时间。
- 超过 `0.5 s` 没有新指令时，将 `vx/vy/wz` 全部归零并只输出一次警告。
- `/esp32_car/timed_velocity` 仍由自身 `duration` 控制，不会被看门狗提前终止。
- Nav2 等连续控制器正常以高于 `2 Hz` 发布时不会触发超时。
- 遥控终端失焦、退出、网络中断或发布节点崩溃时，小车会自动停车。

构建与验证结果：

```text
esp32_car_control 编译成功
serial_protocol_test: 1/1 passed
零速度 /cmd_vel 单次发布后 0.50 s:
“/cmd_vel 超过 0.50 秒未更新，已自动停车”
最终 /esp32_car/command = 0, 0, 0
```

虚拟机 `esp32_car_slam` 工程曾新增以下遥控程序，并已完成第一轮建图验证：

```text
scripts/mapping_teleop.py
scripts/start_mapping_teleop.sh
```

运行方式：

```bash
source /opt/ros/jazzy/setup.bash
source ~/esp32_car_slam_ws/install/setup.bash
ros2 run esp32_car_slam start_mapping_teleop.sh
```

默认线速度 `0.10 m/s`，角速度 `0.20 rad/s`。专用节点按 ROS REP-103 将四向平移和旋转分别映射为 `vx/vy/wz`：

```text
W / S       前进 / 后退（vx 正/负）
A / D       左移 / 右移（vy 正/负）
Q / E       逆时针 / 顺时针旋转（wz 正/负）
K/X/空格    立即停止
Ctrl-C      停车并退出
```

`mapping_teleop.py` 以 `20 Hz` 发布速度，并有 `0.70 s` 松键超时；按住运动键才持续运动，松开后主动发布零速度。若遥控节点崩溃或网络断开，底盘端的 `0.50 s` 看门狗继续兜底停车。两层保护互不依赖。

虚拟机桌面已打开名为 `ESP32 Car Holonomic Mapping Teleop` 的终端，专用节点 `/esp32_car_mapping_teleop` 已运行，且 `/cmd_vel` 的发布端和开发板订阅端已经匹配。正式建图前已经再次重启 EKF 和 SLAM，当前地图为干净原点。

用户已经实际测试 `W/S/A/D`，确认小车前进、后退、左移、右移均可正常控制。

> 2026-07-21 架构调整：上述虚拟机遥控已停用。后续建图统一运行开发板端
> `mapping_teleop_node`，虚拟机只运行扫描预处理、Slam Toolbox 和 RViz。
> 不允许同时运行两个 `/cmd_vel` 发布端。

## 23. LD14 固定点数扫描预处理（2026-07-20）

全向短距离测试后发现 LD14 原始 `/scan` 每圈点数随转速在 `391–393` 之间变化。SLAM Toolbox 会按首帧光束数建立激光模型，之后持续输出：

```text
LaserRangeScan contains 391 range readings, expected 393
```

为避免长距离建图时扫描被拒绝或产生不一致，在虚拟机 `esp32_car_slam` 工程新增：

```text
scripts/scan_normalizer.py
```

数据链路调整为：

```text
开发板 /scan（原始 391–393 点）
  -> 虚拟机 /ld14_scan_normalizer
  -> /scan_slam（固定 360 点）
  -> slam_toolbox
  -> /map + map -> odom
```

处理原则：

- 覆盖 `0–360°`，固定为 360 个约 `1°` 的角度单元。
- 每个原始点落入最近角度单元；同一单元有多个点时保留最近障碍物。
- 无有效测距的角度保持 `inf`。
- 保留原始时间戳、`laser_frame`、量程和扫描周期。
- 原始 `/scan` 不修改，继续用于驱动和雷达诊断。
- `/scan_slam` 使用 `RELIABLE` 发布，同时兼容 SLAM 的 `BEST_EFFORT` 订阅和 RViz 的 `RELIABLE` 订阅。

连续 60 帧验证：

```text
frames = 60
unique_lengths = [360]
valid_points_min = 330
valid_points_max = 341
valid_points_avg = 336.0
```

归一化后 SLAM 日志不再出现 `expected 393` 点数警告。RViz 已改为显示 `/scan_slam`，实测紫色激光点正常显示并与地图边界贴合，`Global Status: Ok`。目前仍可能偶发异步队列丢弃单帧，但不影响扫描数量、TF 或地图发布。

## 24. 首次房间地图与保存结果（2026-07-20）

用户使用全向遥控让小车完整绕房间行驶一圈并回到起点附近。停车后状态：

```text
/esp32_car/command = 0, 0, 0
SLAM lifecycle = active
地图尺寸 = 158 x 144 cells
分辨率 = 0.05 m/cell
覆盖范围约 = 7.9 m x 7.2 m
地图原点 = [-1.781026948, -2.759290282, 0]
```

停车位置相对建图起点：

```text
EKF /odom: x=0.145 m, y=0.045 m, yaw≈-24.8°
SLAM map->base_link: x=0.155 m, y=0.070 m, yaw≈-29.2°
```

位姿图可视化包含约 `269` 个关键帧和 `381` 条图约束；纯相邻链只需 268 条，因此图中存在大量额外扫描匹配约束。需要注意：`slam_toolbox_edges` 中 `id=1` 根据官方源码表示定位模式边，在当前 mapping 模式为 0 是正常现象，不能用它判断是否发生自动回环。

运行期间：

- 固定 360 点扫描无点数警告。
- 没有 TF 断链或 SLAM 错误。
- 整圈只记录到一次异步队列单帧丢弃。
- RViz 中当前紫色扫描与起点附近已建墙线贴合。
- 保存后的地图主要墙体连续，没有发现整圈平移、扇形重影或明显双墙。

地图保存目录：

```text
/home/luohjj/esp32_car_slam_ws/maps/
├── room_20260720.pgm        # Nav2/AMCL 栅格图像，158 x 144
├── room_20260720.yaml       # 分辨率、原点和占用阈值
├── room_20260720.posegraph  # Slam Toolbox 无损位姿图，约 12 MB
└── room_20260720.data       # 位姿图关联扫描数据，约 3.7 MB
```

`serialize_map` 服务返回 `result=0`，位姿图保存成功。系统缺少 `nav2_map_server` 且当前 SSH 会话的 `sudo` 需要密码，因此 Slam Toolbox 自带 `save_map` 返回 `255`。为不依赖管理员权限，虚拟机工程新增 `scripts/save_map.py`，直接订阅 `/map` 并按 Nav2 标准生成 PGM/YAML：

```bash
source /opt/ros/jazzy/setup.bash
source ~/esp32_car_slam_ws/install/setup.bash
ros2 run esp32_car_slam save_map.py \
  --output ~/esp32_car_slam_ws/maps/room_20260720
```

PGM 已独立读取和放大渲染验证：尺寸 `158 x 144`、灰度范围 `0–254`，图像方向、未知区、可通行区和墙体轮廓正确。

下一阶段：

1. 安装 Jazzy 的 Nav2 与 map server（需要用户在虚拟机输入 sudo 密码）。
2. 重载 `room_20260720.yaml` 并在 RViz 验证静态地图。
3. 配置全向轮底盘 footprint、速度限制和 Nav2 控制器。
4. 使用 AMCL 或 Slam Toolbox localization 模式定位。
5. 完成目标点规划、避障和导航实车测试。

## 25. 建图遥控迁移到开发板（2026-07-21）

为减少从 VMware 虚拟机经 DDS 发送控制命令带来的操作延迟，建图控制链路调整为：

```text
开发板 SSH 键盘
  -> /esp32_car_mapping_teleop
  -> /cmd_vel（20 Hz）
  -> esp32_car_node
  -> ESP32 串口

虚拟机
  -> /scan_slam + slam_toolbox + RViz（只观察和建图，不发布控制命令）
```

开发板工程新增独立 C++ 节点：

```text
/home/luohjj/esp32_car_project/control_node/src/mapping_teleop_node.cpp
```

启动命令（必须在开发板交互式 SSH 终端中执行）：

```bash
source /opt/ros/rolling/setup.bash
source /home/luohjj/esp32_car_project/control_node/install/setup.bash
ros2 run esp32_car_control mapping_teleop_node
```

控制键：

```text
W / S       前进 / 后退
A / D       左移 / 右移
Q / E       逆时针 / 顺时针旋转
K / X / 空格 立即停车
Ctrl-C      停车并退出
```

安全机制：

- 默认线速度 `0.10 m/s`，角速度 `0.20 rad/s`。
- 运动期间本地节点以 `20 Hz` 发布 `/cmd_vel`。
- 最后一次运动按键后 `0.70 s` 没有新按键，遥控节点主动停车。
- 遥控进程退出时发布零速度。
- 如果遥控进程卡死或 SSH 断开，底盘节点原有 `0.50 s` 看门狗兜底停车。
- 同一时间只能有一个 `/cmd_vel` 控制源；运行 Nav2 前必须退出手动遥控。

虚拟机原有 `mapping_teleop.py` 和 `start_mapping_teleop.sh` 已从源码及安装目录移除。
虚拟机包当前只安装 `scan_normalizer.py` 和 `save_map.py`，不再包含任何控制程序。

构建与非运动验证结果：

- 开发板 `esp32_car_control` 编译成功，`mapping_teleop_node` 已正确安装。
- `serial_protocol_test` 通过；新遥控源码通过 `ament_uncrustify` 检查。
- 交互式 SSH 启动成功，节点名为 `/esp32_car_mapping_teleop`。
- ROS 图确认该节点发布标准 `geometry_msgs/msg/Twist` 到 `/cmd_vel`。
- 验证过程未输入任何运动键，退出后节点和发布端均已清理。
- 虚拟机 `esp32_car_slam` 清理后重新构建成功，包内仅保留地图保存和扫描预处理可执行程序。

### 25.1 删除旧键盘控制（2026-07-21）

`esp32_car_node.cpp` 中原有的键盘控制已完全移除，包括：

- `/dev/tty` 按键读取类。
- `W/S/A/D/Q/E` 预设速度逻辑。
- `Enter` 启动定时运动逻辑。
- 旧的终端全屏显示线程和按键帮助界面。
- `linear_step`、`angular_step` 旧参数及相关状态变量。

保留功能为串口收发、传感器发布、`/cmd_vel` 订阅、安全看门狗和
`/esp32_car/timed_velocity` 服务。开发板端交互式控制只保留独立的
`mapping_teleop_node`，从而保证键盘输入和底盘串口节点职责分离。

验证结果：

- `esp32_car_control` 重新编译成功，新主节点二进制生成时间为 2026-07-21 14:59。
- `esp32_car_node.cpp` 和 `mapping_teleop_node.cpp` 均通过 `ament_uncrustify`。
- `serial_protocol_test` 1/1 通过。
- 源码和参数文件已确认不存在旧 Keyboard 类、按键处理函数或旧速度步长参数。
- 修改时底盘主节点已于 14:53 启动，因此当时运行中的进程仍使用旧内存映像；
  下次重新启动 `localization.launch.py` 后加载精简后的新二进制。

### 25.2 降低 SSH 松键停车延迟（2026-07-21）

SSH 终端只传输按键字符，不传输 key-up（按键松开）事件。原实现统一等待
`0.70 s` 判断松手，因此车辆停车存在明显延迟。遥控节点已改为自适应超时：

- 首次按键仍允许 `0.70 s`，用于跨过操作系统首次键盘重复延迟。
- 检测到同一运动键连续重复后，松手判断切换为 `0.12 s`。
- `/cmd_vel` 发布和按键轮询频率由 `20 Hz` 提高到 `50 Hz`。
- 切换运动方向时重新检测按键重复，避免把不同方向误判为长按。
- `K/X/空格` 仍立即停车，不经过任何超时。
- 底盘节点 `0.50 s` 看门狗继续处理遥控程序崩溃或断联，不参与正常松手停车。

受 SSH 终端协议限制，无法获得真正的零延迟 key-up；连续按键模式下的理论停车
判断延迟已由 `0.70 s` 降至约 `0.12 s`，再加最多一个 `20 ms` 定时器周期。

### 25.3 增加斜向平移（2026-07-21）

原遥控节点每次收到新按键都会用单轴命令覆盖上一条命令，因此同时按方向键时
只能依次运动。曾尝试增加多键组合控制：

```text
W + A  左前斜移      W + D  右前斜移
S + A  左后斜移      S + D  右后斜移
U      左前斜移      O      右前斜移
M      左后斜移      .      右后斜移
```

`W+D` 产生 `vx>0、vy<0、wz=0`，属于右前方平移；`W+E` 产生
`vx>0、wz<0`，属于前进同时顺时针旋转，两者含义不同。

后续频繁切换方向实测发现，SSH 终端不具备真正的多键 key-down/key-up 状态，
组合窗口会保留上一条命令的速度分量，可能造成仅按前进却夹带横移或旋转。
因此多键合并已在 25.4 节中撤销，最终只保留单键斜移 `U/O/M/.`。

### 25.4 消除方向切换残留（2026-07-21）

最终遥控策略改为“一个按键生成一条完整命令”：

- `W/S` 只设置 `vx`，强制 `vy=0、wz=0`。
- `A/D` 只设置 `vy`，强制 `vx=0、wz=0`。
- `Q/E` 只设置 `wz`，强制 `vx=0、vy=0`。
- `U/O/M/.` 分别一次性设置明确的 `vx+vy` 斜移组合，并强制 `wz=0`。
- 每次切换按键都从全零 `Twist` 重建命令，不继承上一方向的任何分量。

已删除 `W+A/W+D/S+A/S+D` 的多键组合窗口与粘滞状态。该方案牺牲 SSH
多键组合，但能保证快速切换任意方向时绝不因软件状态残留而意外侧移或旋转。

### 25.5 运行时调速（2026-07-21）

开发板遥控节点新增运行时速度档位和增减速按键：

```text
1  低速档：0.10 m/s，0.20 rad/s
2  中速档：0.20 m/s，0.40 rad/s（默认）
3  高速档：0.35 m/s，0.70 rad/s
+  每次增加 0.05 m/s 和 0.10 rad/s
-  每次减少 0.05 m/s 和 0.10 rad/s
```

遥控软件限制最高线速度 `0.60 m/s`、最高角速度 `1.20 rad/s`，最低分别为
`0.05 m/s` 和 `0.10 rad/s`。调速或换档时先发布零速度，防止运动过程中速度
突然跳变；新的速度从下一次方向按键开始生效。当前速度同步写入节点参数
`linear_speed` 和 `angular_speed`，可通过 `ros2 param get` 查询。

## 26. SLAM地图分辨率调整（2026-07-21）

虚拟机Slam Toolbox输出地图分辨率已由 `0.05 m/cell` 调整为：

```yaml
resolution: 0.025
```

配置文件：

```text
/home/luohjj/esp32_car_slam_ws/src/esp32_car_slam/config/slam_toolbox.yaml
```

源码和安装目录均已核对为 `0.025 m/cell`，`esp32_car_slam`重新构建成功。
扫描匹配参数 `correlation_search_space_resolution: 0.01` 和
`loop_search_space_resolution: 0.05` 保持不变。新分辨率只对下次重新启动的
建图会话生效；原有 `room_20260720` 地图仍为 `0.05 m/cell`。

## 27. Nav2 安装与 AMCL 定位入口（2026-07-21）

虚拟机已经安装 ROS 2 Jazzy 的 Nav2。已确认以下包均来自
`/opt/ros/jazzy`：

```text
nav2_map_server
nav2_amcl
nav2_lifecycle_manager
nav2_bringup
```

虚拟机工程新增：

```text
config/amcl.yaml               # Map Server 与 AMCL 参数
launch/localization.launch.py  # 静态地图、扫描归一化、AMCL、可选 RViz
rviz/localization.rviz         # 地图、雷达、粒子云及 2D Pose Estimate
```

定位使用的坐标链为：

```text
map --AMCL--> odom --robot_localization--> base_link --static TF--> laser_frame
```

AMCL 已配置为 `nav2_amcl::OmniMotionModel`，使 `vx`、`vy` 和转动都能进入
粒子运动模型；输入雷达话题为固定 360 点的 `/scan_slam`。AMCL 只负责定位和
发布 `map -> odom`，不会发布 `/cmd_vel`。

默认地图为：

```text
/home/luohjj/esp32_car_slam_ws/maps/room_0025.yaml
分辨率：0.025 m/cell
```

启动命令：

```bash
source /opt/ros/jazzy/setup.bash
source ~/esp32_car_slam_ws/install/setup.bash
ros2 launch esp32_car_slam localization.launch.py use_rviz:=true
```

注意：定位模式和 `mapping.launch.py` 不能同时运行，否则会有两个节点同时发布
`map -> odom`。每次车辆初始位置未知时，都要在 RViz 使用 **2D Pose Estimate**
给出大致位置和车头方向，然后通过少量平移或旋转观察粒子收敛、雷达点贴墙。

无界面启动验证结果：

- `map_server` 与 `amcl` 生命周期状态均为 `active [3]`。
- `room_0025.pgm` 成功加载为 `481 x 292`，分辨率为 `0.025 m/cell`。
- 地图原点为 `[-4.03853837, -1.13577572, 0]`。
- `/scan_slam` 稳定约 `6.0 Hz`。
- AMCL 运行时参数确认：运动模型为 `nav2_amcl::OmniMotionModel`、扫描话题为
  `/scan_slam`、底盘坐标系为 `base_link`。
- 在未发送初始位姿前，AMCL 提示无法发布位姿和 `map -> odom`，属于预期行为。
- 测试进程验证后已正常停止，未向 `/cmd_vel` 发布任何运动命令。

下一步只需在虚拟机桌面启动带 RViz 的定位入口，使用 **2D Pose Estimate** 完成
实车初始定位并观察收敛；这一项完成后再配置 Nav2 的 footprint、代价地图、规划器
和全向控制器。

### 27.1 AMCL 实车收敛验证（2026-07-21）

用户在 RViz 使用 **2D Pose Estimate** 给出初始位姿。刚设置时实时雷达点与地图
并不完全重合；使用开发板端遥控让小车行驶一段路径后，AMCL 粒子通过连续雷达
扫描和里程计更新逐步收敛，最终雷达点与地图墙体基本重合。这个过程符合 AMCL
从近似初始位姿向真实位姿收敛的预期行为。

远程检查结果：

- `/amcl_pose` 已正常发布。
- `map -> odom -> base_link` 完整连通。
- 停车后 `map -> base_link` 连续数秒保持在约
  `x=0.182 m、y=-0.069 m、yaw=13.14 deg`，没有继续漂移。
- 最后一帧 AMCL 估计约为 `x=0.216 m、y=-0.061 m、yaw=13.14 deg`。
- 遥控节点退出后 `/cmd_vel` 发布者数量为 `0`，底盘节点仍保留一个订阅端，
  当前不会收到残留运动命令。

结论：静态地图重载、AMCL 初始定位、运动收敛和完整 TF 链均已通过实车验证。
下一阶段为 Nav2 车体轮廓、全局/局部代价地图、规划器和全向控制器配置。

本次修改前的虚拟机包备份：

```text
/home/luohjj/.codex_backups/esp32_car_slam_before_amcl_localization_20260721
```

## 28. Nav2 全向导航配置（2026-07-21）

实测车体为前后 `0.30 m`、左右 `0.30 m` 的正方形，`base_link` 位于几何
中心。因此全局与局部代价地图统一使用以下 footprint：

```yaml
footprint: "[[0.15, 0.15], [0.15, -0.15], [-0.15, -0.15], [-0.15, 0.15]]"
footprint_padding: 0.02
```

虚拟机工程新增：

```text
config/nav2_params.yaml         # Nav2 全向控制和安全参数
launch/navigation.launch.py     # 只启动规划、控制与避障，复用现有 AMCL
launch/nav2.launch.py           # 一条命令启动地图、AMCL、Nav2 和 RViz
rviz/navigation.rviz            # 目标点、路径、代价地图、footprint 显示
```

主要配置：

- 局部/全局地图分辨率均为 `0.025 m/cell`。
- 障碍膨胀半径 `0.30 m`，膨胀代价系数 `5.0`。
- 全局规划器：`NavfnPlanner`，启用 A*。
- 局部控制器：`nav2_mppi_controller::MPPIController`。
- MPPI 运动模型：`Omni`，允许 `vx`、`vy` 和 `wz`。
- X/Y 最大速度均为 `0.20 m/s`，最大角速度 `0.50 rad/s`。
- 控制频率与 MPPI 模型步长匹配为 `20 Hz` 和 `0.05 s`。
- `/scan_slam` 同时用于局部/全局动态障碍层与 collision monitor。
- 控制链：`controller/behavior -> cmd_vel_nav -> velocity_smoother ->
  cmd_vel_smoothed -> collision_monitor -> /cmd_vel`。

无目标启动测试结果：

- `controller_server`、`smoother_server`、`planner_server`、
  `behavior_server`、`velocity_smoother`、`collision_monitor`、
  `bt_navigator` 和 `waypoint_follower` 均进入 `active [3]`。
- MPPI 运行时参数确认：`Omni`、`vx_max=0.2`、`vy_max=0.2`、
  `wz_max=0.5`。
- 运行时 footprint、padding 和 inflation radius 分别确认为
  `0.30 x 0.30 m`、`0.02 m` 和 `0.30 m`。
- `/global_costmap/costmap`、`/local_costmap/costmap` 与
  `/local_costmap/published_footprint` 均有发布端。
- `navigate_to_pose`、`follow_path`、`compute_path_to_pose` 等动作接口已出现。
- 无目标测试期间 `/odom` 的 `vx/vy/wz` 近似为零，小车未运动。
- 测试后已停止导航部分，当前 `/cmd_vel` 发布者恢复为 `0`，AMCL 定位会话
  保持运行。

调试中修正了两个 Jazzy 参数约束：costmap 的 `width/height` 必须是整数类型，
MPPI 的控制周期不能大于 `model_dt`。最终配置已重新编译并通过生命周期测试。

后续常规完整启动命令：

```bash
source /opt/ros/jazzy/setup.bash
source ~/esp32_car_slam_ws/install/setup.bash
ros2 launch esp32_car_slam nav2.launch.py start_rviz:=true
```

首次设置初始位姿后，Localization 已为 `active` 且完整 TF 建立，但导航自动启动
此前已在等待 `map -> base_link` 时超时，形成 controller/smoother 为 `active`、
planner 及后续节点为 `inactive` 的部分启动状态。导航生命周期管理器因此改为
`autostart: false`：先由 AMCL 建立 `map -> odom`，再点击 RViz Navigation 2
面板的 **Startup** 一次性配置和激活全部导航节点。这样启动顺序具有确定性。

首次实车测试只选择车前方约 `0.30 m` 的空旷目标点，先验证路径生成、低速起步、
到点停车和取消目标，再扩大导航距离。

### 28.1 首次导航 RViz 无地图/雷达问题（已修复）

首次单独启动 `navigation.launch.py` 时，原定位会话已经退出，因此当时
`/map` 与 `/scan_slam` 的发布者数量均为 `0`。RViz 只能显示剩余 TF，属于
数据源未启动，不是地图文件损坏。后续在没有现成定位会话时应直接使用完整的
`nav2.launch.py`，它会同时启动 map server、扫描归一化、AMCL 和导航节点。

同时修正 `navigation.rviz` 的 QoS：

- `/map`、全局代价地图和局部代价地图使用 `Transient Local + Reliable`。
- `/scan_slam` 与 AMCL 粒子云使用 `Volatile + Best Effort`。

这样新打开的 RViz 可以收到 map server 已经发布过的静态地图，并能与传感器
数据 QoS 正确匹配。

组合启动还曾存在两个子入口共用 `use_rviz` 参数的问题：定位入口为了避免重复
窗口传入的 `false` 可能覆盖导航入口的 RViz 开关。导航子入口后来改用独立参数
`use_navigation_rviz`，但最终完整入口改为直接管理唯一 RViz，不再依赖该参数。

用户看到“AMCL cannot publish a pose”与“waiting for transform”后曾按下
`Ctrl-C`，启动日志明确记录 `user interrupted with ctrl-c (SIGINT)`。这些提示在
设置 **2D Pose Estimate** 前属于预期状态，不应退出启动进程。

再次实测发现，虽然所有地图、AMCL 和导航节点均启动，子入口参数作用域仍会使
RViz 条件最终为 false，进程列表中没有 `rviz2`。最终方案不再让任何子入口负责
完整导航界面：最外层 `nav2.launch.py` 直接启动唯一一个 `navigation.rviz`，并
使用独立开关 `start_rviz`。完整启动命令更新为：

```bash
ros2 launch esp32_car_slam nav2.launch.py start_rviz:=true
```

本阶段修改前备份：

```text
/home/luohjj/.codex_backups/esp32_car_slam_before_nav2_navigation_20260721
```

### 28.2 雷达地图精确匹配与起点不可规划问题（2026-07-21）

首次自动导航测试出现路径失败和恢复旋转。排查确认雷达、静态 TF 与地图文件本身
没有方向错误，主要有两个独立原因：

1. 人工 `2D Pose Estimate` 存在约 `4 cm / 2 cm / 7 deg` 偏差。7 度误差会让
   2 米外的雷达点横向错开约 24 cm，因此 RViz 中看起来明显不重合。
2. 实体车左后方障碍距离底盘中心仅约 `0.35 m`。全局代价地图中机器人起点
   代价达到 `99`（内切膨胀区），NavFn 返回 `NO_VALID_PATH (208)`。

新增只读扫描匹配诊断，对 `/scan_slam` 端点与 `room_0025.pgm` 的占用栅格做
距离场搜索。使用 `/set_initial_pose` 服务写入匹配位姿，并调用
`/request_nomotion_update` 强制 AMCL 在静止状态执行一次激光更新。修正后结果：

- AMCL 收敛位姿约为 `x=-0.291 m, y=0.364 m, yaw=12.5 deg`。
- 雷达端点到静态障碍的平均误差约 `1.2 cm`，中位误差为 `0`。
- 车辆后续移动后再次匹配，平均误差约 `0.4 cm`，最佳修正量接近零。
- 这证明 `laser_frame -> base_link` 的 180 度安装角、雷达数据和静态地图正确。

净空判断也由“正方向扇区最短射线”改为“方形车体扫掠区域”。按 `0.30 x 0.30 m`
footprint 加 `0.05 m` 安全边距计算时，当时左移安全距离仅 `0.089 m`，不能执行
0.20 m 左移；右移安全距离约 `0.55 m`。导航暂停后通过
`/esp32_car/timed_velocity` 以 `vy=-0.10 m/s` 运行 `1.5 s`，实际右移约
`0.142 m`，朝向基本不变。移动后起点代价从 `99` 降到 `88`，成功退出不可规划区。

重要经验：选择短测试目标时必须检查完整 footprint 的扫掠净空，不能只看目标
方向的一束或一小段雷达扇区；起点代价为 `99` 时应先低速人工脱困，不能让 Nav2
反复执行恢复行为。

### 28.3 底盘速度死区适配与首次闭环导航成功（2026-07-21）

初始 MPPI 在 0.20 m 近距离目标下最大只输出约 `0.008 m/s`，低于 ESP32 底盘
约 `0.05 m/s` 的线速度起步阈值，因此控制链正常但电机不动。曾短暂测试
`VelocityDeadbandCritic`，发现它会同时惩罚 `vx/vy/wz` 的零值；对三自由度
全向轮底盘会强迫无关轴运动，实车表现为左移同时附带旋转，因此已完全移除。

虚拟机包新增：

```text
scripts/cmd_vel_deadband_compensator.py
```

新控制链为：

```text
controller / behavior
  -> /cmd_vel_nav_raw
  -> cmd_vel_deadband_compensator
  -> /cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel_smoothed
  -> collision_monitor
  -> /cmd_vel
```

适配规则：

- `vx/vy` 作为一个平移向量整体处理，保持原方向，不分别强迫各轴非零。
- 平移幅值不超过 `0.005 m/s` 视为优化噪声并置零。
- 明确非零但低于 `0.05 m/s` 时，等比例放大到 `0.05 m/s`。
- 角速度不超过 `0.03 rad/s` 视为噪声并置零。
- 明确旋转但低于 `0.10 rad/s` 时补偿到 `0.10 rad/s`。
- 后级速度平滑器、加速度限制、碰撞监视器和 ESP32 看门狗继续保留。

闭环测试结果：

1. 0.15 m 前方目标：Nav2 返回 `SUCCEEDED`，无恢复；由于
   `xy_goal_tolerance=0.10 m`，实际前进约 `0.053 m` 后正常判定到达。
2. 将角速度噪声阈值提高到 `0.03 rad/s` 并重启后，0.20 m 前方目标的全局路径
   计算成功（14 个路径点、长度 `0.198 m`）。实车导航返回 `SUCCEEDED`，无恢复，
   实际前进约 `0.103 m`、横向约 `0.007 m`。
3. 第二次测试中 MPPI 原始角速度噪声最大约 `0.014 rad/s`，适配后
   `/cmd_vel_nav` 角速度始终为 `0`，直行不再被强制旋转。
4. 最终 `/odom` 的 `vx/vy/wz` 已恢复近零，车辆确认停车。

测试结束时 AMCL 位姿约为：

```text
x=-0.094 m, y=0.248 m, yaw约12.5 deg
```

雷达扫掠安全距离约为：前 `0.224 m`、后 `0.630 m`、左 `0.257 m`、右
`0.397 m`。导航节点当前为 active，完整启动后仍遵循“先设置初始位姿，再点击
Startup”的顺序。

下一步建议：

1. 在更开阔位置发送 0.5--1.0 m 目标，验证持续跟踪、减速和到点停车。
2. 根据实际导航精度决定是否把 `xy_goal_tolerance` 从 `0.10 m` 收紧到 `0.05 m`。
3. 验证纯横移、组合斜移和原地旋转目标，分别检查三自由度速度适配。
4. 最后再测试动态障碍停车、目标取消和 Nav2 恢复行为。

### 28.4 手动换位后的重新定位与受限空间前进测试（2026-07-21）

用户将车辆人工搬到一个只有前方约 1 m 可通行、左右和后方均有障碍的位置。
由于搬动车辆不会更新轮式里程计，原 AMCL 位姿已经失效。对当前雷达与静态地图
重新执行扫描匹配后，将位姿由约 `(-0.070, 0.248, 10.2 deg)` 修正为
`(-0.530, -0.152, 10.2 deg)`；匹配端点到地图障碍的平均误差由约 `17.7 cm`
降至约 `1.2 cm`，证明人工换位后必须重新设置初始位姿。

没有直接执行用户给出的 1 m 前进距离。规划前检查结果如下：

- 1.0 m 前方目标的全局代价为 `99`，终点已落入障碍物内切膨胀区，禁止执行。
- 0.6 m 目标虽然可以生成路径，但路径相对车体中心线最大横移约 `4.7 cm`；当时
  左侧 footprint 扫掠安全余量只有约 `5.9 cm`，因此仍然拒绝执行。
- 0.4 m 目标路径长度约 `0.433 m`，相对中心线最大偏移约 `2.7 cm`，通过执行前
  安全检查。

最终发送 0.4 m 前方目标，Nav2 返回 `SUCCEEDED`，错误码为 0，恢复行为次数为
0。受 `xy_goal_tolerance=0.10 m` 影响，轮式里程计记录实际位移约：

```text
前向 0.310 m
横向 0.005 m
```

此次控制过程中，死区补偿后的最大速度约为 `vx=0.070 m/s`、
`vy=0.017 m/s`，角速度保持为 0；动作完成后复查 `/odom`，三个速度分量均已
回到数值零，车辆正常停稳。最终 AMCL 位姿约为
`x=-0.199 m, y=-0.115 m, yaw=9.6 deg`。

停车后的雷达最近点约为 `0.302 m`，位于车体右后方；按 `0.30 x 0.30 m`
footprint 加 `0.05 m` 安全边距计算，右移安全距离仅约 `0.055 m`。因此当前位置
不再继续横向测试。后续测试仍必须遵循：人工搬车后先重新定位；发送目标前同时
检查目标代价、完整路径以及 footprint 扫掠余量，不能仅凭肉眼估计或单束雷达
距离直接动车。

### 28.5 终点朝向环绕问题与 Rotation Shim 修复（2026-07-25）

使用新地图 `/home/luohjj/esp32_car_slam_ws/maps/room_20260725.yaml`
（分辨率 `0.025 m`，尺寸 `439 x 368`）测试导航时发现：目标位置可以到达，
但只要目标箭头朝向与车辆当前朝向存在一定夹角，车辆就可能在目标点附近沿圆弧
持续运动，车头无法收敛到指定朝向，随后还会逐渐远离目标。日志中没有
`Running spin`，因此这不是 Nav2 恢复行为中的 Spin，而是 Omni MPPI 在接近
终点时仍同时优化平移和旋转，未形成稳定、独立的终点原地转向阶段。

已在虚拟机的 `esp32_car_slam` 包中将 `FollowPath` 改为：

```text
nav2_rotation_shim_controller::RotationShimController
  -> primary_controller:
     nav2_mppi_controller::MPPIController（motion_model=Omni）
```

关键参数如下：

```yaml
FollowPath:
  plugin: "nav2_rotation_shim_controller::RotationShimController"
  primary_controller: "nav2_mppi_controller::MPPIController"
  angular_dist_threshold: 3.10
  angular_disengage_threshold: 0.10
  rotate_to_heading_angular_vel: 0.20
  max_angular_accel: 0.80
  rotate_to_goal_heading: true
  rotate_to_heading_once: true
  closed_loop: true
```

`angular_dist_threshold` 特意设为接近 pi：除非路径方向几乎与车头完全相反，
起步阶段不强制预旋转，从而继续保留全向轮底盘横移和斜移能力；进入
`xy_goal_tolerance=0.10 m` 后则由 Rotation Shim 接管终点朝向，锁住平移，
只发送角速度，直到进入 `yaw_goal_tolerance=0.15 rad`。

工程依赖已增加 `nav2_rotation_shim_controller`，并已重新执行：

```bash
colcon build --packages-select esp32_car_slam --symlink-install
```

运行期无运动验证已通过：

- `FollowPath` 成功创建为 `RotationShimController`。
- 内部控制器成功创建为 `MPPIController`，且 MPPI 配置完成。
- 参数查询确认 `rotate_to_goal_heading=true`、终点旋转速度为
  `0.20 rad/s`、起步旋转阈值为 `3.10 rad`。
- 验证只把控制器配置到 `inactive`，随后清理回 `unconfigured`。
- 验证结束后 `/cmd_vel` 发布者数量为 0，`/odom` 的 `vx/vy/wz` 均为数值零，
  没有驱动车辆。

修改前完整源码备份位于：

```text
/home/luohjj/.codex_backups/esp32_car_slam_before_rotation_shim_20260725
```

Nav2 已使用新配置重新启动，当前等待用户重新设置初始位姿。下一步实车验证顺序：

1. 使用 `2D Pose Estimate` 设置初始位姿，确认雷达点与静态地图重合。
2. 点击 Navigation 2 面板中的 `Startup`。
3. 先在开阔区域测试约 `0.3 m` 的短目标，目标朝向仅改变 `30--45 deg`。
4. 观察车辆应先完成位置移动，然后停止平移并在原地收敛终点朝向。
5. 通过后再依次测试 `90 deg` 和 `180 deg` 朝向差；测试期间保留取消导航和断电
   手段，不在障碍物附近首次验证。

## 29. 虚拟机工作区迁移到桌面（2026-07-26）

虚拟机实际使用中文 XDG 桌面目录 `/home/luohjj/桌面`。用户指定的旧桌面
`ros2_ws` 已清空，原 SLAM/Nav2 工作区已从：

```text
/home/luohjj/esp32_car_slam_ws
```

完整迁移到：

```text
/home/luohjj/桌面/ros2_ws
```

当前目录职责：

```text
/home/luohjj/桌面/ros2_ws/src/esp32_car_slam  # 工程源码
/home/luohjj/桌面/ros2_ws/maps                 # PGM/YAML/posegraph/data
/home/luohjj/桌面/ros2_ws/build                # colcon 构建输出
/home/luohjj/桌面/ros2_ws/install              # ROS 安装空间
/home/luohjj/桌面/ros2_ws/log                  # 构建与运行日志
```

迁移后清除了包含旧绝对路径的 `build/install` 缓存，并在新路径执行：

```bash
cd /home/luohjj/桌面/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select esp32_car_slam --symlink-install
source install/setup.bash
```

构建成功。`localization.launch.py` 与 `nav2.launch.py` 已改为根据包的安装位置
自动推导工作区根目录，不再写死旧的 `~/esp32_car_slam_ws`；README 内所有操作
命令也已更新为桌面新路径。运行验证确认：

- `ros2 pkg prefix esp32_car_slam` 返回
  `/home/luohjj/桌面/ros2_ws/install/esp32_car_slam`。
- `nav2.launch.py --show-args` 默认地图正确解析为
  `/home/luohjj/桌面/ros2_ws/maps/room_0025.yaml`。
- 最新 `room_20260725` 的 `.yaml/.pgm/.posegraph/.data` 均完整保留。
- 所有 Python 启动文件和脚本通过语法编译检查。
- 原目录 `/home/luohjj/esp32_car_slam_ws` 已不存在。

迁移后的常用环境加载命令统一为：

```bash
source /opt/ros/jazzy/setup.bash
source ~/桌面/ros2_ws/install/setup.bash
```

## 30. 开发板项目目录统一整理（2026-07-26）

开发板端与当前小车有关的两个 ROS 2 工作区已统一归档到：

```text
/home/luohjj/esp32_car_project
```

最终结构：

```text
/home/luohjj/esp32_car_project/
├── README.md
├── control_node/       # 底盘串口、传感器、EKF、TF、启动和建图遥控
└── ldlidar_ros2_ws/    # LD14 雷达驱动
```

原路径：

```text
/home/luohjj/control_node
/home/luohjj/ldlidar_ros2_ws
```

已不存在。`/home/luohjj/ros2_ws/src/my_pkg` 与本项目无关，迁移过程中未修改。

迁移前没有相关 ROS 节点运行。迁移后清除了两个工作区中包含旧绝对路径的
`build/install` 缓存，并按照“先雷达驱动、后底盘工程”的顺序重新构建：

```bash
cd ~/esp32_car_project/ldlidar_ros2_ws
source /opt/ros/rolling/setup.bash
colcon build --symlink-install

cd ~/esp32_car_project/control_node
source /opt/ros/rolling/setup.bash
source ~/esp32_car_project/ldlidar_ros2_ws/install/setup.bash
colcon build --packages-select esp32_car_control --symlink-install
```

两个工程均构建成功。LD14 仍只有原驱动中的枚举与浮点运算弃用警告，不影响
运行。验证结果：

- `serial_protocol_test`：`1/1` 通过。
- `ros2 pkg prefix esp32_car_control`：
  `/home/luohjj/esp32_car_project/control_node/install/esp32_car_control`。
- `ros2 pkg prefix ldlidar_sl_ros2`：
  `/home/luohjj/esp32_car_project/ldlidar_ros2_ws/install/ldlidar_sl_ros2`。
- `esp32_car_node` 和 `mapping_teleop_node` 均正确安装。
- `localization.launch.py` 与 `lidar.launch.py` 参数解析成功，ESP32 默认串口仍为
  `/dev/ttyUSB0`，LD14 稳定串口路径保持不变。

完整 `colcon test` 中，Rolling 的 `ament_lint_auto` 会递归扫描工作区根目录下
刚生成的 `install/` 接口代码，因生成文件格式产生大量 lint 失败；这是当前
“工作区根目录同时也是包根目录”布局下的检查范围问题，不是编译、串口协议或
运行功能失败。项目自己的串口协议单元测试已单独确认通过。

根目录新增 `README.md`，统一记录两个工作区的构建、环境加载和三个常用启动
入口。后续开发板端路径统一使用：

```bash
source /opt/ros/rolling/setup.bash
source ~/esp32_car_project/ldlidar_ros2_ws/install/setup.bash
source ~/esp32_car_project/control_node/install/setup.bash
```

## 31. 增加 base_footprint 并按三层车体实测高度重构 TF（2026-07-26）

为区分“车辆在地面的二维运动基准”和“实体车体中间层”，新增
`base_footprint`。当前采用的实测安装数据为：

```text
base_footprint -> base_link
translation = (0.00, 0.00, 0.11 m)

base_link -> imu_link
translation = (0.00, -0.05, 0.06 m)
rotation = (0, 0, 0)

base_link -> laser_frame
translation = (0.00, 0.00, 0.07 m)
yaw = pi rad
```

因此 IMU 绝对离地高度为 `0.17 m`，雷达扫描平面绝对离地高度为
`0.18 m`。IMU 位于车体右侧 5 cm；雷达仍位于车体平面中心，且其 0°/X
方向相对车头旋转 180°。

完整 TF 链调整为：

```text
map
└── odom
    └── base_footprint
        └── base_link
            ├── imu_link
            └── laser_frame
```

开发板修改：

- `config/ekf.yaml`：EKF 的 `base_link_frame` 改为 `base_footprint`。
- `config/esp32_car.yaml`：轮式里程计的 `base_frame` 改为
  `base_footprint`。
- `src/sensor_publisher.cpp`：`/wheel/odom.child_frame_id` 默认改为
  `base_footprint`。
- `launch/esp32_car.launch.py`：新增
  `base_footprint -> base_link`，并更新 IMU 高度。
- `launch/lidar.launch.py`：更新雷达相对第二层的高度。

虚拟机修改：

- `config/slam_toolbox.yaml`：`base_frame: base_footprint`。
- `config/amcl.yaml`：`base_frame_id: base_footprint`。
- `config/nav2_params.yaml`：BT Navigator、局部/全局代价地图、行为服务器和
  Collision Monitor 全部改用 `base_footprint`。
- 车体 footprint 仍为以地面投影中心为基准的 `0.30 x 0.30 m` 正方形。
- `scripts/scan_normalizer.py`：退出时先检查 ROS 上下文，修复正常 Ctrl+C
  停止临时 SLAM 时重复调用 `rclpy.shutdown()` 的异常堆栈。

构建与运行验证：

- 开发板 `esp32_car_control` 构建成功，`serial_protocol_test` 为 `1/1`
  通过。
- 虚拟机 `esp32_car_slam` 构建成功。
- `/cmd_vel` 发布者为 `0`，验证过程没有下发运动命令。
- 静止 `/odom` 的 `child_frame_id` 为 `base_footprint`，速度均为零。
- `/odom` 和 `/imu/data` 均约 `50 Hz`，`/scan` 为 `6.00 Hz`。
- `tf2_echo` 实测 `base_footprint -> imu_link` 为
  `(0, -0.05, 0.17 m)`。
- `tf2_echo` 实测 `base_footprint -> laser_frame` 为
  `(0, 0, 0.18 m)`、Yaw `180°`。
- 虚拟机短时启动无界面建图后，Slam Toolbox 参数实际为
  `base_frame=base_footprint`，并成功查询
  `map -> base_footprint -> base_link -> laser_frame`。
- 临时建图进程经 SIGINT 停止后，扫描归一化节点和 Slam Toolbox 均干净退出。

修改前备份：

```text
开发板：/home/luohjj/.codex_backups/control_node_before_base_footprint_20260726
虚拟机：/home/luohjj/.codex_backups/esp32_car_slam_before_base_footprint_20260726
```
