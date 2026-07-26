# ESP32 小车虚拟机端 SLAM 工程

本工程只运行在 VMware Ubuntu 虚拟机中。ESP32、底盘控制、轮式里程计、IMU、EKF 和 LD14 雷达驱动均继续运行在开发板上。

数据链路：

```text
开发板: /scan + /odom + TF  --->  虚拟机: /scan_slam + slam_toolbox  --->  /map + map->odom
```

## 启动

开发板的控制、定位和雷达节点启动后，在虚拟机终端执行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/桌面/ros2_ws/install/setup.bash
ros2 launch esp32_car_slam mapping.launch.py use_rviz:=true
```

通过 SSH 做无界面测试时使用 `use_rviz:=false`。

## 建图前检查

```bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_link laser_frame
```

正常启动后应出现 `/map`，并能查询 `map -> odom`。建图时缓慢行驶，优先沿闭合路线回到起点，以便回环优化。

LD14 每圈原始点数会在约 `390–393` 间变化。`scan_normalizer.py` 将 `/scan` 重采样为固定 360 点的 `/scan_slam`，避免 SLAM Toolbox 因每帧光束数量不同而拒绝或警告扫描；原始 `/scan` 保留不变，便于诊断。

## 保存地图

保存可继续建图的位姿图：

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$HOME/桌面/ros2_ws/maps/room'}"
```

如果系统未安装 `nav2_map_server`，使用工程自带工具保存 Nav2 兼容的栅格地图：

```bash
ros2 run esp32_car_slam save_map.py \
  --output "$HOME/桌面/ros2_ws/maps/room"
```

输出 `room.pgm` 和 `room.yaml`；位姿图输出 `room.posegraph` 和 `room.data`。

## 建图遥控

虚拟机不运行控制程序，只负责扫描预处理、Slam Toolbox 和 RViz 显示。
建图时请在开发板的交互式 SSH 终端运行：

```bash
source /opt/ros/rolling/setup.bash
source ~/control_node/install/setup.bash
ros2 run esp32_car_control mapping_teleop_node
```

这样 `/cmd_vel` 在开发板本地发布，不经过 VMware 控制链路。运行 Nav2 前应先退出
手动遥控，保证同一时间只有一个速度控制源。

## 加载地图并定位

完成建图后，不要同时运行 `mapping.launch.py`。在虚拟机终端启动静态地图和
AMCL：

```bash
source /opt/ros/jazzy/setup.bash
source ~/桌面/ros2_ws/install/setup.bash
ros2 launch esp32_car_slam localization.launch.py use_rviz:=true
```

默认加载 `~/桌面/ros2_ws/maps/aaaaccc.yaml`。如需加载其他地图：

```bash
ros2 launch esp32_car_slam localization.launch.py \
  map:=$HOME/桌面/ros2_ws/maps/其他地图.yaml use_rviz:=true
```

启动后，在 RViz 中选择顶部的 **2D Pose Estimate**，在地图上的小车实际位置
按住并拖动箭头，使箭头方向与车头一致。随后少量前后、左右移动或原地旋转，
观察紫色雷达点是否贴合墙体，以及绿色 AMCL 粒子是否收敛。

此入口只启动 `/scan_slam` 预处理、map server、AMCL 和 RViz，不会向
`/cmd_vel` 发布速度。AMCL 使用 `nav2_amcl::OmniMotionModel`，可正确处理
麦克纳姆底盘的横向里程计运动。

## Nav2 自动导航

正式导航前退出开发板上的 `mapping_teleop_node`，保证 `/cmd_vel` 没有其他
发布者。关闭单独的定位入口后，在虚拟机终端执行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/桌面/ros2_ws/install/setup.bash
ros2 launch esp32_car_slam nav2.launch.py start_rviz:=true
```

该入口一次启动静态地图、AMCL、Nav2 和专用 RViz。先使用 **2D Pose Estimate**
完成定位，确认雷达与地图重合后，点击 Navigation 2 面板中的 **Startup**；等待
Navigation 显示 `active` 后，再使用 **Nav2 Goal** 设置目标点。

当前实车安全参数：

- 车体 footprint：以地面上的 `base_footprint` 为中心的 `0.30 x 0.30 m` 正方形。
- footprint padding：`0.02 m`。
- 障碍膨胀半径：`0.30 m`。
- MPPI 运动模型：`Omni`，允许前后和横向运动。
- 最大 X/Y 速度：`0.20 m/s`；最大角速度：`0.50 rad/s`。
- 速度命令依次经过 velocity smoother 和 collision monitor 后才发布到
  `/cmd_vel`。

首次实车目标点测试只选择前方约 `0.30 m` 的空旷位置，并随时准备切断电机
电源。确认启动、跟踪、停车和取消目标均正常后，再逐步扩大距离。

### 低速死区与起点检查

ESP32 实车低于约 `0.05 m/s` 时不能可靠起步。Nav2 控制链中已加入
`cmd_vel_deadband_compensator.py`：它把明确非零但过小的平移向量等比例补偿到
可用速度，同时保持方向；小于 `0.005 m/s` 的平移和小于 `0.03 rad/s` 的角速度
视为 MPPI 噪声并置零。当前完整链路为：

```text
controller / behavior -> /cmd_vel_nav_raw -> deadband compensator
-> /cmd_vel_nav -> velocity smoother -> /cmd_vel_smoothed
-> collision monitor -> /cmd_vel
```

设置初始位姿后，如果车辆静止且 AMCL 尚未刷新，可执行：

```bash
ros2 service call /request_nomotion_update std_srvs/srv/Empty '{}'
```

### 终点朝向控制

正常路径跟踪继续使用全向 MPPI。控制器外层使用
`nav2_rotation_shim_controller::RotationShimController`，并启用
`rotate_to_goal_heading`：车辆进入 `0.10 m` 的 XY 目标容差后，由 Rotation
Shim 接管，停止 `vx/vy` 平移，只进行终点原地旋转，直到进入
`0.15 rad` 的 Yaw 容差。

起步旋转阈值设为接近 180 度，仅当新路径几乎完全指向车尾时才先旋转；普通
路径仍保留麦克纳姆底盘保持车头方向横移、斜移的能力。终点旋转速度限制为
`0.20 rad/s`，角加速度限制为 `0.80 rad/s²`，并启用 footprint 前向碰撞预测。

发送导航目标前不仅要观察目标方向的雷达距离，还要为 `0.30 x 0.30 m` 方形
footprint 留出完整扫掠空间。若机器人位于障碍膨胀区（全局代价接近 `99`），
NavFn 可能返回 `NO_VALID_PATH`；此时应先暂停 Nav2，并以低速人工移到开阔位置，
不能依赖恢复旋转脱困。
