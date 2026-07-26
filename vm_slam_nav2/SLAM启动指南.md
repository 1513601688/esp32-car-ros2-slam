# 虚拟机端 SLAM 快速启动指南

更新时间：2026-07-26

本文档只记录 VMware Ubuntu 虚拟机中的操作。

虚拟机 ROS 2 工作区：

```text
/home/luohjj/桌面/ros2_ws
```

工程源码：

```text
/home/luohjj/桌面/ros2_ws/src/esp32_car_slam
```

地图目录：

```text
/home/luohjj/桌面/ros2_ws/maps
```

## 一、加载虚拟机 ROS 2 环境

每打开一个新的虚拟机终端，都先执行：

```bash
cd ~/桌面/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

如果修改过工程代码，重新构建：

```bash
cd ~/桌面/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select esp32_car_slam --symlink-install
source install/setup.bash
```

## 二、建图前检查输入数据

开始 SLAM 前，虚拟机应能接收到：

```text
/odom
/scan
odom -> base_footprint -> base_link -> laser_frame
```

检查命令：

```bash
source /opt/ros/jazzy/setup.bash
source ~/桌面/ros2_ws/install/setup.bash

ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_link laser_frame
```

也可以检查频率：

```bash
ros2 topic hz /odom
ros2 topic hz /scan
```

数据和 TF 均正常后再启动 SLAM。

## 三、启动新地图建图

在虚拟机终端执行：

```bash
cd ~/桌面/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch esp32_car_slam mapping.launch.py use_rviz:=true
```

正常情况下，RViz 应显示：

- `/map`
- `/scan_slam`
- `map -> odom -> base_footprint -> base_link -> laser_frame`
- 地图分辨率 `0.025 m/cell`

新建图阶段不需要使用 `2D Pose Estimate`，也不需要点击 Navigation 2 面板的
`Startup`。

虚拟机只负责扫描归一化、Slam Toolbox 和 RViz，不运行车辆控制程序。

## 四、建图时观察重点

1. SLAM 启动后先保持车辆静止几秒。
2. 观察实时雷达点是否贴合刚生成的墙线。
3. 经过已经扫描过的位置时，观察新旧墙线是否重合。
4. 出现回环后，地图可能整体进行一次小幅优化，这是正常现象。
5. 如果墙体持续出现明显双层、重影或整体旋转，应停止移动并排查数据。
6. 不要同时启动 `mapping.launch.py`、`localization.launch.py` 和
   `nav2.launch.py`，否则可能有多个节点同时发布 `map -> odom`。

## 五、保存地图

保存前先让车辆停止，等待约 3 秒，但不要关闭正在运行的
`mapping.launch.py`。

在虚拟机另一个终端执行：

```bash
cd ~/桌面/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

MAP_NAME=room_$(date +%Y%m%d_%H%M)
echo "本次地图名称：${MAP_NAME}"
```

保存可继续建图的位姿图：

```bash
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$HOME/桌面/ros2_ws/maps/${MAP_NAME}'}"
```

保存 Nav2 使用的栅格地图：

```bash
ros2 run esp32_car_slam save_map.py \
  --output "$HOME/桌面/ros2_ws/maps/${MAP_NAME}"
```

检查文件：

```bash
ls -lh "$HOME/桌面/ros2_ws/maps/${MAP_NAME}".*
```

正常应生成：

```text
地图名.pgm
地图名.yaml
地图名.posegraph
地图名.data
```

用途：

- `.pgm + .yaml`：Nav2 静态地图。
- `.posegraph + .data`：以后继续建图。

保存成功后，在运行 SLAM 的虚拟机终端按 `Ctrl-C` 退出。

## 六、使用保存的地图启动 Nav2

确认 `mapping.launch.py` 已退出，然后执行：

```bash
cd ~/桌面/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch esp32_car_slam nav2.launch.py \
  map:=$HOME/桌面/ros2_ws/maps/地图名.yaml \
  start_rviz:=true
```

例如加载目前的地图：

```bash
ros2 launch esp32_car_slam nav2.launch.py \
  map:=$HOME/桌面/ros2_ws/maps/aaaaccc.yaml \
  start_rviz:=true
```

RViz 中依次操作：

1. 使用 `2D Pose Estimate` 设置车辆实际位置和车头方向。
2. 确认雷达点与静态地图基本重合。
3. 点击 Navigation 2 面板的 `Startup`。
4. 等待 `Localization: active` 和 `Navigation: active`。
5. 使用 `Nav2 Goal` 设置目标位置和最终朝向。

当前 Nav2 已启用终点朝向控制：进入目标位置容差后停止平移，再原地旋转到目标
箭头方向。

## 七、只启动静态地图定位

如果只想检查地图和 AMCL，不启动 Nav2 控制：

```bash
cd ~/桌面/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch esp32_car_slam localization.launch.py \
  map:=$HOME/桌面/ros2_ws/maps/aaaaccc.yaml \
  use_rviz:=true
```

设置 `2D Pose Estimate` 后，如果车辆静止且 AMCL 没有立即刷新，可以执行：

```bash
ros2 service call /request_nomotion_update std_srvs/srv/Empty '{}'
```

## 八、继续扩建旧地图

继续建图必须加载旧地图的 `.posegraph/.data`，不能只加载 `.pgm/.yaml`。

先启动普通 SLAM：

```bash
cd ~/桌面/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch esp32_car_slam mapping.launch.py use_rviz:=true
```

车辆位于原建图起点且朝向一致时，在另一个虚拟机终端、动车前执行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/桌面/ros2_ws/install/setup.bash

ros2 service call /slam_toolbox/deserialize_map \
  slam_toolbox/srv/DeserializePoseGraph \
  "{filename: '/home/luohjj/桌面/ros2_ws/maps/aaaaccc', match_type: 1, initial_pose: {x: 0.0, y: 0.0, theta: 0.0}}"
```

等待旧地图完整出现，确认雷达点与旧地图重合后再开始扩建。完成后使用第五节的
命令另存为新名称，不要覆盖原始地图。

如果车辆不在原建图起点，需要提供车辆在旧地图中的 `x/y/theta`，并使用
`match_type: 2`。

## 九、虚拟机端常见问题

### RViz 只有 TF，没有地图

检查相关话题：

```bash
ros2 topic info /map
ros2 topic info /scan_slam
ros2 topic info /odom
```

建图时 `/map` 应由 Slam Toolbox 发布；导航时 `/map` 应由 map server 发布。

### 雷达点与地图不重合

- 建图阶段：检查 `/odom`、`/scan` 和 TF 是否连续。
- 定位或导航阶段：重新使用 `2D Pose Estimate`。
- 人工改变车辆位置后必须重新设置初始位姿。

### Navigation 显示 inactive

完成 `2D Pose Estimate` 并确认雷达与地图重合后，点击 Navigation 2 面板的
`Startup`。

### 检查速度发布者

```bash
ros2 topic info /cmd_vel
```

没有导航目标时不应持续出现异常速度命令。

### 查看当前 ROS 节点

```bash
ros2 node list
```

### 查看最新日志

```bash
ls -lt ~/.ros/log | head
```
