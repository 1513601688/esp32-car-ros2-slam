# ESP32 小车开发板工程

开发板端与当前小车相关的代码统一存放在：

```text
/home/luohjj/esp32_car_project
```

目录结构：

```text
esp32_car_project/
├── README.md
├── control_node/       # 底盘串口、传感器发布、EKF、TF、雷达启动和建图遥控
└── ldlidar_ros2_ws/    # LD14 二维激光雷达驱动
```

`/home/luohjj/ros2_ws` 中的 `my_pkg` 与本项目无关。

## 构建

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

## 启动底盘、里程计、IMU 和 EKF

```bash
source /opt/ros/rolling/setup.bash
source ~/esp32_car_project/ldlidar_ros2_ws/install/setup.bash
source ~/esp32_car_project/control_node/install/setup.bash
ros2 launch esp32_car_control localization.launch.py
```

## 启动 LD14 雷达

```bash
source /opt/ros/rolling/setup.bash
source ~/esp32_car_project/ldlidar_ros2_ws/install/setup.bash
source ~/esp32_car_project/control_node/install/setup.bash
ros2 launch esp32_car_control lidar.launch.py
```

## 启动开发板本地建图遥控

```bash
source /opt/ros/rolling/setup.bash
source ~/esp32_car_project/control_node/install/setup.bash
ros2 run esp32_car_control mapping_teleop_node
```

完整项目进度、标定结果、话题、TF 和历史修改记录：

```text
/home/luohjj/esp32_car_project/control_node/PROJECT_PROGRESS.md
```
