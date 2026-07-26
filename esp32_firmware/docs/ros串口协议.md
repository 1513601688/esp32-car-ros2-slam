# ESP32-Car ROS 串口通信协议

## 硬件连接

| ESP32-S3 | 方向 | ROS 上位机 |
|---|---|---|
| IO6 (TX) | → | RX |
| IO7 (RX) | ← | TX |
| GND | — | GND |

- 波特率：115200，8N1
- 多字节数值：小端序
- 坐标约定：x 向前、y 向左、z 向上、绕 z 逆时针为正

## RX：ROS cmd_vel → ESP32

控制命令使用 14 字节二进制帧：

```text
AA [vx float32 LE] [vy float32 LE] [wz float32 LE] 55
```

| 字节 | 内容 | 单位 |
|---|---|---|
| 0 | `0xAA` 帧头 | - |
| 1–4 | `vx` 前进速度 | m/s |
| 5–8 | `vy` 左移速度 | m/s |
| 9–12 | `wz` 逆时针角速度 | rad/s |
| 13 | `0x55` 帧尾 | - |

## TX：ESP32 → ROS

传感器回传采用以换行符结尾的 ASCII 文本帧。轮式里程计和 IMU 是两路独立测量，ROS 2 上位机应分别发布 `/wheel/odom` 和 `/imu/data`，再交给 `robot_localization` 融合。

### 轮式里程计

```text
WHEEL,<stamp_ms>,<vx>,<vy>,<wz>\n
```

| 字段 | 含义 | 单位 |
|---|---|---|
| `stamp_ms` | ESP32 启动后的单调采样时间 | ms |
| `vx` | 编码器解算并校准后的前进速度 | m/s |
| `vy` | 编码器解算并校准后的左移速度 | m/s |
| `wz` | 编码器解算的逆时针角速度 | rad/s |

目标发送频率约 20 Hz，与当前一轮四电机编码器采样周期一致。实际频率受电机串口应答时间影响，烧录后应在 ROS 2 端通过 `ros2 topic hz /wheel/odom` 复核。

示例：

```text
WHEEL,125430,0.215,-0.008,0.0320
```

### IMU

```text
IMU,<stamp_ms>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<roll>,<pitch>,<yaw>\n
```

| 字段 | 含义 | 单位 |
|---|---|---|
| `stamp_ms` | ESP32 启动后的单调采样时间 | ms |
| `ax/ay/az` | JY901 三轴加速度 | m/s² |
| `gx/gy/gz` | JY901 三轴角速度 | rad/s |
| `roll/pitch/yaw` | JY901 姿态角 | rad |

发送频率约 50 Hz。IMU 字段保持传感器坐标系；ROS 端应使用 `imu_link` 作为 `frame_id`，并通过 URDF 或静态 TF 描述 `base_link → imu_link` 的实际安装关系。

示例：

```text
IMU,125440,0.0200,-0.0100,9.8000,0.00100,-0.00200,0.03500,0.00175,-0.00349,0.26704
```

## 时间戳处理

`stamp_ms` 是 ESP32 启动后的单调时间，不是 ROS 系统时间。串口桥接节点可以：

1. 启动时记录 ROS 时间与首个 ESP32 时间戳的偏移；
2. 用该偏移把后续 `stamp_ms` 转换为 ROS 时间；
3. 检测 ESP32 重启或 32 位毫秒计数回绕并重新建立偏移。

初期也可以直接用串口帧接收时的 ROS 时间作为 `header.stamp`。

## Python 解析示例

```python
line = ser.readline().decode("ascii", errors="ignore").strip()
fields = line.split(",")

if fields[0] == "WHEEL" and len(fields) == 5:
    stamp_ms = int(fields[1])
    vx, vy, wz = map(float, fields[2:5])

elif fields[0] == "IMU" and len(fields) == 11:
    stamp_ms = int(fields[1])
    ax, ay, az, gx, gy, gz, roll, pitch, yaw = map(float, fields[2:11])
```

## Python 发送 cmd_vel 示例

```python
import struct

HDR, TAIL = 0xAA, 0x55

def send_cmd_vel(ser, vx, vy, wz):
    ser.write(struct.pack("<BfffB", HDR, vx, vy, wz, TAIL))
```

## 融合注意事项

- `/wheel/odom` 只融合编码器产生的 `vx`、`vy`、`wz`，不要再填入 IMU yaw。
- `/imu/data` 的角速度和姿态已经转换为 ROS 所需的弧度单位。
- 使用 EKF 发布 `odom → base_link` 时，串口桥接节点不要重复发布该 TF。
- 在融合 IMU 姿态前，必须验证传感器轴向、正负号和 ENU 约定；如果不确定，可先只融合 `gz`。
