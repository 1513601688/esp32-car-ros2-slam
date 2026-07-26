# 源码快照说明

本仓库整理于 2026-07-26，三个模块分别取自以下实际运行工程：

| 仓库目录 | 原始位置 | 内容 |
|---|---|---|
| `esp32_firmware/` | `C:\Users\clearlove\Desktop\ESP32_project\esp32_car` | ESP32-S3 固件、Web 页面和 Windows UI 烧录工具 |
| `board_control/` | `/home/luohjj/esp32_car_project` | 开发板 ROS 2 控制包与 LD14 驱动源码 |
| `vm_slam_nav2/` | `/home/luohjj/桌面/ros2_ws` | 虚拟机 SLAM/Nav2 包、启动文档和最新地图 |

## 未纳入版本库的内容

- ESP-IDF、colcon 的 `build/`、`install/`、`log/`。
- Python `__pycache__`、`.pyc` 和运行日志。
- ESP-IDF 下载生成的 `managed_components/`。
- 本地编辑器和临时远程编辑文件。
- ESP32 生成的 `sdkconfig`。
- 真实 Wi-Fi 名称和密码。

## 整理时的兼容性调整

- `sdkconfig.defaults` 中的 Wi-Fi 名称和密码改为空值，烧录时由 UI 注入。
- 虚拟机端默认地图由已经删除的旧地图改为当前保留的 `aaaaccc.yaml`。
- 虚拟机启动文档中的导航、定位和继续建图示例统一使用 `aaaaccc`。
- LD14 驱动原有 `LICENSE` 文件随源码保留。
