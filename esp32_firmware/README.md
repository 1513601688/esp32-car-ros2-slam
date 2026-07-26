# ESP32 Car Multi-Module Repository

本仓库已按模块拆分，便于同时管理底盘下位机、上位机和后续扩展资源。

## 仓库结构

```text
esp32_car/
  modules/
    firmware/
      esp32_car/      # ESP32-S3 底盘固件工程（ESP-IDF）
    host/             # 上位机代码预留目录
    assets_3d/        # 3D 模型、结构件资源预留目录
  docs/               # 协议、排查、结构图等文档
  flash_gui.bat       # 图形化固件编译烧录入口
  README.md
```

## 当前模块说明

- `modules/firmware/esp32_car`
  当前可直接编译烧录的 ESP-IDF 项目，包含底盘控制、ROS 串口通信、IMU、前端页面和标定参数。

- `modules/host`
  预留给 ROS2 节点、串口上位机、调试工具或桌面端程序。

- `modules/assets_3d`
  预留给底盘结构模型、支架模型、外壳模型、STEP/STL/OBJ 等资源。

- `docs`
  集中存放项目说明、串口协议、排查记录和 RTOS 结构图。

## 固件编译与烧录

默认入口：

```bat
flash_gui.bat
```

该脚本会自动进入：

```text
modules/firmware/esp32_car
```

并在该目录下执行 `idf.py set-target esp32s3`、`idf.py build` 和烧录操作。

## 维护建议

- 下位机相关代码只放在 `modules/firmware/esp32_car`
- 上位机代码统一放在 `modules/host`
- 模型、装配件和可视化资源统一放在 `modules/assets_3d`
- 新增模块时，优先放到 `modules/` 下，避免再次把仓库根目录堆满
