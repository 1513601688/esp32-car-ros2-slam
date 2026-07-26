/**
 * IMU Sensor — 维特智能 JY901 姿态传感器 (I2C)
 *
 * JY901 在 I2C 模式下同样输出 0x55 帧数据，每次可批量读取然后逐帧解析。
 * I2C 地址: 0x50
 *
 * 硬件连接:  JY901 SDA → ESP32 GPIO4
 *            JY901 SCL → ESP32 GPIO5
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>

// I2C 配置
#define IMU_I2C_PORT      I2C_NUM_0
#define IMU_I2C_SDA       4
#define IMU_I2C_SCL       5
#define IMU_I2C_FREQ      100000  // JY901 I2C 最大 100kHz
#define JY901_I2C_ADDR    0x50

// 数据单位
// ax/ay/az → m/s²
// gx/gy/gz → °/s
// pitch/roll/yaw → °
typedef struct {
    float ax, ay, az;       // 加速度 m/s²
    float gx, gy, gz;       // 角速度 °/s
    float pitch, roll;      // 姿态角 ° (Pitch/Roll)
    float yaw;              // 偏航角 ° (Yaw, 地磁融合)
    bool  valid;            // 数据是否有效
} imu_data_t;

/**
 * 初始化 I2C 总线和 JY901
 * @return ESP_OK 成功
 */
esp_err_t imu_sensor_init(void);

/**
 * 读取最新传感器数据 (非阻塞)
 * @param data 输出结构体
 * @return ESP_OK 成功
 */
esp_err_t imu_sensor_read(imu_data_t *data);
