#pragma once

#include "esp_err.h"
#include "imu_sensor.h"

typedef struct {
    int32_t rpm[4];
    float   current[4];
    int32_t position[4];
    uint8_t speed_set;
    char    direction[16];
} motor_telemetry_t;

void web_server_log_init(void);
esp_err_t web_server_start(void);
void web_server_update_motor_data(const int32_t rpm[4], const float current[4], const int32_t pos[4]);

/**
 * 更新 IMU 传感器数据（在 telemetry 定时器中调用）
 */
void web_server_update_imu(const imu_data_t *imu);
