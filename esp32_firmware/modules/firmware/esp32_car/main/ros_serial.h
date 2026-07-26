/**
 * ROS Serial.
 *
 * RX cmd_vel: 0xAA + float32(vx, vy, wz) little-endian + 0x55.
 * TX wheel:   "WHEEL,<stamp_ms>,<vx>,<vy>,<wz>\n".
 * TX IMU:     "IMU,<stamp_ms>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<roll>,<pitch>,<yaw>\n".
 *
 * UART2: IO6=TX, IO7=RX, 115200bps.
 */

#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "imu_sensor.h"

#define ROS_MAX_LINEAR_MPS   1.5f
#define ROS_MAX_ANGULAR_RPS  5.0f

typedef struct {
    uint32_t stamp_ms;
    float vx, vy, wz;
    bool  valid;
} ros_wheel_odom_t;

typedef struct {
    uint32_t stamp_ms;
    float ax, ay, az;
    float gx, gy, gz;
    float roll, pitch, yaw;
    bool  valid;
} ros_imu_t;

esp_err_t ros_serial_init(void);
void ros_serial_update_wheel(float vx, float vy, float wz);
void ros_serial_update_imu(const imu_data_t *imu);
