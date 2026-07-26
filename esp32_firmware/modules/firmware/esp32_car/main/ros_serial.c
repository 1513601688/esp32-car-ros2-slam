/**
 * ROS Serial — cmd_vel 接收 & 里程计回传 (UART2, TX=IO6 RX=IO7, 115200bps)
 */

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "car_config.h"
#include "ros_serial.h"
#include "uart_motor.h"

#define TAG "ros"

#define UART_PORT   UART_NUM_2
#define UART_RX     7       // 上位机 TX → ESP32 IO7
#define UART_TX     6       // ESP32 IO6 → 上位机 RX
#define UART_BAUD   115200
#define FRAME_SIZE  14
#define ROS_TX_LINE_MAX_LEN 192
#define ROS_TX_PERIOD_MS 20
#define DEG_TO_RAD       0.01745329251994329577f
#define HDR         0xAA
#define TAIL        0x55

static ros_wheel_odom_t s_wheel_odom;
static ros_imu_t s_ros_imu;
static portMUX_TYPE s_sensor_mux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t sample_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

void ros_serial_update_wheel(float vx, float vy, float wz)
{
    portENTER_CRITICAL(&s_sensor_mux);
    s_wheel_odom.stamp_ms = sample_time_ms();
    s_wheel_odom.vx = vx;
    s_wheel_odom.vy = vy;
    s_wheel_odom.wz = wz;
    s_wheel_odom.valid = true;
    portEXIT_CRITICAL(&s_sensor_mux);
}

void ros_serial_update_imu(const imu_data_t *imu)
{
    if (!imu || !imu->valid) return;

    portENTER_CRITICAL(&s_sensor_mux);
    s_ros_imu.stamp_ms = sample_time_ms();
    s_ros_imu.ax = imu->ax;
    s_ros_imu.ay = imu->ay;
    s_ros_imu.az = imu->az;
    s_ros_imu.gx = imu->gx * DEG_TO_RAD;
    s_ros_imu.gy = imu->gy * DEG_TO_RAD;
    s_ros_imu.gz = imu->gz * DEG_TO_RAD;
    s_ros_imu.roll = imu->roll * DEG_TO_RAD;
    s_ros_imu.pitch = imu->pitch * DEG_TO_RAD;
    s_ros_imu.yaw = imu->yaw * DEG_TO_RAD;
    s_ros_imu.valid = true;
    portEXIT_CRITICAL(&s_sensor_mux);
}

// ── cmd_vel → 电机 ──
static float clamp_unit(float value)
{
    if (value > 1.0f) return 1.0f;
    if (value < -1.0f) return -1.0f;
    return value;
}

static void handle_cmd_vel(const uint8_t *data)
{
    float vx, vy, wz;
    memcpy(&vx, data,     4);
    memcpy(&vy, data + 4, 4);
    memcpy(&wz, data + 8, 4);

    const float calibrated_vx = vx * CAR_CMD_SCALE_VX;
    const float calibrated_vy = vy * CAR_CMD_SCALE_VY;
    const float calibrated_wz = wz * CAR_CMD_SCALE_WZ;

    float joy_x = clamp_unit(-calibrated_vy / ROS_MAX_LINEAR_MPS);
    float joy_y = clamp_unit( calibrated_vx / ROS_MAX_LINEAR_MPS);
    float rot   = clamp_unit( calibrated_wz / ROS_MAX_ANGULAR_RPS);

    if (fabsf(vx) < 0.001f && fabsf(vy) < 0.001f && fabsf(wz) < 0.001f) {
        // 全零 → 停车
        uart_motor_stop();
    } else {
        uart_motor_clear_stop_override();
        uart_motor_joystick(joy_x, joy_y, rot);
    }
}

// ── RX 任务 ──
static void ros_rx_task(void *arg)
{
    uint8_t buf[FRAME_SIZE];
    int pos = 0;
    while (1) {
        uint8_t b;
        if (uart_read_bytes(UART_PORT, &b, 1, pdMS_TO_TICKS(50)) != 1)
            continue;
        if (pos == 0) {
            if (b == HDR) buf[pos++] = b;
        } else {
            buf[pos++] = b;
            if (pos == FRAME_SIZE) {
                pos = 0;
                if (buf[0] == HDR && buf[FRAME_SIZE - 1] == TAIL)
                    handle_cmd_vel(buf + 1);
            }
        }
    }
}

// ── 传感器 TX 任务：IMU 约 50Hz，WHEEL 目标约 20Hz ──
static bool write_sensor_line(const char *line, int len)
{
    if (len <= 0 || len >= ROS_TX_LINE_MAX_LEN) {
        ESP_LOGW(TAG, "Sensor line format failed len=%d", len);
        return false;
    }

    int written = uart_write_bytes(UART_PORT, line, len);
    if (written != len) {
        ESP_LOGW(TAG, "Sensor line short write: %d/%d", written, len);
        return false;
    }
    return true;
}

static void ros_tx_task(void *arg)
{
    uint32_t imu_tx_count = 0;
    uint32_t wheel_tx_count = 0;
    uint32_t last_imu_stamp = UINT32_MAX;
    uint32_t last_wheel_stamp = UINT32_MAX;
    char line[ROS_TX_LINE_MAX_LEN];

    vTaskDelay(pdMS_TO_TICKS(2000));
    while (1) {
        ros_wheel_odom_t wheel;
        ros_imu_t imu;
        portENTER_CRITICAL(&s_sensor_mux);
        wheel = s_wheel_odom;
        imu = s_ros_imu;
        portEXIT_CRITICAL(&s_sensor_mux);

        if (imu.valid && imu.stamp_ms != last_imu_stamp) {
            int len = snprintf(line, sizeof(line),
                               "IMU,%lu,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                               (unsigned long)imu.stamp_ms,
                               (double)imu.ax, (double)imu.ay, (double)imu.az,
                               (double)imu.gx, (double)imu.gy, (double)imu.gz,
                               (double)imu.roll, (double)imu.pitch, (double)imu.yaw);
            if (write_sensor_line(line, len)) {
                last_imu_stamp = imu.stamp_ms;
                if ((++imu_tx_count % 50) == 0) {
                    ESP_LOGI(TAG, "TX IMU stamp=%lu gz=%.4f yaw=%.3f",
                             (unsigned long)imu.stamp_ms, (double)imu.gz, (double)imu.yaw);
                }
            }
        }

        if (wheel.valid && wheel.stamp_ms != last_wheel_stamp) {
            int len = snprintf(line, sizeof(line), "WHEEL,%lu,%.3f,%.3f,%.4f\n",
                               (unsigned long)wheel.stamp_ms,
                               (double)wheel.vx, (double)wheel.vy, (double)wheel.wz);
            if (write_sensor_line(line, len)) {
                last_wheel_stamp = wheel.stamp_ms;
                /* At the target 20 Hz, log once per second. */
                if ((++wheel_tx_count % 20) == 0) {
                    ESP_LOGI(TAG, "TX WHEEL stamp=%lu vx=%.3f vy=%.3f wz=%.4f",
                             (unsigned long)wheel.stamp_ms,
                             (double)wheel.vx, (double)wheel.vy, (double)wheel.wz);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(ROS_TX_PERIOD_MS));
    }
}

esp_err_t ros_serial_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX, UART_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 256, 0, 0, NULL, 0));

    BaseType_t ok = xTaskCreate(ros_rx_task, "ros_rx", 4096, NULL, 3, NULL);
    if (ok != pdPASS) return ESP_ERR_NO_MEM;

    ok = xTaskCreate(ros_tx_task, "ros_tx", 3072, NULL, 3, NULL);
    if (ok != pdPASS) return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "ROS serial ready RX=IO%d TX=IO%d %dbps",
             UART_RX, UART_TX, UART_BAUD);
    return ESP_OK;
}
