/**
 * ESP32-S3 四轮小车 — 主入口
 */

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include <stdbool.h>

#include "wifi_manager.h"
#include "web_server.h"
#include "uart_motor.h"
#include "imu_sensor.h"
#include "ros_serial.h"
#include "car_config.h"

#define TAG "main"

#define STARTUP_REBOOT_DELAY_MS     3000
#define MOTOR_POLL_START_DELAY_MS   2000
/*
 * Four motors are queried in turn. With a 10 ms inter-query interval and the
 * short UART reply time, one complete encoder snapshot is produced at about
 * 20 Hz for wheel odometry fusion.
 */
#define MOTOR_POLL_INTERVAL_MS      10
#define IMU_POLL_START_DELAY_MS     200
#define IMU_POLL_INTERVAL_MS        20
#define MOTOR_POLL_TASK_STACK       4096
#define IMU_POLL_TASK_STACK         3072
#define MOTOR_POLL_TASK_PRIORITY    2
#define IMU_POLL_TASK_PRIORITY      1
#define MOTOR_POLL_TASK_CORE        1
#define MOTOR_POLL_LOG_INTERVAL_MS  1000

static void reboot_after_error(const char *reason)
{
    ESP_LOGE(TAG, "%s, reboot", reason);
    vTaskDelay(pdMS_TO_TICKS(STARTUP_REBOOT_DELAY_MS));
    esp_restart();
}

static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

static void motor_poll_task(void *arg)
{
    motor_status_t st;
    int32_t rpm[CAR_MOTOR_COUNT] = {0};
    float cur[CAR_MOTOR_COUNT] = {0};
    int32_t pos[CAR_MOTOR_COUNT] = {0};
    uint32_t ok_count[CAR_MOTOR_COUNT] = {0};
    uint32_t fail_count[CAR_MOTOR_COUNT] = {0};
    TickType_t next_log_tick = 0;
    uint8_t id = 1;
    bool cycle_ok = true;

    vTaskDelay(pdMS_TO_TICKS(MOTOR_POLL_START_DELAY_MS));

    while (1) {
        uart_motor_watchdog();

        if (uart_motor_query(id, &st) == ESP_OK) {
            rpm[id - 1] = st.rpm;
            pos[id - 1] = st.position;
            ok_count[id - 1]++;
        } else {
            fail_count[id - 1]++;
            cycle_ok = false;
        }
        // 立即推送，不等 4 个全部读完（降低 UI 延迟）
        if (id == CAR_MOTOR_COUNT && cycle_ok) {
            web_server_update_motor_data(rpm, cur, pos);
        }

        TickType_t now = xTaskGetTickCount();
        if (now >= next_log_tick) {
            next_log_tick = now + pdMS_TO_TICKS(MOTOR_POLL_LOG_INTERVAL_MS);
            ESP_LOGI(TAG,
                     "ENC pos=[%ld,%ld,%ld,%ld] ok=[%lu,%lu,%lu,%lu] fail=[%lu,%lu,%lu,%lu]",
                     (long)pos[0], (long)pos[1], (long)pos[2], (long)pos[3],
                     (unsigned long)ok_count[0], (unsigned long)ok_count[1],
                     (unsigned long)ok_count[2], (unsigned long)ok_count[3],
                     (unsigned long)fail_count[0], (unsigned long)fail_count[1],
                     (unsigned long)fail_count[2], (unsigned long)fail_count[3]);
        }

        if (id == CAR_MOTOR_COUNT) {
            cycle_ok = true;
        }
        id = (id % CAR_MOTOR_COUNT) + 1;
        vTaskDelay(pdMS_TO_TICKS(MOTOR_POLL_INTERVAL_MS));
    }
}

static void imu_poll_task(void *arg)
{
    imu_data_t data;
    vTaskDelay(pdMS_TO_TICKS(IMU_POLL_START_DELAY_MS));

    while (1) {
        if (imu_sensor_read(&data) == ESP_OK) {
            web_server_update_imu(&data);
        }
        vTaskDelay(pdMS_TO_TICKS(IMU_POLL_INTERVAL_MS));  // 50Hz 轮询
    }
}

void app_main(void)
{
    web_server_log_init();

    ESP_LOGI(TAG, "ESP32-S3 CAR v1.9");

    // NVS
    esp_err_t ret = init_nvs();
    if (ret != ESP_OK) reboot_after_error("NVS init failed");

    // WiFi
    ret = wifi_manager_init();
    if (ret != ESP_OK) reboot_after_error("WiFi failed");

    // UART
    ret = uart_motor_init(CAR_MOTOR_UART_TX_PIN, CAR_MOTOR_UART_RX_PIN, CAR_MOTOR_UART_BAUDRATE);
    if (ret != ESP_OK) reboot_after_error("Motor UART init failed");

    // IMU (JY901, I2C SDA=IO4, SCL=IO5)
    ret = imu_sensor_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "IMU init failed: %s", esp_err_to_name(ret));
    }

    // ROS 串口 (TX=IO6, RX=IO7)
    ret = ros_serial_init();
    if (ret != ESP_OK) reboot_after_error("ROS serial init failed");

    // HTTP + WebSocket
    ret = web_server_start();
    if (ret != ESP_OK) reboot_after_error("Web server init failed");

    // 电机转速轮询 (Core 1)
    BaseType_t task_ok = xTaskCreatePinnedToCore(
        motor_poll_task, "poll", MOTOR_POLL_TASK_STACK, NULL,
        MOTOR_POLL_TASK_PRIORITY, NULL, MOTOR_POLL_TASK_CORE);
    if (task_ok != pdPASS) reboot_after_error("Motor poll task create failed");

    // IMU 轮询
    task_ok = xTaskCreate(imu_poll_task, "imu_poll", IMU_POLL_TASK_STACK, NULL, IMU_POLL_TASK_PRIORITY, NULL);
    if (task_ok != pdPASS) reboot_after_error("IMU poll task create failed");

    ESP_LOGI(TAG, "Ready: http://esp32car.local");
}
