/**
 * UART 电机通信 — 速度模式协议
 */

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "uart_motor.h"
#include "car_config.h"

#define TAG "motor"

#define MOTOR_FL 0x01
#define MOTOR_FR 0x02
#define MOTOR_RL 0x03
#define MOTOR_RR 0x04

#define MOTOR_UART_PORT         UART_NUM_1
#define MOTOR_UART_RX_BUF_SIZE  1024
#define MOTOR_UART_TX_BUF_SIZE  1024
#define MOTOR_CMD_LEN           8
#define MOTOR_QUERY_LEN         3
#define MOTOR_QUERY_REPLY_LEN   8
#define MOTOR_QUERY_BUF_LEN     16
#define MOTOR_QUERY_TIMEOUT_MS  30
#define MOTOR_QUERY_RETRY_MS    5
#define MOTOR_QUERY_EXTRA_MS    20
#define MOTOR_BUS_GUARD_US      500
#define MOTOR_WATCHDOG_MS       500
#define MOTOR_DEFAULT_ACC       10
#define MOTOR_DEFAULT_MAX_RPM   500
#define MOTOR_MIN_MAX_RPM       10
#define MOTOR_MAX_MAX_RPM       3000

static SemaphoreHandle_t s_uart_mutex;
static uint8_t s_acc = MOTOR_DEFAULT_ACC;
static uint16_t s_max_rpm = MOTOR_DEFAULT_MAX_RPM;
static volatile bool s_stop_override = false;
static bool s_motion_active = true;
static const int8_t s_dir_calib[CAR_MOTOR_COUNT] = {1, 1, -1, -1};
static const uint8_t s_sync_trigger[] = {0x00, 0xFF, 0x66, 0x6B};

// 看门狗: 500ms 无指令自动停车
static volatile TickType_t s_last_cmd_tick = 0;

// 软件归零
static int32_t s_pulse_baseline[CAR_MOTOR_COUNT] = {0};
static bool    s_baseline_set[CAR_MOTOR_COUNT]   = {false, false, false, false};

static inline uint8_t apply_calib(uint8_t addr, uint8_t d)
{
    if (s_dir_calib[addr - 1] < 0) d ^= 0x01;
    return d;
}

static void build_speed_cmd(uint8_t *buf, uint8_t addr, uint8_t dir,
                            uint16_t rpm, uint8_t acc, uint8_t sync)
{
    buf[0] = addr;
    buf[1] = 0xF6;
    buf[2] = dir;
    buf[3] = (uint8_t)((rpm >> 8) & 0xFF);
    buf[4] = (uint8_t)(rpm & 0xFF);
    buf[5] = acc;
    buf[6] = sync;
    buf[7] = 0x6B;
}

static void uart_send(const uint8_t *data, size_t len)
{
    xSemaphoreTake(s_uart_mutex, portMAX_DELAY);
    uart_write_bytes(MOTOR_UART_PORT, data, len);
    ets_delay_us(MOTOR_BUS_GUARD_US);
    xSemaphoreGive(s_uart_mutex);
}

esp_err_t uart_motor_init(uint8_t tx_pin, uint8_t rx_pin, uint32_t baudrate)
{
    s_uart_mutex = xSemaphoreCreateMutex();
    if (!s_uart_mutex) {
        ESP_LOGE(TAG, "Failed to create UART mutex");
        return ESP_ERR_NO_MEM;
    }

    uart_config_t uart_cfg = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .parity = UART_PARITY_DISABLE,
        .rx_flow_ctrl_thresh = 100,
        .source_clk = UART_SCLK_DEFAULT,
        .stop_bits = UART_STOP_BITS_1,
    };
    esp_err_t ret = uart_param_config(MOTOR_UART_PORT, &uart_cfg);
    if (ret != ESP_OK) return ret;

    ret = uart_set_pin(MOTOR_UART_PORT, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) return ret;

    ret = uart_driver_install(MOTOR_UART_PORT, MOTOR_UART_RX_BUF_SIZE, MOTOR_UART_TX_BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "UART ready: TX=%u RX=%u %lu bps",
             tx_pin, rx_pin, (unsigned long)baudrate);
    return ESP_OK;
}

void uart_motor_set_acc(uint8_t acc)     { s_acc = acc; }
void uart_motor_set_max_rpm(uint16_t rpm) { s_max_rpm = (rpm < MOTOR_MIN_MAX_RPM) ? MOTOR_MIN_MAX_RPM : (rpm > MOTOR_MAX_MAX_RPM) ? MOTOR_MAX_MAX_RPM : rpm; }

esp_err_t uart_motor_stop(void)
{
    s_stop_override = true;
    s_last_cmd_tick = 0;
    if (!s_motion_active) {
        return ESP_OK;
    }

    uint8_t cmd[MOTOR_CMD_LEN];
    for (uint8_t addr = 1; addr <= CAR_MOTOR_COUNT; addr++) {
        uint8_t d = apply_calib(addr, 0x01);
        build_speed_cmd(cmd, addr, d, 0, s_acc, 0x01);
        uart_send(cmd, sizeof(cmd));
    }
    uart_send(s_sync_trigger, sizeof(s_sync_trigger));
    s_motion_active = false;
    ESP_LOGI(TAG, "STOP");
    return ESP_OK;
}

void uart_motor_clear_stop_override(void) { s_stop_override = false; }

esp_err_t uart_motor_joystick(float Vx, float Vy, float rot)
{
    s_last_cmd_tick = xTaskGetTickCount();
    if (s_stop_override) return ESP_OK;

    if (sqrtf(Vx * Vx + Vy * Vy) < CAR_CMD_LINEAR_DEADZONE) {
        Vx = 0.0f;
        Vy = 0.0f;
    }
    if (fabsf(rot) < CAR_CMD_ROT_DEADZONE) {
        rot = 0.0f;
    }
    if (Vx == 0.0f && Vy == 0.0f && rot == 0.0f) {
        return uart_motor_stop();
    }

    float w[CAR_MOTOR_COUNT] = {
        Vy - Vx + rot,
        -(Vy + Vx) + rot,
        Vx - Vy + rot,
        Vy + Vx + rot
    };

    float max_abs = 0.0f;
    for (int i = 0; i < CAR_MOTOR_COUNT; i++) { float a = fabsf(w[i]); if (a > max_abs) max_abs = a; }
    if (max_abs > 1.0f) {
        for (int i = 0; i < CAR_MOTOR_COUNT; i++) {
            w[i] /= max_abs;
        }
        max_abs = 1.0f;
    }

    const float min_active_ratio = fminf(1.0f, (float)CAR_MOTOR_MIN_EFFECTIVE_RPM / (float)s_max_rpm);
    if (max_abs > 0.0f && max_abs < min_active_ratio) {
        const float boost = min_active_ratio / max_abs;
        for (int i = 0; i < CAR_MOTOR_COUNT; i++) {
            w[i] *= boost;
        }
    }

    uint8_t cmd[MOTOR_CMD_LEN];
    uint8_t addrs[CAR_MOTOR_COUNT] = {MOTOR_FL, MOTOR_FR, MOTOR_RL, MOTOR_RR};
    for (int i = 0; i < CAR_MOTOR_COUNT; i++) {
        if (s_stop_override) return ESP_OK;
        uint8_t d = (w[i] >= 0) ? 0x01 : 0x00;
        d = apply_calib(addrs[i], d);
        uint16_t rpm = (uint16_t)(fabsf(w[i]) * s_max_rpm);
        build_speed_cmd(cmd, addrs[i], d, rpm, s_acc, 0x01);
        uart_send(cmd, sizeof(cmd));
    }
    if (s_stop_override) return ESP_OK;
    uart_send(s_sync_trigger, sizeof(s_sync_trigger));
    s_motion_active = true;
    return ESP_OK;
}

void uart_motor_zero_all(void)
{
    for (int i = 0; i < CAR_MOTOR_COUNT; i++) s_baseline_set[i] = false;
}

// ── 看门狗 ──
void uart_motor_watchdog(void)
{
    if (s_last_cmd_tick == 0 || s_stop_override) return;
    if ((xTaskGetTickCount() - s_last_cmd_tick) > pdMS_TO_TICKS(MOTOR_WATCHDOG_MS)) {
        ESP_LOGW(TAG, "Watchdog: auto-stop");
        uart_motor_stop();
    }
}

// ── 独立看门狗任务 (简单可靠) ──
// ── 查询编码器 ──
esp_err_t uart_motor_query(uint8_t motor_id, motor_status_t *status)
{
    if (motor_id < 1 || motor_id > CAR_MOTOR_COUNT || !status) return ESP_ERR_INVALID_ARG;

    uint8_t query[MOTOR_QUERY_LEN] = {motor_id, 0x32, 0x6B};
    uint8_t reply[MOTOR_QUERY_BUF_LEN] = {0};

    xSemaphoreTake(s_uart_mutex, portMAX_DELAY);
    uart_flush_input(MOTOR_UART_PORT);
    uart_write_bytes(MOTOR_UART_PORT, query, sizeof(query));

    /*
     * A valid 0x32 reply is exactly 8 bytes. Requesting the whole 16-byte
     * scratch buffer makes uart_read_bytes wait for bytes that never arrive,
     * unnecessarily adding the full timeout to every motor query.
     */
    int total = uart_read_bytes(MOTOR_UART_PORT, reply, MOTOR_QUERY_REPLY_LEN,
                                pdMS_TO_TICKS(MOTOR_QUERY_TIMEOUT_MS));
    if (total < 0) total = 0;
    if (total < MOTOR_QUERY_REPLY_LEN) {
        TickType_t start = xTaskGetTickCount();
        while (total < MOTOR_QUERY_REPLY_LEN) {
            int r = uart_read_bytes(MOTOR_UART_PORT, reply + total, sizeof(reply) - total, pdMS_TO_TICKS(MOTOR_QUERY_RETRY_MS));
            if (r > 0) total += r;
            if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(MOTOR_QUERY_EXTRA_MS)) break;
        }
    }
    xSemaphoreGive(s_uart_mutex);

    char hex[64] = {0};
    int off = 0;
    for (int i = 0; i < total && off < 60; i++)
        off += snprintf(hex + off, sizeof(hex) - off, "%02X ", reply[i]);
    ESP_LOGI(TAG, "QRY M%d raw[%d]: %s", motor_id, total, hex);

    if (total < MOTOR_QUERY_REPLY_LEN) return ESP_ERR_TIMEOUT;

    int found = -1;
    for (int i = 0; i <= total - MOTOR_QUERY_REPLY_LEN; i++) {
        if (reply[i] == motor_id && reply[i + 1] == 0x32 && reply[i + 7] == 0x6B) {
            found = i; break;
        }
    }
    if (found < 0) return ESP_ERR_TIMEOUT;

    uint8_t *r = reply + found;
    int32_t pulse = (int32_t)(((uint32_t)r[3] << 24) |
                              ((uint32_t)r[4] << 16) |
                              ((uint32_t)r[5] << 8)  |
                               (uint32_t)r[6]);
    if (r[2] == 0x01) pulse = -pulse;

    int idx = motor_id - 1;
    if (!s_baseline_set[idx]) {
        s_pulse_baseline[idx] = pulse;
        s_baseline_set[idx] = true;
    }

    status->rpm      = 0;
    status->current  = 0;
    status->position = pulse - s_pulse_baseline[idx];
    return ESP_OK;
}
