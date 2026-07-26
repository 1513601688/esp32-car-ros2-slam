/**
 * Web Server — HTTP + WebSocket (manual WS protocol)
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_spiffs.h"
#include "esp_vfs.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "cJSON.h"
#include "web_server.h"
#include "uart_motor.h"
#include "ros_serial.h"
#include "car_config.h"

#define TAG "web"

#define WS_MAX_CLIENTS              3
#define WS_RX_BUF_SIZE              512
#define HTTP_MAX_OPEN_SOCKETS       5
#define WS_PING_INTERVAL_TICKS      50
#define MAX_VELOCITY_DT_SEC         0.5f
#define PI_F                        3.14159265f
#define SQRT2_F                     1.41421356f
#define YAW_HOLD_WHEEL_DELTA_PULSES 2
#define YAW_HOLD_GYRO_DPS           0.8f
#define YAW_HOLD_SETTLE_US          300000LL
#define ODOM_SPEED_FILTER_ALPHA     0.65f
#define ODOM_ZERO_WHEEL_DELTA_PULSES 2
#define IMU_GYRO_YAW_DEADBAND_DPS   0.35f
#define IMU_MAX_DT_SEC              0.2f

// ——— 日志环形缓冲区 ——— //
#define LOG_BUF_LINES 40
#define LOG_BUF_LEN   160
static char s_log_buf[LOG_BUF_LINES][LOG_BUF_LEN];
static int  s_log_idx, s_log_count;

static int log_vprintf(const char *fmt, va_list args)
{
    vsnprintf(s_log_buf[s_log_idx], LOG_BUF_LEN, fmt, args);
    s_log_idx = (s_log_idx + 1) % LOG_BUF_LINES;
    if (s_log_count < LOG_BUF_LINES) s_log_count++;
    return vprintf(fmt, args);
}

// ——— 全局 ——— //
static httpd_handle_t s_httpd;
static int s_ws_fds[WS_MAX_CLIENTS];
static SemaphoreHandle_t s_state_mutex;
static motor_telemetry_t s_telemetry;
static imu_data_t s_imu;
static esp_timer_handle_t s_telem_timer;

// ── 速度推算 (脉冲 → 物理速度) ──
// 四 omni 轮位于正方形四角，主动滚动方向与车体中心圆相切。
static int32_t s_prev_pulse[CAR_MOTOR_COUNT] = {0};
static int64_t s_prev_velo_us = 0;
static int64_t s_next_odom_log_us = 0;
static int32_t s_odom_delta[CAR_MOTOR_COUNT] = {0};
static float s_odom_wheel_speed[CAR_MOTOR_COUNT] = {0};
static float s_odom_vx = 0.0f;
static float s_odom_vy = 0.0f;
static float s_odom_wz = 0.0f;
static float s_odom_yaw = 0.0f;
static bool s_odom_yaw_held = false;
static bool s_odom_filter_ready = false;
static bool s_yaw_hold_active = false;
static float s_yaw_hold_deg = 0.0f;
static float s_yaw_hold_candidate_deg = 0.0f;
static int64_t s_yaw_static_since_us = 0;
static bool s_gyro_yaw_ready = false;
static float s_gyro_yaw_deg = 0.0f;
static int64_t s_prev_imu_us = 0;

typedef struct {
    motor_telemetry_t telemetry;
    imu_data_t imu;
    float odom_vx;
    float odom_vy;
    float odom_yaw;
    float gyro_yaw_deg;
    bool odom_yaw_held;
} telemetry_snapshot_t;

static char *build_telemetry_json(void);
static void reset_odom_state(void);
static void zero_odom_velocity(void);
static void reset_yaw_state(void);
static void update_gyro_yaw(int64_t now);
static void update_odom_from_pulses(void);
static void sync_ros_wheel_odom(void);

static void state_lock(void)
{
    if (s_state_mutex) {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    }
}

static void state_unlock(void)
{
    if (s_state_mutex) {
        xSemaphoreGive(s_state_mutex);
    }
}

static void get_telemetry_snapshot(telemetry_snapshot_t *snapshot)
{
    if (!snapshot) return;

    state_lock();
    memcpy(&snapshot->telemetry, &s_telemetry, sizeof(snapshot->telemetry));
    memcpy(&snapshot->imu, &s_imu, sizeof(snapshot->imu));
    snapshot->odom_vx = s_odom_vx;
    snapshot->odom_vy = s_odom_vy;
    snapshot->odom_yaw = s_odom_yaw;
    snapshot->gyro_yaw_deg = s_gyro_yaw_deg;
    snapshot->odom_yaw_held = s_odom_yaw_held;
    state_unlock();
}

// ——— WS frame dispatch ——— //

typedef struct {
    int fd;
    httpd_ws_type_t type;
    char *payload;
    size_t len;
} ws_async_msg_t;

static void ws_client_remove(int fd)
{
    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
        if (s_ws_fds[i] == fd) {
            s_ws_fds[i] = 0;
            break;
        }
    }
}

static void ws_send_work(void *arg)
{
    ws_async_msg_t *msg = (ws_async_msg_t *)arg;
    if (!msg) return;

    if (s_httpd && httpd_ws_get_fd_info(s_httpd, msg->fd) == HTTPD_WS_CLIENT_WEBSOCKET) {
        httpd_ws_frame_t frame = {
            .final = true,
            .type = msg->type,
            .payload = (uint8_t *)msg->payload,
            .len = msg->len,
        };
        esp_err_t ret = httpd_ws_send_frame_async(s_httpd, msg->fd, &frame);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "WS send failed fd=%d: %s", msg->fd, esp_err_to_name(ret));
            ws_client_remove(msg->fd);
        }
    } else {
        ws_client_remove(msg->fd);
    }

    free(msg->payload);
    free(msg);
}

static bool ws_queue_send(int fd, httpd_ws_type_t type, const char *data)
{
    if (!s_httpd || !data) return false;

    ws_async_msg_t *msg = calloc(1, sizeof(*msg));
    if (!msg) return false;

    msg->len = strlen(data);
    msg->payload = malloc(msg->len + 1);
    if (!msg->payload) {
        free(msg);
        return false;
    }

    memcpy(msg->payload, data, msg->len + 1);
    msg->fd = fd;
    msg->type = type;

    esp_err_t ret = httpd_queue_work(s_httpd, ws_send_work, msg);
    if (ret != ESP_OK) {
        free(msg->payload);
        free(msg);
        ESP_LOGW(TAG, "WS queue send failed fd=%d: %s", fd, esp_err_to_name(ret));
        return false;
    }
    return true;
}

static void ws_broadcast(const char *data)
{
    if (!data) return;

    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
        int fd = s_ws_fds[i];
        if (fd) {
            ws_queue_send(fd, HTTPD_WS_TYPE_TEXT, data);
        }
    }
}

static void ws_ping_all(void)
{
    // Telemetry frames are sent every 100ms, so an extra server ping is not needed.
    // Avoid queueing control frames from the timer task; slow WiFi can otherwise
    // make the UI reconnect even though the control path is still healthy.
}

// ——— 日志广播 ——— //

// ——— WS 客户端 ——— //

static bool ws_client_add(int fd)
{
    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
        if (s_ws_fds[i] == fd) return true;
        if (!s_ws_fds[i]) {
            s_ws_fds[i] = fd;
            ESP_LOGI(TAG, "WS client fd=%d", fd);
            ws_queue_send(fd, HTTPD_WS_TYPE_TEXT, "{\"msg\":\"connected\"}");
            char *json = build_telemetry_json();
            if (json) {
                ws_queue_send(fd, HTTPD_WS_TYPE_TEXT, json);
                free(json);
            }
            return true;
        }
    }

    ESP_LOGW(TAG, "Too many WS clients, closing fd=%d", fd);
    return false;
}

// ——— JSON 命令 ——— //

static void handle_ws_command(const char *json_str)
{
    cJSON *root = cJSON_Parse(json_str);
    if (!root) return;
    cJSON *c = cJSON_GetObjectItem(root, "cmd");
    if (!c || !cJSON_IsString(c)) { cJSON_Delete(root); return; }

    if (strcmp(c->valuestring, "set_acc") == 0) {
        cJSON *v = cJSON_GetObjectItem(root, "value");
        if (v && cJSON_IsNumber(v))
            uart_motor_set_acc((uint8_t)v->valuedouble);
    } else if (strcmp(c->valuestring, "set_max_speed") == 0) {
        cJSON *v = cJSON_GetObjectItem(root, "value");
        if (v && cJSON_IsNumber(v))
            uart_motor_set_max_rpm((uint16_t)v->valuedouble);
    } else if (strcmp(c->valuestring, "joystick") == 0) {
        cJSON *x = cJSON_GetObjectItem(root, "x");
        cJSON *y = cJSON_GetObjectItem(root, "y");
        cJSON *r = cJSON_GetObjectItem(root, "rot");
        if (x && y && cJSON_IsNumber(x) && cJSON_IsNumber(y)) {
            float fx = (float)x->valuedouble, fy = (float)y->valuedouble;
            float fr = (r && cJSON_IsNumber(r)) ? (float)r->valuedouble : 0.0f;
            uart_motor_clear_stop_override();  // 新摇杆指令解除停止锁定
            uart_motor_joystick(fx, fy, fr);
            float len = sqrtf(fx*fx + fy*fy);
            state_lock();
            s_telemetry.speed_set = (uint8_t)(len * 100);
            snprintf(s_telemetry.direction, sizeof(s_telemetry.direction),
                     len < 0.02f ? "STOP" : "%.0f%%", len * 100);
            state_unlock();
        }
    } else if (strcmp(c->valuestring, "stop") == 0) {
        // 停止指令直接执行，不走队列（避免插队延迟）
        uart_motor_stop();
        state_lock();
        s_telemetry.speed_set = 0;
        snprintf(s_telemetry.direction, sizeof(s_telemetry.direction), "STOP");
        state_unlock();
    }
    else if (strcmp(c->valuestring, "zero_pulse") == 0) {
        uart_motor_zero_all();
        reset_odom_state();
    } else if (strcmp(c->valuestring, "zero_yaw") == 0) {
        reset_yaw_state();
    }
    cJSON_Delete(root);
}

// ——— 遥测 ——— //

static int32_t abs_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

static bool wheels_are_still(const int32_t delta[CAR_MOTOR_COUNT])
{
    for (int i = 0; i < CAR_MOTOR_COUNT; i++) {
        if (abs_i32(delta[i]) > ODOM_ZERO_WHEEL_DELTA_PULSES) {
            return false;
        }
    }
    return true;
}

static float wrap_angle_180(float deg)
{
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static void update_gyro_yaw(int64_t now)
{
    if (!s_imu.valid) {
        s_gyro_yaw_ready = false;
        s_prev_imu_us = 0;
        return;
    }

    if (!s_gyro_yaw_ready || s_prev_imu_us == 0) {
        s_gyro_yaw_ready = true;
        s_prev_imu_us = now;
        return;
    }

    float dt = (float)(now - s_prev_imu_us) / 1000000.0f;
    s_prev_imu_us = now;
    if (dt <= 0.0f || dt > IMU_MAX_DT_SEC) {
        return;
    }

    float gz = s_imu.gz * CAR_IMU_YAW_SIGN;
    if (fabsf(gz) < IMU_GYRO_YAW_DEADBAND_DPS) {
        return;
    }

    s_gyro_yaw_deg = wrap_angle_180(s_gyro_yaw_deg + gz * dt);
}

static float get_output_yaw(int64_t now, const int32_t delta[CAR_MOTOR_COUNT], bool *held)
{
    if (held) {
        *held = false;
    }

    if (!s_imu.valid) {
        s_yaw_hold_active = false;
        s_yaw_static_since_us = 0;
        return 0.0f;
    }

    bool stationary = wheels_are_still(delta) && fabsf(s_imu.gz) <= YAW_HOLD_GYRO_DPS;
    if (!stationary) {
        s_yaw_hold_active = false;
        s_yaw_static_since_us = 0;
        return s_gyro_yaw_deg;
    }

    if (s_yaw_static_since_us == 0) {
        s_yaw_static_since_us = now;
        s_yaw_hold_candidate_deg = s_gyro_yaw_deg;
    }

    if (!s_yaw_hold_active && (now - s_yaw_static_since_us) >= YAW_HOLD_SETTLE_US) {
        s_yaw_hold_active = true;
        s_yaw_hold_deg = s_yaw_hold_candidate_deg;
    }

    if (s_yaw_hold_active) {
        if (held) {
            *held = true;
        }
        return s_yaw_hold_deg;
    }

    return s_gyro_yaw_deg;
}

static void reset_yaw_state(void)
{
    state_lock();
    s_gyro_yaw_deg = 0.0f;
    s_gyro_yaw_ready = false;
    s_prev_imu_us = 0;
    s_yaw_hold_active = false;
    s_yaw_static_since_us = 0;
    s_odom_yaw = 0.0f;
    s_odom_yaw_held = false;
    state_unlock();
}

static void zero_odom_velocity(void)
{
    memset(s_odom_wheel_speed, 0, sizeof(s_odom_wheel_speed));
    s_odom_vx = 0.0f;
    s_odom_vy = 0.0f;
    s_odom_wz = 0.0f;
    s_odom_filter_ready = false;
}

static void reset_odom_state(void)
{
    state_lock();
    memcpy(s_prev_pulse, s_telemetry.position, sizeof(s_prev_pulse));
    memset(s_odom_delta, 0, sizeof(s_odom_delta));
    s_prev_velo_us = 0;
    zero_odom_velocity();
    s_odom_yaw = s_gyro_yaw_deg;
    s_odom_yaw_held = false;
    s_yaw_hold_active = false;
    s_yaw_static_since_us = 0;
    sync_ros_wheel_odom();
    state_unlock();
}

static void sync_ros_wheel_odom(void)
{
    ros_serial_update_wheel(s_odom_vx, s_odom_vy, s_odom_wz);
}

static void update_odom_from_pulses(void)
{
    int64_t now = esp_timer_get_time();
    if (s_prev_velo_us == 0) {
        memcpy(s_prev_pulse, s_telemetry.position, sizeof(s_prev_pulse));
        s_prev_velo_us = now;
        s_odom_yaw = s_gyro_yaw_deg;
        sync_ros_wheel_odom();
        return;
    }

    float dt = (float)(now - s_prev_velo_us) / 1000000.0f;
    if (dt <= 0.0f) return;
    if (dt > MAX_VELOCITY_DT_SEC) {
        memcpy(s_prev_pulse, s_telemetry.position, sizeof(s_prev_pulse));
        s_prev_velo_us = now;
        zero_odom_velocity();
        sync_ros_wheel_odom();
        return;
    }
    s_prev_velo_us = now;

    float scale = PI_F * CAR_WHEEL_DIAMETER_M / CAR_ENCODER_PULSES_PER_REV / dt;
    static const int8_t calib[CAR_MOTOR_COUNT] = {
        CAR_ENCODER_DIR_FL,
        CAR_ENCODER_DIR_FR,
        CAR_ENCODER_DIR_RL,
        CAR_ENCODER_DIR_RR,
    };

    for (int i = 0; i < CAR_MOTOR_COUNT; i++) {
        s_odom_delta[i] = s_telemetry.position[i] - s_prev_pulse[i];
        s_odom_wheel_speed[i] = (float)s_odom_delta[i] * scale * calib[i];
        s_prev_pulse[i] = s_telemetry.position[i];
    }

    float raw_vx = (SQRT2_F * 0.25f) * (s_odom_wheel_speed[0] - s_odom_wheel_speed[1] - s_odom_wheel_speed[2] + s_odom_wheel_speed[3]);
    float raw_vy = (SQRT2_F * 0.25f) * (s_odom_wheel_speed[0] + s_odom_wheel_speed[1] - s_odom_wheel_speed[2] - s_odom_wheel_speed[3]);
    float raw_wz = (s_odom_wheel_speed[0] + s_odom_wheel_speed[1] + s_odom_wheel_speed[2] + s_odom_wheel_speed[3]) /
                   (4.0f * CAR_WHEEL_CENTER_RADIUS_M);

    raw_vx *= CAR_ODOM_SCALE_VX;
    raw_vy *= CAR_ODOM_SCALE_VY;
    raw_wz *= CAR_ODOM_SCALE_WZ;

    if (wheels_are_still(s_odom_delta)) {
        raw_vx = 0.0f;
        raw_vy = 0.0f;
        raw_wz = 0.0f;
        zero_odom_velocity();
    } else if (!s_odom_filter_ready) {
        s_odom_vx = raw_vx;
        s_odom_vy = raw_vy;
        s_odom_wz = raw_wz;
        s_odom_filter_ready = true;
    } else {
        s_odom_vx += ODOM_SPEED_FILTER_ALPHA * (raw_vx - s_odom_vx);
        s_odom_vy += ODOM_SPEED_FILTER_ALPHA * (raw_vy - s_odom_vy);
        s_odom_wz += ODOM_SPEED_FILTER_ALPHA * (raw_wz - s_odom_wz);
    }
    s_odom_yaw = get_output_yaw(now, s_odom_delta, &s_odom_yaw_held);

    sync_ros_wheel_odom();

    if (now >= s_next_odom_log_us) {
        s_next_odom_log_us = now + 1000000;
        ESP_LOGI(TAG,
                 "ODOM delta=[%ld,%ld,%ld,%ld] wheel=[%.3f,%.3f,%.3f,%.3f] vx=%.3f raw_vx=%.3f vy=%.3f raw_vy=%.3f wz=%.3f yaw=%.1f yaw_gyro=%.1f raw_yaw=%.1f gz=%.2f yaw_hold=%d",
                 (long)s_odom_delta[0], (long)s_odom_delta[1], (long)s_odom_delta[2], (long)s_odom_delta[3],
                 (double)s_odom_wheel_speed[0], (double)s_odom_wheel_speed[1],
                 (double)s_odom_wheel_speed[2], (double)s_odom_wheel_speed[3],
                 (double)s_odom_vx, (double)raw_vx,
                 (double)s_odom_vy, (double)raw_vy,
                 (double)s_odom_wz,
                 (double)s_odom_yaw,
                 (double)s_gyro_yaw_deg,
                 (double)(s_imu.valid ? s_imu.yaw : 0.0f),
                 (double)(s_imu.valid ? s_imu.gz : 0.0f),
                 s_odom_yaw_held ? 1 : 0);
    }
}

static char *build_telemetry_json(void)
{
    telemetry_snapshot_t snap;
    get_telemetry_snapshot(&snap);

    cJSON *root = cJSON_CreateObject();
    cJSON *w = cJSON_CreateObject();
    cJSON_AddNumberToObject(w, "fl", snap.telemetry.rpm[0]);
    cJSON_AddNumberToObject(w, "fr", snap.telemetry.rpm[1]);
    cJSON_AddNumberToObject(w, "rl", snap.telemetry.rpm[2]);
    cJSON_AddNumberToObject(w, "rr", snap.telemetry.rpm[3]);
    cJSON_AddItemToObject(root, "wheels", w);
    // 脉冲数 (0x32 查询结果)
    cJSON *p = cJSON_CreateObject();
    cJSON_AddNumberToObject(p, "fl", snap.telemetry.position[0]);
    cJSON_AddNumberToObject(p, "fr", snap.telemetry.position[1]);
    cJSON_AddNumberToObject(p, "rl", snap.telemetry.position[2]);
    cJSON_AddNumberToObject(p, "rr", snap.telemetry.position[3]);
    cJSON_AddItemToObject(root, "pulses", p);
    cJSON_AddNumberToObject(root, "speed", snap.telemetry.speed_set);
    cJSON_AddStringToObject(root, "dir", snap.telemetry.direction);

    // ── 脉冲 → 底盘速度 (四角 omni, 轮径35mm, 轮心半径170mm) ──
    float vx = snap.odom_vx;
    float vy = snap.odom_vy;
    float output_yaw = snap.odom_yaw;
    bool yaw_held = snap.odom_yaw_held;

    // IMU 数据: yaw is stabilized for display/ROS; yaw_raw keeps sensor output visible.
    if (snap.imu.valid) {
        cJSON *imu = cJSON_CreateObject();
        cJSON_AddNumberToObject(imu, "ax", (double)snap.imu.ax);
        cJSON_AddNumberToObject(imu, "ay", (double)snap.imu.ay);
        cJSON_AddNumberToObject(imu, "az", (double)snap.imu.az);
        cJSON_AddNumberToObject(imu, "gx", (double)snap.imu.gx);
        cJSON_AddNumberToObject(imu, "gy", (double)snap.imu.gy);
        cJSON_AddNumberToObject(imu, "gz", (double)snap.imu.gz);
        cJSON_AddNumberToObject(imu, "pitch", (double)snap.imu.pitch);
        cJSON_AddNumberToObject(imu, "roll",  (double)snap.imu.roll);
        cJSON_AddNumberToObject(imu, "yaw",   (double)output_yaw);
        cJSON_AddNumberToObject(imu, "yaw_raw", (double)snap.imu.yaw);
        cJSON_AddNumberToObject(imu, "yaw_gyro", (double)snap.gyro_yaw_deg);
        cJSON_AddBoolToObject(imu, "yaw_hold", yaw_held);
        cJSON_AddItemToObject(root, "imu", imu);
    }

    cJSON *odo = cJSON_CreateObject();
    cJSON_AddNumberToObject(odo, "vx", (double)vx);
    cJSON_AddNumberToObject(odo, "vy", (double)vy);
    cJSON_AddNumberToObject(odo, "yaw", (double)output_yaw);
    cJSON_AddItemToObject(root, "odom", odo);

    char *js = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return js;
}

static int s_ping_counter = 0;

static void telemetry_timer_cb(void *arg)
{
    char *json = build_telemetry_json();
    if (json) {
        ws_broadcast(json);
        free(json);
    }

    // 每 5 秒发一次 WebSocket Ping，防止路由/浏览器断连
    if (++s_ping_counter >= WS_PING_INTERVAL_TICKS) {  // 50 * 100ms = 5s
        s_ping_counter = 0;
        ws_ping_all();
    }
}

// ——— SPIFFS ——— //

static esp_err_t spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs", .partition_label = "spiffs",
        .max_files = 5, .format_if_mount_failed = true,
    };
    esp_err_t r = esp_vfs_spiffs_register(&conf);
    if (r != ESP_OK) { ESP_LOGE(TAG, "SPIFFS fail"); return r; }
    size_t total, used;
    esp_spiffs_info("spiffs", &total, &used);
    ESP_LOGI(TAG, "SPIFFS: %dKB/%dKB", used/1024, total/1024);
    return ESP_OK;
}

// ——— HTTP ——— //

static esp_err_t http_index_handler(httpd_req_t *req)
{
    FILE *f = fopen("/spiffs/index.html", "r");
    if (!f) { httpd_resp_send(req, "<h1>404</h1>", 11); return ESP_FAIL; }
    fseek(f, 0, SEEK_END);
    size_t sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = malloc(sz + 1);
    if (!buf) { fclose(f); httpd_resp_send_500(req); return ESP_FAIL; }
    fread(buf, 1, sz, f); buf[sz] = 0; fclose(f);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send(req, buf, sz);
    free(buf);
    return ESP_OK;
}

// ——— WS 升级 ——— //

static esp_err_t ws_upgrade_handler(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WS handshake ok fd=%d", fd);
        return ws_client_add(fd) ? ESP_OK : ESP_FAIL;
    }

    httpd_ws_frame_t frame = {0};
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WS recv len failed fd=%d: %s", fd, esp_err_to_name(ret));
        ws_client_remove(fd);
        return ret;
    }

    if (frame.type == HTTPD_WS_TYPE_CLOSE) {
        ws_client_remove(fd);
        ESP_LOGI(TAG, "WS closed fd=%d", fd);
        return ESP_OK;
    }

    if (frame.type != HTTPD_WS_TYPE_TEXT || frame.len == 0) {
        return ESP_OK;
    }

    if (frame.len >= WS_RX_BUF_SIZE) {
        ESP_LOGW(TAG, "WS frame too large fd=%d len=%u", fd, (unsigned)frame.len);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t *buf = calloc(1, frame.len + 1);
    if (!buf) return ESP_ERR_NO_MEM;

    frame.payload = buf;
    ret = httpd_ws_recv_frame(req, &frame, frame.len);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "WS data: %s", (char *)buf);
        handle_ws_command((const char *)buf);
    } else {
        ESP_LOGW(TAG, "WS recv payload failed fd=%d: %s", fd, esp_err_to_name(ret));
        ws_client_remove(fd);
    }

    free(buf);
    return ESP_OK;
}

// ——— Public API ——— //

void web_server_log_init(void)
{
    memset(s_log_buf, 0, sizeof(s_log_buf));
    s_log_idx = s_log_count = 0;
    esp_log_set_vprintf(log_vprintf);
    strncpy(s_log_buf[0], "[system] Log capture ready", LOG_BUF_LEN - 1);
    s_log_idx = 1; s_log_count = 1;
}

esp_err_t web_server_start(void)
{
    s_state_mutex = xSemaphoreCreateMutex();
    if (!s_state_mutex) {
        ESP_LOGE(TAG, "Failed to create telemetry state mutex");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = spiffs_mount();
    if (ret != ESP_OK) return ret;

    memset(s_ws_fds, 0, sizeof(s_ws_fds));
    memset(&s_telemetry, 0, sizeof(s_telemetry));
    snprintf(s_telemetry.direction, sizeof(s_telemetry.direction), "STOP");

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CAR_HTTP_PORT;
    config.max_open_sockets = HTTP_MAX_OPEN_SOCKETS;
    config.lru_purge_enable = true;

    ret = httpd_start(&s_httpd, &config);
    if (ret != ESP_OK) return ret;

    httpd_uri_t u1 = { .uri = "/",  .method = HTTP_GET, .handler = http_index_handler };
    httpd_uri_t u2 = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_upgrade_handler,
        .is_websocket = true,
    };
    ret = httpd_register_uri_handler(s_httpd, &u1);
    if (ret != ESP_OK) return ret;

    ret = httpd_register_uri_handler(s_httpd, &u2);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "HTTP+WS started");

    esp_timer_create_args_t ta = { .callback = telemetry_timer_cb, .name = "telem" };
    ret = esp_timer_create(&ta, &s_telem_timer);
    if (ret != ESP_OK) return ret;

    ret = esp_timer_start_periodic(s_telem_timer, CAR_TELEMETRY_PERIOD_US);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

void web_server_update_motor_data(const int32_t rpm[4], const float current[4], const int32_t pos[4])
{
    state_lock();
    memcpy(s_telemetry.rpm,     rpm,     sizeof(s_telemetry.rpm));
    memcpy(s_telemetry.current, current, sizeof(s_telemetry.current));
    memcpy(s_telemetry.position,pos,     sizeof(s_telemetry.position));
    update_odom_from_pulses();
    state_unlock();
}

void web_server_update_imu(const imu_data_t *imu)
{
    if (imu) {
        state_lock();
        memcpy(&s_imu, imu, sizeof(s_imu));
        if (s_imu.valid) {
            int64_t now = esp_timer_get_time();
            update_gyro_yaw(now);
            s_odom_yaw = get_output_yaw(now, s_odom_delta, &s_odom_yaw_held);
            ros_serial_update_imu(&s_imu);
        }
        state_unlock();
    }
}
