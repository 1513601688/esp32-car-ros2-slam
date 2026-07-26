/**
 * IMU Sensor — 维特智能 JY901 驱动实现 (I2C 寄存器模式, max 100kHz)
 *
 * JY901 内置 Mahony 滤波 + AHRS 航姿融合。
 * I2C 寄存器访问: 写寄存器地址 → 读 int16 LE (每个寄存器占 2 字节)。
 *
 * 寄存器映射 (手册):
 *   0x34=AX, 0x35=AY, 0x36=AZ  (±16g,  /32768*16*9.81 → m/s²)
 *   0x37=GX, 0x38=GY, 0x39=GZ  (±2000°/s, /32768*2000   → °/s)
 *   0x3D=Roll, 0x3E=Pitch, 0x3F=Yaw  (±180°, /32768*180 → °)
 */

#include <string.h>
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "imu_sensor.h"

#define TAG "imu"

// JY901 数据寄存器 (每个寄存器 16-bit, 占 2 字节)
// 手册: 0x34=AX, 0x35=AY, 0x36=AZ, 0x37=GX, 0x38=GY, 0x39=GZ
//       0x3D=Roll, 0x3E=Pitch, 0x3F=Yaw
#define REG_ACCEL       0x34   // 6 字节: AX,AY,AZ
#define REG_GYRO        0x37   // 6 字节: GX,GY,GZ
#define REG_ATTITUDE    0x3D   // 6 字节: Roll,Pitch,Yaw

// 量程
#define ACC_SCALE       (32768.0f / 16.0f)   // LSB per g
#define GYRO_SCALE      (32768.0f / 2000.0f) // LSB per °/s
#define ANGLE_SCALE     (32768.0f / 180.0f)  // LSB per °
#define G               9.81f

static bool s_init_ok = false;
static bool s_ever_online = false;
static bool s_did_first_dump = false;
static imu_data_t s_latest;
static int s_read_ok = 0, s_read_fail = 0;
static int64_t s_dbg_next = 0;

// 读取一组 int16 (LE) 寄存器
static esp_err_t read_i16_block(uint8_t reg, int16_t *out, int count)
{
    uint8_t buf[12];  // max 6 int16 = 12 bytes
    size_t len = count * 2;
    if (len > sizeof(buf)) len = sizeof(buf);

    esp_err_t ret = i2c_master_write_read_device(IMU_I2C_PORT, JY901_I2C_ADDR,
                                                  &reg, 1,
                                                  buf, len,
                                                  pdMS_TO_TICKS(10));
    if (ret != ESP_OK) return ret;

    for (int i = 0; i < count; i++) {
        out[i] = (int16_t)(buf[i*2] | (buf[i*2 + 1] << 8));
    }
    return ESP_OK;
}

// 首次读取 dump 原始字节
static void first_dump(void)
{
    // 读 0x34-0x3F (accel 6B + gyro 6B = 12 bytes)
    uint8_t reg = 0x34;
    uint8_t buf[12];
    esp_err_t ret = i2c_master_write_read_device(IMU_I2C_PORT, JY901_I2C_ADDR,
                                                  &reg, 1,
                                                  buf, sizeof(buf),
                                                  pdMS_TO_TICKS(10));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "First dump I2C err=0x%x", ret);
        return;
    }

    char hex[100];
    int pos = 0;
    for (int i = 0; i < (int)sizeof(buf); i++) {
        pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", buf[i]);
    }
    ESP_LOGI(TAG, "Reg 0x34-0x3F dump: %s", hex);

    // 按手册: 每 2 字节一个 int16 LE (AX,AY,AZ,GX,GY,GZ)
    int16_t ax = (int16_t)(buf[0] | (buf[1] << 8));
    int16_t ay = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t az = (int16_t)(buf[4] | (buf[5] << 8));
    int16_t gx = (int16_t)(buf[6] | (buf[7] << 8));
    int16_t gy = (int16_t)(buf[8] | (buf[9] << 8));
    int16_t gz = (int16_t)(buf[10] |(buf[11] << 8));
    ESP_LOGI(TAG, "AX=%d AY=%d AZ=%d  GX=%d GY=%d GZ=%d",
             ax, ay, az, gx, gy, gz);
}

// ── 初始化 ──
esp_err_t imu_sensor_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = IMU_I2C_SDA,
        .scl_io_num = IMU_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = IMU_I2C_FREQ,
    };
    ESP_ERROR_CHECK(i2c_param_config(IMU_I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(IMU_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    // 探测
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (JY901_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t probe = i2c_master_cmd_begin(IMU_I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    if (probe != ESP_OK) {
        ESP_LOGW(TAG, "JY901 not found at I2C 0x%02x, err=0x%x", JY901_I2C_ADDR, probe);
        i2c_driver_delete(IMU_I2C_PORT);
        return ESP_ERR_NOT_FOUND;
    }

    s_init_ok = true;
    memset(&s_latest, 0, sizeof(s_latest));
    s_read_ok = s_read_fail = 0;
    s_dbg_next = esp_timer_get_time() + 2000000;

    ESP_LOGI(TAG, "JY901 found at I2C 0x%02x (SDA=IO%d,SCL=IO%d)",
             JY901_I2C_ADDR, IMU_I2C_SDA, IMU_I2C_SCL);
    return ESP_OK;
}

// ── 读取所有传感器值 ──
esp_err_t imu_sensor_read(imu_data_t *data)
{
    if (!s_init_ok || !data) return ESP_ERR_INVALID_STATE;
    // 首次读取时 dump 寄存器原始数据 (诊断用)
    if (!s_did_first_dump) {
        s_did_first_dump = true;
        first_dump();
    }

    imu_data_t next = s_latest;
    if (!next.valid) {
        memset(&next, 0, sizeof(next));
    }

    int16_t raw[3];
    bool any_ok = false;
    bool gyro_ok = false;

    // 加速度 (0x34-0x36)
    if (read_i16_block(REG_ACCEL, raw, 3) == ESP_OK) {
        next.ax = raw[0] / ACC_SCALE * G;
        next.ay = raw[1] / ACC_SCALE * G;
        next.az = raw[2] / ACC_SCALE * G;
        s_read_ok++;
        any_ok = true;
    } else {
        s_read_fail++;
    }

    // 角速度 (0x37-0x39)
    if (read_i16_block(REG_GYRO, raw, 3) == ESP_OK) {
        next.gx = raw[0] / GYRO_SCALE;
        next.gy = raw[1] / GYRO_SCALE;
        next.gz = raw[2] / GYRO_SCALE;
        s_read_ok++;
        any_ok = true;
        gyro_ok = true;
    } else {
        s_read_fail++;
    }

    // 姿态角 (0x3D-0x3F)
    if (read_i16_block(REG_ATTITUDE, raw, 3) == ESP_OK) {
        next.roll  = raw[0] / ANGLE_SCALE;
        next.pitch = raw[1] / ANGLE_SCALE;
        next.yaw   = raw[2] / ANGLE_SCALE;
        s_read_ok++;
        any_ok = true;
    } else {
        s_read_fail++;
    }

    bool publish_ok = any_ok && gyro_ok;
    if (publish_ok) {
        next.valid = true;
        memcpy(data, &next, sizeof(*data));
        memcpy(&s_latest, &next, sizeof(s_latest));
    }

    if (publish_ok) {
        if (!s_ever_online) {
            s_ever_online = true;
            ESP_LOGI(TAG, "JY901 online! yaw=%.1f pitch=%.1f roll=%.1f",
                     next.yaw, next.pitch, next.roll);
        }
    }

    // 诊断日志
    int64_t now = esp_timer_get_time();
    if (now > s_dbg_next) {
        s_dbg_next = now + 5000000;
        if (s_read_fail > 0) {
            ESP_LOGW(TAG, "I2C stats: ok=%d fail=%d", s_read_ok, s_read_fail);
        }
    }

    return publish_ok ? ESP_OK : ESP_FAIL;
}
