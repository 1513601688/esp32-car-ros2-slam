#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 WiFi STA 模式并连接路由器
 *
 * 连接成功后会设置 mDNS 域名 esp32car.local
 *
 * @return ESP_OK 成功，其他值失败
 */
esp_err_t wifi_manager_init(void);

#ifdef __cplusplus
}
#endif
