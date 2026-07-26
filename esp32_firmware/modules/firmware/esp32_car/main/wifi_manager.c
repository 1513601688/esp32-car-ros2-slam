/**
 * WiFi Manager — STA 模式连接路由器 + mDNS
 *
 * 功能:
 *   1. STA 模式连接指定路由器
 *   2. 断线自动重连（由 WiFi 事件驱动）
 *   3. 启动 mDNS，域名 esp32car.local
 *   4. 连接成功后串口打印 IP
 */

#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mdns.h"
#include "wifi_manager.h"

#define TAG "wifi"

// WiFi 配置来自 menuconfig (Kconfig.projbuild)
// 运行 idf.py menuconfig → ESP32 Car Configuration → WiFi Settings 修改

// 事件标志位
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

static void wifi_event_handler_cb(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *ev =
                (wifi_event_sta_disconnected_t *)event_data;
            if (s_retry_num < CONFIG_CAR_WIFI_MAX_RETRY) {
                esp_wifi_connect();
                s_retry_num++;
                ESP_LOGI(TAG, "WiFi 断连，重试第 %d 次，原因: %d", s_retry_num, ev->reason);
            } else {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                ESP_LOGE(TAG, "WiFi 连接失败，已重试 %d 次", CONFIG_CAR_WIFI_MAX_RETRY);
            }
            break;
        }
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "WiFi 已关联到 AP");
            break;

        default:
            break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "✅ 获取 IP: " IPSTR, IP2STR(&ev->ip_info.ip));

        // 启动 mDNS
        esp_err_t err = mdns_init();
        if (err == ESP_OK) {
            mdns_hostname_set("esp32car");
            mdns_instance_name_set("ESP32 Car Controller");
            ESP_LOGI(TAG, "✅ mDNS 已启动: http://esp32car.local");
        } else {
            ESP_LOGW(TAG, "⚠️  mDNS 启动失败: %s", esp_err_to_name(err));
        }
    }
}

esp_err_t wifi_manager_init(void)
{
    s_wifi_event_group = xEventGroupCreate();

    // 1. 初始化 TCP/IP 协议栈
    ESP_ERROR_CHECK(esp_netif_init());

    // 2. 创建默认事件循环
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 3. 创建 STA 接口
    esp_netif_create_default_wifi_sta();

    // 4. WiFi 默认初始化
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 5. 注册事件回调
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler_cb, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler_cb, NULL, NULL));

    // 6. 配置 STA 参数
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_CAR_WIFI_SSID,
            .password = CONFIG_CAR_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,   // 现在最常用
            .pmf_cfg = { .capable = true, .required = false },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // 7. 启动 WiFi
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);  // 禁用省电模式，避免延迟和断连
    ESP_LOGI(TAG, "WiFi STA 初始化完成，开始连接 %s...", CONFIG_CAR_WIFI_SSID);

    // 8. 等待连接结果
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,   // 不自动清除
        pdFALSE,   // 任一位置位即返回
        portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "🎉 WiFi 连接成功！");
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "❌ WiFi 连接失败！请检查 SSID 和密码");
        return ESP_FAIL;
    }

    ESP_LOGE(TAG, "❌ 未知错误");
    return ESP_FAIL;
}
