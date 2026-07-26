#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int32_t rpm;
    float current;
    int32_t position;
} motor_status_t;

esp_err_t uart_motor_init(uint8_t tx_pin, uint8_t rx_pin, uint32_t baudrate);
esp_err_t uart_motor_joystick(float x, float y, float rot);
esp_err_t uart_motor_stop(void);
void uart_motor_clear_stop_override(void);
void uart_motor_zero_all(void);
void uart_motor_watchdog(void);
void uart_motor_set_acc(uint8_t acc);
void uart_motor_set_max_rpm(uint16_t rpm);
esp_err_t uart_motor_query(uint8_t motor_id, motor_status_t *status);

#ifdef __cplusplus
}
#endif
