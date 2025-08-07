#pragma once
#include <stdint.h>
#include "esp_intr_alloc.h"
#include "gpio.h"


typedef struct {
    struct { uint32_t lstimer0_ovf; } int_ena;
    struct { uint32_t lstimer0_ovf; uint32_t val; } int_clr;
    struct { uint32_t val; } int_st;
} ledc_dev_t;

extern ledc_dev_t LEDC;

#ifndef ESP_ERR_T_DEFINED
typedef int esp_err_t;
#define ESP_ERR_T_DEFINED
#endif
#ifndef ESP_OK
#define ESP_OK 0
#endif

typedef int ledc_mode_t;
typedef int ledc_channel_t;
typedef int ledc_timer_t;
typedef int ledc_timer_bit_t;
typedef int ledc_clk_cfg_t;

#define LEDC_LOW_SPEED_MODE 0
#define LEDC_TIMER_0 0
#define LEDC_AUTO_CLK 0

typedef struct {
    ledc_mode_t speed_mode;
    ledc_timer_bit_t duty_resolution;
    ledc_timer_t timer_num;
    uint32_t freq_hz;
    ledc_clk_cfg_t clk_cfg;
} ledc_timer_config_t;

typedef struct {
    int gpio_num;
    ledc_mode_t speed_mode;
    ledc_channel_t channel;
    ledc_timer_t timer_sel;
    uint32_t duty;
    int hpoint;
} ledc_channel_config_t;

static inline esp_err_t ledc_timer_config(const ledc_timer_config_t*) { return ESP_OK; }
static inline esp_err_t ledc_channel_config(const ledc_channel_config_t*) { return ESP_OK; }

extern gpio_mode_t g_gpio_mode_at_ledc_set_duty;
extern uint32_t g_ledc_last_duty;

static inline esp_err_t ledc_set_duty(ledc_mode_t, ledc_channel_t, uint32_t duty) {
    g_gpio_mode_at_ledc_set_duty = g_last_gpio_mode;
    g_ledc_last_duty = duty;
    return ESP_OK;
}

static inline esp_err_t ledc_update_duty(ledc_mode_t, ledc_channel_t) { return ESP_OK; }
static inline esp_err_t ledc_stop(ledc_mode_t, ledc_channel_t, uint32_t) { return ESP_OK; }

esp_err_t ledc_isr_register(void (*fn)(void*), void* arg, int flags, intr_handle_t* handle);
