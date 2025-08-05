#pragma once
#include <stdint.h>

typedef enum {
    GPIO_INTR_DISABLE = 0,
    GPIO_INTR_POSEDGE = 1,
    GPIO_INTR_NEGEDGE = 2,
} gpio_int_type_t;

typedef enum {
    GPIO_MODE_INPUT = 0,
} gpio_mode_t;

typedef enum {
    GPIO_PULLUP_DISABLE = 0,
    GPIO_PULLUP_ENABLE = 1,
} gpio_pullup_t;

typedef int gpio_num_t;
typedef void (*gpio_isr_t)(void*);

typedef int esp_err_t;
#define ESP_OK 0

typedef struct {
    uint64_t pin_bit_mask;
    gpio_mode_t mode;
    gpio_pullup_t pull_up_en;
    gpio_int_type_t intr_type;
} gpio_config_t;

#ifndef GPIO_STUB_CUSTOM
static inline esp_err_t gpio_config(const gpio_config_t*) { return ESP_OK; }
static inline esp_err_t gpio_install_isr_service(int) { return ESP_OK; }
static inline esp_err_t gpio_isr_handler_add(gpio_num_t, gpio_isr_t, void*) { return ESP_OK; }
#else
static esp_err_t gpio_config(const gpio_config_t* cfg);
static esp_err_t gpio_install_isr_service(int);
static esp_err_t gpio_isr_handler_add(gpio_num_t, gpio_isr_t, void*);
#endif

