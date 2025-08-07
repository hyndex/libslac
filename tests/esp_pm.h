#pragma once
#include <stdint.h>

#ifndef ESP_ERR_T_DEFINED
typedef int esp_err_t;
#define ESP_ERR_T_DEFINED
#endif
#ifndef ESP_OK
#define ESP_OK 0
#endif

typedef void* esp_pm_lock_handle_t;
#define ESP_PM_APB_FREQ_MAX 0

static inline esp_err_t esp_pm_lock_create(int, int, const char*, esp_pm_lock_handle_t*) { return ESP_OK; }
static inline esp_err_t esp_pm_lock_acquire(esp_pm_lock_handle_t) { return ESP_OK; }
static inline esp_err_t esp_pm_lock_release(esp_pm_lock_handle_t) { return ESP_OK; }
