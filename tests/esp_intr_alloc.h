#pragma once
#include <stdint.h>

#define IRAM_ATTR
#define DRAM_ATTR

#ifdef __cplusplus
extern "C" {
#endif

typedef void* intr_handle_t;
typedef int esp_err_t;

#define ESP_INTR_FLAG_IRAM 1
#define ESP_OK 0

esp_err_t esp_intr_free(intr_handle_t handle);

#ifdef __cplusplus
}
#endif
