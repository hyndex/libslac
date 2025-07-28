#pragma once
#include <stdint.h>
#include "esp_intr_alloc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    struct { uint32_t lstimer0_ovf; } int_ena;
    struct { uint32_t lstimer0_ovf; uint32_t val; } int_clr;
    struct { uint32_t val; } int_st;
} ledc_dev_t;

extern ledc_dev_t LEDC;

esp_err_t ledc_isr_register(void (*fn)(void*), void* arg, int flags, intr_handle_t* handle);

#ifdef __cplusplus
}
#endif
