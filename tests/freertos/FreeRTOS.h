#pragma once
#include <stdint.h>
typedef int BaseType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(ms) (ms)
#define portMAX_DELAY 0xFFFFFFFFUL
#define portYIELD_FROM_ISR(...)
typedef void* TimerHandle_t;
typedef void (*TimerCallbackFunction_t)(TimerHandle_t);
