#pragma once
#include <stdint.h>
#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(ms) (ms)
typedef void* TimerHandle_t;
typedef void (*TimerCallbackFunction_t)(TimerHandle_t);
