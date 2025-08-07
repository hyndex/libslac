#pragma once
#include "FreeRTOS.h"
typedef uint32_t TickType_t;
inline void vTaskDelay(TickType_t) {}
inline void vTaskDelayUntil(TickType_t*, TickType_t) {}
inline void xTaskCreatePinnedToCore(void (*)(void*), const char*, uint32_t, void*, uint32_t, void*, int) {}
