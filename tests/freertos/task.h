#pragma once
#include "FreeRTOS.h"
typedef void* TaskHandle_t;
typedef uint32_t TickType_t;
inline void vTaskDelay(TickType_t) {}
inline void vTaskDelayUntil(TickType_t*, TickType_t) {}
inline void xTaskCreatePinnedToCore(void (*)(void*), const char*, uint32_t, void*, uint32_t, TaskHandle_t*, int) {}
uint32_t ulTaskNotifyTake(BaseType_t, TickType_t);
void vTaskNotifyGiveFromISR(TaskHandle_t, BaseType_t*);
