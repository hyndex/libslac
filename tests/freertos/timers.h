#pragma once
#include "FreeRTOS.h"
#ifdef __cplusplus
extern "C" {
#endif
TimerHandle_t xTimerCreate(const char* name, uint32_t period, int auto_reload, void* arg, TimerCallbackFunction_t cb);
void xTimerStart(TimerHandle_t timer, uint32_t ticks);
void xTimerStop(TimerHandle_t timer, uint32_t ticks);
void xTimerChangePeriod(TimerHandle_t timer, uint32_t ticks, uint32_t block);
#ifdef __cplusplus
}
#endif
