#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct hw_timer_s hw_timer_t;

hw_timer_t* timerBegin(int timer, uint16_t divider, bool countUp);
void timerAttachInterrupt(hw_timer_t* timer, void (*fn)(), bool edge);
void timerAlarmWrite(hw_timer_t* timer, uint64_t alarm_value, bool auto_reload);
void timerAlarmEnable(hw_timer_t* timer);
void timerAlarmDisable(hw_timer_t* timer);
void timerWrite(hw_timer_t* timer, uint64_t val);

#ifdef __cplusplus
}
#endif
