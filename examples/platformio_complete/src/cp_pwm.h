#pragma once
#include <Arduino.h>
#include "cp_config.h"

void cpPwmInit();
void cpPwmStart(uint16_t duty_raw = CP_PWM_DUTY_5PCT);
void cpPwmStop();
void cpPwmSetDuty(uint16_t duty_raw);
