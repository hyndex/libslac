#pragma once
#include <Arduino.h>
#include "cp_config.h"

enum CpSubState : uint8_t { CP_A, CP_B1, CP_B2, CP_B3, CP_C, CP_D, CP_E, CP_F };

void     cpMonitorInit();
void     cpLowRateStart(uint32_t period_ms = 5);
void     cpLowRateStop();

uint16_t cpGetVoltageMv();
CpSubState cpGetSubState();
char     cpGetStateLetter();

void     cpSetLastPwmDuty(uint16_t duty);
uint16_t cpGetLastPwmDuty();

bool     cpDigitalCommRequested();
