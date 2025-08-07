#pragma once
#include "cp_config.h"
#include <stdint.h>

enum CpSubState : uint8_t {
    CP_A,
    CP_B1,
    CP_B2,
    CP_B3,
    CP_C,
    CP_D,
    CP_E,
    CP_F
};

void cpMonitorInit();
void cpMonitorStop();

uint16_t cpGetVoltageMv();
uint16_t cpGetVoltageMinMv();
uint16_t voutGetVoltageMv();
uint16_t voutGetVoltageRaw();
CpSubState cpGetSubState();
char cpGetStateLetter();

void cpSetLastPwmDuty(uint16_t duty);
uint16_t cpGetLastPwmDuty();
uint16_t cpGetMeasuredDuty();

bool cpDigitalCommRequested();

#ifdef LIBSLAC_TESTING
void cpMonitorTestProcess();
#endif
