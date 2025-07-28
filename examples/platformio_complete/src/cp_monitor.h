#pragma once
#include <Arduino.h>
#include "cp_config.h"

enum CpSubState : uint8_t { CP_A, CP_B1, CP_B2, CP_B3, CP_C, CP_D, CP_E, CP_F };

void     cpMonitorInit();
#if CP_USE_DMA_ADC
void     cpLowRateStart(uint32_t period_ms = 5);
void     cpLowRateStop();
static inline void cpFastSampleStart() {}
static inline void cpFastSampleStop() {}
void     cpDmaStart();
void     cpDmaStop();
#else
void     cpLowRateStart(uint32_t period_ms = 5);
void     cpLowRateStop();
void     cpFastSampleStart();
void     cpFastSampleStop();
static inline void cpDmaStart() {}
static inline void cpDmaStop() {}
#endif

uint16_t cpGetVoltageMv();
CpSubState cpGetSubState();
char     cpGetStateLetter();

void     cpSetLastPwmDuty(uint16_t duty);
uint16_t cpGetLastPwmDuty();

bool     cpDigitalCommRequested();
