#include "cp_monitor.h"

static hw_timer_t* adcTimer = nullptr;
static volatile uint16_t cp_mv = 0;
static volatile uint16_t cp_duty = 0;
static volatile CpSubState cp_state = CP_A;

static char toLetter(CpSubState s) {
    switch (s) {
        case CP_A: return 'A';
        case CP_B1: case CP_B2: case CP_B3: return 'B';
        case CP_C: return 'C';
        case CP_D: return 'D';
        case CP_E: return 'E';
        case CP_F: return 'F';
        default:   return '?';
    }
}

static CpSubState mv2state(uint16_t mv) {
    if (mv < CP_THR_1V)      return CP_F;
    if (mv > CP_THR_12V)     return CP_A;
    if (mv > CP_THR_9V)      return (cp_duty == 0) ? CP_B1 : CP_B3;
    if (mv > CP_THR_6V) {
        uint16_t pct = (cp_duty * 100) >> CP_PWM_RES_BITS;
        if (pct >= 3 && pct <= 7) return CP_B2;
        return CP_C;
    }
    if (mv > CP_THR_3V)      return CP_D;
    return CP_E;
}

static void IRAM_ATTR onAdc() {
    uint16_t vmax = 0;
    const uint8_t N = 8;
    for (uint8_t i = 0; i < N; ++i) {
        uint16_t v = analogReadMilliVolts(CP_READ_ADC_PIN);
        if (v > vmax)
            vmax = v;
    }
    cp_mv = vmax;
    cp_state = mv2state(vmax);
}

void cpMonitorInit() {
    analogReadResolution(12);
    analogSetPinAttenuation(CP_READ_ADC_PIN, ADC_11db);
    cp_mv = analogReadMilliVolts(CP_READ_ADC_PIN);
    cp_state = mv2state(cp_mv);
}

void cpLowRateStart(uint32_t period_ms) {
    if (!adcTimer)
        adcTimer = timerBegin(3, 80, true);
    timerAttachInterrupt(adcTimer, &onAdc, true);
    timerAlarmWrite(adcTimer, period_ms * 1000, true);
    timerAlarmEnable(adcTimer);
}

void cpLowRateStop() {
    if (adcTimer)
        timerAlarmDisable(adcTimer);
}

float cpGetVoltageMv() { return static_cast<float>(cp_mv); }
CpSubState cpGetSubState() { return cp_state; }
char cpGetStateLetter() { return toLetter(cp_state); }

void cpSetLastPwmDuty(uint16_t duty) { cp_duty = duty; }
uint16_t cpGetLastPwmDuty() { return cp_duty; }
bool cpDigitalCommRequested() { return cp_state == CP_B1 || cp_state == CP_B2; }
