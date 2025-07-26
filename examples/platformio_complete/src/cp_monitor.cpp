#include "cp_monitor.h"
#include <atomic>
#include <driver/ledc.h>
#include <driver/timer.h>
#include <esp_intr_alloc.h>
#include <soc/ledc_struct.h>

static hw_timer_t* adcTimer = nullptr;   // low rate timer
static hw_timer_t* sampleTimer = nullptr; // high precision sample timer
static intr_handle_t ledcIsrHandle = nullptr;
static std::atomic<uint16_t> cp_mv{0};
static std::atomic<uint16_t> cp_duty{0};
static std::atomic<CpSubState> cp_state{CP_A};

static inline uint16_t adc_oneshot_read_inline() {
    return analogReadMilliVolts(CP_READ_ADC_PIN);
}

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
    uint16_t duty = cp_duty.load(std::memory_order_relaxed);
    uint16_t pct  = (duty * 100) >> CP_PWM_RES_BITS;
    if (mv < CP_THR_NEG12)
        return CP_E;
    if (mv < CP_THR_1V_MV)
        return CP_F;
    if (mv > CP_THR_12V_MV)
        return CP_A;
    if (mv > CP_THR_9V_MV)
        return (duty == 0) ? CP_B1 : CP_B3;
    if (mv > CP_THR_6V_MV) {
        if (pct >= 3 && pct <= 7) return CP_B2;
        return CP_C;
    }
    if (mv > CP_THR_3V_MV)
        return CP_D;
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
    cp_mv.store(vmax, std::memory_order_relaxed);
    cp_state.store(mv2state(vmax), std::memory_order_relaxed);
}

static void IRAM_ATTR sample_isr() {
    timerAlarmDisable(sampleTimer);            // one-shot
    uint16_t v = adc_oneshot_read_inline();    // IRAM-safe helper
    cp_mv.store(v, std::memory_order_relaxed);
    cp_state.store(mv2state(v), std::memory_order_relaxed);
}

static void IRAM_ATTR ledc_isr(void*) {
    LEDC.int_clr.lstimer0_ovf = 1;
    if (sampleTimer) {
        timerWrite(sampleTimer, 0);
        timerAlarmEnable(sampleTimer);
    }
}

void cpMonitorInit() {
    analogReadResolution(12);
    analogSetPinAttenuation(CP_READ_ADC_PIN, ADC_11db);
    uint16_t mv = analogReadMilliVolts(CP_READ_ADC_PIN);
    cp_mv.store(mv, std::memory_order_relaxed);
    cp_state.store(mv2state(mv), std::memory_order_relaxed);
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

void cpFastSampleStart() {
    if (!sampleTimer)
        sampleTimer = timerBegin(1, 80, true);
    timerAttachInterrupt(sampleTimer, &sample_isr, true);
    timerAlarmWrite(sampleTimer, CP_SAMPLE_OFFSET_US, false);
    timerAlarmDisable(sampleTimer);

    if (!ledcIsrHandle) {
        esp_err_t rc = ledc_isr_register(ledc_isr, nullptr, ESP_INTR_FLAG_IRAM,
                                         &ledcIsrHandle);
        if (rc != ESP_OK) {
            ledcIsrHandle = nullptr;
        } else {
            LEDC.int_clr.val = LEDC.int_st.val;
        }
    }
}

void cpFastSampleStop() {
    if (ledcIsrHandle) {
        esp_intr_free(ledcIsrHandle);
        ledcIsrHandle = nullptr;
    }
    if (sampleTimer)
        timerAlarmDisable(sampleTimer);
}

uint16_t cpGetVoltageMv() { return cp_mv.load(std::memory_order_relaxed); }
CpSubState cpGetSubState() { return cp_state.load(std::memory_order_relaxed); }
char cpGetStateLetter() { return toLetter(cp_state.load(std::memory_order_relaxed)); }

void cpSetLastPwmDuty(uint16_t duty) {
    cp_duty.store(duty, std::memory_order_relaxed);
}
uint16_t cpGetLastPwmDuty() {
    return cp_duty.load(std::memory_order_relaxed);
}
bool cpDigitalCommRequested() {
    CpSubState s = cp_state.load(std::memory_order_relaxed);
    return s == CP_B2 || s == CP_B3;
}
