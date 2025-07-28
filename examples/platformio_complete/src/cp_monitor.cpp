#include "cp_monitor.h"
#include <atomic>
#include <driver/ledc.h>
#include <driver/timer.h>
#include <esp_intr_alloc.h>
#include <esp_adc/adc_oneshot.h>
#if CP_USE_DMA_ADC
#include <esp_adc/adc_continuous.h>
#endif
#include <soc/ledc_struct.h>

static hw_timer_t* adcTimer = nullptr;   // low rate timer
static hw_timer_t* sampleTimer = nullptr; // high precision sample timer
static intr_handle_t ledcIsrHandle = nullptr;
static std::atomic<uint16_t> cp_mv{0};
static std::atomic<uint16_t> cp_duty{0};
static std::atomic<CpSubState> cp_state{CP_A};
static DRAM_ATTR adc_oneshot_unit_handle_t adc_handle = nullptr;
static adc_channel_t cp_channel;
static adc_unit_t   cp_unit;
#if CP_USE_DMA_ADC
static adc_continuous_handle_t dma_handle = nullptr;
static hw_timer_t* dmaTimer = nullptr;
static constexpr uint32_t DMA_SAMPLE_RATE = 25000;
static constexpr uint32_t DMA_SAMPLES = DMA_SAMPLE_RATE / CP_PWM_FREQ_HZ;
static uint16_t dma_ring[DMA_SAMPLES];
static uint32_t dma_idx = 0;
#endif

static inline uint16_t adc_oneshot_read_inline() {
    int raw = 0;
    adc_oneshot_read(adc_handle, cp_channel, &raw);
    return static_cast<uint16_t>((raw * 3300) / 4095);
}

#if CP_USE_DMA_ADC
static inline uint16_t raw_to_mv(uint16_t raw) {
    return static_cast<uint16_t>((raw * 3300) / 4095);
}
#endif

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

#if CP_USE_DMA_ADC
static void IRAM_ATTR dma_timer_isr() {
    uint8_t buf[DMA_SAMPLES * sizeof(adc_digi_output_data_t)];
    uint32_t len = 0;
    esp_err_t rc = adc_continuous_read(dma_handle, buf, sizeof(buf), &len, 0);
    size_t n = len / sizeof(adc_digi_output_data_t);
    for (size_t i = 0; i < n; ++i) {
        auto* d = reinterpret_cast<adc_digi_output_data_t*>(buf + i * sizeof(adc_digi_output_data_t));
        dma_ring[dma_idx++] = d->type1.data;
        if (dma_idx >= DMA_SAMPLES)
            dma_idx = 0;
    }
    uint16_t vmax = 0;
    for (size_t i = 0; i < DMA_SAMPLES; ++i)
        if (dma_ring[i] > vmax)
            vmax = dma_ring[i];
    uint16_t mv = raw_to_mv(vmax);
    cp_mv.store(mv, std::memory_order_relaxed);
    cp_state.store(mv2state(mv), std::memory_order_relaxed);
}
#endif

static void IRAM_ATTR ledc_isr(void*) {
    LEDC.int_clr.lstimer0_ovf = 1;
    if (sampleTimer) {
        timerWrite(sampleTimer, 0);
        timerAlarmEnable(sampleTimer);
    }
}

void cpMonitorInit() {
    adc_unit_t unit;
    adc_oneshot_io_to_channel(CP_READ_ADC_PIN, &unit, &cp_channel);
    cp_unit = unit;
    adc_oneshot_unit_init_cfg_t unit_cfg{.unit_id = unit, .ulp_mode = ADC_ULP_MODE_DISABLE};
    adc_oneshot_new_unit(&unit_cfg, &adc_handle);
    adc_oneshot_chan_cfg_t chan_cfg{.atten = ADC_ATTEN_DB_11, .bitwidth = ADC_BITWIDTH_12};
    adc_oneshot_config_channel(adc_handle, cp_channel, &chan_cfg);
    int raw = 0;
    adc_oneshot_read(adc_handle, cp_channel, &raw);
    uint16_t mv = static_cast<uint16_t>((raw * 3300) / 4095);
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

#if CP_USE_DMA_ADC
void cpDmaStart() {
    if (dma_handle)
        return;
    adc_continuous_handle_cfg_t cfg{.max_store_buf_size = 1024,
                                   .conv_frame_size = DMA_SAMPLES * sizeof(adc_digi_output_data_t)};
    adc_continuous_new_handle(&cfg, &dma_handle);

    adc_digi_pattern_config_t pattern{};
    pattern.atten = ADC_ATTEN_DB_11;
    pattern.channel = cp_channel;
    pattern.unit = cp_unit;
    pattern.bit_width = ADC_BITWIDTH_12;

    adc_continuous_config_t dig_cfg{};
    dig_cfg.sample_freq_hz = DMA_SAMPLE_RATE;
    dig_cfg.conv_mode = (cp_unit == ADC_UNIT_1) ? ADC_CONV_SINGLE_UNIT_1 : ADC_CONV_SINGLE_UNIT_2;
    dig_cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1;
    dig_cfg.adc_pattern = &pattern;
    dig_cfg.pattern_num = 1;

    adc_continuous_config(dma_handle, &dig_cfg);
    adc_continuous_start(dma_handle);

    if (!dmaTimer)
        dmaTimer = timerBegin(4, 80, true);
    timerAttachInterrupt(dmaTimer, &dma_timer_isr, true);
    timerAlarmWrite(dmaTimer, 1000, true);
    timerAlarmEnable(dmaTimer);
}

void cpDmaStop() {
    if (dmaTimer)
        timerAlarmDisable(dmaTimer);
    if (dma_handle) {
        adc_continuous_stop(dma_handle);
        adc_continuous_deinit(dma_handle);
        dma_handle = nullptr;
    }
}
#endif

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

// Provide the runtime CP state for SLAC match logging
#include <slac/match_log.hpp>

namespace slac {
char slac_get_cp_state() { return cpGetStateLetter(); }
}
