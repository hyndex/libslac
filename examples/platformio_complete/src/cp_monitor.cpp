#include "cp_monitor.h"
#include <atomic>
#include <algorithm>
#include <esp_adc/adc_continuous.h>
#include <freertos/FreeRTOS.h>
#ifndef LIBSLAC_TESTING
#include <freertos/task.h>
#endif

#ifndef ADC_ATTEN_DB_11
#define ADC_ATTEN_DB_11 0
#endif
#ifndef ADC_UNIT_1
#define ADC_UNIT_1 0
#endif
#ifndef ADC_BITWIDTH_12
#define ADC_BITWIDTH_12 12
#endif

static std::atomic<uint16_t> cp_mv{0};
static std::atomic<uint16_t> cp_duty{0};
static std::atomic<CpSubState> cp_state{CP_A};
static CpSubState cp_prev1 = CP_A;
static CpSubState cp_prev2 = CP_A;
static adc_continuous_handle_t adc_handle = nullptr;
#ifndef LIBSLAC_TESTING
static TaskHandle_t cp_task = nullptr;
#endif

static constexpr uint32_t DMA_SAMPLE_RATE = 50000; // 50 kS/s
static constexpr uint32_t DMA_SAMPLES = DMA_SAMPLE_RATE / CP_PWM_FREQ_HZ;
static constexpr size_t DMA_BUF_BYTES = DMA_SAMPLES * sizeof(adc_digi_output_data_t);

static inline uint16_t raw_to_mv(uint16_t raw) {
    return static_cast<uint16_t>((raw * 3300u) / 4095u);
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
    CpSubState prev = cp_state.load(std::memory_order_relaxed);
    if (prev == CP_E) {
        if (mv < CP_THR_NEG12_HIGH)
            return CP_E;
    } else {
        if (mv < CP_THR_NEG12_LOW)
            return CP_E;
    }
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

static void process_samples() {
    if (!adc_handle)
        return;
    uint8_t buf[DMA_BUF_BYTES];
    uint32_t len = 0;
    int rc = adc_continuous_read(adc_handle, buf, sizeof(buf), &len, 1000);
    if (rc != 0)
        return;
    size_t n = len / sizeof(adc_digi_output_data_t);
    uint16_t vmax = 0;
    for (size_t i = 0; i < n; ++i) {
        auto* d = reinterpret_cast<adc_digi_output_data_t*>(buf + i * sizeof(adc_digi_output_data_t));
        if (d->type1.data > vmax)
            vmax = d->type1.data;
    }
    uint16_t mv = raw_to_mv(vmax);
    cp_mv.store(mv, std::memory_order_relaxed);
    CpSubState ns = mv2state(mv);
    if (ns == cp_prev1 && cp_prev1 == cp_prev2)
        cp_state.store(ns, std::memory_order_relaxed);
    cp_prev2 = cp_prev1;
    cp_prev1 = ns;
}

#ifndef LIBSLAC_TESTING
static void cp_dma_task(void*) {
    while (true) {
        process_samples();
    }
}
#endif

void cpMonitorInit() {
    uint16_t mv = analogReadMilliVolts(CP_READ_ADC_PIN);
    cp_mv.store(mv, std::memory_order_relaxed);
    CpSubState ns = mv2state(mv);
    cp_state.store(ns, std::memory_order_relaxed);
    cp_prev1 = cp_prev2 = ns;

    adc_continuous_handle_cfg_t cfg{};
    cfg.max_store_buf_size = static_cast<uint32_t>(DMA_BUF_BYTES * 2);
    cfg.conv_frame_size = static_cast<uint32_t>(DMA_BUF_BYTES);
    adc_continuous_new_handle(&cfg, &adc_handle);

    adc_digi_pattern_config_t pattern{};
    pattern.atten = ADC_ATTEN_DB_11;
    pattern.channel = 0;
    pattern.unit = ADC_UNIT_1;
    pattern.bit_width = ADC_BITWIDTH_12;

    adc_continuous_config_t dig_cfg{};
    dig_cfg.sample_freq_hz = DMA_SAMPLE_RATE;
    dig_cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    dig_cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1;
    dig_cfg.pattern_num = 1;
    dig_cfg.adc_pattern = &pattern;
    adc_continuous_config(adc_handle, &dig_cfg);
    adc_continuous_start(adc_handle);

#ifndef LIBSLAC_TESTING
    xTaskCreatePinnedToCore(cp_dma_task, "cp_dma", 2048, nullptr, 5, &cp_task, 1);
#else
    process_samples();
#endif
}

void cpMonitorStop() {
    if (adc_handle) {
#ifndef LIBSLAC_TESTING
        if (cp_task) {
            vTaskDelete(cp_task);
            cp_task = nullptr;
        }
#endif
        adc_continuous_stop(adc_handle);
        adc_continuous_deinit(adc_handle);
        adc_handle = nullptr;
    }
}

uint16_t cpGetVoltageMv() { return cp_mv.load(std::memory_order_relaxed); }
CpSubState cpGetSubState() { return cp_state.load(std::memory_order_relaxed); }
char cpGetStateLetter() { return toLetter(cp_state.load(std::memory_order_relaxed)); }
void cpSetLastPwmDuty(uint16_t duty) { cp_duty.store(duty, std::memory_order_relaxed); }
uint16_t cpGetLastPwmDuty() { return cp_duty.load(std::memory_order_relaxed); }
bool cpDigitalCommRequested() {
    CpSubState s = cp_state.load(std::memory_order_relaxed);
    return s == CP_B2 || s == CP_B3;
}

#ifdef LIBSLAC_TESTING
void cpMonitorTestProcess() { process_samples(); }
#endif

