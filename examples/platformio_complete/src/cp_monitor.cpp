#include "cp_monitor.h"
#include <atomic>
#include <algorithm>
#include <esp_adc/adc_continuous.h>
#include <freertos/FreeRTOS.h>
#ifndef LIBSLAC_TESTING
#include <freertos/task.h>
#endif
#include <esp_timer.h>
#ifdef ESP_PLATFORM
#include <port/esp32s3/port_config.hpp>
#else
#include <port/port_common.hpp>
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
#ifndef ESP_ERR_TIMEOUT
#define ESP_ERR_TIMEOUT -1
#endif

static std::atomic<uint16_t> cp_mv{0};
static std::atomic<uint16_t> vout_mv{0};
static std::atomic<uint16_t> cp_duty{0};
static std::atomic<CpSubState> cp_state{CP_A};
static std::atomic<uint32_t> cp_ts{0};
static uint16_t last_raw = 0;
static uint8_t stable_cnt = 0;
static adc_continuous_handle_t adc_handle = nullptr;
#ifndef LIBSLAC_TESTING
static TaskHandle_t cp_task = nullptr;
#endif

static constexpr uint32_t DMA_SAMPLE_RATE = 40000; // 40 kS/s total
static constexpr uint32_t DMA_SAMPLES = DMA_SAMPLE_RATE / CP_PWM_FREQ_HZ;
static constexpr size_t DMA_BUF_BYTES = DMA_SAMPLES * sizeof(adc_digi_output_data_t);

static constexpr uint8_t CP_ADC_CHANNEL   = static_cast<uint8_t>(CP_READ_ADC_PIN - 1);
static constexpr uint8_t VOUT_ADC_CHANNEL = static_cast<uint8_t>(VOUT_MON_ADC_PIN - 1);

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
    if (mv > CP_THR_9V_MV) {
#ifdef CP_SUPPORT_B3
        return (duty == 0) ? CP_B1 : CP_B3;
#else
        return CP_B1;
#endif
    }
    if (mv > CP_THR_6V_MV) {
        if (pct >= 3 && pct <= 7) return CP_B2;
        return CP_C;
    }
    if (mv > CP_THR_3V_MV)
        return CP_D;
    return CP_E;
}

static void restart_adc() {
    if (!adc_handle)
        return;
    adc_continuous_stop(adc_handle);
    adc_continuous_start(adc_handle);
}

static void process_samples() {
    if (!adc_handle)
        return;
    uint8_t buf[DMA_BUF_BYTES];
    uint32_t len = 0;
    int rc = adc_continuous_read(adc_handle, buf, sizeof(buf), &len, 1000);
    if (rc == ESP_ERR_TIMEOUT) {
        restart_adc();
        return;
    }
    if (rc != 0)
        return;
    size_t n = len / sizeof(adc_digi_output_data_t);
    uint16_t cp_vmax = 0;
    uint32_t vout_sum = 0;
    uint16_t vout_cnt = 0;
    for (size_t i = 0; i < n; ++i) {
        auto* d = reinterpret_cast<adc_digi_output_data_t*>(buf + i * sizeof(adc_digi_output_data_t));
        uint16_t raw = d->type1.data;
        if (d->type1.channel == CP_ADC_CHANNEL) {
            if (raw > cp_vmax)
                cp_vmax = raw;
        } else if (d->type1.channel == VOUT_ADC_CHANNEL) {
            vout_sum += raw;
            ++vout_cnt;
        }
    }
    uint16_t mv = raw_to_mv(cp_vmax);
    cp_mv.store(mv, std::memory_order_relaxed);
    CpSubState ns = mv2state(mv);
    if (vout_cnt > 0) {
        uint16_t vraw = static_cast<uint16_t>(vout_sum / vout_cnt);
        vout_mv.store(raw_to_mv(vraw), std::memory_order_relaxed);
    }

    if (cp_vmax == last_raw) {
        if (stable_cnt < 0xff)
            ++stable_cnt;
    } else {
        stable_cnt = 1;
        last_raw = cp_vmax;
    }

    CpSubState cur = cp_state.load(std::memory_order_relaxed);
    if (stable_cnt >= 3 && ns != cur) {
        cp_state.store(ns, std::memory_order_relaxed);
        cp_ts.store(static_cast<uint32_t>(esp_timer_get_time() / 1000),
                    std::memory_order_relaxed);
    }
}

#ifndef LIBSLAC_TESTING
static void cp_dma_task(void*) {
    while (true) {
        process_samples();
    }
}
#endif

void cpMonitorInit() {
    stable_cnt = 0;
    last_raw = 0;

    adc_continuous_handle_cfg_t cfg{};
    cfg.max_store_buf_size = static_cast<uint32_t>(DMA_BUF_BYTES * 2);
    cfg.conv_frame_size = static_cast<uint32_t>(DMA_BUF_BYTES);
    adc_continuous_new_handle(&cfg, &adc_handle);

    adc_digi_pattern_config_t patterns[2] = {};
    for (auto &p : patterns) {
        p.atten = ADC_ATTEN_DB_11;
        p.unit = ADC_UNIT_1;
        p.bit_width = ADC_BITWIDTH_12;
    }
    patterns[0].channel = CP_ADC_CHANNEL;
    patterns[1].channel = VOUT_ADC_CHANNEL;

    adc_continuous_config_t dig_cfg{};
    dig_cfg.sample_freq_hz = DMA_SAMPLE_RATE;
    dig_cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    dig_cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1;
    dig_cfg.pattern_num = 2;
    dig_cfg.adc_pattern = patterns;
    adc_continuous_config(adc_handle, &dig_cfg);
    adc_continuous_start(adc_handle);

    // Wait for at least one DMA frame to initialize CP voltage/state
    while (true) {
        uint8_t buf[DMA_BUF_BYTES];
        uint32_t len = 0;
        int rc = adc_continuous_read(adc_handle, buf, sizeof(buf), &len, 1000);
        if (rc == ESP_ERR_TIMEOUT) {
            restart_adc();
            continue;
        }
        if (rc != 0 || len == 0)
            continue;
        size_t n = len / sizeof(adc_digi_output_data_t);
        uint16_t cp_vmax = 0;
        uint32_t vout_sum = 0;
        uint16_t vout_cnt = 0;
        for (size_t i = 0; i < n; ++i) {
            auto* d = reinterpret_cast<adc_digi_output_data_t*>(buf + i * sizeof(adc_digi_output_data_t));
            uint16_t raw = d->type1.data;
            if (d->type1.channel == CP_ADC_CHANNEL) {
                if (raw > cp_vmax)
                    cp_vmax = raw;
            } else if (d->type1.channel == VOUT_ADC_CHANNEL) {
                vout_sum += raw;
                ++vout_cnt;
            }
        }
        uint16_t mv = raw_to_mv(cp_vmax);
        cp_mv.store(mv, std::memory_order_relaxed);
        CpSubState ns = mv2state(mv);
        cp_state.store(ns, std::memory_order_relaxed);
        cp_ts.store(static_cast<uint32_t>(esp_timer_get_time() / 1000),
                    std::memory_order_relaxed);
        last_raw = cp_vmax;
        stable_cnt = 1;
        if (vout_cnt > 0) {
            uint16_t vraw = static_cast<uint16_t>(vout_sum / vout_cnt);
            vout_mv.store(raw_to_mv(vraw), std::memory_order_relaxed);
        }
        break;
    }

#ifndef LIBSLAC_TESTING
    xTaskCreatePinnedToCore(cp_dma_task, "cp_dma", 2048, nullptr, 5, &cp_task, 1);
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
uint16_t voutGetVoltageMv() { return vout_mv.load(std::memory_order_relaxed); }
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

