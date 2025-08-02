#include <cstring>
#include "esp_adc/adc_continuous.h"
#include "cp_monitor_mocks.hpp"

static const adc_digi_output_data_t* g_dma_data = nullptr;
static size_t g_dma_count = 0;

extern "C" {
int adc_continuous_new_handle(const adc_continuous_handle_cfg_t*, adc_continuous_handle_t* out) { if (out) *out = (void*)1; return 0; }
int adc_continuous_config(adc_continuous_handle_t, const adc_continuous_config_t*) { return 0; }
int adc_continuous_start(adc_continuous_handle_t) { return 0; }
int adc_continuous_stop(adc_continuous_handle_t) { return 0; }
int adc_continuous_deinit(adc_continuous_handle_t) { return 0; }
int adc_continuous_read(adc_continuous_handle_t, uint8_t* buf, uint32_t len, uint32_t* outlen, int) {
    if (!g_dma_data) {
        if (outlen) *outlen = 0;
        return 0;
    }
    uint32_t byte_len = static_cast<uint32_t>(g_dma_count * sizeof(adc_digi_output_data_t));
    if (byte_len > len) byte_len = len;
    std::memcpy(buf, g_dma_data, byte_len);
    if (outlen) *outlen = byte_len;
    g_dma_data = nullptr;
    g_dma_count = 0;
    return 0;
}

void adcMockSetDmaData(const adc_digi_output_data_t* samples, size_t count) {
    g_dma_data = samples;
    g_dma_count = count;
}

void adcMockReset() {
    g_dma_data = nullptr;
    g_dma_count = 0;
}
}
