#include <cstring>
#include "esp_adc/adc_continuous.h"
#include "arduino_stubs.hpp"

extern "C" {
uint8_t* g_dma_read_data = nullptr;
uint32_t g_dma_read_len = 0;

int adc_continuous_new_handle(const adc_continuous_handle_cfg_t*, adc_continuous_handle_t* out) { if (out) *out = (void*)1; return 0; }
int adc_continuous_config(adc_continuous_handle_t, const adc_continuous_config_t*) { return 0; }
int adc_continuous_start(adc_continuous_handle_t) { return 0; }
int adc_continuous_stop(adc_continuous_handle_t) { return 0; }
int adc_continuous_deinit(adc_continuous_handle_t) { return 0; }
int adc_continuous_read(adc_continuous_handle_t, uint8_t* buf, uint32_t len, uint32_t* outlen, int) {
    if (!g_dma_read_data) { if (outlen) *outlen = 0; return 0; }
    uint32_t copy = g_dma_read_len;
    if (copy > len) copy = len;
    memcpy(buf, g_dma_read_data, copy);
    if (outlen) *outlen = copy;
    g_dma_read_data = nullptr;
    g_dma_read_len = 0;
    return 0;
}
}

int g_mock_adc_mv = 0;
