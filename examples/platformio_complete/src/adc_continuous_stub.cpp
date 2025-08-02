#include <esp_adc/adc_continuous.h>
#include <stdint.h>

int adc_continuous_new_handle(const adc_continuous_handle_cfg_t*, adc_continuous_handle_t* out) {
    if (out) *out = (adc_continuous_handle_t)1;
    return 0;
}

int adc_continuous_config(adc_continuous_handle_t, const adc_continuous_config_t*) { return 0; }
int adc_continuous_start(adc_continuous_handle_t) { return 0; }
int adc_continuous_stop(adc_continuous_handle_t) { return 0; }
int adc_continuous_deinit(adc_continuous_handle_t) { return 0; }

int adc_continuous_read(adc_continuous_handle_t, uint8_t* buf, uint32_t len, uint32_t* outlen, int) {
    if (outlen) *outlen = 0;
    return 0;
}
