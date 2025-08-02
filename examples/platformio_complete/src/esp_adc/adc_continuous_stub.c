#include "esp_adc/adc_continuous.h"
int adc_continuous_new_handle(const adc_continuous_handle_cfg_t* cfg, adc_continuous_handle_t* out) { if (out) *out = (void*)1; return 0; }
int adc_continuous_config(adc_continuous_handle_t handle, const adc_continuous_config_t* cfg) { return 0; }
int adc_continuous_start(adc_continuous_handle_t handle) { return 0; }
int adc_continuous_stop(adc_continuous_handle_t handle) { return 0; }
int adc_continuous_deinit(adc_continuous_handle_t handle) { return 0; }
int adc_continuous_read(adc_continuous_handle_t handle, uint8_t* buf, uint32_t len, uint32_t* outlen, int timeout_us) {
    if (outlen) *outlen = 0;
    return 0;
}
