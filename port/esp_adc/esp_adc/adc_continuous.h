#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void* adc_continuous_handle_t;

typedef struct {
    uint32_t max_store_buf_size;
    uint32_t conv_frame_size;
} adc_continuous_handle_cfg_t;

typedef struct {
    int atten;
    int channel;
    int unit;
    int bit_width;
} adc_digi_pattern_config_t;

typedef struct {
    uint32_t sample_freq_hz;
    int conv_mode;
    int format;
    adc_digi_pattern_config_t* adc_pattern;
    uint32_t pattern_num;
} adc_continuous_config_t;

typedef struct {
    struct {
        uint16_t data;
        uint16_t channel;
    } type1;
} adc_digi_output_data_t;

#define ADC_CONV_SINGLE_UNIT_1 0
#define ADC_CONV_SINGLE_UNIT_2 1
#define ADC_DIGI_OUTPUT_FORMAT_TYPE1 0

int adc_continuous_new_handle(const adc_continuous_handle_cfg_t*, adc_continuous_handle_t*);
int adc_continuous_config(adc_continuous_handle_t, const adc_continuous_config_t*);
int adc_continuous_start(adc_continuous_handle_t);
int adc_continuous_stop(adc_continuous_handle_t);
int adc_continuous_deinit(adc_continuous_handle_t);
int adc_continuous_read(adc_continuous_handle_t, uint8_t* buf, uint32_t len, uint32_t* outlen, int timeout_ms);

#ifdef __cplusplus
}
#endif
