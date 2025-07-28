#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void* adc_oneshot_unit_handle_t;
typedef int adc_unit_t;
typedef int adc_channel_t;

typedef struct { int unit_id; int ulp_mode; } adc_oneshot_unit_init_cfg_t;
typedef struct { int atten; int bitwidth; } adc_oneshot_chan_cfg_t;

#define ADC_UNIT_1 1
#define ADC_ATTEN_DB_11 0
#define ADC_BITWIDTH_12 12
#define ADC_ULP_MODE_DISABLE 0

int adc_oneshot_io_to_channel(int pin, adc_unit_t* unit, adc_channel_t* chan);
int adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t* cfg, adc_oneshot_unit_handle_t* ret);
int adc_oneshot_config_channel(adc_oneshot_unit_handle_t, adc_channel_t, const adc_oneshot_chan_cfg_t*);
int adc_oneshot_read(adc_oneshot_unit_handle_t, adc_channel_t, int* out);

#ifdef __cplusplus
}
#endif
