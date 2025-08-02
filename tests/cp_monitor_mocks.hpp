#pragma once
#include <cstddef>
#include "esp_adc/adc_continuous.h"

extern "C" {
void adcMockSetDmaData(const adc_digi_output_data_t* samples, size_t count);
void adcMockReset();
}
