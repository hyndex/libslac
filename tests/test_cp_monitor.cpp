#include <gtest/gtest.h>
#define ARDUINO
#include "Arduino.h"

#include "cp_monitor.h"
#include "esp_adc/adc_continuous.h"
#include "cp_monitor_mocks.hpp"

#include "cp_config.h"

static constexpr uint8_t CP_CH = CP_READ_ADC_PIN - 1;
static constexpr uint8_t VOUT_CH = VOUT_MON_ADC_PIN - 1;

static void reset_mocks() {
    adcMockReset();
}

TEST(CpMonitor, DmaPeakDetection) {
    reset_mocks();
    // Provide initial frame for cpMonitorInit()
    adc_digi_output_data_t initbuf[1] = {};
    initbuf[0].type1.data = 0;
    initbuf[0].type1.channel = CP_CH;
    adcMockSetDmaData(initbuf, 1);

    cpMonitorInit();

    adc_digi_output_data_t buf[5] = {};
    buf[0].type1.data = 50;   buf[0].type1.channel = CP_CH;
    buf[1].type1.data = 1000; buf[1].type1.channel = CP_CH;
    buf[2].type1.data = 3000; buf[2].type1.channel = CP_CH; // max
    buf[3].type1.data = 1500; buf[3].type1.channel = CP_CH;
    buf[4].type1.data = 20;   buf[4].type1.channel = CP_CH;

    adcMockSetDmaData(buf, 5);

    cpMonitorTestProcess();

    uint16_t expected_mv = static_cast<uint16_t>((3000u * 3300) / 4095);
    EXPECT_EQ(cpGetVoltageMv(), expected_mv);
}

TEST(CpMonitor, VoutMeasurement) {
    reset_mocks();
    adc_digi_output_data_t initbuf[2] = {};
    initbuf[0].type1.channel = CP_CH;
    initbuf[0].type1.data = 0;
    initbuf[1].type1.channel = VOUT_CH;
    initbuf[1].type1.data = 0;
    adcMockSetDmaData(initbuf, 2);

    cpMonitorInit();

    adc_digi_output_data_t buf[4] = {};
    buf[0].type1.channel = VOUT_CH; buf[0].type1.data = 1000;
    buf[1].type1.channel = VOUT_CH; buf[1].type1.data = 3000;
    buf[2].type1.channel = CP_CH;   buf[2].type1.data = 0;
    buf[3].type1.channel = CP_CH;   buf[3].type1.data = 0;
    adcMockSetDmaData(buf, 4);

    cpMonitorTestProcess();

    uint16_t expected_mv = static_cast<uint16_t>(((1000u + 3000u) / 2u * 3300) / 4095);
    EXPECT_EQ(voutGetVoltageMv(), expected_mv);
}
