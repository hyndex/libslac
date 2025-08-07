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

static void init_monitor() {
    adc_digi_output_data_t initbuf[1] = {};
    initbuf[0].type1.channel = CP_CH;
    initbuf[0].type1.data = 0;
    adcMockSetDmaData(initbuf, 1);
    cpMonitorInit();
}

static void feed_and_process(const adc_digi_output_data_t* buf, size_t count, int times = 3) {
    for (int i = 0; i < times; ++i) {
        adcMockSetDmaData(buf, count);
        cpMonitorTestProcess();
    }
}

TEST(CpMonitor, DutyClassificationB1) {
    reset_mocks();
    init_monitor();

    adc_digi_output_data_t buf[20] = {};
    for (int i = 0; i < 20; ++i) {
        buf[i].type1.channel = CP_CH;
        buf[i].type1.data = 2800; // >9V but <12V
    }
    feed_and_process(buf, 20);

    EXPECT_EQ(cpGetMeasuredDuty(), 0);
    EXPECT_EQ(cpGetSubState(), CP_B1);

    cpMonitorStop();
}

TEST(CpMonitor, DutyClassificationB3) {
    reset_mocks();
    init_monitor();

    adc_digi_output_data_t buf[20] = {};
    for (int i = 0; i < 10; ++i) {
        buf[i].type1.channel = CP_CH;
        buf[i].type1.data = 2800;
    }
    for (int i = 10; i < 20; ++i) {
        buf[i].type1.channel = CP_CH;
        buf[i].type1.data = 0;
    }
    feed_and_process(buf, 20);

    uint16_t expected = static_cast<uint16_t>((10u << CP_PWM_RES_BITS) / 20u);
    EXPECT_EQ(cpGetMeasuredDuty(), expected);
    EXPECT_EQ(cpGetSubState(), CP_B3);

    cpMonitorStop();
}

TEST(CpMonitor, DutyClassificationB2) {
    reset_mocks();
    init_monitor();

    adc_digi_output_data_t buf[20] = {};
    buf[0].type1.channel = CP_CH;
    buf[0].type1.data = 2233; // peak ~8V
    for (int i = 1; i < 20; ++i) {
        buf[i].type1.channel = CP_CH;
        buf[i].type1.data = (i == 1) ? 0 : 2233; // one low sample
    }
    feed_and_process(buf, 20);

    uint16_t expected = static_cast<uint16_t>((1u << CP_PWM_RES_BITS) / 20u);
    EXPECT_EQ(cpGetMeasuredDuty(), expected);
    EXPECT_EQ(cpGetSubState(), CP_B2);

    cpMonitorStop();
}

TEST(CpMonitor, DutyClassificationC) {
    reset_mocks();
    init_monitor();

    adc_digi_output_data_t buf[20] = {};
    for (int i = 0; i < 10; ++i) {
        buf[i].type1.channel = CP_CH;
        buf[i].type1.data = 2233; // ~8V
    }
    for (int i = 10; i < 20; ++i) {
        buf[i].type1.channel = CP_CH;
        buf[i].type1.data = 0;
    }
    feed_and_process(buf, 20);

    uint16_t expected = static_cast<uint16_t>((10u << CP_PWM_RES_BITS) / 20u);
    EXPECT_EQ(cpGetMeasuredDuty(), expected);
    EXPECT_EQ(cpGetSubState(), CP_C);

    cpMonitorStop();
}
