#include <gtest/gtest.h>
#define ARDUINO
#include "Arduino.h"

#include "cp_monitor.h"
#include "esp_adc/adc_continuous.h"

extern "C" {
extern uint64_t g_last_alarm_value;
extern void (*g_last_isr)(void);
extern uint8_t* g_dma_read_data;
extern uint32_t g_dma_read_len;
}

static void reset_mocks() {
    g_last_alarm_value = 0;
    g_last_isr = nullptr;
    g_dma_read_data = nullptr;
    g_dma_read_len = 0;
}

TEST(CpMonitor, FastSampleSchedulesOffset) {
    reset_mocks();
    cpSetLastPwmDuty(CP_PWM_DUTY_5PCT);
    cpFastSampleStart();
    uint32_t period = 1000000 / CP_PWM_FREQ_HZ;
    uint32_t expected = ((period * CP_PWM_DUTY_5PCT) >> CP_PWM_RES_BITS) / 2;
    EXPECT_EQ(g_last_alarm_value, expected);
}

TEST(CpMonitor, DmaPeakDetection) {
    reset_mocks();
    cpMonitorInit();
    cpDmaStart();
    ASSERT_NE(g_last_isr, nullptr);

    adc_digi_output_data_t buf[5] = {};
    buf[0].type1.data = 50;
    buf[1].type1.data = 1000;
    buf[2].type1.data = 3000; // max
    buf[3].type1.data = 1500;
    buf[4].type1.data = 20;

    g_dma_read_data = reinterpret_cast<uint8_t*>(buf);
    g_dma_read_len = sizeof(buf);

    g_last_isr();

    uint16_t expected_mv = static_cast<uint16_t>((3000u * 3300) / 4095);
    EXPECT_EQ(cpGetVoltageMv(), expected_mv);
}

static void push_sample_and_process() {
    for (int i = 0; i < 3; ++i)
        g_last_isr();
    cpLowRateStart();
}

TEST(CpMonitor, CpStateRequiresTripleMatch) {
    reset_mocks();
    g_mock_adc_mv = 3000; // CP_A
    cpMonitorInit();
    cpLowRateStart(); // set ISR and process initial state

    g_mock_adc_mv = CP_THR_9V_MV + 50; // -> B1
    CpSubState initial = cpGetSubState();
    push_sample_and_process();
    EXPECT_EQ(cpGetSubState(), initial);
    push_sample_and_process();
    EXPECT_EQ(cpGetSubState(), initial);
    push_sample_and_process();
    EXPECT_NE(cpGetSubState(), initial);
}

TEST(CpMonitor, NegativeVoltageHysteresis) {
    reset_mocks();
    g_mock_adc_mv = CP_THR_1V_MV - 10; // start in F
    cpMonitorInit();
    cpLowRateStart();

    g_mock_adc_mv = CP_THR_NEG12_LOW - 5; // below enter threshold
    push_sample_and_process();
    push_sample_and_process();
    push_sample_and_process();
    EXPECT_EQ(cpGetSubState(), CP_E);

    g_mock_adc_mv = CP_THR_NEG12 + 10; // between thresholds
    push_sample_and_process();
    push_sample_and_process();
    push_sample_and_process();
    EXPECT_EQ(cpGetSubState(), CP_E);

    g_mock_adc_mv = CP_THR_NEG12_HIGH + 5; // exit negative
    push_sample_and_process();
    push_sample_and_process();
    push_sample_and_process();
    EXPECT_EQ(cpGetSubState(), CP_F);
}
