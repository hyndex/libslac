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
