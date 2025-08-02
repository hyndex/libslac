#include <gtest/gtest.h>
#define ARDUINO
#include "Arduino.h"

#include "cp_monitor.h"
#include "esp_adc/adc_continuous.h"

extern "C" {
extern uint8_t* g_dma_read_data;
extern uint32_t g_dma_read_len;
}
extern int g_mock_adc_mv;

static void reset_mocks() {
    g_dma_read_data = nullptr;
    g_dma_read_len = 0;
    g_mock_adc_mv = 0;
}

TEST(CpMonitor, DmaPeakDetection) {
    reset_mocks();
    cpMonitorInit();
    adc_digi_output_data_t buf[5] = {};
    for (int i = 0; i < 5; ++i)
        buf[i].type1.channel = CP_READ_ADC_PIN - 1;
    buf[0].type1.data = 50;
    buf[1].type1.data = 1000;
    buf[2].type1.data = 3000; // max
    buf[3].type1.data = 1500;
    buf[4].type1.data = 20;

    g_dma_read_data = reinterpret_cast<uint8_t*>(buf);
    g_dma_read_len = sizeof(buf);

    cpMonitorTestProcess();

    uint16_t expected_mv = static_cast<uint16_t>((3000u * 3300) / 4095);
    EXPECT_EQ(cpGetVoltageMv(), expected_mv);
}
