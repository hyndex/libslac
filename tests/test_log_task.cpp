#include <gtest/gtest.h>
#include <atomic>
#include <slac/slac_states.hpp>
#define LIBSLAC_TESTING
#include "esp_log.h"
#undef ESP_LOGI
#define ESP_LOGI(tag, fmt, ...) printf(fmt "\n", ##__VA_ARGS__)
#include "../examples/platformio_complete/src/cp_config.h"

#define cpGetVoltageMv cpGetVoltageMv_stub
#define cpGetLastPwmDuty cpGetLastPwmDuty_stub
#define voutGetVoltageMv voutGetVoltageMv_stub
#define voutGetVoltageRaw voutGetVoltageRaw_stub
#define cpGetStateLetter cpGetStateLetter_stub
#define evseStageName evseStageName_stub
#define evseGetStage evseGetStage_stub

#define g_slac_state g_slac_state_main
#define qca7000ProcessSlice qca7000ProcessSlice_stub
#define qca7000ReadInternalReg qca7000ReadInternalReg_stub

void qca7000ProcessSlice_stub(uint32_t) {}
uint16_t qca7000ReadInternalReg_stub(uint16_t) { return 0; }

uint32_t cpGetVoltageMv_stub() { return 0; }
uint16_t cpGetLastPwmDuty_stub() { return ((1u << CP_PWM_RES_BITS) - 1u) / 2u; }
uint16_t voutGetVoltageMv_stub() { return 5000; }
uint16_t voutGetVoltageRaw_stub() { return 2048; }
char cpGetStateLetter_stub() { return 'B'; }
const char* evseStageName_stub(int) { return "TEST"; }
int evseGetStage_stub() { return 0; }

#include "../examples/platformio_complete/src/main.cpp"
#undef g_slac_state
extern std::atomic<SlacState> g_slac_state_main;

TEST(LogTask, PrintsDutyAndVout) {
    g_slac_state_main.store(SlacState::Matched, std::memory_order_relaxed);
    testing::internal::CaptureStdout();
    logStatus();
    std::string out = testing::internal::GetCapturedStdout();
    EXPECT_NE(out.find("Duty=50.0%"), std::string::npos);
    EXPECT_NE(out.find("Vout=5.000 V"), std::string::npos);
}
