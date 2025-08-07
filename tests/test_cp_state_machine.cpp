#include <gtest/gtest.h>
#define ARDUINO
#include "arduino_stubs.hpp"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include <atomic>

// Redirect functions to stubs to avoid conflicts with other tests
#define cpPwmStart cpPwmStartStub
#define cpPwmStop cpPwmStopStub
#define cpPwmSetDuty cpPwmSetDutyStub
#define cpPwmIsRunning cpPwmIsRunningStub
#define cpGetSubState cpGetSubStateStub
#define voutGetVoltageMv voutGetVoltageMvStub
#define qca7000Sleep qca7000SleepStub
#define qca7000Wake qca7000WakeStub
#define qca7000startSlac qca7000startSlacStub
#define qca7000SetMac qca7000SetMacStub
#define qca7000SetNmk qca7000SetNmkStub

#include "../examples/platformio_complete/src/cp_state_machine.h"
typedef uint32_t TickType_t;

// Stubs for logging and other ESP functions
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"

// Stub implementations
static bool pwm_running = false;
static int pwm_start_calls = 0;
static int pwm_set_calls = 0;

void cpPwmStartStub(uint16_t, bool) { pwm_running = true; ++pwm_start_calls; }
void cpPwmStopStub() { pwm_running = false; }
void cpPwmSetDutyStub(uint16_t, bool) { ++pwm_set_calls; }
bool cpPwmIsRunningStub() { return pwm_running; }

CpSubState g_cp_substate = CP_A;
CpSubState cpGetSubStateStub() { return g_cp_substate; }
uint16_t voutGetVoltageMvStub() { return 0; }

bool qca7000SleepStub() { return true; }
bool qca7000WakeStub() { return true; }
bool qca7000startSlacStub() { return true; }
void qca7000SetMacStub(const uint8_t*) {}
void qca7000SetNmkStub(const uint8_t*) {}

uint64_t esp_timer_get_time() { return 0; }
uint32_t esp_random() { return 0; }
int gpio_get_level(gpio_num_t) { return 1; }
void vTaskDelay(int) {}

bool g_use_random_mac = false;
uint8_t g_mac_addr[ETH_ALEN] = {};
std::atomic<uint32_t> g_slac_ts{0};
std::atomic<uint8_t> g_slac_state{0};

#include "../examples/platformio_complete/src/cp_state_machine.cpp"

TEST(EvseStateMachine, UnlockB1PwmStartThenSetDuty) {
    pwm_running = false;
    pwm_start_calls = pwm_set_calls = 0;
    g_cp_substate = CP_B1;

    stageEnter(EVSE_UNLOCK_B1);
    handleUnlockB1();
    EXPECT_EQ(pwm_start_calls, 1);
    EXPECT_EQ(pwm_set_calls, 0);
    EXPECT_EQ(stage.load(std::memory_order_relaxed), EVSE_UNLOCK_B1);

    t_stage.store(1, std::memory_order_relaxed);
    handleUnlockB1();
    EXPECT_EQ(pwm_start_calls, 1);
    EXPECT_EQ(pwm_set_calls, 1);
}

TEST(EvseStateMachine, UnlockB1ExitToIdle) {
    pwm_running = true;
    g_cp_substate = CP_A;

    stageEnter(EVSE_UNLOCK_B1);
    handleUnlockB1();
    EXPECT_EQ(stage.load(std::memory_order_relaxed), EVSE_IDLE_A);
}
