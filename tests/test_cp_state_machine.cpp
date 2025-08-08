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
static int pwm_stop_calls = 0;
static int sleep_calls = 0;
static uint16_t vout_mv = 0;

void cpPwmStartStub(uint16_t, bool) { pwm_running = true; ++pwm_start_calls; }
void cpPwmStopStub() { pwm_running = false; ++pwm_stop_calls; }
void cpPwmSetDutyStub(uint16_t, bool) { ++pwm_set_calls; }
bool cpPwmIsRunningStub() { return pwm_running; }

CpSubState g_cp_substate = CP_A;
CpSubState cpGetSubStateStub() { return g_cp_substate; }
uint16_t voutGetVoltageMvStub() { return vout_mv; }

bool qca7000SleepStub() { ++sleep_calls; return true; }
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
std::atomic<uint32_t> g_slac_init_ts{0};
std::atomic<bool> g_waiting_for_parm_req{false};
std::atomic<SlacState> g_slac_state{SlacState::Idle};

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

TEST(EvseStateMachine, InitialiseB1Faults) {
    g_cp_substate = CP_E;
    stageEnter(EVSE_INITIALISE_B1);
    handleInitialiseB1();
    EXPECT_EQ(stage.load(std::memory_order_relaxed), EVSE_POWER_DOWN);

    g_cp_substate = CP_A;
    stageEnter(EVSE_INITIALISE_B1);
    handleInitialiseB1();
    EXPECT_EQ(stage.load(std::memory_order_relaxed), EVSE_IDLE_A);
}

TEST(EvseStateMachine, InitialiseB1Success) {
    g_cp_substate = CP_B1;
    stageEnter(EVSE_INITIALISE_B1);
    handleInitialiseB1();
    EXPECT_EQ(stage.load(std::memory_order_relaxed), EVSE_DIGITAL_REQ_B2);
}

TEST(EvseStateMachine, DigitalReqB2Unplug) {
    g_cp_substate = CP_A;
    stageEnter(EVSE_DIGITAL_REQ_B2);
    handleDigitalReqB2();
    EXPECT_EQ(stage.load(std::memory_order_relaxed), EVSE_IDLE_A);
}

TEST(EvseStateMachine, DigitalReqB2AdvanceAndFault) {
    // Successful advance to CableCheckC
    g_slac_state.store(SlacState::Matched, std::memory_order_relaxed);
    g_cp_substate = CP_C;
    stageEnter(EVSE_DIGITAL_REQ_B2);
    handleDigitalReqB2();
    EXPECT_EQ(stage.load(std::memory_order_relaxed), EVSE_CABLE_CHECK_C);

    // Fault to PowerDown
    g_cp_substate = CP_E;
    stageEnter(EVSE_DIGITAL_REQ_B2);
    handleDigitalReqB2();
    EXPECT_EQ(stage.load(std::memory_order_relaxed), EVSE_POWER_DOWN);

    // Timeout back to InitialiseB1
    g_cp_substate = CP_D;
    g_slac_state.store(SlacState::Idle, std::memory_order_relaxed);
    stageEnter(EVSE_DIGITAL_REQ_B2);
    t_stage.store(T_HLC_EST_MS + 1, std::memory_order_relaxed);
    handleDigitalReqB2();
    EXPECT_EQ(stage.load(std::memory_order_relaxed), EVSE_INITIALISE_B1);
}

TEST(EvseStateMachine, CableCheckCFault) {
    g_cp_substate = CP_F;
    stageEnter(EVSE_CABLE_CHECK_C);
    handleCableCheckC();
    EXPECT_EQ(stage.load(std::memory_order_relaxed), EVSE_POWER_DOWN);
}

TEST(EvseStateMachine, CableCheckCSuccess) {
    g_cp_substate = CP_C;
    stageEnter(EVSE_CABLE_CHECK_C);
    handleCableCheckC();
    EXPECT_EQ(stage.load(std::memory_order_relaxed), EVSE_PRECHARGE);
}

TEST(EvseStateMachine, PrechargeUnplug) {
    g_cp_substate = CP_A;
    stageEnter(EVSE_PRECHARGE);
    handlePrecharge();
    EXPECT_EQ(stage.load(std::memory_order_relaxed), EVSE_IDLE_A);
}

TEST(EvseStateMachine, PrechargeFaultAndComplete) {
    // Fault to PowerDown
    g_cp_substate = CP_E;
    stageEnter(EVSE_PRECHARGE);
    handlePrecharge();
    EXPECT_EQ(stage.load(std::memory_order_relaxed), EVSE_POWER_DOWN);

    // Successful completion to EnergyTransfer
    g_cp_substate = CP_C;
    vout_mv = 3301;
    stageEnter(EVSE_PRECHARGE);
    handlePrecharge();
    EXPECT_EQ(stage.load(std::memory_order_relaxed), EVSE_ENERGY_TRANSFER);
    vout_mv = 0;
}

TEST(EvseStateMachine, EnergyTransferFaultsAndTimeout) {
    g_cp_substate = CP_E;
    stageEnter(EVSE_ENERGY_TRANSFER);
    handleEnergyTransfer();
    EXPECT_EQ(stage.load(std::memory_order_relaxed), EVSE_POWER_DOWN);

    g_cp_substate = CP_A;
    stageEnter(EVSE_ENERGY_TRANSFER);
    handleEnergyTransfer();
    EXPECT_EQ(stage.load(std::memory_order_relaxed), EVSE_IDLE_A);

    g_cp_substate = CP_C;
    stageEnter(EVSE_ENERGY_TRANSFER);
    t_stage.store(T_STOP_MAX_MS + 1, std::memory_order_relaxed);
    handleEnergyTransfer();
    EXPECT_EQ(stage.load(std::memory_order_relaxed), EVSE_POWER_DOWN);
}

TEST(EvseStateMachine, PowerDownUnlockTransition) {
    stageEnter(EVSE_POWER_DOWN);
    t_stage.store(T_SAFE_MAX_MS + 1, std::memory_order_relaxed);
    handlePowerDown();
    EXPECT_EQ(stage.load(std::memory_order_relaxed), EVSE_UNLOCK_B1);
}

TEST(EvseStateMachine, IdleATransitionAndSleep) {
    pwm_stop_calls = sleep_calls = pwm_start_calls = 0;
    g_cp_substate = CP_B1;
    stageEnter(EVSE_IDLE_A);
    handleIdleA();
    EXPECT_EQ(pwm_stop_calls, 1);
    EXPECT_EQ(sleep_calls, 1);
    EXPECT_EQ(pwm_start_calls, 1);
    EXPECT_EQ(stage.load(std::memory_order_relaxed), EVSE_INITIALISE_B1);
}
