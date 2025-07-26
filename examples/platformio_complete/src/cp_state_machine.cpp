#include "cp_state_machine.h"
#include "cp_config.h"
#include "cp_pwm.h"
#include <atomic>
#include <port/esp32s3/qca7000.hpp>

static std::atomic<EvseStage> stage{EVSE_IDLE_A};
static std::atomic<uint32_t> t_stage{0};

static const char* stageName(EvseStage s) {
    switch (s) {
    case EVSE_IDLE_A:
        return "Idle";
    case EVSE_INITIALISE_B1:
        return "InitialiseB1";
    case EVSE_DIGITAL_REQ_B2:
        return "DigitalReqB2";
    case EVSE_CABLE_CHECK_C:
        return "CableCheckC";
    case EVSE_PRECHARGE:
        return "Precharge";
    case EVSE_ENERGY_TRANSFER:
        return "EnergyTransfer";
    case EVSE_POWER_DOWN:
        return "PowerDown";
    case EVSE_UNLOCK_B1:
        return "UnlockB1";
    default:
        return "?";
    }
}

const char* evseStageName(EvseStage s) {
    return stageName(s);
}

static inline void stageEnter(EvseStage s) {
    stage.store(s, std::memory_order_relaxed);
    t_stage.store(0, std::memory_order_relaxed);
    Serial.printf("[EVSE] Stage -> %s\n", stageName(s));
}

static void handleIdleA() {
    if (cpGetSubState() == CP_B1) {
        cpPwmStart(CP_PWM_DUTY_5PCT);
        stageEnter(EVSE_INITIALISE_B1);
    }
}

static void handleInitialiseB1() {
    if (t_stage.load(std::memory_order_relaxed) == 0) {
        qca7000startSlac();
        g_slac_ts.store(millis(), std::memory_order_relaxed);
    }
    if (t_stage.load(std::memory_order_relaxed) > T_CP_B1_FAIL_MS) {
        cpPwmStop();
        stageEnter(EVSE_IDLE_A);
    }
    if (cpDigitalCommRequested()) {
        stageEnter(EVSE_DIGITAL_REQ_B2);
    }
}

static void handleDigitalReqB2() {
    if (g_slac_state.load(std::memory_order_relaxed) == 6 && (cpGetSubState() == CP_C || cpGetSubState() == CP_D)) {
        stageEnter(EVSE_CABLE_CHECK_C);
        return;
    }
    if (t_stage.load(std::memory_order_relaxed) > T_HLC_EST_MS && g_slac_state.load(std::memory_order_relaxed) != 6) {
        stageEnter(EVSE_INITIALISE_B1);
    }
}

static void handleCableCheckC() {
    if (t_stage.load(std::memory_order_relaxed) == 0) {
        // Placeholder for lock and isolation checks
    }
    if (digitalRead(ISOLATION_OK_PIN) && digitalRead(LOCK_FB_PIN)) {
        stageEnter(EVSE_PRECHARGE);
    } else if (t_stage.load(std::memory_order_relaxed) > T_ISO_CPLT_MS) {
        cpPwmStop();
        stageEnter(EVSE_POWER_DOWN);
    }
}

static void handlePrecharge() {
    if (t_stage.load(std::memory_order_relaxed) == 0) {
        // enable HV pre-charge converter
    }
    if (t_stage.load(std::memory_order_relaxed) > T_PC_DONE_MS) {
        stageEnter(EVSE_POWER_DOWN);
    }
    // converter output scaled to ADC range (~3.3V = ~400V)
    if (analogReadMilliVolts(VOUT_MON_ADC_PIN) > 3300) {
        stageEnter(EVSE_ENERGY_TRANSFER);
    }
}

static void handleEnergyTransfer() {
    if (cpGetSubState() == CP_B2) {
        stageEnter(EVSE_POWER_DOWN);
    }
}

static void handlePowerDown() {
    if (t_stage.load(std::memory_order_relaxed) == 0) {
        // ramp current down, open contactor
    }
    if (t_stage.load(std::memory_order_relaxed) > T_SAFE_MAX_MS) {
        stageEnter(EVSE_UNLOCK_B1);
    }
}

static void handleUnlockB1() {
    cpPwmStart(CP_PWM_DUTY_5PCT); // hold 9V while waiting for unplug
    if (cpGetSubState() == CP_A) {
        stageEnter(EVSE_IDLE_A);
    }
}

void evseStateMachineInit() {
    stageEnter(EVSE_IDLE_A);
}

EvseStage evseGetStage() {
    return stage.load(std::memory_order_relaxed);
}

void evseStateMachineTask(void*) {
    const TickType_t period = 1;
    while (true) {
        EvseStage s = stage.load(std::memory_order_relaxed);
        switch (s) {
        case EVSE_IDLE_A:
            handleIdleA();
            break;
        case EVSE_INITIALISE_B1:
            handleInitialiseB1();
            break;
        case EVSE_DIGITAL_REQ_B2:
            handleDigitalReqB2();
            break;
        case EVSE_CABLE_CHECK_C:
            handleCableCheckC();
            break;
        case EVSE_PRECHARGE:
            handlePrecharge();
            break;
        case EVSE_ENERGY_TRANSFER:
            handleEnergyTransfer();
            break;
        case EVSE_POWER_DOWN:
            handlePowerDown();
            break;
        case EVSE_UNLOCK_B1:
            handleUnlockB1();
            break;
        }
        t_stage.fetch_add(period, std::memory_order_relaxed);
        vTaskDelay(pdMS_TO_TICKS(period));
    }
}
