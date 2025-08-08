#include "cp_state_machine.h"
#include "cp_config.h"
#include "cp_pwm.h"
#include <atomic>
#include <esp32s3/qca7000.hpp>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_random.h>
#include <driver/gpio.h>

extern bool g_use_random_mac;
extern uint8_t g_mac_addr[ETH_ALEN];

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

static const char* TAG = "EVSE";

static inline void stageEnter(EvseStage s) {
    stage.store(s, std::memory_order_relaxed);
    t_stage.store(0, std::memory_order_relaxed);
    if (s == EVSE_IDLE_A)
        g_waiting_for_parm_req.store(false, std::memory_order_relaxed);
    ESP_LOGI(TAG, "Stage -> %s", stageName(s));
}

static void handleIdleA() {
    if (t_stage.load(std::memory_order_relaxed) == 0) {
        cpPwmStop();
        qca7000Sleep();
    }
    CpSubState s = cpGetSubState();
    if (s == CP_B1 || s == CP_B3) {
        // Apply 5% clamping during the initial handshake
        cpPwmStart(CP_PWM_DUTY_5PCT, true);
        stageEnter(EVSE_INITIALISE_B1);
    }
}

static void handleInitialiseB1() {
    CpSubState ss = cpGetSubState();
    if (ss == CP_A) {
        stageEnter(EVSE_IDLE_A);
        return;
    }
    if (ss == CP_E || ss == CP_F) {
        stageEnter(EVSE_POWER_DOWN);
        return;
    }
    if (t_stage.load(std::memory_order_relaxed) == 0) {
        if (!qca7000Wake()) {
            stageEnter(EVSE_IDLE_A);
            return;
        }
        if (g_use_random_mac) {
            qca7000SetMac(g_mac_addr);
        }
        if (!qca7000startSlac()) {
            stageEnter(EVSE_IDLE_A);
            return;
        }
        auto now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        g_slac_ts.store(now_ms, std::memory_order_relaxed);
        g_slac_init_ts.store(now_ms, std::memory_order_relaxed);
        g_waiting_for_parm_req.store(true, std::memory_order_relaxed);
        stageEnter(EVSE_DIGITAL_REQ_B2);
    }
}

static void handleDigitalReqB2() {
    static bool nmk_switched = false;
    CpSubState ss = cpGetSubState();
    if (ss == CP_A) {
        stageEnter(EVSE_IDLE_A);
        return;
    }
    if (ss == CP_E || ss == CP_F) {
        stageEnter(EVSE_POWER_DOWN);
        return;
    }
    if (g_slac_state.load(std::memory_order_relaxed) == SlacState::Matched &&
        (ss == CP_C || ss == CP_D)) {
        if (g_use_random_mac && !nmk_switched) {
            uint8_t nmk[slac::defs::NMK_LEN];
            for (uint8_t& b : nmk)
                b = static_cast<uint8_t>(esp_random() & 0xFF);
            qca7000SetNmk(nmk);
            nmk_switched = true;
        }
        stageEnter(EVSE_CABLE_CHECK_C);
        return;
    }
    if (t_stage.load(std::memory_order_relaxed) > T_HLC_EST_MS &&
        g_slac_state.load(std::memory_order_relaxed) != SlacState::Matched) {
        stageEnter(EVSE_INITIALISE_B1);
    }
}

static void handleCableCheckC() {
    CpSubState ss = cpGetSubState();
    if (ss == CP_A) {
        cpPwmStop();
        stageEnter(EVSE_IDLE_A);
        return;
    }
    if (ss == CP_E || ss == CP_F) {
        cpPwmStop();
        stageEnter(EVSE_POWER_DOWN);
        return;
    }
    if (t_stage.load(std::memory_order_relaxed) == 0) {
        // Placeholder for lock and isolation checks
    }
    if (gpio_get_level(static_cast<gpio_num_t>(ISOLATION_OK_PIN)) &&
        gpio_get_level(static_cast<gpio_num_t>(LOCK_FB_PIN))) {
        stageEnter(EVSE_PRECHARGE);
    } else if (t_stage.load(std::memory_order_relaxed) > T_ISO_CPLT_MS) {
        cpPwmStop();
        stageEnter(EVSE_POWER_DOWN);
    }
}

static void handlePrecharge() {
    CpSubState ss = cpGetSubState();
    if (ss == CP_A) {
        stageEnter(EVSE_IDLE_A);
        return;
    }
    if (ss == CP_E || ss == CP_F) {
        stageEnter(EVSE_POWER_DOWN);
        return;
    }
    if (t_stage.load(std::memory_order_relaxed) == 0) {
        // enable HV pre-charge converter
    }
    if (t_stage.load(std::memory_order_relaxed) > T_PC_DONE_MS) {
        stageEnter(EVSE_POWER_DOWN);
    }
    // converter output scaled to ADC range (~3.3V = ~400V)
    if (voutGetVoltageMv() > 3300) {
        stageEnter(EVSE_ENERGY_TRANSFER);
    }
}

static void handleEnergyTransfer() {
    CpSubState ss = cpGetSubState();
    if (ss == CP_A) {
        stageEnter(EVSE_IDLE_A);
        return;
    }
    if (ss == CP_E || ss == CP_F || ss == CP_B2) {
        stageEnter(EVSE_POWER_DOWN);
        return;
    }
    if (t_stage.load(std::memory_order_relaxed) > T_STOP_MAX_MS) {
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
    if (t_stage.load(std::memory_order_relaxed) == 0 || !cpPwmIsRunning()) {
        // Keep the pilot at 9V while waiting for unplug
        cpPwmStart(CP_PWM_DUTY_5PCT, true); // clamp as required during handshake
    } else {
        cpPwmSetDuty(CP_PWM_DUTY_5PCT, true);
    }
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
