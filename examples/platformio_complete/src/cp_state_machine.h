#pragma once
#include <Arduino.h>
#include <atomic>
#include "cp_monitor.h"

enum EvseStage : uint8_t {
    EVSE_IDLE_A = 0,
    EVSE_INITIALISE_B1,
    EVSE_DIGITAL_REQ_B2,
    EVSE_CABLE_CHECK_C,
    EVSE_PRECHARGE,
    EVSE_ENERGY_TRANSFER,
    EVSE_POWER_DOWN,
    EVSE_UNLOCK_B1
};

void evseStateMachineInit();
void evseStateMachineTask(void*);
EvseStage evseGetStage();
const char* evseStageName(EvseStage);

extern std::atomic<uint8_t> g_slac_state;
extern std::atomic<uint32_t> g_slac_ts;
