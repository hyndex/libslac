#include <slac/match_log.hpp>
#include <port/logging_compat.hpp>
#include <port/esp32s3/qca7000.hpp>
#include <cstdio>

namespace slac {

static match_log_fn_t g_log_fn = nullptr;

void set_match_log_fn(match_log_fn_t fn) { g_log_fn = fn; }
match_log_fn_t get_match_log_fn() { return g_log_fn; }

__attribute__((weak)) char slac_get_cp_state() { return '?'; }

#ifndef SLAC_DISABLE_MATCH_LOG
static void default_log(const MatchLogInfo& info) {
    char pev[18];
    snprintf(pev, sizeof(pev), "%02X:%02X:%02X:%02X:%02X:%02X",
             info.pev_mac[0], info.pev_mac[1], info.pev_mac[2],
             info.pev_mac[3], info.pev_mac[4], info.pev_mac[5]);
    char evse[18];
    snprintf(evse, sizeof(evse), "%02X:%02X:%02X:%02X:%02X:%02X",
             info.evse_mac[0], info.evse_mac[1], info.evse_mac[2],
             info.evse_mac[3], info.evse_mac[4], info.evse_mac[5]);
    char nmk_hex[defs::NMK_LEN * 2 + 1];
    for (size_t i = 0; i < defs::NMK_LEN; ++i)
        sprintf(&nmk_hex[i * 2], "%02X", info.nmk[i]);

    ESP_LOGI(PLC_TAG, "SLAC MATCH OK");
    ESP_LOGI(PLC_TAG, "  PEV_MAC : %s", pev);
    ESP_LOGI(PLC_TAG, "  EVSE_MAC: %s", evse);
    ESP_LOGI(PLC_TAG, "  TONEMAP : min=%u avg=%u max=%u",
             info.tone_min, info.tone_avg, info.tone_max);
    ESP_LOGI(PLC_TAG, "  NMK     : %s", nmk_hex);
    ESP_LOGI(PLC_TAG, "  CP_STATE: %c", info.cp_state);
}
#endif

void slac_log_match(const MatchLogInfo& info) {
#ifdef SLAC_DISABLE_MATCH_LOG
    (void)info;
#else
    if (g_log_fn) {
        g_log_fn(info);
    } else {
        default_log(info);
    }
#endif
}

} // namespace slac
