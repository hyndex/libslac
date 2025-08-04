#include <cstdio>
#include <slac/match_log.hpp>

namespace slac {

static match_log_fn_t g_log_fn = nullptr;

__attribute__((weak)) const char* SLAC_LOG_TAG = "SLAC";

void set_match_log_fn(match_log_fn_t fn) {
    g_log_fn = fn;
}
match_log_fn_t get_match_log_fn() {
    return g_log_fn;
}

__attribute__((weak)) char slac_get_cp_state() {
    return '?';
}

#ifndef SLAC_DISABLE_MATCH_LOG
static void default_log(const MatchLogInfo& info) {
    char pev[18];
    snprintf(pev, sizeof(pev), "%02X:%02X:%02X:%02X:%02X:%02X", info.pev_mac[0], info.pev_mac[1], info.pev_mac[2],
             info.pev_mac[3], info.pev_mac[4], info.pev_mac[5]);
    char evse[18];
    snprintf(evse, sizeof(evse), "%02X:%02X:%02X:%02X:%02X:%02X", info.evse_mac[0], info.evse_mac[1], info.evse_mac[2],
             info.evse_mac[3], info.evse_mac[4], info.evse_mac[5]);
    char nmk_hex[defs::NMK_LEN * 2 + 1];
    for (size_t i = 0; i < defs::NMK_LEN; ++i)
        sprintf(&nmk_hex[i * 2], "%02X", info.nmk[i]);

    printf("[%s] SLAC MATCH OK\n", SLAC_LOG_TAG);
    printf("[%s]   PEV_MAC : %s\n", SLAC_LOG_TAG, pev);
    printf("[%s]   EVSE_MAC: %s\n", SLAC_LOG_TAG, evse);
    printf("[%s]   TONEMAP : min=%u avg=%u max=%u\n", SLAC_LOG_TAG, info.tone_min, info.tone_avg, info.tone_max);
    printf("[%s]   NMK     : %s\n", SLAC_LOG_TAG, nmk_hex);
    printf("[%s]   CP_STATE: %c\n", SLAC_LOG_TAG, info.cp_state);
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
