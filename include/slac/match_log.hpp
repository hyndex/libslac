#ifndef SLAC_MATCH_LOG_HPP
#define SLAC_MATCH_LOG_HPP

#include <slac/slac.hpp>
#include <stdint.h>

namespace slac {

struct MatchLogInfo {
    uint8_t pev_mac[ETH_ALEN];
    uint8_t evse_mac[ETH_ALEN];
    uint8_t nmk[defs::NMK_LEN];
    uint8_t tone_min;
    uint8_t tone_avg;
    uint8_t tone_max;
    char cp_state;
};

using match_log_fn_t = void (*)(const MatchLogInfo&);

void set_match_log_fn(match_log_fn_t fn);
match_log_fn_t get_match_log_fn();

void slac_log_match(const MatchLogInfo& info);

char slac_get_cp_state();

extern const char* SLAC_LOG_TAG;

} // namespace slac

#endif // SLAC_MATCH_LOG_HPP
