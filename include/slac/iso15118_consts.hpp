#ifndef SLAC_ISO15118_CONSTS_HPP
#define SLAC_ISO15118_CONSTS_HPP

namespace slac {
namespace defs {

const int C_EV_START_ATTEN_CHAR_INDS = 3;
const int C_EV_MATCH_RETRY = 2;
const int C_EV_MATCH_MNBC = 10;
const int TP_EV_BATCH_MSG_INTERVAL_MS = 40; // 20ms - 50ms, interval between start_atten_char and mnbc_sound msgs
const int TT_EV_ATTEN_RESULTS_MS = 1200;    // max. 1200ms
const int TT_EVSE_MATCH_MNBC_MS = 600;
const int TT_MATCH_SEQUENCE_MS = 400;
const int TT_MATCH_RESPONSE_MS = 200;
const int TT_EVSE_MATCH_SESSION_MS = 10000;
const int TT_EVSE_SLAC_INIT_MS = 40000; // (20s - 50s)
const int TT_MATCH_JOIN_MS = 12000;     // max. 12s
const int T_STEP_EF_MS = 4000;          // min. 4s

} // namespace defs
} // namespace slac

#endif // SLAC_ISO15118_CONSTS_HPP
