#ifndef SLAC_FSM_HPP
#define SLAC_FSM_HPP

#include <fsm/buffer.hpp>
#include <fsm/fsm.hpp>
#include <fsm/states.hpp>

namespace slac {

// Re-export libfsm into the slac namespace for convenience
namespace fsm = ::fsm;

enum class SlacEvent {
    GotParmCnf,
    SoundIntervalElapsed,
    GotAttenCharInd,
    GotSetKeyReq,
    GotValidateReq,
    GotMatchReq,
    Timeout,
    Error
};

} // namespace slac

#endif // SLAC_FSM_HPP
