#ifndef SLAC_SLAC_STATES_HPP
#define SLAC_SLAC_STATES_HPP

#include <cstdint>

enum class SlacState : uint8_t {
    Idle = 0,
    WaitParmCnf = 1,
    Sounding = 2,
    WaitSetKey = 3,
    WaitValidate = 4,
    WaitMatch = 5,
    Matched = 6,
    Failed = 0xFF,
};

#endif // SLAC_SLAC_STATES_HPP
