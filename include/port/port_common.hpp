#ifndef SLAC_GENERIC_PORT_CONFIG_HPP
#define SLAC_GENERIC_PORT_CONFIG_HPP

#include <stdint.h>

#ifndef slac_millis
inline __attribute__((weak)) uint32_t slac_millis() { return 0; }
#endif

#ifndef slac_delay
inline __attribute__((weak)) void slac_delay(uint32_t) {}
#endif

#ifndef slac_micros
inline __attribute__((weak)) uint32_t slac_micros() { return 0; }
#endif

#ifndef slac_noInterrupts
inline __attribute__((weak)) void slac_noInterrupts() {}
#endif

#ifndef slac_interrupts
inline __attribute__((weak)) void slac_interrupts() {}
#endif

#endif // SLAC_GENERIC_PORT_CONFIG_HPP
