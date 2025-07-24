#ifndef SLAC_ENDIAN_COMPAT_HPP
#define SLAC_ENDIAN_COMPAT_HPP

#include <stdint.h>

#ifndef htons
static inline uint16_t htons(uint16_t x) { return (x << 8) | (x >> 8); }
#endif
#ifndef ntohs
static inline uint16_t ntohs(uint16_t x) { return htons(x); }
#endif

#endif // SLAC_ENDIAN_COMPAT_HPP
