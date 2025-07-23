#ifndef SLAC_ENDIAN_COMPAT_HPP
#define SLAC_ENDIAN_COMPAT_HPP

#include <stdint.h>

inline uint16_t htons(uint16_t x) { return (x << 8) | (x >> 8); }
inline uint16_t ntohs(uint16_t x) { return htons(x); }

#endif // SLAC_ENDIAN_COMPAT_HPP
