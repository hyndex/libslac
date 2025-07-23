#ifndef SLAC_PORT_CONFIG_HPP
#define SLAC_PORT_CONFIG_HPP

#include <stdint.h>

#ifdef ESP_PLATFORM
static inline uint16_t le16toh(uint16_t v) { return v; }
static inline uint16_t htole16(uint16_t v) { return v; }
static inline uint32_t le32toh(uint32_t v) { return v; }
static inline uint32_t htole32(uint32_t v) { return v; }
#endif

#endif // SLAC_PORT_CONFIG_HPP
