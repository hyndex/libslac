#ifndef SLAC_PORT_CONFIG_HPP
#define SLAC_PORT_CONFIG_HPP

#include <endian.h>
#include <stdint.h>

#ifdef ESP_PLATFORM
#  define le16toh(x) (x)
#  define htole16(x) (x)
#  define le32toh(x) (x)
#  define htole32(x) (x)
#endif

#endif // SLAC_PORT_CONFIG_HPP
