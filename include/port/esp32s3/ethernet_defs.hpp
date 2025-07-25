#ifndef SLAC_ETHERNET_DEFS_HPP
#define SLAC_ETHERNET_DEFS_HPP

#include "../port_common.hpp"
#ifdef ESP_PLATFORM
#include "port_config.hpp"
#endif

#include <stdint.h>

#define ETH_ALEN 6
#define ETH_HLEN 14
#define ETH_FRAME_LEN 1514

struct ether_header {
    uint8_t ether_dhost[ETH_ALEN];
    uint8_t ether_shost[ETH_ALEN];
    uint16_t ether_type;
} __attribute__((packed));

#endif // SLAC_ETHERNET_DEFS_HPP
