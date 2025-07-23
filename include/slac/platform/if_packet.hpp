#pragma once

#include "ethernet_defs.hpp"

struct sockaddr_ll {
    unsigned short sll_family;
    unsigned short sll_protocol;
    int sll_ifindex;
    unsigned short sll_hatype;
    unsigned char sll_pkttype;
    unsigned char sll_halen;
    unsigned char sll_addr[8];
};

#ifndef AF_PACKET
#define AF_PACKET 17
#endif
