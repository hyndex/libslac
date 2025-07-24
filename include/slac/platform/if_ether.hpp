#pragma once

#ifndef ESP_PLATFORM
#include <linux/if_ether.h>
#else
#include "port/microcontroller/ethernet_defs.hpp"
#endif
