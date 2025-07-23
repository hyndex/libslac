#pragma once

#ifndef ESP_PLATFORM
#include <linux/if_ether.h>
#else
#include "port/esp32s3/ethernet_defs.hpp"
#endif
