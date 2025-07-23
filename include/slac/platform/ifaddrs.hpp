#pragma once

#ifndef ESP_PLATFORM
#include <ifaddrs.h>
#else
struct ifaddrs;
static inline int getifaddrs(struct ifaddrs**) { return -1; }
static inline void freeifaddrs(struct ifaddrs*) {}
#endif
