#ifndef SLAC_GENERIC_PORT_CONFIG_HPP
#define SLAC_GENERIC_PORT_CONFIG_HPP

#include <stdint.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <chrono>
#include <thread>
#if defined(_WIN32)
#include <winsock2.h>
#include <stdlib.h>
#endif

#ifndef slac_millis
#if defined(ARDUINO) && !defined(ESP_PLATFORM)
static inline uint32_t slac_millis() { return millis(); }
#elif !defined(ESP_PLATFORM)
static inline uint32_t slac_millis() {
    using namespace std::chrono;
    static const auto start = steady_clock::now();
    return static_cast<uint32_t>(duration_cast<milliseconds>(steady_clock::now() - start).count());
}
#endif
#endif

#ifndef slac_delay
#if defined(ARDUINO) && !defined(ESP_PLATFORM)
static inline void slac_delay(uint32_t ms) { delay(ms); }
#elif !defined(ESP_PLATFORM)
static inline void slac_delay(uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
#endif
#endif

#ifndef slac_noInterrupts
#if defined(ARDUINO) && !defined(ESP_PLATFORM)
static inline void slac_noInterrupts() { noInterrupts(); }
#elif !defined(ESP_PLATFORM)
static inline void slac_noInterrupts() {}
#endif
#endif

#ifndef slac_interrupts
#if defined(ARDUINO) && !defined(ESP_PLATFORM)
static inline void slac_interrupts() { interrupts(); }
#elif !defined(ESP_PLATFORM)
static inline void slac_interrupts() {}
#endif
#endif

#ifndef htole16
#if defined(_WIN32)
#define htole16(x) (x)
#define le16toh(x) (x)
#else
#include <endian.h>
#endif
#endif

#endif // SLAC_GENERIC_PORT_CONFIG_HPP
