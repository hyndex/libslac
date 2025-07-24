#ifndef SLAC_GENERIC_PORT_CONFIG_HPP
#define SLAC_GENERIC_PORT_CONFIG_HPP

#include <stdint.h>

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <chrono>
#include <thread>
#endif

#ifndef slac_millis
#ifdef ARDUINO
#define slac_millis millis
#else
static inline uint32_t slac_millis() {
    using namespace std::chrono;
    return (uint32_t)duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}
#endif
#endif

#ifndef slac_delay
#ifdef ARDUINO
#define slac_delay(ms) delay(ms)
#else
static inline void slac_delay(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
#endif
#endif

#ifndef slac_noInterrupts
#ifdef ARDUINO
#define slac_noInterrupts noInterrupts
#else
static inline void slac_noInterrupts() {}
#endif
#endif

#ifndef slac_interrupts
#ifdef ARDUINO
#define slac_interrupts interrupts
#else
static inline void slac_interrupts() {}
#endif
#endif

#endif // SLAC_GENERIC_PORT_CONFIG_HPP
