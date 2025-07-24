#ifndef SLAC_GENERIC_PORT_CONFIG_HPP
#define SLAC_GENERIC_PORT_CONFIG_HPP

#include <chrono>
#include <mutex>
#include <stdint.h>
#include <thread>

#ifdef ARDUINO
#include <Arduino.h>
#endif

#ifndef slac_millis
#if defined(ARDUINO) && !defined(ESP_PLATFORM)
static inline uint32_t slac_millis() {
    return millis();
}
#elif !defined(ESP_PLATFORM)
static inline uint32_t slac_millis() {
    using namespace std::chrono;
    return (uint32_t)duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}
#else
#error "slac_millis() not defined for this platform"
#endif
#endif

#ifndef slac_delay
#if defined(ARDUINO) && !defined(ESP_PLATFORM)
static inline void slac_delay(uint32_t ms) {
    delay(ms);
}
#elif !defined(ESP_PLATFORM)
static inline void slac_delay(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
#else
#error "slac_delay() not defined for this platform"
#endif
#endif

#ifndef slac_noInterrupts
#if defined(ARDUINO) && !defined(ESP_PLATFORM)
static inline void slac_noInterrupts() {
    noInterrupts();
}
#elif !defined(ESP_PLATFORM)
static inline std::mutex& slac_mutex() {
    static std::mutex m;
    return m;
}
static inline void slac_noInterrupts() {
    slac_mutex().lock();
}
#else
#error "slac_noInterrupts() not defined for this platform"
#endif
#endif

#ifndef slac_interrupts
#if defined(ARDUINO) && !defined(ESP_PLATFORM)
static inline void slac_interrupts() {
    interrupts();
}
#elif !defined(ESP_PLATFORM)
static inline void slac_interrupts() {
    slac_mutex().unlock();
}
#else
#error "slac_interrupts() not defined for this platform"
#endif
#endif

#endif // SLAC_GENERIC_PORT_CONFIG_HPP
