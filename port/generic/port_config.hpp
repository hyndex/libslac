#ifndef SLAC_GENERIC_PORT_CONFIG_HPP
#define SLAC_GENERIC_PORT_CONFIG_HPP

#include <stdint.h>
#include <chrono>
#include <thread>

// Weak default implementations for platforms without a custom port_config.hpp

__attribute__((weak)) static inline uint32_t slac_millis() {
    using namespace std::chrono;
    static const auto start = steady_clock::now();
    return static_cast<uint32_t>(duration_cast<milliseconds>(steady_clock::now() - start).count());
}

__attribute__((weak)) static inline void slac_delay(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

__attribute__((weak)) static inline void slac_noInterrupts() {}
__attribute__((weak)) static inline void slac_interrupts() {}

#endif // SLAC_GENERIC_PORT_CONFIG_HPP
