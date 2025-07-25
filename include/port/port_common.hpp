#ifndef SLAC_GENERIC_PORT_CONFIG_HPP
#define SLAC_GENERIC_PORT_CONFIG_HPP

#include <stdint.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif

#ifndef slac_millis
#if defined(ARDUINO) && !defined(ESP_PLATFORM)
static inline uint32_t slac_millis() { return millis(); }
#endif
#endif

#ifndef slac_delay
#if defined(ARDUINO) && !defined(ESP_PLATFORM)
static inline void slac_delay(uint32_t ms) { delay(ms); }
#endif
#endif

#ifndef slac_micros
#if defined(ARDUINO) && !defined(ESP_PLATFORM)
static inline uint32_t slac_micros() { return micros(); }
#endif
#endif

#ifndef slac_noInterrupts
#if defined(ARDUINO) && !defined(ESP_PLATFORM)
static inline void slac_noInterrupts() { noInterrupts(); }
#endif
#endif

#ifndef slac_interrupts
#if defined(ARDUINO) && !defined(ESP_PLATFORM)
static inline void slac_interrupts() { interrupts(); }
#endif
#endif

#endif // SLAC_GENERIC_PORT_CONFIG_HPP
