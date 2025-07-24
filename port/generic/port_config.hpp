#ifndef SLAC_GENERIC_PORT_CONFIG_HPP
#define SLAC_GENERIC_PORT_CONFIG_HPP

#include <stdint.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif

#ifndef slac_millis
#if defined(ARDUINO) && !defined(ESP_PLATFORM)
#define slac_millis millis
#endif
#endif

#ifndef slac_delay
#if defined(ARDUINO) && !defined(ESP_PLATFORM)
#define slac_delay(ms) delay(ms)
#endif
#endif

#ifndef slac_noInterrupts
#if defined(ARDUINO) && !defined(ESP_PLATFORM)
#define slac_noInterrupts noInterrupts
#endif
#endif

#ifndef slac_interrupts
#if defined(ARDUINO) && !defined(ESP_PLATFORM)
#define slac_interrupts interrupts
#endif
#endif

#endif // SLAC_GENERIC_PORT_CONFIG_HPP
