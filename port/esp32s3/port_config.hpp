#ifndef SLAC_PORT_CONFIG_HPP
#define SLAC_PORT_CONFIG_HPP

#include "../port_config.hpp"

#ifdef ESP_PLATFORM
#include <stdint.h>

static inline uint16_t le16toh(uint16_t v) {
    return v;
}
static inline uint16_t htole16(uint16_t v) {
    return v;
}
static inline uint32_t le32toh(uint32_t v) {
    return v;
}
static inline uint32_t htole32(uint32_t v) {
    return v;
}

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/task.h>

static inline uint32_t slac_millis() {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}
static inline void slac_delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}
static inline void slac_noInterrupts() {
    portDISABLE_INTERRUPTS();
}
static inline void slac_interrupts() {
    portENABLE_INTERRUPTS();
}
#endif // ESP_PLATFORM

#endif // SLAC_PORT_CONFIG_HPP
