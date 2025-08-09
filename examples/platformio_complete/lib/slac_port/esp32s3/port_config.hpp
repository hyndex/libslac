#ifndef SLAC_PORT_CONFIG_HPP
#define SLAC_PORT_CONFIG_HPP

#define slac_millis slac_millis
#define slac_delay slac_delay
#define slac_micros slac_micros
#define slac_noInterrupts slac_noInterrupts
#define slac_interrupts slac_interrupts
#include <port/port_common.hpp>
#undef slac_millis
#undef slac_delay
#undef slac_micros
#undef slac_noInterrupts
#undef slac_interrupts

#ifndef PLC_SPI_CS_PIN
#define PLC_SPI_CS_PIN   36
#endif
#ifndef PLC_SPI_RST_PIN
#define PLC_SPI_RST_PIN  40
#endif
#ifndef PLC_SPI_SCK_PIN
#define PLC_SPI_SCK_PIN  48
#endif
#ifndef PLC_SPI_MISO_PIN
#define PLC_SPI_MISO_PIN 21
#endif
#ifndef PLC_SPI_MOSI_PIN
#define PLC_SPI_MOSI_PIN 47
#endif

#ifdef ESP_PLATFORM
#include <stdint.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/task.h>
#endif

#define PLC_SPI_SLOW_HZ  1000000
#define QCA7000_SPI_FAST_HZ 8000000
#define QCA7000_SPI_BURST_LEN 512

#ifndef QCA7000_HARDRESET_LOW_MS
#define QCA7000_HARDRESET_LOW_MS 20
#endif
#ifndef QCA7000_HARDRESET_HIGH_MS
#define QCA7000_HARDRESET_HIGH_MS 150
#endif
#ifndef QCA7000_CPUON_TIMEOUT_MS
#define QCA7000_CPUON_TIMEOUT_MS 500
#endif

#ifndef QCA7000_MAX_RETRIES
#define QCA7000_MAX_RETRIES 3
#endif

#ifndef CONFIG_RX_RING_SIZE
#define CONFIG_RX_RING_SIZE 8
#endif

#ifndef PLC_PWR_EN_PIN
#define PLC_PWR_EN_PIN -1
#endif
#ifndef PLC_INT_PIN
#define PLC_INT_PIN -1
#endif
static_assert(PLC_INT_PIN >= 0, "PLC_INT_PIN undefined");

#ifndef PLC_SPI_CS_PIN
static_assert(false, "PLC_SPI_CS_PIN undefined");
#else
static_assert(PLC_SPI_CS_PIN >= 0, "CS pin unset");
#endif
#ifndef PLC_SPI_RST_PIN
static_assert(false, "PLC_SPI_RST_PIN undefined");
#else
static_assert(PLC_SPI_RST_PIN >= 0, "RST pin unset");
#endif
static_assert(QCA7000_SPI_BURST_LEN <= 512, "Burst length too large");

namespace slac {
} // namespace slac

#ifdef ESP_PLATFORM
static inline uint32_t slac_millis() {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}
static inline uint32_t slac_micros() {
    return (uint32_t)esp_timer_get_time();
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
