#pragma once

#include <stddef.h>
#include <stdint.h>

#include "port/esp32s3/port_config.hpp"

namespace slac {

struct config {
    uint32_t spi_fast_hz = QCA7000_SPI_FAST_HZ;
    uint32_t spi_slow_hz = PLC_SPI_SLOW_HZ;
    size_t spi_burst_len = QCA7000_SPI_BURST_LEN;
    uint32_t hardreset_low_ms = QCA7000_HARDRESET_LOW_MS;
    uint32_t hardreset_high_ms = QCA7000_HARDRESET_HIGH_MS;
    uint32_t cpuon_timeout_ms = QCA7000_CPUON_TIMEOUT_MS;
    bool disable_validation = false;
};

const config& get_config();
void set_config(const config& cfg);

uint32_t spi_fast_hz();
void set_spi_fast_hz(uint32_t hz);

uint32_t spi_slow_hz();
void set_spi_slow_hz(uint32_t hz);

size_t spi_burst_len();
void set_spi_burst_len(size_t len);

uint32_t hardreset_low_ms();
void set_hardreset_low_ms(uint32_t ms);

uint32_t hardreset_high_ms();
void set_hardreset_high_ms(uint32_t ms);

uint32_t cpuon_timeout_ms();
void set_cpuon_timeout_ms(uint32_t ms);

bool validation_disabled();
void set_validation_disabled(bool disabled);

} // namespace slac
