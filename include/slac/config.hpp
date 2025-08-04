#pragma once

#include <stddef.h>
#include <stdint.h>


namespace slac {

struct config {
    uint32_t spi_fast_hz = 8000000;
    uint32_t spi_slow_hz = 1000000;
    size_t spi_burst_len = 512;
    uint32_t hardreset_low_ms = 20;
    uint32_t hardreset_high_ms = 150;
    uint32_t cpuon_timeout_ms = 200;
    uint32_t max_retries = 3;
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

uint32_t max_retries();
void set_max_retries(uint32_t retries);

bool validation_disabled();
void set_validation_disabled(bool disabled);

} // namespace slac
