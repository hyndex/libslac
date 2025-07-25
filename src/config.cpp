#include <slac/config.hpp>

namespace slac {

static config g_cfg{}; // default-initialized with default values

const config& get_config() { return g_cfg; }

void set_config(const config& cfg) { g_cfg = cfg; }

uint32_t spi_fast_hz() { return g_cfg.spi_fast_hz; }
void set_spi_fast_hz(uint32_t hz) { g_cfg.spi_fast_hz = hz; }

uint32_t spi_slow_hz() { return g_cfg.spi_slow_hz; }
void set_spi_slow_hz(uint32_t hz) { g_cfg.spi_slow_hz = hz; }

size_t spi_burst_len() { return g_cfg.spi_burst_len; }
void set_spi_burst_len(size_t len) { g_cfg.spi_burst_len = len; }

uint32_t hardreset_low_ms() { return g_cfg.hardreset_low_ms; }
void set_hardreset_low_ms(uint32_t ms) { g_cfg.hardreset_low_ms = ms; }

uint32_t hardreset_high_ms() { return g_cfg.hardreset_high_ms; }
void set_hardreset_high_ms(uint32_t ms) { g_cfg.hardreset_high_ms = ms; }

uint32_t cpuon_timeout_ms() { return g_cfg.cpuon_timeout_ms; }
void set_cpuon_timeout_ms(uint32_t ms) { g_cfg.cpuon_timeout_ms = ms; }

} // namespace slac
