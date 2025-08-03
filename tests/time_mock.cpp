#include <cstdint>

extern "C" uint32_t g_mock_millis = 0;

uint32_t slac_millis() { return g_mock_millis; }
uint32_t slac_micros() { return g_mock_millis * 1000; }
void slac_delay(uint32_t ms) { g_mock_millis += ms; }
void slac_noInterrupts() {}
void slac_interrupts() {}
