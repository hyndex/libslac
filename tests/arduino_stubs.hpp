#pragma once
#include <cstdint>
#include <cstddef>

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define MSBFIRST 1
#define SPI_MODE3 3

struct SPISettings {
    SPISettings(uint32_t, uint8_t, uint8_t) {}
};

using transfer_cb_t = uint8_t (*)(uint8_t);
using transfer16_cb_t = uint16_t (*)(uint16_t);
using write_cb_t = void (*)(const uint8_t*, size_t);

extern transfer_cb_t g_transfer_cb;
extern transfer16_cb_t g_transfer16_cb;
extern write_cb_t g_write_cb;

class SPIClass {
public:
    void begin() {}
    void beginTransaction(const SPISettings&) {}
    void endTransaction() {}
    uint8_t transfer(uint8_t v) { return g_transfer_cb ? g_transfer_cb(v) : 0; }
    uint16_t transfer16(uint16_t v) {
        return g_transfer16_cb ? g_transfer16_cb(v) : 0;
    }
    void writeBytes(const uint8_t* d, size_t l) {
        if (g_write_cb)
            g_write_cb(d, l);
    }
};

extern SPIClass SPI;
inline SPIClass SPI;

inline transfer_cb_t g_transfer_cb = nullptr;
inline transfer16_cb_t g_transfer16_cb = nullptr;
inline write_cb_t g_write_cb = nullptr;

inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
extern uint32_t g_mock_millis;
inline uint32_t millis() { return g_mock_millis; }
inline uint32_t micros() { return g_mock_millis * 1000; }
inline void delay(unsigned int ms) { g_mock_millis += ms; }
inline void noInterrupts() {}
inline void interrupts() {}
extern int g_mock_adc_mv;
inline int analogReadMilliVolts(int) { return g_mock_adc_mv; }
