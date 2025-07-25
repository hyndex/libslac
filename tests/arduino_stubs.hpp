#pragma once
#include <cstdint>
#include <cstddef>

#define HIGH 1
#define LOW 0
#define OUTPUT 1

struct SPISettings {
    SPISettings(uint32_t, uint8_t, uint8_t) {}
};

class SPIClass {
public:
    void begin() {}
    void beginTransaction(const SPISettings&) {}
    void endTransaction() {}
    uint8_t transfer(uint8_t) { return 0; }
    uint16_t transfer16(uint16_t) { return 0; }
    void writeBytes(const uint8_t*, size_t) {}
};

extern SPIClass SPI;
inline SPIClass SPI;

inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline uint32_t millis() { return 0; }
inline uint32_t micros() { return 0; }
inline void delay(unsigned int) {}
inline void noInterrupts() {}
inline void interrupts() {}
