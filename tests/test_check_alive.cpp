#include <gtest/gtest.h>
#define ARDUINO
#include "arduino_stubs.hpp"
#include "port/esp32s3/qca7000.hpp"

extern uint16_t mock_signature;
extern uint16_t mock_wrbuf;
extern uint16_t mock_intr_cause;

TEST(Qca7000CheckAlive, ReturnsTrue) {
    mock_signature = 0xAA55;
    mock_wrbuf = 0x0C5B;
    mock_intr_cause = SPI_INT_CPU_ON;
    EXPECT_TRUE(qca7000CheckAlive());
}

TEST(Qca7000CheckAlive, WrongSignature) {
    mock_signature = 0x1234;
    mock_wrbuf = 0x0C5B;
    mock_intr_cause = SPI_INT_CPU_ON;
    EXPECT_FALSE(qca7000CheckAlive());
}

TEST(Qca7000CheckAlive, CpuOff) {
    mock_signature = 0xAA55;
    mock_wrbuf = 0x0C5B;
    mock_intr_cause = 0;
    EXPECT_FALSE(qca7000CheckAlive());
}
