#include <gtest/gtest.h>
#include "arduino_stubs.hpp"
#include "qca7000.hpp"

extern bool reset_called;
extern uint16_t mock_signature;
extern uint16_t mock_wrbuf;
extern uint16_t mock_intr_cause;

TEST(Qca7000Hal, ResetAndCheck) {
    reset_called = false;
    mock_signature = 0xAA55;
    mock_wrbuf = 0x0C5B;
    mock_intr_cause = SPI_INT_CPU_ON;
    ASSERT_TRUE(qca7000ResetAndCheck());
    EXPECT_TRUE(reset_called);
}
