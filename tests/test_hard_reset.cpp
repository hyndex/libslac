#include <gtest/gtest.h>
#define ARDUINO
#include "arduino_stubs.hpp"
#include "port/esp32s3/qca7000.hpp"

extern uint16_t mock_spi_config;
extern bool reset_called;

TEST(Qca7000HardReset, ClearsMultiCs) {
    mock_spi_config = QCASPI_MULTI_CS_BIT;
    reset_called = false;
    ASSERT_TRUE(qca7000ResetAndCheck());
    EXPECT_TRUE(reset_called);
    EXPECT_EQ(0, mock_spi_config & QCASPI_MULTI_CS_BIT);
}
