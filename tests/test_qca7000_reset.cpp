#include <gtest/gtest.h>
#define ARDUINO
#include "arduino_stubs.hpp"
#include "port/esp32s3/qca7000.hpp"

extern bool reset_called;

TEST(Qca7000Hal, ResetAndCheck) {
    reset_called = false;
    ASSERT_TRUE(qca7000ResetAndCheck());
    EXPECT_TRUE(reset_called);
}
