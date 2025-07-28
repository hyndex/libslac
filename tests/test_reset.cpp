#include <gtest/gtest.h>
#include "arduino_stubs.hpp"
#include "port/esp32s3/qca7000.hpp"

TEST(Qca7000ResetSimple, ReturnsTrue) {
    ASSERT_TRUE(qca7000ResetAndCheck());
}
