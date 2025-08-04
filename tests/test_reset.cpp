#include <gtest/gtest.h>
#define ARDUINO
#include "arduino_stubs.hpp"
#include "qca7000.hpp"

TEST(Qca7000ResetSimple, ReturnsTrue) {
    ASSERT_TRUE(qca7000ResetAndCheck());
}
