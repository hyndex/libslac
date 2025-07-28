#include <gtest/gtest.h>
#include "arduino_stubs.hpp"
#include "port/esp32s3/qca7000.hpp"
#include <slac/iso15118_consts.hpp>

extern uint32_t g_mock_millis;
static int toggle_count = 0;
void qca7000ToggleCpEf() { ++toggle_count; }

TEST(SlacRetry, ParmCnfTimeout) {
    g_mock_millis = 0;
    toggle_count = 0;
    ASSERT_TRUE(qca7000startSlac());
    EXPECT_EQ(qca7000getSlacResult(), 1);

    g_mock_millis += slac::defs::TT_EVSE_SLAC_INIT_MS + 1;
    EXPECT_EQ(qca7000getSlacResult(), 1);
    EXPECT_EQ(toggle_count, 1);

    g_mock_millis += slac::defs::TT_EVSE_SLAC_INIT_MS + 1;
    EXPECT_EQ(qca7000getSlacResult(), 0xFF);
    EXPECT_EQ(toggle_count, 2);
}
