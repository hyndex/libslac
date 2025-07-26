#include <gtest/gtest.h>
#define ARDUINO
#include "arduino_stubs.hpp"
#include "port/esp32s3/qca7000.hpp"

extern "C" void mock_trigger_timeout();
extern "C" uint8_t mock_get_toggle_count();
extern "C" int mock_get_parm_req_count();

TEST(ParmRetry, Exhausted) {
    ASSERT_TRUE(qca7000startSlac());
    EXPECT_EQ(mock_get_parm_req_count(), 1);

    mock_trigger_timeout();
    EXPECT_EQ(mock_get_parm_req_count(), 2);
    EXPECT_EQ(mock_get_toggle_count(), 1);
    EXPECT_EQ(qca7000getSlacResult(), 1);

    mock_trigger_timeout();
    EXPECT_EQ(mock_get_parm_req_count(), 3);
    EXPECT_EQ(mock_get_toggle_count(), 2);
    EXPECT_EQ(qca7000getSlacResult(), 1);

    mock_trigger_timeout();
    EXPECT_EQ(qca7000getSlacResult(), 0xFF);
    EXPECT_EQ(mock_get_parm_req_count(), 3);
    EXPECT_EQ(mock_get_toggle_count(), 2);
}
