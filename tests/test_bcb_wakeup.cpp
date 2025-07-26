#include <gtest/gtest.h>
#define ARDUINO
#include "arduino_stubs.hpp"
#include "port/esp32s3/qca7000.hpp"

extern bool mock_bcb_toggle;
extern bool wake_called;

static int ready_count = 0;
static void ready_cb(bool ready, void*) {
    if (ready)
        ++ready_count;
}

TEST(Qca7000Wake, ResumeOnBcbToggle) {
    ready_count = 0;
    wake_called = false;
    qca7000SetLinkReadyCallback(ready_cb, nullptr);
    ASSERT_TRUE(qca7000Sleep());
    mock_bcb_toggle = true;
    qca7000Process();
    EXPECT_TRUE(wake_called);
    EXPECT_EQ(ready_count, 1);
}
