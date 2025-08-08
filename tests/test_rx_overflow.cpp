#include <gtest/gtest.h>
#define ARDUINO
#include "arduino_stubs.hpp"
#include "qca7000.hpp"

extern "C" void mock_ring_reset();
extern "C" void mock_receive_frame(const uint8_t*, size_t);

TEST(Qca7000RxRing, OverflowCounter) {
    mock_ring_reset();
    EXPECT_EQ(0u, qca7000GetRxOverflowCount());

    uint8_t f[1] = {0};
    for (int i = 0; i < CONFIG_RX_RING_SIZE; ++i) {
        f[0] = static_cast<uint8_t>(i);
        mock_receive_frame(f, sizeof(f));
    }

    EXPECT_EQ(1u, qca7000GetRxOverflowCount());
}
