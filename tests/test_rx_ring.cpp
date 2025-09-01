#include <gtest/gtest.h>
#include "arduino_stubs.hpp"
#include "qca7000.hpp"

extern "C" void mock_ring_reset();
extern "C" void mock_receive_frame(const uint8_t*, size_t);

TEST(Qca7000RxRing, StoresTwoFrames) {
    mock_ring_reset();
    uint8_t f1[3] = {1,2,3};
    uint8_t f2[4] = {4,5,6,7};
    mock_receive_frame(f1, sizeof(f1));
    mock_receive_frame(f2, sizeof(f2));

    uint8_t buf[10];
    size_t got;

    got = spiQCA7000checkForReceivedData(buf, sizeof(buf));
    ASSERT_EQ(got, sizeof(f1));
    EXPECT_EQ(0, memcmp(buf, f1, sizeof(f1)));

    got = spiQCA7000checkForReceivedData(buf, sizeof(buf));
    ASSERT_EQ(got, sizeof(f2));
    EXPECT_EQ(0, memcmp(buf, f2, sizeof(f2)));

    EXPECT_EQ(0u, spiQCA7000checkForReceivedData(buf, sizeof(buf)));
}
