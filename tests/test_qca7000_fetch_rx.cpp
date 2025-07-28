#include <gtest/gtest.h>
#define ARDUINO
#include "arduino_stubs.hpp"
#include "port/esp32s3/qca7000.hpp"

extern "C" void mock_ring_reset();
extern "C" void mock_spi_feed_raw(const uint8_t*, size_t);
extern bool soft_reset_called;
void fetchRx();

TEST(Qca7000FetchRx, ValidFrameParsing) {
    mock_ring_reset();
    soft_reset_called = false;
    uint8_t eth[6] = {1,2,3,4,5,6};
    const uint16_t fl = sizeof(eth);
    const uint16_t total = 12 + fl + 2;
    uint8_t raw[32] = {};
    raw[0] = total & 0xFF;
    raw[1] = total >> 8;
    raw[2] = 0; raw[3] = 0;
    raw[4] = 0xAA; raw[5] = 0xAA; raw[6] = 0xAA; raw[7] = 0xAA;
    raw[8] = fl & 0xFF; raw[9] = fl >> 8;
    raw[10] = 0; raw[11] = 0;
    memcpy(raw + 12, eth, fl);
    raw[12 + fl] = 0x55;
    raw[13 + fl] = 0x55;
    mock_spi_feed_raw(raw, total);
    fetchRx();

    uint8_t buf[16];
    size_t got = spiQCA7000checkForReceivedData(buf, sizeof(buf));
    ASSERT_EQ(got, sizeof(eth));
    EXPECT_EQ(0, memcmp(buf, eth, sizeof(eth)));
    EXPECT_FALSE(soft_reset_called);
}

TEST(Qca7000FetchRx, LenExcludesHeader) {
    mock_ring_reset();
    soft_reset_called = false;
    uint8_t eth[6] = {1,2,3,4,5,6};
    const uint16_t fl = sizeof(eth);
    const uint16_t total = 12 + fl + 2;
    const uint16_t len_no_hdr = total - 4;
    uint8_t raw[32] = {};
    raw[0] = len_no_hdr & 0xFF;
    raw[1] = len_no_hdr >> 8;
    raw[2] = 0;
    raw[3] = 0;
    raw[4] = 0xAA; raw[5] = 0xAA; raw[6] = 0xAA; raw[7] = 0xAA;
    raw[8] = fl & 0xFF; raw[9] = fl >> 8;
    raw[10] = 0; raw[11] = 0;
    memcpy(raw + 12, eth, fl);
    raw[12 + fl] = 0x55;
    raw[13 + fl] = 0x55;
    mock_spi_feed_raw(raw, total);
    fetchRx();

    uint8_t buf[16];
    size_t got = spiQCA7000checkForReceivedData(buf, sizeof(buf));
    ASSERT_EQ(got, sizeof(eth));
    EXPECT_EQ(0, memcmp(buf, eth, sizeof(eth)));
    EXPECT_FALSE(soft_reset_called);
}

TEST(Qca7000FetchRx, FrameErrorTriggersReset) {
    mock_ring_reset();
    soft_reset_called = false;
    uint8_t eth[6] = {1,2,3,4,5,6};
    const uint16_t fl = sizeof(eth);
    const uint16_t total = 12 + fl + 2;
    uint8_t raw[32] = {};
    raw[0] = total & 0xFF;
    raw[1] = total >> 8;
    raw[2] = 0; raw[3] = 0;
    raw[4] = 0xAA; raw[5] = 0xAA; raw[6] = 0xAA; raw[7] = 0xAA;
    raw[8] = fl & 0xFF; raw[9] = fl >> 8;
    raw[10] = 0; raw[11] = 0;
    memcpy(raw + 12, eth, fl);
    raw[12 + fl] = 0x00; // corrupt EOF
    raw[13 + fl] = 0x00;
    mock_spi_feed_raw(raw, total);
    fetchRx();

    uint8_t buf[16];
    size_t got = spiQCA7000checkForReceivedData(buf, sizeof(buf));
    EXPECT_EQ(got, 0u);
    EXPECT_TRUE(soft_reset_called);
}
