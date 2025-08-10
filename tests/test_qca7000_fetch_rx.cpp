#include <gtest/gtest.h>
#define ARDUINO
#include "arduino_stubs.hpp"
#include "qca7000.hpp"

extern "C" void mock_ring_reset();
extern "C" void mock_spi_feed_raw(const uint8_t*, size_t);
extern bool soft_reset_called;
esp_err_t fetchRx();

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

TEST(Qca7000FetchRx, ConcatenatedFrames) {
    mock_ring_reset();
    soft_reset_called = false;

    uint8_t eth1[6] = {1,2,3,4,5,6};
    uint8_t eth2[3] = {7,8,9};

    const uint16_t fl1 = sizeof(eth1);
    const uint16_t fl2 = sizeof(eth2);
    const uint16_t total1 = 12 + fl1 + 2;
    const uint16_t total2 = 12 + fl2 + 2;

    uint8_t raw[64] = {};

    // First frame
    raw[0] = total1 & 0xFF; raw[1] = total1 >> 8;
    raw[2] = 0; raw[3] = 0;
    raw[4] = 0xAA; raw[5] = 0xAA; raw[6] = 0xAA; raw[7] = 0xAA;
    raw[8] = fl1 & 0xFF; raw[9] = fl1 >> 8;
    raw[10] = 0; raw[11] = 0;
    memcpy(raw + 12, eth1, fl1);
    raw[12 + fl1] = 0x55; raw[13 + fl1] = 0x55;

    // Second frame concatenated after first
    size_t off = total1;
    raw[off + 0] = total2 & 0xFF; raw[off + 1] = total2 >> 8;
    raw[off + 2] = 0; raw[off + 3] = 0;
    raw[off + 4] = 0xAA; raw[off + 5] = 0xAA; raw[off + 6] = 0xAA; raw[off + 7] = 0xAA;
    raw[off + 8] = fl2 & 0xFF; raw[off + 9] = fl2 >> 8;
    raw[off +10] = 0; raw[off +11] = 0;
    memcpy(raw + off + 12, eth2, fl2);
    raw[off + 12 + fl2] = 0x55; raw[off + 13 + fl2] = 0x55;

    mock_spi_feed_raw(raw, total1 + total2);
    fetchRx();

    uint8_t buf[16];
    size_t got = spiQCA7000checkForReceivedData(buf, sizeof(buf));
    ASSERT_EQ(got, sizeof(eth1));
    EXPECT_EQ(0, memcmp(buf, eth1, sizeof(eth1)));

    got = spiQCA7000checkForReceivedData(buf, sizeof(buf));
    ASSERT_EQ(got, sizeof(eth2));
    EXPECT_EQ(0, memcmp(buf, eth2, sizeof(eth2)));

    EXPECT_FALSE(soft_reset_called);
}
