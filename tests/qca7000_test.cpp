#include <gtest/gtest.h>

#define LIBSLAC_TESTING
#define ARDUINO
#define SLAC_ETHERNET_DEFS_HPP
#include "../port/esp32s3/qca7000.cpp"
#include "stubs/SPI.h"
#include <endian.h>
#include <net/ethernet.h>

class QCA7000Test : public ::testing::Test {
protected:
    void SetUp() override {
        g_spi = &SPI;
        g_cs = 5;
        SPI.read_queue.clear();
        SPI.read16_queue.clear();
        SPI.written.clear();
        SPI.transferred.clear();
        SPI.transferred16.clear();
    }
};

TEST_F(QCA7000Test, txFramePaddingAndMarkers) {
    uint8_t frame[4] = {1, 2, 3, 4};
    SPI.read16_queue.push_back(0);    // dummy for first transfer16
    SPI.read16_queue.push_back(1000); // space available for second transfer16
    bool ok = txFrame(frame, sizeof(frame));
    ASSERT_TRUE(ok);
    ASSERT_GE(SPI.transferred16.size(), 6u);
    size_t off = SPI.transferred16.size() - 6;
    EXPECT_EQ(SPI.transferred16[off + 1], 0xAAAA);      // SOF
    EXPECT_EQ(SPI.transferred16[off + 2], 0xAAAA);      // SOF
    EXPECT_EQ(SPI.transferred16[off + 3], htole16(60)); // length
    EXPECT_EQ(SPI.transferred16[off + 5], 0x5555);      // EOF
    ASSERT_EQ(SPI.written.size(), 60u);
    EXPECT_TRUE(std::equal(frame, frame + sizeof(frame), SPI.written.begin()));
}

TEST_F(QCA7000Test, fetchRxExtractsFrame) {
    const uint16_t frame_len = 14;
    const uint16_t avail = RX_HDR + frame_len + FTR_LEN;
    SPI.read16_queue.push_back(0);     // dummy first transfer16
    SPI.read16_queue.push_back(avail); // avail from spiRd16_fast
    std::vector<uint8_t> data(avail + 2, 0);
    uint8_t* p = data.data() + 2;
    uint32_t len = avail;
    p[0] = len & 0xFF;
    p[1] = (len >> 8) & 0xFF;
    p[2] = (len >> 16) & 0xFF;
    p[3] = (len >> 24) & 0xFF;
    p[4] = 0xAA;
    p[5] = 0xAA;
    p[6] = 0xAA;
    p[7] = 0xAA;
    p[8] = frame_len & 0xFF;
    p[9] = (frame_len >> 8) & 0xFF;
    for (int i = 0; i < frame_len; ++i) {
        p[RX_HDR + i] = static_cast<uint8_t>(i + 1);
    }
    p[RX_HDR + frame_len] = 0x55;
    p[RX_HDR + frame_len + 1] = 0x55;
    for (uint8_t b : data)
        SPI.read_queue.push_back(b);

    fetchRx();
    uint8_t out[64] = {0};
    size_t got = spiQCA7000checkForReceivedData(out, sizeof(out));
    ASSERT_EQ(got, frame_len);
    for (int i = 0; i < frame_len; ++i) {
        EXPECT_EQ(out[i], static_cast<uint8_t>(i + 1));
    }
}
