#include <gtest/gtest.h>

#include "../port/esp32s3/qca7000_uart.hpp"
#include "stubs/HardwareSerial.h"
#include <endian.h>

static constexpr uint16_t TX_HDR = 8;
static constexpr uint16_t FTR_LEN = 2;

extern HardwareSerial* g_serial;
extern bool uartTxFrame(const uint8_t* frame, size_t len);
extern void uartFetchRx();
extern size_t uartQCA7000checkForReceivedData(uint8_t* dst, size_t len);

class QCA7000UartTest : public ::testing::Test {
protected:
    void SetUp() override {
        g_serial = &Serial;
        Serial.read_queue.clear();
        Serial.written.clear();
    }
};

TEST_F(QCA7000UartTest, txFramePaddingAndMarkers) {
    uint8_t frame[4] = {1, 2, 3, 4};
    bool ok = uartTxFrame(frame, sizeof(frame));
    ASSERT_TRUE(ok);
    ASSERT_EQ(Serial.written.size(), TX_HDR + 60 + FTR_LEN);
    EXPECT_EQ(Serial.written[0], 0xAA);
    EXPECT_EQ(Serial.written[1], 0xAA);
    EXPECT_EQ(Serial.written[2], 0xAA);
    EXPECT_EQ(Serial.written[3], 0xAA);
    EXPECT_EQ(*reinterpret_cast<uint16_t*>(&Serial.written[4]), htole16(60));
    EXPECT_EQ(Serial.written[TX_HDR + 60], 0x55);
    EXPECT_EQ(Serial.written[TX_HDR + 61], 0x55);
}

TEST_F(QCA7000UartTest, fetchRxExtractsFrame) {
    const uint16_t frame_len = 20;
    Serial.read_queue.clear();
    for (int i = 0; i < 4; ++i)
        Serial.read_queue.push_back(0xAA);
    Serial.read_queue.push_back(frame_len & 0xFF);
    Serial.read_queue.push_back((frame_len >> 8) & 0xFF);
    Serial.read_queue.push_back(0);
    Serial.read_queue.push_back(0);
    for (int i = 0; i < frame_len; ++i)
        Serial.read_queue.push_back(static_cast<uint8_t>(i + 1));
    Serial.read_queue.push_back(0x55);
    Serial.read_queue.push_back(0x55);

    uartFetchRx();
    uint8_t out[64] = {0};
    size_t got = uartQCA7000checkForReceivedData(out, sizeof(out));
    ASSERT_EQ(got, frame_len);
    for (int i = 0; i < frame_len; ++i)
        EXPECT_EQ(out[i], static_cast<uint8_t>(i + 1));
}
