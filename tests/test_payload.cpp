#include <gtest/gtest.h>
#include <slac/slac.hpp>
#include <cstring>

using namespace slac;

TEST(Payload, BuildAV10) {
    messages::HomeplugMessage msg;
    uint8_t payload[10];
    for (int i = 0; i < 10; ++i)
        payload[i] = i;

    ASSERT_TRUE(msg.setup_payload(payload, sizeof(payload), 0x1234, defs::MMV::AV_1_0));
    EXPECT_TRUE(msg.is_valid());
    EXPECT_EQ(msg.get_mmtype(), 0x1234);
    EXPECT_EQ(msg.get_raw_msg_len(), defs::MME_MIN_LENGTH);

    auto raw = msg.get_raw_message_ptr();
    EXPECT_EQ(raw->homeplug_header.mmv, static_cast<uint8_t>(defs::MMV::AV_1_0));
    EXPECT_EQ(memcmp(raw->payload, payload, sizeof(payload)), 0);
}

TEST(Payload, BuildAV20Frag) {
    messages::HomeplugMessage msg;
    uint8_t payload[8];
    for (int i = 0; i < 8; ++i)
        payload[i] = i + 1;

    ASSERT_TRUE(msg.setup_payload(payload, sizeof(payload), 0x4321, defs::MMV::AV_2_0));
    auto raw = msg.get_raw_message_ptr();
    EXPECT_EQ(raw->homeplug_header.mmv, static_cast<uint8_t>(defs::MMV::AV_2_0));
    const messages::homeplug_fragmentation_part* frag =
        reinterpret_cast<const messages::homeplug_fragmentation_part*>(raw->payload);
    EXPECT_EQ(frag->fmni, 0);
    EXPECT_EQ(frag->fmsn, 0);
    EXPECT_EQ(memcmp(raw->payload + sizeof(*frag), payload, sizeof(payload)), 0);
}

TEST(Payload, TooLong) {
    messages::HomeplugMessage msg;
    const size_t big_len = sizeof(messages::homeplug_message::payload) + 1;
    std::vector<uint8_t> big(big_len, 0);
    EXPECT_FALSE(msg.setup_payload(big.data(), big.size(), 0x1000, defs::MMV::AV_1_0));
}
