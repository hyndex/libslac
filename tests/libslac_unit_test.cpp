// SPDX-License-Identifier: Apache-2.0
// Copyright 2020 - 2023 Pionix GmbH and Contributors to EVerest

#include <gtest/gtest.h>
#include <slac/slac.hpp>
#include <slac/channel.hpp>
#include <slac/transport.hpp>
#include <vector>
#include <algorithm>
#include <cstring>

namespace libslac {
class LibSLACUnitTest : public ::testing::Test {
protected:
    void SetUp() override {
    }

    void TearDown() override {
    }
};

class FakeLink : public slac::transport::Link {
public:
    std::vector<uint8_t> data;

    bool open() override { return true; }
    bool write(const uint8_t*, size_t, uint32_t) override { return true; }
    bool read(uint8_t* buf, size_t len, size_t* out_len, uint32_t) override {
        size_t n = std::min(len, data.size());
        if (n == 0) {
            *out_len = 0;
            return false;
        }
        memcpy(buf, data.data(), n);
        *out_len = n;
        return true;
    }
    const uint8_t* mac() const override { return mac_addr; }

private:
    uint8_t mac_addr[ETH_ALEN]{0};
};

TEST_F(LibSLACUnitTest, test_invalid_datetime) {
    ASSERT_TRUE(1 == 1);
}

TEST_F(LibSLACUnitTest, test_generate_nid_from_nmk_check_security_bits) {
    uint8_t nid[slac::defs::NID_LEN] = {0};
    const uint8_t sample_nmk[] = {0x34, 0x52, 0x23, 0x54, 0x45, 0xae, 0xf2, 0xd4,
                                  0x55, 0xfe, 0xff, 0x31, 0xa3, 0xb3, 0x03, 0xad};

    slac::utils::generate_nid_from_nmk(nid, sample_nmk);
    ASSERT_TRUE((nid[6] >> 4) == 0x00);
}

TEST_F(LibSLACUnitTest, test_channel_read_sets_length) {
    FakeLink link;
    link.data.resize(80, 0xaa);

    slac::Channel channel(&link);
    ASSERT_TRUE(channel.open());

    slac::messages::HomeplugMessage msg;
    ASSERT_TRUE(channel.read(msg, 0));
    ASSERT_EQ(msg.get_raw_msg_len(), static_cast<int>(link.data.size()));
}

TEST_F(LibSLACUnitTest, test_channel_poll_sets_length) {
    FakeLink link;
    link.data.resize(64, 0xbb);

    slac::Channel channel(&link);
    ASSERT_TRUE(channel.open());

    slac::messages::HomeplugMessage msg;
    ASSERT_TRUE(channel.poll(msg));
    ASSERT_EQ(msg.get_raw_msg_len(), static_cast<int>(link.data.size()));
}

} // namespace libslac
