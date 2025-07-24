#include <gtest/gtest.h>
#include <slac/channel.hpp>
#include <slac/slac.hpp>
#include <vector>
#include <cstring>

class MockLink : public slac::transport::Link {
public:
    bool open() override { return true; }
    bool write(const uint8_t* b, size_t l, uint32_t) override {
        tx.assign(b, b + l);
        rx = tx;
        return true;
    }
    slac::transport::LinkError read(uint8_t* b, size_t l, size_t* out, uint32_t) override {
        if (rx.empty()) {
            if (out)
                *out = 0;
            return slac::transport::LinkError::Timeout;
        }
        size_t copy = std::min(l, rx.size());
        memcpy(b, rx.data(), copy);
        rx.clear();
        if (out)
            *out = copy;
        return slac::transport::LinkError::Ok;
    }
    const uint8_t* mac() const override { return MAC; }
    static inline uint8_t MAC[6] = {0x02,0,0,0,0,1};
    std::vector<uint8_t> tx;
    std::vector<uint8_t> rx;
};

TEST(Channel, RoundTrip) {
    MockLink link;
    slac::Channel channel(&link);
    ASSERT_TRUE(channel.open());

    slac::messages::HomeplugMessage msg;
    slac::messages::cm_slac_parm_req req{};
    req.application_type = 0;
    req.security_type = 0;
    memset(req.run_id, 0, sizeof(req.run_id));

    ASSERT_TRUE(msg.setup_payload(&req, sizeof(req),
                                  slac::defs::MMTYPE_CM_SLAC_PARAM | slac::defs::MMTYPE_MODE_REQ,
                                  slac::defs::MMV::AV_1_0));
    uint8_t dst[ETH_ALEN] = {0xff,0xff,0xff,0xff,0xff,0xff};
    msg.setup_ethernet_header(dst);

    ASSERT_TRUE(channel.write(msg, 100));

    slac::messages::HomeplugMessage in;
    auto err = channel.read(in, 100);
    ASSERT_EQ(err, slac::transport::LinkError::Ok);
    EXPECT_EQ(in.get_mmtype(), msg.get_mmtype());
    EXPECT_EQ(in.get_raw_msg_len(), msg.get_raw_msg_len());
    EXPECT_EQ(0, memcmp(in.get_raw_message_ptr(), msg.get_raw_message_ptr(), msg.get_raw_msg_len()));
}

TEST(Channel, WriteAfterSetupFailure) {
    MockLink link;
    slac::Channel channel(&link);
    ASSERT_TRUE(channel.open());

    slac::messages::HomeplugMessage msg;
    slac::messages::cm_slac_parm_req req{};
    req.application_type = 0;
    req.security_type = 0;
    memset(req.run_id, 0, sizeof(req.run_id));

    ASSERT_TRUE(msg.setup_payload(&req, sizeof(req),
                                  slac::defs::MMTYPE_CM_SLAC_PARAM | slac::defs::MMTYPE_MODE_REQ,
                                  slac::defs::MMV::AV_1_0));
    uint8_t dst[ETH_ALEN] = {0xff,0xff,0xff,0xff,0xff,0xff};
    msg.setup_ethernet_header(dst);

    ASSERT_TRUE(channel.write(msg, 100));

    const size_t big_len = sizeof(slac::messages::homeplug_message::payload) + 1;
    std::vector<uint8_t> big(big_len, 0);
    EXPECT_FALSE(msg.setup_payload(big.data(), big.size(),
                                   slac::defs::MMTYPE_CM_SLAC_PARAM | slac::defs::MMTYPE_MODE_REQ,
                                   slac::defs::MMV::AV_1_0));
    EXPECT_FALSE(msg.is_valid());
    msg.setup_ethernet_header(dst);
    EXPECT_FALSE(channel.write(msg, 100));
}
