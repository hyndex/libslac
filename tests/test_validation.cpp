#include <gtest/gtest.h>
#include "arduino_stubs.hpp"
#include "qca7000.hpp"
#include <slac/slac.hpp>
#include <slac/config.hpp>
#include <slac/endian.hpp>

extern "C" void mock_receive_frame(const uint8_t*, size_t);
extern "C" void mock_ring_reset();
extern bool mock_bcb_toggle;

static void send_frame(uint16_t mmtype, const uint8_t src[ETH_ALEN]) {
    uint8_t frame[sizeof(ether_header) + 3]{};
    ether_header* eth = reinterpret_cast<ether_header*>(frame);
    memcpy(eth->ether_shost, src, ETH_ALEN);
    eth->ether_type = slac::htons(slac::defs::ETH_P_HOMEPLUG_GREENPHY);
    frame[sizeof(ether_header)] = static_cast<uint8_t>(slac::defs::MMV::AV_1_0);
    uint16_t t = slac::htole16(mmtype);
    memcpy(frame + sizeof(ether_header) + 1, &t, 2);
    mock_receive_frame(frame, sizeof(frame));
}

static void goto_validate(const uint8_t mac[ETH_ALEN]) {
    send_frame(slac::defs::MMTYPE_CM_SLAC_PARAM | slac::defs::MMTYPE_MODE_CNF, mac);
    EXPECT_EQ(qca7000getSlacResult(), SlacState::Sounding);
    send_frame(slac::defs::MMTYPE_CM_ATTEN_CHAR | slac::defs::MMTYPE_MODE_IND, mac);
    EXPECT_EQ(qca7000getSlacResult(), SlacState::WaitSetKey);
    send_frame(slac::defs::MMTYPE_CM_SET_KEY | slac::defs::MMTYPE_MODE_REQ, mac);
    EXPECT_EQ(qca7000getSlacResult(), SlacState::WaitValidate);
}

TEST(Validation, Success) {
    const uint8_t mac[ETH_ALEN] = {0,1,2,3,4,5};
    slac::set_validation_disabled(false);
    mock_ring_reset();
    ASSERT_TRUE(qca7000startSlac());
    EXPECT_EQ(qca7000getSlacResult(), SlacState::WaitParmCnf);
    goto_validate(mac);
    mock_bcb_toggle = true;
    send_frame(slac::defs::MMTYPE_CM_VALIDATE | slac::defs::MMTYPE_MODE_REQ, mac);
    EXPECT_EQ(qca7000getSlacResult(), SlacState::WaitMatch);
}

TEST(Validation, Failure) {
    const uint8_t mac[ETH_ALEN] = {0,1,2,3,4,5};
    slac::set_validation_disabled(false);
    mock_ring_reset();
    ASSERT_TRUE(qca7000startSlac());
    EXPECT_EQ(qca7000getSlacResult(), SlacState::WaitParmCnf);
    goto_validate(mac);
    mock_bcb_toggle = false;
    send_frame(slac::defs::MMTYPE_CM_VALIDATE | slac::defs::MMTYPE_MODE_REQ, mac);
    EXPECT_EQ(qca7000getSlacResult(), SlacState::Failed);
}
