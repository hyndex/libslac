#include <gtest/gtest.h>
#define ARDUINO
#include "arduino_stubs.hpp"
#include "port/esp32s3/qca7000.hpp"
#include <slac/slac.hpp>
#include <slac/config.hpp>
#include <arpa/inet.h>

extern "C" void mock_receive_frame(const uint8_t*, size_t);
extern "C" void mock_ring_reset();

static void send_frame(uint16_t mmtype, const uint8_t src[ETH_ALEN]) {
    uint8_t frame[sizeof(ether_header) + 3]{};
    ether_header* eth = reinterpret_cast<ether_header*>(frame);
    memcpy(eth->ether_shost, src, ETH_ALEN);
    eth->ether_type = htons(slac::defs::ETH_P_HOMEPLUG_GREENPHY);
    frame[sizeof(ether_header)] = static_cast<uint8_t>(slac::defs::MMV::AV_1_0);
    uint16_t t = slac::htole16(mmtype);
    memcpy(frame + sizeof(ether_header) + 1, &t, 2);
    mock_receive_frame(frame, sizeof(frame));
}

static void run_match_sequence(const uint8_t mac[ETH_ALEN]) {
    send_frame(slac::defs::MMTYPE_CM_SLAC_PARAM | slac::defs::MMTYPE_MODE_CNF, mac);
    EXPECT_EQ(qca7000getSlacResult(), 2);
    send_frame(slac::defs::MMTYPE_CM_ATTEN_CHAR | slac::defs::MMTYPE_MODE_IND, mac);
    EXPECT_EQ(qca7000getSlacResult(), 3);
    send_frame(slac::defs::MMTYPE_CM_SET_KEY | slac::defs::MMTYPE_MODE_REQ, mac);
    EXPECT_EQ(qca7000getSlacResult(), 4);
    send_frame(slac::defs::MMTYPE_CM_VALIDATE | slac::defs::MMTYPE_MODE_REQ, mac);
    EXPECT_EQ(qca7000getSlacResult(), 5);
    send_frame(slac::defs::MMTYPE_CM_SLAC_MATCH | slac::defs::MMTYPE_MODE_REQ, mac);
    EXPECT_EQ(qca7000getSlacResult(), 6);
}

TEST(SlacFilter, IgnoreOtherMac) {
    const uint8_t pev1[ETH_ALEN] = {0,1,2,3,4,5};
    const uint8_t pev2[ETH_ALEN] = {6,7,8,9,10,11};
    slac::set_validation_disabled(true);
    mock_ring_reset();
    ASSERT_TRUE(qca7000startSlac());
    EXPECT_EQ(qca7000getSlacResult(), 1);
    run_match_sequence(pev1);
    send_frame(slac::defs::MMTYPE_CM_SLAC_PARAM | slac::defs::MMTYPE_MODE_CNF, pev2);
    EXPECT_EQ(qca7000getSlacResult(), 6);
}

TEST(SlacFilter, StartClearsFilter) {
    const uint8_t pev1[ETH_ALEN] = {0,1,2,3,4,5};
    const uint8_t pev2[ETH_ALEN] = {6,7,8,9,10,11};
    slac::set_validation_disabled(true);
    mock_ring_reset();
    ASSERT_TRUE(qca7000startSlac());
    EXPECT_EQ(qca7000getSlacResult(), 1);
    run_match_sequence(pev1);
    ASSERT_TRUE(qca7000startSlac());
    EXPECT_EQ(qca7000getSlacResult(), 1);
    send_frame(slac::defs::MMTYPE_CM_SLAC_PARAM | slac::defs::MMTYPE_MODE_CNF, pev2);
    EXPECT_EQ(qca7000getSlacResult(), 2);
}
