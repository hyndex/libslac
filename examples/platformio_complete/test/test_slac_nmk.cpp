#include <gtest/gtest.h>
#define ARDUINO
#include "arduino_stubs.hpp"
#include "qca7000.hpp"
#include <slac/slac.hpp>
#include <slac/config.hpp>
#include <slac/endian.hpp>
#include <cstring>

static uint8_t mock_state = 0;
static uint8_t current_nmk[slac::defs::NMK_LEN]{};

bool qca7000startSlac() {
    mock_state = 1;
    return true;
}

uint8_t qca7000getSlacResult() { return mock_state; }

void mock_ring_reset() { mock_state = 0; }

void mock_receive_frame(const uint8_t* f, size_t) {
    const uint8_t* p = f + sizeof(ether_header);
    uint16_t mmtype;
    memcpy(&mmtype, p + 1, 2);
    mmtype = slac::le16toh(mmtype);
    switch (mock_state) {
    case 1:
        if (mmtype == (slac::defs::MMTYPE_CM_SLAC_PARAM | slac::defs::MMTYPE_MODE_CNF))
            mock_state = 2;
        break;
    case 2:
        if (mmtype == (slac::defs::MMTYPE_CM_ATTEN_CHAR | slac::defs::MMTYPE_MODE_IND))
            mock_state = 3;
        break;
    case 3:
        if (mmtype == (slac::defs::MMTYPE_CM_SET_KEY | slac::defs::MMTYPE_MODE_REQ))
            mock_state = 4;
        break;
    case 4:
        if (mmtype == (slac::defs::MMTYPE_CM_VALIDATE | slac::defs::MMTYPE_MODE_REQ))
            mock_state = 5;
        break;
    case 5:
        if (mmtype == (slac::defs::MMTYPE_CM_SLAC_MATCH | slac::defs::MMTYPE_MODE_REQ))
            mock_state = 6;
        break;
    default:
        break;
    }
}

void qca7000SetNmk(const uint8_t nmk[slac::defs::NMK_LEN]) {
    if (nmk)
        memcpy(current_nmk, nmk, slac::defs::NMK_LEN);
    else
        memset(current_nmk, 0, slac::defs::NMK_LEN);
}

const uint8_t* qca7000GetNmk() { return current_nmk; }

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

static void run_match_sequence(const uint8_t mac[ETH_ALEN]) {
    send_frame(slac::defs::MMTYPE_CM_SLAC_PARAM | slac::defs::MMTYPE_MODE_CNF, mac);
    ASSERT_EQ(qca7000getSlacResult(), 2);
    send_frame(slac::defs::MMTYPE_CM_ATTEN_CHAR | slac::defs::MMTYPE_MODE_IND, mac);
    ASSERT_EQ(qca7000getSlacResult(), 3);
    send_frame(slac::defs::MMTYPE_CM_SET_KEY | slac::defs::MMTYPE_MODE_REQ, mac);
    ASSERT_EQ(qca7000getSlacResult(), 4);
    send_frame(slac::defs::MMTYPE_CM_VALIDATE | slac::defs::MMTYPE_MODE_REQ, mac);
    ASSERT_EQ(qca7000getSlacResult(), 5);
    send_frame(slac::defs::MMTYPE_CM_SLAC_MATCH | slac::defs::MMTYPE_MODE_REQ, mac);
    ASSERT_EQ(qca7000getSlacResult(), 6);
}

TEST(SlacNmk, DefaultThenEvse) {
    const uint8_t pev_mac[ETH_ALEN] = {0,1,2,3,4,5};
    const uint8_t evse_nmk[slac::defs::NMK_LEN] = {
        0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
        0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};
    slac::set_validation_disabled(true);
    qca7000SetNmk(nullptr);
    mock_ring_reset();
    ASSERT_TRUE(qca7000startSlac());
    EXPECT_EQ(qca7000getSlacResult(), 1);
    const uint8_t* cur = qca7000GetNmk();
    for (size_t i = 0; i < slac::defs::NMK_LEN; ++i)
        EXPECT_EQ(cur[i], 0u);
    run_match_sequence(pev_mac);
    EXPECT_EQ(qca7000getSlacResult(), 6);
    cur = qca7000GetNmk();
    for (size_t i = 0; i < slac::defs::NMK_LEN; ++i)
        EXPECT_EQ(cur[i], 0u);
    qca7000SetNmk(evse_nmk);
    cur = qca7000GetNmk();
    for (size_t i = 0; i < slac::defs::NMK_LEN; ++i)
        EXPECT_EQ(cur[i], evse_nmk[i]);
}

