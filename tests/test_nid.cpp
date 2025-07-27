#include <gtest/gtest.h>
#include <slac/slac.hpp>
#include <cstring>

using namespace slac;

TEST(NID, GenerateKnownValue) {
    uint8_t nmk[defs::NMK_LEN];
    for (int i = 0; i < defs::NMK_LEN; ++i)
        nmk[i] = i; // 0x00,0x01,...0x0f
    uint8_t nid[defs::NID_LEN]{};
    utils::generate_nid_from_nmk(nid, nmk, defs::NID_SECURITY_LEVEL_SIMPLE_CONNECT);
    const uint8_t expected[] = {0x4d, 0x30, 0xa0, 0xf8, 0x45, 0x5d, 0x0b};
    ASSERT_EQ(memcmp(nid, expected, sizeof(expected)), 0);
}


