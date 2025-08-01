#include <gtest/gtest.h>
#include <slac/endian.hpp>

TEST(Endian, Swap) {
    EXPECT_EQ(slac::bswap16(0x1234u), 0x3412u);
    EXPECT_EQ(slac::bswap32(0x11223344u), 0x44332211u);
    EXPECT_EQ(slac::bswap64(0x1122334455667788ull), 0x8877665544332211ull);
}

TEST(Endian, HostLE) {
    uint16_t v16 = 0xA1B2u;
    uint32_t v32 = 0xA1B2C3D4u;
    uint64_t v64 = 0x1122334455667788ull;
    EXPECT_EQ(slac::htole16(v16), v16);
    EXPECT_EQ(slac::le16toh(v16), v16);
    EXPECT_EQ(slac::htole32(v32), v32);
    EXPECT_EQ(slac::le32toh(v32), v32);
    EXPECT_EQ(slac::htole64(v64), v64);
    EXPECT_EQ(slac::le64toh(v64), v64);
}
