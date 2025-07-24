#include <gtest/gtest.h>
#include <hash_library/sha256.h>

TEST(SHA256, KnownVectors) {
    SHA256 sha;
    EXPECT_EQ(sha("abc"),
              std::string("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"));
    EXPECT_EQ(sha(""),
              std::string("e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"));
}
