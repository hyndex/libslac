#include <gtest/gtest.h>
#define ARDUINO
#include "arduino_stubs.hpp"
#include "qca7000_link.hpp"
#include "qca7000.hpp"

extern "C" void mock_receive_frame(const uint8_t*, size_t);

using slac::port::Qca7000Link;

TEST(Qca7000LinkIntegration, BasicReadWrite) {
    qca7000_config cfg{};
    cfg.spi = &SPI;
    cfg.cs_pin = 5;
    cfg.rst_pin = 6;
    Qca7000Link link(cfg);
    ASSERT_TRUE(link.open());

    uint8_t frame[6] = {1,2,3,4,5,6};
    EXPECT_TRUE(link.write(frame, sizeof(frame), 0));

    mock_receive_frame(frame, sizeof(frame));

    uint8_t buf[10];
    size_t out = 0;
    auto err = link.read(buf, sizeof(buf), &out, 10);
    EXPECT_EQ(err, slac::transport::LinkError::Ok);
    EXPECT_EQ(out, sizeof(frame));
    EXPECT_EQ(0, memcmp(buf, frame, sizeof(frame)));

    link.close();
}
