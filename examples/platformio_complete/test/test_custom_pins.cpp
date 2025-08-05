#include <gtest/gtest.h>
#include "qca7000_link.hpp"
#include "qca7000.hpp"

static spi_device_handle_t last_spi = nullptr;
static int last_cs = -1;
static int last_rst = -1;
static int last_int = -1;
static int last_pwr = -1;

bool qca7000setup(spi_device_handle_t spi,
                  int cs,
                  int rst,
                  int intr,
                  int pwr) {
    last_spi = spi;
    last_cs = cs;
    last_rst = rst;
    last_int = intr;
    last_pwr = pwr;
    return true;
}

void qca7000teardown() {}

void qca7000SetMac(const uint8_t*) {}
void qca7000SetErrorCallback(qca7000_error_cb_t, void*, bool*) {}
bool spiQCA7000SendEthFrame(const uint8_t*, size_t) { return true; }
size_t spiQCA7000checkForReceivedData(uint8_t*, size_t) { return 0; }

TEST(Qca7000Pins, CustomValuesForwarded) {
    spi_device_handle_t spi = reinterpret_cast<spi_device_handle_t>(0x1);
    qca7000_config cfg{spi, 2, 3, 4, 5, nullptr};
    slac::port::Qca7000Link link(cfg);
    ASSERT_TRUE(link.open());
    EXPECT_EQ(last_int, 4);
    EXPECT_EQ(last_pwr, 5);
}
