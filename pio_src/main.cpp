#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <slac/channel.hpp>
#include <port/esp32s3/qca7000_link.hpp>

#ifdef ARDUINO
static slac::port::Qca7000Link* g_link = nullptr;
static slac::Channel* channel = nullptr;

void setup() {
    static const uint8_t my_mac[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
    qca7000_config cfg{&SPI, PLC_SPI_CS_PIN, my_mac};
    g_link = new slac::port::Qca7000Link(cfg);
    channel = new slac::Channel(g_link);
    channel->open();
}

void loop() {
    // placeholder main loop
}
#endif
