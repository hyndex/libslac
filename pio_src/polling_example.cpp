#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <slac/channel.hpp>
#include <port/esp32s3/qca7000.hpp>
#include <port/esp32s3/qca7000_link.hpp>

#ifdef ARDUINO
static slac::Channel* g_channel = nullptr;

void setup() {
    static const uint8_t my_mac[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
    qca7000_config cfg{&SPI, PLC_SPI_CS_PIN, my_mac};
    static slac::port::Qca7000Link link(cfg);
    static slac::Channel channel(&link);
    g_channel = &channel;
    channel.open();
}

void loop() {
    qca7000Process();
    slac::messages::HomeplugMessage msg;
    if (g_channel && g_channel->poll(msg)) {
        // handle incoming packet
    }
    delay(1);
}
#endif
