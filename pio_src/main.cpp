#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <slac/channel.hpp>
#include <port/esp32s3/qca7000_link.hpp>

#ifdef ARDUINO
void setup() {
    static const uint8_t my_mac[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
    qca7000_config cfg{&SPI, PLC_SPI_CS_PIN, my_mac};
    static slac::port::Qca7000Link link(cfg);
    static slac::Channel channel(&link);
    channel.open();
}

void loop() {
    // placeholder main loop
}
#endif
