#include <Arduino.h>
#include <slac/channel.hpp>
#include <port/esp32s3/qca7000_link.hpp>

static const uint8_t MY_MAC[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
static slac::Channel* g_channel = nullptr;

void setup() {
    Serial.begin(115200);
    SPI.begin(48 /*SCK*/, 21 /*MISO*/, 47 /*MOSI*/, PLC_SPI_CS_PIN);
    qca7000_config cfg{&SPI, PLC_SPI_CS_PIN, PLC_SPI_RST_PIN, MY_MAC};
    static slac::port::Qca7000Link link(cfg);
    static slac::Channel channel(&link);
    g_channel = &channel;
    channel.open();
}

void loop() {
    qca7000Process();
    slac::messages::HomeplugMessage msg;
    if (g_channel && g_channel->poll(msg)) {
        // handle incoming SLAC messages
    }
    delay(1);
}
