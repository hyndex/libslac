#include <Arduino.h>
#include <slac/channel.hpp>
#include <port/esp32s3/qca7000_link.hpp>

// Default MAC address for the modem. Adjust as required.
static const uint8_t MY_MAC[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

// Global pointer used by the polling loop
static slac::Channel* g_channel = nullptr;

void setup() {
    Serial.begin(115200);

    // Initialise the SPI bus with custom chip select pin.
    // PLC_SPI_CS_PIN and PLC_SPI_RST_PIN can be overridden via
    // build flags in platformio.ini to match your wiring.
    SPI.begin(48 /*SCK*/, 21 /*MISO*/, 47 /*MOSI*/, PLC_SPI_CS_PIN);
    qca7000_config cfg{&SPI, PLC_SPI_CS_PIN, PLC_SPI_RST_PIN, MY_MAC};

    static slac::port::Qca7000Link link(cfg);
    static slac::Channel channel(&link);
    g_channel = &channel;
    if (!channel.open()) {
        Serial.println("Failed to open SLAC channel, aborting");
        g_channel = nullptr;
        while (true)
            delay(1000);
    }
}

void loop() {
    // Poll the modem even when the IRQ line is not connected.
    qca7000Process();

    slac::messages::HomeplugMessage msg;
    if (g_channel && g_channel->poll(msg)) {
        // Handle incoming SLAC messages here
    }
    delay(1);
}
