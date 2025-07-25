#include <Arduino.h>
#include <slac/channel.hpp>
#include <slac/slac.hpp>
#include <port/esp32s3/qca7000_link.hpp>

// Default MAC address for the modem. Adjust as required.
static const uint8_t MY_MAC[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

// Global pointer used by the polling loop
static slac::Channel* g_channel = nullptr;

void setup() {
    Serial.begin(115200);
    delay(4000); // Wait for Serial to be ready
    // Initialise the SPI bus with custom chip select pin.
    // PLC_SPI_CS_PIN and PLC_SPI_RST_PIN can be overridden via
    // build flags in platformio.ini to match your wiring.
    // QCA7000setup() will initialise the SPI bus using the pin macros defined
    // in port/esp32s3/qca7000.hpp. Override PLC_SPI_*_PIN in platformio.ini if
    // your wiring differs from the defaults.
    Serial.println("Starting SLAC modem...");
    // Use custom SPI pins matching the modem wiring. Chip select is handled
    // manually by the driver, therefore SPI.begin is called with -1 for the CS
    // parameter. PLC_SPI_CS_PIN and PLC_SPI_RST_PIN are provided via build
    // flags in platformio.ini.
    Serial.println("Starting SLAC modem...");
    SPI.begin(48 /*SCK*/, 21 /*MISO*/, 47 /*MOSI*/, -1);
    Serial.println("Starting SPI");
    qca7000_config cfg{&SPI, PLC_SPI_CS_PIN, PLC_SPI_RST_PIN, MY_MAC};
    Serial.println("Starting QCA7000 Link ");
    static slac::port::Qca7000Link link(cfg);
    static slac::Channel channel(&link);
    g_channel = &channel;
    if (!channel.open()) {
        Serial.println("Failed to open SLAC channel, aborting");
        g_channel = nullptr;
        while (true)
            delay(1000);
    }

    // send a minimal CM_SLAC_PARM.REQ as an example
    slac::messages::HomeplugMessage msg;
    slac::messages::cm_slac_parm_req req{};
    req.application_type = 0;
    req.security_type = 0;
    memset(req.run_id, 0x00, sizeof(req.run_id));

    if (!msg.setup_payload(&req, sizeof(req),
                           slac::defs::MMTYPE_CM_SLAC_PARAM | slac::defs::MMTYPE_MODE_REQ,
                           slac::defs::MMV::AV_1_0)) {
        Serial.println("setup_payload failed: payload too large");
    } else {
        uint8_t dst_mac[ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
        msg.setup_ethernet_header(dst_mac);
        if (!channel.write(msg, 1000)) {
            Serial.println("Failed to transmit SLAC parameter request");
        }
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
