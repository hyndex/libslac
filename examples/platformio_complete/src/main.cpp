#include <Arduino.h>
#include <slac/channel.hpp>
#include <slac/slac.hpp>
#include <port/esp32s3/qca7000_link.hpp>
#include <port/esp32s3/qca7000.hpp>

// qca7000ProcessSlice is declared in qca7000.hpp with a default timeout.
// The extern declaration here must not specify the default again to avoid
// multiple default argument definitions.
extern void qca7000ProcessSlice(uint32_t max_us);

// Default MAC address for the modem. Adjust as required.
static const uint8_t MY_MAC[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

// Global pointer used by the polling loop
static slac::Channel* g_channel = nullptr;
volatile bool plc_irq = false;

void IRAM_ATTR plc_isr() { plc_irq = true; }
// Timestamp for SLAC restart logic
static uint32_t g_slac_ts = 0;

void setup() {
    Serial.begin(115200);
    delay(4000);
    Serial.println("Starting SLAC modem...");
    SPI.begin(48 /*SCK*/, 21 /*MISO*/, 47 /*MOSI*/, -1);
    Serial.println("Starting SPI");
    pinMode(PLC_INT_PIN, INPUT);
    attachInterrupt(PLC_INT_PIN, plc_isr, FALLING);
    qca7000_config cfg{&SPI, PLC_SPI_CS_PIN, PLC_SPI_RST_PIN, MY_MAC};
    Serial.println("Starting QCA7000 Link ");
    static slac::port::Qca7000Link link(cfg);
    link.set_error_callback([](Qca7000ErrorStatus, void*) {
        Serial.println("[PLC] Fatal error - driver auto-reset");
    }, nullptr);
    static slac::Channel channel(&link);
    g_channel = &channel;
    Serial.println("Starting SLAC channel");
    if (!channel.open()) {
        Serial.println("Failed to open SLAC channel, aborting");
        g_channel = nullptr;
        while (true)
            delay(1000);
    }

    Serial.println("Starting SLAC FSM");
    if (!qca7000startSlac())
        Serial.println("startSlac failed");
    // Remember when the handshake was (re)started
    g_slac_ts = millis();
}

void loop() {
    Serial.println("Loop Started");
    if (plc_irq) {
        plc_irq = false;
        qca7000ProcessSlice();
    }
    Serial.println("Loop Middle");

    slac::messages::HomeplugMessage msg;
    if (g_channel && g_channel->poll(msg)) {
        // Handle incoming SLAC messages here
    }

    // Update the SLAC state machine and restart if no progress for 60s
    uint8_t state = qca7000getSlacResult();
    if (state != 0 && state != 5) {
        if (millis() - g_slac_ts > 60000) {
            Serial.println("Restarting SLAC handshake");
            if (!qca7000startSlac())
                Serial.println("startSlac failed");
            g_slac_ts = millis();
        }
    } else {
        g_slac_ts = millis();
    }

    delay(1);
}
