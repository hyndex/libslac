#include <Arduino.h>
#include <slac/channel.hpp>
#include <slac/slac.hpp>
#include <port/esp32s3/qca7000_link.hpp>
#include <port/esp32s3/qca7000.hpp>
#include <atomic>
#include "cp_pwm.h"
#include "cp_monitor.h"
#include "cp_state_machine.h"

// qca7000ProcessSlice is declared in qca7000.hpp with a default timeout.
// The extern declaration here must not specify the default again to avoid
// multiple default argument definitions.
extern void qca7000ProcessSlice(uint32_t max_us);

// Default MAC address for the modem. Adjust as required.
static const uint8_t MY_MAC[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

// Global pointer used by the polling loop
static slac::Channel* g_channel = nullptr;
volatile bool plc_irq = false;
std::atomic<uint8_t> g_slac_state{0};

void IRAM_ATTR plc_isr() { plc_irq = true; }
// Timestamp for SLAC restart logic
std::atomic<uint32_t> g_slac_ts{0};

static void logTask(void*) {
    const TickType_t period = pdMS_TO_TICKS(1000);
    while (true) {
        uint16_t mv = cpGetVoltageMv();
        Serial.printf("[STAT] CP=%c %u.%03u V Stage=%s SLAC=%u\n",
                      cpGetStateLetter(), mv / 1000, mv % 1000,
                      evseStageName(evseGetStage()),
                      g_slac_state.load(std::memory_order_relaxed));
        vTaskDelay(period);
    }
}

void setup() {
    Serial.begin(115200);
    delay(4000);
    Serial.println("Starting SLAC modem...");
    pinMode(LOCK_FB_PIN, INPUT_PULLUP);
    pinMode(CONTACTOR_FB_PIN, INPUT_PULLUP);
    pinMode(ISOLATION_OK_PIN, INPUT_PULLUP);

    cpPwmInit();
    cpMonitorInit();
    cpLowRateStart(10);
    evseStateMachineInit();
    xTaskCreatePinnedToCore(evseStateMachineTask, "evseSM", 4096, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(logTask, "log", 6144, nullptr, 1, nullptr, 0);
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

}

void loop() {
    if (plc_irq) {
        plc_irq = false;
        qca7000ProcessSlice(500);
    }

    slac::messages::HomeplugMessage msg;
    if (g_channel && g_channel->poll(msg)) {
        // Handle incoming SLAC messages here
    }

    // Update the SLAC state machine and restart if no progress for 60s
    g_slac_state.store(qca7000getSlacResult(), std::memory_order_relaxed);
    if (g_slac_state.load(std::memory_order_relaxed) != 0 &&
        g_slac_state.load(std::memory_order_relaxed) != 5) {
        if (millis() - g_slac_ts.load(std::memory_order_relaxed) > 60000) {
            Serial.println("Restarting SLAC handshake");
            if (!qca7000startSlac())
                Serial.println("startSlac failed");
            g_slac_ts.store(millis(), std::memory_order_relaxed);
        }
    } else {
        g_slac_ts.store(millis(), std::memory_order_relaxed);
    }

    delay(1);
}
