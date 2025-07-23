#pragma once

#ifdef ESP_PLATFORM
#include "port/esp32s3/port_config.hpp"
#endif


#include "ethernet_defs.hpp"
#include <Arduino.h>
#include <SPI.h>
#include <stddef.h>
#include <stdint.h>

#ifndef V2GTP_BUFFER_SIZE
#define V2GTP_BUFFER_SIZE 1536
#endif

#ifndef PLC_SPI_RST_PIN
#define PLC_SPI_RST_PIN 5
#endif
#ifndef PLC_SPI_CS_PIN
#define PLC_SPI_CS_PIN 17
#endif

struct qca7000_config {
    SPIClass* spi;
    int cs_pin;
};

bool qca7000setup(SPIClass* spi, int cs_pin);
bool qca7000ResetAndCheck();
uint16_t qca7000ReadInternalReg(uint8_t reg);
bool qca7000ReadSignature(uint16_t* sig = nullptr, uint16_t* ver = nullptr);
size_t spiQCA7000checkForReceivedData(uint8_t* dst, size_t maxLen);
bool spiQCA7000SendEthFrame(const uint8_t* frame, size_t len);
void qca7000Process();

extern uint8_t myethtransmitbuffer[V2GTP_BUFFER_SIZE];
extern size_t myethtransmitlen;
extern uint8_t myethreceivebuffer[V2GTP_BUFFER_SIZE];
extern size_t myethreceivelen;
extern const char* PLC_TAG;
