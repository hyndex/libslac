#pragma once

#include "ethernet_defs.hpp"
#ifdef ARDUINO
#include <Arduino.h>
#include <SPI.h>
#endif
#include <stddef.h>
#include <stdint.h>

#ifndef V2GTP_BUFFER_SIZE
#define V2GTP_BUFFER_SIZE 1536
#endif

// Placeholder register and interrupt definitions for building the library
#ifndef SPI_INT_CPU_ON
#define SPI_INT_CPU_ON 0x0001
#endif
#ifndef SPI_INT_PKT_AVLBL
#define SPI_INT_PKT_AVLBL 0x0002
#endif
#ifndef SPI_INT_RDBUF_ERR
#define SPI_INT_RDBUF_ERR 0x0004
#endif
#ifndef SPI_INT_WRBUF_ERR
#define SPI_INT_WRBUF_ERR 0x0008
#endif
#ifndef SPI_REG_SIGNATURE
#define SPI_REG_SIGNATURE 0x0000
#endif
#ifndef SPI_REG_WRBUF_SPC_AVA
#define SPI_REG_WRBUF_SPC_AVA 0x0000
#endif
#ifndef SPI_REG_INTR_CAUSE
#define SPI_REG_INTR_CAUSE 0x0000
#endif
#ifndef SPI_REG_BFR_SIZE
#define SPI_REG_BFR_SIZE 0x0000
#endif
#ifndef SPI_REG_RDBUF_BYTE_AVA
#define SPI_REG_RDBUF_BYTE_AVA 0x0000
#endif
#ifndef SPI_REG_INTR_ENABLE
#define SPI_REG_INTR_ENABLE 0x0000
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
