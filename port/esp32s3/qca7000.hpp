#pragma once

#include "../port_common.hpp"
#include "port_config.hpp"

#include "ethernet_defs.hpp"
#ifdef ARDUINO
#include <Arduino.h>
#include <SPI.h>
#endif
#include <slac/channel.hpp>
#include <stddef.h>
#include <stdint.h>

#ifndef V2GTP_BUFFER_SIZE
#define V2GTP_BUFFER_SIZE 1536
#endif

static_assert(ETH_FRAME_LEN <= V2GTP_BUFFER_SIZE,
              "ETH_FRAME_LEN must not exceed V2GTP_BUFFER_SIZE");

// Register and interrupt definitions (see QCA7000 datasheet)
#ifndef SPI_INT_CPU_ON
#define SPI_INT_CPU_ON 0x0040
#endif
#ifndef SPI_INT_PKT_AVLBL
#define SPI_INT_PKT_AVLBL 0x0001
#endif
#ifndef SPI_INT_RDBUF_ERR
#define SPI_INT_RDBUF_ERR 0x0002
#endif
#ifndef SPI_INT_WRBUF_ERR
#define SPI_INT_WRBUF_ERR 0x0004
#endif
#ifndef SPI_REG_SIGNATURE
#define SPI_REG_SIGNATURE 0x1A00
#endif
#ifndef SPI_REG_WRBUF_SPC_AVA
#define SPI_REG_WRBUF_SPC_AVA 0x0200
#endif
#ifndef SPI_REG_INTR_CAUSE
#define SPI_REG_INTR_CAUSE 0x0C00
#endif
#ifndef SPI_REG_BFR_SIZE
#define SPI_REG_BFR_SIZE 0x0100
#endif
#ifndef SPI_REG_RDBUF_BYTE_AVA
#define SPI_REG_RDBUF_BYTE_AVA 0x0300
#endif
#ifndef SPI_REG_INTR_ENABLE
#define SPI_REG_INTR_ENABLE 0x0D00
#endif
#ifndef SPI_REG_SPI_CONFIG
#define SPI_REG_SPI_CONFIG 0x0400
#endif
#ifndef QCASPI_SLAVE_RESET_BIT
#define QCASPI_SLAVE_RESET_BIT (1 << 6)
#endif


struct qca7000_config {
    SPIClass* spi;
    int cs_pin;
    int rst_pin{PLC_SPI_RST_PIN};
    const uint8_t* mac_addr{nullptr};
};

bool qca7000setup(SPIClass* spi, int cs_pin, int rst_pin = PLC_SPI_RST_PIN);
void qca7000teardown();
bool qca7000ResetAndCheck();
bool qca7000SoftReset();
uint16_t qca7000ReadInternalReg(uint16_t reg);
bool qca7000ReadSignature(uint16_t* sig = nullptr, uint16_t* ver = nullptr);
size_t spiQCA7000checkForReceivedData(uint8_t* dst, size_t maxLen);
bool spiQCA7000SendEthFrame(const uint8_t* frame, size_t len);
bool qca7000startSlac();
uint8_t qca7000getSlacResult();
// Poll the modem for events and service the RX ring.
// If a CPU_ON or buffer error interrupt is detected the driver
// attempts a soft reset via qca7000SoftReset(). Should that fail the
// reset pin is toggled using a hard reset.
void qca7000Process();

typedef void (*qca7000_error_cb_t)(void*);
void qca7000SetErrorCallback(qca7000_error_cb_t cb, void* arg, bool* flag);
#ifdef ESP_PLATFORM
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

struct Qca7000TaskContext {
    slac::Channel* channel;
    QueueHandle_t queue;
};

void qca7000_task(void* arg);
#endif

extern uint8_t myethtransmitbuffer[V2GTP_BUFFER_SIZE];
extern size_t myethtransmitlen;
extern uint8_t myethreceivebuffer[V2GTP_BUFFER_SIZE];
extern size_t myethreceivelen;
extern const char* PLC_TAG;
