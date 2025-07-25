#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>
#include <stddef.h>
#include "port_config.hpp"          //  V2GTP_BUFFER_SIZE, pin aliases, etc.

#ifndef V2GTP_BUFFER_SIZE
#define V2GTP_BUFFER_SIZE 1536
#endif

/* ─────── Register map (QCA AN‑4 rev‑5) ─────── */
#define SPI_REG_BFR_SIZE        0x0100
#define SPI_REG_WRBUF_SPC_AVA   0x0200
#define SPI_REG_RDBUF_BYTE_AVA  0x0300
#define SPI_REG_SPI_CONFIG      0x0400
#define SPI_REG_INTR_CAUSE      0x0C00
#define SPI_REG_INTR_ENABLE     0x0D00
#define SPI_REG_SIGNATURE       0x1A00
#define SPI_REG_VERSION         0x1B00   /* undocumented but present */

/* ─────── Bit masks ─────── */
#define QCASPI_SLAVE_RESET_BIT  (1u << 6)

#define SPI_INT_PKT_AVLBL       (1u << 0)
#define SPI_INT_RDBUF_ERR       (1u << 1)
#define SPI_INT_WRBUF_ERR       (1u << 2)
#define SPI_INT_CPU_ON          (1u << 6)

/* ─────── CRC helper ─────── */
static inline uint16_t qca7000_crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= uint16_t(*data++) << 8;
        for (int i = 0; i < 8; ++i)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

/* ─────── Public API ─────── */
bool      qca7000setup               (SPIClass* spi, int cs_pin);
bool      qca7000ResetAndCheck       ();
uint16_t  qca7000ReadInternalReg     (uint8_t reg);
bool      qca7000ReadSignature       (uint16_t* sig = nullptr,
                                      uint16_t* ver = nullptr);

size_t    spiQCA7000checkForReceivedData(uint8_t* dst, size_t maxLen);
bool      spiQCA7000SendEthFrame     (const uint8_t* frame, size_t len);
bool      myEthTransmit              (const uint8_t* frame, size_t len);

bool      qca7000startSlac           ();
uint8_t   qca7000getSlacResult       ();
void      qca7000Process             ();

/* ─────── Shared scratch buffers ─────── */
extern uint8_t myethtransmitbuffer[V2GTP_BUFFER_SIZE];
extern size_t  myethtransmitlen;
extern uint8_t myethreceivebuffer[V2GTP_BUFFER_SIZE];
extern size_t  myethreceivelen;

/* ─────── Public log tag ─────── */
extern const char* PLC_TAG;
