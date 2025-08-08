#pragma once

#include "port_config.hpp"
// port_common.hpp is pulled in by port_config.hpp after guarding

#include <slac/ethernet_defs.hpp>
#ifdef ESP_PLATFORM
#include "driver/gpio.h"
#include "driver/spi_master.h"
#else
using spi_device_handle_t = void*;
using gpio_num_t = int;
#endif
#include <slac/channel.hpp>
#include <slac/slac.hpp>
#include <stddef.h>
#include <stdint.h>

#ifndef V2GTP_BUFFER_SIZE
#define V2GTP_BUFFER_SIZE 1536
#endif

static_assert(ETH_FRAME_LEN <= V2GTP_BUFFER_SIZE,
              "ETH_FRAME_LEN must not exceed V2GTP_BUFFER_SIZE");

static_assert(PLC_SPI_CS_PIN >= 0, "CS pin unset");
static_assert(PLC_SPI_RST_PIN >= 0, "RST pin unset");
static_assert(PLC_SPI_SCK_PIN >= 0, "SCK pin unset");
static_assert(PLC_SPI_MOSI_PIN >= 0, "MOSI pin unset");
static_assert(PLC_SPI_MISO_PIN >= 0, "MISO pin unset");

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
#ifndef QCASPI_MULTI_CS_BIT
#define QCASPI_MULTI_CS_BIT (1 << 1)
#endif


struct qca7000_config {
    spi_device_handle_t spi;
    int cs_pin;
    int rst_pin{PLC_SPI_RST_PIN};
    int int_pin{PLC_INT_PIN};
    int pwr_en_pin{PLC_PWR_EN_PIN};
    const uint8_t* mac_addr{nullptr};
};

bool qca7000setup(spi_device_handle_t spi,
                  int cs_pin,
                  int rst_pin = PLC_SPI_RST_PIN,
                  int int_pin = PLC_INT_PIN,
                  int pwr_en_pin = PLC_PWR_EN_PIN);
void qca7000teardown();
bool qca7000ResetAndCheck();
bool qca7000SoftReset();
// Leave the current AVLN and reset internal state.
bool qca7000LeaveAvln();
uint16_t qca7000ReadInternalReg(uint16_t reg);
bool qca7000ReadSignature(uint16_t* sig = nullptr, uint16_t* ver = nullptr);
// Poll a few internal registers to verify that the modem is responsive.
// Returns ``true`` when ``SPI_REG_SIGNATURE`` matches 0xAA55 and the
// ``CPU_ON`` interrupt cause is asserted.  Applications should invoke this
// check roughly once per minute to detect a stalled device.
bool qca7000CheckAlive();
size_t spiQCA7000checkForReceivedData(uint8_t* dst, size_t maxLen);
bool spiQCA7000SendEthFrame(const uint8_t* frame, size_t len);
bool qca7000startSlac();
uint8_t qca7000getSlacResult();
void qca7000ToggleCpEf();
// Poll the modem for events and service the RX ring.
// If a CPU_ON or buffer error interrupt is detected the driver
// attempts a soft reset via qca7000SoftReset(). Should that fail the
// reset pin is toggled using a hard reset.
void qca7000Process();

enum class Qca7000ErrorStatus { Reset, DriverFatal };
typedef void (*qca7000_error_cb_t)(Qca7000ErrorStatus, void*);
void qca7000SetErrorCallback(qca7000_error_cb_t cb, void* arg, bool* flag);
bool qca7000DriverFatal();
uint32_t qca7000GetRxOverflowCount();
void qca7000SetIds(const uint8_t pev_id[slac::messages::PEV_ID_LEN],
                   const uint8_t evse_id[slac::messages::EVSE_ID_LEN]);
void qca7000SetNmk(const uint8_t nmk[slac::defs::NMK_LEN]);
const uint8_t* qca7000GetNmk();
void qca7000SetMac(const uint8_t mac[ETH_ALEN]);
const uint8_t* qca7000GetMac();
/// Returns the MAC address of the matched PEV, or all zeros when none.
const uint8_t* qca7000GetMatchedMac();

enum class qca7000_region : uint8_t {
    EU = 0x00,
    NA = 0x01,
};

qca7000_region qca7000GetRegion();
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

  typedef void (*qca7000_link_ready_cb_t)(bool ready, void* arg);
  void qca7000SetLinkReadyCallback(qca7000_link_ready_cb_t cb, void* arg);
  bool qca7000Sleep();
  bool qca7000Wake();

  // Helpers for SLAC message handling
  void qca7000HandleSlacParmCnf(slac::messages::HomeplugMessage& msg);
  void qca7000HandleStartAttenCharInd(slac::messages::HomeplugMessage& msg);
  void qca7000HandleAttenProfileInd(slac::messages::HomeplugMessage& msg);
  void qca7000HandleAttenCharInd(slac::messages::HomeplugMessage& msg);
  void qca7000HandleSetKeyReq(slac::messages::HomeplugMessage& msg);
  void qca7000HandleValidateReq(slac::messages::HomeplugMessage& msg);
  void qca7000HandleSlacMatchReq(slac::messages::HomeplugMessage& msg);
