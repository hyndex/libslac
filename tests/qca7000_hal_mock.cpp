#include "arduino_stubs.hpp"
#include "qca7000.hpp"
#include <slac/iso15118_consts.hpp>
#include <slac/config.hpp>
#include <slac/endian.hpp>
#include <cstdint>
#include <atomic>
#include <cstdio>
extern uint32_t g_mock_millis;
void qca7000ToggleCpEf();
bool qca7000CheckBcbToggle();
#include <cstring>
#ifdef le16toh
#undef le16toh
#endif
#ifdef htole16
#undef htole16
#endif
#ifdef le32toh
#undef le32toh
#endif
#ifdef htole32
#undef htole32
#endif

spi_device_handle_t spi_used = nullptr;
int spi_cs = -1;
int spi_rst = -1;
int spi_int = -1;
int spi_pwr = -1;

bool reset_called = false;
bool soft_reset_called = false;

static uint8_t spi_read_buf[V2GTP_BUFFER_SIZE];
static size_t spi_read_len = 0;

static constexpr uint16_t RX_HDR = 12;
static constexpr uint16_t FTR_LEN = 2;

uint8_t myethtransmitbuffer[V2GTP_BUFFER_SIZE];
size_t myethtransmitlen = 0;
uint8_t myethreceivebuffer[V2GTP_BUFFER_SIZE];
size_t myethreceivelen = 0;
const char* PLC_TAG = "mock";

namespace {
struct RxEntry { size_t len; uint8_t data[V2GTP_BUFFER_SIZE]; };
static constexpr uint8_t RING_SIZE = CONFIG_RX_RING_SIZE;
static constexpr uint8_t RING_MASK = RING_SIZE - 1;
static RxEntry ring[RING_SIZE];
static std::atomic<uint8_t> head{0}, tail{0};
static std::atomic<uint32_t> rx_overflow_count{0};
}

uint16_t mock_signature = 0;
uint16_t mock_wrbuf = 0;
uint16_t mock_intr_cause = SPI_INT_CPU_ON;

extern "C" void mock_ring_reset() {
    head.store(0);
    tail.store(0);
    rx_overflow_count.store(0, std::memory_order_relaxed);
}
extern "C" void mock_receive_frame(const uint8_t* f, size_t l) {
    if (l > V2GTP_BUFFER_SIZE) l = V2GTP_BUFFER_SIZE;
    uint8_t h = head.load();
    uint8_t t = tail.load();
    uint8_t next = (h + 1) & RING_MASK;
    if (next == t) {
        rx_overflow_count.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    memcpy(ring[h].data, f, l);
    ring[h].len = l;
    head.store(next);
}

uint32_t qca7000GetRxOverflowCount() {
    return rx_overflow_count.load(std::memory_order_relaxed);
}

extern "C" void mock_spi_feed_raw(const uint8_t* d, size_t l) {
    if (l > V2GTP_BUFFER_SIZE)
        l = V2GTP_BUFFER_SIZE;
    memcpy(spi_read_buf, d, l);
    spi_read_len = l;
    soft_reset_called = false;
}

bool qca7000setup(spi_device_handle_t spi,
                  int cs,
                  int rst,
                  int intr,
                  int pwr,
                  bool /*auto_irq*/) {
    spi_used = spi;
    spi_cs = cs;
    spi_rst = rst;
    spi_int = intr;
    spi_pwr = pwr;
    return true;
}

void qca7000teardown() { spi_used = nullptr; }

bool qca7000ResetAndCheck() {
    reset_called = true;
    if (mock_signature != 0xAA55 || mock_wrbuf != 0x0C5B) {
        fprintf(stderr, "Reset probe failed (SIG=%04X BUF=%04X)\n",
                mock_signature, mock_wrbuf);
        return false;
    }
    if (!(mock_intr_cause & SPI_INT_CPU_ON)) {
        fprintf(stderr, "CPU_ON not asserted after hard reset\n");
        return false;
    }
    return true;
}
bool qca7000SoftReset() {
    soft_reset_called = true;
    if (!(mock_intr_cause & SPI_INT_CPU_ON)) {
        fprintf(stderr, "CPU_ON not asserted after soft reset\n");
        return false;
    }
    return true;
}
esp_err_t qca7000ReadInternalReg(uint16_t reg, uint16_t* out) {
    if (reg == SPI_REG_SIGNATURE)
        *out = mock_signature;
    else if (reg == SPI_REG_WRBUF_SPC_AVA)
        *out = mock_wrbuf;
    else if (reg == SPI_REG_INTR_CAUSE)
        *out = mock_intr_cause;
    else
        *out = 0;
    return ESP_OK;
}
bool qca7000CheckAlive() {
    uint16_t sig = 0, cause = 0, dummy = 0;
    qca7000ReadInternalReg(SPI_REG_SIGNATURE, &sig);
    qca7000ReadInternalReg(SPI_REG_WRBUF_SPC_AVA, &dummy);
    qca7000ReadInternalReg(SPI_REG_INTR_CAUSE, &cause);
    return sig == 0xAA55 && (cause & SPI_INT_CPU_ON);
}
bool qca7000ReadSignature(uint16_t* sig, uint16_t* ver) { if(sig) *sig = 0xAA55; if(ver) *ver=1; return true; }

esp_err_t fetchRx() {
    if (spi_read_len == 0)
        return ESP_OK;

    size_t remaining = spi_read_len;
    const uint8_t* p = spi_read_buf;
    if (remaining < RX_HDR + FTR_LEN || remaining > V2GTP_BUFFER_SIZE) {
        spi_read_len = 0;
        return ESP_OK;
    }

    while (remaining >= RX_HDR + FTR_LEN) {
        uint32_t len = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                        ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
        if (memcmp(p + 4, "\xAA\xAA\xAA\xAA", 4) != 0) {
            qca7000SoftReset();
            spi_read_len = 0;
            return ESP_OK;
        }

        uint16_t fl = slac::le16toh(static_cast<uint16_t>((p[9] << 8) | p[8]));
        uint16_t frame_total = RX_HDR + fl + FTR_LEN;
        if (len != frame_total && len + 4 != frame_total) {
            qca7000SoftReset();
            spi_read_len = 0;
            return;
        }
        if (frame_total > remaining) {
            qca7000SoftReset();
            spi_read_len = 0;
            return;
        }
        if (p[RX_HDR + fl] != 0x55 || p[RX_HDR + fl + 1] != 0x55) {
            qca7000SoftReset();
            spi_read_len = 0;
            return;
        }
        mock_receive_frame(p + RX_HDR, fl);
        p += frame_total;
        remaining -= frame_total;
    }
    spi_read_len = 0;
}

size_t spiQCA7000checkForReceivedData(uint8_t* dst, size_t len) {
    uint8_t t = tail.load();
    if (head.load() == t)
        return 0;
    size_t l = ring[t].len;
    size_t c = l > len ? len : l;
    memcpy(dst, ring[t].data, c);
    tail.store((t + 1) & RING_MASK);
    return c;
}
bool spiQCA7000SendEthFrame(const uint8_t* f, size_t l) {
    if(l>sizeof(myethtransmitbuffer)) l = sizeof(myethtransmitbuffer);
    memcpy(myethtransmitbuffer, f, l); myethtransmitlen = l; return true;
}
static uint8_t mock_retry = 0;
static uint32_t mock_timer = 0;
static SlacState mock_result = SlacState::Idle;
static uint8_t matched_mac[ETH_ALEN] = {};
static bool filter_active = false;

bool qca7000startSlac() {
    mock_retry = slac::defs::C_EV_MATCH_RETRY;
    mock_timer = g_mock_millis;
    mock_result = SlacState::WaitParmCnf;
    filter_active = false;
    memset(matched_mac, 0, sizeof(matched_mac));
    return true;
}

bool qca7000LeaveAvln() {
    filter_active = false;
    memset(matched_mac, 0, sizeof(matched_mac));
    mock_result = SlacState::Idle;
    return true;
}

static void handle_frame(const uint8_t* d, size_t l) {
    if (l < sizeof(ether_header) + 3)
        return;
    const ether_header* eth = reinterpret_cast<const ether_header*>(d);
    if (filter_active && memcmp(eth->ether_shost, matched_mac, ETH_ALEN) != 0)
        return;
    const uint8_t* p = d + sizeof(ether_header);
    uint8_t mmv = p[0];
    uint16_t mmtype;
    memcpy(&mmtype, p + 1, 2);
    mmtype = slac::le16toh(mmtype);
    if (mmv != static_cast<uint8_t>(slac::defs::MMV::AV_1_0))
        return;

    switch (mock_result) {
    case SlacState::WaitParmCnf:
        if (mmtype == (slac::defs::MMTYPE_CM_SLAC_PARAM | slac::defs::MMTYPE_MODE_CNF))
            mock_result = SlacState::Sounding;
        break;
    case SlacState::Sounding:
        if (mmtype == (slac::defs::MMTYPE_CM_ATTEN_CHAR | slac::defs::MMTYPE_MODE_IND))
            mock_result = SlacState::WaitSetKey;
        break;
    case SlacState::WaitSetKey:
        if (mmtype == (slac::defs::MMTYPE_CM_SET_KEY | slac::defs::MMTYPE_MODE_REQ))
            mock_result = SlacState::WaitValidate;
        break;
    case SlacState::WaitValidate:
        if (mmtype == (slac::defs::MMTYPE_CM_VALIDATE | slac::defs::MMTYPE_MODE_REQ)) {
            if (slac::validation_disabled() || qca7000CheckBcbToggle())
                mock_result = SlacState::WaitMatch;
            else
                mock_result = SlacState::Failed;
        }
        break;
    case SlacState::WaitMatch:
        if (mmtype == (slac::defs::MMTYPE_CM_SLAC_MATCH | slac::defs::MMTYPE_MODE_REQ)) {
            mock_result = SlacState::Matched;
            memcpy(matched_mac, eth->ether_shost, ETH_ALEN);
            filter_active = true;
        }
        break;
    default:
        break;
    }
    spi_read_len = 0;
    return ESP_OK;
}

SlacState qca7000getSlacResult() {
    if (mock_result == SlacState::WaitParmCnf &&
        g_mock_millis - mock_timer > slac::defs::TT_EVSE_SLAC_INIT_MS) {
        if (mock_retry > 0) {
            --mock_retry;
            mock_timer = g_mock_millis;
            qca7000ToggleCpEf();
            if (mock_retry == 0)
                mock_result = SlacState::Failed;
        } else {
            mock_result = SlacState::Failed;
        }
    }

    uint8_t buf[V2GTP_BUFFER_SIZE];
    size_t l;
    while ((l = spiQCA7000checkForReceivedData(buf, sizeof(buf))) > 0)
        handle_frame(buf, l);

    return mock_result;
}
void qca7000Process() {
    if (qca7000CheckBcbToggle())
        qca7000Wake();
}
void qca7000ProcessSlice(uint32_t) {}
bool qca7000DriverFatal() { return false; }
void qca7000SetErrorCallback(qca7000_error_cb_t, void*, bool*) {}
void qca7000SetNmk(const uint8_t[slac::defs::NMK_LEN]) {}
void qca7000SetMac(const uint8_t[ETH_ALEN]) {}
const uint8_t* qca7000GetMac() { static uint8_t mac[ETH_ALEN] = {}; return mac; }
const uint8_t* qca7000GetMatchedMac() { return matched_mac; }
static bool sleeping = false;
bool wake_called = false;
bool mock_bcb_toggle = false;
static qca7000_link_ready_cb_t ready_cb = nullptr;
static void* ready_arg = nullptr;
void qca7000SetLinkReadyCallback(qca7000_link_ready_cb_t cb, void* arg) {
    ready_cb = cb;
    ready_arg = arg;
}
bool qca7000Sleep() {
    sleeping = true;
    if (ready_cb)
        ready_cb(false, ready_arg);
    return true;
}
bool qca7000Wake() {
    wake_called = true;
    sleeping = false;
    if (ready_cb)
        ready_cb(true, ready_arg);
    return true;
}
bool qca7000CheckBcbToggle() {
    if (mock_bcb_toggle) {
        mock_bcb_toggle = false;
        return true;
    }
    return false;
}

void qca7000HandleSlacParmCnf(slac::messages::HomeplugMessage& msg) {
    handle_frame(reinterpret_cast<const uint8_t*>(msg.get_raw_message_ptr()),
                 msg.get_raw_msg_len());
}
void qca7000HandleStartAttenCharInd(slac::messages::HomeplugMessage& msg) {
    handle_frame(reinterpret_cast<const uint8_t*>(msg.get_raw_message_ptr()),
                 msg.get_raw_msg_len());
}
void qca7000HandleAttenProfileInd(slac::messages::HomeplugMessage& msg) {
    handle_frame(reinterpret_cast<const uint8_t*>(msg.get_raw_message_ptr()),
                 msg.get_raw_msg_len());
}
void qca7000HandleAttenCharInd(slac::messages::HomeplugMessage& msg) {
    handle_frame(reinterpret_cast<const uint8_t*>(msg.get_raw_message_ptr()),
                 msg.get_raw_msg_len());
}
void qca7000HandleSetKeyReq(slac::messages::HomeplugMessage& msg) {
    handle_frame(reinterpret_cast<const uint8_t*>(msg.get_raw_message_ptr()),
                 msg.get_raw_msg_len());
}
void qca7000HandleValidateReq(slac::messages::HomeplugMessage& msg) {
    handle_frame(reinterpret_cast<const uint8_t*>(msg.get_raw_message_ptr()),
                 msg.get_raw_msg_len());
}
void qca7000HandleSlacMatchReq(slac::messages::HomeplugMessage& msg) {
    handle_frame(reinterpret_cast<const uint8_t*>(msg.get_raw_message_ptr()),
                 msg.get_raw_msg_len());
}
