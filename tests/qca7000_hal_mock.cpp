#include "arduino_stubs.hpp"
#include "port/esp32s3/qca7000.hpp"
#include <slac/iso15118_consts.hpp>
#include <slac/config.hpp>
#include <cstdint>
#include <atomic>
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

SPIClass* spi_used = nullptr;
int spi_cs = -1;
int spi_rst = -1;

bool reset_called = false;

uint8_t myethtransmitbuffer[V2GTP_BUFFER_SIZE];
size_t myethtransmitlen = 0;
uint8_t myethreceivebuffer[V2GTP_BUFFER_SIZE];
size_t myethreceivelen = 0;
const char* PLC_TAG = "mock";

namespace {
struct RxEntry { size_t len; uint8_t data[V2GTP_BUFFER_SIZE]; };
static constexpr uint8_t RING_SIZE = 8;
static constexpr uint8_t RING_MASK = RING_SIZE - 1;
static RxEntry ring[RING_SIZE];
static std::atomic<uint8_t> head{0}, tail{0};
}

uint16_t mock_signature = 0;
uint16_t mock_wrbuf = 0;
uint16_t mock_intr_cause = 0;

extern "C" void mock_ring_reset() { head.store(0); tail.store(0); }
extern "C" void mock_receive_frame(const uint8_t* f, size_t l) {
    if (l > V2GTP_BUFFER_SIZE) l = V2GTP_BUFFER_SIZE;
    uint8_t h = head.load();
    uint8_t t = tail.load();
    uint8_t next = (h + 1) & RING_MASK;
    if (next == t) return;
    memcpy(ring[h].data, f, l);
    ring[h].len = l;
    head.store(next);
}

bool qca7000setup(SPIClass* spi, int cs, int rst) {
    spi_used = spi; spi_cs = cs; spi_rst = rst; return true;
}

void qca7000teardown() { spi_used = nullptr; }

bool qca7000ResetAndCheck() {
    reset_called = true;
    return true;
}
bool qca7000SoftReset() { return true; }
uint16_t qca7000ReadInternalReg(uint16_t reg) {
    if (reg == SPI_REG_SIGNATURE)
        return mock_signature;
    if (reg == SPI_REG_WRBUF_SPC_AVA)
        return mock_wrbuf;
    if (reg == SPI_REG_INTR_CAUSE)
        return mock_intr_cause;
    return 0;
}
bool qca7000CheckAlive() {
    uint16_t sig = qca7000ReadInternalReg(SPI_REG_SIGNATURE);
    (void)qca7000ReadInternalReg(SPI_REG_WRBUF_SPC_AVA);
    uint16_t cause = qca7000ReadInternalReg(SPI_REG_INTR_CAUSE);
    return sig == 0xAA55 && (cause & SPI_INT_CPU_ON);
}
bool qca7000ReadSignature(uint16_t* sig, uint16_t* ver) { if(sig) *sig = 0xAA55; if(ver) *ver=1; return true; }
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
static uint8_t mock_result = 0;
static uint8_t matched_mac[ETH_ALEN] = {};
static bool filter_active = false;

bool qca7000startSlac() {
    mock_retry = slac::defs::C_EV_MATCH_RETRY;
    mock_timer = g_mock_millis;
    mock_result = 1;
    filter_active = false;
    memset(matched_mac, 0, sizeof(matched_mac));
    return true;
}

bool qca7000LeaveAvln() {
    filter_active = false;
    memset(matched_mac, 0, sizeof(matched_mac));
    mock_result = 0;
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
    case 1:
        if (mmtype == (slac::defs::MMTYPE_CM_SLAC_PARAM | slac::defs::MMTYPE_MODE_CNF))
            mock_result = 2;
        break;
    case 2:
        if (mmtype == (slac::defs::MMTYPE_CM_ATTEN_CHAR | slac::defs::MMTYPE_MODE_IND))
            mock_result = 3;
        break;
    case 3:
        if (mmtype == (slac::defs::MMTYPE_CM_SET_KEY | slac::defs::MMTYPE_MODE_REQ))
            mock_result = 4;
        break;
    case 4:
        if (mmtype == (slac::defs::MMTYPE_CM_VALIDATE | slac::defs::MMTYPE_MODE_REQ)) {
            if (slac::validation_disabled() || qca7000CheckBcbToggle())
                mock_result = 5;
            else
                mock_result = 0xFF;
        }
        break;
    case 5:
        if (mmtype == (slac::defs::MMTYPE_CM_SLAC_MATCH | slac::defs::MMTYPE_MODE_REQ)) {
            mock_result = 6;
            memcpy(matched_mac, eth->ether_shost, ETH_ALEN);
            filter_active = true;
        }
        break;
    default:
        break;
    }
}

uint8_t qca7000getSlacResult() {
    if (mock_result == 1 && g_mock_millis - mock_timer > slac::defs::TT_EVSE_SLAC_INIT_MS) {
        if (mock_retry > 0) {
            --mock_retry;
            mock_timer = g_mock_millis;
            qca7000ToggleCpEf();
            if (mock_retry == 0)
                mock_result = 0xFF;
        } else {
            mock_result = 0xFF;
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
