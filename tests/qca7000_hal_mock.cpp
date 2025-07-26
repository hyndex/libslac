#include "arduino_stubs.hpp"
#include "port/esp32s3/qca7000.hpp"
#include <cstring>
#include <atomic>

SPIClass* spi_used = nullptr;
int spi_cs = -1;
int spi_rst = -1;

bool reset_called = false;

uint8_t myethtransmitbuffer[V2GTP_BUFFER_SIZE];
size_t myethtransmitlen = 0;
uint8_t myethreceivebuffer[V2GTP_BUFFER_SIZE];
size_t myethreceivelen = 0;
const char* PLC_TAG = "mock";

uint16_t mock_signature = 0;
uint16_t mock_wrbuf = 0;
uint16_t mock_intr_cause = 0;

namespace {
struct RxEntry {
    size_t len;
    uint8_t data[V2GTP_BUFFER_SIZE];
};
static constexpr uint8_t RING_SIZE = 8;
static constexpr uint8_t RING_MASK = RING_SIZE - 1;
static RxEntry ring[RING_SIZE];
static std::atomic<uint8_t> head{0}, tail{0};

inline void ring_reset() {
    head.store(0, std::memory_order_release);
    tail.store(0, std::memory_order_release);
}

inline void ring_push(const uint8_t* d, size_t l) {
    if (l > V2GTP_BUFFER_SIZE)
        l = V2GTP_BUFFER_SIZE;
    uint8_t h = head.load(std::memory_order_acquire);
    uint8_t t = tail.load(std::memory_order_acquire);
    uint8_t next = (h + 1) & RING_MASK;
    if (next == t)
        return;
    memcpy(ring[h].data, d, l);
    ring[h].len = l;
    head.store(next, std::memory_order_release);
}

inline bool ring_pop(const uint8_t** d, size_t* l) {
    uint8_t t = tail.load(std::memory_order_acquire);
    if (head.load(std::memory_order_acquire) == t)
        return false;
    *d = ring[t].data;
    *l = ring[t].len;
    tail.store((t + 1) & RING_MASK, std::memory_order_release);
    return true;
}
} // namespace

extern "C" void mock_ring_reset() { ring_reset(); }
extern "C" void mock_receive_frame(const uint8_t* f, size_t l) { ring_push(f, l); }

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
    const uint8_t* s;
    size_t l;
    if (!ring_pop(&s, &l))
        return 0;
    size_t c = l > len ? len : l;
    memcpy(dst, s, c);
    size_t store = l > V2GTP_BUFFER_SIZE ? V2GTP_BUFFER_SIZE : l;
    memcpy(myethreceivebuffer, s, store);
    myethreceivelen = store;
    return c;
}
bool spiQCA7000SendEthFrame(const uint8_t* f, size_t l) {
    if(l>sizeof(myethtransmitbuffer)) l = sizeof(myethtransmitbuffer);
    memcpy(myethtransmitbuffer, f, l); myethtransmitlen = l; return true;
}
bool qca7000startSlac() { return true; }
uint8_t qca7000getSlacResult() { return 0; }
void qca7000Process() {}
void qca7000ProcessSlice(uint32_t) {}
bool qca7000DriverFatal() { return false; }
void qca7000SetErrorCallback(qca7000_error_cb_t, void*, bool*) {}
void qca7000SetNmk(const uint8_t[slac::defs::NMK_LEN]) {}
void qca7000SetMac(const uint8_t[ETH_ALEN]) {}
const uint8_t* qca7000GetMac() { static uint8_t mac[ETH_ALEN] = {}; return mac; }
