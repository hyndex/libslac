#include "qca7000.hpp"
#include "../port_common.hpp"
#include "port_config.hpp"
#ifdef ESP_PLATFORM
#include <esp_log.h>
#include <esp_system.h>
#else
#include <arpa/inet.h>
#include <stdint.h>
#ifndef ESP_LOGE
#include <mutex>
#define ESP_LOGE(tag, fmt, ...)
#endif
#ifndef ESP_LOGI
#define ESP_LOGI(tag, fmt, ...)
#endif
#ifndef ESP_LOGW
#define ESP_LOGW(tag, fmt, ...)
#endif
static inline uint32_t esp_random() {
    return 0x12345678u;
}
#endif
#include <slac/slac.hpp>
#include <string.h>

const char* PLC_TAG = "PLC_IF";

uint8_t myethtransmitbuffer[V2GTP_BUFFER_SIZE]{};
size_t myethtransmitlen = 0;
uint8_t myethreceivebuffer[V2GTP_BUFFER_SIZE]{};
size_t myethreceivelen = 0;

static constexpr uint16_t SIG = 0xAA55;
static constexpr uint16_t WRBUF_RST = 0x0C5B;
static constexpr uint32_t FAST_HZ = 8000000;
static constexpr uint32_t SLOW_HZ = 1000000;
static constexpr uint16_t SOF_WORD = 0xAAAA;
static constexpr uint16_t EOF_WORD = 0x5555;
static constexpr uint16_t TX_HDR = 8;
static constexpr uint16_t RX_HDR = 12;
static constexpr uint16_t FTR_LEN = 2;
static constexpr uint16_t INTR_MASK = SPI_INT_CPU_ON | SPI_INT_PKT_AVLBL | SPI_INT_RDBUF_ERR | SPI_INT_WRBUF_ERR;

#ifdef LIBSLAC_TESTING
SPIClass* g_spi = nullptr;
int g_cs = -1;
int g_rst = PLC_SPI_RST_PIN;
#else
static SPIClass* g_spi = nullptr;
static int g_cs = -1;
static int g_rst = PLC_SPI_RST_PIN;
#endif
static SPISettings setSlow(SLOW_HZ, MSBFIRST, SPI_MODE3);
static SPISettings setFast(FAST_HZ, MSBFIRST, SPI_MODE3);

namespace {
struct RxEntry {
    size_t len;
    uint8_t data[V2GTP_BUFFER_SIZE];
};
static RxEntry ring[4];
static volatile uint8_t head = 0, tail = 0;
#ifdef ESP_PLATFORM
static portMUX_TYPE ring_mux = portMUX_INITIALIZER_UNLOCKED;
#else
static std::mutex ring_mutex;
#endif
inline bool ringEmpty() {
    return head == tail;
}
inline void ringPush(const uint8_t* d, size_t l) {
#ifdef ESP_PLATFORM
    portENTER_CRITICAL(&ring_mux);
#else
    ring_mutex.lock();
#endif
    if (l > V2GTP_BUFFER_SIZE)
        l = V2GTP_BUFFER_SIZE;
    uint8_t next = (head + 1) & 3;
    if (next == tail) {
#ifdef ESP_PLATFORM
        portEXIT_CRITICAL(&ring_mux);
#else
        ring_mutex.unlock();
#endif
        ESP_LOGW(PLC_TAG, "RX ring full - dropping frame");
        return;
    }
    memcpy(ring[head].data, d, l);
    ring[head].len = l;
    head = next;
#ifdef ESP_PLATFORM
    portEXIT_CRITICAL(&ring_mux);
#else
    ring_mutex.unlock();
#endif
}
inline bool ringPop(const uint8_t** d, size_t* l) {
#ifdef ESP_PLATFORM
    portENTER_CRITICAL(&ring_mux);
#else
    ring_mutex.lock();
#endif
    if (ringEmpty()) {
#ifdef ESP_PLATFORM
        portEXIT_CRITICAL(&ring_mux);
#else
        ring_mutex.unlock();
#endif
        return false;
    }
    *d = ring[tail].data;
    *l = ring[tail].len;
    tail = (tail + 1) & 3;
#ifdef ESP_PLATFORM
    portEXIT_CRITICAL(&ring_mux);
#else
    ring_mutex.unlock();
#endif
    return true;
}
} // namespace

static inline uint16_t cmd16(bool rd, bool intr, uint16_t reg) {
    return (rd ? 0x8000u : 0) | (intr ? 0x4000u : 0) |
           (((reg << 8) & 0x3FFFu));
}

static uint16_t spiRd16_fast(uint16_t reg) {
    g_spi->beginTransaction(setFast);
    digitalWrite(g_cs, LOW);
    g_spi->transfer16(cmd16(true, true, reg));
    uint16_t v = g_spi->transfer16(0);
    digitalWrite(g_cs, HIGH);
    g_spi->endTransaction();
    return v;
}

static void spiWr16_fast(uint16_t reg, uint16_t val) {
    g_spi->beginTransaction(setFast);
    digitalWrite(g_cs, LOW);
    g_spi->transfer16(cmd16(false, true, reg));
    g_spi->transfer16(val);
    digitalWrite(g_cs, HIGH);
    g_spi->endTransaction();
}

static bool hardReset() {
    pinMode(g_rst, OUTPUT);
    digitalWrite(g_rst, LOW);
    slac_delay(10);
    digitalWrite(g_rst, HIGH);
    slac_delay(100);

    auto slowRd16 = [&](uint16_t reg) -> uint16_t {
        g_spi->beginTransaction(setSlow);
        digitalWrite(g_cs, LOW);
        g_spi->transfer16(cmd16(true, true, reg));
        uint16_t v = g_spi->transfer16(0);
        digitalWrite(g_cs, HIGH);
        g_spi->endTransaction();
        return v;
    };

    uint32_t t0 = slac_millis();
    uint16_t sig = 0, buf = 0;
    do {
        sig = slowRd16(SPI_REG_SIGNATURE);
        buf = slowRd16(SPI_REG_WRBUF_SPC_AVA);
        if (sig == SIG && buf == WRBUF_RST)
            break;
        slac_delay(5);
    } while (slac_millis() - t0 < 200);

    if (sig != SIG || buf != WRBUF_RST) {
        ESP_LOGE(PLC_TAG, "Reset probe failed (SIG=0x%04X BUF=0x%04X)", sig, buf);
        return false;
    }
    ESP_LOGI(PLC_TAG, "Reset probe OK (SIG=0x%04X)", sig);
#ifndef ESP_LOGW
#define ESP_LOGW(tag, fmt, ...)
#endif

    t0 = slac_millis();
    while (!(slowRd16(SPI_REG_INTR_CAUSE) & SPI_INT_CPU_ON) && slac_millis() - t0 < 80)
        ;

    spiWr16_fast(SPI_REG_INTR_CAUSE, 0xFFFF);
    return true;
}

uint16_t qca7000ReadInternalReg(uint16_t r) {
    return spiRd16_fast(r);
}
bool qca7000ReadSignature(uint16_t* s, uint16_t* v) {
    uint16_t sig = qca7000ReadInternalReg(SPI_REG_SIGNATURE),
             ver = qca7000ReadInternalReg(0x1B);
    if (s)
        *s = sig;
    if (v)
        *v = ver;
    return sig == SIG;
}

#ifdef LIBSLAC_TESTING
bool txFrame(const uint8_t* eth, size_t ethLen) {
#else
static bool txFrame(const uint8_t* eth, size_t ethLen) {
#endif
    if (ethLen > 1522)
        return false;
    size_t frameLen = ethLen;
    if (frameLen < 60)
        frameLen = 60;
    uint16_t spiLen = TX_HDR + frameLen + FTR_LEN;
    if (spiRd16_fast(SPI_REG_WRBUF_SPC_AVA) < spiLen)
        return false;

    spiWr16_fast(SPI_REG_BFR_SIZE, spiLen);

    g_spi->beginTransaction(setFast);
    digitalWrite(g_cs, LOW);
    g_spi->transfer16(cmd16(false, false, 0));
    g_spi->transfer16(SOF_WORD);
    g_spi->transfer16(SOF_WORD);
    g_spi->transfer16(htole16(static_cast<uint16_t>(frameLen)));
    g_spi->transfer16(0);
    if (ethLen)
        g_spi->writeBytes(eth, ethLen);
    if (frameLen > ethLen) {
        uint8_t pad[60]{};
        g_spi->writeBytes(pad, frameLen - ethLen);
    }
    g_spi->transfer16(EOF_WORD);
    digitalWrite(g_cs, HIGH);
    g_spi->endTransaction();
    return true;
}
#ifdef LIBSLAC_TESTING
void fetchRx() {
#else
static void fetchRx() {
#endif
    uint16_t avail = spiRd16_fast(SPI_REG_RDBUF_BYTE_AVA);
    if (avail < RX_HDR + FTR_LEN || avail > V2GTP_BUFFER_SIZE)
        return;

    uint16_t requested = avail;
    spiWr16_fast(SPI_REG_BFR_SIZE, requested);

    static uint8_t buf[V2GTP_BUFFER_SIZE + 2];
    g_spi->beginTransaction(setFast);
    digitalWrite(g_cs, LOW);
    g_spi->transfer16(cmd16(true, false, 0));
    for (uint16_t i = 0; i < avail + 2; ++i)
        buf[i] = g_spi->transfer(0);
    digitalWrite(g_cs, HIGH);
    g_spi->endTransaction();

    const uint8_t* p = buf + 2;
    uint32_t len = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    if (len != requested) {
        ESP_LOGE(PLC_TAG, "RX len mismatch: req=%u got=%u", requested, len);
        return;
    }
    if (memcmp(p + 4, "\xAA\xAA\xAA\xAA", 4) != 0)
        return;
    uint16_t fl = le16toh(static_cast<uint16_t>((p[9] << 8) | p[8]));
    if (p[RX_HDR + fl] != 0x55 || p[RX_HDR + fl + 1] != 0x55)
        return;
    ringPush(p + RX_HDR, fl);
}

bool spiQCA7000SendEthFrame(const uint8_t* f, size_t l) {
    bool ok = txFrame(f, l);
    if (ok && l <= V2GTP_BUFFER_SIZE) {
        memcpy(myethtransmitbuffer, f, l);
        myethtransmitlen = l;
    }
    return ok;
}
size_t spiQCA7000checkForReceivedData(uint8_t* d, size_t m) {
    fetchRx();
    const uint8_t* s;
    size_t l;
    if (!ringPop(&s, &l))
        return 0;
    size_t c = l > m ? m : l;
    memcpy(d, s, c);
    size_t store = l > V2GTP_BUFFER_SIZE ? V2GTP_BUFFER_SIZE : l;
    memcpy(myethreceivebuffer, s, store);
    myethreceivelen = l;
    return c;
}

// Current SLAC handshake state. 0 = idle, 1 = waiting for parameter confirmation,
// 2 = waiting for match request, 3 = handshake complete, 0xFF = mismatch.
static uint8_t g_slac = 0;
static uint8_t g_run_id[slac::defs::RUN_ID_LEN]{};
static const uint8_t g_src_mac[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

// Issue a CM_SLAC_PARM.REQ to start the SLAC matching handshake.
bool qca7000startSlac() {
    g_slac = 1; // waiting for CNF

    for (size_t i = 0; i < sizeof(g_run_id); ++i)
        g_run_id[i] = static_cast<uint8_t>(esp_random() & 0xFF);

    struct __attribute__((packed)) {
        ether_header eth;
        struct {
            uint8_t mmv;
            uint16_t mmtype;
        } hp;
        slac::messages::cm_slac_parm_req req;
    } msg{};

    memset(&msg, 0, sizeof(msg));
    memset(msg.eth.ether_dhost, 0xFF, ETH_ALEN);
    memcpy(msg.eth.ether_shost, g_src_mac, ETH_ALEN);
    msg.eth.ether_type = htons(slac::defs::ETH_P_HOMEPLUG_GREENPHY);
    msg.hp.mmv = static_cast<uint8_t>(slac::defs::MMV::AV_1_0);
    msg.hp.mmtype = htole16(slac::defs::MMTYPE_CM_SLAC_PARAM | slac::defs::MMTYPE_MODE_REQ);
    msg.req.application_type = slac::defs::COMMON_APPLICATION_TYPE;
    msg.req.security_type = slac::defs::COMMON_SECURITY_TYPE;
    memcpy(msg.req.run_id, g_run_id, sizeof(g_run_id));

    bool ok = txFrame(reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
    if (!ok)
        g_slac = 0;
    return ok;
}

// Poll for SLAC confirmation frames and update g_slac accordingly.
uint8_t qca7000getSlacResult() {
    fetchRx();
    const uint8_t* d;
    size_t l;
    while (ringPop(&d, &l)) {
        if (l < sizeof(ether_header) + 3)
            continue;
        const ether_header* eth = reinterpret_cast<const ether_header*>(d);
        if (eth->ether_type != htons(slac::defs::ETH_P_HOMEPLUG_GREENPHY))
            continue;
        const uint8_t* p = d + sizeof(ether_header);
        uint8_t mmv = p[0];
        uint16_t mmtype = le16toh(*reinterpret_cast<const uint16_t*>(p + 1));
        if (mmv != static_cast<uint8_t>(slac::defs::MMV::AV_1_0))
            continue;
        if (mmtype == (slac::defs::MMTYPE_CM_SLAC_PARAM | slac::defs::MMTYPE_MODE_CNF)) {
            const auto* cnf = reinterpret_cast<const slac::messages::cm_slac_parm_cnf*>(p + 3);
            if (!memcmp(cnf->run_id, g_run_id, sizeof(g_run_id)))
                g_slac = 2; // waiting for match
            else
                g_slac = 0xFF;
        } else if (mmtype == (slac::defs::MMTYPE_CM_SLAC_MATCH | slac::defs::MMTYPE_MODE_REQ)) {
            const auto* req = reinterpret_cast<const slac::messages::cm_slac_match_req*>(p + 3);
            if (!memcmp(req->run_id, g_run_id, sizeof(g_run_id)))
                g_slac = 3; // success
            else
                g_slac = 0xFF;
        }
    }
    return g_slac;
}

void qca7000Process() {
    uint16_t cause;
    do {
        spiWr16_fast(SPI_REG_INTR_ENABLE, 0);
        cause = spiRd16_fast(SPI_REG_INTR_CAUSE);
        spiWr16_fast(SPI_REG_INTR_CAUSE, cause);
        spiWr16_fast(SPI_REG_INTR_ENABLE, INTR_MASK);

        if (cause & SPI_INT_CPU_ON) {
            hardReset();
            qca7000setup(g_spi, g_cs, g_rst);
            return;
        }
        if (cause & (SPI_INT_WRBUF_ERR | SPI_INT_RDBUF_ERR)) {
            hardReset();
            return;
        }
        if (cause & SPI_INT_PKT_AVLBL)
            fetchRx();

        cause = spiRd16_fast(SPI_REG_INTR_CAUSE);
    } while (cause != 0);
    spiWr16_fast(SPI_REG_INTR_CAUSE, cause);
}

bool qca7000setup(SPIClass* bus, int csPin, int rstPin) {
    ESP_LOGI(PLC_TAG, "QCA7000 setup: bus=%p CS=%d RST=%d", bus, csPin, rstPin);
#ifndef ESP_LOGW
#define ESP_LOGW(tag, fmt, ...)
#endif
    g_spi = bus;
    g_cs = csPin;
    g_rst = rstPin;
    if (g_spi)
        g_spi->begin();
    pinMode(g_cs, OUTPUT);
    digitalWrite(g_cs, HIGH);

    if (!hardReset()) {
        ESP_LOGE(PLC_TAG, "hardReset failed – modem missing");
        return false;
    }

    spiWr16_fast(SPI_REG_INTR_ENABLE, INTR_MASK);
    ESP_LOGI(PLC_TAG, "QCA7000 ready");
#ifndef ESP_LOGW
#define ESP_LOGW(tag, fmt, ...)
#endif
    return true;
}

bool qca7000ResetAndCheck() {
    return hardReset();
}

#ifdef ESP_PLATFORM
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <slac/channel.hpp>

void qca7000_task(void* arg) {
    auto* ctx = static_cast<Qca7000TaskContext*>(arg);
    slac::messages::HomeplugMessage msg;

    while (true) {
        qca7000Process();
        if (ctx && ctx->channel && ctx->channel->poll(msg)) {
            if (ctx->queue) {
                xQueueSend(ctx->queue, &msg, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
#endif
