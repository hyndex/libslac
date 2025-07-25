#include "qca7000.hpp"
#include "../port_common.hpp"
#include "port_config.hpp"
#ifdef ESP_LOGW
#pragma push_macro("ESP_LOGW")
#undef ESP_LOGW
#define RESTORE_ESP_LOGW
#endif
#include "../logging_compat.hpp"
#ifdef RESTORE_ESP_LOGW
#pragma pop_macro("ESP_LOGW")
#undef RESTORE_ESP_LOGW
#endif
#include <arpa/inet.h>
#include <stdint.h>
#ifndef ESP_PLATFORM
static inline uint32_t esp_random() {
    return 0x12345678u;
}
#endif
#include <slac/slac.hpp>
#include <slac/iso15118_consts.hpp>
#include <slac/fsm.hpp>
#include <string.h>
#include <atomic>

const char* PLC_TAG = "PLC_IF";

uint8_t myethtransmitbuffer[V2GTP_BUFFER_SIZE]{};
size_t myethtransmitlen = 0;
uint8_t myethreceivebuffer[V2GTP_BUFFER_SIZE]{};
size_t myethreceivelen = 0;

static constexpr uint16_t SIG = 0xAA55;
static constexpr uint16_t WRBUF_RST = 0x0C5B;
static constexpr uint32_t FAST_HZ = QCA7000_SPI_FAST_HZ;
static constexpr uint32_t SLOW_HZ = 1000000;
static constexpr uint16_t SOF_WORD = 0xAAAA;
static constexpr uint16_t EOF_WORD = 0x5555;
static constexpr uint16_t TX_HDR = 8;
static constexpr uint16_t RX_HDR = 12;
static constexpr uint16_t FTR_LEN = 2;
static constexpr size_t BURST_LEN = QCA7000_SPI_BURST_LEN;
static constexpr uint16_t INTR_MASK = SPI_INT_CPU_ON | SPI_INT_PKT_AVLBL | SPI_INT_RDBUF_ERR | SPI_INT_WRBUF_ERR;

using FSMBuffer = slac::fsm::buffer::SwapBuffer<64, 0, 1>;
using FSM = slac::fsm::FSM<slac::SlacEvent, int, FSMBuffer>;

struct SlacContext {
    uint8_t run_id[slac::defs::RUN_ID_LEN]{};
    uint32_t timer{0};
    uint8_t sound_sent{0};
    uint8_t result{0};
    slac::messages::cm_slac_match_req match_req{};
    uint8_t match_src_mac[ETH_ALEN]{};
};

static FSMBuffer g_fsm_buf{};
static SlacContext g_slac_ctx{};
static FSM g_fsm(g_fsm_buf);

struct ErrorCallbackCtx {
    qca7000_error_cb_t cb{nullptr};
    void* arg{nullptr};
    bool* flag{nullptr};
};
static ErrorCallbackCtx g_err_cb;

void qca7000SetErrorCallback(qca7000_error_cb_t cb, void* arg, bool* flag) {
    g_err_cb.cb = cb;
    g_err_cb.arg = arg;
    g_err_cb.flag = flag;
}

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
static constexpr uint8_t RING_SIZE = 4;
static constexpr uint8_t RING_MASK = RING_SIZE - 1;
static RxEntry ring[RING_SIZE];
static std::atomic<uint8_t> head{0}, tail{0};

inline bool ringEmpty() {
    return head.load(std::memory_order_acquire) ==
           tail.load(std::memory_order_acquire);
}

inline void ringPush(const uint8_t* d, size_t l) {
    if (l > V2GTP_BUFFER_SIZE)
        l = V2GTP_BUFFER_SIZE;
    auto h = head.load(std::memory_order_acquire);
    auto t = tail.load(std::memory_order_acquire);
    uint8_t next = (h + 1) & RING_MASK;
    if (next == t) {
        ESP_LOGW(PLC_TAG, "RX ring full - dropping frame");
        return;
    }
    memcpy(ring[h].data, d, l);
    ring[h].len = l;
    head.store(next, std::memory_order_release);
}

inline bool ringPop(const uint8_t** d, size_t* l) {
    auto t = tail.load(std::memory_order_acquire);
    if (head.load(std::memory_order_acquire) == t)
        return false;
    *d = ring[t].data;
    *l = ring[t].len;
    tail.store((t + 1) & RING_MASK, std::memory_order_release);
    return true;
}
} // namespace

static inline uint16_t cmd16(bool rd, bool intr, uint16_t reg) {
    return (rd ? 0x8000u : 0) | (intr ? 0x4000u : 0) |
           (reg & 0x3FFFu);
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

    (void)slowRd16(SPI_REG_SIGNATURE); // dummy read as recommended

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

    t0 = slac_millis();
    while (!(slowRd16(SPI_REG_INTR_CAUSE) & SPI_INT_CPU_ON) && slac_millis() - t0 < 80)
        ;

    spiWr16_fast(SPI_REG_INTR_CAUSE, 0xFFFF);
    spiWr16_fast(SPI_REG_INTR_ENABLE, INTR_MASK);
    return true;
}

static bool softReset() {
    auto slowRd16 = [&](uint16_t reg) -> uint16_t {
        g_spi->beginTransaction(setSlow);
        digitalWrite(g_cs, LOW);
        g_spi->transfer16(cmd16(true, true, reg));
        uint16_t v = g_spi->transfer16(0);
        digitalWrite(g_cs, HIGH);
        g_spi->endTransaction();
        return v;
    };
    auto slowWr16 = [&](uint16_t reg, uint16_t val) {
        g_spi->beginTransaction(setSlow);
        digitalWrite(g_cs, LOW);
        g_spi->transfer16(cmd16(false, true, reg));
        g_spi->transfer16(val);
        digitalWrite(g_cs, HIGH);
        g_spi->endTransaction();
    };

    uint16_t cfg = slowRd16(SPI_REG_SPI_CONFIG);
    slowWr16(SPI_REG_SPI_CONFIG, cfg | QCASPI_SLAVE_RESET_BIT);
    slac_delay(10);

    (void)slowRd16(SPI_REG_SIGNATURE); // dummy read

    uint32_t t0 = slac_millis();
    uint16_t sig = 0, buf = 0;
    do {
        sig = slowRd16(SPI_REG_SIGNATURE);
        buf = slowRd16(SPI_REG_WRBUF_SPC_AVA);
        if (sig == SIG && buf == WRBUF_RST)
            break;
        slac_delay(5);
    } while (slac_millis() - t0 < 200);

    if (sig != SIG || buf != WRBUF_RST)
        return false;

    t0 = slac_millis();
    while (!(slowRd16(SPI_REG_INTR_CAUSE) & SPI_INT_CPU_ON) &&
           slac_millis() - t0 < 80)
        ;
    spiWr16_fast(SPI_REG_INTR_CAUSE, 0xFFFF);
    return true;
}

uint16_t qca7000ReadInternalReg(uint16_t r) {
    return spiRd16_fast(r);
}
bool qca7000ReadSignature(uint16_t* s, uint16_t* v) {
    uint16_t sig = qca7000ReadInternalReg(SPI_REG_SIGNATURE),
             ver = qca7000ReadInternalReg(0x1B00);
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
    g_spi->transfer16(slac::htole16(static_cast<uint16_t>(frameLen)));
    g_spi->transfer16(0);
    if (ethLen) {
        size_t off = 0;
        while (off < ethLen) {
            size_t chunk = ethLen - off;
            if (chunk > BURST_LEN)
                chunk = BURST_LEN;
            g_spi->writeBytes(eth + off, chunk);
            off += chunk;
        }
    }
    if (frameLen > ethLen) {
        uint8_t pad[60]{};
        size_t padLen = frameLen - ethLen;
        size_t off = 0;
        while (off < padLen) {
            size_t chunk = padLen - off;
            if (chunk > BURST_LEN)
                chunk = BURST_LEN;
            g_spi->writeBytes(pad, chunk);
            off += chunk;
        }
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
    uint16_t fl = slac::le16toh(static_cast<uint16_t>((p[9] << 8) | p[8]));
    if (fl > avail - RX_HDR - FTR_LEN)
        return;
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
    size_t store = l;
    if (l > V2GTP_BUFFER_SIZE) {
        ESP_LOGW(PLC_TAG,
                 "RX frame larger than buffer (%zu > %d) - truncating",
                 l, V2GTP_BUFFER_SIZE);
        store = V2GTP_BUFFER_SIZE;
    }
    memcpy(myethreceivebuffer, s, store);
    myethreceivelen = store;
    return c;
}

static uint8_t g_run_id[slac::defs::RUN_ID_LEN]{};
static const uint8_t g_src_mac[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

static bool send_start_atten_char(const SlacContext& ctx);
static bool send_mnbc_sound(const SlacContext& ctx, uint8_t remaining);
static bool send_atten_char_rsp(const SlacContext& ctx,
                                const uint8_t* dst,
                                const slac::messages::cm_atten_char_ind* ind);
static bool send_set_key_cnf(const SlacContext& ctx,
                             const uint8_t* dst,
                             const slac::messages::cm_set_key_req* req);

static bool send_start_atten_char(const SlacContext& ctx) {
    struct __attribute__((packed)) {
        ether_header eth;
        struct {
            uint8_t mmv;
            uint16_t mmtype;
        } hp;
        slac::messages::cm_start_atten_char_ind ind;
    } msg{};

    memset(&msg, 0, sizeof(msg));
    memset(msg.eth.ether_dhost, 0xFF, ETH_ALEN);
    memcpy(msg.eth.ether_shost, g_src_mac, ETH_ALEN);
    msg.eth.ether_type = htons(slac::defs::ETH_P_HOMEPLUG_GREENPHY);
    msg.hp.mmv = static_cast<uint8_t>(slac::defs::MMV::AV_1_0);
    msg.hp.mmtype = slac::htole16(slac::defs::MMTYPE_CM_START_ATTEN_CHAR |
                            slac::defs::MMTYPE_MODE_IND);
    msg.ind.application_type = slac::defs::COMMON_APPLICATION_TYPE;
    msg.ind.security_type = slac::defs::COMMON_SECURITY_TYPE;
    msg.ind.num_sounds = slac::defs::C_EV_MATCH_MNBC;
    msg.ind.timeout = slac::defs::TT_EVSE_MATCH_MNBC_MS / 100;
    msg.ind.resp_type = slac::defs::CM_SLAC_PARM_CNF_RESP_TYPE;
    memcpy(msg.ind.forwarding_sta, g_src_mac, ETH_ALEN);
    memcpy(msg.ind.run_id, ctx.run_id, sizeof(ctx.run_id));
    return txFrame(reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

static bool send_mnbc_sound(const SlacContext& ctx, uint8_t remaining) {
    struct __attribute__((packed)) {
        ether_header eth;
        struct {
            uint8_t mmv;
            uint16_t mmtype;
        } hp;
        slac::messages::cm_mnbc_sound_ind ind;
    } msg{};

    memset(&msg, 0, sizeof(msg));
    memset(msg.eth.ether_dhost, 0xFF, ETH_ALEN);
    memcpy(msg.eth.ether_shost, g_src_mac, ETH_ALEN);
    msg.eth.ether_type = htons(slac::defs::ETH_P_HOMEPLUG_GREENPHY);
    msg.hp.mmv = static_cast<uint8_t>(slac::defs::MMV::AV_1_0);
    msg.hp.mmtype = slac::htole16(slac::defs::MMTYPE_CM_MNBC_SOUND |
                            slac::defs::MMTYPE_MODE_IND);
    msg.ind.application_type = slac::defs::COMMON_APPLICATION_TYPE;
    msg.ind.security_type = slac::defs::COMMON_SECURITY_TYPE;
    memset(msg.ind.sender_id, 0, sizeof(msg.ind.sender_id));
    msg.ind.remaining_sound_count = remaining;
    memcpy(msg.ind.run_id, ctx.run_id, sizeof(ctx.run_id));
    for (uint8_t& b : msg.ind.random)
        b = static_cast<uint8_t>(esp_random() & 0xFF);
    return txFrame(reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

static bool send_atten_char_rsp(const SlacContext& ctx,
                                const uint8_t* dst,
                                const slac::messages::cm_atten_char_ind* ind) {
    struct __attribute__((packed)) {
        ether_header eth;
        struct {
            uint8_t mmv;
            uint16_t mmtype;
        } hp;
        slac::messages::cm_atten_char_rsp rsp;
    } msg{};

    memset(&msg, 0, sizeof(msg));
    memcpy(msg.eth.ether_dhost, dst, ETH_ALEN);
    memcpy(msg.eth.ether_shost, g_src_mac, ETH_ALEN);
    msg.eth.ether_type = htons(slac::defs::ETH_P_HOMEPLUG_GREENPHY);
    msg.hp.mmv = static_cast<uint8_t>(slac::defs::MMV::AV_1_0);
    msg.hp.mmtype = slac::htole16(slac::defs::MMTYPE_CM_ATTEN_CHAR |
                            slac::defs::MMTYPE_MODE_RSP);
    msg.rsp.application_type = slac::defs::COMMON_APPLICATION_TYPE;
    msg.rsp.security_type = slac::defs::COMMON_SECURITY_TYPE;
    memcpy(msg.rsp.source_address, g_src_mac, ETH_ALEN);
    memcpy(msg.rsp.run_id, ind->run_id, sizeof(ind->run_id));
    memcpy(msg.rsp.source_id, ind->source_id, sizeof(ind->source_id));
    memcpy(msg.rsp.resp_id, ind->resp_id, sizeof(ind->resp_id));
    msg.rsp.result = slac::defs::CM_ATTEN_CHAR_RSP_RESULT;
    return txFrame(reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

static bool send_set_key_cnf(const SlacContext& ctx,
                             const uint8_t* dst,
                             const slac::messages::cm_set_key_req* req) {
    struct __attribute__((packed)) {
        ether_header eth;
        struct {
            uint8_t mmv;
            uint16_t mmtype;
        } hp;
        slac::messages::cm_set_key_cnf cnf;
    } msg{};

    memset(&msg, 0, sizeof(msg));
    memcpy(msg.eth.ether_dhost, dst, ETH_ALEN);
    memcpy(msg.eth.ether_shost, g_src_mac, ETH_ALEN);
    msg.eth.ether_type = htons(slac::defs::ETH_P_HOMEPLUG_GREENPHY);
    msg.hp.mmv = static_cast<uint8_t>(slac::defs::MMV::AV_1_0);
    msg.hp.mmtype = slac::htole16(slac::defs::MMTYPE_CM_SET_KEY |
                            slac::defs::MMTYPE_MODE_CNF);
    msg.cnf.result = slac::defs::CM_SET_KEY_CNF_RESULT_SUCCESS;
    msg.cnf.my_nonce = req->my_nonce;
    msg.cnf.your_nonce = req->your_nonce;
    msg.cnf.pid = req->pid;
    msg.cnf.prn = req->prn;
    msg.cnf.pmn = req->pmn;
    msg.cnf.cco_capability = req->cco_capability;
    return txFrame(reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

static bool send_match_cnf(const SlacContext& ctx) {
    struct __attribute__((packed)) {
        ether_header eth;
        struct {
            uint8_t mmv;
            uint16_t mmtype;
        } hp;
        slac::messages::cm_slac_match_cnf cnf;
    } msg{};

    memset(&msg, 0, sizeof(msg));
    memcpy(msg.eth.ether_dhost, ctx.match_src_mac, ETH_ALEN);
    memcpy(msg.eth.ether_shost, g_src_mac, ETH_ALEN);
    msg.eth.ether_type = htons(slac::defs::ETH_P_HOMEPLUG_GREENPHY);
    msg.hp.mmv = static_cast<uint8_t>(slac::defs::MMV::AV_1_0);
    msg.hp.mmtype = slac::htole16(slac::defs::MMTYPE_CM_SLAC_MATCH |
                            slac::defs::MMTYPE_MODE_CNF);
    msg.cnf.application_type = ctx.match_req.application_type;
    msg.cnf.security_type = ctx.match_req.security_type;
    msg.cnf.mvf_length = slac::htole16(slac::defs::CM_SLAC_MATCH_CNF_MVF_LENGTH);
    memcpy(msg.cnf.pev_id, ctx.match_req.pev_id, sizeof(msg.cnf.pev_id));
    memcpy(msg.cnf.pev_mac, ctx.match_req.pev_mac, sizeof(msg.cnf.pev_mac));
    memcpy(msg.cnf.evse_id, ctx.match_req.evse_id, sizeof(msg.cnf.evse_id));
    memcpy(msg.cnf.evse_mac, ctx.match_req.evse_mac, sizeof(msg.cnf.evse_mac));
    memcpy(msg.cnf.run_id, ctx.match_req.run_id, sizeof(msg.cnf.run_id));
    memset(msg.cnf.nid, 0, sizeof(msg.cnf.nid));
    msg.cnf._reserved2 = 0;
    memset(msg.cnf.nmk, 0, sizeof(msg.cnf.nmk));

    return txFrame(reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

// FSM state implementations
struct SoundingState;
struct WaitSetKeyState;
struct WaitMatchState;
struct IdleState : public FSM::SimpleStateType {
    SlacContext& ctx;
    IdleState(SlacContext& c) : ctx(c) {}
    void enter() override { ctx.result = 0; }
    fsm::states::HandleEventResult handle_event(FSM::StateAllocatorType&, slac::SlacEvent) override {
        return FSM::StateAllocatorType::PASS_ON;
    }
};

struct WaitParmCnfState : public FSM::SimpleStateType {
    SlacContext& ctx;
    WaitParmCnfState(SlacContext& c) : ctx(c) {}
    void enter() override { ctx.result = 1; ctx.sound_sent = 0; ctx.timer = slac_millis(); }
    fsm::states::HandleEventResult handle_event(FSM::StateAllocatorType& alloc, slac::SlacEvent ev) override {
        if (ev == slac::SlacEvent::GotParmCnf) {
            send_start_atten_char(ctx);
            ctx.timer = slac_millis();
            ctx.result = 2;
            return alloc.create_simple<SoundingState>(ctx);
        }
        if (ev == slac::SlacEvent::Timeout || ev == slac::SlacEvent::Error) {
            ctx.result = 0xFF;
            return alloc.create_simple<IdleState>(ctx);
        }
        return FSM::StateAllocatorType::PASS_ON;
    }
};

struct SoundingState : public FSM::SimpleStateType {
    SlacContext& ctx;
    SoundingState(SlacContext& c) : ctx(c) {}
    void enter() override { ctx.result = 2; }
    fsm::states::HandleEventResult handle_event(FSM::StateAllocatorType& alloc, slac::SlacEvent ev) override {
        if (ev == slac::SlacEvent::SoundIntervalElapsed) {
            uint8_t remaining = slac::defs::C_EV_MATCH_MNBC - ctx.sound_sent - 1;
            send_mnbc_sound(ctx, remaining);
            ++ctx.sound_sent;
            return FSM::StateAllocatorType::HANDLED_INTERNALLY;
        }
        if (ev == slac::SlacEvent::GotAttenCharInd) {
            ctx.timer = slac_millis();
            ctx.result = 3;
            return alloc.create_simple<WaitSetKeyState>(ctx);
        }
        if (ev == slac::SlacEvent::Timeout || ev == slac::SlacEvent::Error) {
            ctx.result = 0xFF;
            return alloc.create_simple<IdleState>(ctx);
        }
        return FSM::StateAllocatorType::PASS_ON;
    }
};

struct WaitSetKeyState : public FSM::SimpleStateType {
    SlacContext& ctx;
    WaitSetKeyState(SlacContext& c) : ctx(c) {}
    void enter() override { ctx.result = 3; ctx.timer = slac_millis(); }
    fsm::states::HandleEventResult handle_event(FSM::StateAllocatorType& alloc, slac::SlacEvent ev) override {
        if (ev == slac::SlacEvent::GotSetKeyReq) {
            ctx.timer = slac_millis();
            ctx.result = 4;
            return alloc.create_simple<WaitMatchState>(ctx);
        }
        if (ev == slac::SlacEvent::Timeout || ev == slac::SlacEvent::Error) {
            ctx.result = 0xFF;
            return alloc.create_simple<IdleState>(ctx);
        }
        return FSM::StateAllocatorType::PASS_ON;
    }
};

struct WaitMatchState : public FSM::SimpleStateType {
    SlacContext& ctx;
    WaitMatchState(SlacContext& c) : ctx(c) {}
    void enter() override { ctx.result = 4; ctx.timer = slac_millis(); }
    fsm::states::HandleEventResult handle_event(FSM::StateAllocatorType& alloc, slac::SlacEvent ev) override {
        if (ev == slac::SlacEvent::GotMatchReq) {
            send_match_cnf(ctx);
            ctx.result = 5;
            return alloc.create_simple<IdleState>(ctx);
        }
        if (ev == slac::SlacEvent::Timeout || ev == slac::SlacEvent::Error) {
            ctx.result = 0xFF;
            return alloc.create_simple<IdleState>(ctx);
        }
        return FSM::StateAllocatorType::PASS_ON;
    }
};

// Issue a CM_SLAC_PARM.REQ to start the SLAC matching handshake.
bool qca7000startSlac() {
    g_slac_ctx.result = 1;
    g_slac_ctx.timer = slac_millis();
    g_slac_ctx.sound_sent = 0;
    memset(&g_slac_ctx.match_req, 0, sizeof(g_slac_ctx.match_req));
    memset(g_slac_ctx.match_src_mac, 0, sizeof(g_slac_ctx.match_src_mac));

    for (size_t i = 0; i < sizeof(g_slac_ctx.run_id); ++i)
        g_slac_ctx.run_id[i] = static_cast<uint8_t>(esp_random() & 0xFF);

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
    msg.hp.mmtype = slac::htole16(slac::defs::MMTYPE_CM_SLAC_PARAM | slac::defs::MMTYPE_MODE_REQ);
    msg.req.application_type = slac::defs::COMMON_APPLICATION_TYPE;
    msg.req.security_type = slac::defs::COMMON_SECURITY_TYPE;
    memcpy(msg.req.run_id, g_slac_ctx.run_id, sizeof(g_slac_ctx.run_id));

    bool ok = txFrame(reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
    if (ok)
        g_fsm.reset<WaitParmCnfState>(g_slac_ctx);
    else
        g_fsm.reset<IdleState>(g_slac_ctx);
    return ok;
}

// Poll for SLAC confirmation frames and update state accordingly.
uint8_t qca7000getSlacResult() {
    fetchRx();
    const uint32_t now = slac_millis();
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
        uint16_t mmtype = slac::le16toh(*reinterpret_cast<const uint16_t*>(p + 1));
        if (mmv != static_cast<uint8_t>(slac::defs::MMV::AV_1_0))
            continue;

        if (mmtype == (slac::defs::MMTYPE_CM_SLAC_PARAM | slac::defs::MMTYPE_MODE_CNF)) {
            const auto* cnf = reinterpret_cast<const slac::messages::cm_slac_parm_cnf*>(p + 3);
            if (!memcmp(cnf->run_id, g_slac_ctx.run_id, sizeof(g_slac_ctx.run_id))) {
                g_fsm.handle_event(slac::SlacEvent::GotParmCnf);
            } else {
                g_fsm.handle_event(slac::SlacEvent::Error);
            }
        } else if (mmtype == (slac::defs::MMTYPE_CM_ATTEN_CHAR | slac::defs::MMTYPE_MODE_IND)) {
            const auto* ind = reinterpret_cast<const slac::messages::cm_atten_char_ind*>(p + 3);
            if (!memcmp(ind->run_id, g_slac_ctx.run_id, sizeof(g_slac_ctx.run_id))) {
                send_atten_char_rsp(g_slac_ctx, eth->ether_shost, ind);
                g_fsm.handle_event(slac::SlacEvent::GotAttenCharInd);
            } else {
                g_fsm.handle_event(slac::SlacEvent::Error);
            }
        } else if (mmtype == (slac::defs::MMTYPE_CM_SET_KEY | slac::defs::MMTYPE_MODE_REQ)) {
            const auto* req = reinterpret_cast<const slac::messages::cm_set_key_req*>(p + 3);
            send_set_key_cnf(g_slac_ctx, eth->ether_shost, req);
            g_fsm.handle_event(slac::SlacEvent::GotSetKeyReq);
        } else if (mmtype == (slac::defs::MMTYPE_CM_SLAC_MATCH | slac::defs::MMTYPE_MODE_REQ)) {
            const auto* req = reinterpret_cast<const slac::messages::cm_slac_match_req*>(p + 3);
            if (!memcmp(req->run_id, g_slac_ctx.run_id, sizeof(g_slac_ctx.run_id))) {
                memcpy(&g_slac_ctx.match_req, req, sizeof(g_slac_ctx.match_req));
                memcpy(g_slac_ctx.match_src_mac, eth->ether_shost, ETH_ALEN);
                g_fsm.handle_event(slac::SlacEvent::GotMatchReq);
            } else {
                g_fsm.handle_event(slac::SlacEvent::Error);
            }
        }
    }

    if (g_slac_ctx.result == 2) {
        if (g_slac_ctx.sound_sent < slac::defs::C_EV_MATCH_MNBC &&
            now - g_slac_ctx.timer >= slac::defs::TP_EV_BATCH_MSG_INTERVAL_MS) {
            g_fsm.handle_event(slac::SlacEvent::SoundIntervalElapsed);
            g_slac_ctx.timer = now;
            if (g_slac_ctx.sound_sent == slac::defs::C_EV_MATCH_MNBC)
                g_slac_ctx.timer = now; // start wait for ATTEN_CHAR
        }
        if (g_slac_ctx.sound_sent == slac::defs::C_EV_MATCH_MNBC &&
            now - g_slac_ctx.timer > slac::defs::TT_EV_ATTEN_RESULTS_MS)
            g_fsm.handle_event(slac::SlacEvent::Timeout);
    } else if (g_slac_ctx.result == 3 && now - g_slac_ctx.timer > slac::defs::TT_MATCH_SEQUENCE_MS) {
        g_fsm.handle_event(slac::SlacEvent::Timeout);
    } else if (g_slac_ctx.result == 4 && now - g_slac_ctx.timer > slac::defs::TT_MATCH_JOIN_MS) {
        g_fsm.handle_event(slac::SlacEvent::Timeout);
    }

    return g_slac_ctx.result;
}

void qca7000Process() {
    spiWr16_fast(SPI_REG_INTR_ENABLE, 0);
    while (true) {
        uint16_t cause = spiRd16_fast(SPI_REG_INTR_CAUSE);
        if (!cause)
            break;

        uint16_t clear_mask = 0;

        if (cause & SPI_INT_CPU_ON) {
            clear_mask |= SPI_INT_CPU_ON;
            hardReset();
            if (g_err_cb.flag)
                *g_err_cb.flag = true;
            if (g_err_cb.cb)
                g_err_cb.cb(g_err_cb.arg);
            qca7000setup(g_spi, g_cs, g_rst);
            spiWr16_fast(SPI_REG_INTR_CAUSE, clear_mask);
            spiWr16_fast(SPI_REG_INTR_ENABLE, INTR_MASK);
            return;
        }
        if (cause & (SPI_INT_WRBUF_ERR | SPI_INT_RDBUF_ERR)) {
            clear_mask |= SPI_INT_WRBUF_ERR | SPI_INT_RDBUF_ERR;
            hardReset();
            if (g_err_cb.flag)
                *g_err_cb.flag = true;
            if (g_err_cb.cb)
                g_err_cb.cb(g_err_cb.arg);
            spiWr16_fast(SPI_REG_INTR_CAUSE, clear_mask);
            spiWr16_fast(SPI_REG_INTR_ENABLE, INTR_MASK);
            return;
        }
        if (cause & SPI_INT_PKT_AVLBL) {
            fetchRx();
            clear_mask |= SPI_INT_PKT_AVLBL;
        }

        if (clear_mask)
            spiWr16_fast(SPI_REG_INTR_CAUSE, clear_mask);
    }
    spiWr16_fast(SPI_REG_INTR_ENABLE, INTR_MASK);
}

bool qca7000setup(SPIClass* bus, int csPin, int rstPin) {
    ESP_LOGI(PLC_TAG, "QCA7000 setup: bus=%p CS=%d RST=%d", bus, csPin, rstPin);
    g_spi = bus;
    g_cs = csPin;
    g_rst = rstPin;
    if (g_spi)
        g_spi->begin(PLC_SPI_SCK_PIN, PLC_SPI_MISO_PIN, PLC_SPI_MOSI_PIN, g_cs);
    pinMode(g_cs, OUTPUT);
    digitalWrite(g_cs, HIGH);

    if (!hardReset()) {
        ESP_LOGE(PLC_TAG, "hardReset failed – modem missing");
        return false;
    }
    ESP_LOGI(PLC_TAG, "QCA7000 ready");
    return true;
}

void qca7000teardown() {
    if (g_spi) {
        g_spi->end();
    }
    g_spi = nullptr;
}

bool qca7000ResetAndCheck() {
    return hardReset();
}

bool qca7000SoftReset() {
    return softReset();
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
