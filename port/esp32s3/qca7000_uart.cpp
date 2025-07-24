#include "qca7000_uart.hpp"
#include "qca7000.hpp"
#include "port_config.hpp"
#ifdef ESP_PLATFORM
#include <esp_log.h>
#else
#ifndef ESP_LOGI
#define ESP_LOGI(tag, fmt, ...)
#endif
#ifndef ESP_LOGW
#define ESP_LOGW(tag, fmt, ...)
#endif
#ifndef ESP_LOGE
#define ESP_LOGE(tag, fmt, ...)
#endif
#endif
#include <string.h>




static constexpr uint16_t SOF_WORD = 0xAAAA;
static constexpr uint16_t EOF_WORD = 0x5555;
static constexpr uint16_t TX_HDR = 8;
static constexpr uint16_t RX_HDR = 8;
static constexpr uint16_t FTR_LEN = 2;

#ifdef LIBSLAC_TESTING
HardwareSerial* g_serial = nullptr;
#else
static HardwareSerial* g_serial = nullptr;
#endif

namespace {
struct RxEntry {
    size_t len;
    uint8_t data[V2GTP_BUFFER_SIZE];
};
static RxEntry ring[4];
static volatile uint8_t head = 0, tail = 0;
inline bool ringEmpty() { return head == tail; }
static uint32_t last_rx_time = 0;
static uint32_t frame_timeout_ms = 0;
inline void ringPush(const uint8_t* d, size_t l) {
    slac_noInterrupts();
    if (l > V2GTP_BUFFER_SIZE)
        l = V2GTP_BUFFER_SIZE;
    uint8_t next = (head + 1) & 3;
    if (next == tail) {
        slac_interrupts();
        ESP_LOGW(PLC_TAG, "RX ring full - dropping frame");
        return;
    }
    memcpy(ring[head].data, d, l);
    ring[head].len = l;
    head = next;
    slac_interrupts();
}
inline bool ringPop(const uint8_t** d, size_t* l) {
    slac_noInterrupts();
    if (ringEmpty()) {
        slac_interrupts();
        return false;
    }
    *d = ring[tail].data;
    *l = ring[tail].len;
    tail = (tail + 1) & 3;
    slac_interrupts();
    return true;
}

static enum State { WAIT_SOF, LEN1, LEN2, RSVD1, RSVD2, PAYLOAD, EOF1, EOF2 } state = WAIT_SOF;
static size_t sof_count = 0;
static uint16_t rx_len = 0;
static uint16_t rx_pos = 0;
static uint8_t rx_buf[V2GTP_BUFFER_SIZE];

inline void processByte(uint8_t b) {
    switch (state) {
    case WAIT_SOF:
        if (b == 0xAA) {
            if (++sof_count == 4) {
                sof_count = 0;
                state = LEN1;
            }
        } else {
            sof_count = 0;
        }
        break;
    case LEN1:
        rx_len = b;
        state = LEN2;
        break;
    case LEN2:
        rx_len |= static_cast<uint16_t>(b) << 8;
        if (rx_len > V2GTP_BUFFER_SIZE) {
            state = WAIT_SOF;
            break;
        }
        state = RSVD1;
        break;
    case RSVD1:
        state = RSVD2;
        break;
    case RSVD2:
        rx_pos = 0;
        state = PAYLOAD;
        break;
    case PAYLOAD:
        rx_buf[rx_pos++] = b;
        if (rx_pos >= rx_len)
            state = EOF1;
        break;
    case EOF1:
        if (b == 0x55)
            state = EOF2;
        else
            state = WAIT_SOF;
        break;
    case EOF2:
        if (b == 0x55)
            ringPush(rx_buf, rx_len);
        state = WAIT_SOF;
        break;
    }
}

inline void pollRx() {
    uint32_t now = slac_millis();
    if (state != WAIT_SOF && frame_timeout_ms &&
        now - last_rx_time > frame_timeout_ms) {
        state = WAIT_SOF;
        sof_count = 0;
        rx_pos = 0;
    }
    while (g_serial && g_serial->available()) {
        int v = g_serial->read();
        if (v < 0)
            break;
        processByte(static_cast<uint8_t>(v));
        last_rx_time = slac_millis();
    }
}
} // namespace

#ifdef LIBSLAC_TESTING
bool uartTxFrame(const uint8_t* eth, size_t ethLen) {
#else
static bool uartTxFrame(const uint8_t* eth, size_t ethLen) {
#endif
    if (!g_serial || ethLen > 1522)
        return false;
    size_t frameLen = ethLen;
    if (frameLen < 60)
        frameLen = 60;
    uint8_t hdr[TX_HDR];
    hdr[0] = hdr[1] = hdr[2] = hdr[3] = 0xAA;
    hdr[4] = frameLen & 0xFF;
    hdr[5] = (frameLen >> 8) & 0xFF;
    hdr[6] = 0;
    hdr[7] = 0;
    g_serial->write(hdr, TX_HDR);
    if (ethLen)
        g_serial->write(eth, ethLen);
    if (frameLen > ethLen) {
        uint8_t pad[60]{};
        g_serial->write(pad, frameLen - ethLen);
    }
    uint8_t eof[2] = {0x55, 0x55};
    g_serial->write(eof, 2);
    return true;
}

#ifdef LIBSLAC_TESTING
void uartFetchRx() {
#else
static void uartFetchRx() {
#endif
    pollRx();
}

bool uartQCA7000SendEthFrame(const uint8_t* f, size_t l) {
    bool ok = uartTxFrame(f, l);
    if (ok && l <= V2GTP_BUFFER_SIZE) {
        memcpy(myethtransmitbuffer, f, l);
        myethtransmitlen = l;
    }
    return ok;
}

size_t uartQCA7000checkForReceivedData(uint8_t* d, size_t m) {
    pollRx();
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

namespace slac {
namespace port {

Qca7000UartLink::Qca7000UartLink(const qca7000_uart_config& c) : cfg(c) {
    memset(mac_addr, 0, sizeof(mac_addr));
}

bool Qca7000UartLink::open() {
    if (initialized)
        return true;
    if (initialization_error)
        return false;

    g_serial = cfg.serial ? cfg.serial : &Serial;
#ifdef ARDUINO
    if (g_serial) {
        uint32_t baud = cfg.baud ? cfg.baud : 115200;
        g_serial->begin(baud);
        frame_timeout_ms =
            static_cast<uint32_t>(((V2GTP_BUFFER_SIZE + TX_HDR + FTR_LEN) * 10ULL * 1000ULL + baud - 1) / baud);
    }
#else
    uint32_t baud = cfg.baud ? cfg.baud : 115200;
    frame_timeout_ms =
        static_cast<uint32_t>(((V2GTP_BUFFER_SIZE + TX_HDR + FTR_LEN) * 10ULL * 1000ULL + baud - 1) / baud);
#endif
    last_rx_time = slac_millis();
    if (cfg.mac_addr)
        memcpy(mac_addr, cfg.mac_addr, ETH_ALEN);
    else {
        const uint8_t def_mac[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
        memcpy(mac_addr, def_mac, ETH_ALEN);
    }
    initialized = true;
    return true;
}

bool Qca7000UartLink::write(const uint8_t* b, size_t l, uint32_t) {
    if (!initialized || initialization_error)
        return false;
    return uartQCA7000SendEthFrame(b, l);
}

bool Qca7000UartLink::read(uint8_t* b, size_t l, size_t* out, uint32_t timeout_ms) {
    if (!initialized || initialization_error) {
        *out = 0;
        return false;
    }
    uint32_t start = slac_millis();
    do {
        size_t got = uartQCA7000checkForReceivedData(b, l);
        if (got) {
            *out = got;
            return true;
        }
        if (timeout_ms == 0)
            break;
        slac_delay(1);
    } while (slac_millis() - start < timeout_ms);
    *out = 0;
    return false;
}

const uint8_t* Qca7000UartLink::mac() const {
    return mac_addr;
}

} // namespace port
} // namespace slac

