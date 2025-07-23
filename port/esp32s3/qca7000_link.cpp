#ifdef ESP_PLATFORM
#include "port_config.hpp"
#endif
#include "qca7000_link.hpp"
#include "qca7000.hpp"

namespace slac {
namespace port {

Qca7000Link::Qca7000Link(const qca7000_config& cfg) : cfg_(cfg) {
    memcpy(mac_addr, cfg_.mac, ETH_ALEN);
}

bool Qca7000Link::open() {
    if (initialized)
        return true;

    if (!qca7000setup(cfg_.spi, cfg_.cs_pin))
        return false;

    memcpy(mac_addr, cfg_.mac, ETH_ALEN);
    initialized = true;
    return true;
}

bool Qca7000Link::write(const uint8_t* b, size_t l, uint32_t) {
    if (!initialized)
        return false;
    return spiQCA7000SendEthFrame(b, l);
}

bool Qca7000Link::read(uint8_t* b, size_t l, size_t* out, uint32_t timeout_ms) {
    if (!initialized) {
        *out = 0;
        return false;
    }
    uint32_t start = millis();
    do {
        size_t got = spiQCA7000checkForReceivedData(b, l);
        if (got) {
            *out = got;
            return true;
        }
        if (timeout_ms == 0)
            break;
        delay(1);
    } while (millis() - start < timeout_ms);
    *out = 0;
    return false;
}

const uint8_t* Qca7000Link::mac() const {
    return mac_addr;
}

} // namespace port
} // namespace slac
