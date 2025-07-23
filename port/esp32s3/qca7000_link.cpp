#include "qca7000_link.hpp"
#include "qca7000.hpp"

#ifdef ESP_PLATFORM
#include "port_config.hpp"
#endif

namespace slac {
namespace port {

Qca7000Link::Qca7000Link() {
    memset(mac_addr, 0, sizeof(mac_addr));
}

bool Qca7000Link::open() {
    if (initialized)
        return true;

    if (!qca7000setup(&SPI, /*CS*/ PLC_SPI_CS_PIN))
        return false;

    // use a fixed MAC address for now
    const uint8_t def_mac[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
    memcpy(mac_addr, def_mac, ETH_ALEN);
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
