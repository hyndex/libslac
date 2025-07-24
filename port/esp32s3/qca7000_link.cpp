#include "qca7000_link.hpp"
#include "../port_config.hpp"
#include "port_config.hpp"
#include "qca7000.hpp"
#include <cstring>

namespace slac {
namespace port {

Qca7000Link::Qca7000Link(const qca7000_config& c) : cfg(c) {
    memset(mac_addr, 0, sizeof(mac_addr));
}

bool Qca7000Link::open() {
    if (initialized)
        return true;
    if (initialization_error)
        return false;

    SPIClass* bus = cfg.spi ? cfg.spi : &SPI;
    int cs = cfg.cs_pin ? cfg.cs_pin : PLC_SPI_CS_PIN;
    int rst = cfg.rst_pin ? cfg.rst_pin : PLC_SPI_RST_PIN;

    if (!qca7000setup(bus, cs, rst)) {
        initialization_error = true;
        return false;
    }

    if (cfg.mac_addr)
        memcpy(mac_addr, cfg.mac_addr, ETH_ALEN);
    else {
        const uint8_t def_mac[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
        memcpy(mac_addr, def_mac, ETH_ALEN);
    }
    initialized = true;
    return true;
}

bool Qca7000Link::write(const uint8_t* b, size_t l, uint32_t) {
    if (!initialized || initialization_error)
        return false;
    return spiQCA7000SendEthFrame(b, l);
}

bool Qca7000Link::read(uint8_t* b, size_t l, size_t* out, uint32_t timeout_ms) {
    if (!initialized || initialization_error) {
        *out = 0;
        return false;
    }
    uint32_t start = slac_millis();
    do {
        size_t got = spiQCA7000checkForReceivedData(b, l);
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

const uint8_t* Qca7000Link::mac() const {
    return mac_addr;
}

} // namespace port
} // namespace slac
