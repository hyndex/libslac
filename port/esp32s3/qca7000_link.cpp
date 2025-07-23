#include "qca7000_link.hpp"

#ifdef ESP_PLATFORM
#include "port_config.hpp"
#endif

namespace slac {
namespace port {

bool Qca7000Link::open() {
    // Placeholder for actual initialization
    return true;
}

bool Qca7000Link::write(const uint8_t* b, size_t l, uint32_t) {
    // TODO: send via SPI
    (void)b; (void)l;
    return false;
}

bool Qca7000Link::read(uint8_t* b, size_t l, size_t* out, uint32_t) {
    // TODO: receive via SPI
    (void)b; (void)l; *out = 0;
    return false;
}

const uint8_t* Qca7000Link::mac() const {
    static uint8_t dummy[6] = {0};
    return dummy;
}

} // namespace port
} // namespace slac
