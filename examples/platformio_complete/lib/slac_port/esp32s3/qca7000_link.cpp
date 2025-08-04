#include "qca7000_link.hpp"
#include <port/port_common.hpp>
#include "port_config.hpp"
#include "qca7000.hpp"
#include <cstring>

namespace slac {
namespace port {

Qca7000Link::Qca7000Link(const qca7000_config& c,
                         ErrorCallback cb,
                         void* cb_arg)
    : error_cb(cb), error_arg(cb_arg), cfg(c) {
    memset(mac_addr, 0, sizeof(mac_addr));
    if (cfg.mac_addr)
        memcpy(mac_addr, cfg.mac_addr, ETH_ALEN);
    else {
        const uint8_t def_mac[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
        memcpy(mac_addr, def_mac, ETH_ALEN);
    }
    qca7000SetMac(mac_addr);
}

Qca7000Link::~Qca7000Link() {
    close();
    qca7000SetErrorCallback(nullptr, nullptr, nullptr);
}

void Qca7000Link::set_error_callback(ErrorCallback cb, void* arg) {
    error_cb = cb;
    error_arg = arg;
    qca7000SetErrorCallback(error_cb, error_arg, &fatal_error_flag);
}

bool Qca7000Link::open() {
    if (initialized)
        return true;
    if (initialization_error)
        return false;

    spi_device_handle_t bus = cfg.spi;
    int cs = cfg.cs_pin ? cfg.cs_pin : PLC_SPI_CS_PIN;
    int rst = cfg.rst_pin ? cfg.rst_pin : PLC_SPI_RST_PIN;

    if (ETH_FRAME_LEN > V2GTP_BUFFER_SIZE || !bus) {
        initialization_error = true;
        return false;
    }

    if (!qca7000setup(bus, cs, rst)) {
        initialization_error = true;
        return false;
    }
    qca7000SetErrorCallback(error_cb, error_arg, &fatal_error_flag);
    qca7000SetMac(mac_addr);

    initialized = true;
    return true;
}

bool Qca7000Link::write(const uint8_t* b, size_t l, uint32_t) {
    if (!initialized || initialization_error)
        return false;
    return spiQCA7000SendEthFrame(b, l);
}

void Qca7000Link::close() {
    if (!initialized)
        return;
    qca7000teardown();
    initialized = false;
}

transport::LinkError Qca7000Link::read(uint8_t* b, size_t l, size_t* out, uint32_t timeout_ms) {
    if (!initialized || initialization_error) {
        *out = 0;
        return transport::LinkError::Transport;
    }
    uint32_t start = slac_millis();
    do {
        size_t got = spiQCA7000checkForReceivedData(b, l);
        if (got) {
            *out = got;
            return transport::LinkError::Ok;
        }
        if (timeout_ms == 0)
            break;
        slac_delay(1);
    } while (slac_millis() - start < timeout_ms);
    *out = 0;
    return transport::LinkError::Timeout;
}

const uint8_t* Qca7000Link::mac() const {
    return mac_addr;
}

} // namespace port
} // namespace slac
