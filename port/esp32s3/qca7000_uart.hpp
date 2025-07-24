#pragma once

#ifdef ESP_PLATFORM
#include "port_config.hpp"
#endif

#include "ethernet_defs.hpp"
#ifndef V2GTP_BUFFER_SIZE
#define V2GTP_BUFFER_SIZE 1536
#endif
#include <slac/transport.hpp>

#ifdef ARDUINO
#include <HardwareSerial.h>
#endif

struct qca7000_uart_config {
    HardwareSerial* serial;
    uint32_t baud;
    const uint8_t* mac_addr{nullptr};
};

namespace slac {
namespace port {

class Qca7000UartLink : public transport::Link {
public:
    explicit Qca7000UartLink(const qca7000_uart_config& cfg);

    bool open() override;
    bool write(const uint8_t* b, size_t l, uint32_t timeout_ms) override;
    transport::LinkError read(uint8_t* b, size_t l, size_t* out, uint32_t timeout_ms) override;
    const uint8_t* mac() const override;

    bool init_failed() const {
        return initialization_error;
    }
    bool is_initialized() const {
        return initialized;
    }

private:
    bool initialized{false};
    bool initialization_error{false};
    qca7000_uart_config cfg;
    uint8_t mac_addr[ETH_ALEN]{};
};

} // namespace port
} // namespace slac
