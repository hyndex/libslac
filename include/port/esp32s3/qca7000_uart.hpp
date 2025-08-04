#pragma once

#ifdef ESP_PLATFORM
#include "port_config.hpp"
#include "driver/uart.h"
#else
using uart_port_t = int;
#endif

#include "ethernet_defs.hpp"
#ifndef V2GTP_BUFFER_SIZE
#define V2GTP_BUFFER_SIZE 1536
#endif

static_assert(ETH_FRAME_LEN <= V2GTP_BUFFER_SIZE,
              "ETH_FRAME_LEN must not exceed V2GTP_BUFFER_SIZE");
#include <slac/transport.hpp>

struct qca7000_uart_config {
    uart_port_t uart_num;
    uint32_t baud;
    const uint8_t* mac_addr{nullptr};
};

void uartQca7000Teardown();

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

    /// Close the UART connection and reset internal state
    void close();

    ~Qca7000UartLink();

private:
    bool initialized{false};
    bool initialization_error{false};
    qca7000_uart_config cfg;
    uint8_t mac_addr[ETH_ALEN]{};
};

} // namespace port
} // namespace slac
