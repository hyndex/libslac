#ifndef SLAC_QCA7000_LINK_HPP
#define SLAC_QCA7000_LINK_HPP

#ifdef ESP_PLATFORM
#include "port_config.hpp"
#else
#include <port/port_common.hpp>
#endif

#include <slac/ethernet_defs.hpp>
#include "qca7000.hpp"
#include <slac/transport.hpp>

namespace slac {
namespace port {

/**
 * @brief Link implementation for the QCA7000 powerline modem.
 *
 * \note Thread Safety: Qca7000Link is not thread-safe. The caller must
 * serialise access when used from multiple threads.
 */
class Qca7000Link : public transport::Link {
public:
    using ErrorCallback = qca7000_error_cb_t;

    explicit Qca7000Link(const qca7000_config& cfg,
                         ErrorCallback cb = nullptr,
                         void* cb_arg = nullptr);

    void set_error_callback(ErrorCallback cb, void* arg);
    bool fatal_error() const { return fatal_error_flag; }
    void clear_fatal_error() { fatal_error_flag = false; }

    ~Qca7000Link();

    /// Close the underlying bus and reset internal state
    void close();

    bool open() override;
    bool write(const uint8_t* b, size_t l, uint32_t timeout_ms) override;
    /// Poll for a frame and service modem events while waiting
    transport::LinkError read(uint8_t* b, size_t l, size_t* out, uint32_t timeout_ms) override;
    const uint8_t* mac() const override;

    /**
     * @brief Returns true if an initialization error has occurred.
     */
    bool init_failed() const {
        return initialization_error;
    }

    /**
     * @brief Returns true if the link was successfully opened.
     */
    bool is_initialized() const {
        return initialized;
    }

private:
    bool initialized{false};
    bool initialization_error{false};
    bool fatal_error_flag{false};
    ErrorCallback error_cb{nullptr};
    void* error_arg{nullptr};
    qca7000_config cfg;
    uint8_t mac_addr[ETH_ALEN]{};
};

} // namespace port
} // namespace slac

#endif // SLAC_QCA7000_LINK_HPP
