#ifndef SLAC_QCA7000_LINK_HPP
#define SLAC_QCA7000_LINK_HPP

#include "../port_config.hpp"
#ifdef ESP_PLATFORM
#include "port_config.hpp"
#endif

#include "ethernet_defs.hpp"
#include "qca7000.hpp"
#include <slac/transport.hpp>

namespace slac {
namespace port {

class Qca7000Link : public transport::Link {
public:
    explicit Qca7000Link(const qca7000_config& cfg);

    bool open() override;
    bool write(const uint8_t* b, size_t l, uint32_t timeout_ms) override;
    bool read(uint8_t* b, size_t l, size_t* out, uint32_t timeout_ms) override;
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
    qca7000_config cfg;
    uint8_t mac_addr[ETH_ALEN]{};
};

} // namespace port
} // namespace slac

#endif // SLAC_QCA7000_LINK_HPP
