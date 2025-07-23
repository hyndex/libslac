#ifndef SLAC_QCA7000_LINK_HPP
#define SLAC_QCA7000_LINK_HPP

#ifdef ESP_PLATFORM
#include "port_config.hpp"
#endif

#include <slac/transport.hpp>
#include "ethernet_defs.hpp"
#include "qca7000.hpp"

namespace slac {
namespace port {

class Qca7000Link : public transport::Link {
public:
    explicit Qca7000Link(const qca7000_config& cfg);

    bool open() override;
    bool write(const uint8_t* b, size_t l, uint32_t timeout_ms) override;
    bool read(uint8_t* b, size_t l, size_t* out, uint32_t timeout_ms) override;
    const uint8_t* mac() const override;

private:
    bool initialized{false};
    qca7000_config cfg;
    uint8_t mac_addr[ETH_ALEN]{};
};

} // namespace port
} // namespace slac

#endif // SLAC_QCA7000_LINK_HPP
