#ifndef SLAC_QCA7000_LINK_HPP
#define SLAC_QCA7000_LINK_HPP

#include <slac/transport.hpp>

namespace slac {
namespace port {

class Qca7000Link : public transport::Link {
public:
    bool open() override;
    bool write(const uint8_t* b, size_t l, uint32_t timeout_ms) override;
    bool read(uint8_t* b, size_t l, size_t* out, uint32_t timeout_ms) override;
    const uint8_t* mac() const override;
};

} // namespace port
} // namespace slac

#endif // SLAC_QCA7000_LINK_HPP
