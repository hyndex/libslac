#ifndef SLAC_TRANSPORT_HPP
#define SLAC_TRANSPORT_HPP

#include "port/port_common.hpp"

#include <cstddef>
#include <cstdint>

namespace slac::transport {

enum class LinkError {
    Ok,
    Timeout,
    Transport,
};

class Link {
public:
    virtual bool open() = 0;
    virtual bool write(const uint8_t* buf, size_t len, uint32_t timeout_ms) = 0;
    virtual LinkError read(uint8_t* buf, size_t len, size_t* out_len, uint32_t timeout_ms) = 0;
    virtual const uint8_t* mac() const = 0;
    virtual ~Link() = default;
};
} // namespace slac::transport

#endif // SLAC_TRANSPORT_HPP
