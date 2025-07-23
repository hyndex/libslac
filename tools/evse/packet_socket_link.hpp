#ifndef PACKET_SOCKET_LINK_HPP
#define PACKET_SOCKET_LINK_HPP

#include <memory>
#include <slac/transport.hpp>
#include "../../src/packet_socket.hpp"

class PacketSocketLink : public slac::transport::Link {
public:
    explicit PacketSocketLink(const std::string& if_name);
    bool open() override;
    bool write(const uint8_t* buf, size_t len, uint32_t timeout_ms) override;
    bool read(uint8_t* buf, size_t len, size_t* out_len, uint32_t timeout_ms) override;
    const uint8_t* mac() const override;

private:
    std::string if_name;
    std::unique_ptr<utils::InterfaceInfo> if_info;
    std::unique_ptr<utils::PacketSocket> socket;
};

#endif // PACKET_SOCKET_LINK_HPP
