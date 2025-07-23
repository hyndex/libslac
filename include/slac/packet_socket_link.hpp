#ifndef SLAC_PACKET_SOCKET_LINK_HPP
#define SLAC_PACKET_SOCKET_LINK_HPP

#include <slac/transport.hpp>
#include <slac/packet_socket.hpp>
#include <memory>

namespace slac {

class PacketSocketLink : public transport::Link {
public:
    explicit PacketSocketLink(const std::string& if_name);
    bool open() override;
    bool write(const uint8_t* buf, size_t len, uint32_t timeout_ms) override;
    bool read(uint8_t* buf, size_t len, size_t* out_len, uint32_t timeout_ms) override;
    const uint8_t* mac() const override;

private:
    std::string interface_name;
    std::unique_ptr<::utils::PacketSocket> socket;
    uint8_t mac_addr[ETH_ALEN]{};
};

} // namespace slac

#endif // SLAC_PACKET_SOCKET_LINK_HPP
