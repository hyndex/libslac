#include <slac/packet_socket_link.hpp>

#include <linux/if_ether.h>
#include <memory>
#include <cstring>
#include <slac/slac.hpp>

namespace slac {

PacketSocketLink::PacketSocketLink(const std::string& if_name) : interface_name(if_name) {}

bool PacketSocketLink::open() {
    ::utils::InterfaceInfo if_info(interface_name);
    if (!if_info.is_valid()) {
        return false;
    }

    socket = std::make_unique<::utils::PacketSocket>(if_info, slac::defs::ETH_P_HOMEPLUG_GREENPHY);
    if (!socket->is_valid()) {
        return false;
    }

    memcpy(mac_addr, if_info.get_mac(), ETH_ALEN);
    return true;
}

bool PacketSocketLink::write(const uint8_t* buf, size_t len, uint32_t timeout_ms) {
    auto res = socket->write(buf, len, static_cast<int>(timeout_ms));
    return res == ::utils::PacketSocket::IOResult::Ok;
}

bool PacketSocketLink::read(uint8_t* buf, size_t len, size_t* out_len, uint32_t timeout_ms) {
    auto res = socket->read(buf, static_cast<int>(timeout_ms));
    if (res == ::utils::PacketSocket::IOResult::Ok) {
        *out_len = socket->get_last_read_size();
        return true;
    }
    if (res == ::utils::PacketSocket::IOResult::Timeout) {
        *out_len = 0;
        return false;
    }
    return false;
}

const uint8_t* PacketSocketLink::mac() const {
    return mac_addr;
}

} // namespace slac
