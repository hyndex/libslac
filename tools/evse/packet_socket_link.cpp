#include "packet_socket_link.hpp"

PacketSocketLink::PacketSocketLink(const std::string& if_name) : if_name(if_name) {
}

bool PacketSocketLink::open() {
    if_info = std::make_unique<utils::InterfaceInfo>(if_name);
    if (!if_info->is_valid()) {
        return false;
    }
    socket = std::make_unique<utils::PacketSocket>(*if_info, slac::defs::ETH_P_HOMEPLUG_GREENPHY);
    return socket->is_valid();
}

bool PacketSocketLink::write(const uint8_t* buf, size_t len, uint32_t timeout_ms) {
    if (!socket)
        return false;
    return socket->write(buf, len, timeout_ms) == utils::PacketSocket::IOResult::Ok;
}

bool PacketSocketLink::read(uint8_t* buf, size_t len, size_t* out_len, uint32_t timeout_ms) {
    if (!socket) {
        *out_len = 0;
        return false;
    }
    auto res = socket->read(buf, timeout_ms);
    if (res == utils::PacketSocket::IOResult::Ok) {
        *out_len = socket->get_last_read_size();
        return true;
    }
    *out_len = 0;
    return false;
}

const uint8_t* PacketSocketLink::mac() const {
    if (!if_info)
        return nullptr;
    return if_info->get_mac();
}

