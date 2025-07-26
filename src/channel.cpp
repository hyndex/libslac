// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Joulepoint Private Limited (Author Chinmoy Bhuyan)
#include <slac/channel.hpp>
#ifdef ESP_PLATFORM
#include "port/port_common.hpp"
#endif

#include <algorithm>
#include <cassert>
#include <cstring>

#include <slac/transport.hpp>

namespace slac {

Channel::Channel(transport::Link* l) : link(l) {
}

bool Channel::open() {
    did_timeout = false;
    if (!link)
        return false;

    if (!link->open())
        return false;

    memcpy(orig_if_mac, link->mac(), sizeof(orig_if_mac));
    return true;
}

Channel::~Channel() = default;

transport::LinkError Channel::read(slac::messages::HomeplugMessage& msg, int timeout) {
    did_timeout = false;
    if (!link)
        return transport::LinkError::Transport;

    size_t out_len = 0;
    auto res = link->read(reinterpret_cast<uint8_t*>(msg.get_raw_message_ptr()),
                          sizeof(messages::homeplug_message), &out_len, timeout);
    did_timeout = res == transport::LinkError::Timeout;
    if (res != transport::LinkError::Ok) {
        return res;
    }
    if (out_len == 0 && timeout > 0) {
        return transport::LinkError::Timeout;
    }

    if (out_len < defs::MME_MIN_LENGTH || out_len > sizeof(messages::homeplug_message)) {
        return transport::LinkError::Transport;
    }

    msg.set_raw_msg_len(static_cast<int>(out_len));
    return transport::LinkError::Ok;
}

bool Channel::poll(slac::messages::HomeplugMessage& msg) {
    did_timeout = false;
    if (!link) {
        error = "No transport link";
        return false;
    }

    size_t out_len = 0;
    const int timeout = 0;
    auto res = link->read(reinterpret_cast<uint8_t*>(msg.get_raw_message_ptr()),
                          sizeof(messages::homeplug_message), &out_len, timeout);
    did_timeout = res == transport::LinkError::Timeout;
    if (res != transport::LinkError::Ok) {
        switch (res) {
        case transport::LinkError::Timeout:
            error = "Timeout";
            break;
        case transport::LinkError::Transport:
            error = "Transport error";
            break;
        default:
            error = "Link error";
            break;
        }
        return false;
    }

    if (out_len < defs::MME_MIN_LENGTH || out_len > sizeof(messages::homeplug_message)) {
        error = "Invalid frame length";
        return false;
    }

    msg.set_raw_msg_len(static_cast<int>(out_len));
    return true;
}

transport::LinkError Channel::write(slac::messages::HomeplugMessage& msg, int timeout) {
    did_timeout = false;

    if (!link) {
        error = "No transport link";
        return transport::LinkError::Transport;
    }

    if (!msg.is_valid()) {
        error = "Invalid HomeplugMessage";
        return transport::LinkError::Transport;
    }

    auto raw_msg_ether_shost = msg.get_src_mac();
    if (!msg.keep_source_mac()) {
        memcpy(raw_msg_ether_shost, orig_if_mac, sizeof(orig_if_mac));
    }

    auto res = link->write(reinterpret_cast<const uint8_t*>(msg.get_raw_message_ptr()),
                           msg.get_raw_msg_len(), timeout);
    did_timeout = res == transport::LinkError::Timeout;
    if (res != transport::LinkError::Ok) {
        switch (res) {
        case transport::LinkError::Timeout:
            error = "Timeout";
            break;
        case transport::LinkError::Transport:
        default:
            error = "Write failed";
            break;
        }
        return res;
    }

    return transport::LinkError::Ok;
}

const uint8_t* Channel::get_mac_addr() {
    return orig_if_mac;
}

} // namespace slac
