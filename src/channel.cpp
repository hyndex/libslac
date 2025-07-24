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

bool Channel::read(slac::messages::HomeplugMessage& msg, int timeout) {
    did_timeout = false;
    if (!link)
        return false;

    size_t out_len = 0;
    bool ok = link->read(reinterpret_cast<uint8_t*>(msg.get_raw_message_ptr()),
                         sizeof(messages::homeplug_message),
                         &out_len, timeout);
    if (!ok) {
        did_timeout = timeout > 0;
        return false;
    }

    if (out_len < defs::MME_MIN_LENGTH || out_len > sizeof(messages::homeplug_message)) {
        return false;
    }

    msg.set_raw_msg_len(static_cast<int>(out_len));
    return true;
}

bool Channel::poll(slac::messages::HomeplugMessage& msg) {
    did_timeout = false;
    if (!link)
        return false;

    size_t out_len = 0;
    const int timeout = 0;
    bool ok = link->read(reinterpret_cast<uint8_t*>(msg.get_raw_message_ptr()),
                         sizeof(messages::homeplug_message),
                         &out_len, timeout);
    if (!ok) {
        did_timeout = timeout > 0;
        return false;
    }

    if (out_len < defs::MME_MIN_LENGTH || out_len > sizeof(messages::homeplug_message)) {
        return false;
    }

    msg.set_raw_msg_len(static_cast<int>(out_len));
    return true;
}

bool Channel::write(slac::messages::HomeplugMessage& msg, int timeout) {
    assert(msg.is_valid() && "Homeplug message is not valid");
    did_timeout = false;

    if (!link)
        return false;

    auto raw_msg_ether_shost = msg.get_src_mac();
    if (!msg.keep_source_mac()) {
        memcpy(raw_msg_ether_shost, orig_if_mac, sizeof(orig_if_mac));
    }

    return link->write(reinterpret_cast<const uint8_t*>(msg.get_raw_message_ptr()), msg.get_raw_msg_len(), timeout);
}

const uint8_t* Channel::get_mac_addr() {
    return orig_if_mac;
}

} // namespace slac
