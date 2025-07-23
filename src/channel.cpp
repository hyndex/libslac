// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 - 2022 Pionix GmbH and Contributors to EVerest
#include <slac/channel.hpp>
#ifdef ESP_PLATFORM
#include "port/esp32s3/port_config.hpp"
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
    bool ok = link->read(reinterpret_cast<uint8_t*>(msg.get_raw_message_ptr()), sizeof(messages::homeplug_message),
                         &out_len, timeout);
    if (ok) {
        (void)out_len; // TODO handle length
        return true;
    }
    return false;
}

bool Channel::poll(slac::messages::HomeplugMessage& msg) {
    did_timeout = false;
    if (!link)
        return false;

    size_t out_len = 0;
    bool ok = link->read(reinterpret_cast<uint8_t*>(msg.get_raw_message_ptr()), sizeof(messages::homeplug_message),
                         &out_len, 0);
    if (ok) {
        (void)out_len;
        return true;
    }
    return false;
}

bool Channel::write(slac::messages::HomeplugMessage& msg, int timeout) {
    assert(("Homeplug message is not valid\n", msg.is_valid()));
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
