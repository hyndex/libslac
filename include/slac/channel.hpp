// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 - 2022 Pionix GmbH and Contributors to EVerest
#ifndef SLAC_CHANNEL_HPP
#define SLAC_CHANNEL_HPP

#ifdef ESP_PLATFORM
#include "port/esp32s3/port_config.hpp"
#endif

#include <string>
#include <slac/transport.hpp>

// SPDX-License-Identifier: Apache-2.0
// Copyright 2020 - 2021 Pionix GmbH and Contributors to EVerest
#include <slac/slac.hpp>



namespace slac {

// TODO (aw):
//  - do we need to handle VLAN tags?
//  - probably we need to handle different sessions ...
//  - channel could own the interface handle and pass it to the packet
//    socket

class Channel {
public:
    explicit Channel(transport::Link* link);
    ~Channel();

    bool open();
    bool read(slac::messages::HomeplugMessage& msg, int timeout);
    bool write(slac::messages::HomeplugMessage& msg, int timeout);

    const std::string& get_error() const {
        return error;
    }

    bool got_timeout() const {
        return did_timeout;
    }

    const uint8_t* get_mac_addr();

private:
    transport::Link* link;
    uint8_t orig_if_mac[ETH_ALEN]{};

    std::string error;
    bool did_timeout{false};
};

} // namespace slac

#endif // SLAC_CHANNEL_HPP
