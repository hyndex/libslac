// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Joulepoint Private Limited (Author Chinmoy Bhuyan)
#ifndef SLAC_CHANNEL_HPP
#define SLAC_CHANNEL_HPP

#include "port/port_common.hpp"

#include <slac/transport.hpp>
#include <string>

// SPDX-License-Identifier: Apache-2.0
// Copyright 2020 - 2021 Joulepoint Private Limited (Chinmoy Bhuyan) and Contributors to EVerest
#include <slac/slac.hpp>

namespace slac {

/**
 * \brief Convenience wrapper around a transport::Link.
 *
 * Channel does not manage the lifetime of the underlying Link; the caller must
 * create and maintain the link instance.  The current implementation operates
 * solely on untagged Ethernet frames and assumes that at most one SLAC session
 * is active.  Applications requiring VLAN encapsulation or multiplexing of
 * multiple sessions need to handle those aspects outside of this class.
 *
 * \note Thread Safety: Channel instances are \b not thread-safe.  Concurrent
 * access must be externally synchronized.
 */

class Channel {
public:
    explicit Channel(transport::Link* link);
    ~Channel();

    bool open();
    transport::LinkError read(slac::messages::HomeplugMessage& msg, int timeout);
    bool poll(slac::messages::HomeplugMessage& msg);
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
