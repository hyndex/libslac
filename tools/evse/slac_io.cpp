// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 - 2022 Pionix GmbH and Contributors to EVerest
#include "slac_io.hpp"
#include "packet_socket_link.hpp"

#include <stdexcept>
#include <thread>

SlacIO::SlacIO(const std::string& if_name) : link(if_name), slac_channel(&link) {
    if (!slac_channel.open()) {
        throw std::runtime_error("Failed to open channel");
    }
}

void SlacIO::run(std::function<InputHandlerFnType> callback) {
    input_handler = callback;

    running = true;

    loop_thread = std::thread(&SlacIO::loop, this);
}

void SlacIO::quit() {
    if (!running) {
        return;
    }

    running = false;

    loop_thread.join();
}

void SlacIO::loop() {

    while (running) {
        if (slac_channel.poll(incoming_msg)) {
            input_handler(incoming_msg);
            continue;
        }

        if (slac_channel.read(incoming_msg, 10)) {
            input_handler(incoming_msg);
        }
    }
}

void SlacIO::send(slac::messages::HomeplugMessage& msg) {
    // FIXME (aw): handle errors
    slac_channel.write(msg, 1);
}
