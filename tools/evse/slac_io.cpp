// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 - 2022 Pionix GmbH and Contributors to EVerest
#include "slac_io.hpp"
#ifdef ESP_PLATFORM
#include <port/esp32s3/qca7000_link.hpp>
#else
#include <slac/packet_socket_link.hpp>
#endif

#include <stdexcept>
#ifdef ESP_PLATFORM
#include <freertos/task.h>
#else
#include <thread>
#endif

#ifdef ESP_PLATFORM
SlacIO::SlacIO(const slac::port::qca7000_config& cfg) : link(cfg), slac_channel(&link) {
#else
SlacIO::SlacIO(const std::string& if_name) : link(if_name), slac_channel(&link) {
#endif
    if (!slac_channel.open()) {
        throw std::runtime_error("Failed to open channel");
    }
}

void SlacIO::release_input() {
    input_handler = nullptr;
}

void SlacIO::run(std::function<InputHandlerFnType> callback, bool spawn_background) {
    input_handler = std::move(callback);
    running = true;

    if (!spawn_background) {
        return;
    }
#ifdef ESP_PLATFORM
    xTaskCreate([](void* arg) {
        static_cast<SlacIO*>(arg)->loop();
    }, "slac_io", 4096, this, 5, &loop_task);
#else
    loop_thread = std::thread(&SlacIO::loop, this);
#endif
}

void SlacIO::quit() {
    if (!running) {
        return;
    }

    running = false;

#ifdef ESP_PLATFORM
    if (loop_task) {
        while (eTaskGetState(loop_task) != eDeleted) {
            vTaskDelay(1);
        }
        loop_task = nullptr;
    }
#else
    loop_thread.join();
#endif
}

void SlacIO::loop() {

    while (running) {
        process(10);
    }
}

void SlacIO::process(int timeout_ms) {
    if (slac_channel.poll(incoming_msg)) {
        input_handler(incoming_msg);
        return;
    }

    if (slac_channel.read(incoming_msg, timeout_ms)) {
        input_handler(incoming_msg);
    }
}

void SlacIO::send(slac::messages::HomeplugMessage& msg) {
    // FIXME (aw): handle errors
    slac_channel.write(msg, 1);
}
