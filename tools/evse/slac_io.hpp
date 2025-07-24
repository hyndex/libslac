// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 - 2022 Pionix GmbH and Contributors to EVerest
#ifndef SLAC_IO_HPP
#define SLAC_IO_HPP

#include <functional>
#include <memory>
#include <string>
#ifdef ESP_PLATFORM
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#else
#include <thread>
#endif

#include <slac/channel.hpp>
#include <port/esp32s3/qca7000_link.hpp>

class SlacIO {
public:
    using InputHandlerFnType = void(slac::messages::HomeplugMessage&);
    explicit SlacIO(const qca7000_config& cfg);

    void release_input();
    /**
     * Start SLAC I/O processing.
     *
     * When @p spawn_background is true a background thread (or FreeRTOS task on
     * ESP platforms) is created that continuously polls the PLC interface and
     * dispatches incoming messages to the provided callback.  If set to false no
     * thread is spawned and the user must call process() periodically.
     */
    void run(std::function<InputHandlerFnType> callback, bool spawn_background = true);

    /// Poll the PLC interface once and invoke the registered callback on
    /// incoming messages. This is intended for use when run() was called with
    /// spawn_background set to false.
    void process(int timeout_ms = 10);
    bool send(slac::messages::HomeplugMessage& msg);
    void quit();

private:
    void loop();
    slac::port::Qca7000Link link;

    slac::Channel slac_channel;
    slac::messages::HomeplugMessage incoming_msg;
    std::function<InputHandlerFnType> input_handler;
#ifdef ESP_PLATFORM
    TaskHandle_t loop_task{nullptr};
#else
    std::thread loop_thread;
#endif

    bool running{false};
};

#endif // SLAC_IO_HPP
