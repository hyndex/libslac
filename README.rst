libslac
=======

Simple ISO15118-3 SLAC library.

This repository contains a minimal implementation of the ISO15118-3 Signal Level Attenuation Characterization (SLAC) protocol. It can be used in desktop applications and embedded systems alike. The library is designed with a small dependency footprint and focuses on providing the core data structures and helper functions required to implement the SLAC handshake.

.. contents:: Table of Contents
   :depth: 2
   :local:

Getting the Source
------------------

Clone the repository:

.. code-block:: bash

   git clone https://github.com/hyndex/libslac.git
   cd libslac

All required third-party code is bundled in ``3rd_party`` so no
additional ``git submodule`` commands are needed.

Prerequisites
-------------

Install the tools used for embedded builds and unit testing before
configuring the project:

.. code-block:: bash

   pip install platformio
   apt-get install libgtest-dev

Building with CMake
-------------------

The project uses ``CMake`` (>= 3.11) and standard commands for dependency management.

A typical build looks as follows:

.. code-block:: bash

   mkdir build
   cd build
   cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release
   cmake --build .

This will build the ``slac`` static library. The library is exported as the CMake target ``slac::slac``. Set ``SLAC_INSTALL`` to ``ON`` and run ``cmake --install .`` to install it.

Building with PlatformIO
-----------------------

For embedded targets the library can be built using `PlatformIO <https://platformio.org/>`_. The following environment is provided:

``env:esp32s3``
    Example configuration for ESP32-S3 development boards. It shows the required compiler flags and source filters.

Invoke PlatformIO using:

.. code-block:: bash

   pio run -e esp32s3

This command builds the example firmware for ESP32-S3 boards and is
useful to verify compilation after refactoring.

Library Concepts
----------------

``libslac`` exposes only a few classes in ``include/slac``:

:class:`slac::transport::Link`
    Abstract interface to send and receive raw ethernet frames. Applications must provide an implementation that matches their environment (e.g. raw sockets on Linux or a driver on microcontrollers).
:class:`slac::Channel`
    Helper around a :class:`transport::Link` adding timeout handling and convenience helpers for reading and writing SLAC messages.
:class:`slac::messages::HomeplugMessage`
    Representation of a HomePlug AV frame used to carry SLAC payloads.

The header ``slac/slac.hpp`` also defines all SLAC message structures and constants.
Timing constants used during ISO15118-3 matching are provided in ``slac/iso15118_consts.hpp``.

Using the Library
-----------------

1. Implement ``slac::transport::Link`` for your environment.
2. Create a :class:`slac::Channel` instance with the link implementation.
3. Use :class:`slac::messages::HomeplugMessage` to construct and parse SLAC messages.

An example for the ESP32-S3 port:

.. code-block:: cpp

   #include <port/esp32s3/qca7000_link.hpp>

   const uint8_t my_mac[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
   qca7000_config cfg{&SPI, PLC_SPI_CS_PIN, my_mac};
   slac::port::Qca7000Link link(cfg);
   slac::Channel channel(&link);
   if (!channel.open()) {
       // initialization failed, query link.init_failed() for details
       return;
   }

When :func:`channel.open()` fails, the link enters an error state and further
calls will not attempt to reinitialise the modem.  Call
``link.init_failed()`` to query this condition and react accordingly.

QCA7000 Configuration
---------------------

The SPI pins used to communicate with the QCA7000 modem are defined in
``port/esp32s3/qca7000.hpp`` as ``PLC_SPI_CS_PIN`` and ``PLC_SPI_RST_PIN``.
Override these macros when building to match your hardware wiring.

The modem's MAC address can be specified via ``qca7000_config`` when
creating :class:`slac::port::Qca7000Link`:

.. code-block:: cpp

   const uint8_t my_mac[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
   qca7000_config cfg{&SPI, PLC_SPI_CS_PIN, my_mac};
   slac::port::Qca7000Link link(cfg);

Tools and Examples
------------------

The ``tools`` directory contains small utilities demonstrating how to use ``libslac``. ``tools/evse`` contains a simple state machine for the EVSE side of the SLAC handshake. ``tools/bridge.cpp`` can forward packets between two virtual interfaces on Linux and is disabled on microcontroller builds.

Porting to Other Boards
-----------------------

``libslac`` only ships an ESP32-S3 port. When targeting another MCU you need to
provide two pieces:

1. A :class:`transport::Link` implementation for sending and receiving ethernet
   frames.
2. A ``port_config.hpp`` defining ``slac_millis`` and ``slac_delay`` as well as
   optional interrupt helpers.

``transport::Link`` exposes ``open()``, ``write()``, ``read()`` and ``mac()``.
``open()`` should initialise the hardware and return ``true`` on success. The
``write()`` and ``read()`` methods transfer raw frames with millisecond timeouts
while ``mac()`` returns the local MAC address.

``port_config.hpp`` is included by the library and provides platform specific
timing helpers. A minimal bare-metal variant might look like:

.. code-block:: cpp

   #pragma once
   #include <stdint.h>
   extern "C" uint32_t board_millis();
   static inline uint32_t slac_millis() { return board_millis(); }
   static inline void slac_delay(uint32_t ms) { /* busy wait */ }

For PlatformIO builds place your implementation under ``port/<board>`` and add
the files to ``src_filter``. A sample STM32 configuration is shown below:

.. code-block:: ini

   [env:stm32]
   platform = ststm32
   board = nucleo-f429zi
   framework = arduino
   build_unflags = -std=gnu++11
   build_flags = -std=gnu++17 -Iinclude -I3rd_party -Iport/stm32 -Os \
       -fdata-sections -ffunction-sections -fno-exceptions -fno-rtti
   src_filter = +<src/channel.cpp> +<src/slac.cpp> \
       +<port/stm32/my_link.cpp> +<3rd_party/hash_library/sha256.cpp> \
       +<path/to/main.cpp>

Running the Tests
-----------------

Unit tests are based on GoogleTest. Enable ``BUILD_TESTING`` when configuring CMake and run ``ctest`` after building:

.. code-block:: bash

   cmake .. -G Ninja -DBUILD_TESTING=ON
   ninja
   ctest

Vendored Dependencies
---------------------

Small helper libraries are shipped with the source under ``3rd_party``:

- ``hash_library`` provides SHA-256 routines.
- ``libfsm`` contains lightweight state machine helpers.

See ``THIRD_PARTY.rst`` for license information.

License
-------

This project is licensed under the Apache-2.0 License. See ``LICENSE`` for full license information.

