libslac
=======

Simple ISO15118-3 SLAC library.

This repository contains a minimal implementation of the ISO15118-3 Signal Level Attenuation Characterization (SLAC) protocol. It can be used in desktop applications and embedded systems alike. The library is designed with a small dependency footprint and focuses on providing the core data structures and helper functions required to implement the SLAC handshake.

.. contents:: Table of Contents
   :depth: 2
   :local:

Getting the Source
------------------

Clone the repository and its submodules:

.. code-block:: bash

   git clone https://github.com/EVerest/libslac.git
   cd libslac
   git submodule update --init

Building with CMake
-------------------

The project uses ``CMake`` (>= 3.11) and standard commands for dependency management.

A typical build looks as follows:

.. code-block:: bash

   mkdir build
   cd build
   cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release
   ninja

This will build the ``slac`` static library. The library is exported as the CMake target ``slac::slac``. It can be installed with ``ninja install`` when ``SLAC_INSTALL`` is enabled.

Building with PlatformIO
-----------------------

For embedded targets the library can be built using `PlatformIO <https://platformio.org/>`_. Two environments are provided:

``env:host``
    Builds the library and a small example for the native host using the ``platform = native`` platform.
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

Using the Library
-----------------

1. Implement ``slac::transport::Link`` for your environment.
2. Create a :class:`slac::Channel` instance with the link implementation.
3. Use :class:`slac::messages::HomeplugMessage` to construct and parse SLAC messages.

A minimal example can be found in ``pio_src/main.cpp``:

.. code-block:: cpp

   slac::transport::Link* link = nullptr; // provide your implementation
   slac::Channel channel(link);
   channel.open();
   // send/receive messages using channel.read() and channel.write()

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

The ``tools`` directory contains small utilities demonstrating how to use ``libslac``. ``tools/bridge.cpp`` shows how to forward packets between two virtual interfaces. The ``tools/evse`` directory contains a simple state machine for the EVSE side of the SLAC handshake.

Running the Tests
-----------------

Unit tests are based on GoogleTest. Enable ``BUILD_TESTING`` when configuring CMake:

.. code-block:: bash

   cmake .. -G Ninja -DBUILD_TESTING=ON
   ninja
   ctest

License
-------

This project is licensed under the Apache-2.0 License. See ``LICENSE`` for full license information.

