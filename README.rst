libslac
=======

Simple ISO15118-3 SLAC library for microcontrollers.

``libslac`` implements the Signal Level Attenuation Characterization (SLAC)
protocol with a focus on the ESP32 family running the Arduino framework.  The
library aims to be small and self-contained, providing just the data structures
and helpers required to perform the SLAC handshake.

Earlier revisions shipped Linux utilities and a PF\_PACKET based transport
layer.  These components have been removed in favour of a lean
microcontroller-only design.

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


Install PlatformIO via pip:

.. code-block:: bash

   pip install platformio

The unit tests rely on the system provided GoogleTest libraries.  When
missing, ``run_tests.sh`` attempts to install ``libgtest-dev`` using
``apt``.  If ``apt`` is not available install the package manually
before running ``run_tests.sh``:

.. code-block:: bash

   sudo apt-get install libgtest-dev

Building with PlatformIO
-----------------------

PlatformIO is the recommended way to build firmware for ESP32 boards.
Add ``libslac`` to ``lib_deps`` in your ``platformio.ini`` and PlatformIO will
fetch the library automatically:

.. code-block:: ini

   lib_deps = https://github.com/hyndex/libslac.git

The repository ships a ``library.json`` so no additional configuration is
required.  Building the example firmware for ESP32‑S3 boards can be used
to verify a working setup.  The configuration is provided under the
``env:esp32s3`` environment and can be invoked with:

.. code-block:: bash

   pio run -e esp32s3

Control Pilot Monitoring
-----------------------

The example firmware samples the Control Pilot voltage using the ESP32-S3 ADC in continuous mode with DMA. Samples are collected at 50 kS/s into a ring buffer and a background task selects the peak value for each PWM cycle to update the library's Control Pilot state. This approach removes the timing jitter of one-shot conversions and provides accurate peak detection with minimal CPU load.

After changing any of these options run ``platformio run`` to verify that the project still builds correctly.


Platform Limitations
--------------------

``libslac`` currently provides only an ESP32‑S3 port targeting the QCA7000
power line modem.  Other boards must supply their own
``transport::Link`` implementation and platform timing helpers.  No host
implementation or Linux packet socket support is included.

Migration from Linux Tools
-------------------------

Previous releases offered Linux utilities and a packet‑socket transport
layer.  These have been removed.  Users needing the old functionality
should use an older commit or tag.

Library Concepts
----------------

``libslac`` exposes only a few classes in ``include/slac``:

:class:`slac::transport::Link`
    Abstract interface to send and receive raw Ethernet frames. Applications must provide an implementation that matches their environment.
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
   qca7000_config cfg{&SPI, PLC_SPI_CS_PIN, PLC_SPI_RST_PIN, my_mac};
   slac::port::Qca7000Link link(cfg);
   slac::Channel channel(&link);
   if (!channel.open()) {
       // initialization failed, query link.init_failed() for details
       return;
   }

When :func:`channel.open()` fails, the link enters an error state and further
calls will not attempt to reinitialise the modem.  Call
``link.init_failed()`` to query this condition and react accordingly.

Unexpected modem resets are reported through an optional callback or
error flag. Use ``link.set_error_callback(cb, arg)`` to register a
callback and periodically check ``link.fatal_error()`` when polling the
driver. Call ``qca7000CheckAlive()`` roughly once per minute to
confirm that the modem is still responsive.

``Channel::read()`` and ``Channel::write()`` return a
``slac::transport::LinkError`` describing the result.  Besides ``Ok`` and
``Timeout`` the enumeration includes ``Transport`` for modem-level failures,
``InvalidArgument`` when an invalid frame is supplied, ``InvalidLength`` for
malformed packets and ``NoLink`` if the underlying ``Link`` is missing.  A
``Transport`` or ``NoLink`` error usually indicates that the modem needs a
reset while ``Timeout`` suggests retrying the operation.

QCA7000 Configuration
---------------------

The SPI pins used to communicate with the QCA7000 modem are defined in
``port/esp32s3/qca7000.hpp`` as ``PLC_SPI_CS_PIN`` and ``PLC_SPI_RST_PIN``.
Override these macros when building to match your hardware wiring or
specify the pins through ``qca7000_config`` when opening the link.
``PLC_SPI_SLOW_HZ`` controls the bus speed during modem reset and can
be overridden in ``platformio.ini`` as well.
Chip select is toggled manually by the driver, therefore ``SPI.begin`` is
called with ``-1`` as the CS pin and the configured pin is controlled via
``digitalWrite``.

The ``qca7000_config`` struct allows selecting the SPI bus, chip select
and reset pins as well as the modem's MAC address when creating
``slac::port::Qca7000Link``:

.. code-block:: cpp

   const uint8_t my_mac[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
   qca7000_config cfg{&SPI, PLC_SPI_CS_PIN, PLC_SPI_RST_PIN, my_mac};
   slac::port::Qca7000Link link(cfg);

Polling Without IRQ
-------------------

The QCA7000 driver can be polled instead of relying on an interrupt
line.  Earlier revisions called ``qca7000Process()`` from the ``loop()``
function and then polled the channel for new packets.  When using this
approach the IRQ pin on the modem may remain unconnected.  Chip select is
controlled manually and the example initialises the SPI bus with ``SPI.begin``
using ``-1`` for the CS
parameter.

.. code-block:: cpp

   void loop() {
       if (plc_irq) {
           plc_irq = false;
           qca7000ProcessSlice();
       }
       slac::messages::HomeplugMessage msg;
       if (channel.poll(msg)) {
           // handle message
       }
       delay(1);
   }
Polling Operation
-----------------

``libslac`` does not require the QCA7000 interrupt pin. Calling
``qca7000ProcessSlice()`` from an IRQ-driven loop keeps the main thread
responsive while processing at most 500 µs of modem activity per call.

UART Mode
---------

If ``SLAC_USE_UART`` is defined, ``libslac`` provides
``slac::port::Qca7000UartLink``. Select the serial port and baud rate
via ``qca7000_uart_config``:

.. code-block:: cpp

   qca7000_uart_config cfg{&Serial2, 1250000};
   slac::port::Qca7000UartLink link(cfg);

Tools and Examples
------------------

Refer to the example projects under ``examples`` for usage. See ``docs/BoardExample.md`` for a complete PlatformIO configuration, ``docs/PlatformIOExample.md`` for a detailed tutorial, ``docs/qca7000-bring-up.md`` for pin wiring and typical logs, and ``docs/qca7000-troubleshooting.md`` for a step-by-step checklist when the modem fails to respond.

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

SLAC State Machine
------------------

``libslac`` vendors the lightweight `libfsm` library for implementing
state machines.  Include ``<slac/fsm.hpp>`` and derive your states from
``slac::fsm::states::SimpleStateBase`` or
``slac::fsm::states::CompoundStateBase``.  The helper classes manage
state transitions and optionally operate without heap allocations.

Running Unit Tests
------------------

The ``run_tests.sh`` script builds and executes the unit tests.  It
links against the system provided GoogleTest libraries.  When the
headers are missing, the script attempts to install the
``libgtest-dev`` package via ``apt``.  If ``apt`` is unavailable,
install the dependency manually before running the script.  For example:

.. code-block:: bash

   sudo apt-get install libgtest-dev

Execute the tests with:

.. code-block:: bash

   ./run_tests.sh


Vendored Dependencies
---------------------

Small helper libraries are shipped with the source under ``3rd_party``:

- ``hash_library`` provides SHA-256 routines.
- ``fsm`` offers a minimal finite state machine implementation.

See ``THIRD_PARTY.rst`` for license information.

License
-------

This project is licensed under the Apache-2.0 License. See ``LICENSE`` for full license information.

