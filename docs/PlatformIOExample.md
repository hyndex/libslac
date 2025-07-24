# PlatformIO Basic SLAC Example

This guide walks through creating a minimal PlatformIO project that uses
`libslac` to communicate with a QCA7000 based power line modem.  It is
structured as a step-by-step tutorial explaining why each step is
required and includes a small unit test to verify the setup.

## 1. Install PlatformIO

Install the PlatformIO command line tools via `pip`:

```bash
pip install platformio
```

PlatformIO handles building the firmware and running the example tests.

## 2. Create a new Project

Create a fresh directory and initialise a PlatformIO project targeting an
ESP32‑S3 development board:

```bash
mkdir my_slac_project
cd my_slac_project
pio project init --board esp32-s3-devkitc-1
```

The `--board` argument selects the hardware configuration.  All files
below are located inside this project folder.

## 3. Add libslac

Clone or copy the `libslac` repository next to your project.  The
example in this repository assumes the following layout:

```
my_slac_project/
  platformio.ini
  src/
  test/
libslac/
  include/
  src/
```

The project references the library sources directly via `src_filter` so
no additional `library.json` is required.

## 4. Configure `platformio.ini`

The `examples/platformio_basic` folder contains a complete
configuration.  The important parts are the include paths and the list
of source files to build.  A minimal configuration looks like:

```ini
[platformio]
src_dir = src

[env:esp32s3]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
build_unflags = -std=gnu++11
build_flags = -std=gnu++17 -I../libslac/include -I../libslac/3rd_party \
    -I../libslac/port/esp32s3 -DESP_PLATFORM -Os -fdata-sections \
    -ffunction-sections -fno-exceptions -fno-rtti
lib_ldf_mode = chain
src_filter = +<../libslac/src/channel.cpp> +<../libslac/src/slac.cpp> \
    +<../libslac/port/esp32s3/qca7000.cpp> \
    +<../libslac/port/esp32s3/qca7000_link.cpp> \
    +<../libslac/3rd_party/hash_library/sha256.cpp> +<src/main.cpp>
```

The include paths expose the headers of `libslac`, while `src_filter`
adds the library sources to the build.  This approach keeps the example
self contained without installing the library globally.

A second environment named `native` can be added with `platform =
native` to build and run the unit tests on the host PC.

## 5. Example `main.cpp`

The example firmware simply initialises the QCA7000 link and polls the
modem:

```cpp
#include <Arduino.h>
#include <slac/channel.hpp>
#include <port/esp32s3/qca7000_link.hpp>

static const uint8_t MY_MAC[ETH_ALEN] = {0x02,0x00,0x00,0x00,0x00,0x01};
static slac::Channel* g_channel = nullptr;

void setup() {
    Serial.begin(115200);
    SPI.begin(48 /*SCK*/, 21 /*MISO*/, 47 /*MOSI*/, PLC_SPI_CS_PIN);
    qca7000_config cfg{&SPI, PLC_SPI_CS_PIN, PLC_SPI_RST_PIN, MY_MAC};
    static slac::port::Qca7000Link link(cfg);
    static slac::Channel channel(&link);
    g_channel = &channel;
    channel.open();
}

void loop() {
    qca7000Process();
    slac::messages::HomeplugMessage msg;
    if (g_channel && g_channel->poll(msg)) {
        // handle incoming packets
    }
    delay(1);
}
```

This program repeatedly calls `qca7000Process()` to service the modem
and uses `Channel::poll()` to check for incoming SLAC frames.  It can be
extended to implement the full ISO15118‑3 handshake.

## 6. Build the Firmware

Compile the project for the ESP32‑S3 environment:

```bash
pio run -e esp32s3
```

If everything is set up correctly PlatformIO will produce an Arduino
firmware binary.  Any compiler errors usually indicate missing include
paths or source files in `platformio.ini`.

## 7. Run the Tests

The example ships with a small unit test that instantiates a
`slac::Channel` using a dummy transport link.  Running the test confirms
that the project configuration can build code using `libslac`:

```bash
pio test -e native
```

The `native` environment builds the project for the host PC and executes
the test binary.  A successful run prints `\*\*\* [native] Success`.

## 8. Next Steps

This basic setup provides the foundation for SLAC communication.  From
here you can integrate the state machine in `tools/evse` or add your own
application logic to perform matching and handle network traffic.  When
modifying the configuration remember to keep the include paths and
`src_filter` entries in sync with your project layout.


