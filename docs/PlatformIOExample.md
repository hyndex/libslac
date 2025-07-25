# PlatformIO Basic SLAC Example

This guide walks through creating a minimal PlatformIO project that uses
`libslac` with an ESP32 board running the Arduino framework.  It
communicates with a QCA7000 based power line modem and explains the
required steps in a short tutorial.

## 1. Install PlatformIO

Install the PlatformIO command line tools via `pip`:

```bash
pip install platformio
```

PlatformIO handles building the firmware for the selected target.

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

Include the library via `lib_deps` so PlatformIO automatically fetches
it from GitHub:

```ini
lib_deps = https://github.com/hyndex/libslac.git
```

No extra `library.json` or source filtering is necessary as the
repository already provides a `library.json` configuration.

## 4. Configure `platformio.ini`

The `examples/platformio_complete` folder contains a complete
configuration.  When using ``lib_deps`` only a few build flags are
required.  A minimal configuration looks like:

```ini
[platformio]
src_dir = src

[env:esp32s3]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
build_unflags = -std=gnu++11
build_flags = -std=gnu++17 -DESP_PLATFORM
lib_deps = https://github.com/hyndex/libslac.git
```

PlatformIO downloads the library and sets up include paths
automatically.  This keeps the example self contained without copying
the sources into your project.

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
    Serial.println("Starting SLAC modem...");
    // Initialise the SPI bus. Chip select is controlled by the driver so
    // SPI.begin is called with -1 for the CS pin.
    SPI.begin(48 /*SCK*/, 21 /*MISO*/, 47 /*MOSI*/, -1);
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

## 7. Next Steps

This basic setup provides the foundation for SLAC communication.  From
here you can integrate the state machine in `tools/evse` or add your own
application logic to perform matching and handle network traffic.
When modifying the configuration make sure to keep any custom build
flags in sync with your project layout.


