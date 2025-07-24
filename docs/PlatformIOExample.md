# PlatformIO SLAC Example

This guide walks through creating a minimal PlatformIO project using **libslac** on a generic microcontroller board. It demonstrates how to configure the build, initialise the QCA7000 modem and verify basic communication.

## 1. Create a new project

Install PlatformIO and create a project for your target board:

```bash
pip install platformio
pio project init --board <your_board>
```

Replace `<your_board>` with the PlatformIO identifier of your microcontroller.

## 2. Add libslac

Clone `libslac` into the project directory and reference the library from `platformio.ini`:

```ini
[env:microcontroller]
platform = espressif32@6.5.0
board = <your_board>
framework = arduino
lib_extra_dirs = libslac
build_flags = -Ilibslac/include -Ilibslac/3rd_party -Ilibslac/port/microcontroller
```

The include paths expose the library headers and the microcontroller port. Adjust the platform and framework for your board.

## 3. Example `src/main.cpp`

```cpp
#include <slac/channel.hpp>
#include <port/microcontroller/qca7000_link.hpp>

void setup() {
    static const uint8_t my_mac[ETH_ALEN] = {0x02,0x00,0x00,0x00,0x00,0x01};
    qca7000_config cfg{&SPI, PLC_SPI_CS_PIN, PLC_SPI_RST_PIN, my_mac};
    static slac::port::Qca7000Link link(cfg);
    static slac::Channel channel(&link);
    channel.open();
}

void loop() {
    qca7000Process();
    // application code
}
```

`channel.open()` initialises the modem and must succeed before sending or receiving frames. The polling loop calls `qca7000Process()` regularly.

## 4. Build and upload

Run `pio run` to build the project and `pio upload` to flash it on the board. Successful compilation confirms the project is configured correctly.

## 5. Optional tests

If the board supports unit tests, enable the `test` directory and run `pio test` to compile and execute the libslac tests on the target.

