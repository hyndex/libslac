# PlatformIO Complete Example

This directory contains a fully self-contained PlatformIO project
showing how to use **libslac** on an ESP32-S3 board.  The example
initialises a QCA7000 modem, polls it for incoming SLAC frames and can
be built directly with PlatformIO.

## Building

Install PlatformIO and compile the firmware with:

```bash
pio run -e esp32s3
```

The project pulls `libslac` automatically via `lib_deps`.
Custom chip select and reset pins for the modem can be configured by
editing `build_flags` in `platformio.ini`.  Make sure the SPI bus is
initialised with the correct pin numbers via `SPI.begin()` as shown in
`src/main.cpp`.

