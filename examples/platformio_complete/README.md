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
editing `build_flags` in `platformio.ini`.

Defining `RANDOM_MAC` in `build_flags` or sending `R` over the serial
console shortly after reset causes the example to generate a random
locally administered MAC address.  The address is applied with
`qca7000SetMac()` right before starting the SLAC handshake.

