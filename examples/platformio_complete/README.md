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

### Required build flags

`cp_monitor.cpp` uses the ESP-IDF continuous ADC API.  The Arduino core
for ESP32-S3 does not currently ship this driver, so the example links a
stub implementation from `../../port/esp_adc`.  Ensure the following
lines are present in `platformio.ini` to compile and include the stub:

```
build_flags =
    ...
    -I../../port/esp_adc       ; ADC continuous driver headers

build_src_filter =
    +<src/*>
    +<../../port/esp_adc/adc_continuous_stub.c>
```

These settings add `adc_continuous_stub.c` to the build and make the
corresponding headers available.

Defining `RANDOM_MAC` in `build_flags` or sending `R` over the serial
console shortly after reset causes the example to generate a random
locally administered MAC address.  The address is applied with
`qca7000SetMac()` right before starting the SLAC handshake.

