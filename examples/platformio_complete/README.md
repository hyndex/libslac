# PlatformIO Complete Example

This directory contains a fully self-contained PlatformIO project
showing how to use **libslac** on an ESP32-S3 board.  The example
initialises a QCA7000 modem, polls it for incoming SLAC frames and can
be built directly with PlatformIO.  QCA7005-based PLC Stamp micro
modules operate the same way and can be used without code changes.

## Building

Install PlatformIO and compile the firmware with:

```bash
pio run -e esp32s3
```

The project pulls `libslac` automatically via `lib_deps`.
Custom chip select and reset pins for the modem can be configured by
editing `build_flags` in `platformio.ini`.

### Interrupt edge configuration

The QCA7000 signals new data by pulsing its `INT` line.  Depending on the
modem or carrier board the pulse can be active-high or active-low.  The
GPIO interrupt edge is selected with the `PLC_INT_EDGE` build flag and
defaults to `GPIO_INTR_NEGEDGE`.  Set it to `GPIO_INTR_POSEDGE` if your
hardware drives the interrupt line low-to-high when data is ready:

```
build_flags =
    ...
    -DPLC_INT_EDGE=GPIO_INTR_POSEDGE
```

### Required build flags

`cp_monitor.cpp` uses the ESP-IDF continuous ADC API.  The Arduino core
for ESP32-S3 does not currently ship this driver, so the example links a
stub implementation from `lib/slac_port/esp_adc`.  Ensure the following
lines are present in `platformio.ini` to compile and include the stub:

```
build_flags =
    ...
    -Ilib/slac_port
    -Ilib/slac_port/esp_adc    ; ADC continuous driver headers

build_src_filter =
    +<src/*>
```

These settings add `adc_continuous_stub.c` to the build and make the
corresponding headers available.

Defining `RANDOM_MAC` in `build_flags` or sending `R` over the serial
console shortly after reset causes the example to generate a random
locally administered MAC address.  The address is applied with
`qca7000SetMac()` right before starting the SLAC handshake.

