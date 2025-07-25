# ESP32-S3 DevKit Board Example

This example demonstrates how to wire the QCA7000 modem to an
ESP32-S3 board using PlatformIO. The board uses custom SPI pins and
it is recommended to connect the interrupt line (`PLC_INT_PIN`, IO16 by
default) so the driver can react to modem events.

## PlatformIO configuration

```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
monitor_filters = esp32_exception_decoder
board_build.partitions = default_8MB.csv
board_upload.flash_size = 8MB
board_build.arduino.memory_type = dio_qspi
build_flags = \
    -DPLC_SPI_CS_PIN=41 \
    -DPLC_SPI_RST_PIN=40 \
    -DPLC_SPI_SLOW_HZ=500000
```

The SPI bus is initialised by the driver using the pin macros from
`qca7000.hpp`. Chip select is driven manually, so `SPI.begin()` is
called with `-1` for the CS pin.

```cpp
volatile bool plc_irq = false;
void IRAM_ATTR plc_isr() { plc_irq = true; }

qca7000_config cfg{&SPI, 41, 40, my_mac};
slac::port::Qca7000Link link(cfg);
pinMode(PLC_INT_PIN, INPUT);
attachInterrupt(PLC_INT_PIN, plc_isr, FALLING);
while (true) {
    if (plc_irq) {
        plc_irq = false;
        qca7000ProcessSlice();
    }
    // application tasks
}
```

The loop above processes incoming modem events whenever the interrupt
fires, ensuring timely handling while leaving the main thread free for
other tasks.
