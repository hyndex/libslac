# ESP32-S3 DevKit Board Example

This example demonstrates how to wire the QCA7000 modem to an
ESP32-S3 board using PlatformIO. The board uses custom SPI pins and
it is recommended to connect the interrupt line (`PLC_INT_PIN`, IO16 in
this example) so the driver can react to modem events. QCA7005-based
PLC Stamp micro modules are fully compatible and can be wired in the
same way.

## Pin mapping

The ESP32‑S3 port provides default pin macros (`PLC_SPI_CS_PIN`
defaults to 36, `PLC_SPI_RST_PIN` to 40, `PLC_SPI_SCK_PIN` to 48,
`PLC_SPI_MISO_PIN` to 21 and `PLC_SPI_MOSI_PIN` to 47). This DevKit
uses a different chip‑select pin and enables optional interrupt and
power‑enable signals. The table below lists the pins used in this
example; all signals use 3.3 V logic levels.

| Signal        | Macro                | Example Pin | Notes |
|---------------|----------------------|-------------|-------|
| SPI SCK       | `PLC_SPI_SCK_PIN`    | 48          | push‑pull output, no pull‑ups required |
| SPI MISO      | `PLC_SPI_MISO_PIN`   | 21          | driven by modem, tri‑stated when idle; no pull‑up needed |
| SPI MOSI      | `PLC_SPI_MOSI_PIN`   | 47          | push‑pull output, no pull‑ups required |
| Chip Select   | `PLC_SPI_CS_PIN`     | 41          | keep high when idle; optional pull‑up |
| Reset         | `PLC_SPI_RST_PIN`    | 40          | active low; tie high with resistor if GPIO is high‑Z at boot |
| Interrupt     | `PLC_INT_PIN`        | 16          | active‑low open‑drain; requires pull‑up (e.g. 10 kΩ); no default |
| Power Enable  | `PLC_PWR_EN_PIN`     | 15          | active high; pull‑down to keep modem off at reset; optional |

If your hardware provides external resistors the internal ones of the
ESP32 can be disabled. Otherwise configure them appropriately with
`pinMode`.

Override the library defaults by defining the macros in your source before
including `qca7000.hpp` or by adding `-D` flags in `platformio.ini` as
shown below.

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
    -DPLC_SPI_SCK_PIN=48 \
    -DPLC_SPI_MISO_PIN=21 \
    -DPLC_SPI_MOSI_PIN=47 \
    -DPLC_SPI_CS_PIN=41 \
    -DPLC_SPI_RST_PIN=40 \
    -DPLC_INT_PIN=16 \
    -DPLC_PWR_EN_PIN=15 \
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
