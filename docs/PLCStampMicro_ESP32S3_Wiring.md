# Wiring the PLC Stamp micro 2 to an ESP32‑S3

This guide explains how to connect a PLC Stamp micro 2 module to an ESP32‑S3 host.
It focuses on the SPI host interface, GPIO strapping and any required pull‑up or
pull‑down resistors. All signals operate at **3.3 V logic levels**. Do not expose
the module to 5 V.

## 1. Power and Ground

1. Provide a stable 3.3 V supply capable of at least 300 mA to pin&nbsp;1 (*VDD*).
2. Connect **all** ground pins (7–9 and 11–17 and 28) to the ESP32's ground to
   ensure a low‑impedance reference.
3. Place the supply decoupling capacitors close to the module as recommended in the
   datasheet.
4. If the host controls the module's power via an external switch or regulator,
   keep the module powered down until the ESP32 is ready to configure it.

## 2. SPI Interface Wiring

The PLC Stamp micro 2 uses the pins labelled *SERIAL_0* – *SERIAL_4* for its
serial interface. In SPI mode the mapping is:

| Module Pin | SPI Function | Description | ESP32‑S3 Example GPIO | Notes |
|-----------:|--------------|-------------|------------------------|-------|
| SERIAL_1   | `SCK`        | Serial clock output from ESP32 to modem | GPIO48 | Push‑pull, no pull‑up needed |
| SERIAL_3   | `MISO`       | Data from modem to ESP32 | GPIO21 | Tri‑stated when idle; no pull‑up required |
| SERIAL_4   | `MOSI`       | Data from ESP32 to modem | GPIO47 | Push‑pull, no pull‑up required |
| SERIAL_2   | `CS`         | Chip‑select, active low | GPIO41 | Keep high when idle. Optional pull‑up to prevent spurious accesses during boot |
| SERIAL_0   | `INT`        | Interrupt request, active‑low open‑drain | GPIO16 | **Requires pull‑up** to 3.3 V (e.g. 10 kΩ) because the modem only drives it low |

* All SPI lines must share the ESP32's `VSPI` or `FSPI` hardware peripheral. Use
  `pinMode` to configure them as inputs/outputs as shown.
* The interrupt line is optional for simple polling implementations but highly
  recommended to avoid losing packets. When unused, leave `SERIAL_0` floating and
  disable interrupts in the driver.

## 3. Reset Control

* Pin&nbsp;22 (*RESET_L*) is an **active‑low** reset input. Tie it to an ESP32 GPIO so
  the host can force a hardware reset. Keep the line high when the modem should
  run.
* Because many ESP32 GPIOs default to high‑impedance at boot, add a pull‑up (10–
  100 kΩ) on *RESET_L* to prevent the modem from being held in reset before
  firmware initializes the pin.

## 4. Optional Power Enable

Some carrier boards expose a power‑enable pin that gates the QCA7000's 3.3 V
supply. If such a pin is available, connect it to an ESP32 GPIO named
`PLC_PWR_EN_PIN` and:

1. Hold it **low** during ESP32 boot to keep the modem off.
2. Drive it **high** once the ESP32 firmware is ready to initialise the modem.
3. Add a pull‑down resistor so the modem remains off when the GPIO is high‑Z.

If your hardware lacks a dedicated power‑enable pin simply feed VDD continuously
and omit this control line.

## 5. GPIO Strap Pins

The QCA7000 uses GPIO0–GPIO2 as bootstrap inputs during reset. The PLC Stamp
micro 2 preload uses a pull‑down on GPIO1 selecting **SPI burst mode** as default.
External connections must not disturb these strap levels:

| GPIO | Boot Function | Strap on Module | Recommendations |
|-----:|---------------|-----------------|-----------------|
| GPIO0 | Boot source (flash vs host) | none | Leave floating unless changing boot mode |
| GPIO1 | Host interface select (SPI vs UART) | 10 kΩ pull‑down | Leave floating to keep SPI; do not add pull‑up |
| GPIO2 | SPI mode (burst vs legacy) | none | Leave floating for burst mode. Add pull‑down only if legacy mode is required |
| GPIO3 | No boot role | none | Free for status LED or push button |

After boot the firmware drives these pins for status indications (link
status, Simple Connect, etc.) and they are **not** general‑purpose I/O for the
host. If you attach LEDs or buttons, follow the datasheet examples so the
strapping resistors are not overridden.

## 6. Additional Recommendations

* Connect the ESP32 and PLC module grounds directly and keep SPI traces short to
  minimise noise.
* Route the differential TX_P/TX_N and RX_P/RX_N lines to the coupling network as
  laid out by the hardware reference design. They should remain isolated from the
  low‑voltage logic domain except at the coupling transformer.
* If the `ZC_IN` pin (pin 6) is unused, leave it unconnected. It is intended for
  zero‑cross detection in EV applications.
* Ensure the ESP32 pins used for SPI do not conflict with other boot strapping
  functions of the microcontroller.

Following these guidelines yields a reliable connection between the ESP32‑S3 and
the PLC Stamp micro 2 for HomePlug Green PHY communication.

