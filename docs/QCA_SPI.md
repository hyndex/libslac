# QCA7000 Ethernet-over-SPI Interface with ESP32 for EV Communication

## Overview

The **Qualcomm QCA7000 (QCA7K)** is a HomePlug Green PHY (HPGP) powerline communication chipset that provides a bridge between a host microcontroller and a powerline network, effectively presenting as an Ethernet interface accessible over SPI or UART. In Electric Vehicle (EV) communication systems, the QCA7000 plays a crucial role by enabling data exchange over the charging cable using powerline communication (PLC) techniques (HPGP is used in EV charging standards like ISO/IEC 15118). When used in SPI mode, the QCA7000 allows an ESP32 microcontroller (or other host) to send and receive Ethernet frames over an SPI bus, with the QCA7000 handling the PLC transceiver functions. This **Ethernet-over-SPI bridge** configuration is essential for EV charging stations and vehicles, as it carries high-level protocols (e.g. TCP/IP or ISO-15118 messages) between the EV and charging station using the existing power lines.

In summary, QCA7000 abstracts the PLC link as an Ethernet-like network interface for the ESP32. The ESP32 communicates with QCA7K as if it were an external Ethernet MAC/PHY, but all data is transferred via SPI. This document provides a comprehensive guide to interfacing QCA7000 with ESP32 in both bare-metal and RTOS (FreeRTOS/ESP-IDF) environments, covering hardware connections, the SPI protocol, framing format, register operations, software integration, and best practices for robust communication.

## Hardware Interface

**SPI Connection and Signals:** The QCA7000 acts as an SPI slave device to the ESP32. A standard 4-wire SPI connection is used: ESP32’s SPI Master-Out (MOSI) connects to QCA7K’s MOSI input, Master-In (MISO) connects to QCA7K’s MISO output, SPI Clock (SCLK) from ESP32 connects to QCA7K’s SPI CLK, and a dedicated Chip Select (CS) (also called SPI slave select) from ESP32 selects the QCA7000. The QCA7000 requires **SPI Mode 3** (CPOL=1, CPHA=1) where clock is idle high and data is sampled on the rising edge. It is crucial to configure the ESP32’s SPI peripheral accordingly, or communication will fail (e.g., reading the signature register would not return the expected value if polarity or phase are wrong). The SPI clock frequency should be within QCA7000’s supported range: typically up to 8 MHz by default, with an absolute maximum of \~12–16 MHz under ideal conditions. For initial bring-up, 8 MHz is a safe choice (the Linux driver defaults to 8 MHz if not specified). The SPI lines should be kept short and properly routed, with optional series resistors (\~10–20 Ω) on clock and data lines to dampen ringing if signal integrity issues are observed.
Note: Some early examples mention SPI Mode 1, but the QCA7000 actually operates in Mode 3. Using Mode 1 leads to corrupted reads (e.g. wrong signature).

**Voltage Levels and Power:** The QCA7000 operates on a single **3.3 V supply** and its digital I/O lines (SPI and GPIOs) are 3.3 V logic, which is convenient for direct connection to the ESP32’s 3.3 V GPIOs. Ensure a common ground between the ESP32 and QCA7000. Both devices should be appropriately decoupled (place decoupling capacitors near QCA7K’s Vcc pins) because high-frequency SPI and PLC operations can introduce noise.

**GPIO Interrupt (IRQ):** Earlier revisions of the firmware supported an optional
interrupt line from the modem. The current implementation uses simple polling
instead, so no IRQ connection is required.

**Reset and Bootstrapping:** The QCA7000 typically has a **reset input pin** that can be driven by the ESP32 (or an external reset circuit). It is good practice to wire this pin to an ESP32 GPIO to allow the software to reset the QCA7000 if needed (e.g., on startup or to recover from errors). When the QCA7000 is reset or powered on, it runs its internal bootloader/firmware. The chip can either boot from an external SPI Flash or wait for the host to load firmware via a specified mechanism. In most EV/PLC modules, an external QCA7000 comes pre-flashed with firmware, so typically the host does not need to upload code; we assume QCA7K is ready after reset. **Boot mode** (SPI vs UART) is usually determined by a configuration strap pin or firmware setting. Ensure that the hardware/firmware is set for **SPI operation**. On some boards, this might mean strapping a mode pin high or low, or using a specific QCA7000 part number variant dedicated to SPI. The Linux device tree notes that the QCA7000 host interface mode “depends on how the QCA7000 is setup via GPIO pin strapping”, implying hardware configuration at reset. Verify the QCA7000 module’s documentation for the correct strap (for example, one of the *BOOT\_MODE* or *CONFIG* pins typically selects SPI vs UART).

**Clock Speed Constraints:** As noted, QCA7000 supports SPI clock up to about 12–16 MHz. The data sheet and drivers indicate valid SPI speeds between 1 MHz and 16 MHz. Empirically, many integrations use 8 MHz for stable operation. If a higher throughput is needed, one can experiment with 12 MHz; 16 MHz is the upper limit and might not be reliable on longer wires or certain boards. The period must not be less than 83.3 ns (≈12 MHz) according to an application note. **Clock polarity/phase must be correct (Mode 3)** – mismatching this will result in reading incorrect values (e.g., the signature register won’t read as 0xAA55 if clock mode is wrong, which is a primary symptom of misconfiguration). Also note that the ESP32’s SPI peripheral supports Mode 3, so this configuration is straightforward with ESP-IDF (`spi_device_interface_config_t.mode = 3`) or via register (set CK\_OUT\_EDGE and CK\_IDLE\_LEVEL accordingly for mode 3).

**Chip Select Modes (Burst vs Legacy):** The QCA7000 supports two SPI transfer modes regarding the chip select behavior: **burst mode** and **legacy mode**. In **burst mode**, the ESP32 should keep the CS line asserted low for the entire transfer of a command or data burst (e.g., an entire frame), without toggling CS between bytes/words. This is the recommended mode for efficiency – it allows streaming many bytes in one continuous SPI transaction. In **legacy mode**, the SPI master toggles CS for each 16-bit word transferred (each word still being sent CPHA=1, CPOL=1). Legacy mode is significantly slower due to the gaps introduced between every word and is generally not used unless required by some host limitation. The QCA7000’s mode is determined either by a strap or by a register setting. If the QCA7K is strapped for burst (multi-byte) mode (often the default in modern designs), the ESP32 should hold CS low throughout each read/write operation. If for some reason the chip is strapped to legacy (multi\_cs) mode (where it expects CS to toggle per word), the driver must be aware: the Linux driver provides a `qca,legacy-mode` flag to handle this. In practice, **ensure QCA7000 is in burst mode** (multi-CS *disabled*) for EV use; this can often be verified by reading the QCA7000’s SPI configuration register bit for multi-CS (discussed later) or simply observing that multi-byte transfers work when CS remains low. Operating in legacy mode would require special handling in the ESP32 SPI driver (multiple small transactions), so it’s best avoided for high-throughput EV communications. For completeness, if legacy mode were needed, the ESP32 could still manage it by issuing separate 16-bit transactions back-to-back and toggling CS via software control, but the throughput penalty is high.
The ESP32 driver automatically reads `SPI_REG_SPI_CONFIG` after each modem reset and clears the `MULTI_CS` bit if it finds the device in legacy mode.

**Other Connections:** If using a QCA7000 module, refer to its pinout for any additional signals (e.g., **SPI flash interface** if external flash is present – these pins connect to the QCA7000’s own flash, not directly to ESP32). The ESP32 generally does not interact with the QCA’s flash or analog front-end; it only needs the SPI and IRQ lines. Also consider the **grounding and shielding** since PLC involves high-frequency signals on the powerline; good ground reference between ESP32 and QCA7K will reduce noise on SPI signals.

## Software Architecture

**Ethernet-over-SPI Protocol Stack:** Using QCA7000 with ESP32 essentially inserts a small link-layer protocol beneath the standard IP stack. From an OSI model perspective, the QCA7000 (with its driver) implements the Data Link Layer as an “Ethernet-over-SPI” link. Above it, the Network (IP) and Transport (TCP/UDP) layers run on the ESP32 (e.g., lwIP if using FreeRTOS/ESP-IDF). The QCA7000’s internal firmware handles the PLC MAC and PHY layers, making the powerline network appear like a simple Ethernet segment to the host. The **software architecture** on the ESP32 side typically comprises:

* A **QCA7000 driver** (handling SPI transfers, framing, and register access). This is the core of the implementation, often running partly in interrupt context (for receiving frames and events) and partly in a transmit function.
* A **Network interface integration** that connects the driver to the TCP/IP stack. In an RTOS environment (ESP-IDF), this might be an `esp_netif` or `ethernetif` driver binding that supplies function pointers for transmit and receive to the stack. In a bare-metal scenario, the driver itself may call into a user-defined callback or processing function for received packets.

**Driver Responsibilities:** The QCA7K driver must implement the SPI transport protocol defined by QCA: this includes formatting outgoing frames with the proper header/footer, parsing incoming frames, managing the QCA7000’s internal buffers via register reads/writes, and handling interrupts (buffer status, errors, etc.). It essentially treats the QCA7000 as a NIC (Network Interface Controller) accessible over SPI. Key tasks include:

* **Initialization & Sync:** Reset or sync with the QCA7000 at startup, verify communication (signature register), configure SPI settings (if needed) and enable relevant interrupts on the QCA7000.
* **Transmit path:** Accept an outgoing Ethernet packet (e.g., from the IP stack), encapsulate it in QCA7K’s framing, check that QCA’s internal buffer has room, then send it via SPI. Possibly queue packets if QCA’s buffer is full.
* **Receive path:** When QCA signals a packet is available, read it via SPI, validate and strip framing, then hand the raw Ethernet frame up to the host network stack.
* **Interrupt handling:** Service QCA7000 interrupt events such as packet arrival, buffer errors, or device reset notifications, and take appropriate action.
* **Error recovery:** Detect and handle error conditions (e.g., buffer overflow, out-of-sync SPI data) by resetting internal state or even resetting the QCA7000 if necessary, to keep the link robust.
* **Mode support:** Ensure compatibility with both bare-metal single-loop operation and multi-threaded RTOS operation (e.g., using locks or atomic sections as needed).

**Bare-Metal Integration:** In a bare-metal environment (no RTOS), the driver can be implemented as a set of ISRs and polling routines in the main loop. A common structure is:

* Configure SPI and the IRQ pin.
* Use the QCA7000 IRQ to trigger an ISR whenever a packet is available or an event occurs. This ISR would typically read the interrupt cause registers from QCA7000 to determine what happened.
* For a **packet available event**, the ISR (or a deferred routine it triggers) reads the packet from QCA7K and then processes it or stores it in a buffer for the main loop to handle (depending on desired context separation).
* For transmitting, the main loop (or a higher-priority function) could directly call the driver to send data when needed (ensuring not to conflict with any ongoing SPI from the ISR). Because the ESP32 has multiple cores and can handle concurrent tasks even without a full RTOS, one strategy is to dedicate one core to running a main loop managing QCA7000 I/O and network processing, leaving the other core for application logic. If only one core (or if simplicity is desired), the main loop may handle periodic tasks and polling in between network events.
* Concurrency considerations: In bare-metal, if using an ISR for RX while the main loop can also initiate SPI for TX, you need to ensure the SPI bus is not accessed concurrently. Typically one would disable interrupts or set a flag when a transmit is in progress, or handle all SPI transfers from within a critical section. Simpler designs might handle both RX and TX sequentially in the main loop via polling (for example, polling a "packet available" flag periodically), but this increases latency and risk of missing frames under high traffic. Therefore, at minimum, using the hardware interrupt for RX is recommended.

**RTOS (FreeRTOS/ESP-IDF) Integration:** With an RTOS, you can take advantage of tasks and synchronization:

* **Receive Task:** Configure the QCA7000 IRQ to trigger a GPIO ISR. In that ISR, instead of doing heavy work, use an interrupt service routine to notify a task (e.g., give a binary semaphore or send a queue event). A dedicated *QCA7000 RX Task* can then unblock and handle the SPI communication to fetch the packet(s). This task would then pass the raw packet to the networking stack. In ESP-IDF, for instance, you might call `esp_netif_receive()` or `ethernetif_input()` (depending on IDF version) to hand the packet to the TCP/IP stack.
* **Transmit Path:** The network stack will call a driver-provided output function whenever it has a packet to send. In ESP-IDF’s model, you implement a `transmit` function (provided in `esp_netif_driver_ifconfig_t`) that the stack calls with a packet buffer. This function can either send immediately (if it can acquire the SPI bus) or enqueue the packet if the QCA’s buffer is full. A simple approach is to protect the SPI transfer with a mutex (to prevent collision with any concurrent RX task SPI use) and perform the SPI write in the context of the calling task (which might be the TCP/IP task). Alternatively, one can have a dedicated *QCA7000 TX Task* that waits on a queue of outgoing packets: the transmit function then just posts the packet to the queue and signals the TX task. The TX task serializes all SPI write operations to QCA7000. This approach is clean and ensures only one context touches SPI at a time, at the cost of some latency. For many cases, though, directly performing the SPI send in the transmit callback (with appropriate locking) is fine, since it offloads the work to the core running the network task.
* **Concurrency & Synchronization:** Use FreeRTOS synchronization primitives to guard shared resources. For example, wrap all SPI register or data accesses in a mutex (`xSemaphoreTake`/`Give`) if they can be called from different tasks. The interrupt (ISR) must also coordinate: often the pattern is ISR gives a semaphore to the RX task, and the RX task will take the mutex, do SPI reads, etc. To avoid deadlock, you ensure the TX path either doesn’t interrupt an active RX or vice versa. If using separate tasks, a single mutex around SPI bus operations suffices. If implementing within one task (e.g., handle TX and RX sequentially in the same context), then a mutex might not even be needed – just ensure the ISR defers processing to that task.
* **Memory buffers:** In an RTOS, incoming data might be allocated as pbufs (if using lwIP). The driver can allocate a pbuf for the incoming frame (pbuf\_alloc) and then copy the frame data from the SPI buffer into it, or use zero-copy if possible (e.g., reading directly into a pre-allocated buffer in a pbuf). For outgoing data, the stack will often pass a pbuf chain; the driver might need to copy that into a contiguous buffer with the QCA7K framing. Some efficiency can be gained by prepping the SPI transfer directly from the pbuf payload if the pbuf has contiguous data (most Ethernet frames will fit in one pbuf unless chained).
* **Network Interface Configuration:** With ESP-IDF’s esp\_netif (or older tcpip\_adapter), you typically create a custom netif for the QCA7000. For example, you would configure an `esp_netif_config_t` with `.driver = qca_driver` and implement `esp_netif_driver_ifconfig_t` for it (setting the `transmit` and `driver_free_rx_buffer` callbacks to your functions, plus a `post_attach` to tie things together). The forum discussion indicated the need to attach a custom driver; essentially, you set up those hooks so the system knows how to send packets out and how to free RX buffers after the stack is done. Once integrated, the QCA7000 interface can be brought up like a normal Ethernet interface (assign an IP, etc.), and the stack will use the provided callbacks to exchange data.

In both bare-metal and RTOS scenarios, the core logic for interacting with the QCA7000 (SPI transactions, frame formatting, register handling) remains the same. The difference lies in how you schedule these operations and ensure they cooperate with the rest of the system. The following sections delve into the details of the QCA7000’s SPI register protocol, framing, and data flow, which form the foundation of the driver’s implementation.

## Register-Level Protocol

Communicating with the QCA7000 over SPI involves two types of accesses: **internal register operations** and **external buffer (data) transfers**. The QCA7000 defines a simple 16-bit command word that precedes each SPI transaction to specify the type of operation and address. Following the command phase, data is transferred (either read or written) according to the command. All internal registers are 16 bits wide, and external buffer transfers can be of variable length (configured via a register). Below is a summary of relevant registers and their usage:

| Register Name                  | Address (hex) | Access     | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| ------------------------------ | ------------- | ---------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **SPI\_REG\_BFR\_SIZE**        | 0x0100        | Write-only | **Buffer Size:** Host sets this 16-bit register to the number of bytes for the next external transfer (read or write). This must be done *before* initiating an external read or write command.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| **SPI\_REG\_WRBUF\_SPC\_AVA**  | 0x0200        | Read-only  | **Write Buffer Space Available:** Indicates how many bytes of free space remain in QCA7000’s internal buffer for writing (TX from host). After reset, this value is typically 0x0C5B (3163 bytes), reflecting total buffer capacity before any data is sent. The host should not reserve more bytes than this value for a write; doing so is “not allowed” and will cause an error.                                                                                                                                                                                                                                                                                                                                           |
| **SPI\_REG\_RDBUF\_BYTE\_AVA** | 0x0300        | Read-only  | **Read Buffer Bytes Available:** Indicates how many bytes of data are ready to be read from QCA7000’s internal read buffer (RX for host). The host must not attempt to read more bytes than this value (else a read overflow error occurs). Typically, this will correspond to the size of one or more received frames currently buffered.                                                                                                                                                                                                                                                                                                                                                                                    |
| **SPI\_REG\_SPI\_CONFIG**      | 0x0400        | Read/Write | **SPI Configuration Register:** Contains configuration bits for the SPI interface. The key bit here is the **Slave Reset bit (bit 6)** – writing a `1` to this bit triggers a software reset of the QCA7000’s internal processor. This is used to restart the QCA7000 if needed. Other bits include possibly the **Multi-CS (Legacy Mode) enable** bit. In QCA’s registers, there is a bit (bit 1 in the SIGNATURE/CONFIG region, see below) controlling legacy vs burst mode. By default (strapping), it may be `1` (legacy enabled) or `0`. The recommended operation is to leave all other bits unchanged (preserve their values) when modifying SPI\_CONFIG – i.e., do a read-modify-write to set or clear specific bits. |
| **SPI\_REG\_INTR\_CAUSE**      | 0x0C00        | Read/Write | **Interrupt Cause Register:** After an interrupt, the host reads this 16-bit register to determine which event(s) triggered the interrupt. Each bit corresponds to a cause (documented below). Writing a `1` back to those bit positions clears the respective interrupt cause. This register is typically **write-to-clear**; the QCA7000’s interrupt line will deassert once all asserted causes are cleared. (It’s worth noting that writing the same content back, as advised, clears all pending causes at once.)                                                                                                                                                                                                        |
| **SPI\_REG\_INTR\_ENABLE**     | 0x0D00        | Read/Write | **Interrupt Enable Register:** Enables or masks specific interrupt causes from triggering the QCA7000’s interrupt pin. A ‘1’ in a bit position enables that interrupt cause. The bit definitions mirror those in INTR\_CAUSE. The host uses this to control which events should generate interrupts.                                                                                                                                                                                                                                                                                                                                                                                                                          |
| **SPI\_REG\_SIGNATURE**        | 0x1A00        | Read-only  | **Signature Register:** Always reads as a fixed 16-bit pattern `0xAA55`. This is used to confirm endianness and basic SPI communication is working. The signature serves as a handy **alive test** – if the value read is not 0xAA55, the host and QCA7000 are out-of-sync or misconfigured. Reading this register twice after reset is part of the initialization sequence (detailed later).                                                                                                                                                                                                                                                                                                                                 |

**SPI Command Format:** The first 16 bits clocked out by the master (ESP32) after asserting CS constitute the command. The bits are defined as follows:

* **Bit 15** – Read/Write flag: `1` for read operations, `0` for write operations.
* **Bit 14** – Internal/External address flag: `1` for *internal register* access (address mode), `0` for *external buffer* access (buffer mode).
* **Bits 13:0** – Address field: If Internal/External = 1, these bits specify the 14-bit address of the internal register to access. If Internal/External = 0 (external/buffer mode), these bits must be all zero (the buffer is not addressed by offset here).

For **internal register writes**: the host will send the 16-bit command (bit15=0, bit14=1, addr set) followed by 16 bits of data (in the next 16 clock cycles) to write the register. For **internal reads**: the host sends the 16-bit command (bit15=1, bit14=1, addr set), then continues to clock 16 more bits; the QCA7000 will output the 16-bit register value on MISO during this data phase. In both cases, the entire 32-bit transaction (command + data) must be enclosed in one CS assertion (CS low). The SPI host *must* send/receive the full 16 bits of data in the same transaction after the command; otherwise the QCA7000 will not latch or return the register properly (it doesn’t support splitting the read/write across multiple CS toggles, since legacy mode expects CS toggling per *word* not half-word).

For **external buffer writes**: the sequence is (a) write SPI\_REG\_BFR\_SIZE with the length, (b) assert CS, send a 16-bit command with bit15=0 (write) and bit14=0 (external), which effectively is `0x0000` (since address bits are all 0), then (c) send the specified number of data bytes over MOSI in one continuous burst. The QCA7000 will write these bytes into its internal “write buffer”. The CS is then deasserted. It is important that the number of bytes sent exactly matches the count previously written into BFR\_SIZE – no more, no less. If fewer bytes are sent than BFR\_SIZE, the QCA7000 will wait (its internal SPI engine will not complete the transaction properly, possibly causing a lock or requiring recovery). If more bytes are clocked than BFR\_SIZE, the additional bytes might be discarded or could overflow the buffer, and an error will be flagged. In practice, the SPI master (ESP32) should structure the transaction such that it sends exactly the intended number of bytes then raises CS. (When using ESP-IDF’s SPI APIs, you can set the transaction length accordingly; if using registers directly, ensure to count bytes and manually control CS).

For **external buffer reads**: similarly, (a) the host reads SPI\_REG\_RDBUF\_BYTE\_AVA to determine how many bytes are available to read (or uses the value provided by interrupt cause), (b) writes that value to BFR\_SIZE, then (c) asserts CS and sends a 16-bit command with bit15=1 (read) and bit14=0 (external), i.e., `0x8000`. After sending the command, the host continues to clock out bits while reading from MISO – the QCA7000 will output exactly BFR\_SIZE bytes (the SPI engine in QCA will sequentially send bytes from its read buffer FIFO). The host should provide dummy data on MOSI during this read (the MOSI input is ignored by QCA in external read operations). Again, CS must remain low until all bytes are read, and the host must clock out exactly the number of bytes requested. If the host clocks fewer bytes than BFR\_SIZE and deasserts CS early, not only will the frame be incomplete, but the QCA7000’s internal pointer will be left mid-frame, effectively **out-of-sync**. Subsequent reads would then start mid-stream, leading to corrupted data and likely prompting the signature check to fail. In such a case, the QCA7000 would probably raise an RDBUF error interrupt. The only safe recovery is to reset or flush the QCA’s buffers (there isn’t a documented “discard rest of buffer” command aside from reading it out or resetting the chip).

**Internal Registers and Byte Order:** The QCA7000’s registers are 16-bit; multi-byte values in those registers (like BFR\_SIZE, available bytes, etc.) are given in **little-endian** format when read/written over SPI. However, note that the SPI itself shifts bits MSB-first. The host should construct the 16-bit command and data words in big-endian when sending over the wire so that the correct little-endian value is written. For example, to write `0x0100` into BFR\_SIZE (0x0100 is the address) with a value of 100 (0x0064), the host would send command 0x4100 (0x4000 for internal write + 0x0100 address) followed by 0x0064 as data. On the wire, the bytes sequence would be: 0x41 0x00 0x64 0x00 (since ESP32 SPI sends MSB first by default). The QCA7000 would interpret that as address 0x0100, value 0x0064 (which in little-endian is 100 decimal). Reading registers returns the 16-bit value; the host may need to swap bytes depending on how it reads them. A good test is the SIGNATURE register: QCA will output 0xAA 0x55 in that order on MISO. If the host reads two bytes and combines them little-endian, it should get 0x55AA if it naively treats the first byte as LSB. The correct way is to assemble as MSB-first (0xAA as high byte, 0x55 as low byte) to get 0xAA55. Many SPI libraries handle this by letting you specify endianness for 16-bit transfers or by manually swapping. The QCA7000’s Linux driver explicitly calls `cpu_to_be16()` when preparing the 16-bit command, to ensure proper ordering on the SPI bus.

**Relevant Interrupt Bits:** The INTR\_CAUSE/ENABLE registers contain several bit flags. According to documentation and known driver code, the main bits are:

* **PKT\_AVLBL (Packet Available)** – Bit 0. Indicates that one or more Ethernet frames are available to read (i.e., the read buffer is non-empty). The recommended reaction is to perform an external read to retrieve the packet.
* **RDBUF\_ERR (Read Buffer Error)** – Bit 1. This typically means the host attempted to read more data than was available (buffer underflow) or possibly that the QCA7000’s internal read buffer overflowed because the host wasn’t keeping up. The recommended reaction is to perform a QCA7000 restart via the slave reset bit (essentially, reset the QCA7000, since this condition indicates loss of data synchronization).
* **WRBUF\_ERR (Write Buffer Error)** – Bit 2. This likely means the host tried to write more data than the QCA’s buffer could accept (buffer overflow) or some misalignment in a write operation. The reaction is also to reset the QCA7000 via the reset bit.
* **(Reserved or unused)** – Bits 3–5 are not defined for known events in QCA7000 (they might be reserved; the cause register jumps from bit2 to bit6 in documented flags).
* **CPU\_ON (CPU On/Reset Complete)** – Bit 6. This indicates that the QCA7000 has (re)booted and its internal CPU is running. Essentially, QCA7000 is telling the host “I just powered on or reset.” This is seen at initial power-up or after a software reset (either issued by the host or perhaps due to a watchdog inside QCA). The recommended reaction is to perform the initial setup sequence again (read signature, etc., see Initialization section).
* **WRBUF\_BELOW\_WM (Write Buffer Below Watermark)** – Bit 10 (as defined in some driver headers). This bit is not mentioned in older app notes but appears in newer driver definitions. It likely means that the QCA7000’s internal write buffer has emptied below a certain threshold or is now available to accept more data. In other words, if the host had previously filled the QCA’s buffer (or nearly) and stopped sending, the QCA can signal when there is space again. Not all implementations use this, and many drivers leave it disabled, because the host can always poll `WRBUF_SPC_AVA` or simply attempt sends periodically. However, enabling this interrupt could allow an event-driven wakeup of a sending task when the QCA is ready for more data. Use of this feature is optional – for simplicity, one can design the host to only send when it has a full frame ready and check space at that time.

The host should **enable** the relevant bits in INTR\_ENABLE after initialization. Typically, you’d enable `PKT_AVLBL`, `RDBUF_ERR`, `WRBUF_ERR`, and `CPU_ON` (bits 0,1,2,6). `WRBUF_BELOW_WM` (bit10) can be enabled if you plan to handle that event, but it can also be left disabled if your transmit logic is straightforward.

**Partial Transfers and Edge Cases:** The QCA7000’s protocol does not support splitting an external transfer arbitrarily. The **BFR\_SIZE mechanism** exists to ensure the host and QCA agree on exactly how many bytes will be exchanged in the next transfer. Some edge cases and rules to follow:

* Always set SPI\_REG\_BFR\_SIZE *exactly* to the number of bytes you intend to transfer in the next operation, and never exceed the available space/bytes reported. If you violate this (e.g., set a size larger than available), the QCA7000 will not allow it – you will either get no response or an immediate error (WRBUF\_ERR or RDBUF\_ERR).
* Do not deassert CS (don’t end the SPI transaction) until all bytes in an external transfer are clocked. If you accidentally end the transaction early (sending fewer bytes than BFR\_SIZE or stopping mid-transfer due to an SPI error), the QCA’s internal buffer pointers will be misaligned. There is no command to “undo” or resume that partial transfer; typically the next best step is to issue a **slave reset** (or possibly read the signature to detect the out-of-sync condition and then reset). Partial transfers will likely trigger an error cause as well.
* Misaligned buffer access: All internal register accesses must be 16-bit aligned. The QCA7000 essentially ignores any SPI operations that are not 16-bit multiples when in register mode. The host’s SPI controller should send commands in 16-bit chunks for internal registers. (In practice, this means if you use an 8-bit transfer for a 16-bit register read, you’ll only send half the command, which obviously fails. Most high-level SPI APIs treat the command+data as a unit, so this is only a concern in low-level bit-banging scenarios.)
* **Endianness pitfalls:** If you read the signature register and get 0x55AA instead of 0xAA55, this indicates a byte swap issue – the bytes on MISO are correct, but you combined them incorrectly on the host side. The host driver should correct for this so that all multi-byte fields from QCA (e.g., lengths, buffer counts) are interpreted correctly. The QCA7000’s protocol is essentially little-endian for multi-byte values (frame lengths, etc., are transmitted LSB first in the framing as we’ll see), so keep that consistent.
* The **signature register** is a great sanity check: reading it twice can also help flush the SPI input. The first read after reset might be ignored or return a stale value if the QCA7K wasn’t fully ready, which is why the initialization uses two reads (the first is ignored). Always ensure you can reliably read 0xAA55 before proceeding with normal operation.
* **Concurrent access:** Make sure only one SPI command is in flight at a time. The QCA7000, being SPI slave, expects a single master stream. If using DMA or an RTOS with tasks, avoid scenarios where two tasks try to talk to QCA simultaneously. This can be enforced with a mutex or by designing a single point of SPI access.
* **Atomic register operations:** For registers like SPI\_CONFIG where only one bit (reset) is to be changed, read the register first then write back the modified value (R-M-W). Do not blindly write constants to SPI\_CONFIG, as you might inadvertently change or clear other bits (some bits in SPI\_CONFIG could be undocumented or reserved flags).
* **Time between operations:** Normally, operations can happen back-to-back. If you issue a BFR\_SIZE and then an external read/write, QCA7K expects those in sequence. If you delay too long after setting BFR\_SIZE before starting the transfer, it’s usually fine – the value simply sits until used. However, avoid doing any other internal register operations between setting BFR\_SIZE and the corresponding data transfer; it’s meant to apply to the very next external access. (If you had to do something else, you could always rewrite BFR\_SIZE again later, but best to keep it paired closely with the actual transfer.)
* **Unaligned frame sizes:** Ethernet frames can be any length from 60 bytes to 1518 bytes (not including FCS). The QCA7K framing will pad if needed (frame smaller than 60 gets padded with zeros to 60), so the host should either pad the Ethernet payload itself or rely on QCA? Actually, per spec, frames < 60 bytes (without FCS) should be padded to 60. The QCA7000 likely expects the host to include padding in the frame data it sends (since it states “in case the frame body is smaller than 60 bytes it needs to be padded with zero”). So the driver must ensure to pad short frames. Conversely, when receiving, if a small frame comes in, it will already be padded (the FL field will reflect the original length and the data beyond that up to 60 will be zeros).
* **Multiple frames in buffer:** It’s possible for QCA7000 to accumulate multiple received frames in its RX buffer if the host doesn’t read them out quickly. The protocol (as we will see in Framing) tags each frame with start-of-frame markers and lengths, so the data in the buffer is self-delimiting. The host can read frames one by one or potentially multiple at once. However, typically the driver will read one full frame per interrupt (because the interrupt signals at least one frame is available). After reading one frame, if another is still buffered, QCA may either assert a new interrupt or the driver can simply check RDBUF\_BYTE\_AVA again in a loop. The safe approach: **always read exactly one frame at a time** (as indicated by its internal length field or by reading only the first frame’s length from RDBUF). Reading multiple frames in one go would require the host to parse multiple EOF markers in a single SPI burst. The app note suggests doing one external read per PKT\_AVLBL event. In practice, if the QCA7K sets PKT\_AVLBL and you find RDBUF\_BYTE\_AVA is, say, 3000 bytes (meaning possibly two large frames), you could either read all 3000 at once and then split frames in software, or read just the first frame. The challenge is that RDBUF\_BYTE\_AVA likely reports the total queued bytes, not just one frame. The QCA7K doesn’t have an explicit “one frame length” register, but it does prefix each frame with a length field (LEN/SOF, covered next). So a strategy is:

  * Read the first 4 bytes of the buffer (which is the hardware LEN of the first packet) by doing a small external read (set BFR\_SIZE=4 and do read).
  * From that, extract the total length of that packet (which likely includes the whole frame + overhead). Then you know how many more bytes to read to get the rest of frame.
  * Then read the remaining bytes for that frame.
  * This is complicated and not strictly necessary if you trust the interrupt to indicate one frame at a time. It’s probably easier to just do: read RDBUF\_BYTE\_AVA (maybe it gives exactly one frame’s size if the QCA firmware is designed to queue frames individually).

  The Linux driver and app note approach is simpler: they assume one interrupt corresponds to at least one frame and they loop reading frames until RDBUF\_BYTE\_AVA is zero. So implement: while (RDBUF\_BYTE\_AVA > 0) { read that many bytes (one frame) }. If multiple frames were there, the first read will consume the first frame (and perhaps exactly that length), then QCA might update RDBUF\_BYTE\_AVA to the remaining bytes (which would correspond to next frame length). This works only if QCA’s RDBUF\_BYTE\_AVA reduces accordingly after each read. Since reading exactly one frame’s worth is tricky without knowing its size upfront, it implies RDBUF\_BYTE\_AVA might initially only report the first frame’s total bytes rather than the whole queue. That might indeed be the case – QCA might not lump them; it could internally queue but present them sequentially. Without absolute clarity, the robust method is to handle possibly multiple frames per interrupt in a loop, using the framing to parse if needed. We’ll proceed assuming one frame at a time is retrieved (which matches typical usage).

In the next section, we describe the framing of Ethernet packets over SPI, which ties into how the host formats transmit data and interprets received data.

## Framing Layer

Because the QCA7000 encapsulates Ethernet frames for transfer over serial (SPI or UART), a specific **framing format** is used to delineate packets and provide length information and markers for validation. This framing is applied to each Ethernet packet payload exchanged. The framing differs slightly between transmit (host-to-QCA) and receive (QCA-to-host), particularly with an extra length field on receive. Below we detail the format of these frames and the logic to encode/decode them.

**Transmit Frame Format (Host → QCA7K):** When sending an Ethernet frame from the ESP32 to the QCA7000, the host must wrap the raw Layer 2 frame with a header and footer, as shown below:

* **SOF (Start of Frame):** 4 bytes, constant value `0xAAAAAAAA`. This serves as a marker indicating the beginning of a frame. In memory, you would place the bytes 0xAA 0xAA 0xAA 0xAA at the start of the buffer.
* **FL (Frame Length):** 2 bytes, little-endian. This is the length of the Ethernet frame payload (from destination MAC through the end of payload, excluding FCS). Minimum is 60, maximum 1518 bytes for standard Ethernet (or 1522 if a VLAN tag is present). Essentially, this equals the number of bytes in the actual Ethernet frame that will follow. If the original frame is smaller than 60, it must be padded to 60 (and FL should be set to 60).
* **RSVD (Reserved):** 2 bytes, must be `0x0000`. These are reserved for alignment and future use. They ensure that the payload starts on a 4-byte boundary offset (the SOF is 4 bytes, FL+RSVD 4 bytes, so payload starts at an 8-byte offset which is 4-byte aligned).
* **ETH Frame Payload:** This is the actual Ethernet frame content (Layer 2 data) without the frame check sequence (FCS). It includes: Destination MAC (6 bytes), Source MAC (6 bytes), optional 802.1Q VLAN tag (4 bytes) if present, Ethertype (2 bytes), and payload data. The length of this section is given by the FL field. If FL < 60, the payload should be padded with zeros up to 60 bytes.
* **EOF (End of Frame):** 2 bytes, constant value `0x5555`. This marks the end of frame.

Putting it together: if an application wants to send an Ethernet frame of length 100 bytes (which is above minimum), the host will allocate (or use a static buffer) for 100 + 8 (SOF+FL+RSVD) + 2 (EOF) = 110 bytes total. It will populate:

```
buf[0..3]   = 0xAA 0xAA 0xAA 0xAA            (SOF)
buf[4..5]   = 0x64 0x00                      (FL = 100, in little-endian -> 0x0064)
buf[6..7]   = 0x00 0x00                      (RSVD = 0)
buf[8..107] = [100 bytes of Ethernet frame data] (destination MAC through payload)
buf[108..109] = 0x55 0x55                    (EOF)
```

These 110 bytes would then be sent via the SPI external write procedure (after writing BFR\_SIZE=110). The QCA7000 upon receiving this through SPI knows that the frame starts after the 8-byte header, and it will process the Ethernet content accordingly (and ultimately transmit it over PLC).

**Receive Frame Format (QCA7K → Host):** When the QCA7000 delivers a frame to the host, it prepends an additional length field at the very start and then follows a structure similar to the transmit format:

* **LEN (Overall Length):** 4 bytes. This is a hardware-generated field indicating the total length of the received frame *plus* the framing overhead that follows. Essentially, it’s the number of bytes in this entire SPI packet (from SOF through EOF). It allows the host to know how many bytes to expect or to validate the length. When reading from QCA, typically the host already knows how many bytes to read (from RDBUF\_BYTE\_AVA or BFR\_SIZE), and that should match this LEN. Think of LEN as a redundant safety check.
* **SOF:** 4 bytes, `0xAAAAAAAA` as in transmit. Should always appear immediately after the LEN.
* **FL (Frame Length):** 2 bytes, little-endian. Length of the Ethernet frame contained (same definition as in transmit).
* **RSVD:** 2 bytes, `0x0000` reserved for alignment.
* **ETH Frame Payload:** The actual Ethernet frame (no FCS), of length FL bytes. If FL < 60, zeros will appear here as padding up to 60.
* **EOF:** 2 bytes, `0x5555` marking end of frame.

The offset positions (if LEN is at 0x0000) given in the app note are:

* LEN: offset 0x0000, 4 bytes
* SOF: offset 0x0004, 4 bytes
* FL: offset 0x0008, 2 bytes
* RSVD: offset 0x000A, 2 bytes
* Frame: offset 0x000C, (FL bytes, minimum 60)
* EOF: offset (0x000C + FL) = 0x000C + FL, 2 bytes.

For example, if a 100-byte Ethernet frame was received, FL=100. The total bytes from SOF to EOF would be 4 (SOF) + 2 (FL) + 2 (RSVD) + 100 (frame) + 2 (EOF) = 110 bytes. LEN (4 bytes) in that case would likely be 110 (0x0000006E in little-endian) or possibly 114 including itself? It says “LEN field is hardware generated packet length”. It’s reasonable to assume LEN includes everything *after* it (SOF through EOF), or possibly the entire thing including itself. We can deduce: If RDBUF\_BYTE\_AVA equals the number of bytes host needs to read, does that equal LEN+4 or just LEN? The simplest design is that LEN includes the bytes from SOF to EOF (so in the above example, LEN=110). Then RDBUF\_BYTE\_AVA (16-bit) would be 110 + 0? Actually, wait, RDBUF\_BYTE\_AVA is a 16-bit register, so it can only report up to 65535 bytes, which is fine since frames are <= \~1530. If QCA had multiple frames queued, RDBUF might reflect more, but if we assume one at a time, RDBUF\_BYTE\_AVA should equal LEN+4 (if LEN doesn’t count itself) or LEN (if LEN does count itself). The exact interpretation isn’t explicitly clear from the snippet, but likely:
\- LEN = FL + 8 (header/trailer) in the simplest sense, or
\- LEN = FL + 10 (header/trailer plus maybe counting the two bytes of EOF and reserved? Actually header is 4(SOF)+2(FL)+2(RSVD) + 2(EOF) = 10 bytes overhead, plus the frame itself).
Actually, overhead is 4(SOF)+2(FL)+2(RSVD)+2(EOF) = 10 bytes. So if FL=60, overhead is 10, total = 70. They suggested something like offset 0x004A for EOF if FL minimum, which was confusing, but anyway.

In any case, the host doesn’t need to calculate LEN because it is given by RDBUF\_BYTE\_AVA or by the LEN field itself. The host should:

* Use RDBUF\_BYTE\_AVA value as the number of bytes to read via SPI.
* Once data is read, verify the LEN field in the first 4 bytes. If it matches the number of bytes (minus the 4 of LEN itself) you expected, good. If not, something’s off (potential out-of-sync).
* Verify the SOF = 0xAAAAAAAA and EOF = 0x5555 to ensure frame integrity.
* Use the FL field to know the actual payload length and to extract the Ethernet frame.

**State Machine for Frame Parsing:** In the host driver, especially if reading a stream of bytes (like on UART or if reading multiple frames in one go), you would implement a simple state machine (FSM) to validate and extract frames:

1. **IDLE/SEARCH\_SOF:** Looking for 0xAAAAAAAA sequence. (In SPI, if you always start read at frame boundary, you won’t need to search; but on UART stream, you might have to byte-wise find SOF).
2. **READ\_HEADER:** Once SOF is found, read the next fields (FL, RSVD). Ensure RSVD is 0x0000 (if not, frame might be misaligned or corrupted).
3. **READ\_PAYLOAD:** Read `FL` bytes of payload into a buffer.
4. **VERIFY\_EOF:** Expect 0x5555 following the payload. If the EOF marker is correct, you have a complete frame. If not, then the data is corrupted or mis-framed; an error handling routine should kick in (e.g., reset state machine, maybe flush input until the next potential SOF).
5. **PASS\_UP\_FRAME:** Deliver the extracted payload (Ethernet frame) to the higher layer (network stack) for processing.
6. **Loop to IDLE:** After EOF, if more data is in the stream (e.g., another frame contiguous), you might immediately see the next frame’s SOF or length.

On SPI, since we typically retrieve one frame at a time by design, the FSM can be simplified: you always start at the beginning of a frame (because you read in chunks as delivered by QCA). Thus, you expect the first 4 bytes to be LEN (on receive) or 0xAAAAAAAA (if you decided to exclude LEN for some reason). In practice, because you know exactly how many bytes were read, you can parse them in one go:

* Check bytes \[4..7] of the received buffer for SOF (since \[0..3] are LEN on RX).
* Check the last two bytes for EOF.
* Check that the FL field \[8..9] plus 10 equals the total length minus 4 (depending on interpretation of LEN).
* If all checks out, extract the frame at \[12 .. 11+FL] (i.e., starting at offset 0x000C, length FL).
* If any check fails, log an error (and likely reset QCA or flush buffers). A common check is the signature register read if things seem off – if you suspect misalignment, reading signature 0x1A00 should still return 0xAA55, otherwise you know the SPI stream is out-of-sync and a reset is required.

**Encoding Example (Transmit):** Here’s pseudo-code to **encode an outgoing frame**:

```c
uint16_t ether_len = frame_len;  // length of actual Ethernet frame data (without FCS)
if (ether_len < 60) {
    // Pad short frames to minimum 60 bytes
    memset(frame + ether_len, 0, 60 - ether_len);
    ether_len = 60;
}
uint16_t total_len = ether_len + 8 + 2; // 8-byte header, 2-byte EOF
// Allocate buffer for SOF+hdr+payload+EOF
uint8_t *tx_buf = malloc(total_len);
if (!tx_buf) return ERROR;
 
// Fill framing
*(uint32_t*)(tx_buf + 0) = __builtin_bswap32(0xAAAAAAAA);   // SOF (0xAAAAAAAA). Ensure correct endianness if needed
*(uint16_t*)(tx_buf + 4) = ether_len;                       // FL (little-endian representation of ether_len)
*(uint16_t*)(tx_buf + 6) = 0x0000;                          // RSVD = 0
memcpy(tx_buf + 8, frame_data, ether_len);                  // copy Ethernet frame (now padded to ether_len)
*(uint16_t*)(tx_buf + 8 + ether_len) = 0x5555;              // EOF = 0x5555
```

A few notes: we used `__builtin_bswap32(0xAAAAAAAA)` to account for a potential endianness issue in writing 0xAAAAAAAA directly; depending on the system’s endianness, writing a 32-bit constant might need swapping. But one can also just do `tx_buf[0..3] = {0xAA,0xAA,0xAA,0xAA}` explicitly to avoid any ambiguity. The rest we write little-endian (assuming the CPU is little-endian, writing a 16-bit value directly places it in little-endian in memory).

Now, after preparing `tx_buf`, the driver would check `WRBUF_SPC_AVA` to see if `total_len` bytes can be written. If yes, it will write `total_len` to SPI\_REG\_BFR\_SIZE, then perform the SPI transfer with command=write external and send `total_len` bytes from `tx_buf`. If not enough space, the driver may queue this frame or wait until space is available (see Data Transmission section).

**Decoding Example (Receive):** Pseudo-code to **decode a received frame** from a buffer `rx_buf` of length `n_bytes` read via SPI:

```c
// rx_buf length n_bytes was obtained from RDBUF_BYTE_AVA (which should equal n_bytes)
if (n_bytes < 14) {
    // Minimum frame size is 60 + overhead 10 + LEN 4 = 74 bytes; 
    // if we got less than that, something’s wrong (maybe a fragmented read)
    error("Short read");
    goto reset_or_recover;
}
uint32_t rx_len_field = *(uint32_t*)(rx_buf + 0);
uint32_t rx_len_val = rx_len_field; // already little-endian in memory if host is little-endian
// Alternatively, reconstruct: rx_len_val = rx_buf[0] | (rx_buf[1]<<8) | ... etc.
 
// Verify LEN matches expectation (which is n_bytes - 4 or n_bytes?). 
// Often, rx_len_val should equal (n_bytes - 4). Let's assume LEN = bytes after it:
if (rx_len_val != (n_bytes - 4)) {
    error("Length mismatch: LEN=%u, expected=%u", rx_len_val, n_bytes - 4);
    // It's possible QCA includes the 4 bytes of LEN in that count (so check if rx_len_val == n_bytes).
    if (rx_len_val != n_bytes) {
        goto reset_or_recover;
    }
}
 
// Check SOF
uint32_t sof_field = *(uint32_t*)(rx_buf + 4);
if (__builtin_bswap32(sof_field) != 0xAAAAAAAA) {  // assuming little-endian host, need swap to compare constant
    error("Bad SOF marker");
    goto reset_or_recover;
}
// Or simply check bytes: if(rx_buf[4]!=0xAA || rx_buf[5]!=0xAA || rx_buf[6]!=0xAA || rx_buf[7]!=0xAA) error.
 
uint16_t FL = *(uint16_t*)(rx_buf + 8);  // little-endian frame length
uint16_t rsvd = *(uint16_t*)(rx_buf + 10);
if (rsvd != 0x0000) {
    error("Non-zero reserved field");
    // Not critical enough to reset, but indicates potential misalignment. We can attempt to continue.
}
 
if (FL < 60) frame_len = 60;
else frame_len = FL;
 
if (frame_len + 10 != rx_len_val) {
    // The LEN should equal FL + 10 (SOF(4)+FL(2)+RSVD(2)+EOF(2)) if LEN excludes itself.
    // If LEN included itself, then LEN = FL+10+4, and rx_len_val (if equal n_bytes-4) effectively was FL+10.
    error("Length field mismatch: FL=%u, LEN indicates %u bytes payload+overhead", FL, rx_len_val);
    // We can try to recover by using whichever is consistent.
}
 
// Verify EOF marker at end:
uint16_t eof_marker = *(uint16_t*)(rx_buf + n_bytes - 2);
if (eof_marker != 0x5555) {
    error("Missing EOF marker");
    goto reset_or_recover;
}
 
// If all checks passed:
uint16_t eth_frame_len = FL; // actual Ethernet frame length (could be <60 or >60 up to 1518).
// Allocate or reuse a buffer/pbuf for eth_frame_len bytes
uint8_t *eth_frame = allocate_buffer(eth_frame_len);
memcpy(eth_frame, rx_buf + 12, eth_frame_len);  // copy the payload (starting at offset 0xC)
 
// At this point, eth_frame contains the raw Ethernet frame to pass to network stack.
```

In a real implementation, a lot of these checks could be simplified or omitted in the interest of speed (especially once the driver is stable). But for debugging and reliability, it’s good to have sanity checks like SOF/EOF verification. The QCA7000’s use of known patterns (0xAAAAAAAA and 0x5555) is intentionally to help detect misalignment. If, for example, the host lost a byte somewhere, you might find the expected 0x5555 doesn’t show up where it should, or the SOF isn’t in the expected place.

**Frame Validation using FSM:** The above logic can be conceptualized as a state machine:

* **State 0 (Waiting for Frame):** Idle until an interrupt indicates a frame, then transition to next state by initiating a read of that frame.
* **State 1 (Header)**: After reading the data, check the header fields (LEN, SOF, FL, RSVD). If any of these are wrong, go to an **Error state** (where recovery actions occur). If OK, transition to State 2.
* **State 2 (Payload)**: Process the payload of length FL (copy or pass pointer). Then transition to State 3.
* **State 3 (Footer)**: Verify EOF. If correct, we have a complete frame; report it to the higher layer (deliver frame). If EOF is incorrect, go to Error state.
* **State 4 (Done)**: Frame processed. If more data is in the buffer (unlikely in SPI case if we only read one frame at a time), you might loop back to State 1 to attempt parsing another frame from the remaining bytes.
* **Error State**: If any validation fails, the driver should probably reset the QCA7000 or reinitialize the SPI interface, because a framing error usually means sync is lost. The QCA app note explicitly recommends resetting the QCA7K if an RDBUF or WRBUF error occurs (which would be triggered by such a mismatch). The host might also flush any partially read data (though on SPI, partial data flush is basically just discarding what you got and resetting the chip’s buffer by a reset).

In summary, the FSM ensures that frames are properly delimited and valid, which is critical in a system like EV communication where reliability is paramount. With framing understood, we can now examine how data flows end-to-end and how to manage buffers and throughput.

## Data Transmission and Reception

This section covers the end-to-end data flow for sending and receiving data, how to handle multiple packets (queuing), and specifics of burst vs legacy mode in operation.

**Transmit Data Flow (ESP32 → EV over QCA7000):** From the perspective of the application to the powerline network:

1. **Application Layer:** An application or higher protocol (e.g., ISO 15118 stack or a simple TCP socket) generates data to send. This goes into the TCP/IP stack on ESP32 which encapsulates it in a TCP/IP packet, which then gets wrapped in an Ethernet frame (with appropriate source/destination MAC, etc.) via the network interface.
2. **Network Interface (Driver Output):** The ESP32’s network stack calls the QCA7000 driver’s transmit function with the Ethernet frame. In an RTOS, this might be triggered by `esp_netif_transmit()` or similar, providing a pointer to a buffer and length.
3. **Driver Prepares Frame:** The driver encapsulates the frame with SOF, length, etc., as described in the framing section. It also calculates the total size including overhead.
4. **Check and Reserve Buffer Space:** The driver reads `SPI_REG_WRBUF_SPC_AVA` (0x0200) to see how much space is available in QCA’s buffer. If the space available is >= the total frame size, it can proceed. If not (meaning the QCA7000’s internal buffer is currently full or busy transmitting previous data), the driver must handle this:

   * **Queuing:** The driver can add this frame to a software queue (waiting list) to send later. This queue could be as simple as one-deep (i.e., just wait and retry) or could buffer multiple frames if expecting bursts of data. The size of this queue is a design choice; a small queue (1-3 frames) might be sufficient because the QCA buffer itself can hold about 2 frames.
   * **Blocking/Waiting:** If using RTOS and you prefer not to queue many packets, the transmit function could block (with a timeout) until space is available. E.g., it could wait on a semaphore that is given by an “buffer available” interrupt (like WRBUF\_BELOW\_WM or simply after a frame is sent the driver can check again). In bare-metal, you might spin-wait for space or drop the packet if it can’t be sent immediately (depending on latency requirements).
   * **Flow Control to upper layers:** If the queue is full or the network is congested, the driver can signal backpressure. In TCP, this naturally happens if send buffers fill. If using UDP or other protocols, you may need to handle it (maybe dropping or returning an error from transmit function so the app knows).
5. **Write BFR\_SIZE and Transmit:** To send, the driver writes the total byte count to `SPI_REG_BFR_SIZE` (0x0100), then initiates the SPI transfer. The transfer is done in **burst mode** (CS held low throughout) containing the 16-bit external write command followed by the frame bytes. On ESP32 with ESP-IDF, this could be a single `spi_device_transmit()` call with a tx buffer of the entire prepared frame. In bare-metal, you’d manually toggle CS low, feed the command and data through the SPI hardware or bit-bang, then toggle CS high. The length must match exactly the BFR\_SIZE set.
6. **QCA7000 Buffering:** The QCA7000 receives the bytes into its internal buffer. After the transfer, QCA begins processing the frame. It will likely queue it for PLC transmission (according to its internal MAC/PHY scheduler). At this point, from the host’s perspective, the frame is “out the door” – the host driver can free the buffer it used. The QCA7000 will take it from here and eventually put it on the powerline (this might take some milliseconds depending on the PLC medium, but is out of host control).
7. **Post-Transmit:** The driver should update any statistics (like packets sent count). Also, the `WRBUF_SPC_AVA` register will have decreased by the frame size. If the driver has additional frames queued, it may check `WRBUF_SPC_AVA` again to see if the next frame can be sent (the space might free up only after QCA transmits or at least moves data out of the buffer). Typically, the internal buffer space frees when the QCA7K sends data over PLC or otherwise processes it. The `WRBUF_BELOW_WM` interrupt, if enabled, would notify the host that the buffer has space again, which is useful if the host was waiting to send another frame. If the driver is simply polling or trying to send back-to-back, it can also just attempt to send the next frame – if the space isn’t there, it will find out by reading `WRBUF_SPC_AVA` again.
8. **Legacy Mode Consideration (Transmit):** If the system were in legacy mode, step 5 would differ: The driver would have to break the data into 16-bit chunks and toggle CS for each. The QCA7000’s multi-CS mode expects each word to be framed by CS. On ESP32, this is inefficient: it would mean performing possibly hundreds of short SPI transactions per frame (a 100-byte frame is 50 words, so 50 transactions). This can drastically reduce throughput and increase CPU usage (due to overhead per transaction). That’s why we emphasize using burst mode. If legacy mode must be used (due to hardware strapped that way and not changeable at runtime), the driver might have to configure the ESP32’s SPI controller to automatically toggle CS every 16 bits, if such mode exists (the ESP32’s SPI can be configured in command/data mode, but auto-toggling per 16-bit chunk is not a typical feature; you might simulate it via queueing multiple transfers). Alternatively, bit-banging with manual control could achieve it, but again at huge cost. In summary, **avoid legacy mode unless absolutely required** – it’s essentially a fallback compatibility mode.

**Receive Data Flow (EV → ESP32 via QCA7000):** Now for incoming data from the PLC network:

1. **Frame Reception by QCA7000:** A remote device (EV or EVSE) transmits a PLC frame which is received by the QCA7000’s analog front-end and PLC PHY. The QCA7000’s internal MAC will process the PLC frame and, if it is a data frame for the host, decapsulate it to an Ethernet frame. This Ethernet frame (Layer 2) is then placed into QCA7000’s internal **read buffer**, wrapped with the QCA7K framing (LEN, SOF, etc.). The length of the packet is written into the appropriate fields.
2. **Interrupt to Host:** The QCA7000 asserts its IRQ line to notify the host that a packet is available. In the INTR\_CAUSE register, the PKT\_AVLBL bit (bit0) will be set. If properly enabled, this generates an interrupt to the ESP32. If interrupts were not enabled (e.g., during initialization or temporarily masked), the host could still discover the packet by polling `RDBUF_BYTE_AVA` periodically, but this is less efficient. Assuming interrupts are on:

   * The ESP32 receives a rising edge on the QCA\_IRQ GPIO. In an RTOS, this triggers the ISR which likely gives a semaphore to the networking task; in bare-metal, the ISR might set a flag and possibly even start servicing immediately if quick.
3. **ISR and Interrupt Handling:** (We will detail the general interrupt handling logic in the next section, but focusing on packet flow…) The host reads the `SPI_REG_INTR_CAUSE` register to see what caused the interrupt. It finds PKT\_AVLBL=1 (and possibly other bits if concurrent events). The driver will then handle the packet retrieval. Best practice as per QCA documentation is to **mask further interrupts** during processing – e.g., write 0 to INTR\_ENABLE or otherwise prevent re-entry – so that we don’t get another interrupt until we finish dealing with this one.
4. **Determine Packet Length:** The host should find out how many bytes to read. The typical method:

   * Read `SPI_REG_RDBUF_BYTE_AVA` (0x0300) to get the count of bytes available. Let’s call this count N. If N is 0, it means maybe by the time we handled cause, data was gone (shouldn’t happen unless a race condition or the QCA reset). If N > 0, proceed.
   * **Optional:** Another way is to do a small SPI read of just 4 bytes to fetch the LEN field first, then decide how many more to read. However, since RDBUF\_BYTE\_AVA should give the total, and presumably that corresponds exactly to LEN+4 or similar, most implementations just use RDBUF\_BYTE\_AVA directly. We’ll assume RDBUF\_BYTE\_AVA = total bytes available for immediate read (which should equal the entire first frame’s length including its overhead).
5. **Allocate Buffer:** The driver should allocate a buffer to store the incoming data. In RTOS, this might be a pbuf or a static buffer of max frame size. Let’s say N is, for example, 100 bytes (very small example). The driver ensures the buffer is at least N bytes.
6. **Read BFR\_SIZE and SPI Read:** The driver writes N to `SPI_REG_BFR_SIZE` to tell QCA the size of the upcoming read. Then it performs an SPI transaction to read N bytes: it sends the 16-bit read-external command (0x8000) and clocks out N bytes from MISO. This could be done using DMA on the ESP32’s SPI for efficiency, especially for large N (e.g., 300+ bytes).
7. **Process Frame:** Now the driver has N bytes of data in the buffer. It will parse them as per the framing. Usually:

   * Verify the framing markers (SOF, EOF) and integrity as described.
   * Extract the Ethernet frame of length FL from the data (skipping the QCA overhead).
   * Hand off the Ethernet frame to the network stack. In an RTOS, you might call `ethernet_input(pbuf, netif)` or a similar API. In bare-metal, you might directly call your EtherType handlers or place it in a queue for main loop processing.
   * Free or reuse the buffer for future receives after the data is consumed by the stack.
8. **Handle Multiple Frames:** After reading one frame, it’s possible more data was in the QCA’s buffer (e.g., if two PLC packets arrived in quick succession). How to handle:

   * If using a loop: After the above read, read `RDBUF_BYTE_AVA` again. If it’s non-zero, that means another packet is waiting. You can then repeat: set BFR\_SIZE for that and do another external read. All within the same interrupt context if desired (or you could exit ISR and rely on another interrupt, but if the interrupt was edge-triggered and you already cleared the cause, you might not get another edge unless QCA toggled line again).
   * Many drivers indeed loop in the ISR or handler until `RDBUF_BYTE_AVA` reads zero. This ensures all queued frames are drained before exiting the interrupt. This is important to avoid losing frames: if you only serviced one and returned, the interrupt line might have been deasserted when you cleared cause even though a second frame was there, potentially leading to no further edge trigger. (Unless QCA reasserts it, which might require cause to clear and space to be free.)
   * Therefore, implement: `do { … read frame … } while(RDBUF_BYTE_AVA > 0);`
   * Note: The QCA7000’s internal buffer (3163 bytes) can hold roughly two maximum-size frames or several small frames. So it’s possible to get multiple frames in one go, especially if the host was busy for a moment. The driver should be capable of reading them back-to-back in a loop.
9. **Acknowledge Interrupt:** After all available packets are read, the driver writes the value it read from INTR\_CAUSE back to INTR\_CAUSE register to clear those bits. Specifically, it will write PKT\_AVLBL=1 (and any others that were set) to clear them. Then it re-enables interrupts by writing the appropriate bits to INTR\_ENABLE (if they were masked). The QCA7000’s IRQ line will go low once the cause bits are cleared.
10. **Packet to Application:** Finally, the network stack processes the received frame (e.g., if it’s an IP packet, it may get passed to a socket, etc.). From the application’s perspective, it just receives normal network data – it doesn’t need to know it came via PLC.

**Queuing Logic and Flow Control:**

* **Transmit Queuing:** As mentioned, if multiple frames are to be sent in a short time, the driver might queue them to wait for the QCA’s buffer. Because QCA7000 can buffer around 3K, it can hold at most \~2 full frames. If the host tries to send a third frame quickly, `WRBUF_SPC_AVA` will be low or zero. The driver could drop the packet or queue it. Dropping might be fine for something like UDP logs, but for reliable protocols or if you want to ensure delivery, a queue with a few slots is better. A simple ring buffer or linked list can hold pending packets. The driver should attempt to transmit the next packet as soon as space is reported (via an interrupt or by polling in a transmit loop). If using the `WRBUF_BELOW_WM` interrupt (bit10), that would be the cue that “enough space has cleared, you may send more.” If not using it, you could piggyback on other events or set up a timer to retry.
* **Receive Queuing:** On the receive side, typically each packet is immediately handed to the network stack. If the network stack is slower or busy, you might need to queue incoming packets (store them until the stack can handle them). In FreeRTOS, lwIP usually handles this by its internal mailbox (the `tcpip_thread` will process each packet in turn). If the driver is calling `ip_input` or similar directly from the ISR, that’s not ideal (should instead send to mailbox). So design: the ISR or RX task takes the data and either directly calls the netif input function (if allowed from that context) or pushes it to a safe context. In ESP-IDF, `esp_netif_receive()` is thread-safe and will queue the buffer to the TCP/IP task.
* **Flow Control / Backpressure:** If the host is overwhelmed with incoming packets (unlikely at 10 Mbps line rate and given ESP32’s capabilities, but suppose the CPU is busy), the QCA7000 can buffer some data but eventually its buffer will fill (3163 bytes \~ two max frames). If that happens, presumably new PLC frames might be dropped by QCA or QCA might assert an overflow error (RDBUF\_ERR). To avoid that, the host should process interrupts promptly. In extreme cases, one might implement flow control at the application layer (like have the EV and EVSE throttle message frequency if responses lag), but generally it’s not needed because 10 Mbps is not very high and the ESP32 can typically handle it as long as the driver is efficient. Still, our driver can mitigate overflow by quickly reading multiple frames per interrupt as described.
* **Burst Mode vs Legacy Mode (Data Implications):** With burst mode (CS held low), an entire frame is transferred in one contiguous burst on SPI. This maximizes bus utilization and minimizes per-frame overhead. In legacy mode (CS toggling per word), after each 16 bits the bus is idle (CS high) for a brief time until the master reasserts CS for the next word. This gap can significantly reduce throughput. For example, at 8 MHz SPI, one 16-bit word takes 2 µs to clock out, but if there’s even a 2 µs gap to reassert CS and start next word, you effectively doubled the time. Many microcontrollers also insert some delay automatically when toggling CS frequently. Therefore, in legacy mode, effective throughput might drop well below 50% of burst mode. Moreover, the increased interrupt or DMA overhead (since each word might need separate handling) is burdensome. The QCA7000 was designed to use burst mode for performance (the “legacy” mode is only there for compatibility with hosts that can’t do continuous clocking or had errata). So, in practice for EV communications (which might involve transferring SSL/TLS handshake messages, etc. in ISO 15118), burst mode is essential to meet timing. We reiterate: ensure burst mode by proper configuration (if QCA is strapped to legacy, see if you can override via SPI\_CONFIG’s multi\_cs\_enable bit – setting it to 0 might switch to burst if the QCA firmware supports that dynamically).

**Full Data Flow Summary:**

* **ESP32 Application** generates network data -> passes to **TCP/IP stack** (lwIP) -> handed to **QCA7000 driver (TX)** -> formatted and sent via **SPI** -> stored in **QCA7000 write buffer** -> transmitted over **Powerline (HomePlug GreenPHY)** to remote device.
* **Remote Device** sends PLC data -> received by **QCA7000 PHY** -> processed into an Ethernet frame -> placed in **QCA7000 read buffer** -> QCA triggers **interrupt** -> ESP32 **QCA7000 driver (RX)** reads data via **SPI** -> driver strips framing -> passes Ethernet frame to **TCP/IP stack** -> delivered to **ESP32 Application**.

Throughout this flow, synchronization between ESP32 and QCA7000 is maintained by the protocol (BFR\_SIZE, interrupts, etc.) and by careful driver design to avoid collisions on the SPI bus. Next, we detail how the interrupt from QCA7000 is managed and how to implement the handler logic on the ESP32 side.

## Interrupt Management

Managing the QCA7000’s interrupt (IRQ) line is critical for timely and efficient data transfers. The interrupt notifies the ESP32 of events like incoming data or error conditions, reducing the need for constant polling. Here’s how to handle QCA7000 interrupts and what the various causes mean:

**Interrupt Setup:** After initializing the QCA7000 and its SPI interface, configure the ESP32’s GPIO connected to QCA7K IRQ as an input with rising-edge interrupt trigger. In ESP-IDF, this can be done using the GPIO ISR service (`gpio_isr_handler_add(pin, handler, arg)`) with `GPIO_INTR_POSEDGE`. Ensure the input has the correct pull-ups/downs as needed (the QCA7000 likely drives it actively high and low, so usually no pull is needed, but consult the module datasheet). The ISR handler should be a short routine, typically just recording the event (e.g., setting a flag or giving a semaphore) and *not* doing heavy SPI operations directly if it can be deferred. If a bare-metal approach is used, you can do more in the ISR, but be mindful of SPI timing and length (reading a large frame inside an ISR can block other critical interrupts if running at high priority, etc.).

**Basic ISR Flow:**

1. **Mask interrupts (optional):** On entry, if you expect multiple events or want to prevent reentry, you might disable the QCA7000 IRQ or mask it in the QCA itself. A common pattern:

   * Write 0x0000 to `SPI_REG_INTR_ENABLE` to temporarily mask all QCA interrupts. This prevents the QCA from retriggering the line while we’re processing. Alternatively, on ESP32, you could disable the GPIO interrupt, but manipulating it at hardware level isn’t usually necessary if you handle masking in QCA.
   * Note: If the QCA IRQ is level-triggered (which it effectively is, because as long as a cause is un-cleared, the line stays high), you must mask or clear cause to drop the line. Since we do plan to clear cause later, masking at start is just to avoid multiple triggers in the meantime.

2. **Read interrupt cause:** Perform an SPI internal read of `SPI_REG_INTR_CAUSE` (0x0C00). This yields a 16-bit value where bits indicate which events have occurred. Save this in a local variable (e.g., `cause = ...`).

3. **Service each cause:** Check which bits are set in `cause` and handle them:

   * **CPU\_ON (bit6):** If set, the QCA7000 has just booted or reset. This means all previous state was lost and we need to re-initialize. The handler for this would typically schedule or perform the **initial setup sequence** (see next section). In an ISR, you might not want to do the full sequence immediately; maybe set a flag like `qca_reset = true` to be handled after exiting ISR. However, minimal steps like reading signature could also be done here if necessary. Usually, you would want to reset any driver state (flush host-side queues, etc.) and then re-configure QCA as done at startup. The QCA7000 itself raising CPU\_ON suggests either power-on or a soft reset (maybe from previous error handling). Acknowledge it but treat it as an event that requires outside-of-ISR work.
   * **WRBUF\_ERR (bit2):** QCA reports a write buffer error. This likely means the host attempted something invalid (like sending too much). It’s a serious condition: the documentation explicitly says the recommended reaction is to restart the QCA7000 via the QCASPI slave reset bit. So the handler should note this and initiate a reset. In Linux driver, they log an error and set up to reset the device. In an RTOS, you might not reset directly in ISR (because resetting QCA might involve toggling a GPIO or lengthy waits); instead, set a flag or send a message to a management task to perform the reset. If bare-metal, you can perhaps directly toggle the reset line or set the reset bit via SPI (if SPI is still functional) here.
   * **RDBUF\_ERR (bit1):** Similarly, a read buffer error (indicating data loss or sync issue on RX). The recommended action is the same – reset the QCA7000. Both these errors suggest something went badly out-of-sync, so restarting is the safe recovery.
   * **PKT\_AVLBL (bit0):** Indicates one or more packets are available to read. The handler should initiate reading them as described in the Data Reception flow. This can be done directly in the ISR if the system is simple and can tolerate the latency of reading possibly kilobytes of data in interrupt context (not usually ideal). In a more sophisticated design, you would instead signal a dedicated task to do the reading:

     * For example, set `packet_pending = true` and give a semaphore to an RX task. The RX task (running at perhaps lower priority than critical ISRs) will loop reading all available packets. This offloads the heavy SPI I/O from the ISR. The risk of deferring is minimal as long as the task runs soon, but if the system is heavily loaded and there's a delay, QCA’s buffer might overflow if another packet comes in. However, since our interrupt woke the task immediately, it should handle it promptly.
     * If bare-metal or if you choose to handle in ISR for speed, make sure to not stay in ISR too long or else FreeRTOS might get upset if other ISRs need attention. A hybrid approach is possible: read small amounts or critical registers in ISR, but defer large data copies.
   * **WRBUF\_BELOW\_WM (bit10) \[if used]:** If this is set (and enabled), it means QCA’s write buffer has space again (went below watermark, likely meaning roughly half-empty or fully empty). The host can resume sending if it had paused. The handler could check if any queued outgoing frames exist and trigger sending them. In a design with a separate TX task, you might simply un-pause that task or send it a notification. If handling in ISR, you could try sending one packet right away (though doing a full SPI write in ISR might be heavy). It might be better to just signal the TX routine to run. If using FreeRTOS, maybe use a higher-level event group or a direct-to-task notification for the TX task. If bare-metal, perhaps set a flag like `can_send_more = true` which the main loop checks.
   * **Other bits:** If any reserved bits (or bit3-5) are unexpectedly set, you might log a warning and ignore them. It’s possible none will appear unless new features are added by updated QCA firmware.

4. **Clear interrupts:** After handling all relevant causes, the driver must clear them so that the QCA7000 will deassert the IRQ line. To do this, write the same value back to `SPI_REG_INTR_CAUSE` (i.e., writing a ‘1’ to each bit that was set). For example, if cause was 0x41 (bit6 and bit0 set: CPU\_ON and PKT\_AVLBL), write 0x0041 to INTR\_CAUSE. This resets those flags inside QCA. *Important:* Only after this write will the QCA drop the IRQ line (assuming no other pending events remain). If you leave any cause un-cleared, the line will stay high (potentially causing an interrupt storm or blocking future edges).

   * In practice, clear *after* servicing to ensure you don’t miss another event that might occur while processing. If another packet arrives while you are reading the first, the QCA might set PKT\_AVLBL again. But since you masked interrupts at start, the second event won’t trigger a new interrupt line toggle. However, the cause bit might already be set in the register (ORed in). By reading cause at the start, you got the initial state. If a second packet arrived after that, the QCA could either (a) keep the IRQ high (which it is) and just set PKT\_AVLBL bit still (it was already set, so no change), or (b) if it was cleared and re-set, we might not know. This is tricky. The solution is to loop reading packets until none remain, which we do.
   * Thus, when we finally clear, we know we’ve drained all data and handled all events that were present up to that point.

5. **Re-enable interrupts:** Finally, re-enable the desired interrupts by writing the appropriate bitmask to `SPI_REG_INTR_ENABLE`. Typically, it’s the same mask we set during initialization: e.g., enable CPU\_ON, PKT\_AVLBL, RDBUF\_ERR, WRBUF\_ERR (and optionally WRBUF\_WM) bits. If you had masked at the start of ISR, you definitely need to re-enable them now. If you didn’t mask at start, but simply relied on edge triggering, you still need to ensure the QCA is configured to continue sending interrupts. Usually, intr\_enable wouldn’t be turned off spontaneously, but some strategies might disable it on entry. The Linux driver shows an approach: it writes 0 to INTR\_ENABLE, reads/clears cause, then writes back the enable mask. We can mirror that.

   * One nuance: Some drivers might leave interrupts disabled until after the cause is cleared to avoid the scenario of QCA raising an interrupt while we’re still in handler. But since we clear cause right before enabling, that’s fine.

6. **Post-processing outside ISR:** If the ISR set flags (like `qca_reset` for CPU\_ON or queued multiple packets to process), ensure those are handled. For example:

   * If `qca_reset` was set due to CPU\_ON or an error, then after ISR (in main loop or a separate handler task) call the initialization routine to sync with QCA (read signature, etc., re-enable interrupts).
   * If packets were read in the ISR and directly passed to stack, nothing else needed. If instead ISR gave a semaphore and an RX task does the reading, then the ISR is actually minimal and steps 2-5 (reading cause, reading packets, clearing) would be done in that RX task rather than in the ISR context. In that design, the “ISR” is just a GPIO handler that gives control to the RX thread, and the RX thread would do all the SPI interactions but still following the logical steps above (read cause, etc.). This might simplify things because FreeRTOS doesn’t like you doing SPI (which may use DMA and etc.) in a high-priority ISR. Many implementations prefer a *bottom-half* approach (deferred interrupt handling) for such drivers.
   * If WRBUF\_WM indicated space, and if you had any frames queued to send, now would be a good time to try sending one (perhaps directly from ISR if trivial, or schedule the sending in the TX task). Possibly your TX function was already periodically checking `WRBUF_SPC_AVA` on attempts, but an interrupt can prompt an immediate retry.

**Avoiding Interrupt Storms:** An “interrupt storm” could occur if the IRQ line is constantly asserted faster than the host can handle. To prevent this:

* Always clear the cause bits after handling. If you forget to clear, the line will stay high, and if configured as edge-triggered, you might not get a new edge at all (leading to a stuck situation), or if level-triggered, you’d continuously re-enter (in ESP32 edge is default so likely the first case).
* Do not re-enable interrupts on QCA until you have cleared existing ones. That’s why the recommended sequence masks -> read cause -> clear cause -> re-enable ensures we don’t double-handle an event.
* If you find that PKT\_AVLBL is being raised faster than you can read (e.g., maybe the EV is sending a flurry of small packets), consider reading multiple packets per interrupt as described. This way, by the time you clear, you’ve handled a batch. The QCA might aggregate multiple events into one IRQ assertion which is efficient.
* If using WRBUF\_BELOW\_WM, be cautious: If your strategy is to send one packet and then wait solely for WM interrupt to send next, there’s a scenario: what if the QCA buffer never empties below watermark because it’s almost full but not quite (like threshold is 50% and you always keep it \~60% by continuous sending)? Possibly no interrupt would come. Better to either set watermark appropriately or use a combination of polling and interrupts. The Linux driver doesn’t even enable this bit (in the snippet we saw, they only enabled up to bit2), likely because it manages output via net stack queue without needing a special interrupt.
* For simplicity, you might initially ignore the watermark interrupt. Instead, whenever you attempt a send and find insufficient space, set a flag and periodically (or upon next PKT\_AVLBL or some event) check again. Many times, receiving data often correlates with buffer being freed (e.g., after sending some request, you get a response etc.), though not guaranteed. Or use a timer tick to retry sends.

**Interrupt Handler Pseudocode (combining above):**

```c
// Pseudocode assuming we handle in a deferred manner (e.g., called in a task context after ISR signals).
void qca_handle_interrupt() {
    // Mask QCA interrupts (prevent new edges) - optional
    qcaspi_write_register(SPI_REG_INTR_ENABLE, 0x0000);
    // Read cause
    uint16_t cause;
    qcaspi_read_register(SPI_REG_INTR_CAUSE, &cause);
    // Debug: printf("QCA IRQ cause = 0x%04X\n", cause);
    bool need_reinit = false;
    if (cause & SPI_INT_CPU_ON) {
        // QCA rebooted
        need_reinit = true;
    }
    if (cause & SPI_INT_WRBUF_ERR) {
        // Write buffer overflow
        log_error("QCA write buffer error!");
        need_reinit = true;
    }
    if (cause & SPI_INT_RDBUF_ERR) {
        log_error("QCA read buffer error!");
        need_reinit = true;
    }
    if (cause & SPI_INT_PKT_AVLBL) {
        // Process all available packets
        do {
            uint16_t bytes;
            qcaspi_read_register(SPI_REG_RDBUF_BYTE_AVA, &bytes);
            if (bytes == 0) break; // no more data (shouldn't happen if cause was set)
            // Allocate buffer of 'bytes' length
            uint8_t *buf = malloc(bytes);
            if (!buf) {
                log_error("Memory alloc fail for RX");
                // We might choose to flush the data by reading and discarding
            }
            else {
                // Set BFR_SIZE = bytes
                qcaspi_write_register(SPI_REG_BFR_SIZE, bytes);
                // Do external read of 'bytes' into buf
                qcaspi_read_buffer(buf, bytes);
                // Validate and handle frame(s) in buf
                if (!validate_and_pass_up(buf, bytes)) {
                    // if validation fails, maybe set need_reinit true
                }
            }
        } while(1);
    }
    // Clear the interrupts we handled
    qcaspi_write_register(SPI_REG_INTR_CAUSE, cause);
    // Re-enable interrupts (except maybe mask errors if we want to ignore them, but better to enable all and handle)
    uint16_t enable_mask = SPI_INT_CPU_ON | SPI_INT_WRBUF_ERR | SPI_INT_RDBUF_ERR | SPI_INT_PKT_AVLBL;
    qcaspi_write_register(SPI_REG_INTR_ENABLE, enable_mask);
    // If needed, perform re-init (outside this function ideally, as it might involve resets and delays)
    if (need_reinit) {
        trigger_reinit_sequence = true;
    }
}
```

In reality, `qcaspi_read_register` and `write_register` and `read_buffer` would themselves be SPI transactions. Those should ideally be protected by a mutex if this function can be called concurrently with others (but since this is the sole handler, it’s fine). If this is being done in an ISR directly, replace `malloc` with static buffer or preallocated buffer usage (to avoid heap in ISR), or better, don’t do it fully in ISR.

The production driver repeats the interrupt handling sequence while the IRQ line
remains asserted so that no pending events are missed. When a write or read
buffer error is reported, it asserts `QCASPI_SLAVE_RESET_BIT` and waits up to two
seconds for the `CPU_ON` cause to appear before running the initialization
sequence again. If the modem never signals `CPU_ON`, the timeout is logged but
execution continues.

**Gpio ISR (if deferring):** The actual GPIO ISR could be:

```c
void IRAM_ATTR gpio_qca_irq_isr(void* arg) {
    BaseType_t xHigherPriorityWoken = pdFALSE;
    // Notify the handler task
    vTaskNotifyGiveFromISR(qca_task_handle, &xHigherPriorityWoken);
    if(xHigherPriorityWoken) {
        portYIELD_FROM_ISR();
    }
}
```

Then `qca_handle_interrupt()` would be running in `qca_task_handle` context when unblocked by the notification.

**Interrupt Storm / Misaligned Buffers Warning:** One common mistake is not properly clearing interrupts, leading to an “interrupt storm” where the CPU keeps servicing the ISR thinking a new event happened. In our design, because we disabled and cleared properly, we avoid that. Another mistake is enabling the interrupt pin in the MCU incorrectly (e.g. as level instead of edge in an environment that expects manual ack). On ESP32, if you were to use level-triggered, you must clear the cause *before* exiting ISR, else it will immediately retrigger. We followed that by clearing cause. So we’d be fine either way, but edge is simpler.

Next, we will cover how to initialize and synchronize with the QCA7000 (including using these interrupt bits properly from the start and handling the CPU\_ON event).

## Initialization and Synchronization

Proper initialization ensures the QCA7000 and ESP32 start in lockstep and maintain synchronization. The steps below outline the power-up sequence, configuration, and synchronization measures for both bare-metal and RTOS setups, highlighting any differences.

**Power-Up/Reset Sequence:**

1. **Hardware Reset:** On ESP32 startup (or whenever enabling the QCA7000 interface), assert a reset to the QCA7000 if possible. For example, drive the QCA7000’s RESETN pin low for a few milliseconds then high. This ensures the QCA starts from a known state. If the QCA7000 is on a fresh power cycle, this might not be needed as it’s already in reset until power stabilized. But doing it explicitly under host control is a good practice, especially if the ESP32 might reboot independently of QCA (to avoid QCA being in an old state while ESP32 is starting fresh). After deasserting reset, allow time for the QCA7000 to boot its firmware. The datasheet or app notes may suggest a specific time; the Linux driver uses up to 1000 ms (1 second) as a safe maximum for QCA7K to fully reboot. In practice, it might be ready in tens of milliseconds, but a conservative delay (e.g., 100 ms) is fine if not known.
2. **SPI Configuration:** Initialize the ESP32’s SPI peripheral (mode 3, correct CS, frequency as decided) and test basic SPI communication. One simple test is trying to read the SIGNATURE register. However, note that immediately after reset, the QCA7000 may not respond to SPI for a short time until its internal CPU is up and running (especially if booting from flash). The first attempt might fail or return junk. The documented sequence suggests reading signature twice: the first read might be ignored by QCA if it’s not ready. So be prepared to try multiple times.
3. **Signature Verification:** Read `SPI_REG_SIGNATURE (0x1A00)`:

   * Perform an internal read of 0x1A00. **Ignore the first response**. This could be done by reading and not checking the value, or simply issuing a dummy read (some drivers do a dummy write to a benign register first as well).
   * Read `SPI_REG_SIGNATURE` again. This time, check that the 16-bit value is 0xAA55. If it matches, SPI communication is established and endianness is correct. If it does not match, there’s a problem: possibly the SPI mode is wrong, the bytes are swapped, or the QCA7000 is still in reset/booting. If not match, one strategy is to keep trying a few times (with a small delay between tries). If after, say, 5 attempts over 100ms you still get a wrong value, treat it as an error (perhaps log “QCA7000 not responding” and reset again or halt init).
   * A common wrong value is 0x55AA, which hints the byte swap issue. If you see that consistently, you can infer that you need to swap bytes in your SPI read routine.
   * This signature check is crucial for “synchronization logic” – it confirms the host and QCA7000 agree on the framing of SPI bits. The app note even suggests that if you get an incorrect signature, you might improve hardware signal quality (terminators). In software, repeated failure triggers a reset.
4. **Initial Register Configuration:** After confirming QCA presence:

   * Optionally, read `SPI_REG_SPI_CONFIG (0x0400)` to inspect or adjust settings. Specifically, check the **multi\_cs (legacy mode) bit**. If this bit (likely bit1 of some register, possibly ACT\_CTRL or part of config) is set (meaning legacy mode on), and if your design allows, you might clear it to enable burst mode. However, caution: if QCA7000 was strapped to legacy, it might require that mode and changing it on the fly might not be supported or could be transient. The documentation snippet in the QCA datasheet suggests “multi\_cs\_enable” bit with default 1 (which implies legacy by default). If we interpret that, to use burst mode we’d write that bit to 0. If you decide to do this:

     * Do a read-modify-write on SPI\_CONFIG or the relevant register containing multi\_cs. For example, if multi\_cs\_enable is bit1 at address 0x1A00 or 0x1B00 (the misuxin excerpt shows it under SIGNATURE or ACT\_CTRL registers?), actually it appears under SIGNATURE region:
       They show in \[19]: *Field Name Bits Access Reset Value Description* and line 19 shows "*multi\_cs\_enable 1 RW 0x1 only used when the SPI Slave interface is operating ...*". That looks like bit1 of some register (maybe ACT\_CTRL at 0x1B00?). It’s not entirely clear if this is separate or part of signature register’s high bits (15:2 reserved, 1 multi\_cs\_enable). Possibly the SIGNATURE register double-purposes bit1? That would be odd since signature is R only. Maybe ACT\_CTRL is 0x1B00 and bit1 is multi\_cs\_enable. If so, one would write to 0x1B00 to disable legacy. We might not have enough info to implement that here; instead, one might rely on the strap or assume either it’s already in burst (likely in many EV modules it is).
     * If leaving as is, ensure your SPI handling matches. If stuck in legacy inadvertently, the symptom would be that after sending the 16-bit command, QCA expects CS to toggle before data. If you kept it low, QCA might not register the command properly. In practice, if signature read only works when you toggle CS per byte, that’s a clue you’re in legacy mode. That would complicate code. In any case, if signature returned correctly with one continuous transaction for command+data, you are likely fine (burst mode).
   * Write `SPI_REG_INTR_ENABLE (0x0D00)` to 0x0000 to start with. Actually, at power-up, interrupts might be disabled by default, but the safe step is to ensure it’s masked until you finish setup.
   * Clear any stale interrupt causes: read `SPI_REG_INTR_CAUSE (0x0C00)` just in case something is set (the CPU\_ON might have triggered an interrupt already). If you see non-zero, write that value back to clear it. Alternatively, some drivers do this: write 0xFFFF to INTR\_CAUSE to clear all bits (though writing reserved bits is not explicitly documented, it likely just clears what’s defined).
5. **Configure Interrupt Handling:** Now set up how you want to handle interrupts:

   * If using an ISR (which we are), register the GPIO interrupt handler now (if not done earlier). It can be active now because we haven’t enabled QCA’s interrupts yet, so no spurious triggers should occur.
   * Prepare any tasks or semaphores for deferred handling if using RTOS.
6. **Enable QCA Interrupts:** Write to `SPI_REG_INTR_ENABLE` the bitmask of interrupts you want. A typical enable mask (as discussed) is `0x0047` (binary: 0100 0111) which corresponds to CPU\_ON (bit6), WRBUF\_ERR (bit2), RDBUF\_ERR (bit1), PKT\_AVLBL (bit0). If including bit10 for watermark, that would be `0x0447` (bit10 plus those).

   * **Note:** Some designs might exclude CPU\_ON from enable, expecting that event only at startup which they handle by polling. But it's safer to include it in case QCA resets later due to internal errors.
   * After writing intr\_enable, the QCA7000 is armed to trigger its IRQ line for those events.
7. **Initial Buffer Check (Optional):** It may be prudent to read `WRBUF_SPC_AVA` (should be 3163) and `RDBUF_BYTE_AVA` (should be 0) to confirm buffers are empty and available. If `RDBUF_BYTE_AVA` is not zero at this point, it means somehow data is present (perhaps leftover from a previous run if QCA didn’t reset, or noise?). If that happens, one strategy is to flush by reading whatever bytes it indicates (could be garbage) and discarding, or simply resetting QCA again. Typically, on a clean start, RDBUF\_BYTE\_AVA = 0 and WRBUF\_SPC\_AVA = 3163.
8. **Synchronization Considerations:** At this point, the host and QCA7000 should be in sync:

   * QCA’s SPI interface is properly configured (we verified via signature).
   * Interrupts are enabled and will inform us of new events.
   * We have cleared any prior state.
   * We should also ensure our local driver variables are set: e.g., clear any TX queues, reset sequence numbers (if any), zero out stats, etc.
   * If the QCA7000 requires a firmware download (only if it wasn’t flashed, which is beyond our scope), you would do that now. Typically, that would involve writing to some register to put it in download mode and streaming a firmware file via SPI. This is only if QCA doesn’t boot on its own. Most likely for EV usage, QCA7000 has its firmware ready.
   * If QCA7000 has any network-specific config (like setting PLC coupling profiles or encryption keys), those would be done via network packets or vendor-specific mechanisms, not via these SPI registers (except maybe some in-band management frames). The driver doesn’t handle those in this layer – those are higher-level concerns.

**Bare-Metal vs RTOS Initialization Differences:** The sequence is largely the same. The main difference is event handling post-init:

* In bare-metal, after enabling interrupts, you might just rely on the global interrupt to call an ISR which does a lot of the work. There is no separate task to schedule, so the ISR may both read causes and handle data or set flags for the main loop.
* In RTOS, after enabling interrupts, you likely have an idle driver task waiting for notifications from the ISR. You also need to create that task before enabling IRQs to avoid lost signals. So, ensure your “qca\_driver\_task” is created and ready by the time you do `intr_enable`.
* In an RTOS, you also register the `esp_netif` or netif in lwIP before or right after initialization so that when packets come, you have a place to deliver them. It might even make sense to bring up the network interface (assign MAC, etc.) at this point or slightly later. Usually, one sets the QCA7000’s MAC address either by reading it from somewhere (maybe the QCA’s OTP or module sticker) or generating one. The QCA7000 might not have an inherent MAC address (some PLC modules have one). If needed, you can program the MAC into your network interface (esp\_netif\_set\_mac()) now.

**Handling CPU\_ON (Synchronization at runtime):** If the QCA7000 resets **after** init (e.g., due to internal error or host issuing reset):

* The QCA will assert CPU\_ON interrupt. When that is handled, we should essentially rerun the crucial parts of initialization:

  * Read signature twice (if desired) to resync.
  * Re-enable interrupts (they might default off after QCA reboot).
  * Possibly reconfigure any bits (like multi\_cs if needed, though likely strap remains same).
  * It's like a mini re-init routine. The app note’s initial setup after each reset is exactly that. They list reading signature twice, ensuring value, presumably then enabling interrupts, etc.
  * If our driver maintained any state (like partially transmitted frames or pending receives), those should be cleared because QCA lost them. The host might want to inform upper layers – e.g., in a network context, losing the link might be handled by reporting link down/up events. Possibly treat a QCA reset as a link reset (maybe drop TCP connections or at least expect reconfiguration). That’s more system-level, but something to consider. At minimum, log it for debugging.

**Polling Flow (Alternative to IRQ):** In some bare-metal cases, one might not use the IRQ and instead poll periodically. This is not recommended for performance, but if chosen:

* You would skip configuring the IRQ pin and not enable QCA interrupts. Instead, your main loop would regularly read `RDBUF_BYTE_AVA`. If >0, read the frame. And regularly check if QCA’s CPU reset (maybe by reading signature periodically or checking some status).
* Polling for transmit space similarly by reading `WRBUF_SPC_AVA`.
* Implement timeouts: e.g., if waiting for space for too long, maybe something is wrong.
* Polling frequency: Ideally should be faster than packet interarrival in worst case. For example, if expecting at most 100 packets per second, polling at 1kHz is fine. But if a burst can come (which at 10 Mbps, you could theoretically get \~800 packets per second of minimum size), you’d need to poll much faster to catch them before overflow. Therefore, interrupt-driven is far superior for high throughput.
* Polling might be acceptable if the communications are infrequent (like a charge negotiation message every few seconds). But if any firmware updates or high data flows occur, it would be a bottleneck.

**Post-Initialization Test:** Once initialization is done, it’s prudent to test the link:

* Possibly have the ESP32 send a dummy Ethernet frame (if network stack is up, maybe an ARP request or ping).
* Or simply verify that no errors triggered immediately.
* If you have a second QCA7000 node or some PLC emulator, try sending a frame from the other side to ensure our driver picks it up.
* The QCA7000 might also support a loopback or diagnostic mode, but easier is to test with actual networking if possible.

With initialization covered, we move on to error handling and edge cases to ensure our integration is robust in all scenarios.

## Error Handling and Edge Cases

Robust operation requires anticipating and handling various error conditions and unusual scenarios. Below we discuss known error causes, debugging strategies, and recommendations for making the system reliable.

**Buffer Overflow / Underflow Errors:**

* **Write Buffer Overflow (WRBUF\_ERR):** This error (interrupt cause bit2) indicates the host attempted to write more data than the QCA7000 could buffer. This typically arises from a logic mistake in the driver: either not checking `WRBUF_SPC_AVA` properly, or setting BFR\_SIZE too high, or performing multiple writes without waiting for space. It could also happen if the QCA7000’s buffer was presumed empty but wasn’t (perhaps due to an earlier partial transfer or because QCA hadn’t transmitted previous data yet). When WRBUF\_ERR occurs, the QCA likely discards the current transfer and its internal state might be inconsistent. The recommended approach is to **reset the QCA7000** (via SPI\_CONFIG’s reset bit or hardware reset). Upon reset, re-run initialization. To avoid WRBUF overflow:

  * Always obey `WRBUF_SPC_AVA`. If a frame is larger than available space, do not attempt to send it. Instead, wait or split the frame if splitting were supported (but splitting one Ethernet frame across two PLC frames is not typical or straightforward).
  * Ensure only one context is writing at a time (no two tasks both doing SPI writes).
  * If using DMA or long transfers, ensure the length exactly matches BFR\_SIZE; sending even 1 extra byte could trigger an overflow.
  * Handle the case where QCA’s buffer size is smaller than an Ethernet frame: The standard QCA buffer (3163 bytes) comfortably fits a full frame, so this shouldn’t happen. If it were smaller (in some hypothetical scenario), the driver would need to fragment the Ethernet frame across multiple PLC packets (which would break the semantics at the Ethernet layer, so likely not supported).
* **Read Buffer Overflow (RDBUF\_ERR):** This error (bit1) usually means the QCA7000’s internal read buffer overflowed because the host didn’t read data out quickly enough. In other words, PLC frames came in and filled the buffer when the host was slow or not responding. It could also mean a host under-read – e.g., host set BFR\_SIZE less than available and cleared the interrupt (though typically that yields misalignment rather than QCA raising RDBUF\_ERR, but QCA might detect if data still in buffer when PKT\_AVLBL cause was cleared incorrectly). The remedy is again to **reset the QCA7000**. Avoiding RDBUF overflow:

  * Service receive interrupts promptly. If the microcontroller was in a disabled interrupt state or very busy, multiple frames could stack up. Try to ensure the QCA IRQ has a high priority and that the driver empties the buffer fully in one go (loop until empty).
  * If expecting bursts, perhaps enlarge any host queue or processing speed (e.g., dedicate CPU to handle incoming data).
  * As a failsafe, the driver could monitor how full the buffer is by reading `RDBUF_BYTE_AVA` on each interrupt and maybe issue warnings if it’s near the max (though with only 3163 bytes, if you get a full buffer it's basically just two max frames).
  * The QCA7000 might have some internal flow control on PLC side (HPGP has protocol to tell peers to slow if you can’t handle). The specifics of that are not exposed to the host, but likely the PLC MAC would NACK or drop frames if the buffer is full. So an overflow might correspond to lost network frames.
* **Buffer Underflow (read/write mismatch):** If the host reads fewer bytes than available or writes fewer bytes than declared by BFR\_SIZE, it doesn’t map to an explicit interrupt cause but will lead to bad data:

  * If host drops CS early on a read, the remaining bytes of that frame stay in QCA’s buffer. QCA might not automatically throw them away. The next time the host tries to read, it will likely start from where it left off (which will not align to a proper frame boundary, causing framing errors). The QCA might set RDBUF\_ERR if it detects host didn’t finish a frame properly. Regardless, once this happens, the host and QCA are out-of-sync on the framing, so the safe recovery is to reset the QCA or at least flush the buffer completely. Flushing could be done by reading the rest of the bytes (if you know how many remain) or issuing the reset bit which clears buffers.
  * Similarly for write: if host asserted CS and maybe got interrupted and never sent the full frame, the QCA’s buffer might contain a partial frame which it won’t forward (since it never got EOF maybe). QCA might wait until timeout or until host sends the rest. There’s no documented timeout though. In such a case, eventually communications will stall (and possibly WBRUF\_ERR might be triggered if the host tries something else). Again, recovery is a reset.
  * These scenarios underscore the importance of disabling interrupts or otherwise ensuring atomic SPI transfers (e.g., not allowing a context switch in the middle of a critical SPI operation).
* **Invalid Lengths (Protocol Mismatch):** If the host accidentally set BFR\_SIZE incorrectly (like off by a few bytes) or misinterpreted RDBUF\_BYTE\_AVA:

  * If BFR\_SIZE < actual frame data length on a write: you will send a truncated frame (missing some bytes and EOF). The QCA likely will see wrong EOF marker (or none) and might drop that data or treat as a malformed frame. There’s no explicit interrupt, but the effect is lost frame. If BFR\_SIZE > frame length, you might send random memory content as padding up to that size or just send a bunch of zeros if using DMA with longer length. The QCA would then see 0x5555 somewhere in the middle of data, possibly and incorrectly mark end of frame early (though no, QCA doesn’t parse within the write buffer; it assumes host’s provided proper framing). Actually, in that case, the QCA might take whatever you sent as the frame including whatever trailing bytes, and if by chance one of those looked like an EOF, it might cut the frame there and leave extra bytes. All in all, it’s a mess – avoid it by precise length calculation.
  * If host misreads the receive LEN field (e.g., due to endianness) and uses a wrong number to parse, it might think frame is shorter or longer. If shorter and you then clear interrupt, you’d leave some bytes unread (similar to underflow case). If longer (not likely unless you literally treat LEN’s bytes swapped so a small number becomes huge, but RDBUF\_AVA prevents you from reading more anyway).
  * There’s also the possibility of **mismatched endianness**: The QCA7000 is little-endian for multi-byte fields (FL, LEN, etc.), but the ESP32 is little-endian too, so reading into a structure normally should yield correct values, except for the 32-bit SOF which is symmetric (0xAAAAAAAA is the same bytes regardless) and signature (0xAA55 you have to assemble correctly). If someone attempted to treat the data as big-endian wrongly, lengths would appear swapped. For instance, an FL of 0x003C (60) might be read as 0x3C00 (15360) – which would terribly confuse the logic. So always handle these fields as little-endian.
* **Clock and Signal Issues:** If the SPI signals have integrity problems, you might get sporadic errors (signature mismatches, data corruption). The QCA note recommends series resistors if signature read is flaky. If you suspect this (especially at high SPI speeds), try lower speed or check signals with a scope. Also ensure the SPI clock mode is exactly correct; even using Mode 0 vs Mode 3 can sometimes by coincidence read some registers correctly (due to symmetric patterns like AA55) but fail on others.

**Debugging Strategies:**

* **Register Dump:** Implement a function to read and print all relevant QCA7000 registers. For example, `dump_qca7k_regs()` could read SIGNATURE, SPI\_CONFIG, WRBUF\_SPC\_AVA, RDBUF\_BYTE\_AVA, INTR\_CAUSE, INTR\_ENABLE, maybe ACT\_CTRL if known. Having this to call when things go wrong (or periodically) can illuminate what’s happening:

  * For instance, if an interrupt storm happens, reading INTR\_CAUSE might show a cause bit stuck that you forgot to clear.
  * If communication stops, reading WRBUF\_SPC\_AVA and RDBUF\_BYTE\_AVA can tell if QCA thinks it has data pending or if it’s not getting new data (like if PLC side is idle).
  * If signature becomes wrong mid-run, that’s a big red flag (maybe the SPI mode changed or QCA rebooted unexpectedly).
  * You can call dump in an error handler or even from a debugger if you halt the CPU.
* **Logging:** Use UART or other means on ESP32 to log significant events: e.g., log every interrupt cause, log whenever a frame is sent/received (maybe just sizes and some ID), log error conditions. In FreeRTOS, ensure logging in ISR is done carefully (typically by storing messages to print later to avoid long waits in ISR). On bare-metal, you could directly print but keep messages short or at least not in time-critical sections.
* **Use of Logic Analyzer:** As recommended by the app note, a logic analyzer is extremely helpful. Capture CS, CLK, MOSI, MISO, and IRQ lines when troubleshooting:

  * You can decode the SPI traffic and see if the bytes on MISO/MOSI match what you expect (e.g., see the signature bytes, see the frame headers).
  * Check if CS is glitching (should not toggle during a frame).
  * Check timing between IRQ and the SPI transactions. For example, see how quickly after an IRQ the host starts the read – if there’s a long delay, you might need to optimize.
  * If an error like WRBUF\_ERR occurs, by analyzing prior SPI activity you might pinpoint the cause (like host tried to send despite WRBUF\_SPC\_AVA smaller than needed).
* **Software Assertions:** Put sanity checks in code. For example, if ever `RDBUF_BYTE_AVA` > 3163 or weird values, assert because that means your reading method is wrong (it shouldn’t report more than buffer capacity, presumably).
* **Memory consistency:** Ensure that any buffer shared between ISR and tasks is protected (e.g., if an ISR writes into a rx buffer while a task is reading it – but in our design, we copy then handle, so not an issue).
* **Packet filtering for debug:** If possible, reduce network traffic to isolate issues. For example, test with a single known message rather than heavy load when debugging initial driver bring-up.

**Robustness Recommendations:**

* **Timeouts:** Implement timeouts for waiting operations. For instance, if your transmit function decides to wait for space (rather than drop or queue indefinitely), use a timeout to avoid deadlock. Similarly, if you expect an interrupt (like waiting for CPU\_ON or packet) and it doesn’t come, have a fallback. One example: after sending a frame, you might expect an acknowledgment at higher protocol – if nothing comes and you see no IRQ, maybe QCA hung; you could choose to ping the QCA (e.g., read signature or a register periodically to ensure it’s alive).
* **Retries:** If a certain operation fails, you can retry it. For example, if reading a register randomly returns a wrong value (maybe due to a transient line glitch), reading again could succeed. Many drivers implement a small number of retry attempts for critical operations. The code snippet in Linux has a `wr_verify` parameter allowing multiple tries for register writes. For reads, we can do similar if needed.
* **Use QCASPI\_SLAVE\_RESET carefully:** Writing the reset bit (SPI\_CONFIG bit6) resets QCA’s internal CPU. After doing this, you must go through the initialization steps again. Possibly you should also toggle your local notion of link state (could inform user “reconnecting…”). This is a heavy-handed approach but necessary on serious errors. Ensure no SPI operation is ongoing when you assert reset bit. A typical sequence:

  1. Mask QCA interrupts to avoid spurious events during reset.
  2. Write SPI\_CONFIG with QCASPI\_SLAVE\_RESET\_BIT (0x40) set.
  3. Delay a bit (maybe 10 ms).
  4. Optionally, poll for CPU\_ON cause or just wait the known boot time.
  5. Clear reset bit? Actually, I think writing it triggers the reset and QCA will clear it internally. But the app note said “use R-M-W and keep other bits”. Possibly you set bit6=1 and write, QCA reboots, and that bit would read as 0 again after reboot.
  6. Then proceed with reading signature, etc.
* **Watchdog integration:** Consider using a hardware or software watchdog on the ESP32 in case the driver gets stuck (e.g., in an infinite loop due to some unforeseen state). This is more of a generic system design note.
* **Interrupt Storm Handling:** If for any reason you end up in an interrupt flood (maybe a bug where you keep re-enabling without clearing properly), you might observe high CPU and the system not progressing. If you detect repeated interrupts with the same cause quickly, add logic to break out. For example, if you receive 100 interrupts of PKT\_AVLBL in a very short time but each time nothing to read (maybe cause wasn’t cleared properly), then maybe disable QCA interrupts and do a full reinit.
* **Memory Overflow (host side):** When large packets come, ensure your buffers (pbufs) are large enough. Standard Ethernet MTU (1518 bytes frame, 1522 with VLAN) should be accounted for. If you are using fixed-size buffers of 1500 bytes for payload, that’s not enough because of overhead and that excludes 14 bytes of Ethernet header etc. So allocate at least \~1536 bytes for a frame to be safe (common practice). Or chunk it: some network stacks can chain pbufs but easier is allocate a contiguous buffer of max frame size. The QCA7000 frame including overhead is at most \~1530 + 10 = 1540 bytes plus 4 bytes LEN = 1544 bytes (just slightly above typical). If memory is constrained, you could allocate on demand from a pool.
* **Thread Safety:** If in RTOS, ensure no race conditions: e.g., the TX task might be sending a frame at the same time an RX interrupt triggers. If both try to use the SPI bus concurrently, results are undefined. The solution is a mutex around all SPI transfers to QCA (or design such that one context handles all SPI sequentially). Using a single task for both TX and RX serialized is simplest but not lowest latency (though likely fine). Another approach: in the ISR, instead of performing SPI read directly, set a flag and have the TX task check that flag right after finishing what it was doing. Or have a dedicated “SPI driver” thread that all requests funnel through. Simpler for our scale is just use a mutex in functions qcaspi\_write\_register, qcaspi\_read\_register, qcaspi\_transfer, etc., if those can be called from different tasks. The ESP-IDF’s underlying `spi_device_transmit` might be thread-safe by design via its queue, but mixing with an ISR complicates it. For ISR, you might need to use `spi_device_polling_transmit` because in ISR context you cannot wait on a semaphore (which the default might do).
* **Misaligned Buffer Access (ESP32 DMA):** The ESP32 SPI driver, when using DMA, often requires buffers to be aligned to 4 bytes or at least not in flash etc., for optimal performance. If you use `spi_device_queue_trans` etc., IDF handles it. If writing your own, ensure the buffer is in DRAM (not flash const) and aligned. For instance, if you use `heap_caps_malloc(..., MALLOC_CAP_DMA)`, you get a DMA-capable buffer. Or use the `SPI_TRANS_USE_TXDATA` for small transfers (less than 32 bytes can be queued in registers without DMA). These details matter for performance but not necessarily for correctness unless an unaligned buffer triggers some slower path or problems.

**Common Mistakes (What Not to Do recap):**

* Do **not** use the wrong SPI mode – always Mode 3. This cannot be stressed enough; it’s a simple setting but if wrong, nothing works properly.
* Do not neglect to set BFR\_SIZE before every external transfer.
* Do not ignore the values of WRBUF\_SPC\_AVA/RDBUF\_BYTE\_AVA – requesting more or less data than available leads to errors.
* Do not toggle CS during a burst (unless in legacy mode deliberately). Keep it low for the whole frame exchange.
* Do not leave interrupts enabled without servicing them – always handle the cause and clear it. Also, do not forget to re-enable them after handling, or you’ll stop getting notifications.
* Do not call blocking SPI functions in an ISR without caution – better to defer or use polling versions to avoid deadlocks.
* Avoid busy-waiting in interrupt context for long periods. If you must wait (say, for a condition or just delaying), exit the ISR and handle in task. The system should remain responsive.
* Do not assume the QCA7000 will retry or recover on its own if something goes wrong – the host typically must intervene (e.g., by resetting it).
* Avoid using too high an SPI clock until you have a stable baseline; start with lower speeds such as 2MHz and gradually increase (e.g. 8MHz, then 12MHz) while verifying modem signature and stability. Jumping straight to 16MHz might cause subtle issues if wiring is not perfect, leading to intermittent failures.
* Do not forget the padding of Ethernet frames to 60 bytes. If you send a frame shorter than 60 without padding, the QCA might still expect 60 bytes of payload as per FL. This could result in the next 0x5555 being in the wrong place (possibly the reserved field or early). Always either ensure your higher-level stack pads (most Ethernet drivers do pad to min frame size) or do it in driver.
* In FreeRTOS, do not call OS APIs that are not ISR-safe from the ISR (like `printf` or normal allocation). Use the proper deferred mechanisms.
* If porting this to another MCU, do not assume bit-endianness is the same – always verify 0xAA55 at the start.

By following the above guidelines and handling errors as recommended, the integration will be robust even under heavy network traffic or unexpected conditions. Next, we provide some integration examples to illustrate pieces of code tying these concepts together for ESP32.

## Integration Examples

To cement the concepts, here are examples of how to implement key parts of the QCA7000-ESP32 interface. These include code snippets for transmitting and receiving frames and configuring the ESP32’s SPI and GPIO interfaces. We provide both ESP-IDF style (which uses FreeRTOS and the driver APIs) and a conceptual bare-metal approach (using registers or simplified logic) for comparison.

### Example 1: ESP32 SPI and GPIO Configuration (ESP-IDF)

In ESP-IDF (FreeRTOS-based), you typically use the SPI Master driver to configure the bus and device. Below is an example configuration:

```c
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define QCA_SPI_HOST    HSPI_HOST  // Use HSPI (SPI2) or VSPI (SPI3) as available
#define GPIO_MISO       12        // Example GPIO pins, adjust as per wiring
#define GPIO_MOSI       13
#define GPIO_SCLK       14
#define GPIO_CS         15
#define GPIO_QCA_IRQ    4         // QCA7000 interrupt pin

void init_qca_spi() {
    // 1. Configure SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,    // Not used
        .quadhd_io_num = -1,    // Not used
        .max_transfer_sz = 1600 // max transfer size (slightly above max frame)
    };
    ESP_ERROR_CHECK(spi_bus_initialize(QCA_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // 2. Configure SPI device (QCA7000)
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2000000,           // 2 MHz
        .mode = 3,                           // SPI mode 3 (CPOL=1, CPHA=1):contentReference[oaicite:126]{index=126}
        .spics_io_num = GPIO_CS,             // CS pin
        .queue_size = 3,                     // Number of transactions that can be queued
        .flags = SPI_DEVICE_HALFDUPLEX,      // Half duplex (MOSI or MISO used at a time), typical for SPI Ethernet
        // .pre_cb = NULL, .post_cb = NULL (optional callbacks)
    };
    spi_device_handle_t qca_spi;
    ESP_ERROR_CHECK(spi_bus_add_device(QCA_SPI_HOST, &devcfg, &qca_spi));

    // 3. Configure IRQ pin
    gpio_config_t irq_cfg = {
        .pin_bit_mask = 1ULL<<GPIO_QCA_IRQ,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,   // QCA7000 likely drives actively; enable pull if needed
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE      // Interrupt on rising edge:contentReference[oaicite:127]{index=127}
    };
    gpio_config(&irq_cfg);

    // 4. Install GPIO ISR service and add handler
    gpio_install_isr_service(0);  // 0 = no flags
    gpio_isr_handler_add(GPIO_QCA_IRQ, qca_gpio_isr, (void*)0);
}
```

In the above:

* We initialized the SPI bus on `HSPI_HOST` (which corresponds to SPI2 on ESP32). We specify the MOSI, MISO, SCLK pins. `max_transfer_sz` is set to 1600 bytes, which is enough for our frames (including overhead).
* We add the QCA7000 as an SPI device on that bus with `mode=3` and `clock_speed_hz=2MHz`. The `SPI_DEVICE_HALFDUPLEX` flag is used because the QCA7000’s protocol is effectively half-duplex: we never use full-duplex transfers where MOSI and MISO carry unrelated data simultaneously; instead, we either send (write) or receive (read) at a time. This flag can optimize the driver’s behavior for such use.
* The IRQ GPIO is configured as input, no pulls (assuming the QCA7000 has a push-pull output for IRQ, which is likely; if it were open-drain, we’d enable pull-up). It triggers on rising edge.
* We install the ISR service and attach `qca_gpio_isr` as the handler for that pin. The handler would be defined elsewhere (likely to give a semaphore or notify a task as discussed).

**Note:** In a real application, we’d also want to set the GPIO for QCA7000 RESET (as output) and perhaps toggle it low/high during init.

### Example 2: Basic Register Read/Write (ESP-IDF SPI driver)

With the `spi_device_handle_t qca_spi` obtained above, we can perform register reads/writes. We will craft the 16-bit command as needed:

```c
spi_device_handle_t qca_spi; // assume this is globally stored from init

// Helper to swap bytes for 16-bit command if needed (ESP-IDF uses MSB first by default)
static inline uint16_t host_to_be16(uint16_t x) {
    return (x>>8) | (x<<8);
}

// Read a 16-bit internal register
esp_err_t qca_read_reg(uint16_t reg_addr, uint16_t *out_val) {
    // Construct command: bit15=1 (read), bit14=1 (internal), bits13-0 = reg_addr
    uint16_t cmd = 0x8000 | 0x4000 | (reg_addr & 0x3FFF);
    cmd = host_to_be16(cmd);
    uint16_t rx_buf;
    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = 16 + 16,  // 16 bits cmd + 16 bits data
        .tx_data = { cmd >> 8, cmd & 0xFF, 0x00, 0x00 }, // we can actually just set tx_data for cmd, data phase TX ignored
    };
    esp_err_t ret = spi_device_transmit(qca_spi, &trans);
    if (ret != ESP_OK) return ret;
    // The received 32-bit (4 bytes) contains some undefined bits for cmd phase and then the 16-bit data.
    // Actually, using halfduplex, the driver might ensure the first 2 bytes are sent, and next 2 bytes read.
    // We use .flags to use tx_data and rx_data arrays conveniently.
    uint8_t *rx_bytes = trans.rx_data;
    // The data returned should be in rx_bytes[2], rx_bytes[3] (because first 2 might be garbage or previous MISO output during command).
    uint16_t val = (rx_bytes[2] << 8) | rx_bytes[3];
    *out_val = val;
    return ESP_OK;
}

// Write a 16-bit internal register
esp_err_t qca_write_reg(uint16_t reg_addr, uint16_t value) {
    uint16_t cmd = 0x0000 | 0x4000 | (reg_addr & 0x3FFF); // bit15=0 write, bit14=1 internal
    cmd = host_to_be16(cmd);
    // Prepare 4 bytes: 2 bytes cmd, 2 bytes value (also in big-endian for wire)
    uint16_t be_val = host_to_be16(value);
    uint8_t tx_buf[4] = { cmd >> 8, cmd & 0xFF, be_val >> 8, be_val & 0xFF };
    spi_transaction_t trans = {
        .length = 32, // 32 bits (command + data)
        .tx_buffer = tx_buf,
        .rx_buffer = NULL // we don't expect meaningful MISO data
    };
    return spi_device_transmit(qca_spi, &trans);
}
```

In this snippet:

* We use `SPI_TRANS_USE_TXDATA` and `SPI_TRANS_USE_RXDATA` for read, which allow using small internal buffers (for performance).
* The read might receive the register data in the latter half of `rx_data`. Since we set half-duplex, the driver ensures the MOSI output during the data phase is just sending zeroes (don’t care), and MISO is captured. The exact alignment of `rx_data` is something to test; we might also split into two transactions: one 16-bit command (with `SPI_TRANS_USE_TXDATA`) followed by one 16-bit read. But doing it in one transaction as above should work given half-duplex.
* The write just sends 4 bytes (no need to read anything back).

**Usage example:**

```c
// Example usage of register read/write
uint16_t signature;
qca_read_reg(0x1A00, &signature);
printf("Signature = 0x%04X\n", signature);  // Expect 0xAA55:contentReference[oaicite:128]{index=128}

// Reset QCA via SPI_CONFIG
uint16_t spi_config;
qca_read_reg(0x0400, &spi_config);
spi_config |= (1 << 6); // QCASPI_SLAVE_RESET_BIT:contentReference[oaicite:129]{index=129}
qca_write_reg(0x0400, spi_config);
// After this, QCA will reset; perhaps wait a bit and re-read signature to confirm reboot.
```

### Example 3: Transmit Frame Function (Pseudo-code, Bare-Metal)

Below is a conceptual bare-metal function to send an Ethernet frame. It assembles the QCA7K framing, checks buffer space, and performs the SPI transfer by directly manipulating registers (this is pseudo-code, actual ESP32 register names not used for brevity).

```c
#define SPI_CMD_WRITE_EXT 0x0000  // bit15=0, bit14=0, rest 0
// Assume we have low-level SPI functions: spi_begin_transaction(), spi_transfer_byte(), spi_end_transaction()

int qca_send_frame(uint8_t *eth_frame, uint16_t eth_len) {
    // Pad frame if needed
    uint16_t frame_len = eth_len;
    uint8_t padding[60];
    if (frame_len < 60) {
        uint16_t pad_len = 60 - frame_len;
        memset(padding, 0, pad_len);
    }
    uint16_t payload_len = (frame_len < 60) ? 60 : frame_len;
    uint16_t total_len = payload_len + 8 + 2; // overhead + payload + EOF

    // Check QCA write buffer space
    uint16_t space;
    qca_read_reg(0x0200, &space);  // SPI_REG_WRBUF_SPC_AVA
    if (space < total_len) {
        // Not enough space: return error or queue frame
        return -1;
    }

    // Prepare frame buffer (could also stream directly via SPI without full copy)
    uint8_t *buf = malloc(total_len);
    if (!buf) return -1;
    // Fill header
    buf[0] = buf[1] = buf[2] = buf[3] = 0xAA;
    buf[4] = (uint8_t)(payload_len & 0xFF);
    buf[5] = (uint8_t)(payload_len >> 8);
    buf[6] = buf[7] = 0x00;
    // Copy Ethernet frame
    memcpy(buf + 8, eth_frame, frame_len);
    if (frame_len < 60) {
        memset(buf + 8 + frame_len, 0, 60 - frame_len);
    }
    // EOF
    buf[8 + payload_len] = 0x55;
    buf[8 + payload_len + 1] = 0x55;

    // Set BFR_SIZE
    qca_write_reg(0x0100, total_len);

    // Perform SPI transfer:
    spi_begin_transaction();                // Pull CS low, etc.
    // Send 16-bit command (write external)
    uint16_t cmd = SPI_CMD_WRITE_EXT;
    // Mode3: send MSB first
    spi_transfer_byte(cmd >> 8);
    spi_transfer_byte(cmd & 0xFF);
    // Send data bytes
    for (int i = 0; i < total_len; ++i) {
        spi_transfer_byte(buf[i]);
    }
    spi_end_transaction();                  // Release CS

    free(buf);
    return 0;
}
```

In this pseudo-code:

* We manually build the framed buffer. Note that FL (Frame Length) we place at buf\[4..5] should be little-endian, meaning if `payload_len` is 60, buf\[4]=0x3C, buf\[5]=0x00.
* We then set BFR\_SIZE register and do the SPI transfer. We directly toggle CS in `spi_begin_transaction`/`spi_end_transaction`.
* This is simplistic; a real bare-metal implementation on ESP32 would involve setting up the SPI controller registers (e.g., load command into data buffer, etc.) rather than byte-by-byte in a loop. However, if SPI is configured for manual control, one could bit-bang or use the hardware FIFO in a loop. For brevity, we just illustrate logically sending bytes.
* Also, this function blocks until transfer done. In an RTOS, you might yield after starting DMA, etc. But bare-metal typically you wait for SPI to finish in-line (or use polling flags).

### Example 4: Receive Handler (Pseudo-code, Bare-Metal)

This function would be called either in an ISR or polling loop when data is available. It reads one frame and returns it (or processes it).

```c
int qca_receive_frame(uint8_t *out_frame_buf, uint16_t *out_len) {
    // Check if data available
    uint16_t bytes;
    qca_read_reg(0x0300, &bytes);  // SPI_REG_RDBUF_BYTE_AVA
    if (bytes == 0) {
        return -1; // no data
    }
    // bytes now holds total length of data to read (including framing)
    if (bytes > MAX_QCA_FRAME_BYTES) {
        // This should not happen, but protect against buffer overflow
        bytes = MAX_QCA_FRAME_BYTES;
    }
    // Set BFR_SIZE = bytes
    qca_write_reg(0x0100, bytes);
    // SPI read 'bytes' into a local buffer
    uint8_t *buf = malloc(bytes);
    if (!buf) return -1;
    spi_begin_transaction();
    uint16_t cmd = 0x8000; // read external (bit15=1, bit14=0)
    spi_transfer_byte(cmd >> 8);
    spi_transfer_byte(cmd & 0xFF);
    for (int i = 0; i < bytes; ++i) {
        buf[i] = spi_transfer_byte(0x00); // send dummy, read data
    }
    spi_end_transaction();
    // Now parse buf:
    uint32_t len_field = buf[0] | (buf[1]<<8) | (buf[2]<<16) | (buf[3]<<24);
    // If needed, adjust endianness: here we reconstructed as little-endian 32-bit.
    // Check SOF
    if (buf[4]!=0xAA || buf[5]!=0xAA || buf[6]!=0xAA || buf[7]!=0xAA) {
        // Bad frame marker
        free(buf);
        return -2;
    }
    uint16_t frame_len = buf[8] | (buf[9] << 8);  // little-endian FL
    // reserved buf[10..11] are 0x00 0x00
    // Check EOF
    if (buf[ bytes-2 ] != 0x55 || buf[ bytes-1 ] != 0x55) {
        // EOF missing
        free(buf);
        return -3;
    }
    // Validate lengths:
    // Ideally, len_field (if it includes everything after it) should equal (bytes - 4).
    // For safety, clamp frame_len to what we can copy.
    if (frame_len > MAX_ETH_FRAME_SIZE) {
        frame_len = MAX_ETH_FRAME_SIZE;
    }
    // Copy out the Ethernet frame
    memcpy(out_frame_buf, buf + 12, frame_len);
    *out_len = frame_len;
    free(buf);
    return 0;
}
```

Here:

* We first read how many bytes are available. In a real ISR, we’d have gotten that from INTR\_CAUSE PKT\_AVLBL and likely trust RDBUF\_BYTE\_AVA.
* We then allocate and read exactly that many bytes via SPI.
* We parse the framing: check SOF at buf\[4..7], EOF at the end, etc. We assemble the frame to deliver (from buf\[12] for length `frame_len` bytes).
* We ignore the LEN (first 4 bytes) except maybe for consistency checks with `bytes`.
* After copying, we would typically deliver `out_frame_buf` to the network stack (or return it to caller who will).
* In an actual driver, rather than allocating for each frame, you might use a static buffer or reuse one, especially in an ISR context to avoid malloc. Or have a pool of buffers.
* Also note, after reading, we should acknowledge the interrupt cause by writing to INTR\_CAUSE (the main interrupt handler would handle that outside this function, likely).

### Example 5: FreeRTOS Task for Handling QCA7000

Combining all, here’s a sketch of how a FreeRTOS task might handle the QCA7000 events:

```c
void qca_task(void *arg) {
    // Assume init_qca_spi() has been called, QCA is reset and signature verified.
    // Enable interrupts on QCA:
    qca_write_reg(0x0D00, 0x0047);  // CPU_ON, WRBUF_ERR, RDBUF_ERR, PKT_AVLBL:contentReference[oaicite:130]{index=130}

    // Maybe configure QCA MAC address via some mgmt frame or such if needed (not shown).

    while (1) {
        // Wait for interrupt notification from ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // When notified, read and handle interrupt causes as done in qca_handle_interrupt above
        uint16_t cause;
        qca_read_reg(0x0C00, &cause);
        // Mask further interrupts
        qca_write_reg(0x0D00, 0x0000);
        if (cause == 0) {
            // spurious wakeup, re-enable interrupts and continue
            qca_write_reg(0x0D00, 0x0047);
            continue;
        }
        bool need_reset = false;
        if (cause & 0x40) { // CPU_ON
            need_reset = true;
        }
        if (cause & 0x4) { // WRBUF_ERR
            printf("WRBUF_ERR!\n");
            need_reset = true;
        }
        if (cause & 0x2) { // RDBUF_ERR
            printf("RDBUF_ERR!\n");
            need_reset = true;
        }
        if (cause & 0x1) { // PKT_AVLBL
            // Read all packets
            uint16_t bytes;
            qca_read_reg(0x0300, &bytes);
            while (bytes > 0) {
                // Allocate buffer for packet
                struct pbuf *p = pbuf_alloc(PBUF_RAW, bytes, PBUF_POOL);
                if (!p) {
                    // No memory, drop packet by reading and discarding
                    uint16_t throwaway_bytes = bytes;
                    qca_write_reg(0x0100, throwaway_bytes);
                    // perform SPI read and ignore data...
                    // (Better: could try to reuse a static buffer to drop data instead of pbuf)
                    for (int i = 0; i < throwaway_bytes; ++i) {
                        uint8_t dummy;
                        spi_transfer_byte(0x00, &dummy);
                    }
                } else {
                    // Actually read the packet into p->payload
                    qca_write_reg(0x0100, bytes);
                    // Use ESP-IDF SPI to transfer directly into p->payload:
                    spi_transaction_t t = {
                        .length = bytes * 8,
                        .rx_buffer = p->payload,
                        .flags = 0,
                    };
                    uint16_t cmd = host_to_be16(0x8000);
                    // We send command first
                    spi_device_polling_transmit(qca_spi, &(spi_transaction_t){
                        .length = 16, .tx_buffer = &cmd});
                    // Now receive the data bytes
                    spi_device_polling_transmit(qca_spi, &t);
                    // Now validate frame in p->payload
                    // (We might validate SOF/EOF here, or assume it's correct to save time.)
                    // Remove QCA header/footer:
                    uint16_t fl = *(uint16_t*)((uint8_t*)p->payload + 8);
                    uint16_t eth_len = fl;
                    if (eth_len < 60) eth_len = 60;
                    // Trim off QCA overhead
                    pbuf_remove_header(p, 12); // remove LEN(4)+SOF(4)+FL(2)+RSVD(2)
                    pbuf_realloc(p, eth_len);   // drop EOF
                    // Pass to TCP/IP stack
                    esp_netif_receive(qca_netif, p, NULL); // or in raw LwIP: netif->input(p, netif)
                }
                qca_read_reg(0x0300, &bytes); // check if another packet arrived
            }
        }
        // Clear interrupts
        qca_write_reg(0x0C00, cause);
        // Re-enable interrupts
        qca_write_reg(0x0D00, 0x0047);
        if (need_reset) {
            // Re-init QCA (this might involve toggling reset pin or using SPI_CONFIG reset bit)
            perform_qca_reset_and_init();
        }
    }
}
```

This is an illustrative FreeRTOS task:

* It waits for a notification (given by the GPIO ISR).
* When woken, reads causes, masks interrupts, handles each cause, and clears them.
* It reads multiple packets if available, allocating pbufs for each and passing them to the network.
* If memory not available, it simply discards the data to avoid blocking QCA (by reading it out and ignoring).
* Uses `esp_netif_receive` to deliver the packet to the stack (this is ESP-IDF specific).
* On error conditions, triggers a re-init (not fully implemented here).
* The use of `spi_device_polling_transmit` ensures we perform SPI operations quickly and synchronously. We split the command and data phases for the external read to integrate easier with IDF API (since building a single transaction with command and data might require the half-duplex usage as earlier).

**Note:** This code might need refinement for an actual product, but it demonstrates structure and interplay between pieces.

### Example 6: What Not to Do (Illustrative Mistakes)

To highlight pitfalls, here are some incorrect usages and why they are wrong:

* **Not setting BFR\_SIZE:**

  ```c
  // WRONG: forgetting to set BFR_SIZE before an external write
  qca_write_reg(0x0200, some_value); // attempt to write data directly to WRBUF (if one assumed it works like offset)
  // Then doing SPI write without setting 0x0100 BFR_SIZE.
  ```

  This will not work because QCA needs BFR\_SIZE to know how many bytes to expect in an external write. Always set 0x0100 before the transfer.

* **Using wrong SPI mode:**

  ```c
  devcfg.mode = 0; // CPOL=0, CPHA=0 (Mode 0) - WRONG for QCA7000
  ```

  This misconfiguration would cause MISO/MOSI sampling at wrong times. The signature 0xAA55 might come back as 0xAB?? or vary. Always use CPOL=1, CPHA=1 (Mode 3).

* **Clearing interrupts incorrectly:**

  ```c
  uint16_t cause;
  qca_read_reg(0x0C00, &cause);
  // Suppose cause = 0x0001 (PKT_AVLBL).
  qca_write_reg(0x0C00, 0xFFFF); // writing 0xFFFF to clear, thinking it clears all
  ```

  While writing 0xFFFF might clear the defined bits, it's not explicitly documented to do so (writing undefined bits might be ignored or could clear them if reserved are R/W). It’s safer to write back exactly what was read. Overclearing could potentially mask an event that occurred between read and write (though if between read and write a new event came, writing 0xFFFF could inadvertently clear it without processing). So always clear only those you handled.

* **Toggling CS incorrectly in burst mode:**

  ```c
  // WRONG: toggling CS for each byte (legacy style) while QCA is in burst mode
  for (int i = 0; i < len; ++i) {
      spi_select(); // CS low
      spi_transfer(data[i]);
      spi_deselect(); // CS high after each byte
  }
  ```

  This would send each byte as a separate transaction. QCA in burst mode expects CS low for entire frame. Doing this will likely result in QCA seeing the first byte as the command, then next CS assert resets its SPI state machine expecting a new command, etc., completely mangling communication. Only do this if QCA is in legacy mode (in which case each 16-bit chunk, not each byte, needs separate CS toggle, but still inefficient).

* **Not handling minimum frame length:**
  If the host passes a 54-byte Ethernet frame (which is below 60), and driver doesn’t pad:

  ```c
  uint16_t FL = 54;
  // ... fill frame ...
  // put FL=54 in header and don't pad bytes 54-59
  ```

  The QCA will consider 54 bytes of actual Ethernet data, but Ethernet spec requires at least 60 bytes data (without FCS). The QCA note explicitly says pad to 60. If not padded, those 6 bytes could be random memory (if you still sent some bytes, or if you sent none, QCA might have gotten less bytes than FL indicates, causing confusion).
  Always pad frames < 60 with zeros to meet minimum length.

* **Ignoring CPU\_ON:**
  If QCA reboots and the driver doesn’t catch CPU\_ON, you might continue trying to send/receive as if nothing happened. You’d start seeing signature mismatches or timeouts. Ignoring CPU\_ON means you won’t re-init interrupts (since QCA likely defaulted them off on reboot). So the device would be unresponsive. Thus, handle CPU\_ON by re-syncing as described.

* **Infinite blocking wait on space:**

  ```c
  while (qca_get_space() < frame_len) {
      // wait (no timeout)
  }
  send_frame();
  ```

  If something went wrong such that space never frees (maybe QCA hung), this would lock the thread forever. Always incorporate a timeout or break condition to escape and possibly reset QCA or at least log an error.

* **Not protecting SPI from concurrent access:**
  If your design has, say, one task sending and an ISR receiving that both call `spi_device_transmit` without a mutex, the ESP32 SPI driver is thread-safe but not ISR-safe (it uses semaphores). This can cause deadlocks or crashes. Always protect or sequence accesses (the single qca\_task approach above does sequence).

These examples should give a concrete sense of how to implement the QCA7000 interface and avoid common pitfalls. In practice, refer also to any existing driver (e.g., Linux qca\_spi.c) for additional reference, and thoroughly test each part of the communication (register access, sending, receiving) under various conditions.

## What Not to Do

To summarize some **common mistakes and pitfalls** when using the QCA7000 with ESP32 (or any host):

* **Incorrect SPI Mode:** As emphasized, do not use the wrong SPI mode. Mode 3 is required. Using mode 0 or 1 will cause mis-sampling of data. A classic symptom is the signature register not returning 0xAA55. Always double-check the SPI mode configuration if the signature is wrong or data seems bit-shifted.

* **Neglecting Frame Boundaries:** Do not send or expect raw Ethernet frames without QCA framing. The QCA7000 is not a simple SPI SRAM or MAC that takes raw frames directly; it needs the SOF/EOF markers and length fields. So, do not attempt to bypass the framing. Similarly, do not forget to pad frames to the minimum length of 60 bytes (payload length) – sending a shorter frame without padding will result in an improperly formatted Ethernet packet (which could confuse the receiving node or violate spec). Likewise, when receiving, don't assume every frame is exactly one SPI transaction unless you coded it that way; always parse using the markers, not just length, in case data alignment issues occur.

* **Chip Select Misuse:** If QCA7000 is configured for burst mode (which it usually is by default for SPI usage), do not toggle CS in the middle of a transfer. This means do not configure the ESP32 SPI to de-assert CS after each byte or 16-bit word. The entire command + data sequence must be one CS-low session. Toggling CS too frequently can cause lost sync and essentially garbage data to be latched. Conversely, if the QCA were in legacy mode, *failing* to toggle CS after each 16 bits would cause its SPI state machine to misalign. So, ensure CS behavior matches the mode: continuous in burst, toggling in legacy (but again, strive to use burst to avoid this complexity).

* **Ignoring Interrupt Storms:** Do not enable interrupts and then never clear the causes. A common mistake is to handle an interrupt at the MCU side but forget to write back to INTR\_CAUSE to acknowledge it. This will leave the IRQ line high (level-triggered scenario) or you just won't get another rising edge (if edge-triggered) and you’ll think no further interrupts are coming even though QCA is trying to signal you. Always clear interrupts by writing the cause bits back once you’ve handled them. Also, do not re-enable the interrupts on QCA before clearing the current ones; doing so could lead to immediate re-triggering if the cause is still set.

* **Mismatched Buffer Management:** Don’t mis-manage the read/write buffer pointers:

  * Do not read fewer or more bytes than indicated. If RDBUF\_BYTE\_AVA says 100 bytes, do not issue a read for 98 or 102 bytes – stick to 100. Reading less will leave 2 bytes in QCA that you might never see or that might corrupt the next frame; reading more is not allowed and QCA will either not output extra or will output garbage and flag an error.
  * Similarly for writes, do not send more bytes than you set in BFR\_SIZE, and do not set BFR\_SIZE larger than the actual data you plan to send. The QCA7000 has no concept of “early EOF” from the host side – it expects exactly the number of bytes told. If you set a larger BFR\_SIZE and then only send that many by padding with dummy data, those dummy bytes will become part of a frame (likely confusing the PLC side). If you terminate the transfer early (CS high early), QCA thinks bytes are missing and will wait or error.
  * Don’t forget that QCA’s internal buffers are finite. If you continuously send without checking WRBUF\_SPC\_AVA, you'll overflow it (triggering WRBUF\_ERR). Similarly, don't ignore the possibility of QCA’s read buffer filling up; if your host for some reason can’t read for a while (maybe interrupts off), you might get RDBUF\_ERR.

* **Forgetting to Reset on Errors:** If you do hit an error condition like WRBUF\_ERR or RDBUF\_ERR (or suspect a loss of sync), don’t attempt to continue normal operation without a reset. These errors indicate the QCA7000’s internal state (and possibly the host’s view of it) is compromised. For instance, do not just clear the interrupt and pretend nothing happened – the data pipeline might be broken. The safe move is a reset and re-init of the QCA7000 in these cases. It’s better to endure a short reconnection downtime than to operate on bad data.

* **Hard-coding values without R-M-W:** Do not write to registers like SPI\_CONFIG blindly. The QCA docs advise to preserve other bits. For example, if multi\_cs bit is in SPI\_CONFIG and you want to change it, read SPI\_CONFIG, modify that bit, and write it back, rather than writing an assumed value that might inadvertently clear the reset bit or other flags. Similarly, for INTR\_ENABLE, it’s usually fine to write a full value since we know the bits, but for any config register with unknown reserved bits, always R-M-W.

* **Lack of Flow Control on Host Side:** Don’t overload the host or network stack. If you are receiving packets faster than you can process, and you have no backpressure, you’ll eventually overflow QCA or run out of buffers. In an RTOS, this might mean ensuring the TCP/IP stack has adequate buffering (increase pbuf pool if needed) or your application slows down when needed. For sending, if you attempt to send packets faster than QCA can put on PLC, and you just busy-wait in a loop, you may hog the CPU. Instead, use the interrupt or a delay to pace transmissions. The QCA7000's 10 Mbps PLC link likely is the bottleneck anyway, so the host should not exceed that data rate by a wide margin in sustained fashion.

* **Interrupt Priority Mismanagement:** On ESP32, the default GPIO ISR might be at a certain priority that allows FreeRTOS calls or not. Don’t try to call complex FreeRTOS or SPI APIs in an ISR if they are not ISR-safe. For instance, `spi_device_transmit` is not safe in a high-level ISR (it might wait on a semaphore). If you must do SPI in the ISR, use polling versions or set appropriate flags. Or better, do as we did: defer heavy work to a task. The mistake would be doing `esp_netif_receive()` directly in the ISR (not allowed) or `spi_device_transmit()` in ISR (can deadlock). Always adhere to which functions can be called from an ISR (often they have suffix `_fromISR` in FreeRTOS).

* **Not Preserving Alignment (Performance):** While not a functional mistake, ignoring alignment can degrade performance. For example, if you allocate a buffer for DMA that isn’t 32-bit aligned, the ESP32 SPI DMA might copy via bounce buffers (slower). So, try to allocate with `MALLOC_CAP_DMA` or declare static buffers in DRAM for big transfers. This tip ensures you don’t inadvertently slow the throughput. A user might not realize their throughput is low because every SPI transfer falls back to non-DMA due to alignment issues.

* **Ignoring Electrical Considerations:** Outside of software – do not connect the QCA7000 without considering signal integrity. For instance, using long wires for SPI without proper termination or level shifting (if voltage mismatch, though QCA7000 is 3.3 V, so fine with ESP32) can cause intermittent failures. Also ensure the QCA7000’s power rails are stable and that its clock (if an external crystal) is stable before SPI communication. If the QCA7000 is part of a module, usually these are handled, but these are general warnings.

* **Neglecting EV-specific requirements:** Since this is for EV communication, note that there might be times the QCA7000 goes into a low-power mode or reset triggered by the car’s state or pilot signal. Make sure your software handles reinitialization in such cases. It's more of a system design note: do not assume the QCA is always on – if the EV is not connected or goes to sleep, the PLC might stop. The driver should be ready for “link down” events (which may appear as CPU\_ON when it comes back).

By avoiding these mistakes, you greatly increase the chances of a stable and reliable integration. Testing under various scenarios (power cycles, heavy traffic, error injection) will further ensure the system behaves as expected in the field.

## Appendix

**Relevant Constants and Macros:**

```c
// SPI Register Addresses
#define QCA7K_SPI_REG_BFR_SIZE       0x0100  // Buffer size register (W):contentReference[oaicite:148]{index=148}
#define QCA7K_SPI_REG_WRBUF_SPC_AVA  0x0200  // Write buffer space available (R):contentReference[oaicite:149]{index=149}
#define QCA7K_SPI_REG_RDBUF_BYTE_AVA 0x0300  // Read buffer bytes available (R):contentReference[oaicite:150]{index=150}
#define QCA7K_SPI_REG_SPI_CONFIG     0x0400  // SPI configuration (R/W):contentReference[oaicite:151]{index=151}
#define QCA7K_SPI_REG_INTR_CAUSE     0x0C00  // Interrupt cause (R/W):contentReference[oaicite:152]{index=152}
#define QCA7K_SPI_REG_INTR_ENABLE    0x0D00  // Interrupt enable (R/W):contentReference[oaicite:153]{index=153}
#define QCA7K_SPI_REG_SIGNATURE      0x1A00  // Signature register (R):contentReference[oaicite:154]{index=154}

// Bit masks for SPI_CONFIG register
#define QCASPI_SLAVE_RESET_BIT   (1 << 6)    // Writing 1 triggers QCA7000 reset:contentReference[oaicite:155]{index=155}
#define QCASPI_MULTI_CS_BIT      (1 << 1)    // Multi-CS (legacy mode) enable bit (if applicable, default 1 = legacy):contentReference[oaicite:156]{index=156}
                                           // Note: ensure correct register for multi_cs (it might be in ACT_CTRL at 0x1B00)

// Bit masks for Interrupt Cause/Enable (0x0C00 / 0x0D00)
#define SPI_INT_PKT_AVLBL        (1 << 0)    // Packet available interrupt:contentReference[oaicite:157]{index=157}
#define SPI_INT_RDBUF_ERR        (1 << 1)    // Read buffer error interrupt:contentReference[oaicite:158]{index=158}
#define SPI_INT_WRBUF_ERR        (1 << 2)    // Write buffer error interrupt:contentReference[oaicite:159]{index=159}
#define SPI_INT_CPU_ON           (1 << 6)    // CPU on (startup) interrupt:contentReference[oaicite:160]{index=160}
#define SPI_INT_WRBUF_BELOW_WM   (1 << 10)   // Write buffer below watermark (if used):contentReference[oaicite:161]{index=161}

// SPI Command bit definitions
#define QCA7K_SPI_CMD_READ        (1 << 15)  // Read operation (MSB of command):contentReference[oaicite:162]{index=162}
#define QCA7K_SPI_CMD_WRITE       (0 << 15)  // Write operation
#define QCA7K_SPI_CMD_INTERNAL    (1 << 14)  // Internal register access:contentReference[oaicite:163]{index=163}
#define QCA7K_SPI_CMD_EXTERNAL    (0 << 14)  // External buffer access:contentReference[oaicite:164]{index=164}
#define QCA7K_SPI_CMD_ADDR_MASK   0x3FFF    // Mask for 14-bit address in command:contentReference[oaicite:165]{index=165}

// Framing constants
#define QCA7K_SOF_MARKER   0xAAAAAAAA  // Start-of-frame marker:contentReference[oaicite:166]{index=166}:contentReference[oaicite:167]{index=167}
#define QCA7K_EOF_MARKER   0x5555      // End-of-frame marker:contentReference[oaicite:168]{index=168}:contentReference[oaicite:169]{index=169}
#define QCA7K_MIN_FRAME_LEN 60        // Minimum Ethernet payload length (no FCS):contentReference[oaicite:170]{index=170}
#define QCA7K_MAX_FRAME_LEN 1518      // Maximum Ethernet payload length (no VLAN):contentReference[oaicite:171]{index=171}
#define QCA7K_MAX_FRAME_LEN_VLAN 1522 // Maximum with VLAN

// SPI timing and mode
#define QCA7K_SPI_MODE    3           // CPOL=1, CPHA=1:contentReference[oaicite:172]{index=172}
#define QCA7K_SPI_MAX_FREQ 16000000   // 16 MHz max (as per Linux binding):contentReference[oaicite:173]{index=173}
#define QCA7K_SPI_MIN_FREQ 1000000    // 1 MHz min (for stable ops):contentReference[oaicite:174]{index=174}
```

**SPI Transaction Sequence Diagrams:**

Below we describe typical SPI waveforms for QCA7000 operations:

* **Internal Register Read (16-bit):**
  **CS**: Goes low at start of transaction, stays low for 16 command bits + 16 data bits, then high.
  **CLK**: Idle high (CPOL=1). Once CS is low, clock toggles (CPHA=1 means first transition is a no-data half-period, data is sampled on rising edges).
  **MOSI**: Carries the 16-bit command. For example, reading the SIGNATURE register (0x1A00): the command bits = `1 1 011010000000000` (binary for 0xD800, if we break it: bit15=1 read, bit14=1 internal, bits13-0=0x1A00). This will be sent MSB first: MOSI will output `11011010 00000000` (0xDA 0x00). After those 16 bits, during the next 16 clocks MOSI is don't-care (the host typically outputs 0x00 bytes).
  **MISO**: Undefined during command phase (QCA might hold it high or low). During the data phase, QCA outputs the register value MSB first. For SIGNATURE, expected 0xAA55, the MISO will output `10101010 01010101` bit sequence. The host reads that in and should interpret it correctly as 0xAA55.
  Timing: There is essentially a continuous 32-clock sequence while CS is low. Data on MOSI is latched on QCA on rising edges; data on MISO is driven by QCA on falling edges and read by host on rising edges.

  *Diagram (conceptual):*

  ```
  MOSI: [D7 D6 ... D0] [X  ... X] (D7..D0 = 0xDA, X= don't care in data phase)
  MISO: [?? ?? ... ??] [A5 A4 ... A0] (??=indeterminate during cmd, A bits = register value bits)
  CS:   _________--------------------------___________   (low for duration of 32 clocks) 
  CLK:  ----____----____----____----____----____----____  (CPOL=1 so idle high, toggling on edges)
  ```

  (Where `----` is high half, `____` is low half for each clock pulse; data bits change on falling edges and are sampled on rising edges for CPHA=1.)

* **External Write (burst mode):**
  Suppose we send a 10-byte frame (including QCA overhead) as an example.
  **CS**: Low for entire command+10 bytes transfer.
  **CLK**: continuous clock for (16 + 10\*8) bits.
  **MOSI**: First 16 bits is command (e.g., 0x0000 for write external). Then byte-by-byte, the frame data. For instance, if the frame data (with overhead) bytes are `AA AA AA AA  0A 00 00 00  ... 55 55` (where 0x0A00 is FL=10 etc.), MOSI will output them in sequence right after the command.
  **MISO**: Likely undefined throughout, because in write mode QCA isn't sending meaningful data. The QCA might drive MISO to 0 or leave it floating (the Linux driver configures SPI as half-duplex so it ignores MISO during write).

  *Waveform highlights:* The CS is asserted, MOSI sends 0x00 0x00 (command), then immediately sends the data bytes one after another without releasing CS. MISO may remain at 0 (or last read data) – it should be ignored.

* **External Read (burst mode):**
  E.g., reading 10 bytes.
  **CS**: Low through command + 10 bytes of data.
  **MOSI**: 16-bit command (0x8000). After that, host keeps clocking but MOSI can send 0x00 bytes (or any dummy pattern) for each byte of data expected.
  **MISO**: Indeterminate during the 2 bytes of command (QCA isn't sending yet). After command is received, on the next clock cycles QCA outputs the requested data bytes in order. If the data was, say, the same sequence as above, MISO will output `AA AA AA AA  0A 00 00 00  ... 55 55`. MOSI's output during this time is ignored by QCA.

  The host SPI in half-duplex mode typically handles this by not driving MOSI at all during the data phase, but if full-duplex mode is used, the host must send dummy bytes. On ESP32, one can do full-duplex and just disregard MOSI content. The key is CS stays low and continuous clocks ensure bytes shift out sequentially.

* **Legacy Mode Timing Difference:**
  If multi\_cs (legacy) mode is enabled, the expectation is that CS will deassert between each 16-bit word. For example, an external write of 4 bytes in legacy mode would look like: send command+first byte as one transaction (CS low for 16 bits only?), then drop CS, then raise CS again to send next byte, etc. Actually legacy mode's exact requirement per documentation: "the SPI master must toggle the chip select between each data word". So likely after every 16 bits (which can be one byte command + one byte data or however alignment, but presumably 16 bits). It's a bit ambiguous but presumably each 16-bit chunk (like in internal regs or maybe even each byte of data) needs separate CS. That is an inefficient mode and rarely used. The timing diagram would just show CS pulsing low for each chunk with gaps of idle between.

**Timing Diagram Consideration:**
One might draw a diagram for a multi-byte transfer. However, due to text format, the above descriptions serve as our "diagrammatic" explanation. The key points are:

* **CS**: single low pulse per message in burst mode.
* **MOSI/MISO phase alignment**: Mode3 ensures that on each rising clock edge, data is sampled. Setup occurs on the prior falling edge.
* **Inter-byte gaps**: In burst mode, ideally none (the ESP32 SPI will clock bytes back-to-back). If using bit-banging, ensure minimal gaps. In legacy, gaps introduced by CS toggling.

**Performance Consideration:**
At 8 MHz, each bit is 125 ns, a byte 1 µs. A full 1500-byte (approx 1536 bytes including overhead) frame read or write = \~1536 µs = \~1.5 ms on the bus, which is fine. At 16 MHz, \~0.75 ms. The interrupt overhead and processing will add some, but generally the system can handle full 10 Mbps throughput (which is \~1.25 MB/s, and SPI at 8 MHz can do \~1 MB/s raw, so it's borderline; at 12-16 MHz it should handle 10 Mbps easily).

In conclusion, this document covered all aspects of integrating QCA7000 with ESP32: **overview** of QCA7K for EV, detailed **hardware interface** and SPI requirements, **software architecture** for bare-metal vs RTOS, thorough **register-level** protocol and usage, the **framing layer** specifics, end-to-end **data flow** with queuing and modes, **interrupt management** best practices, **initialization & synchronization** sequences, **error handling** strategies with robust design tips, integration **code examples**, and a list of **things to avoid** to prevent common errors. Following these guidelines, one can implement a reliable Ethernet-over-SPI driver for QCA7000 on ESP32, enabling the EV communication features to function correctly and efficiently.
