# Design of AMBA APB Slave UART Wrapper

A UART (Universal Asynchronous Receiver/Transmitter) wrapped behind an AMBA APB slave interface, written in Verilog — plus an interactive, in-browser simulator that visualizes data physically moving between a CPU, the UART peripheral, and an external serial device.

**🔴 Live simulator:** https://iamparv7043.github.io/Design-of-AMBA-APB-Slave-UART-Wrapper/

---

## Overview

This project implements a memory-mapped UART controller that a CPU (or any APB master) can drive through two simple registers — one to send/receive a byte, one to check status — while the UART core itself independently serializes and deserializes data on the physical TX/RX pins at a fixed baud rate.

Default configuration:
- **Clock:** 50 MHz
- **Baud rate:** 115,200
- **Frame format:** 1 start bit + 8 data bits (LSB first) + 1 stop bit, no parity

## Why an APB wrapper?

Real UART peripherals are almost never talked to directly by combinational logic — they sit on a bus so a CPU can configure and poll them like any other peripheral (GPIO, timers, etc.). APB (part of ARM's AMBA family) is the simplest of these buses: no burst transfers, no pipelining, just a straightforward address/data/ready handshake. That makes it a natural fit for a low-throughput peripheral like a UART.

## Architecture

The design is split into independently-clocked-but-synchronous blocks, all running on `pclk`:

| Block | Responsibility |
|---|---|
| **Baud tick generator** | Free-running counter that pulses `baud_tick` once every bit period (`CLK_FREQ / BAUD_RATE` cycles) |
| **RX synchronizer** | Two-flop synchronizer on the async `rx` pin to avoid metastability before it touches any FSM |
| **TX FSM** | `IDLE → START → DATA (×8) → STOP`, serializes `tx_wdata` onto the `tx` pin, one bit per `baud_tick` |
| **RX FSM** | `IDLE → START → DATA (×8) → STOP`, deserializes incoming bits off `rx_s` into `rxdata_reg` |
| **APB slave interface** | Single-wait-state, address-decoded register access into/out of the TX/RX blocks |

### TX path

Writing the DATA register only *arms* a transmission — it loads `tx_wdata` and sets `tx_busy`, moving the FSM to `TX_START`. The actual line activity (driving the start bit, then each data bit, then the stop bit) only happens on `baud_tick` boundaries, which is what makes the transmission take exactly one bit period per step rather than one clock cycle. `tx_busy` clears automatically once the stop bit has been driven.

### RX path

The RX FSM is purely reactive to whatever the synchronized `rx_s` line does — there's no APB involvement until a full byte has arrived. A subtlety worth calling out: `RX_START` doesn't commit to receiving a byte the instant it sees `rx_s` go low. It waits **half a bit period**, then re-checks — if the line is still low, it's treated as a real start bit; if not, it was a glitch and the FSM falls back to `RX_IDLE`. This is the standard mid-bit sampling technique that keeps the RX FSM from being fooled by short noise pulses. If the stop bit isn't sampled high at the end of the frame, the byte is silently dropped as a framing error rather than being latched into `rxdata_reg`.

### Simultaneous access

TX and RX are fully decoupled — a byte can be transmitted and received in the same window of time with no interaction between the two FSMs, which is normal for full-duplex UART.

## Register map

| Address | Name | Access | Description |
|---|---|---|---|
| `0x0` | `DATA` | R/W | **Write:** loads the byte to transmit and starts sending it (ignored if `tx_busy = 1`). **Read:** returns the last byte received and clears `rx_valid`. |
| `0x4` | `STATUS` | R | Bit 0 = `tx_busy` (transmission in progress). Bit 1 = `rx_valid` (a received byte is waiting to be read). |

A typical polling sequence from firmware:
```
poll STATUS until tx_busy == 0
write DATA = byte_to_send
...
poll STATUS until rx_valid == 1
read DATA  → received byte (also clears rx_valid)
```

## Port list

```verilog
module apb_uart #(
    parameter CLK_FREQ  = 50_000_000,
    parameter BAUD_RATE = 115_200
) (
    input  wire        pclk,
    input  wire        presetn,   // active-low, single reset

    // UART pins
    input  wire        rx,
    output reg         tx,

    // APB slave interface
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [31:0] paddr,
    input  wire [7:0]  pwdata,
    output reg  [7:0]  prdata,
    output reg         pready
);
```

## Files

| File | Description |
|---|---|
| `apb_uart.v` | The Verilog module (baud generator, RX synchronizer, TX/RX FSMs, APB slave logic) |
| `index.html` | Interactive simulator — see below |

## Using the simulator

Open `index.html` (or the live link above). It models three separate "devices" and the wires between them:

- **CPU (APB master)** — enter a hex byte and click **write** to arm a transmission; **read DATA** / **read STATUS** pull values back over the APB bus, shown as a pulse traveling the wire.
- **APB UART peripheral** — shows the TX and RX paths as they'd actually look on a logic analyzer: live 8-bit shift registers (bit 7 → bit 0), current FSM state, and the `tx_busy` / `rx_valid` flags.
- **External device** — the thing on the other end of the serial wire. Enter a byte and click **send** to drive it onto the RX pin bit by bit; it also shows what it has received from the UART's TX pin, and can inject a corrupted stop bit to demonstrate the framing-error path.

Click **step one bit period** repeatedly to advance the whole system by one baud tick at a time and watch data physically move from one box's register into the other's.

## Possible extensions

- Parity bit support (even/odd)
- Configurable data width (7/8/9 bits) and stop bits
- TX/RX FIFOs instead of single-byte registers, to decouple CPU polling rate from line rate
- Interrupt output instead of pure polling (`irq` asserted on `rx_valid` or `tx_busy` falling edge)

## License

MIT
