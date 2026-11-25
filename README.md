# CAN Bus

## Project 1: Two-Node CAN Network

- Goal: Build a 2-node CAN bus network in C between STM32F429 (bxCAN) and STM32H723 (FDCAN) Nucleo boards, sending periodic sensor frames at 500 kbps with hardware acceptance filtering and interrupt-driven RX

### Wiring

Transceivers: Waveshare SN65HVD230 (3.3 V). Pins are the same on both boards (PD0/PD1, AF9, on the CN9 connector).

| Waveshare pin | NUCLEO-F429ZI | NUCLEO-H723ZG |
|---|---|---|
| 3V3 | 3V3 | 3V3 |
| GND | GND | GND |
| CTX | PD1 (CAN1_TX) | PD1 (FDCAN1_TX) |
| CRX | PD0 (CAN1_RX) | PD0 (FDCAN1_RX) |

Bus connections between the two transceivers:

| Waveshare #1 | Waveshare #2 |
|---|---|
| CANH | CANH |
| CANL | CANL |
| GND | GND |

- Wire TX to CTX and RX to CRX straight through (not crossed).
- Each Waveshare board has a 120 Ω termination resistor, so the bus is terminated at both ends. With power off, CANH to CANL should measure about 60 Ω.
- The GND wire between transceivers is optional here: with both Nucleos plugged into the same laptop, their grounds are already connected through USB. Add it if a board is powered from a different source (charger, power bank, another computer) or the bus gets longer.

![Project 1 wiring: NUCLEO-F429ZI (top) and NUCLEO-H723ZG (bottom), each wired to a Waveshare SN65HVD230 transceiver on the breadboard](images/project1-wiring.jpg)

### Bit Timing (500 kbps)

Bit rate = CAN clock ÷ (prescaler × (1 + Seg1 + Seg2))

The formula builds a bit in three steps: clock ticks → time quanta (tq) → one bit.

#### 1. CAN clock

The CAN clock is the clock feeding the CAN peripheral (see the "CAN clock" row in the table below), not the CPU core clock:

- **F429ZI:** the APB1 peripheral clock (PCLK1).
- **H723ZG:** the FDCAN kernel clock, selected by the FDCANSEL mux (HSE, PLL1Q or PLL2Q). FDCAN also uses APB1 for register access, but bit timing comes only from the kernel clock.

The **period** of the CAN clock is how long one tick lasts: period = 1 ÷ CAN clock (12.5 ns at 80 MHz).

#### 2. Time quantum (tq)

A **time quantum (tq)** is the basic unit of time the CAN controller uses to build a bit. Every part of the bit (Sync, Seg1, Seg2, SJW) is set as a whole number of tq, because tq are what the hardware counts.

The **prescaler** creates the tq. It is a counter that counts CAN clock ticks and marks one tq every `prescaler` ticks:

```
CAN clock ticks: |1|2|3|4|5|6|7|8|9|10|1|2|3|4|5|6|7|8|9|10| ...
tq:              |<------ tq #1 ------>|<------ tq #2 ------>|      (prescaler = 10)
```

So one tq lasts `prescaler` ticks:

```
tq = prescaler × CAN clock period = prescaler × (1 ÷ CAN clock) = prescaler ÷ CAN clock
```

A tq has **no fixed duration**. Like a beat in music, its real duration depends on the tempo, which is set by the CAN clock and prescaler:

| | NUCLEO-F429ZI | NUCLEO-H723ZG |
|---|---|---|
| CAN clock period (1 ÷ CAN clock) | 1 ÷ 45 MHz = 22.2 ns | 1 ÷ 80 MHz = 12.5 ns |
| Ticks per tq (prescaler) | 6 | 10 |
| **1 tq** | **133.3 ns** | **125 ns** |

A bigger prescaler means more ticks per tq, so a longer tq and a slower bit rate.

#### 3. One bit is a whole number of tq

Each bit is split into three segments, all measured in tq:

```
|<------------------------- one bit = 16 tq = 2 µs (H723) ------------------------->|
| Sync |<----------------------- Seg1 (13 tq) ----------------------->|<- Seg2 (2) ->|
| 1 tq |                                                              ^              |
                                                                sample point (87.5%)
```

| Segment | Duration | Purpose |
|---|---|---|
| Sync | Always 1 tq (the "1 +" in the formula) | Where a bit edge is expected. Nodes compare edges against it to stay in step |
| Seg1 | Configurable | Time before the sample point. Lets the signal travel across the bus and settle before it is read |
| Seg2 | Configurable | Time after the sample point. Leaves room to process the bit and to shorten the bit when resynchronizing |

#### 4. Putting it together

```
bit time = tq × (1 + Seg1 + Seg2) = (prescaler ÷ CAN clock) × (1 + Seg1 + Seg2)
bit rate = 1 ÷ bit time           = CAN clock ÷ (prescaler × (1 + Seg1 + Seg2))
```

| | NUCLEO-F429ZI | NUCLEO-H723ZG |
|---|---|---|
| 1 tq | 133.3 ns | 125 ns |
| tq per bit (1 + Seg1 + Seg2) | 1 + 12 + 2 = 15 | 1 + 13 + 2 = 16 |
| Bit time | 15 × 133.3 ns = 2 µs | 16 × 125 ns = 2 µs |
| Bit rate | 1 ÷ 2 µs = 500 kbps | 1 ÷ 2 µs = 500 kbps |

The two boards use different clocks, prescalers and tq counts, but arrive at the same 2 µs bit time. That is the only thing nodes on a bus must agree on.

#### Sample point

Sample point = (1 + Seg1) ÷ (1 + Seg1 + Seg2)

The bus is read at the end of Seg1. A longer Seg1 moves the sample point later; 87.5% is the CiA-recommended value for rates up to 800 kbps. The CAN spec splits Seg1 into Prop_Seg and Phase_Seg1; STM32 combines them into one setting.

#### SJW

**SJW (synchronization jump width)** is the most the controller may adjust one bit, in tq, to stay in step with the other nodes. When an edge arrives late, it lengthens Seg1 by up to SJW; when it arrives early, it shortens Seg2 by up to SJW. SJW must not exceed Seg2. A larger SJW tolerates more clock mismatch between nodes. With SJW 1 tq the two nodes' clocks may differ by about 0.3%; raising SJW to 2 (the Seg2 limit) allows about 0.6%. 1 tq assumed both boards' HSE clocks are crystal-accurate. That turned out not to hold for the H723: see the open issue in step 7.

#### Settings

| Setting | NUCLEO-F429ZI (bxCAN) | NUCLEO-H723ZG (FDCAN) |
|---|---|---|
| CAN clock | APB1 = 45 MHz (180 MHz system clock) | FDCAN kernel clock = 80 MHz |
| Prescaler | 6 | 10 |
| Seg1 | 12 tq (BS1) | 13 tq (NominalTimeSeg1) |
| Seg2 | 2 tq (BS2) | 2 tq (NominalTimeSeg2) |
| SJW | 1 tq | 1 tq |
| Time quanta per bit | 15 | 16 |
| Sample point | 86.7% | 87.5% |

FDCAN notes for the H723ZG:

- Set `FrameFormat = FDCAN_FRAME_CLASSIC` so it can talk to the F429's bxCAN. Don't send CAN FD frames on this bus.

#### FDCAN message RAM

**Why it must be set:** bxCAN has fixed hardware mailboxes and FIFOs. FDCAN has none; it stores filters and messages in a shared message RAM that `HAL_FDCAN_Init()` divides up using the sizes below. A size of 0 means that storage doesn't exist, so nothing can be sent, received or filtered.

| Setting | What it sets | Value | Why this value |
|---|---|---|---|
| `StdFiltersNbr` | Number of standard (11-bit) ID filters | 1 | Project 1 accepts one ID range from the F429 |
| `RxFifo0ElmtsNbr` | Receive FIFO slots | 8 | Holds a burst of about 2 ms of back-to-back frames at 500 kbps, so none are lost while the CPU is busy |
| `RxFifo0ElmtSize` | Data bytes per RX slot | `FDCAN_DATA_BYTES_8` | Classic CAN payloads are at most 8 bytes; larger sizes are only for CAN FD |
| `TxFifoQueueElmtsNbr` | Transmit queue slots | 8 | Lets the application queue several frames without waiting for each to send |
| `TxElmtSize` | Data bytes per TX slot | `FDCAN_DATA_BYTES_8` | Same as RX: 8 bytes is the classic CAN maximum |

These take about 260 bytes (roughly 2.5% of the message RAM), leaving room for more filters or FDCAN2/3 later.

### Bring-up

Each step adds one new thing, so a failure points to one cause.

#### Step 1: Hardware check ✅

Done before powering the boards, so wiring faults are found with a multimeter instead of showing up later as software bugs.

| Check | Result | Why |
|---|---|---|
| Transceiver power on 3V3, not 5V | ✅ | The SN65HVD230 is a 3.3 V part, and its CRX output drives the STM32's PD0 directly. This is the one wiring mistake that could damage hardware |
| CTX on PD1, CRX on PD0 (both boards) | ✅ | These are the CAN1/FDCAN1 TX/RX pins (AF9). Wrong pins mean frames never reach the bus |
| CANH to CANL resistance, power off | ✅ 60 Ω | Two 120 Ω terminators in parallel. Confirms both are fitted and the CANH/CANL wires connect the boards (120 Ω would mean a missing terminator or broken wire) |
| Shared ground | ✅ via USB | Both Nucleos plug into the same laptop, so their grounds are connected without a separate GND wire |

#### Step 2: HAL libraries and build setup ✅

Needed before any firmware can be built. ST's HAL provides the peripheral drivers (`HAL_CAN_*`, `HAL_UART_*`), and CMSIS provides the register definitions and startup code.

The libraries are git submodules in `lib/`, pinned to fixed releases, so every clone builds against the same code without installing STM32CubeIDE:

| Submodule | Version | Provides |
|---|---|---|
| `lib/stm32f4xx-hal-driver` | v1.8.6 | F4 HAL drivers |
| `lib/cmsis-device-f4` | v2.6.12 | F4 register definitions, startup code, `system_stm32f4xx.c` |
| `lib/cmsis_core` | v5.9.0 | Cortex-M core headers |

`f429/` is a Makefile project with a linker script, a HAL config that enables only the modules in use, and a minimal `main.c` that sets the clocks used for CAN (180 MHz system clock, 45 MHz APB1) and blinks LD1. Done when it compiles; the CAN code is added in step 3.

Verified on hardware: flashed with `make flash`, and LD1 blinks at 1 Hz, confirming the clock configuration runs.

Build and flash:

```bash
git clone --recurse-submodules https://github.com/noaros/CAN-Bus.git
cd CAN-Bus/f429
make
make flash
```

`make flash` runs `tools/flash.sh`, which uses `STM32_Programmer_CLI` over the ST-LINK. With both Nucleos plugged in there are two ST-LINKs, so it flashes only the one whose target reports the board's device ID (`0x419` for the F429, `0x483` for the H723) and stops with an error if none does. Requires `arm-none-eabi-gcc`, GNU Make and STM32CubeProgrammer.

#### Step 3: F429 bxCAN in loopback

Tests the F429's CAN setup on its own, before the H723 is involved. In loopback mode the controller feeds every transmitted frame back to its own receiver, so a failure here is in the F429 code, not the wiring or the other board.

What it adds to `f429/Src/main.c`:

| Part | Setting | Why |
|---|---|---|
| CAN1 init | Prescaler 6, BS1 12 tq, BS2 2 tq, SJW 1 tq | 500 kbps from the 45 MHz APB1 clock (see Settings above) |
| Mode | `CAN_MODE_LOOPBACK` | Frames come back without a second node, and missing ACKs are ignored. TX is still driven on PD1, so frames are visible on a scope |
| Filter | Bank 0, 32-bit mask mode, mask 0 → FIFO0 | bxCAN discards all received frames until a filter is active. Mask 0 accepts every ID, so the filter can't be the cause if nothing arrives. Narrowed once the bus works |
| RX | FIFO0 message pending interrupt | Project 1 uses interrupt-driven RX. The callback checks the ID, DLC (data length code: the frame's 4-bit field giving the number of data bytes, 0–8 in classic CAN) and payload against the frame just sent |
| Errors | Error, bus-off and last-error-code interrupts | Any CAN error stops the test with the red LED on |

Every 500 ms it sends ID `0x123` with 8 bytes: a counter in bytes 0–3 and `CA FE BA BE` in bytes 4–7.

| LED | Meaning |
|---|---|
| LD2 blue | Toggles on every frame sent |
| LD1 green | Toggles on every matching frame received |
| LD3 red | On for good after a mismatch, CAN error or failed HAL call (`hcan1.ErrorCode` says which) |

Done when blue and green both blink at 1 Hz and red stays off. If blue blinks but green doesn't, the frame is sent but not received, so check the filter and RX interrupt first.

Verified on hardware: flashed with `make flash`. Read over SWD while running, the TX counter rose by one every 500 ms, the green and blue LEDs toggled together with red off, CAN1's error status register (ESR: the transmit and receive error counters, the last error code and the bus-off flag) read 0 (no errors, both error counters 0), and its bit timing register (BTR: the prescaler, Seg1, Seg2 and SJW, each stored as value − 1, plus the loopback and silent mode bits) read `0x401B0005` (loopback on, prescaler 6, BS1 12 tq, BS2 2 tq, SJW 1 tq).

#### Step 4: H723 HAL libraries and build setup

The same as step 2, for the NUCLEO-H723ZG. Two more submodules, pinned like the F4 ones (`lib/cmsis_core` is shared):

| Submodule | Version | Provides |
|---|---|---|
| `lib/stm32h7xx-hal-driver` | v1.11.7 | H7 HAL drivers, including FDCAN |
| `lib/cmsis-device-h7` | v1.10.8 | H7 register definitions, startup code, `system_stm32h7xx.c` |

`h723/` mirrors `f429/`: a Makefile, a linker script, a HAL config that enables only the modules in use, and a minimal `main.c` that sets the clocks and blinks LD1.

Clocks, from the 8 MHz ST-LINK clock (HSE bypass) through PLL1: 8 MHz ÷ 2 (M) = 4 MHz, × 200 (N) = 800 MHz VCO.

| Output | Divider | Frequency | Used for |
|---|---|---|---|
| PLL1P | /2 | 400 MHz | System clock (CPU) |
| PLL1Q | /10 | 80 MHz | FDCAN kernel clock, selected by the FDCANSEL mux. Sets the bit timing (see Settings above) |
| AHB / APB1–4 | /2, then /2 | 200 / 100 MHz | Buses. FDCAN registers are on APB1 |

H7 differences from the F429 setup:

- **Power supply:** the H723 has no SMPS (switched-mode power supply: an on-chip regulator that switches rapidly to step the voltage down efficiently, found on some H7 parts), so the core runs from the internal LDO (low-dropout regulator: a simpler linear regulator that burns the excess voltage as heat). This is set with `HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY)`, and `USE_PWR_LDO_SUPPLY` for the startup code
- **Voltage scale 0:** allows up to 550 MHz, so 400 MHz has margin. Flash needs 2 wait states at 200 MHz AXI
- **RAM:** data, heap and stack go in the 128 KB DTCM (data tightly coupled memory: RAM wired directly to the CPU core, single-cycle and uncached) at `0x20000000`, the fastest RAM for the CPU. Because it isn't on the bus matrix, the DMA controllers DMA1 and DMA2 (direct memory access: hardware that copies data between peripherals and memory without the CPU) can't reach it. That doesn't matter here: FDCAN doesn't use DMA, it moves frames into its own message RAM by itself. A DMA buffer added later (for example for UART) must go in AXI SRAM (`RAM_D1`) or D2 SRAM (`RAM_D2`) instead

To check the clocks without a scope, `main.c` stores the frequencies the HAL computes from the RCC registers in `clk_sys_hz` and `clk_fdcan_hz`, readable over SWD. Done when LD1 blinks at 1 Hz, `clk_sys_hz` is 400000000 and `clk_fdcan_hz` is 80000000.

Verified on hardware: flashed with `make flash`, and LD1 blinks at 1 Hz with LD3 off. Read over SWD: `clk_sys_hz` = 400000000, `clk_fdcan_hz` = 80000000, FDCANSEL = `01` (PLL1Q), and the HAL tick (`uwTick`: the millisecond counter behind `HAL_Delay()`, incremented by the SysTick interrupt, which fires every 400,000 CPU cycles) advanced 5102 over a 5 s wait plus read time (roughly 5.1 s of wall time: real elapsed time). So the CPU runs at about the 400 MHz the HAL assumes. This is a coarse check, accurate to a few percent: it rules out a wrong clock source or divider, but not a small error. `clk_sys_hz` and `clk_fdcan_hz` don't catch that either, because the HAL computes them from the register settings assuming HSE is exactly `HSE_VALUE` (8 MHz). A later, precise measurement found the H723 running 1.23% fast (see step 7).

```bash
cd h723
make
make flash
```

#### Step 5: H723 FDCAN in loopback

The same test as step 3, on the H723's FDCAN. External loopback mode feeds every transmitted frame back to the controller's own receiver and still drives TX on PD1, so a failure here is in the H723 code, not the wiring or the F429.

What it adds to `h723/Src/main.c`:

| Part | Setting | Why |
|---|---|---|
| FDCAN1 init | `FDCAN_FRAME_CLASSIC`, NominalPrescaler 10, Seg1 13 tq, Seg2 2 tq, SJW 1 tq | Classic CAN at 500 kbps from the 80 MHz kernel clock (see Settings above). The data-phase settings only apply to CAN FD, so they're set to the minimum valid values |
| Mode | `FDCAN_MODE_EXTERNAL_LOOPBACK` | The FDCAN equivalent of bxCAN's loopback: frames come back without a second node, and missing ACKs are ignored |
| Message RAM | Sizes from the FDCAN message RAM table above | 1 standard filter, 8-slot Rx FIFO 0, 8-slot TX queue, 8 data bytes each |
| Filter | Standard ID, mask mode, ID 0, mask 0 → Rx FIFO 0 | Mask 0 accepts every ID, as in step 3 |
| Global filter | Reject non-matching standard, extended and remote frames | Unlike bxCAN, FDCAN accepts frames that match no filter into Rx FIFO 0 by default. Rejecting them means frames only get in through the filter above, so the test checks the filter too |
| RX | Rx FIFO 0 new message interrupt, on interrupt line 0 (`FDCAN1_IT0_IRQHandler`) | The callback checks the ID, frame type, classic format, DLC and payload against the frame just sent |
| Errors | Rx FIFO 0 message lost, protocol error, error warning, error passive and bus-off interrupts | Any of them stops the test with the red LED on |

It sends the same frame as the F429: ID `0x123`, 8 bytes, a counter in bytes 0–3 and `CA FE BA BE` in bytes 4–7.

| LED | Meaning |
|---|---|
| LD2 yellow | Toggles on every frame sent |
| LD1 green | Toggles on every matching frame received |
| LD3 red | On for good after a mismatch, CAN error or failed HAL call (`hfdcan1.ErrorCode` says which) |

For checking over SWD, `tx_count` and `rx_ok_count` count frames sent and frames received that matched. Done when yellow and green both blink at 1 Hz, red stays off, and `rx_ok_count` keeps up with `tx_count`.

Verified on hardware: flashed with `make flash`. Read over SWD while running, `tx_count` and `rx_ok_count` stayed equal (17/17, then 23/23 three seconds later), the green and yellow LEDs toggled with red off, and the FDCAN registers matched the setup:

| Register | Read | Meaning |
|---|---|---|
| CCCR (CC control register: operating mode switches) | `0x1080` | Running (INIT 0), test mode on, not silent (MON 0), classic CAN only (FDOE 0), automatic retransmission on (DAR 0) |
| TEST (test register: loopback and TX/RX pin control) | `0x90` | Loopback on (LBCK 1); RX pin reads recessive |
| NBTP (nominal bit timing and prescaler, FDCAN's BTR; each field stored as value − 1) | `0x00090C01` | Prescaler 10, Seg1 13 tq, Seg2 2 tq, SJW 1 tq |
| ECR (error counter register) | `0x0` | Transmit and receive error counters 0 |
| PSR (protocol status register: last error code, error warning, error passive, bus-off) | `0x708` | No errors, idle between frames, not error warning, error passive or bus-off |
| GFC (global filter configuration) | `0x2B` | Non-matching standard, extended and remote frames rejected |

#### Step 6: First exchange over the real bus

Both boards switch from loopback to normal mode, so frames now go through the transceivers, the CANH/CANL wires and the terminators, and each board must understand the other's bit timing. This is the first test of the wiring and of the two different clock setups working together.

Each board sends one frame every 500 ms and checks the other's:

| | Sends ID | Receives ID |
|---|---|---|
| F429 | `0x100` | `0x200` |
| H723 | `0x200` | `0x100` |

The IDs and payload layout live in `common/project1_frames.h`, which both builds include, so the boards can't disagree about them. The payload is the same as before: a counter in bytes 0–3 and `CA FE BA BE` in bytes 4–7. A received frame can't be compared with one just sent any more, since it comes from the other board. Instead the receiver checks the ID, DLC and pattern, and that the counter goes up by exactly 1, which shows lost frames.

**Errors no longer stop the program.** On a real bus some errors are normal. While one board is missing or still starting up, nobody sends the ACK (acknowledge: the bit every receiving node sets to confirm it got a frame). The sender then gets an ACK error on every attempt and keeps retrying, and its TEC (transmit error counter) rises by 8 each time until it reaches error passive (TEC or REC, the receive error counter, at 128 or more: the node still works but signals errors less forcefully). Once the other board is back, each successful frame takes 1 off the counter, and the node returns to normal by itself. REC works the same way for receiving: +1 for most errors seen while receiving a frame, −1 for each frame received cleanly. Only TEC can cause bus-off, since a node with a high REC may just be listening to a noisy bus, while one with a high TEC is likely the cause. Halting on the first error, as steps 3 and 5 did, would stop a board just because the other was flashed second. So now:

- Bus errors are polled every 500 ms and kept in `bus` (TEC, REC, last error code, error passive, bus-off) for reading over SWD
- If a board reaches bus-off (TEC above 255: the node disconnects itself from the bus), it rejoins after the bus has been idle for 128 × 11 bits. bxCAN does this by itself (`AutoBusOff`); FDCAN needs software to clear `CCCR.INIT`, which `update_bus_status()` does
- When the TX mailboxes (F429) or TX queue (H723) are full because nobody acknowledges, sends are skipped and counted in `tx_skipped`
- Only a failed HAL setup call still stops everything

The filters stay accept-all. Narrowing them is a separate step, so a failure here points at the bus.

| LED | F429 | H723 | Meaning |
|---|---|---|---|
| TX | LD2 blue | LD2 yellow | Toggles on every frame queued for sending |
| RX | LD1 green | LD1 green | Toggles on every good frame from the other board |
| Problem | LD3 red | LD3 red | On while error passive or bus-off, or no good frame from the other board in the last 1.5 s. Goes off by itself once the bus recovers |

Done when both boards' TX and green LEDs blink at 1 Hz, red stays off, each board's `rx.ok` matches the other's `tx_count`, and `rx.lost`, `rx.bad` and the error counters stay at 0.

Verified on hardware: flashed both with `make flash`, then read both over SWD:

| | F429 | H723 |
|---|---|---|
| Frames sent (`tx_count`) | 177 | 181 |
| Good frames from the other board (`rx.ok`) | 180 | 177 |
| Lost / restarts | 0 / 0 | 0 / 0 |
| TEC / REC | 0 / 0 | 0 / 0 |
| Error passive / bus-off | no / no | no / no |
| Red LED | off | off |

The H723 received exactly the 177 frames the F429 sent. The F429 shows 180 of the H723's 181 because the values are read one at a time over SWD, not at the same moment: the F429's `rx.ok` was read before the H723's `tx_count`, and the H723 sent one more frame in between. Two leftovers from the flashing order, both expected:

- The H723 counted 1 bad frame: between the two flashes, the F429 was still running its step 3 loopback firmware, which drives TX and sent one `0x123` frame. Step 6 doesn't expect that ID
- The H723's TEC was 112 right after both were flashed: during that same window nobody acknowledged its frames, since a node in loopback mode ignores the bus. It fell by 1 per frame and reached 0 within about a minute

#### Step 7: Hardware acceptance filtering

Each board's filter goes from accept-all to only the other board's ID, so the CAN hardware drops every other frame before the CPU sees it. This is the "hardware acceptance filtering" in the Project 1 goal.

| | Accepts | Filter setting |
|---|---|---|
| F429 | `0x200` only | Bank 0, 32-bit ID/mask: ID `0x200 << 21`, mask `0x7FF << 21` plus the IDE and RTR bits, so only standard data frames with exactly ID `0x200` pass. In 32-bit scale the registers hold the identifier field as bxCAN stores it (standard ID in bits 31:21, IDE bit 2, RTR bit 1), hence the shift |
| H723 | `0x100` only | Standard filter element 0, classic ID/mask: ID `0x100`, mask `0x7FF`, as plain 11-bit values with no shift. The global filter still rejects non-matching and remote frames |

**Testing a filter needs frames it should reject.** With only the two real IDs on the bus, a filter that still accepted everything would look the same as a working one. So each board also sends a decoy right after its real frame, with an ID one bit off the real one: `0x101` from the F429 and `0x201` from the H723. One bit off also catches a mask that ignores low ID bits. A decoy that reached the other board's software would be counted in `rx.bad`.

A rejected frame is still acknowledged: every node that receives a frame correctly sends the ACK bit, before and regardless of filtering. So the decoys cause no ACK errors.

**Control build:** `make FILTER=all` keeps the accept-all filter (it defines `P1_FILTER_ACCEPT_ALL`), to show that the decoys do reach the receiver when the filter lets them. A stamp file in `build/` makes `main.c` rebuild when `FILTER` changes.

Both builds also count bus errors by type in `errs`, from the controller's last error code (LEC). bxCAN and FDCAN use the same codes: 1 stuff (6 equal bits in a row, where the sender must have inserted an opposite stuff bit), 2 form (a fixed-format bit such as a delimiter was wrong), 3 ACK, 4 bit 1 (sent recessive, read dominant), 5 bit 0 (sent dominant, read recessive), 6 CRC (received CRC didn't match the frame).

Verified on hardware, reading both boards over SWD:

| Run | Decoys sent by the other board | `rx.bad` | `rx.ok` |
|---|---|---|---|
| Control (`FILTER=all`), F429 receiving | 8 → 19 (+11) | 6 → 17 (+11) | Kept up with the H723's frames |
| Control (`FILTER=all`), H723 receiving | 8 → 19 (+11) | 8 → 19 (+11) | Kept up with the F429's frames |
| Filtered, F429 receiving | about 128 | **0** | 130, matching the H723's 128 sent (read at different moments) |
| Filtered, H723 receiving | about 128 | **0** | 129, matching the F429's 128 sent |

Filter registers read back as intended: F429 bank 0 FR1 `0x40000000` (ID), FR2 `0xFFE00006` (mask); H723 filter element 0 in message RAM `0x890007FF` (classic mask filter to Rx FIFO 0, ID `0x100`, mask `0x7FF`).

**Open issue: bus errors, traced to the H723's clock running 1.23% fast.** Over 2 minutes (about 1,000 frames on the bus) the error counters showed:

| | Stuff | Form | ACK | Bit 1 | Bit 0 | CRC |
|---|---|---|---|---|---|---|
| F429 | 1 | 0 | 0 | 0 | 0 | 3 |
| H723 | 0 | 0 | 3 | 1 | 0 | 0 |

These are one fault seen from both ends: errors on frames sent by the H723. The F429 sees stuff and CRC errors; the H723 sees the matching bit 1 errors (an error flag overriding its recessive bits) and ACK errors (no acknowledgement for a frame the F429 found faulty). CAN retransmits them automatically, so no frames were lost (`rx.lost` 0), but a two-node bench bus should have essentially none. It isn't caused by the filters, which act only after a frame has been received correctly, and probably existed in step 6 unnoticed, since a few errors are counted back down within seconds.

Reseating the wires didn't help. A 15-minute run afterwards showed the same pattern in bursts: 2, then 32, then 9 errors per 5 minutes.

The cause showed up in the frame counts. Both boards send once per `HAL_Delay(500)` loop, yet over the same 902 s the F429 sent 1799 frames (501.4 ms per loop, as expected) and the H723 sent 1821 (495.3 ms). Reading both boards' `uwTick` over 240 s of wall time confirmed it:

| | Ticks | vs wall time |
|---|---|---|
| F429 | 240,106 | +0.00% |
| H723 | 243,051 | **+1.23%** |

The H723's clock registers are as configured (HSE bypass → PLL1 ÷2 ×200; FDCANSEL = PLL1Q), so the error is in the HSE input itself: about 8.098 MHz instead of 8.000. On this board HSE comes from the on-board STLINK-V3's clock output, not a crystal next to the H723. Every H723 clock inherits the error, so its FDCAN clock is about 80.98 MHz and it sends at about 506 kbps instead of 500. With SJW 1 tq the nodes tolerate only about 0.3% mismatch, so frames survive or fail depending on their bit pattern: payloads with long runs of equal bits give the receiver fewer edges to resynchronize on. That fits the bursts, as the counter in the payload moves through different bit patterns. Why the errors show up only on H723 → F429 frames and not the other way is not explained yet.

Still to decide: how to give the H723 an accurate clock.
