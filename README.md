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

**SJW (synchronization jump width)** is the most the controller may adjust one bit, in tq, to stay in step with the other nodes. When an edge arrives late, it lengthens Seg1 by up to SJW; when it arrives early, it shortens Seg2 by up to SJW. SJW must not exceed Seg2. A larger SJW tolerates more clock mismatch between nodes. Both clocks here come from crystals, so 1 tq is enough; raising it to 2 (the Seg2 limit) is a first step if the bus shows intermittent errors.

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
