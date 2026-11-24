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

![Project 1 wiring: NUCLEO-F429ZI (top) and NUCLEO-H723ZG (bottom), each wired to a Waveshare SN65HVD230 transceiver on the breadboard](images/project1-wiring.jpg)

### Bit Timing (500 kbps)

Bit rate = CAN clock ÷ (prescaler × (1 + Seg1 + Seg2)). Sample point = (1 + Seg1) ÷ (1 + Seg1 + Seg2).

The CAN clock is the clock feeding the CAN peripheral (see the "CAN clock" row in the table below), not the CPU core clock:

- **F429ZI:** the APB1 peripheral clock (PCLK1).
- **H723ZG:** the FDCAN kernel clock, selected by the FDCANSEL mux (HSE, PLL1Q or PLL2Q). FDCAN also uses APB1 for register access, but bit timing comes only from the kernel clock.

How the formula works:

- **Prescaler** divides the CAN clock into time quanta (tq), the smallest unit of time inside a bit. One tq = prescaler ÷ CAN clock (125 ns on the H723: 10 ÷ 80 MHz).
- **Each bit is a whole number of tq**, split into three segments:

```
|<------------------------- one bit = 16 tq = 2 µs (H723) ------------------------->|
| Sync |<----------------------- Seg1 (13 tq) ----------------------->|<- Seg2 (2) ->|
| 1 tq |                                                              ^              |
                                                                sample point (87.5%)
```

| Segment | Length | Purpose |
|---|---|---|
| Sync | Always 1 tq (the "1 +" in the formula) | Where a bit edge is expected. Nodes compare edges against it to stay in step |
| Seg1 | Configurable | Time before the sample point. Lets the signal travel across the bus and settle before it is read |
| Seg2 | Configurable | Time after the sample point. Leaves room to process the bit and to shorten the bit when resynchronizing |

The bus is read at the end of Seg1. A longer Seg1 moves the sample point later; 87.5% is the CiA-recommended value for rates up to 800 kbps. The CAN spec splits Seg1 into Prop_Seg and Phase_Seg1; STM32 combines them into one setting.

**SJW (synchronization jump width)** is the most the controller may adjust one bit, in tq, to stay in step with the other nodes. When an edge arrives late, it lengthens Seg1 by up to SJW; when it arrives early, it shortens Seg2 by up to SJW. SJW must not exceed Seg2. A larger SJW tolerates more clock mismatch between nodes. Both clocks here come from crystals, so 1 tq is enough; raising it to 2 (the Seg2 limit) is a first step if the bus shows intermittent errors.

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
- Set nonzero message RAM sizes (`StdFiltersNbr`, `RxFifo0ElmtsNbr`, `TxFifoQueueElmtsNbr` ≥ 1, with 8-byte data fields). If they are left at zero, the peripheral will not send or receive.
