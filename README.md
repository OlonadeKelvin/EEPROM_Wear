# Hardware EEPROM Wear‑Leveling Controller

A tiny, formally verified, attack-resistant wear-leveling controller that dramatically extends the lifetime of external EEPROM and flash memories in embedded and edge systems.

![TinyTapeout](https://img.shields.io/badge/TinyTapeout-SKY130-8A2BE2)
![Cells](https://img.shields.io/badge/Size-~350--450_cells-00B140)
![Verified](https://img.shields.io/badge/Verified-SVA_FORMAL-2E8B57)
![Attack Resistant](https://img.shields.io/badge/Attack_Resistant-Feistel-FF4500)


## Overview

EEPROM and flash cells typically endure only **10k–100k** program/erase cycles. Without wear leveling, hot data quickly destroys individual blocks while the rest of the array remains fresh.

This IP implements the **Start-Gap** algorithm, the same technique used in commercial 3D-XPoint and phase-change memories, in a highly optimized hardware design that fits inside a single **TinyTapeout SKY130 1×1 tile** (~350–450 standard cells).

It adds enhancements: Feistel-based address randomization for attack resistance, Hamming ECC protection on persistent state, automatic bad-block retirement, saturating wear counters, and rich telemetry — all while remaining extremely compact and power-fail safe.


## Key Features

- **Start-Gap Wear Leveling** (Qureshi et al., MICRO 2009) — no large mapping table

- **Attack Resistance** — 2-round 3-bit Feistel scrambler keyed by LFSR

- **ECC Protection** — Hamming(6,3) SEC on all persistent state

- **Bad-Block Retirement** — automatic detection and skipping of worn-out blocks

- **Saturating 8-bit Wear Counters** — 2× area savings vs 16-bit linear

- **In-Band Telemetry** — real-time max-min skew and total write count

- **Formal Verification** — full SVA suite (monotonicity, bounded skew, liveness, range safety)

- **Power-Fail Safe** — atomic shadow-register commit

- **Tiny Footprint** — fits comfortably in one TinyTapeout tile


## Why Start-Gap?


Traditional table-based wear leveling requires large mapping RAMs, min-search trees, and reverse-lookup logic. Start-Gap eliminates all of that with just two registers (`Start` and `Gap`), providing a **provable wear bound** of `ψ + 1` (where `ψ` is the rotation period) regardless of access pattern.


## Architecture


### Start-Gap Algorithm


```verilog

phys = (logical + Start) mod N          if logical < Gap

phys = (logical + Start + 1) mod N      otherwise

```


Every `ψ` writes, `Gap` advances, smoothly rotating the "hot spot" across the array.


### Attack Resistance


A plain Start-Gap implementation is vulnerable to targeted wear attacks. This design adds a lightweight **2-round Feistel cipher** (Seznec, IEEE CAL 2010) on the logical address before feeding it into Start-Gap. The round key comes from a 10-bit LFSR, making targeted wear-out computationally infeasible for embedded adversaries.


### Reliability Features


- **Hamming(6,3) ECC** on `Start` and `Gap` — single-bit correction with error flag

- **Bad-block retirement** — saturated blocks are automatically retired and skipped

- **Atomic commit** of persistent state for power-loss tolerance


## Interface (TinyTapeout)


### Inputs (`ui_in`)


| Bits    | Signal      | Description                                      |

|---------|-------------|--------------------------------------------------|

| [2:0]   | `logical`   | Logical block address (0–7)                      |

| [4:3]   | `cmd`       | `00`=read_req, `01`=write_req, `10`=write_commit, `11`=telem_req |

| [5]     | `move_ack`  | Acknowledge block migration                      |

| [7:6]   | `telem_sel` | Select telemetry output                          |


### Outputs (`uo_out`)


| Bits | Signal         | Description                              |

|------|----------------|------------------------------------------|

| [2:0]| `phys`         | Physical address                         |

| [3]  | `busy`         | Operation in progress                    |

| [4]  | `move_req`     | Request data migration                   |

| [5]  | `ecc_error`    | Single-bit correction occurred           |

| [6]  | `block_retired`| Current block has been retired           |

| [7]  | `telem_valid`  | Telemetry data on `uio_out` is valid     |


**Bidirectional (`uio_out`)**: Carries destination address during moves and telemetry data.


## Protocol Summary


- **Read/Write**: Issue command → receive physical address in next cycle

- **Write Commit**: Triggers wear counter update, possible rotation, and retirement logic

- **Move Handshake**: When a block is retired, `move_req` is asserted with source/destination addresses

- **Telemetry**: Request skew or total write count (16-bit) on demand


## Area (SKY130)


| Component                  | Baseline Table | This Design      |

|----------------------------|----------------|------------------|

| Mapping + Min Search       | ~200 cells     | 0                |

| Wear Counters              | 128 FF         | 64 FF (8-bit)    |

| Start/Gap + Logic          | —              | ~90 cells        |

| Feistel + LFSR             | —              | ~40 cells        |

| ECC + Retired Flags        | —              | ~40 cells        |

| **Total**                  | ~500 cells     | **~350–450**     |


## Formal Verification


All critical properties are proven with SystemVerilog Assertions and SymbiYosys:


- Bounded wear skew (`max - min ≤ ψ + 1`)

- Monotonicity of write counters

- Range safety and liveness

- No deadlock in move handshake


Run with:

```bash

cd formal

sby -f wearlevel.sby

```


## Integration


The module is fully synthesizable Verilog-2001 and highly parameterizable (`NUM_BLOCKS`, `PSI`, address width, etc.). It has been physically implemented and validated on the SKY130 PDK via Tiny Tapeout.


**Ideal for:**

- IoT and edge sensor nodes

- Industrial data loggers

- Battery-powered devices

- Any system using serial EEPROM or small flash arrays


## References


- M. K. Qureshi et al., "Enhancing Lifetime and Security of PCM-Based Main Memory with Start-Gap Wear Leveling," MICRO 2009.

- A. Seznec, "A Phase Change Memory as a Secure Main Memory," IEEE Computer Architecture Letters, 2010.


## License


**Apache 2.0** — Fully open source.


---


**Built for reliable, long-life embedded systems.**


