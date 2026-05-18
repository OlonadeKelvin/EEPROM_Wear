# Hardware EEPROM Wear‑Leveling Controller

A compact, attack‑resistant wear‑leveling controller that dramatically extends the lifetime of external EEPROM and flash memories in embedded systems.  
Implements the **Start‑Gap** algorithm with a lightweight **Feistel address scrambler**, saturating wear counters, automatic bad‑block retirement, and in‑band telemetry.

![TinyTapeout](https://img.shields.io/badge/TinyTapeout-SKY130-8A2BE2)
![Cells](https://img.shields.io/badge/Size-~350--450_cells-00B140)
![Verified](https://img.shields.io/badge/Verified-Simulation-2E8B57)
![Attack Resistant](https://img.shields.io/badge/Attack_Resistant-Feistel-FF4500)

---

## Overview

EEPROM and flash cells typically endure only **10k–100k** program/erase cycles. Without wear leveling, frequently written logical blocks quickly destroy a few physical blocks while the rest of the array remains fresh.

This IP implements the proven **Start‑Gap** algorithm (used in commercial 3D‑XPoint and phase‑change memories) in a highly optimized hardware design that fits inside a single **TinyTapeout SKY130 1×1 tile** (~350–450 standard cells).  

Key enhancements:
- **2‑round Feistel address scrambler** with an LFSR key – defeats targeted wear attacks.
- **Automatic bad‑block retirement** – saturated blocks are marked and skipped.
- **Saturating 8‑bit wear counters** per logical block – small and efficient.
- **In‑band telemetry** – real‑time max/min skew and total write count.
- **Atomic state update** – safe for power loss.
- **Rich test suite** – passed all cocotb tests on the SKY26b shuttle.

> **Note:** The original Hamming(6,3) ECC was removed to guarantee successful physical placement on the SKY130 PDK. The design remains functionally correct and passes all simulation tests.

---

## How It Works (Short Version)

1. **Read/Write commands** are issued together with a logical block address (0‑7).
2. The **Feistel scrambler** randomises the logical address using an LFSR key.
3. The **Start‑Gap mapping** converts the scrambled address to a physical block:
   ```
   phys = (scrambled + Start) mod N          if scrambled < Gap
   phys = (scrambled + Start + 1) mod N      otherwise
   ```
4. **Writes** increment the logical block's wear counter.
5. Every **PSI = 8** writes, the `Gap` pointer advances (and `Start` when `Gap` wraps), rotating the "hot spot".
6. If a wear counter **saturates** (reaches 255), the physical block that currently holds that logical block is **retired** and a `move_req` handshake asks an external agent to copy data.
7. **Telemetry** queries return the current max‑min skew (wear distribution) or the total write count (20 bits).

---

## Interface (TinyTapeout SKY130)

### Inputs (`ui_in`)

| Bits   | Signal      | Description                                      |
|--------|-------------|--------------------------------------------------|
| [2:0]  | `logical`   | Logical block address (0–7)                     |
| [4:3]  | `cmd`       | `00`=read_req, `01`=write_req, `11`=telem_req   |
| [5]    | `move_ack`  | Acknowledge block migration (handshake)         |
| [7:6]  | `telem_sel` | Select telemetry output (see below)             |

### Outputs (`uo_out`)

| Bit | Signal         | Description                              |
|-----|----------------|------------------------------------------|
| 2:0 | `phys`         | Physical block address (0–7)            |
| 3   | `busy`         | Controller busy (do not issue new commands) |
| 4   | `move_req`     | Request data migration (destination on `uio_out[2:0]`) |
| 5   | `ecc_error`    | Always 0 (ECC removed)                  |
| 6   | `block_retired`| The currently accessed physical block is retired |
| 7   | `telem_valid`  | Telemetry data on `uio_out` is valid    |

### Bidirectional (`uio_out`)

- During `move_req`: `uio_out[2:0]` = destination physical block (to be used by external agent).
- During telemetry: `uio_out[7:0]` = telemetry data (valid when `telem_valid=1`).

---

## Telemetry Selection (`telem_sel`)

| `telem_sel` | Value                               |
|-------------|-------------------------------------|
| 00          | Skew = max(wear counter) – min(wear counter) (0–255) |
| 01          | Total write count, low 8 bits      |
| 10          | Total write count, bits 15–8       |
| 11          | Total write count, bits 19–16 (top 4 bits) |

The total write counter is 20 bits and can be reconstructed by reading three telemetry requests.

---

## Usage Example (Pseudocode)

### Write a block
```
1. Wait until `busy` == 0.
2. Set `ui_in` = (cmd=01, logical address).
3. Wait one clock cycle, then clear `ui_in`.
4. Poll `busy` until 0.
5. If `move_req` == 1 during the operation:
   - Read `uio_out[2:0]` as the destination block.
   - Copy data from the current physical block to that destination.
   - Set `ui_in[5] = 1` (move_ack) for one clock cycle.
   - Wait for `move_req` to clear.
6. At the end, `phys` on `uo_out[2:0]` holds the physical block used.
```

### Read a block
```
1. Wait until `busy` == 0.
2. Set `ui_in` = (cmd=00, logical address).
3. Read `phys` from `uo_out[2:0]` immediately (next clock edge).
4. No busy is asserted for reads.
```

### Query telemetry (e.g., skew)
```
1. Wait until `busy` == 0.
2. Set `ui_in` = (cmd=11, telem_sel=00).
3. Wait 2 clock cycles.
4. Sample `telem_valid` and `uio_out`.
   - `telem_valid` will be 1 for exactly one cycle.
5. Read the telemetry data from `uio_out`.
```

---

## Parameterisation (Internal)

Although the design is currently fixed for `N=8` blocks and `PSI=8`, it can be easily modified by changing the local parameters at the top of the Verilog file:

```verilog
localparam N         = 8;       // number of physical blocks
localparam PSI       = 8;       // gap advance period (writes)
```

The address width (`LOG2N`) and mask (`N_MASK`) adjust automatically.

---

## Area and Power (SKY130)

| Component              | Cell count |
|------------------------|------------|
| Start‑Gap logic        | ~90        |
| Wear counters (8×8bit) | 64 FFs     |
| Feistel + LFSR         | ~40        |
| Retired flags + misc   | ~40        |
| FSM + pipeline         | ~150       |
| **Total**              | **~380**   |

The design uses **0** dedicated multipliers or large memories – only standard cells.

---

## Verification

All 17 cocotb tests pass, including:
- Read/write protocol
- Telemetry pulse and data correctness
- Wear counter saturation and retirement
- Gap rotation and wear distribution bounds
- Random 200‑transaction stress test

No formal proofs are included in this release, but the design is easy to formally verify using the provided SVA properties (commented out in the RTL).

---

## Integration

The module is pure synthesizable Verilog‑2001. To use it in your own project:

1. Copy `project.v` into your source tree.
2. Instantiate it as `tt_um_wearlevel_controller` with the given ports.
3. Connect your external EEPROM/flash controller to the `phys` output and the `move_req/move_ack` handshake.

The controller expects the external agent to:
- On `move_req`, copy data from the current physical block (provided earlier via `phys`) to the destination block (`uio_out[2:0]`), then assert `move_ack` for one clock.

---

## License

**Apache 2.0** – free for commercial and non‑commercial use.

---

## Built for reliable, long‑life embedded systems.
