# Hardware EEPROM Wear‑Leveling Controller

**A plug‑and‑play digital IP for extending the lifetime of external EEPROM / flash memories in embedded and edge‑AI systems.**

## 1. Overview

Many low‑power edge devices rely on serial EEPROMs for configuration and data logging.  
Frequent writes to the same memory block create **localised oxide degradation**, leading to early failure.  
This controller implements **dynamic wear‑levelling** entirely in hardware, transparently remapping logical block addresses to physical blocks so that write cycles are distributed evenly.

The core is supplied as a single synthesizable Verilog module targeting the Tiny Tapeout SKY26b shuttle, but is parameterized and ready to be integrated into any FPGA or ASIC system.

---

## 2. Architecture

![Block diagram](https://via.placeholder.com/600x300.png?text=Block+Diagram+-+Wear+Leveling+Controller)

The controller maintains two on‑chip tables:
- **Mapping table** (`map[8]`) : logical → physical block (3‑bit indices, 8 blocks)
- **Wear counter table** (`wr_count[8]`) : 16‑bit write count per physical block

### 2.1 Command Interface

| `ui_in[4:3]` | Command         | Description |
|--------------|-----------------|-------------|
| `00`         | `read_req`      | Return physical address of a logical block. |
| `01`         | `write_req`     | Return the physical address where data should be written. |
| `10`         | `write_commit`  | Signal that a write has taken place → increment wear counter and possibly remap. |
| `11`         | `move_ack`      | Acknowledge that data movement (copy between physical blocks) is complete. |

A host first issues `write_req`, receives the physical address, performs the actual low‑level write, then asserts `write_commit`. The controller then updates wear statistics and triggers a background block swap if the difference between the most‑worn block and the least‑worn block exceeds the `THRESHOLD` (default 4).

When a swap is required, `move_request` goes high and the source (`uo_out[2:0]`) and destination (`uio_out[2:0]`) physical addresses are presented. The host must copy the data and respond with `move_ack`.

---

## 3. Mathematical Foundation – Dynamic Wear Leveling

Let \(P = \{0,1,\dots,7\}\) be the set of physical blocks.  
For each \(p \in P\) we maintain a wear counter \(c(p)\).  
A write to logical block \(l\) currently mapped to \(p = \text{map}(l)\) causes:

\[
c(p) \leftarrow c(p) + 1
\]

After the increment, we compute

\[
p_{\text{min}} = \arg\min_{q \in P} \, c(q)
\]

If

\[
c(p) - c(p_{\text{min}}) > T \quad (T = 4),
\]

the mapping is swapped:

- Let \(l_{\text{other}}\) be the logical block that currently maps to \(p_{\text{min}}\).
- Then:
  \[
  \text{map}(l) \leftarrow p_{\text{min}}, \qquad
  \text{map}(l_{\text{other}}) \leftarrow p
  \]

This ensures that any hot logical block is migrated to the physical block with the lowest write count, thereby levelling wear without the need for a global wear‑reorder cycle.

---

## 4. FSM Implementation

The core finite‑state machine operates in six sequential states:

1. **IDLE** – wait for commands.
2. **S_INC** – increment \(c(p)\).
3. **S_MIN** – combinational minimum search over all 8 counters, result latched.
4. **S_CHECK** – compare difference, decide whether to remap.
5. **S_FIND_LOG** – find the logical block owner of \(p_{\text{min}}\).
6. **S_SWAP** → **S_WAIT_ACK** – update the mapping arrays, assert `move_request`, wait for host acknowledgement, then return to IDLE.

All states are pipelined naturally; the entire transaction takes **4 clock cycles** without remap and **6 + `move_ack` wait** with remap.

---

## 5. Reuse and Integration Guide

### 5.1 Parameterization
The `NUM_BLOCKS` and `THRESHOLD` parameters are defined at the top of the Verilog file.  
To use the IP with a larger memory, increase `NUM_BLOCKS` and the address widths accordingly.  
The min‑search loop scales linearly and can be pipelined for very large arrays.

### 5.2 Integration into an SoC
- Connect `ui_in` to a simple command bus driven by a processor or a dedicated DMA engine.
- Route `uo_out[2:0]` to the address bus of your EEPROM controller.
- Use `busy` to stall further memory commands until the current operation is complete.
- The `move_request` / `move_ack` handshake can be connected to a lightweight DMA that copies one block.

### 5.3 Example SPI EEPROM usage
MCU → wear‑level controller → SPI master → EEPROM
The controller outputs the physical address; the MCU then issues the appropriate SPI read/write sequence.

### 5.4 Portable to any technology
The design uses only synthesizable Verilog-2001 constructs and has been verified with OpenLane on the SKY130 PDK. It meets timing at 10 MHz without any special efforts.

---

## 6. Verification

A comprehensive Cocotb testbench (`test/test.py`) verifies:
- Reset default mapping and counter state
- Write‑commit without remap (below threshold)
- Automatic remap when threshold crossed, including correct mapping swap and `move_request` assertion
- Correct handling of `move_ack`
- Read requests during busy state (no corruption)
- Rapid back‑to‑back commands
- Many sequential writes to confirm wear spreading

The testbench is part of the Tiny Tapeout GitHub verification flow and will pass the `make verify` step.

---

## 7. IEEE Sponsorship Statement

This design addresses an important reliability challenge in embedded non‑volatile memory systems. It is **purely digital, fully self‑contained, and does not rely on any proprietary algorithms**. The open‑source community can immediately reuse it in IoT loggers, environmental sensor nodes, and safety‑critical edge devices where memory endurance directly affects system lifetime. The methods demonstrated (hardware wear‑levelling FSM with on‑the‑fly remapping) are applicable to a wide range of NAND/NOR flash and EEPROM devices, making it a valuable, long‑lived contribution to open‑source silicon.

---

*Prepared for Tiny Tapeout SKY26b – IEEE Division 1 Sponsorship*
