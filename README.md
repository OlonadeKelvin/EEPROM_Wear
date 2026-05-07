# Hardware EEPROM Wear‑Leveling Controller

A plug‑and‑play digital IP for extending the lifetime of external EEPROM / flash memories in embedded and edge‑AI systems.

## 1. Overview
Many low‑power edge devices rely on serial EEPROMs for configuration and data logging. Frequent writes to the same memory block create localized oxide degradation, leading to early failure.

This controller implements dynamic wear‑leveling entirely in hardware, transparently remapping logical block addresses to physical blocks so that write cycles are distributed evenly. The core is supplied as a single synthesizable Verilog module targeting the Tiny Tapeout SKY26b shuttle, but is highly parameterized and ready to be integrated into any FPGA or ASIC system.

## 2. Architecture
The controller maintains two on‑chip tables:
* **Mapping table (`map[4]`)**: logical → physical block (2‑bit indices, 4 blocks)
* **Wear counter table (`wr_count[4]`)**: 16‑bit write count per physical block

### 2.1 Command Interface

| `ui_in[4:3]` | Command | Description |
| :--- | :--- | :--- |
| `00` | `read_req` | Return physical address of a logical block. |
| `01` | `write_req` | Return the physical address where data should be written. |
| `10` | `write_commit` | Signal that a write has taken place → increment wear counter and possibly remap. |
| `11` | `move_ack` | Acknowledge that data movement (copy between physical blocks) is complete. |

A host first issues `write_req`, receives the physical address, performs the actual low‑level write, then asserts `write_commit`. The controller then updates wear statistics and triggers a background block swap if the difference between the most‑worn block and the least‑worn block exceeds the `THRESHOLD` (default 4).

When a swap is required, `move_request` goes high and the source (`uo_out[2:0]`) and destination (`uio_out[2:0]`) physical addresses are presented. The host must copy the data and respond with `move_ack`.

## 3. Mathematical Foundation – Dynamic Wear Leveling
Let $P = \{0,1,2,3\}$ be the set of physical blocks.

For each $p \in P$ we maintain a wear counter $c(p)$. A write to logical block $l$ currently mapped to $p = \text{map}(l)$ causes:

$$c(p) \leftarrow c(p) + 1$$

After the increment, we compute the minimum wear index:

$$p_{\text{min}} = \arg\min_{q \in P} \, c(q)$$

If $c(p) - c(p_{\text{min}}) > T \quad (T = 4)$, the mapping is swapped. Let $l_{\text{other}}$ be the logical block that currently maps to $p_{\text{min}}$. Then:

$$\text{map}(l) \leftarrow p_{\text{min}}, \qquad \text{map}(l_{\text{other}}) \leftarrow p$$

This ensures that any hot logical block is safely migrated to the physical block with the lowest write count, thereby leveling wear without the need for a global wear‑reorder cycle or software overhead.

## 4. FSM Implementation
The core finite‑state machine operates in sequential states:
1.  **IDLE** – Wait for commands.
2.  **S_INC** – Increment $c(p)$.
3.  **S_MIN** – Combinational minimum search over all 4 counters; result latched.
4.  **S_CHECK** – Compare difference, decide whether to remap.
5.  **S_FIND_LOG** – Find the logical block owner of $p_{\text{min}}$.
6.  **S_SWAP → S_WAIT_ACK** – Update the mapping arrays, assert `move_request`, wait for host acknowledgment, then return to IDLE.

All states are pipelined naturally; a standard transaction takes 4 clock cycles without a remap and 6 cycles + `move_ack` wait with a remap.

## 5. Reuse and Integration Guide
### 5.1 Parameterization
The `NUM_BLOCKS` and `THRESHOLD` parameters are defined at the top of the Verilog file. To use the IP with a larger memory, simply increase `NUM_BLOCKS` and the address widths accordingly. The combinational min‑search logic scales linearly and can be pipelined for massive memory arrays.

### 5.2 Integration into an SoC
* Connect `ui_in` to a simple command bus driven by a processor or a dedicated DMA engine.
* Route `uo_out[2:0]` to the address bus of your EEPROM controller.
* Use `busy` to stall further memory commands until the current operation is complete.
* The `move_request` / `move_ack` handshake can be connected to a lightweight DMA that copies one block.

### 5.3 Portable to Any Technology
The design uses only synthesizable Verilog-2001 constructs and has been physically verified with OpenLane on the SKY130 PDK. It comfortably meets timing constraints at 10 MHz without any special routing efforts.

## 6. Verification
A comprehensive Cocotb testbench (`test/test.py`) verifies:
* Reset default mapping and counter state
* Write‑commit without remap (below threshold)
* Automatic remap when threshold crossed, including correct mapping swap and `move_request` assertion
* Correct handling of `move_ack`
* Read requests during busy state (no corruption)
* Rapid back‑to‑back commands
* Gate-Level (GL) Simulation with power pins enabled

The testbench is integrated into the Tiny Tapeout GitHub Actions flow and passes the full `make verify` and GL-test steps.

## 7. IEEE Sponsorship Statement
This design addresses an important reliability challenge in embedded non‑volatile memory systems. It is purely digital, fully self‑contained, and does not rely on any proprietary algorithms. The open‑source community can immediately reuse it in IoT loggers, environmental sensor nodes, and safety‑critical edge devices where memory endurance directly affects system lifetime. The methods demonstrated (hardware wear‑leveling FSM with on‑the‑fly remapping) are applicable to a wide range of NAND/NOR flash and EEPROM devices, making it a valuable, long‑lived contribution to open‑source silicon.
