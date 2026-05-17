# test/test.py
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer
 
# ─── Design constants ───────────────────────────────────────────────────────
N        = 8
PSI      = 8
CNT_MAX  = 0xFF
TOT_WIDTH = 20
 
CMD_READ  = 0b00
CMD_WRITE = 0b01
CMD_TELEM = 0b11
 
TSEL_SKEW     = 0b00
TSEL_TOTAL_LO = 0b01
TSEL_TOTAL_HI = 0b10
TSEL_TOTAL_TOP= 0b11
 
# ─── Bit-field extractors ───────────────────────────────────────────────────
 
def _ui(cmd, addr=0, move_ack=0, telem_sel=0):
    return (telem_sel & 0x3) << 6 | (move_ack & 1) << 5 | (cmd & 0x3) << 3 | (addr & 0x7)
 
def _phys(dut):        return int(dut.uo_out.value) & 0x7
def _busy(dut):        return (int(dut.uo_out.value) >> 3) & 1
def _move_req(dut):    return (int(dut.uo_out.value) >> 4) & 1
def _ecc_err(dut):     return (int(dut.uo_out.value) >> 5) & 1
def _blk_ret(dut):     return (int(dut.uo_out.value) >> 6) & 1
def _telem_vld(dut):   return (int(dut.uo_out.value) >> 7) & 1
def _uio(dut):         return int(dut.uio_out.value) & 0xFF
 
# ─── Core helpers ───────────────────────────────────────────────────────────
 
async def reset_dut(dut):
    """Hard reset; leaves clock running."""
    dut.rst_n.value  = 0
    dut.ui_in.value  = 0
    dut.uio_in.value = 0
    dut.ena.value    = 1
    await Timer(40, units="ns")
    await RisingEdge(dut.clk)
    dut.rst_n.value  = 1
    await ClockCycles(dut.clk, 3)   # allow internals to settle
 
 
async def wait_idle(dut, timeout=64):
    """Poll until busy de-asserts. Raises if timed out."""
    for _ in range(timeout):
        if not _busy(dut):
            return
        await RisingEdge(dut.clk)
    raise AssertionError(f"DUT stuck busy for >{timeout} cycles")
 
 
async def do_read(dut, addr):
    """
    Combinational read — no FSM cycles required.
    Drive cmd=00 and sample phys on the SAME rising edge.
    Returns physical address. busy must be 0 before and after.
    """
    assert _busy(dut) == 0, "do_read called while DUT is busy"
    dut.ui_in.value = _ui(CMD_READ, addr)
    await RisingEdge(dut.clk)
    # phys_out = read_phys (combinational mux; valid this cycle)
    phys = _phys(dut)
    dut.ui_in.value = 0
    assert _busy(dut) == 0, "busy asserted after a read"
    return phys
 
 
async def do_write(dut, addr):
    """
    Full write transaction:
      1. Assert cmd=01 for one clock (FSM latches in ST_IDLE).
      2. Release bus.
      3. Wait for pipeline to drain; handle retirement if needed.
    Returns (phys, retired_flag).
    """
    assert _busy(dut) == 0, "do_write called while DUT is busy"
    dut.ui_in.value = _ui(CMD_WRITE, addr)
    await RisingEdge(dut.clk)   # ST_IDLE samples → ST_FEISTEL; busy asserts
    dut.ui_in.value = 0
 
    retired = False
    for _ in range(64):
        if _move_req(dut):
            # A block just saturated — acknowledge the migration
            dut.ui_in.value = _ui(CMD_WRITE, 0, move_ack=1)
            await RisingEdge(dut.clk)
            dut.ui_in.value = 0
            retired = True
            # busy clears on the same edge; break after one more check
            await RisingEdge(dut.clk)
            break
        if not _busy(dut):
            break
        await RisingEdge(dut.clk)
 
    phys = _phys(dut)
    return phys, retired
 
 
async def do_telem(dut, sel):
    """
    Telemetry request.
    Timing (all rising edges):
      edge 0: cmd=11 sampled in ST_IDLE → telem_valid asserts, → ST_TELEM
      edge 1: ST_TELEM state — telem_valid=1, data on uio_out  ← we sample here
      edge 2: ST_TELEM → ST_IDLE, telem_valid clears
    Returns (telem_valid, uio_byte) sampled at edge 1.
    """
    assert _busy(dut) == 0, "do_telem called while DUT is busy"
    dut.ui_in.value = _ui(CMD_TELEM, telem_sel=sel)
    await RisingEdge(dut.clk)   # edge 0: cmd sampled, → ST_TELEM
    dut.ui_in.value = 0
    # edge 1: we are now in ST_TELEM; telem_valid registered high
    await RisingEdge(dut.clk)
    valid = _telem_vld(dut)
    data  = _uio(dut)
    # edge 2: ST_TELEM → ST_IDLE clears valid
    await RisingEdge(dut.clk)
    return valid, data
 
# ─── Tests ──────────────────────────────────────────────────────────────────
 
@cocotb.test()
async def test_reset_defaults(dut):
    """After reset all status flags clear; phys output in valid range."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    assert _busy(dut)      == 0, "busy not clear after reset"
    assert _move_req(dut)  == 0, "move_req not clear after reset"
    assert _ecc_err(dut)   == 0, "ecc_err not clear after reset"
    assert _telem_vld(dut) == 0, "telem_vld not clear after reset"
 
    for addr in range(N):
        p = await do_read(dut, addr)
        assert 0 <= p < N, f"phys {p} out of range for logical {addr}"
 
 
@cocotb.test()
async def test_read_never_asserts_busy(dut):
    """Reads are combinational — busy must stay 0 for all logical addresses."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    for addr in range(N):
        await do_read(dut, addr)
        assert _busy(dut) == 0, f"busy set after read of logical {addr}"
 
 
@cocotb.test()
async def test_read_phys_in_range(dut):
    """Every logical address maps to a physical address in [0, N-1]."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    for addr in range(N):
        p = await do_read(dut, addr)
        assert 0 <= p < N, f"logical {addr} → phys {p} out of range"
 
 
@cocotb.test()
async def test_write_busy_asserts_and_clears(dut):
    """
    One clock after a write cmd, busy must be 1.
    After the pipeline drains, busy must be 0.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    dut.ui_in.value = _ui(CMD_WRITE, 0)
    await RisingEdge(dut.clk)   # ST_IDLE → ST_FEISTEL; busy latched 1
    dut.ui_in.value = 0
    assert _busy(dut) == 1, "busy not asserted one cycle into write pipeline"
 
    await wait_idle(dut)
    assert _busy(dut) == 0, "busy still set after pipeline complete"
 
 
@cocotb.test()
async def test_write_phys_in_range(dut):
    """phys output must always be in [0, N-1] for every write."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    for i in range(N * 2):
        p, _ = await do_write(dut, i % N)
        assert 0 <= p < N, f"write {i}: phys {p} out of range"
 
 
@cocotb.test()
async def test_write_increments_total(dut):
    """3 writes → total_wr_lo == 3 (verified via telemetry)."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    for i in range(3):
        await do_write(dut, i % N)
 
    valid, lo = await do_telem(dut, TSEL_TOTAL_LO)
    assert valid == 1, "telem_valid not asserted"
    assert lo == 3,    f"Expected total_wr_lo=3, got {lo}"
 
 
@cocotb.test()
async def test_no_retirement_under_saturation(dut):
    """
    PSI-1 writes to the same logical block must complete without any
    retirement (wear counters do not reach 0xFF in 7 writes).
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    for i in range(PSI - 1):
        _, retired = await do_write(dut, 0)
        assert not retired,      f"Unexpected retirement on write {i}"
        assert _move_req(dut) == 0, f"move_req set on write {i}"
        assert _busy(dut)     == 0, f"busy stuck after write {i}"
 
 
@cocotb.test()
async def test_gap_advances_causing_rotation(dut):
    """
    After PSI*2 writes to the same logical address, the Start-Gap rotation
    must have targeted more than one physical block.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    seen = set()
    for i in range(PSI * 2):
        p, _ = await do_write(dut, 0)
        seen.add(p)
        assert _busy(dut) == 0, f"busy stuck after write {i}"
 
    assert len(seen) > 1, (
        f"Start-Gap never rotated: always mapped to {seen}"
    )
 
 
@cocotb.test()
async def test_no_ecc_error_clean_run(dut):
    """ECC error flag stays clear during normal operation."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    for i in range(16):
        await do_write(dut, i % N)
        assert _ecc_err(dut) == 0, f"Spurious ECC error on write {i}"
 
 
@cocotb.test()
async def test_telem_valid_is_one_cycle_pulse(dut):
    """
    telem_valid must be 1 exactly on the ST_TELEM cycle,
    then 0 when back in ST_IDLE.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    # Edge 0: send telem cmd
    dut.ui_in.value = _ui(CMD_TELEM, telem_sel=TSEL_SKEW)
    await RisingEdge(dut.clk)   # ST_IDLE → ST_TELEM; telem_vld registered 1
    dut.ui_in.value = 0
 
    # Edge 1: in ST_TELEM — valid must be 1
    await RisingEdge(dut.clk)
    assert _telem_vld(dut) == 1, "telem_vld not asserted in ST_TELEM"
 
    # Edge 2: back in ST_IDLE — valid must have cleared
    await RisingEdge(dut.clk)
    assert _telem_vld(dut) == 0, "telem_vld did not clear after ST_TELEM"
 
 
@cocotb.test()
async def test_telem_skew_bounded(dut):
    """
    After PSI*N evenly distributed writes, reported skew ≤ PSI+1
    (Start-Gap theoretical bound).
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    for i in range(PSI * N):
        await do_write(dut, i % N)
 
    valid, skew = await do_telem(dut, TSEL_SKEW)
    assert valid == 1, "telem_valid not asserted"
    assert skew <= PSI + 1, (
        f"Wear skew {skew} exceeds Start-Gap bound of {PSI + 1}"
    )
 
 
@cocotb.test()
async def test_telem_total_write_16bit(dut):
    """
    Perform WRITES_COUNT writes; reconstruct 16-bit total_wr from
    lo + hi telemetry bytes and verify it matches.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    WRITES_COUNT = 25
    for i in range(WRITES_COUNT):
        await do_write(dut, i % N)
 
    valid_lo, lo = await do_telem(dut, TSEL_TOTAL_LO)
    valid_hi, hi = await do_telem(dut, TSEL_TOTAL_HI)
 
    assert valid_lo and valid_hi, "telem_valid not asserted"
    total = (hi << 8) | lo
    assert total == WRITES_COUNT, (
        f"Expected total_wr={WRITES_COUNT}, got {total}"
    )
 
 
@cocotb.test()
async def test_retirement_on_saturation(dut):
    """
    Flood one logical address with writes until a block saturates (cnt=0xFF).
    Verify:
      • move_req asserts
      • move_req de-asserts after move_ack
      • busy clears
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    retirement_seen = False
 
    # CNT_MAX+16 gives enough margin for any Gap rotation
    for i in range(CNT_MAX + 16):
        assert _busy(dut) == 0, f"Iteration {i}: busy before write"
 
        # Issue write
        dut.ui_in.value = _ui(CMD_WRITE, 0)
        await RisingEdge(dut.clk)
        dut.ui_in.value = 0
 
        # Drain pipeline; watch for move_req
        for _ in range(64):
            if _move_req(dut):
                # Verify busy is still high during ACK wait
                assert _busy(dut) == 1, "busy not high during move_req"
                retirement_seen = True
 
                # Send move_ack
                dut.ui_in.value = _ui(CMD_WRITE, 0, move_ack=1)
                await RisingEdge(dut.clk)
                dut.ui_in.value = 0
 
                # Verify handshake cleared
                await RisingEdge(dut.clk)
                assert _move_req(dut) == 0, "move_req still set after ack"
                assert _busy(dut)     == 0, "busy still set after ack"
                break
 
            if not _busy(dut):
                break
            await RisingEdge(dut.clk)
 
        if retirement_seen:
            break
 
    assert retirement_seen, (
        "move_req never asserted after flooding writes to saturation"
    )
 
 
@cocotb.test()
async def test_move_req_held_until_ack(dut):
    """move_req must stay asserted every cycle until move_ack is received."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    # Flood to saturation
    for i in range(CNT_MAX + 16):
        dut.ui_in.value = _ui(CMD_WRITE, 0)
        await RisingEdge(dut.clk)
        dut.ui_in.value = 0
 
        reached_wait = False
        for _ in range(64):
            if _move_req(dut):
                reached_wait = True
                # Hold off ack for 5 cycles; move_req must stay 1
                for hold in range(5):
                    assert _move_req(dut) == 1, (
                        f"move_req dropped without ack (hold cycle {hold})"
                    )
                    await RisingEdge(dut.clk)
                # Now ack
                dut.ui_in.value = _ui(CMD_WRITE, 0, move_ack=1)
                await RisingEdge(dut.clk)
                dut.ui_in.value = 0
                await RisingEdge(dut.clk)
                assert _move_req(dut) == 0, "move_req not cleared after ack"
                break
            if not _busy(dut):
                break
            await RisingEdge(dut.clk)
 
        if reached_wait:
            break
 
 
@cocotb.test()
async def test_all_logical_addresses_writable(dut):
    """Every logical block (0–7) can be written without hang or error."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    for addr in range(N):
        p, _ = await do_write(dut, addr)
        assert 0 <= p < N, f"logical {addr} → phys {p} out of range"
        assert _busy(dut) == 0, f"busy stuck after write to logical {addr}"
 
 
@cocotb.test()
async def test_interleaved_reads_and_writes(dut):
    """
    Interleave reads between writes.
    Reads must never assert busy; writes must always complete cleanly.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    for i in range(16):
        await do_write(dut, (i * 2) % N)
        p = await do_read(dut, (i * 2 + 1) % N)
        assert 0 <= p < N, f"Read returned out-of-range phys {p} (iter {i})"
        assert _busy(dut) == 0, f"busy after read (iter {i})"
 
 
@cocotb.test()
async def test_stress_random_200(dut):
    """
    200-transaction pseudo-random write workload.
    Verifies: no hangs, phys always in range, total_wr counter correct.
    Uses a deterministic LCG for reproducibility.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    # Knuth multiplicative LCG
    lcg = 0xACE1
    expected = 0
 
    for i in range(200):
        lcg = (lcg * 6364136223846793005 + 1442695040888963407) & 0xFFFFFFFF
        addr = lcg % N
 
        p, _ = await do_write(dut, addr)
        expected += 1
 
        assert 0 <= p < N, f"[{i}] phys {p} out of range"
        assert _busy(dut) == 0, f"[{i}] busy stuck after write"
 
    # Cross-check total_wr lower 16 bits
    _, lo = await do_telem(dut, TSEL_TOTAL_LO)
    _, hi = await do_telem(dut, TSEL_TOTAL_HI)
    total = (hi << 8) | lo
    assert total == (expected & 0xFFFF), (
        f"total_wr mismatch: expected {expected & 0xFFFF:#x}, got {total:#x}"
    )

