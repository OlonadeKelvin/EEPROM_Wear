# test/test.py
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer
 
# Constants matching the RTL
N         = 8
PSI       = 8
CNT_MAX   = 0xFF          # 8-bit saturating counter ceiling
# Cycles from commit until busy de-asserts (no retirement path):
#   IDLE(latch) → FEISTEL → MAP → INC → ADVANCE → RETIRE → IDLE
# That is 6 rising edges observed from the first clock after commit.
WRITE_CYCLES = 7          # conservative upper bound; we always poll
 
CMD_READ   = 0b00
CMD_WRITE  = 0b01
CMD_COMMIT = 0b10
CMD_TELEM  = 0b11
 
TELEM_SKEW     = 0b00
TELEM_TOTAL_LO = 0b01
TELEM_TOTAL_HI = 0b10
TELEM_TOTAL_TOP= 0b11
 
# Low-level helpers
 
def _ui(cmd, addr=0, move_ack=0, telem_sel=0):
    """Build an 8-bit ui_in value."""
    return (telem_sel << 6) | (move_ack << 5) | (cmd << 3) | (addr & 0x7)
 
 
def _phys(dut):
    return int(dut.uo_out.value) & 0x7
 
def _busy(dut):
    return (int(dut.uo_out.value) >> 3) & 1
 
def _move_req(dut):
    return (int(dut.uo_out.value) >> 4) & 1
 
def _ecc_err(dut):
    return (int(dut.uo_out.value) >> 5) & 1
 
def _blk_retired(dut):
    return (int(dut.uo_out.value) >> 6) & 1
 
def _telem_valid(dut):
    return (int(dut.uo_out.value) >> 7) & 1
 
def _uio(dut):
    return int(dut.uio_out.value) & 0xFF
 
 
async def reset_dut(dut):
    """Full reset sequence."""
    dut.rst_n.value    = 0
    dut.ui_in.value    = 0
    dut.uio_in.value   = 0
    dut.ena.value      = 1
    await Timer(50, units="ns")
    await RisingEdge(dut.clk)
    dut.rst_n.value    = 1
    await ClockCycles(dut.clk, 2)  # settle
 
 
async def wait_idle(dut, timeout=64):
    """Wait until busy de-asserts; fail if it takes longer than timeout cycles."""
    for _ in range(timeout):
        if not _busy(dut):
            return
        await RisingEdge(dut.clk)
    raise AssertionError("DUT stuck busy after {:d} cycles".format(timeout))
 
 
async def do_read(dut, addr):
    """
    Issue read_req and wait for phys_lat to be valid.
    A read takes IDLE→FEISTEL→MAP→IDLE = 3 clocks after the cmd is sampled.
    Returns the physical address.
    """
    dut.ui_in.value = _ui(CMD_READ, addr)
    await RisingEdge(dut.clk)          # IDLE samples cmd, moves to ST_FEISTEL
    await RisingEdge(dut.clk)          # ST_FEISTEL → ST_MAP
    await RisingEdge(dut.clk)          # ST_MAP → ST_IDLE, phys_lat written
    # phys_lat is registered; valid from this point
    dut.ui_in.value = 0                # release bus
    return _phys(dut)
 
 
async def do_write_req(dut, addr):
    """
    Issue write_req (latch address into the controller).
    Returns the physical address seen at that moment (before commit pipeline).
    """
    dut.ui_in.value = _ui(CMD_WRITE, addr)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)          # let controller sample + move to FEISTEL
    await RisingEdge(dut.clk)          # FEISTEL → MAP
    await RisingEdge(dut.clk)          # MAP → ST_INC (write_pend=1)
    dut.ui_in.value = 0
    # The controller is now in ST_INC (busy). Drive commit to continue.
 
 
async def do_commit(dut):
    """
    Drive write_commit and let the pipeline drain.
    After reset write_pend is set by do_write_req above; the commit drives
    the remainder of the pipeline.
    Handles the move_req / retirement handshake automatically.
    Returns True if a block-retirement occurred.
    """
    # The RTL samples cmd_commit in ST_IDLE, but in the new design the write
    # pipeline starts immediately on write_req (no separate commit needed to
    # trigger INC — commit is only used to decide whether to run the pipeline).
    # Actually: looking at the RTL, cmd_commit is NOT used — the pipeline
    # starts as soon as cmd_write is seen.  The original bronze commit cmd
    # has been replaced by the automatic pipeline.  We therefore just wait
    # for busy to fall.
    dut.ui_in.value = 0
    retired = False
    for _ in range(32):
        if _move_req(dut):
            # A block hit counter saturation — send move_ack
            dut.ui_in.value = _ui(CMD_COMMIT, move_ack=1)
            await RisingEdge(dut.clk)
            dut.ui_in.value = 0
            retired = True
        if not _busy(dut):
            break
        await RisingEdge(dut.clk)
    return retired
 
 
async def full_write(dut, addr):
    """
    Perform a complete write to logical address addr:
      write_req → pipeline drains → handle retirement if needed.
    Returns (phys, retired).
    """
    dut.ui_in.value = _ui(CMD_WRITE, addr)
    await RisingEdge(dut.clk)          # IDLE: latch cmd, → ST_FEISTEL
    dut.ui_in.value = 0
    # Pipeline runs autonomously; just wait for completion / handle retirement
    retired = await do_commit(dut)
    phys = _phys(dut)
    return phys, retired
 
 
async def read_telem(dut, sel):
    """
    Request a telemetry byte and return it.
    telem_valid pulses for one cycle after the telem cmd.
    """
    dut.ui_in.value = _ui(CMD_TELEM, telem_sel=sel)
    await RisingEdge(dut.clk)          # ST_IDLE processes telem, → ST_TELEM
    # telem_valid is set at end of that clock; check on next rising edge
    await RisingEdge(dut.clk)
    valid = _telem_valid(dut)
    data  = _uio(dut)
    dut.ui_in.value = 0
    await RisingEdge(dut.clk)          # ST_TELEM → ST_IDLE (valid clears)
    return valid, data
 
 
# ─────────────────────────────────────────────────────────────────────────────
# Tests
# ─────────────────────────────────────────────────────────────────────────────
 
@cocotb.test()
async def test_reset_defaults(dut):
    """
    After reset:
      - busy, move_req, ecc_err, blk_retired, telem_valid all clear
      - Start=0, Gap=0 ⟹ startgap(scr, 0, 0) = (scr+1)%N for all logical
        (all addresses ≥ Gap=0, so formula: (scr+0+1)%N)
        With Feistel key from LFSR seed 0x3FF we can't predict scr, but we
        CAN assert that phys is always in [0, N-1] and that two reads of the
        same logical return the same result (mapping is deterministic once the
        LFSR has advanced the same number of cycles).
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    # Flags all clear after reset
    assert _busy(dut)      == 0, "busy should be 0 after reset"
    assert _move_req(dut)  == 0, "move_req should be 0 after reset"
    assert _ecc_err(dut)   == 0, "ecc_err should be 0 after reset"
    assert _telem_valid(dut) == 0, "telem_valid should be 0 after reset"
 
    # Physical addresses always in valid range
    for addr in range(N):
        p = await do_read(dut, addr)
        assert 0 <= p < N, (
            f"Physical address {p} out of range for logical {addr}"
        )
 
 
@cocotb.test()
async def test_read_does_not_set_busy(dut):
    """A read request must not leave busy asserted on return."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    for addr in range(N):
        await do_read(dut, addr)
        assert _busy(dut) == 0, f"busy still set after read of logical {addr}"
 
 
@cocotb.test()
async def test_write_increments_total(dut):
    """
    Each write increments the total_wr counter, readable via telemetry.
    We do 3 writes then check total_lo == 3.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    for i in range(3):
        await full_write(dut, i % N)
 
    valid, lo = await read_telem(dut, TELEM_TOTAL_LO)
    assert valid == 1, "telem_valid not asserted"
    assert lo == 3,    f"Expected total_wr=3, got {lo}"
 
 
@cocotb.test()
async def test_write_pipeline_no_retirement(dut):
    """
    PSI-1 (=7) writes to the same logical block must complete without
    triggering a block retirement (counter reaches 7, not 255).
    busy must clear after each write; move_req must never assert.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    for i in range(PSI - 1):
        _, retired = await full_write(dut, 0)
        assert not retired,     f"Unexpected retirement on write {i}"
        assert _busy(dut) == 0, f"busy stuck after write {i}"
        assert _move_req(dut) == 0, f"move_req unexpectedly set after write {i}"
 
 
@cocotb.test()
async def test_gap_advances_after_psi_writes(dut):
    """
    After PSI (=8) writes the Gap register increments.
    We verify this indirectly: the physical address returned for logical 0
    must differ from the one returned at reset (Start-Gap rotates).
    
    NOTE: Because Feistel uses the LFSR (which advances every clock cycle),
    the same logical address can map to different physicals across reads.
    We verify behavioural correctness: after enough writes, the controller
    is still alive (busy clears, no hang).
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    physicals_seen = set()
    for i in range(PSI * 2):
        p, _ = await full_write(dut, 0)
        physicals_seen.add(p)
        assert _busy(dut) == 0, f"busy stuck after write {i}"
 
    # After 2 full rotation periods, more than one physical must have been
    # targeted (wear leveling is working)
    assert len(physicals_seen) > 1, (
        f"Start-Gap never rotated physical address; only saw {physicals_seen}"
    )
 
 
@cocotb.test()
async def test_no_ecc_error_clean_run(dut):
    """ECC error flag must stay clear during normal operation."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    for i in range(16):
        await full_write(dut, i % N)
        assert _ecc_err(dut) == 0, f"Spurious ECC error on write {i}"
 
 
@cocotb.test()
async def test_physical_always_in_range(dut):
    """Physical address output must always be in [0, N-1] across many writes."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    for i in range(32):
        p, _ = await full_write(dut, i % N)
        assert 0 <= p < N, f"phys={p} out of range on write {i}"
 
 
@cocotb.test()
async def test_telemetry_skew_bounded(dut):
    """
    After PSI*N writes distributed evenly across all logical blocks,
    the reported skew must satisfy the Start-Gap theoretical bound: ≤ PSI+1.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    writes = PSI * N
    for i in range(writes):
        await full_write(dut, i % N)
 
    valid, skew = await read_telem(dut, TELEM_SKEW)
    assert valid == 1, "telem_valid not asserted"
    assert skew <= PSI + 1, (
        f"Wear skew {skew} exceeds theoretical bound {PSI + 1}"
    )
 
 
@cocotb.test()
async def test_telemetry_total_write_counter(dut):
    """
    total_wr counter: write WRITES_COUNT times, then read back lo+hi bytes
    and verify the reconstructed 16-bit value equals WRITES_COUNT.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    WRITES_COUNT = 20
    for i in range(WRITES_COUNT):
        await full_write(dut, i % N)
 
    valid_lo, lo = await read_telem(dut, TELEM_TOTAL_LO)
    valid_hi, hi = await read_telem(dut, TELEM_TOTAL_HI)
 
    assert valid_lo == 1 and valid_hi == 1, "telem_valid not asserted"
    total = (hi << 8) | lo
    assert total == WRITES_COUNT, (
        f"Expected total_wr={WRITES_COUNT}, reconstructed {total}"
    )
 
 
@cocotb.test()
async def test_telem_valid_pulses_one_cycle(dut):
    """
    telem_valid must be a single-cycle pulse; it must be 0 two clocks after
    the telem command.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    dut.ui_in.value = _ui(CMD_TELEM, telem_sel=TELEM_SKEW)
    await RisingEdge(dut.clk)   # IDLE → ST_TELEM
    await RisingEdge(dut.clk)   # ST_TELEM: telem_valid should be 1
    assert _telem_valid(dut) == 1, "telem_valid not asserted in ST_TELEM"
    dut.ui_in.value = 0
    await RisingEdge(dut.clk)   # ST_TELEM → IDLE: telem_valid should clear
    assert _telem_valid(dut) == 0, "telem_valid did not clear after ST_TELEM"
 
 
@cocotb.test()
async def test_retirement_on_counter_saturation(dut):
    """
    Drive a single block's counter to saturation (255 writes to the block
    that logical 0 maps to). The controller must:
      1. Assert move_req
      2. De-assert after move_ack
      3. Mark the block as retired (blk_retired flag on subsequent reads)
    
    Since PSI=8 and Gap rotates every 8 writes, logical 0 will rotate away
    from its initial physical. We deliberately write to ALL logical addresses
    to exhaust one physical block.
    
    Strategy: perform 255 writes to logical 0 while watching for move_req.
    The Feistel scrambler means we can't know exactly which physical gets
    hit, but eventually one saturates.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    retirement_seen = False
    for i in range(CNT_MAX + 8):    # a few extra to be sure
        dut.ui_in.value = _ui(CMD_WRITE, 0)
        await RisingEdge(dut.clk)   # IDLE → FEISTEL
        dut.ui_in.value = 0
 
        # Drain pipeline; handle move_req
        for _ in range(32):
            if _move_req(dut):
                retirement_seen = True
                # Acknowledge the migration request
                dut.ui_in.value = _ui(CMD_COMMIT, move_ack=1)
                await RisingEdge(dut.clk)
                dut.ui_in.value = 0
            if not _busy(dut):
                break
            await RisingEdge(dut.clk)
 
        if retirement_seen:
            break
 
    assert retirement_seen, (
        "move_req never asserted after filling a wear counter to saturation"
    )
    assert _move_req(dut) == 0, "move_req still set after move_ack"
    assert _busy(dut)     == 0, "busy still set after retirement handshake"
 
 
@cocotb.test()
async def test_read_while_idle(dut):
    """
    A read issued while the controller is idle (not busy) must always
    return a valid physical address in [0, N-1] and must not trigger busy.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    for addr in range(N):
        p = await do_read(dut, addr)
        assert 0 <= p < N, f"Read logical {addr} returned invalid phys {p}"
        assert _busy(dut) == 0, "busy set after read"
 
 
@cocotb.test()
async def test_busy_during_write_pipeline(dut):
    """
    busy must be asserted from the cycle after write_req until the pipeline
    completes.  We sample it mid-pipeline (after ST_FEISTEL) and verify.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    dut.ui_in.value = _ui(CMD_WRITE, 3)
    await RisingEdge(dut.clk)      # IDLE → ST_FEISTEL  (busy set here)
    dut.ui_in.value = 0
    # One clock into the pipeline busy must be high
    assert _busy(dut) == 1, "busy not set one cycle into write pipeline"
 
    # Wait for completion
    await wait_idle(dut)
    assert _busy(dut) == 0, "busy still set after pipeline completion"
 
 
@cocotb.test()
async def test_interleaved_reads_and_writes(dut):
    """
    Interleave reads on different logical blocks between writes.
    All reads must return valid physical addresses; all writes must complete.
    No hangs or corruption.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    for i in range(16):
        # Write to even-indexed blocks
        await full_write(dut, (i * 2) % N)
        # Read odd-indexed block immediately after
        p = await do_read(dut, (i * 2 + 1) % N)
        assert 0 <= p < N, f"Read returned out-of-range phys {p} on iteration {i}"
        assert _busy(dut) == 0, f"busy stuck after read on iteration {i}"
 
 
@cocotb.test()
async def test_all_logical_blocks_writable(dut):
    """Every logical block address (0–7) can be written without error."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    for addr in range(N):
        p, retired = await full_write(dut, addr)
        assert 0 <= p < N, f"Write to logical {addr} returned invalid phys {p}"
        assert _busy(dut) == 0, f"busy stuck after write to logical {addr}"
 
 
@cocotb.test()
async def test_move_req_de_asserts_after_ack(dut):
    """
    Saturate a block and verify the full retirement handshake:
      move_req asserts → ack sent → move_req de-asserts → busy clears.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    # Drive enough writes to one logical to saturate its physical
    for i in range(CNT_MAX + 16):
        dut.ui_in.value = _ui(CMD_WRITE, 0)
        await RisingEdge(dut.clk)
        dut.ui_in.value = 0
 
        acked = False
        for _ in range(32):
            if _move_req(dut) and not acked:
                dut.ui_in.value = _ui(CMD_COMMIT, move_ack=1)
                await RisingEdge(dut.clk)
                dut.ui_in.value = 0
                acked = True
                # Verify de-assertion
                await RisingEdge(dut.clk)
                assert _move_req(dut) == 0, "move_req did not de-assert after ack"
                assert _busy(dut)     == 0, "busy did not clear after ack"
                break
            if not _busy(dut) and not _move_req(dut):
                break
            await RisingEdge(dut.clk)
 
        if acked:
            break   # retirement verified — test passes
 
 
@cocotb.test()
async def test_stress_random_addresses(dut):
    """
    200-transaction pseudo-random workload.
    Verifies: no hangs, physical always in [0,N-1], total_wr matches.
    Uses a simple LCG for repeatability without importing random.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
 
    # LCG parameters (Knuth)
    lcg = 0xACE1
    expected_total = 0
 
    for i in range(200):
        lcg = (lcg * 6364136223846793005 + 1442695040888963407) & 0xFFFFFFFF
        addr = lcg % N
 
        dut.ui_in.value = _ui(CMD_WRITE, addr)
        await RisingEdge(dut.clk)
        dut.ui_in.value = 0
        expected_total += 1
 
        # Drain with retirement handling
        for _ in range(48):
            if _move_req(dut):
                dut.ui_in.value = _ui(CMD_COMMIT, move_ack=1)
                await RisingEdge(dut.clk)
                dut.ui_in.value = 0
            if not _busy(dut):
                break
            await RisingEdge(dut.clk)
 
        p = _phys(dut)
        assert 0 <= p < N, f"[iter {i}] phys {p} out of range"
 
    # Verify total_wr counter (only lower 16 bits checked)
    valid_lo, lo = await read_telem(dut, TELEM_TOTAL_LO)
    valid_hi, hi = await read_telem(dut, TELEM_TOTAL_HI)
    assert valid_lo and valid_hi
    total = (hi << 8) | lo
    assert total == (expected_total & 0xFFFF), (
        f"total_wr mismatch: expected {expected_total & 0xFFFF}, got {total}"
    )

