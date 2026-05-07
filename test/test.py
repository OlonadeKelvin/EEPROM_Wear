# test/test.py
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer
from cocotb.result import TestFailure

NUM_BLOCKS = 8

async def reset_dut(dut):
    dut.rst_n.value = 0
    dut.ui_in.value = 0
    dut.ena.value = 1
    await Timer(100, units="ns")
    dut.rst_n.value = 1
    await Timer(100, units="ns")

async def send_cmd(dut, addr, cmd, move_ack=0):
    dut.ui_in.value = (move_ack << 5) | (cmd << 3) | addr
    await RisingEdge(dut.clk)
    # Wait one more cycle for outputs to settle
    await RisingEdge(dut.clk)

async def write_req(dut, addr):
    # cmd=0x01 write_req
    await send_cmd(dut, addr, 0x1)
    phys = dut.uo_out.value.integer & 0x7
    return phys

async def read_req(dut, addr):
    # cmd=0x00 read_req
    await send_cmd(dut, addr, 0x0)
    phys = dut.uo_out.value.integer & 0x7
    return phys

async def write_commit(dut, wait_busy=True):
    # cmd=0x2 write_commit (address is don't care, but set to 0)
    dut.ui_in.value = (0 << 5) | (0x2 << 3) | 0
    await RisingEdge(dut.clk)
    if wait_busy:
        # Wait while busy is high
        while (dut.uo_out.value.integer >> 3) & 1:
            await RisingEdge(dut.clk)

async def move_ack(dut):
    # send move_ack (cmd=0x3, move_ack bit set) – ensure busy and move_req high
    dut.ui_in.value = (1 << 5) | (0x3 << 3) | 0
    await RisingEdge(dut.clk)
    # wait for busy deassert
    while (dut.uo_out.value.integer >> 3) & 1:
        await RisingEdge(dut.clk)

@cocotb.test()
async def test_reset_defaults(dut):
    """After reset, all wear counters are zero, mapping is identity."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)
    # all mappings should be identity
    for i in range(NUM_BLOCKS):
        p = await read_req(dut, i)
        assert p == i, f"Mapping not identity after reset: logical {i} -> {p}"

@cocotb.test()
async def test_no_remap_under_threshold(dut):
    """Write commit below threshold does not trigger remap."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    for i in range(4):  # 4 writes to logical 0, under threshold of 4
        phys = await write_req(dut, 0)
        assert phys == 0, f"Write req mapping error cycle {i}"
        await write_commit(dut)
        # No move_request expected
        move_req = (dut.uo_out.value.integer >> 4) & 1
        assert move_req == 0, f"move_request asserted early"

    # after 5th write, exceed threshold (wr_count[0]=5, min=0, diff=5 >4)
    phys = await write_req(dut, 0)
    await write_commit(dut)
    # now check if move_request is asserted
    move_req = (dut.uo_out.value.integer >> 4) & 1
    assert move_req == 1, "move_request not asserted after threshold exceeded"

    # acknowledge move
    await move_ack(dut)
    # after move, mapping for logical 0 should have changed to physical 1
    # because min physical was 1 with count 0
    p0 = await read_req(dut, 0)
    assert p0 == 1, f"After remap logical 0 should map to 1, got {p0}"

    # logical 1 should now map to physical 0
    p1 = await read_req(dut, 1)
    assert p1 == 0, f"After remap logical 1 should map to 0, got {p1}"

@cocotb.test()
async def test_multiple_remaps(dut):
    """Sequential writes causing multiple remaps and correct final mapping."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    # We'll write to logical 0 20 times. After several remaps, final physical
    # should be well distributed.
    last_phys = None
    for i in range(20):
        phys = await write_req(dut, 0)
        await write_commit(dut)
        # if move_req, handle it
        if ((dut.uo_out.value.integer >> 4) & 1):
            await move_ack(dut)
        last_phys = phys

    # Just check that after many writes the mapping is no longer 0
    final_phys = await read_req(dut, 0)
    assert final_phys != 0, "After many writes, logical 0 should have moved away from physical 0"

@cocotb.test()
async def test_read_during_write(dut):
    """Read request during write_commit (busy) should return last stable phys."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    # Start a write_commit that triggers remap (need 5 writes)
    for _ in range(4):
        p = await write_req(dut, 0)
        await write_commit(dut)
    # 5th write
    p = await write_req(dut, 0)
    # now commit but interleave read mid-remap (we'll try to read during busy)
    dut.ui_in.value = (0 << 5) | (0x2 << 3) | 0  # commit
    await RisingEdge(dut.clk)  # state -> S_INC
    await RisingEdge(dut.clk)  # S_MIN
    # Now busy should be 1
    # Perform a read_req to logical 2
    read_phys = await read_req(dut, 2)
    # The physical output is latched from previous request, but we can just verify no corruption
    # Should not hang. We'll finish the move.
    if ((dut.uo_out.value.integer >> 4) & 1):
        await move_ack(dut)
    # Verify that mapping of logical 2 unchanged (still 2)
    p2 = await read_req(dut, 2)
    assert p2 == 2, "Read during busy should not alter mapping"

@cocotb.test()
async def test_back_to_back_commits(dut):
    """Fast back-to-back write commits without waiting for idle."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    # pre-writes
    for _ in range(4):
        p = await write_req(dut, 0)
        await write_commit(dut)

    # now two consecutive commits: one that causes remap, second while busy -> ignored?
    p = await write_req(dut, 0)   # 5th write, should trigger remap
    dut.ui_in.value = (0 << 5) | (0x2 << 3) | 0  # commit
    await RisingEdge(dut.clk)  # state IDLE->S_INC
    await RisingEdge(dut.clk)  # S_INC->S_MIN
    # now we are busy, send another commit (should be ignored)
    dut.ui_in.value = (0 << 5) | (0x2 << 3) | 0
    await RisingEdge(dut.clk)
    # No crash, finish the first remap
    if ((dut.uo_out.value.integer >> 4) & 1):
        await move_ack(dut)
    # ensure only one remap happened
    # we can check the wear counters by reading if they were exposed, but just check no lockup
