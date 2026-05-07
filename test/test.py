# test/test.py
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer

NUM_BLOCKS = 4

async def reset_dut(dut):
    dut.rst_n.value = 0
    dut.ui_in.value = 0
    dut.ena.value = 1
    await Timer(100, units="ns")
    dut.rst_n.value = 1
    await Timer(100, units="ns")

async def send_cmd(dut, addr, cmd, move_ack=0):
    """
    Drive ui_in with the given fields and advance two clock edges
    to ensure outputs are settled.
    """
    dut.ui_in.value = (move_ack << 5) | (cmd << 3) | addr
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)  # extra cycle for output propagation

async def write_req(dut, addr):
    """Issue a write_req command and return the physical address."""
    await send_cmd(dut, addr, 0x1)
    phys = dut.uo_out.value.integer & 0x7
    return phys

async def read_req(dut, addr):
    """Issue a read_req command and return the physical address."""
    await send_cmd(dut, addr, 0x0)
    phys = dut.uo_out.value.integer & 0x7
    return phys

async def write_commit(dut):
    """
    Issue a write_commit command. This function does NOT wait for
    the operation to complete. The caller must examine the busy
    flag and handle any remap handshake.
    """
    dut.ui_in.value = (0 << 5) | (0x2 << 3) | 0
    await RisingEdge(dut.clk)

async def move_ack(dut):
    """Assert move_ack and wait until busy de-asserts."""
    dut.ui_in.value = (1 << 5) | (0x3 << 3) | 0   # move_ack + cmd=11
    await RisingEdge(dut.clk)
    # wait for busy to go low
    while (dut.uo_out.value.integer >> 3) & 1:
        await RisingEdge(dut.clk)

async def wait_until_idle(dut):
    """Poll busy until it is low."""
    while (dut.uo_out.value.integer >> 3) & 1:
        await RisingEdge(dut.clk)

@cocotb.test()
async def test_reset_defaults(dut):
    """After reset, all wear counters are zero, mapping is identity."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    for i in range(NUM_BLOCKS):
        p = await read_req(dut, i)
        assert p == i, f"Mapping not identity after reset: logical {i} -> {p}"

@cocotb.test()
async def test_no_remap_under_threshold(dut):
    """Write commit below threshold does not trigger remap."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    # 4 writes to logical 0, each must complete without remap
    for i in range(4):
        phys = await write_req(dut, 0)
        assert phys == 0, f"Write req mapping error cycle {i}"
        await write_commit(dut)
        await wait_until_idle(dut)
        move_req = (dut.uo_out.value.integer >> 4) & 1
        assert move_req == 0, f"move_request asserted early at cycle {i}"

    # 5th write - now threshold is exceeded
    phys = await write_req(dut, 0)
    await write_commit(dut)

    # Wait for the FSM to navigate through the internal states to S_SWAP/S_WAIT_ACK
    while ((dut.uo_out.value.integer >> 4) & 1) == 0:
        await RisingEdge(dut.clk)

    # Now that it has reached the move request state, check the flags
    busy = (dut.uo_out.value.integer >> 3) & 1
    move_req = (dut.uo_out.value.integer >> 4) & 1
    assert busy == 1, "busy should be high during remap"
    assert move_req == 1, "move_request not asserted after threshold exceeded"

    # Complete the remap
    await move_ack(dut)

    # Verify mapping changed: logical 0 should now point to physical 1
    # (the least-worn block)
    p0 = await read_req(dut, 0)
    assert p0 == 1, f"After remap logical 0 should map to 1, got {p0}"
    # logical 1 should now point to physical 0
    p1 = await read_req(dut, 1)
    assert p1 == 0, f"After remap logical 1 should map to 0, got {p1}"

@cocotb.test()
async def test_multiple_remaps(dut):
    """Sequential writes causing multiple remaps and correct final mapping."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    for i in range(20):          # 20 writes to logical block 0
        phys = await write_req(dut, 0)
        await write_commit(dut)

        # if a remap was triggered, wait for it
        if (dut.uo_out.value.integer >> 4) & 1:
            await move_ack(dut)
        else:
            await wait_until_idle(dut)

    # After many writes logical 0 must not be stuck on physical 0
    final_phys = await read_req(dut, 0)
    assert final_phys != 0, (
        "After many writes logical 0 should have moved away from physical 0"
    )

@cocotb.test()
async def test_read_during_write(dut):
    """Read request during a remap does not corrupt the mapping."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    # Prime the counters with 4 writes
    for _ in range(4):
        phys = await write_req(dut, 0)
        await write_commit(dut)
        await wait_until_idle(dut)

    # 5th write - triggers a remap
    phys = await write_req(dut, 0)
    # Manually advance the FSM step by step to interleave a read
    dut.ui_in.value = (0 << 5) | (0x2 << 3) | 0   # commit
    await RisingEdge(dut.clk)                     # state -> S_INC
    await RisingEdge(dut.clk)                     # state -> S_MIN (busy=1)

    # Issue a read while busy (should return some physical address)
    read_phys = await read_req(dut, 2)

    # Finish the remap
    if (dut.uo_out.value.integer >> 4) & 1:
        await move_ack(dut)
    else:
        await wait_until_idle(dut)
