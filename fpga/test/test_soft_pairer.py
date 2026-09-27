"""
Test soft_pairer -- pairs sequential soft bits for the Viterbi.

Verifies the stall_in contract (lever 2a-prime): while stalled the
output pair is held (valid_out stays asserted) and incoming bits are
NOT consumed -- no lost or duplicated bits.
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer


async def reset(dut):
    dut.rst_n.value = 0
    dut.frame_start.value = 0
    dut.stall_in.value = 0
    dut.valid_in.value = 0
    dut.soft_in.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


async def send_bit(dut, value):
    dut.valid_in.value = 1
    dut.soft_in.value = value
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    dut.valid_in.value = 0


@cocotb.test()
async def test_stall_holds_pair_and_defers_consumption(dut):
    """stall_in holds the output pair and prevents input consumption."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    # Emit first pair: bits 0x01, 0x02 (outputs are negated)
    await send_bit(dut, 0x01)
    await send_bit(dut, 0x02)
    assert int(dut.valid_out.value) == 1, "pair not emitted"
    s0 = int(dut.soft0.value)
    s1 = int(dut.soft1.value)
    assert s0 == (-0x01) & 0xFF, f"soft0 = {s0:#x}"
    assert s1 == (-0x02) & 0xFF, f"soft1 = {s1:#x}"

    # Stall: the emitted pair must remain stable
    dut.stall_in.value = 1
    for _ in range(5):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        assert int(dut.valid_out.value) == 1, "valid_out dropped during stall"
        assert int(dut.soft0.value) == s0, "soft0 changed during stall"
        assert int(dut.soft1.value) == s1, "soft1 changed during stall"

    # Present a new bit during the stall -- must NOT be consumed
    dut.valid_in.value = 1
    dut.soft_in.value = 0x03
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    assert int(dut.valid_out.value) == 1, "valid_out dropped while stalled with input pending"
    assert int(dut.soft0.value) == s0, f"soft0 changed during stall with input pending: {int(dut.soft0.value):#x}"
    assert int(dut.soft1.value) == s1, f"soft1 changed during stall with input pending: {int(dut.soft1.value):#x}"
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    assert int(dut.valid_out.value) == 1, "valid_out dropped while stalled with input pending"
    assert int(dut.soft0.value) == s0, "soft0 changed during stall with input pending"
    assert int(dut.soft1.value) == s1, "soft1 changed during stall with input pending"

    # Release stall while the bit is still presented (upstream holds its
    # valid, like the deinterleaver will) -- now it may be consumed
    dut.stall_in.value = 0
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    dut.valid_in.value = 0

    # Second pair: (0x03, 0x04)
    await send_bit(dut, 0x04)
    assert int(dut.valid_out.value) == 1, "second pair not emitted"
    assert int(dut.soft0.value) == (-0x03) & 0xFF, f"soft0 = {int(dut.soft0.value):#x}"
    assert int(dut.soft1.value) == (-0x04) & 0xFF, f"soft1 = {int(dut.soft1.value):#x}"
