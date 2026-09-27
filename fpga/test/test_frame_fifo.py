"""
test_frame_fifo.py — Unit test for frame descriptor FIFO.

Tests frame_fifo.v (4-deep registered FIFO for acquisition→decode handoff).
Validates:
  1. Push 1, pop 1 (data integrity)
  2. Fill to capacity (4), verify full flag
  3. Drain to empty, verify empty flag
  4. Interleaved push/pop (FIFO ordering)
  5. Flush clears all entries
  6. Overflow protection (push when full ignored)

DUT: frame_fifo (standalone)
Gate test (test_*). Runs in sim.sh.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge


# =========================================================
# Helpers
# =========================================================

async def reset_dut(dut):
    """Reset the FIFO."""
    dut.rst_n.value = 0
    dut.flush.value = 0
    dut.push.value = 0
    dut.pop.value = 0
    dut.push_ltf_pos.value = 0
    dut.push_phase_inc.value = 0
    for _ in range(5):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


async def push_entry(dut, ltf_pos, phase_inc):
    """Push one entry (1 cycle)."""
    dut.push.value = 1
    dut.push_ltf_pos.value = ltf_pos
    dut.push_phase_inc.value = phase_inc
    await RisingEdge(dut.clk)
    dut.push.value = 0


async def pop_entry(dut):
    """Pop one entry, return {ltf_pos, phase_inc}."""
    # Read combinational output before popping
    ltf_pos = int(dut.pop_ltf_pos.value)
    phase_inc = int(dut.pop_phase_inc.value)
    dut.pop.value = 1
    await RisingEdge(dut.clk)
    dut.pop.value = 0
    return {'ltf_pos': ltf_pos, 'phase_inc': phase_inc}


# =========================================================
# Tests
# =========================================================

@cocotb.test()
async def test_push_pop_single(dut):
    """Push 1 entry, pop 1 — data integrity check."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Initially empty
    assert int(dut.empty.value) == 1, "Should start empty"
    assert int(dut.full.value) == 0, "Should not start full"
    assert int(dut.count.value) == 0, "Count should be 0"

    # Push
    await push_entry(dut, ltf_pos=0x1234, phase_inc=0xABCD)
    await RisingEdge(dut.clk)  # Let it register

    assert int(dut.empty.value) == 0, "Should not be empty after push"
    assert int(dut.count.value) == 1, "Count should be 1"

    # Pop
    entry = await pop_entry(dut)
    await RisingEdge(dut.clk)

    assert entry['ltf_pos'] == 0x1234, f"ltf_pos mismatch: {entry['ltf_pos']:#x}"
    assert entry['phase_inc'] == 0xABCD, f"phase_inc mismatch: {entry['phase_inc']:#x}"
    assert int(dut.empty.value) == 1, "Should be empty after pop"

    dut._log.info("Push/pop single: OK")


@cocotb.test()
async def test_fill_to_capacity(dut):
    """Push 4 entries → full flag asserts. 5th push ignored."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Push 4 entries
    for i in range(4):
        await push_entry(dut, ltf_pos=i * 100, phase_inc=i * 0x1000)
        await RisingEdge(dut.clk)

    assert int(dut.full.value) == 1, "Should be full after 4 pushes"
    assert int(dut.count.value) == 4, f"Count should be 4, got {int(dut.count.value)}"

    # 5th push — should be ignored
    await push_entry(dut, ltf_pos=0x7FFF, phase_inc=0xFFFF)
    await RisingEdge(dut.clk)

    # Still full, count still 4
    assert int(dut.full.value) == 1, "Should still be full"
    assert int(dut.count.value) == 4, "Count should still be 4"

    # Pop first entry — should be entry 0, not the overflow
    entry = await pop_entry(dut)
    assert entry['ltf_pos'] == 0, f"First entry wrong: {entry['ltf_pos']}"
    assert entry['phase_inc'] == 0, f"First entry phase wrong: {entry['phase_inc']}"

    dut._log.info("Fill to capacity: OK")


@cocotb.test()
async def test_drain_to_empty(dut):
    """Push 4, pop 4 — verify FIFO ordering and empty at end."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Push 4
    test_data = [(100, 0x1111), (200, 0x2222), (300, 0x3333), (400, 0x4444)]
    for pos, inc in test_data:
        await push_entry(dut, ltf_pos=pos, phase_inc=inc)
        await RisingEdge(dut.clk)

    # Pop all 4 — verify FIFO order
    for i, (exp_pos, exp_inc) in enumerate(test_data):
        entry = await pop_entry(dut)
        await RisingEdge(dut.clk)
        assert entry['ltf_pos'] == exp_pos, \
            f"Entry {i}: ltf_pos {entry['ltf_pos']} != {exp_pos}"
        assert entry['phase_inc'] == exp_inc, \
            f"Entry {i}: phase_inc {entry['phase_inc']:#x} != {exp_inc:#x}"

    assert int(dut.empty.value) == 1, "Should be empty after draining"
    assert int(dut.count.value) == 0, "Count should be 0"

    dut._log.info("Drain to empty: OK (FIFO ordering correct)")


@cocotb.test()
async def test_interleaved_push_pop(dut):
    """Interleaved push/pop — verify ordering maintained."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Push 2
    await push_entry(dut, ltf_pos=10, phase_inc=0xA)
    await RisingEdge(dut.clk)
    await push_entry(dut, ltf_pos=20, phase_inc=0xB)
    await RisingEdge(dut.clk)

    assert int(dut.count.value) == 2

    # Pop 1 (should be entry 0)
    entry = await pop_entry(dut)
    await RisingEdge(dut.clk)
    assert entry['ltf_pos'] == 10
    assert int(dut.count.value) == 1

    # Push 2 more
    await push_entry(dut, ltf_pos=30, phase_inc=0xC)
    await RisingEdge(dut.clk)
    await push_entry(dut, ltf_pos=40, phase_inc=0xD)
    await RisingEdge(dut.clk)

    assert int(dut.count.value) == 3

    # Pop remaining 3 — order should be 20, 30, 40
    expected = [(20, 0xB), (30, 0xC), (40, 0xD)]
    for exp_pos, exp_inc in expected:
        entry = await pop_entry(dut)
        await RisingEdge(dut.clk)
        assert entry['ltf_pos'] == exp_pos, \
            f"Expected pos={exp_pos}, got {entry['ltf_pos']}"
        assert entry['phase_inc'] == exp_inc

    assert int(dut.empty.value) == 1

    dut._log.info("Interleaved push/pop: OK")


@cocotb.test()
async def test_flush(dut):
    """Flush clears all entries regardless of fill level."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Push 3 entries
    for i in range(3):
        await push_entry(dut, ltf_pos=i * 50, phase_inc=i * 0x100)
        await RisingEdge(dut.clk)

    assert int(dut.count.value) == 3

    # Flush
    dut.flush.value = 1
    await RisingEdge(dut.clk)
    dut.flush.value = 0
    await RisingEdge(dut.clk)

    assert int(dut.empty.value) == 1, "Should be empty after flush"
    assert int(dut.count.value) == 0, "Count should be 0 after flush"
    assert int(dut.full.value) == 0, "Should not be full after flush"

    # Can push again after flush
    await push_entry(dut, ltf_pos=999, phase_inc=0xBEEF)
    await RisingEdge(dut.clk)

    assert int(dut.count.value) == 1
    entry = await pop_entry(dut)
    assert entry['ltf_pos'] == 999
    assert entry['phase_inc'] == 0xBEEF

    dut._log.info("Flush: OK")
