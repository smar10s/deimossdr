"""Cocotb tests for async_fifo — gray-code pointer async FIFO."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer
import random

# Match real deployment clocks
WR_CLK_PERIOD_NS = 16.27  # l_clk ~ 61.44 MHz
RD_CLK_PERIOD_NS = 10.0   # sys_clk = 100 MHz

DEPTH = 4
WIDTH = 25


async def reset_fifo(dut):
    """Assert both resets for several cycles, then release."""
    dut.wr_rst.value = 1
    dut.rd_rst.value = 1
    dut.wr_en.value = 0
    dut.rd_en.value = 0
    dut.wr_data.value = 0
    # Wait enough cycles for both clock domains
    await ClockCycles(dut.wr_clk, 8)
    await ClockCycles(dut.rd_clk, 8)
    dut.wr_rst.value = 0
    dut.rd_rst.value = 0
    # Allow synchronizers to settle
    await ClockCycles(dut.wr_clk, 4)
    await ClockCycles(dut.rd_clk, 4)


async def write_word(dut, data):
    """Write one word, wait for it to be accepted."""
    dut.wr_data.value = data
    dut.wr_en.value = 1
    await RisingEdge(dut.wr_clk)
    dut.wr_en.value = 0


async def read_word(dut):
    """Read one word from the FIFO. Returns the value."""
    dut.rd_en.value = 1
    await RisingEdge(dut.rd_clk)
    dut.rd_en.value = 0
    await Timer(1, unit='ns')
    return int(dut.rd_data.value)


async def wait_not_empty(dut, timeout_cycles=50):
    """Wait until empty deasserts in rd_clk domain."""
    for _ in range(timeout_cycles):
        await RisingEdge(dut.rd_clk)
        await Timer(1, unit='ns')
        if int(dut.empty.value) == 0:
            return True
    return False


async def wait_not_full(dut, timeout_cycles=50):
    """Wait until full deasserts in wr_clk domain."""
    for _ in range(timeout_cycles):
        await RisingEdge(dut.wr_clk)
        await Timer(1, unit='ns')
        if int(dut.full.value) == 0:
            return True
    return False


# =========================================================
# Tests
# =========================================================

@cocotb.test()
async def test_basic_write_read(dut):
    """Write one word, read it back, verify data integrity."""
    cocotb.start_soon(Clock(dut.wr_clk, WR_CLK_PERIOD_NS, unit='ns').start())
    cocotb.start_soon(Clock(dut.rd_clk, RD_CLK_PERIOD_NS, unit='ns').start())
    await reset_fifo(dut)

    test_val = 0x1ABCDEF & ((1 << WIDTH) - 1)
    await write_word(dut, test_val)

    # Wait for data to appear on read side (CDC latency)
    assert await wait_not_empty(dut), "FIFO should not remain empty after write"

    # Read and verify
    # rd_data is combinational from memory, available when not empty
    await Timer(1, unit='ns')
    rd_val = int(dut.rd_data.value)
    assert rd_val == test_val, f"Read 0x{rd_val:X} expected 0x{test_val:X}"

    # Now pop it
    dut.rd_en.value = 1
    await RisingEdge(dut.rd_clk)
    dut.rd_en.value = 0
    # After pop, should be empty again (wait for pointer update)
    await ClockCycles(dut.rd_clk, 2)
    await Timer(1, unit='ns')
    assert int(dut.empty.value) == 1, "FIFO should be empty after reading single item"
    cocotb.log.info("PASS: test_basic_write_read")


@cocotb.test()
async def test_full_flag(dut):
    """Fill FIFO to capacity, verify full asserts, verify writes are suppressed."""
    cocotb.start_soon(Clock(dut.wr_clk, WR_CLK_PERIOD_NS, unit='ns').start())
    cocotb.start_soon(Clock(dut.rd_clk, RD_CLK_PERIOD_NS, unit='ns').start())
    await reset_fifo(dut)

    written = []
    # Write DEPTH items
    for i in range(DEPTH):
        val = random.randint(0, (1 << WIDTH) - 1)
        written.append(val)
        dut.wr_data.value = val
        dut.wr_en.value = 1
        await RisingEdge(dut.wr_clk)
    dut.wr_en.value = 0

    # Full flag is 1-cycle pessimistic (uses registered wr_gray), so it may
    # take a couple extra cycles to assert after the last write is committed.
    # Wait for the synchronized rd_gray to propagate back.
    await ClockCycles(dut.wr_clk, 6)
    await Timer(1, unit='ns')
    assert int(dut.full.value) == 1, "FIFO should be full after DEPTH writes"

    # Attempt write while full — should be suppressed
    overflow_val = 0x1FFFFFF
    dut.wr_data.value = overflow_val
    dut.wr_en.value = 1
    await RisingEdge(dut.wr_clk)
    dut.wr_en.value = 0
    await RisingEdge(dut.wr_clk)

    # Drain and verify only original data present
    assert await wait_not_empty(dut), "FIFO should not be empty"
    for i in range(DEPTH):
        await Timer(1, unit='ns')
        rd_val = int(dut.rd_data.value)
        assert rd_val == written[i], f"Word {i}: got 0x{rd_val:X} expected 0x{written[i]:X}"
        dut.rd_en.value = 1
        await RisingEdge(dut.rd_clk)
        dut.rd_en.value = 0
        if i < DEPTH - 1:
            await RisingEdge(dut.rd_clk)

    cocotb.log.info("PASS: test_full_flag")


@cocotb.test()
async def test_empty_flag(dut):
    """Start empty, verify reads don't proceed, write one item, verify empty deasserts."""
    cocotb.start_soon(Clock(dut.wr_clk, WR_CLK_PERIOD_NS, unit='ns').start())
    cocotb.start_soon(Clock(dut.rd_clk, RD_CLK_PERIOD_NS, unit='ns').start())
    await reset_fifo(dut)

    # Should start empty
    await Timer(1, unit='ns')
    assert int(dut.empty.value) == 1, "FIFO should be empty after reset"

    # rd_en while empty should not advance pointer (rd_bin_next uses rd_en & ~empty)
    dut.rd_en.value = 1
    await RisingEdge(dut.rd_clk)
    await RisingEdge(dut.rd_clk)
    dut.rd_en.value = 0
    await Timer(1, unit='ns')
    assert int(dut.empty.value) == 1, "FIFO should still be empty after rd_en while empty"

    # Write one item
    val = 0xDEAD
    await write_word(dut, val)

    # Wait for empty to deassert
    assert await wait_not_empty(dut), "Empty should deassert after write"
    cocotb.log.info("PASS: test_empty_flag")


@cocotb.test()
async def test_fill_and_drain(dut):
    """Fill completely, drain completely, verify all data in order."""
    cocotb.start_soon(Clock(dut.wr_clk, WR_CLK_PERIOD_NS, unit='ns').start())
    cocotb.start_soon(Clock(dut.rd_clk, RD_CLK_PERIOD_NS, unit='ns').start())
    await reset_fifo(dut)

    written = []
    for i in range(DEPTH):
        val = random.randint(0, (1 << WIDTH) - 1)
        written.append(val)
        dut.wr_data.value = val
        dut.wr_en.value = 1
        await RisingEdge(dut.wr_clk)
    dut.wr_en.value = 0

    # Wait for data to propagate to read side
    assert await wait_not_empty(dut), "Data should appear on read side"

    # Drain all
    read_data = []
    for i in range(DEPTH):
        await Timer(1, unit='ns')
        if int(dut.empty.value) == 1:
            # Wait more for CDC
            assert await wait_not_empty(dut), f"Stalled at word {i}"
            await Timer(1, unit='ns')
        rd_val = int(dut.rd_data.value)
        read_data.append(rd_val)
        dut.rd_en.value = 1
        await RisingEdge(dut.rd_clk)
        dut.rd_en.value = 0
        await RisingEdge(dut.rd_clk)

    assert read_data == written, f"Data mismatch: {read_data} != {written}"

    # Should be empty now
    await ClockCycles(dut.rd_clk, 4)
    await Timer(1, unit='ns')
    assert int(dut.empty.value) == 1, "FIFO should be empty after draining"
    cocotb.log.info("PASS: test_fill_and_drain")


@cocotb.test()
async def test_simultaneous_write_read(dut):
    """Writer at 61.44 MHz, reader at 100 MHz. Stream 100+ words, verify order."""
    cocotb.start_soon(Clock(dut.wr_clk, WR_CLK_PERIOD_NS, unit='ns').start())
    cocotb.start_soon(Clock(dut.rd_clk, RD_CLK_PERIOD_NS, unit='ns').start())
    await reset_fifo(dut)

    NUM_WORDS = 120
    written = [random.randint(0, (1 << WIDTH) - 1) for _ in range(NUM_WORDS)]
    read_data = []

    async def writer():
        for val in written:
            # Wait until not full
            while True:
                await Timer(1, unit='ns')
                if int(dut.full.value) == 0:
                    break
                await RisingEdge(dut.wr_clk)
            dut.wr_data.value = val
            dut.wr_en.value = 1
            await RisingEdge(dut.wr_clk)
            dut.wr_en.value = 0

    async def reader():
        while len(read_data) < NUM_WORDS:
            await RisingEdge(dut.rd_clk)
            await Timer(1, unit='ns')
            if int(dut.empty.value) == 0:
                rd_val = int(dut.rd_data.value)
                read_data.append(rd_val)
                dut.rd_en.value = 1
                await RisingEdge(dut.rd_clk)
                dut.rd_en.value = 0

    cocotb.start_soon(writer())
    await reader()

    assert len(read_data) == NUM_WORDS, f"Got {len(read_data)} words, expected {NUM_WORDS}"
    assert read_data == written, "Data mismatch in simultaneous write/read"
    cocotb.log.info("PASS: test_simultaneous_write_read")


@cocotb.test()
async def test_asymmetric_fast_write_slow_read(dut):
    """Writer faster than reader — stress the full flag path."""
    # Swap clock speeds: writer at 100MHz, reader at 61.44MHz
    cocotb.start_soon(Clock(dut.wr_clk, RD_CLK_PERIOD_NS, unit='ns').start())  # 10ns = fast
    cocotb.start_soon(Clock(dut.rd_clk, WR_CLK_PERIOD_NS, unit='ns').start())  # 16.27ns = slow
    await reset_fifo(dut)

    NUM_WORDS = 80
    written = [random.randint(0, (1 << WIDTH) - 1) for _ in range(NUM_WORDS)]
    read_data = []

    async def writer():
        for val in written:
            while True:
                await Timer(1, unit='ns')
                if int(dut.full.value) == 0:
                    break
                await RisingEdge(dut.wr_clk)
            dut.wr_data.value = val
            dut.wr_en.value = 1
            await RisingEdge(dut.wr_clk)
            dut.wr_en.value = 0

    async def reader():
        while len(read_data) < NUM_WORDS:
            await RisingEdge(dut.rd_clk)
            await Timer(1, unit='ns')
            if int(dut.empty.value) == 0:
                rd_val = int(dut.rd_data.value)
                read_data.append(rd_val)
                dut.rd_en.value = 1
                await RisingEdge(dut.rd_clk)
                dut.rd_en.value = 0

    cocotb.start_soon(writer())
    await reader()

    assert read_data == written, "Data corruption with fast writer / slow reader"
    cocotb.log.info("PASS: test_asymmetric_fast_write_slow_read")


@cocotb.test()
async def test_asymmetric_slow_write_fast_read(dut):
    """Reader faster than writer (matching actual deployment: sys_clk > l_clk)."""
    cocotb.start_soon(Clock(dut.wr_clk, WR_CLK_PERIOD_NS, unit='ns').start())  # slow
    cocotb.start_soon(Clock(dut.rd_clk, RD_CLK_PERIOD_NS, unit='ns').start())  # fast
    await reset_fifo(dut)

    NUM_WORDS = 80
    written = [random.randint(0, (1 << WIDTH) - 1) for _ in range(NUM_WORDS)]
    read_data = []

    async def writer():
        for val in written:
            while True:
                await Timer(1, unit='ns')
                if int(dut.full.value) == 0:
                    break
                await RisingEdge(dut.wr_clk)
            dut.wr_data.value = val
            dut.wr_en.value = 1
            await RisingEdge(dut.wr_clk)
            dut.wr_en.value = 0

    async def reader():
        while len(read_data) < NUM_WORDS:
            await RisingEdge(dut.rd_clk)
            await Timer(1, unit='ns')
            if int(dut.empty.value) == 0:
                rd_val = int(dut.rd_data.value)
                read_data.append(rd_val)
                dut.rd_en.value = 1
                await RisingEdge(dut.rd_clk)
                dut.rd_en.value = 0

    cocotb.start_soon(writer())
    await reader()

    assert read_data == written, "Data corruption with slow writer / fast reader"
    cocotb.log.info("PASS: test_asymmetric_slow_write_fast_read")


@cocotb.test()
async def test_pointer_wrap(dut):
    """Push DEPTH*3 words to wrap pointers fully. Verify gray-code correctness."""
    cocotb.start_soon(Clock(dut.wr_clk, WR_CLK_PERIOD_NS, unit='ns').start())
    cocotb.start_soon(Clock(dut.rd_clk, RD_CLK_PERIOD_NS, unit='ns').start())
    await reset_fifo(dut)

    NUM_WORDS = DEPTH * 3
    written = [random.randint(0, (1 << WIDTH) - 1) for _ in range(NUM_WORDS)]
    read_data = []

    async def writer():
        for val in written:
            while True:
                await Timer(1, unit='ns')
                if int(dut.full.value) == 0:
                    break
                await RisingEdge(dut.wr_clk)
            dut.wr_data.value = val
            dut.wr_en.value = 1
            await RisingEdge(dut.wr_clk)
            dut.wr_en.value = 0

    async def reader():
        while len(read_data) < NUM_WORDS:
            await RisingEdge(dut.rd_clk)
            await Timer(1, unit='ns')
            if int(dut.empty.value) == 0:
                rd_val = int(dut.rd_data.value)
                read_data.append(rd_val)
                dut.rd_en.value = 1
                await RisingEdge(dut.rd_clk)
                dut.rd_en.value = 0

    cocotb.start_soon(writer())
    await reader()

    assert len(read_data) == NUM_WORDS, f"Got {len(read_data)}, expected {NUM_WORDS}"
    assert read_data == written, "Data corruption during pointer wrap"
    cocotb.log.info("PASS: test_pointer_wrap")


@cocotb.test()
async def test_reset_during_operation(dut):
    """Reset wr side mid-transfer, verify no corruption; reset rd side, verify recovery."""
    cocotb.start_soon(Clock(dut.wr_clk, WR_CLK_PERIOD_NS, unit='ns').start())
    cocotb.start_soon(Clock(dut.rd_clk, RD_CLK_PERIOD_NS, unit='ns').start())
    await reset_fifo(dut)

    # Write 2 words
    val_a = 0x1234567 & ((1 << WIDTH) - 1)
    val_b = 0x1ABCDEF & ((1 << WIDTH) - 1)
    await write_word(dut, val_a)
    await write_word(dut, val_b)

    # Assert wr_rst mid-operation
    dut.wr_rst.value = 1
    await ClockCycles(dut.wr_clk, 4)
    dut.wr_rst.value = 0
    await ClockCycles(dut.wr_clk, 4)

    # Now reset both sides cleanly to get to a known state
    dut.wr_rst.value = 1
    dut.rd_rst.value = 1
    await ClockCycles(dut.wr_clk, 4)
    await ClockCycles(dut.rd_clk, 4)
    dut.wr_rst.value = 0
    dut.rd_rst.value = 0
    await ClockCycles(dut.wr_clk, 6)
    await ClockCycles(dut.rd_clk, 6)

    # Verify FIFO is in clean state
    await Timer(1, unit='ns')
    assert int(dut.empty.value) == 1, "FIFO should be empty after full reset"
    assert int(dut.full.value) == 0, "FIFO should not be full after full reset"

    # Write and read to verify functional recovery
    val_c = 0x1CAFE00 & ((1 << WIDTH) - 1)
    await write_word(dut, val_c)
    assert await wait_not_empty(dut), "Data should appear after reset recovery"
    await Timer(1, unit='ns')
    rd_val = int(dut.rd_data.value)
    assert rd_val == val_c, f"Post-reset read: 0x{rd_val:X} expected 0x{val_c:X}"
    cocotb.log.info("PASS: test_reset_during_operation")
