"""cocotb tests for debug_snap parametric snapshot buffer."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer


DEPTH = 1024
POST_DEPTH = 512


async def reset_dut(dut):
    dut.rst.value = 1
    dut.sample_valid.value = 0
    dut.sample_data.value = 0
    dut.trig.value = 0
    dut.rearm.value = 0
    dut.circular_en.value = 0
    dut.rd_addr.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 3)


async def do_write(dut, data):
    """Write one sample and return after the posedge."""
    dut.sample_data.value = data
    dut.sample_valid.value = 1
    await RisingEdge(dut.clk)


async def write_samples(dut, start, count):
    """Write `count` samples. Leaves sample_valid=0 with a settling cycle."""
    for i in range(count):
        await do_write(dut, start + i)
    dut.sample_valid.value = 0
    await RisingEdge(dut.clk)
    await Timer(1, unit='ns')


async def read_br(dut, addr):
    """Read BRAM at `addr` (2-cycle pipeline)."""
    dut.rd_addr.value = addr
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await Timer(1, unit='ns')
    return int(dut.rd_data.value)


async def read_all(dut, base=0, count=DEPTH):
    result = []
    for i in range(count):
        v = await read_br(dut, (base + i) & (DEPTH - 1))
        result.append(v)
    return result


async def get_state(dut):
    """Read status registers with delta-cycle settle."""
    await Timer(1, unit='ns')
    return (
        int(dut.captured.value),
        int(dut.trig_wr.value),
        int(dut.trig_cycle.value),
        int(dut.wr_count.value),
    )


# =========================================================
# Tests
# =========================================================

@cocotb.test()
async def test_basic_capture(dut):
    """200 pre-trigger + trigger + 512 post-trigger. Verify all."""
    clock = Clock(dut.clk, 10, unit='ns')
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    await write_samples(dut, 0, 200)

    dut.trig.value = 1
    await RisingEdge(dut.clk)
    dut.trig.value = 0
    await Timer(1, unit='ns')

    captured, trig_wr_val, _, wr_cnt = await get_state(dut)
    cocotb.log.info(f"trig_wr={trig_wr_val} wr_cnt={wr_cnt}")
    assert captured == 0
    assert trig_wr_val == 200

    await write_samples(dut, 200, POST_DEPTH)

    captured, _, _, _ = await get_state(dut)
    assert captured == 1, "should capture after POST_DEPTH + idle"

    all_data = await read_all(dut, 0, DEPTH)
    assert all_data[0] == 0
    assert all_data[100] == 100
    assert all_data[199] == 199
    assert all_data[200] == 200
    assert all_data[400] == 400
    cocotb.log.info("PASS: basic_capture")


@cocotb.test()
async def test_trig_simple(dut):
    """5 writes, trigger — verify trig_wr and pre-trigger data."""
    clock = Clock(dut.clk, 10, unit='ns')
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    await write_samples(dut, 10, 5)

    dut.trig.value = 1
    await RisingEdge(dut.clk)
    dut.trig.value = 0
    await Timer(1, unit='ns')

    _, trig_wr_val, _, _ = await get_state(dut)
    assert trig_wr_val == 5

    all_data = await read_all(dut, 0, 6)
    assert all_data[0] == 10
    assert all_data[4] == 14
    cocotb.log.info("PASS: trig_simple")


@cocotb.test()
async def test_trig_with_sample(dut):
    """Trigger concurrent with sample_valid — that sample is pre-trigger."""
    clock = Clock(dut.clk, 10, unit='ns')
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    await write_samples(dut, 0, 5)

    dut.sample_valid.value = 1
    dut.sample_data.value = 5
    dut.trig.value = 1
    await RisingEdge(dut.clk)
    dut.trig.value = 0
    dut.sample_valid.value = 0
    await Timer(1, unit='ns')

    _, trig_wr_val, _, _ = await get_state(dut)
    assert trig_wr_val == 5, "concurrent sample should be pre-trigger"

    await write_samples(dut, 6, POST_DEPTH)
    captured, _, _, _ = await get_state(dut)
    assert captured == 1

    all_data = await read_all(dut, 0, DEPTH)
    assert all_data[5] == 5, f"addr[5]={all_data[5]} expected 5"
    assert all_data[6] == 6, f"addr[6]={all_data[6]} expected 6"
    cocotb.log.info("PASS: trig_with_sample")


@cocotb.test()
async def test_rearm(dut):
    """Capture, rearm, capture again."""
    clock = Clock(dut.clk, 10, unit='ns')
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    await write_samples(dut, 0, 20)
    dut.trig.value = 1
    await RisingEdge(dut.clk)
    dut.trig.value = 0
    await write_samples(dut, 20, POST_DEPTH)
    _, trig1, _, _ = await get_state(dut)
    assert trig1 == 20

    dut.rearm.value = 1
    await RisingEdge(dut.clk)
    dut.rearm.value = 0
    await ClockCycles(dut.clk, 3)
    captured, _, _, _ = await get_state(dut)
    assert captured == 0, "should not be captured after rearm"

    await write_samples(dut, 500, 20)
    dut.trig.value = 1
    await RisingEdge(dut.clk)
    dut.trig.value = 0
    await write_samples(dut, 520, POST_DEPTH)
    captured2, trig2, _, _ = await get_state(dut)
    cocotb.log.info(f"trig1={trig1} trig2={trig2}")
    assert captured2 == 1, "second capture should complete"

    all_data = await read_all(dut, 0, 20)
    assert all_data[0] == 500, f"second capture addr[0]={all_data[0]}"
    assert all_data[19] == 519, f"second capture addr[19]={all_data[19]}"
    cocotb.log.info("PASS: rearm")


@cocotb.test()
async def test_wrap_around(dut):
    """Write DEPTH+100, trigger — verify wrap overwrites."""
    clock = Clock(dut.clk, 10, unit='ns')
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    await write_samples(dut, 0, DEPTH + 100)

    dut.trig.value = 1
    await RisingEdge(dut.clk)
    dut.trig.value = 0
    await write_samples(dut, DEPTH + 100, POST_DEPTH)

    captured, trig_wr_val, _, _ = await get_state(dut)
    assert captured == 1

    all_data = await read_all(dut, 0, DEPTH)
    assert all_data[0] == DEPTH, f"addr[0]={all_data[0]} expected {DEPTH}"
    cocotb.log.info("PASS: wrap_around")


@cocotb.test()
async def test_wr_count_saturates(dut):
    """wr_count stops at DEPTH."""
    clock = Clock(dut.clk, 10, unit='ns')
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    await write_samples(dut, 0, DEPTH + 100)
    _, _, _, wr_cnt = await get_state(dut)
    assert wr_cnt == DEPTH, f"wr_cnt={wr_cnt}"
    cocotb.log.info("PASS: wr_count_saturates")


@cocotb.test()
async def test_post_trigger_exact(dut):
    """POST_DEPTH post-trigger writes + idle freezes the buffer."""
    clock = Clock(dut.clk, 10, unit='ns')
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    await write_samples(dut, 0, 50)

    dut.trig.value = 1
    await RisingEdge(dut.clk)
    dut.trig.value = 0
    await Timer(1, unit='ns')

    for i in range(POST_DEPTH - 1):
        await do_write(dut, 50 + i)
    dut.sample_valid.value = 0
    await RisingEdge(dut.clk)
    await Timer(1, unit='ns')
    captured, _, _, _ = await get_state(dut)
    assert captured == 0, "not frozen after POST_DEPTH-1"

    await do_write(dut, 50 + POST_DEPTH - 1)
    dut.sample_valid.value = 0
    await RisingEdge(dut.clk)
    await Timer(1, unit='ns')
    captured, _, _, _ = await get_state(dut)
    assert captured == 1, "frozen after POST_DEPTH + idle"
    cocotb.log.info("PASS: post_trigger_exact")


# =========================================================
# 32-bit wide data test
# =========================================================

@cocotb.test()
async def test_wide_data(dut):
    """32-bit data values survive BRAM write/read cycle."""
    clock = Clock(dut.clk, 10, unit='ns')
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    test_vals = [0xAAAA_BBBB, 0x1111_2222, 0xDEAD_BEEF,
                 0xFFFF_FFFF, 0x0000_0001]
    for i, v in enumerate(test_vals):
        dut.sample_data.value = v
        dut.sample_valid.value = 1
        await RisingEdge(dut.clk)
    dut.sample_valid.value = 0
    await RisingEdge(dut.clk)
    await Timer(1, unit='ns')

    for i, v in enumerate(test_vals):
        rd = await read_br(dut, i)
        assert rd == v, f"addr[{i}]: rd=0x{rd:08X} expected=0x{v:08X}"
    cocotb.log.info("PASS: wide_data")


# =========================================================
# Circular mode tests
# =========================================================

@cocotb.test()
async def test_circular_never_captures(dut):
    """Circular mode: captured never asserts, triggers are re-triggerable."""
    clock = Clock(dut.clk, 10, unit='ns')
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    dut.circular_en.value = 1

    await write_samples(dut, 0, 200)

    # First trigger
    dut.trig.value = 1
    await RisingEdge(dut.clk)
    dut.trig.value = 0
    await Timer(1, unit='ns')

    captured, trig1, _, _ = await get_state(dut)
    assert captured == 0, "circular: should not capture"
    assert trig1 == 200

    # Write POST_DEPTH+100 more — captured should never assert
    await write_samples(dut, 200, POST_DEPTH + 100)
    captured, _, _, _ = await get_state(dut)
    assert captured == 0, "circular: should never assert captured"

    # Second trigger overwrites trig_wr
    dut.trig.value = 1
    await RisingEdge(dut.clk)
    dut.trig.value = 0
    await Timer(1, unit='ns')

    captured, trig2, _, _ = await get_state(dut)
    assert captured == 0
    assert trig2 != trig1, "circular: second trigger should update trig_wr"
    cocotb.log.info("PASS: circular_never_captures")


@cocotb.test()
async def test_circular_wrap_overwrite(dut):
    """Circular mode: writing >DEPTH wraps and data is overwritten."""
    clock = Clock(dut.clk, 10, unit='ns')
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    dut.circular_en.value = 1

    # Fill buffer with 0..1023
    await write_samples(dut, 0, DEPTH)

    # Write 100 more: first 100 entries are overwritten
    await write_samples(dut, DEPTH, 100)

    # Addr 0 should now hold 1024 (DEPTH), not 0
    rd = await read_br(dut, 0)
    assert rd == DEPTH, f"circular wrap: addr[0]={rd} expected {DEPTH}"

    # Addr 99 should hold 1123
    rd = await read_br(dut, 99)
    assert rd == DEPTH + 99, f"circular wrap: addr[99]={rd} expected {DEPTH + 99}"

    # Trigger during continuous writes
    dut.trig.value = 1
    await RisingEdge(dut.clk)
    dut.trig.value = 0
    await Timer(1, unit='ns')

    captured, trig_wr_val, _, _ = await get_state(dut)
    assert captured == 0
    cocotb.log.info(f"trig_wr during circular wrap: {trig_wr_val}")
    cocotb.log.info("PASS: circular_wrap_overwrite")


@cocotb.test()
async def test_circular_rearm(dut):
    """Circular mode: rearm resets buffer and state."""
    clock = Clock(dut.clk, 10, unit='ns')
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    dut.circular_en.value = 1

    await write_samples(dut, 0, 500)

    dut.trig.value = 1
    await RisingEdge(dut.clk)
    dut.trig.value = 0
    await Timer(1, unit='ns')

    _, trig_before, _, _ = await get_state(dut)
    assert trig_before == 500

    # Rearm
    dut.rearm.value = 1
    await RisingEdge(dut.clk)
    dut.rearm.value = 0
    await ClockCycles(dut.clk, 3)

    captured, trig_wr_val, _, wr_cnt = await get_state(dut)
    assert captured == 0, "rearm in circular: still not captured"
    assert trig_wr_val == 0, "rearm in circular: trig_wr reset"
    assert wr_cnt == 0, "rearm in circular: wr_cnt reset"

    # New data should be fresh (from 0)
    await write_samples(dut, 1000, 10)
    rd = await read_br(dut, 0)
    assert rd == 1000, f"fresh data after rearm: {rd}"
    cocotb.log.info("PASS: circular_rearm")
