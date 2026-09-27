"""Cocotb tests for snap_axi — AXI-Lite wrapper around debug_snap."""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
from axi_lite_driver import AxiLiteMaster

# Register offsets
REG_CONTROL    = 0x00
REG_STATUS     = 0x04
REG_TRIG_CYCLE = 0x08
REG_RD_ADDR    = 0x0C
REG_RD_DATA    = 0x10

# CONTROL bits
CTRL_ARM       = 1 << 0
CTRL_SW_TRIG   = 1 << 1
CTRL_CIRCULAR  = 1 << 2

# STATUS bits
STATUS_CAPTURED = 1 << 0
STATUS_ARMED    = 1 << 1

POST_DEPTH = 512


async def reset(dut):
    """Drive rst=1 for 5 cycles, deassert, wait 3 cycles."""
    dut.sample_data.value = 0
    dut.sample_valid.value = 0
    dut.ext_trig.value = 0
    dut.rst.value = 1
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 3)


async def feed_samples(dut, start_val, count):
    """Feed `count` sequential samples starting at start_val."""
    for i in range(count):
        dut.sample_data.value = (start_val + i) & 0xFFFFFFFF
        dut.sample_valid.value = 1
        await RisingEdge(dut.clk)
    dut.sample_valid.value = 0
    await RisingEdge(dut.clk)


async def setup(dut):
    """Start clock, reset, return AXI master."""
    clock = Clock(dut.clk, 10, unit='ns')
    cocotb.start_soon(clock.start())
    axi = AxiLiteMaster(dut, prefix="s_axi_", clk=dut.clk)
    await axi._init_signals()
    await reset(dut)
    return axi


@cocotb.test()
async def test_arm_and_capture(dut):
    """Arm via CONTROL, feed 200 samples, sw_trigger, feed POST_DEPTH more, verify capture."""
    axi = await setup(dut)

    # Arm
    await axi.write(REG_CONTROL, CTRL_ARM)

    # Verify armed in STATUS
    status, _ = await axi.read(REG_STATUS)
    assert status & STATUS_ARMED, f"Expected armed, got STATUS=0x{status:08x}"

    # Feed 200 pre-trigger samples
    await feed_samples(dut, 0, 200)

    # Software trigger
    await axi.write(REG_CONTROL, CTRL_ARM | CTRL_SW_TRIG)

    # Feed POST_DEPTH samples to complete capture
    await feed_samples(dut, 200, POST_DEPTH)

    # Wait a few cycles for capture to settle
    await ClockCycles(dut.clk, 5)

    # Check STATUS: captured=1, trig_pos should reflect write pointer at trigger time
    status, _ = await axi.read(REG_STATUS)
    assert status & STATUS_CAPTURED, f"Expected captured, got STATUS=0x{status:08x}"

    # trig_pos is STATUS[25:16]
    trig_pos = (status >> 16) & 0x3FF
    # trig_pos should be 200 (the write address at trigger time)
    assert trig_pos == 200, f"Expected trig_pos=200, got {trig_pos}"


@cocotb.test()
async def test_sw_trigger(dut):
    """Arm, feed 50 samples, sw_trigger, feed POST_DEPTH, verify capture completes."""
    axi = await setup(dut)

    # Arm
    await axi.write(REG_CONTROL, CTRL_ARM)

    # Feed 50 pre-trigger samples
    await feed_samples(dut, 0, 50)

    # Software trigger
    await axi.write(REG_CONTROL, CTRL_ARM | CTRL_SW_TRIG)

    # Feed POST_DEPTH samples
    await feed_samples(dut, 50, POST_DEPTH)

    await ClockCycles(dut.clk, 5)

    # Verify captured
    status, _ = await axi.read(REG_STATUS)
    assert status & STATUS_CAPTURED, f"Expected captured, got STATUS=0x{status:08x}"

    # trig_pos should be 50
    trig_pos = (status >> 16) & 0x3FF
    assert trig_pos == 50, f"Expected trig_pos=50, got {trig_pos}"


@cocotb.test()
async def test_readback(dut):
    """Feed known data, trigger, complete capture, sweep RD_ADDR and verify RD_DATA."""
    axi = await setup(dut)

    # Arm
    await axi.write(REG_CONTROL, CTRL_ARM)

    # Feed 100 samples with known values (0..99)
    await feed_samples(dut, 0, 100)

    # Software trigger
    await axi.write(REG_CONTROL, CTRL_ARM | CTRL_SW_TRIG)

    # Feed POST_DEPTH samples to complete capture (values 100..611)
    await feed_samples(dut, 100, POST_DEPTH)

    await ClockCycles(dut.clk, 5)

    # Verify captured
    status, _ = await axi.read(REG_STATUS)
    assert status & STATUS_CAPTURED, f"Not captured: STATUS=0x{status:08x}"

    # Read back addresses 0..99 and verify sequential data
    for addr in range(100):
        await axi.write(REG_RD_ADDR, addr)
        data, _ = await axi.read(REG_RD_DATA)
        assert data == addr, f"RD_DATA[{addr}] = {data}, expected {addr}"


@cocotb.test()
async def test_rearm(dut):
    """First capture, re-arm, second capture with new data, verify second data."""
    axi = await setup(dut)

    # --- First capture ---
    await axi.write(REG_CONTROL, CTRL_ARM)
    await feed_samples(dut, 0, 100)
    await axi.write(REG_CONTROL, CTRL_ARM | CTRL_SW_TRIG)
    await feed_samples(dut, 100, POST_DEPTH)
    await ClockCycles(dut.clk, 5)

    status, _ = await axi.read(REG_STATUS)
    assert status & STATUS_CAPTURED, "First capture failed"

    # Verify first capture data at address 0
    await axi.write(REG_RD_ADDR, 0)
    data, _ = await axi.read(REG_RD_DATA)
    assert data == 0, f"First capture addr 0: expected 0, got {data}"

    # --- Re-arm ---
    await axi.write(REG_CONTROL, CTRL_ARM)
    await ClockCycles(dut.clk, 5)

    # Verify re-armed: captured should be cleared
    status, _ = await axi.read(REG_STATUS)
    assert not (status & STATUS_CAPTURED), f"Still captured after rearm: STATUS=0x{status:08x}"
    assert status & STATUS_ARMED, f"Not armed after rearm: STATUS=0x{status:08x}"

    # --- Second capture with different data ---
    await feed_samples(dut, 1000, 100)
    await axi.write(REG_CONTROL, CTRL_ARM | CTRL_SW_TRIG)
    await feed_samples(dut, 1100, POST_DEPTH)
    await ClockCycles(dut.clk, 5)

    status, _ = await axi.read(REG_STATUS)
    assert status & STATUS_CAPTURED, "Second capture failed"

    # Verify second capture data at address 0
    await axi.write(REG_RD_ADDR, 0)
    data, _ = await axi.read(REG_RD_DATA)
    assert data == 1000, f"Second capture addr 0: expected 1000, got {data}"


@cocotb.test()
async def test_circular(dut):
    """Circular mode: feed >1024 samples, captured stays 0. Trigger updates trig_pos."""
    axi = await setup(dut)

    # Arm with circular mode enabled
    await axi.write(REG_CONTROL, CTRL_ARM | CTRL_CIRCULAR)

    # Feed 1200 samples (more than DEPTH=1024)
    await feed_samples(dut, 0, 1200)

    # Verify NOT captured (circular suppresses capture)
    status, _ = await axi.read(REG_STATUS)
    assert not (status & STATUS_CAPTURED), f"Circular should not capture: STATUS=0x{status:08x}"

    # Fire software trigger
    await axi.write(REG_CONTROL, CTRL_ARM | CTRL_CIRCULAR | CTRL_SW_TRIG)

    # Feed more samples after trigger
    await feed_samples(dut, 1200, POST_DEPTH + 100)

    await ClockCycles(dut.clk, 5)

    # Still NOT captured
    status, _ = await axi.read(REG_STATUS)
    assert not (status & STATUS_CAPTURED), f"Circular should still not capture: STATUS=0x{status:08x}"

    # trig_pos should have been updated (should be 1200 mod 1024 = 176)
    trig_pos = (status >> 16) & 0x3FF
    # The write address at trigger time: 1200 samples fed, wr_addr = 1200 % 1024 = 176
    assert trig_pos == (1200 % 1024), f"Expected trig_pos={1200 % 1024}, got {trig_pos}"


@cocotb.test()
async def test_ext_trigger(dut):
    """Arm, feed samples, assert ext_trig port, verify capture completes."""
    axi = await setup(dut)

    # Arm
    await axi.write(REG_CONTROL, CTRL_ARM)

    # Feed 150 pre-trigger samples
    await feed_samples(dut, 0, 150)

    # Assert external trigger for 1 cycle
    dut.ext_trig.value = 1
    await RisingEdge(dut.clk)
    dut.ext_trig.value = 0
    await RisingEdge(dut.clk)

    # Feed POST_DEPTH samples to complete capture
    await feed_samples(dut, 150, POST_DEPTH)

    await ClockCycles(dut.clk, 5)

    # Verify captured
    status, _ = await axi.read(REG_STATUS)
    assert status & STATUS_CAPTURED, f"Expected captured with ext_trig, got STATUS=0x{status:08x}"

    # trig_pos should be 150
    trig_pos = (status >> 16) & 0x3FF
    assert trig_pos == 150, f"Expected trig_pos=150, got {trig_pos}"
