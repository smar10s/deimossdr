"""
Test LTF peak detector (ltf_peak_detect).

Tests:
1. Peak latch: triangle metric waveform → peak_found at correct position
2. Ack clear: peak_found deasserts on ack
3. Last-wins: second peak overwrites first when not ack'd
4. Below threshold: metric that never exceeds threshold → no peak_found
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles


# Default threshold matches the module parameter
THRESHOLD = 250000


async def reset_dut(dut):
    """Assert reset for 10 clocks."""
    dut.rst_n.value = 0
    dut.enable.value = 1
    dut.metric_valid.value = 0
    dut.metric.value = 0
    dut.sample_pos.value = 0
    dut.ack.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


async def drive_metric(dut, value, pos):
    """Drive one metric_valid pulse with given value and sample_pos."""
    dut.metric_valid.value = 1
    dut.metric.value = value
    dut.sample_pos.value = pos
    await RisingEdge(dut.clk)
    dut.metric_valid.value = 0
    # Wait 4 clocks (simulating 1-per-5 timing from correlator)
    await ClockCycles(dut.clk, 4)


@cocotb.test()
async def test_peak_latch(dut):
    """Triangle metric waveform → peak_found asserts with correct position."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Drive a triangle: ramp up then sharp drop
    # Peak at position 100 with metric = 4000000
    # After peak, drop immediately below threshold to avoid re-trigger
    positions = list(range(95, 108))
    metrics = []
    for p in positions:
        if p < 100:
            # Ramp up: 600k → 4M over 5 samples
            m = 600000 + (p - 95) * 680000
        elif p == 100:
            m = 4000000  # peak
        elif p == 101:
            m = 1800000  # sharp drop: below run_max/2 = 2M
        else:
            m = 100000   # well below threshold
        metrics.append(m)

    # Feed noise before
    for i in range(5):
        await drive_metric(dut, 50000, i)
    assert dut.peak_found.value == 0, "peak_found should be 0 during noise"

    # Feed triangle
    for p, m in zip(positions, metrics):
        await drive_metric(dut, m, p)

    # Feed some noise after to trigger the drop detection
    for i in range(5):
        await drive_metric(dut, 50000, 120 + i)

    # Check peak_found
    assert dut.peak_found.value == 1, "peak_found should be asserted after peak"
    peak_pos = int(dut.peak_sample_pos.value)
    peak_met = int(dut.peak_metric.value)
    dut._log.info(f"Peak found at pos={peak_pos}, metric={peak_met}")
    assert peak_pos == 100, f"Expected peak at 100, got {peak_pos}"
    assert peak_met == 4000000, f"Expected metric 4000000, got {peak_met}"


@cocotb.test()
async def test_ack_clear(dut):
    """Ack pulse clears peak_found."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Create a quick peak
    await drive_metric(dut, 50000, 0)
    await drive_metric(dut, 1000000, 1)
    await drive_metric(dut, 2000000, 2)  # peak
    await drive_metric(dut, 800000, 3)
    await drive_metric(dut, 100000, 4)   # drop below threshold

    # Should be found
    assert dut.peak_found.value == 1, "peak_found should be 1"

    # Ack it
    dut.ack.value = 1
    await RisingEdge(dut.clk)
    dut.ack.value = 0
    await RisingEdge(dut.clk)

    assert dut.peak_found.value == 0, "peak_found should be 0 after ack"


@cocotb.test()
async def test_last_wins(dut):
    """Second peak overwrites first when first hasn't been ack'd."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # First peak at position 10, metric 2M
    await drive_metric(dut, 50000, 0)
    await drive_metric(dut, 1000000, 8)
    await drive_metric(dut, 2000000, 10)  # peak 1
    await drive_metric(dut, 800000, 11)
    await drive_metric(dut, 100000, 12)   # drop → latch

    assert dut.peak_found.value == 1
    assert int(dut.peak_sample_pos.value) == 10
    dut._log.info("First peak latched at pos=10")

    # Second peak at position 50, metric 3M (no ack between)
    await drive_metric(dut, 50000, 40)
    await drive_metric(dut, 1500000, 48)
    await drive_metric(dut, 3000000, 50)  # peak 2
    await drive_metric(dut, 1200000, 51)
    await drive_metric(dut, 100000, 52)   # drop → overwrite

    assert dut.peak_found.value == 1
    peak_pos = int(dut.peak_sample_pos.value)
    peak_met = int(dut.peak_metric.value)
    dut._log.info(f"After second peak: pos={peak_pos}, metric={peak_met}")
    assert peak_pos == 50, f"Expected last-wins pos=50, got {peak_pos}"
    assert peak_met == 3000000, f"Expected metric 3000000, got {peak_met}"


@cocotb.test()
async def test_below_threshold(dut):
    """Metric never exceeds threshold → peak_found stays 0."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Feed 20 samples all below threshold
    for i in range(20):
        await drive_metric(dut, 200000 + i * 1000, i)

    assert dut.peak_found.value == 0, "peak_found should stay 0 below threshold"


async def search_stall_then_recover(dut):
    """Stall S_SEARCH past the watchdog, then verify a fresh triangle
    peak latches with the CORRECT position (a wedged FSM would keep the
    stale run_max/run_max_pos and report the wrong peak)."""
    # Fresh triangle peak at pos 200, metric 3M
    await drive_metric(dut, 50000, 190)
    await drive_metric(dut, 1500000, 195)
    await drive_metric(dut, 3000000, 200)  # peak
    await drive_metric(dut, 1400000, 201)  # below run_max/2 = 1.5M
    await drive_metric(dut, 100000, 202)

    assert dut.peak_found.value == 1, "peak_found should assert after recovery"
    peak_pos = int(dut.peak_sample_pos.value)
    peak_met = int(dut.peak_metric.value)
    assert peak_pos == 200, f"Expected peak pos=200 after recovery, got {peak_pos}"
    assert peak_met == 3000000, f"Expected metric 3000000, got {peak_met}"
    dut._log.info(f"Recovered: peak at pos={peak_pos}, metric={peak_met}")


@cocotb.test()
async def test_search_timeout_metric_stop(dut):
    """metric_valid stops dead mid-search → watchdog returns to S_IDLE and
    a later peak is tracked cleanly."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Rise above threshold, then the metric stream dies mid-search
    await drive_metric(dut, 2000000, 10)

    # Stall: no metric_valid for the full watchdog + margin
    await ClockCycles(dut.clk, 4096 + 100)
    assert dut.peak_found.value == 0, "no peak should latch during the stall"

    await search_stall_then_recover(dut)


@cocotb.test()
async def test_search_timeout_plateau(dut):
    """Metric stays above threshold without ever dropping below max/2
    (sustained plateau) → watchdog bounds the search; a later clean peak
    latches correctly."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Plateau: constant metric above threshold, never dropping
    # (metric_valid keeps pulsing every 5 clocks — the watchdog counts
    # wall clocks, not metric pulses)
    plateau_cycles = (4096 // 5) + 100
    for i in range(plateau_cycles):
        await drive_metric(dut, 1000000, i)
    assert dut.peak_found.value == 0, "no peak should latch on a plateau"

    await search_stall_then_recover(dut)
