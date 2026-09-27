"""Cocotb tests for adc_sync — ADC IQ clock domain crossing."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer
import random

# Match real deployment clocks
ADC_CLK_PERIOD_NS = 16.27  # l_clk ~ 61.44 MHz
SYS_CLK_PERIOD_NS = 10.0   # sys_clk = 100 MHz

SAMPLE_WIDTH = 12
MAX_UNSIGNED = (1 << SAMPLE_WIDTH) - 1  # 0xFFF


def to_unsigned(signed_val, width=SAMPLE_WIDTH):
    """Convert a signed value to its unsigned representation."""
    if signed_val < 0:
        return signed_val + (1 << width)
    return signed_val


def to_signed(unsigned_val, width=SAMPLE_WIDTH):
    """Convert unsigned to signed (two's complement)."""
    if unsigned_val >= (1 << (width - 1)):
        return unsigned_val - (1 << width)
    return unsigned_val


async def reset_dut(dut):
    """Assert both resets for several cycles, then release."""
    dut.adc_rst.value = 1
    dut.sys_rst.value = 1
    dut.valid_in.value = 0
    dut.re_in.value = 0
    dut.im_in.value = 0
    await ClockCycles(dut.adc_clk, 8)
    await ClockCycles(dut.sys_clk, 8)
    dut.adc_rst.value = 0
    dut.sys_rst.value = 0
    # Allow synchronizers to settle
    await ClockCycles(dut.adc_clk, 6)
    await ClockCycles(dut.sys_clk, 6)


async def push_sample(dut, re_val, im_val):
    """Push one IQ sample on the ADC clock edge."""
    dut.re_in.value = to_unsigned(re_val) if re_val < 0 else re_val
    dut.im_in.value = to_unsigned(im_val) if im_val < 0 else im_val
    dut.valid_in.value = 1
    await RisingEdge(dut.adc_clk)
    dut.valid_in.value = 0


async def collect_outputs(dut, count, timeout_cycles=5000):
    """Collect `count` output samples from the sys_clk side."""
    results = []
    cycles = 0
    while len(results) < count and cycles < timeout_cycles:
        await RisingEdge(dut.sys_clk)
        await Timer(1, unit='ns')
        if int(dut.valid_out.value) == 1:
            re = to_signed(int(dut.re_out.value) & MAX_UNSIGNED)
            im = to_signed(int(dut.im_out.value) & MAX_UNSIGNED)
            results.append((re, im))
        cycles += 1
    return results


# =========================================================
# Tests
# =========================================================

@cocotb.test()
async def test_basic_crossing(dut):
    """Push one sample, verify it appears on sys_clk side with correct values."""
    cocotb.start_soon(Clock(dut.adc_clk, ADC_CLK_PERIOD_NS, unit='ns').start())
    cocotb.start_soon(Clock(dut.sys_clk, SYS_CLK_PERIOD_NS, unit='ns').start())
    await reset_dut(dut)

    re_val = 0x123
    im_val = 0x456
    await push_sample(dut, re_val, im_val)

    results = await collect_outputs(dut, 1, timeout_cycles=100)
    assert len(results) == 1, f"Expected 1 output, got {len(results)}"
    assert results[0] == (to_signed(re_val), to_signed(im_val)), \
        f"Got {results[0]}, expected ({to_signed(re_val)}, {to_signed(im_val)})"
    cocotb.log.info("PASS: test_basic_crossing")


@cocotb.test()
async def test_continuous_stream(dut):
    """Stream 200 samples at full rate, verify all arrive in order, no drops."""
    cocotb.start_soon(Clock(dut.adc_clk, ADC_CLK_PERIOD_NS, unit='ns').start())
    cocotb.start_soon(Clock(dut.sys_clk, SYS_CLK_PERIOD_NS, unit='ns').start())
    await reset_dut(dut)

    NUM_SAMPLES = 200
    # Generate random 12-bit signed samples
    tx_data = [(random.randint(0, MAX_UNSIGNED), random.randint(0, MAX_UNSIGNED))
               for _ in range(NUM_SAMPLES)]

    async def driver():
        for re_val, im_val in tx_data:
            dut.re_in.value = re_val
            dut.im_in.value = im_val
            dut.valid_in.value = 1
            await RisingEdge(dut.adc_clk)
        dut.valid_in.value = 0

    cocotb.start_soon(driver())
    results = await collect_outputs(dut, NUM_SAMPLES, timeout_cycles=NUM_SAMPLES * 5)

    assert len(results) == NUM_SAMPLES, \
        f"Dropped samples: got {len(results)}/{NUM_SAMPLES}"

    for i, ((exp_re, exp_im), (got_re, got_im)) in enumerate(zip(tx_data, results)):
        exp_re_s = to_signed(exp_re)
        exp_im_s = to_signed(exp_im)
        assert got_re == exp_re_s and got_im == exp_im_s, \
            f"Sample {i}: got ({got_re},{got_im}) expected ({exp_re_s},{exp_im_s})"

    cocotb.log.info("PASS: test_continuous_stream")


@cocotb.test()
async def test_gapped_input(dut):
    """valid_in asserted every 3rd cycle, verify output tracks correctly."""
    cocotb.start_soon(Clock(dut.adc_clk, ADC_CLK_PERIOD_NS, unit='ns').start())
    cocotb.start_soon(Clock(dut.sys_clk, SYS_CLK_PERIOD_NS, unit='ns').start())
    await reset_dut(dut)

    NUM_SAMPLES = 50
    tx_data = [(random.randint(0, MAX_UNSIGNED), random.randint(0, MAX_UNSIGNED))
               for _ in range(NUM_SAMPLES)]

    async def driver():
        for re_val, im_val in tx_data:
            dut.re_in.value = re_val
            dut.im_in.value = im_val
            dut.valid_in.value = 1
            await RisingEdge(dut.adc_clk)
            dut.valid_in.value = 0
            # Gap: 2 idle cycles
            await RisingEdge(dut.adc_clk)
            await RisingEdge(dut.adc_clk)

    cocotb.start_soon(driver())
    results = await collect_outputs(dut, NUM_SAMPLES, timeout_cycles=NUM_SAMPLES * 10)

    assert len(results) == NUM_SAMPLES, \
        f"Dropped samples with gaps: got {len(results)}/{NUM_SAMPLES}"

    for i, ((exp_re, exp_im), (got_re, got_im)) in enumerate(zip(tx_data, results)):
        exp_re_s = to_signed(exp_re)
        exp_im_s = to_signed(exp_im)
        assert got_re == exp_re_s and got_im == exp_im_s, \
            f"Sample {i}: got ({got_re},{got_im}) expected ({exp_re_s},{exp_im_s})"

    cocotb.log.info("PASS: test_gapped_input")


@cocotb.test()
async def test_sign_extension(dut):
    """Push negative values, verify sign preserved on output."""
    cocotb.start_soon(Clock(dut.adc_clk, ADC_CLK_PERIOD_NS, unit='ns').start())
    cocotb.start_soon(Clock(dut.sys_clk, SYS_CLK_PERIOD_NS, unit='ns').start())
    await reset_dut(dut)

    # Test specific negative values
    test_cases = [
        (0xFFF, 0xFFF),  # -1, -1
        (0x800, 0x800),  # -2048, -2048
        (0x801, 0xFFE),  # -2047, -2
        (0x000, 0x7FF),  # 0, +2047
        (0x001, 0x400),  # +1, +1024
    ]

    async def driver():
        for re_val, im_val in test_cases:
            dut.re_in.value = re_val
            dut.im_in.value = im_val
            dut.valid_in.value = 1
            await RisingEdge(dut.adc_clk)
        dut.valid_in.value = 0

    cocotb.start_soon(driver())
    results = await collect_outputs(dut, len(test_cases), timeout_cycles=200)

    assert len(results) == len(test_cases), \
        f"Missing outputs: got {len(results)}/{len(test_cases)}"

    expected = [(to_signed(re), to_signed(im)) for re, im in test_cases]
    for i, (exp, got) in enumerate(zip(expected, results)):
        assert got == exp, f"Case {i}: got {got} expected {exp}"

    cocotb.log.info("PASS: test_sign_extension")


@cocotb.test()
async def test_reset_recovery(dut):
    """Assert adc_rst mid-stream, release, verify no stale data appears."""
    cocotb.start_soon(Clock(dut.adc_clk, ADC_CLK_PERIOD_NS, unit='ns').start())
    cocotb.start_soon(Clock(dut.sys_clk, SYS_CLK_PERIOD_NS, unit='ns').start())
    await reset_dut(dut)

    # Push a few samples
    for i in range(3):
        await push_sample(dut, i * 100, i * 200)

    # Wait for them to cross
    await ClockCycles(dut.sys_clk, 20)

    # Assert adc_rst (and sys_rst to clear output regs)
    dut.adc_rst.value = 1
    dut.sys_rst.value = 1
    await ClockCycles(dut.adc_clk, 6)
    await ClockCycles(dut.sys_clk, 6)
    dut.adc_rst.value = 0
    dut.sys_rst.value = 0
    await ClockCycles(dut.adc_clk, 8)
    await ClockCycles(dut.sys_clk, 8)

    # After reset, valid_out should be 0 (no stale data)
    await Timer(1, unit='ns')
    assert int(dut.valid_out.value) == 0, "valid_out should be 0 after reset"

    # Push fresh data and verify it arrives correctly
    fresh_re = 0x111
    fresh_im = 0x222
    await push_sample(dut, fresh_re, fresh_im)

    results = await collect_outputs(dut, 1, timeout_cycles=100)
    assert len(results) == 1, "Fresh sample should arrive after reset"
    assert results[0] == (to_signed(fresh_re), to_signed(fresh_im)), \
        f"Post-reset data: got {results[0]} expected ({to_signed(fresh_re)}, {to_signed(fresh_im)})"
    cocotb.log.info("PASS: test_reset_recovery")


@cocotb.test()
async def test_high_throughput(dut):
    """Run 10000 samples at full rate, verify zero drops."""
    cocotb.start_soon(Clock(dut.adc_clk, ADC_CLK_PERIOD_NS, unit='ns').start())
    cocotb.start_soon(Clock(dut.sys_clk, SYS_CLK_PERIOD_NS, unit='ns').start())
    await reset_dut(dut)

    NUM_SAMPLES = 10000
    # Use a PRNG with fixed seed for reproducibility
    rng = random.Random(42)
    tx_data = [(rng.randint(0, MAX_UNSIGNED), rng.randint(0, MAX_UNSIGNED))
               for _ in range(NUM_SAMPLES)]

    async def driver():
        for re_val, im_val in tx_data:
            dut.re_in.value = re_val
            dut.im_in.value = im_val
            dut.valid_in.value = 1
            await RisingEdge(dut.adc_clk)
        dut.valid_in.value = 0

    cocotb.start_soon(driver())
    results = await collect_outputs(dut, NUM_SAMPLES, timeout_cycles=NUM_SAMPLES * 5)

    assert len(results) == NUM_SAMPLES, \
        f"Dropped samples in stress test: got {len(results)}/{NUM_SAMPLES}"

    mismatches = 0
    for i, ((exp_re, exp_im), (got_re, got_im)) in enumerate(zip(tx_data, results)):
        exp_re_s = to_signed(exp_re)
        exp_im_s = to_signed(exp_im)
        if got_re != exp_re_s or got_im != exp_im_s:
            if mismatches < 5:
                cocotb.log.error(
                    f"Sample {i}: got ({got_re},{got_im}) expected ({exp_re_s},{exp_im_s})")
            mismatches += 1

    assert mismatches == 0, f"{mismatches} samples corrupted out of {NUM_SAMPLES}"
    cocotb.log.info("PASS: test_high_throughput — 10000 samples, zero drops")
