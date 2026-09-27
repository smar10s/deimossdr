"""
test_latency_ratchet.py — Latency regression gate tests.

RATCHET TESTS. Assert that per-symbol compute windows do not exceed
established baselines. These prevent silent latency regressions when
RTL is modified — any change that makes the pipeline slower will fail
here before it can reach hardware.

Unlike test_adc_replay (which is a correctness ratchet on decode results),
this is a *performance* ratchet on pipeline throughput.

Baselines are the feed-independent compute window
(`deint_done - symbol_start`), NOT the `symbol_start` period. Once the
pipeline is faster than the live feed the period clamps near 400 and can no
longer show further improvement (or a wrap of it). The compute window is
measured entirely inside one symbol's decode, so it is feed-independent.
Helper: `frontend_helpers.measure_compute_window`.

Baseline history: latency_baseline.jsonl. Steady-state periods were
366/390/486 (rates 6/12/24) from the pipe3a carry-chain cut (b345efd,
2026-08-24) until the streaming demapper (lever C, 2026-09-16), which cut the
compute windows to 314/338/386 and the periods to 327/340/388. Before pipe3a
they were 365/389/485 (lever 2a-prime), 391/439/583 (demap-deint
parallelization), and 390/486/678 (feat/eapol-capture). The ratchet only
tightens: if you improve latency, update the baselines here.

Target: 400 clocks/symbol (wire-speed at 100 MHz fabric / 20 MSPS ADC).
"""

import cocotb

from frontend_helpers import measure_compute_window, measure_symbol_cycle

# Feed-independent compute-window ratchets: measured value + 4 clocks margin.
# Measured 2026-09-16 after lever C: 6: 314, 12: 338, 24: 386.
RATCHET_COMPUTE = {6: 318, 12: 342, 24: 390}

# Wire-speed bound on the symbol_start period. The period may be feed-limited
# at ~400 once the pipeline is under budget; it is never an exact sub-400.
TARGET_RATE24 = 400


async def _check_compute(dut, rate):
    avg, _, decoded = await measure_compute_window(dut, rate=rate)
    assert decoded, f"rate {rate} frame did not decode to tag_valid"
    bound = RATCHET_COMPUTE[rate]
    dut._log.info(f"Rate {rate} compute window: {avg:.1f} clk/sym "
                  f"(ratchet: {bound}, target: 400)")
    assert avg is not None and avg <= bound, (
        f"LATENCY REGRESSION: rate {rate} compute window {avg} clk/sym "
        f"exceeds ratchet of {bound}. Revert or investigate the pipeline change."
    )


@cocotb.test()
async def test_latency_rate6(dut):
    """Rate 6 compute window must not exceed the ratchet."""
    await _check_compute(dut, 6)


@cocotb.test()
async def test_latency_rate12(dut):
    """Rate 12 compute window must not exceed the ratchet."""
    await _check_compute(dut, 12)


@cocotb.test()
async def test_latency_rate24(dut):
    """Rate 24 compute window must not exceed the ratchet."""
    await _check_compute(dut, 24)


@cocotb.test()
async def test_latency_rate24_wirespeed(dut):
    """Rate 24 must also reach the 400-clock wire-speed symbol period."""
    avg, _, decoded = await measure_symbol_cycle(dut, rate=24)
    assert decoded, "rate 24 frame did not decode to tag_valid"
    dut._log.info(f"Rate 24 period: {avg:.1f} clk/sym "
                  f"(wire-speed target: {TARGET_RATE24})")
    assert avg <= TARGET_RATE24, (
        f"RATE 24 NOT WIRE-SPEED: {avg:.1f} clk/sym exceeds the "
        f"{TARGET_RATE24}-clock budget. The streaming demapper (lever C) is "
        f"not complete, or the serialized demapper emit phase is back on the "
        f"critical path."
    )
