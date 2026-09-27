"""
test_rx_frontend.py — Gate tests for the full front-end + decode pipeline.

THESE ARE TESTS. They validate correct code. ALL must pass on main. Always.
A failing test means broken code. No exceptions, no "expected failures."

If a test here fails, the code is wrong. Fix the code or revert.
Do NOT weaken assertions, skip tests, or move tests to diagnostics to
make the suite pass. The gate only ratchets UP (more tests, stricter
thresholds), never down.

What these tests prove:
  - STF detection fires correctly on golden vectors
  - The stf_end -> rx_pipeline timing alignment produces correct decode
  - Live-mode decode (pre-frame noise, 1-per-5 valid timing) works
  - All rates that the hardware baseline passes also pass in sim

The DUT is rx_frontend.v — a sim-only integration wrapper that wires
stf_detect + cfo_est + cfo_mixer + rx_pipeline, matching system_bd.tcl.

NOTE: Live-mode tests MUST come before clean-mode tests. The stf_detect
module uses BRAM delay lines that are not cleared by rst_n. In Verilator
sim, BRAMs start zeroed at t=0 (matching real FPGA power-up), but after a
clean test writes frame data, stale BRAM contents persist across rst_n
cycles. Live-mode tests require pristine BRAMs for correct autocorrelation
accumulation. In real hardware this is not an issue: the ADC runs
continuously, filling BRAMs with live data before any frame arrives.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

from frontend_helpers import (
    load_waveform_float, quantize_12bit, s12_to_unsigned,
    reset_dut, run_frontend_decode, run_frontend_decode_live,
    verify_psdu_path, RATE_CODES, EXPECTED_PSDU_LEN, SAMPLE_RATE,
)


# =========================================================
# Gate: Live-mode decode (models cable loopback conditions)
#
# MUST RUN FIRST — see module docstring for BRAM ordering note.
#
# Cable loopback uses same crystal (CFO ~ 0), with pre-frame noise
# filling delay lines and 1-per-5-clock valid timing.
# =========================================================

@cocotb.test()
async def test_live_mode_rate6(dut):
    """Rate 6 live-mode, no CFO — pre-frame noise + realistic timing."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    iq = load_waveform_float(6)
    # NO CFO — same crystal loopback
    samples = quantize_12bit(iq)
    dut._log.info(f"Live-mode rate 6: {len(samples)} frame samples + 500 pre-noise")

    r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500)

    assert r['frame_detect_seen'], "STF detection did not fire in live mode"
    assert r['stf_end_seen'], "stf_end did not fire in live mode"
    assert r['tag_valid'], "No tag output — pipeline stalled in live mode"
    assert r['tag_fcs_ok'] == 1, \
        f"FCS failed in live mode (rate=0b{r['tag_rate']:04b}, len={r['tag_length']})"
    psdu_ok, psdu_reason = verify_psdu_path(r)
    assert psdu_ok, \
        f"PSDU byte path broken in live mode: {psdu_reason} " \
        f"(this path was untested until 2026-09-04 — the coverage gap that " \
        f"allowed the TAG_HI bug to live undetected)"
    dut._log.info(f"PASS: live-mode rate 6, FCS OK in {r['total_cycles']} cycles, "
                  f"PSDU {len(r['psdu_bytes'])} bytes verified")


@cocotb.test()
async def test_all_rates_live(dut):
    """All 8 rates live-mode, no CFO — the cable-loopback gate test.

    Models the actual hardware cable loopback condition: same crystal (no CFO),
    continuous ADC (pre-frame noise), live timing (1-per-5 clocks).

    Gate threshold: >= 8/8 rates must pass.
    Current passing: all 8 (6, 9, 12, 18, 24, 36, 48, 54).

    RATCHET RULE: This threshold only goes UP. When a rate is fixed,
    increase the threshold. Never decrease it.
    """
    GATE_THRESHOLD = 8  # Minimum passing rates (ratchet: only increase)

    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    rates = [6, 9, 12, 18, 24, 36, 48, 54]
    pass_count = 0
    pass_rates = []
    fail_rates = []

    for rate in rates:
        await reset_dut(dut)
        iq = load_waveform_float(rate)
        # NO CFO — same crystal loopback
        samples = quantize_12bit(iq)
        r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500)
        passed = r['tag_valid'] and r['tag_fcs_ok']
        if passed:
            psdu_ok, psdu_reason = verify_psdu_path(r)
            passed = psdu_ok
        else:
            psdu_ok, psdu_reason = False, "decode did not pass FCS"
        if passed:
            pass_count += 1
            pass_rates.append(rate)
        else:
            fail_rates.append(rate)
        status = "PASS" if passed else "FAIL"
        dut._log.info(f"  Rate {rate:2d}: {status} "
                      f"(length={r['parsed_length']}, phase_inc={r['phase_inc']}, "
                      f"psdu={psdu_ok})")
        if not psdu_ok:
            dut._log.info(f"      psdu: {psdu_reason}")

    dut._log.info(f"Live-mode no-CFO: {pass_count}/8 rates pass")
    dut._log.info(f"  Passing: {pass_rates}")
    if fail_rates:
        dut._log.info(f"  Failing: {fail_rates}")

    assert pass_count >= GATE_THRESHOLD, \
        f"Only {pass_count}/8 rates pass (gate requires >= {GATE_THRESHOLD}). " \
        f"Failing: {fail_rates}"

    # Specific rates that MUST pass (known-good in sim AND hardware).
    # Add rates here as they are fixed. Never remove.
    MUST_PASS = [6, 9, 12, 18, 24, 36, 48, 54]
    must_fail = [r for r in MUST_PASS if r in fail_rates]
    assert not must_fail, \
        f"Rates {must_fail} MUST pass (known-good) but failed. This is a regression."


# =========================================================
# Gate: Basic wiring (valid every clock, no impairments)
# =========================================================

@cocotb.test()
async def test_rate6_clean(dut):
    """Rate 6 clean golden vector — confirms wrapper wiring is correct."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    iq = load_waveform_float(6)
    samples = quantize_12bit(iq)
    dut._log.info(f"Rate 6 clean: {len(samples)} samples")

    r = await run_frontend_decode(dut, samples)

    assert r['frame_detect_seen'], "STF detection did not fire"
    assert r['stf_end_seen'], "stf_end did not fire"
    assert r['tag_valid'], "No tag output — pipeline did not complete"
    assert r['tag_fcs_ok'] == 1, f"FCS failed (rate=0b{r['tag_rate']:04b}, len={r['tag_length']})"
    assert r['tag_length'] == EXPECTED_PSDU_LEN
    psdu_ok, psdu_reason = verify_psdu_path(r)
    assert psdu_ok, f"PSDU byte path broken (clean): {psdu_reason}"
    dut._log.info(f"PASS: rate 6 clean, FCS OK in {r['total_cycles']} cycles, "
                  f"PSDU {len(r['psdu_bytes'])} bytes verified")


@cocotb.test()
async def test_rate24_clean(dut):
    """Rate 24 clean golden vector — second critical rate (EAPOL)."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    iq = load_waveform_float(24)
    samples = quantize_12bit(iq)
    dut._log.info(f"Rate 24 clean: {len(samples)} samples")

    r = await run_frontend_decode(dut, samples)

    assert r['frame_detect_seen'], "STF detection did not fire"
    assert r['tag_valid'], "No tag output"
    assert r['tag_fcs_ok'] == 1, f"FCS failed"
    assert r['tag_length'] == EXPECTED_PSDU_LEN
    psdu_ok, psdu_reason = verify_psdu_path(r)
    assert psdu_ok, f"PSDU byte path broken (clean): {psdu_reason}"
    dut._log.info(f"PASS: rate 24 clean, FCS OK, PSDU {len(r['psdu_bytes'])} bytes verified")
