"""
diag_rx_frontend.py — Diagnostic tools for rx_frontend characterization.

THESE ARE NOT TESTS. They are measurement instruments. They report numbers.
They never assert (except trivial "did it run" checks). They cannot "fail"
in the testing sense — they produce data for the developer to interpret.

Use these to:
  - Characterize CFO tolerance at different rates
  - Measure pilot_track behavior under residual CFO
  - Sweep parameters (noise, timing, skip values)
  - Replay ADC captures from hardware failures
  - Debug specific failure modes in isolation

Run via: make diag_rx_frontend
Or selectively: make diag_rx_frontend TESTCASE=diag_cfo_sweep
"""

import os
import sys
import json
import math

import numpy as np
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer

from frontend_helpers import (
    load_waveform_float, quantize_12bit, s12_to_unsigned,
    reset_dut, run_frontend_decode, run_frontend_decode_realistic,
    run_frontend_decode_live, load_adc_capture,
    add_cfo, add_awgn, add_dc_offset,
    RATE_CODES, EXPECTED_PSDU_LEN, SAMPLE_RATE, VECTORS_DIR,
)


# =========================================================
# CFO characterization
# =========================================================

@cocotb.test()
async def diag_cfo_sweep(dut):
    """Sweep CFO from -30 kHz to +30 kHz at rate 6 — measures correction range.

    Reports phase_inc and FCS result at each CFO. Use to identify:
    - Dead-zone boundary (where phase_inc goes to 0)
    - Correction accuracy (expected phase_inc vs actual)
    - Residual that pilot_track must handle
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    cfo_values = [-30000, -15000, -10000, -5000, -2000, 0, 2000, 5000, 10000, 15000, 30000]
    dut._log.info(f"CFO sweep: rate 6, {len(cfo_values)} values")

    for cfo_hz in cfo_values:
        await reset_dut(dut)
        iq = load_waveform_float(6)
        iq = add_cfo(iq, cfo_hz=cfo_hz, sample_rate=SAMPLE_RATE)
        samples = quantize_12bit(iq)

        r = await run_frontend_decode(dut, samples)
        status = "PASS" if (r['tag_valid'] and r['tag_fcs_ok']) else "FAIL"
        dut._log.info(f"  CFO={cfo_hz:+7d} Hz: {status} "
                      f"(phase_inc={r['phase_inc']}, len={r['parsed_length']})")

    dut._log.info("CFO sweep complete")


@cocotb.test()
async def diag_cfo_residual_amplification(dut):
    """Sweep small CFO values at rate 24 with 35 dB noise — pilot_track stress test.

    Tests the interaction between residual CFO (below dead-zone) and pilot_track.
    At cable-loopback SNR (35 dB), pilot_track's dead-zone may cause flicker
    at boundary conditions.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    cfo_values = [500, 1000, 1500, 2000, 3000, 4000, 5000, 7500, 10000]
    dut._log.info(f"CFO residual sweep: rate 24, 35 dB SNR, {len(cfo_values)} values")

    for cfo_hz in cfo_values:
        await reset_dut(dut)
        iq = load_waveform_float(24)
        iq = add_cfo(iq, cfo_hz=cfo_hz, sample_rate=SAMPLE_RATE)
        iq = add_awgn(iq, snr_db=35, seed=cfo_hz)
        samples = quantize_12bit(iq)

        r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500)
        status = "PASS" if (r['tag_valid'] and r['tag_fcs_ok']) else "FAIL"
        dut._log.info(f"  CFO={cfo_hz:5d} Hz: {status} (phase_inc={r['phase_inc']})")

    dut._log.info("CFO residual sweep complete")


@cocotb.test()
async def diag_all_rates_with_cfo(dut):
    """All 8 rates with 5 kHz CFO + 35 dB SNR in live mode.

    This is the OTA-readiness diagnostic. Currently fails on main because
    the CFO dead-zone (threshold=1024, ~19.5 kHz) suppresses correction
    of 5 kHz offsets. Will start passing once CFO correction is enabled
    for live mode.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    rates = [6, 9, 12, 18, 24, 36, 48, 54]
    pass_count = 0

    for rate in rates:
        await reset_dut(dut)
        iq = load_waveform_float(rate)
        iq = add_cfo(iq, cfo_hz=5000, sample_rate=SAMPLE_RATE)
        iq = add_awgn(iq, snr_db=35, seed=rate)
        samples = quantize_12bit(iq)
        r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500)
        passed = r['tag_valid'] and r['tag_fcs_ok']
        if passed:
            pass_count += 1
        status = "PASS" if passed else "FAIL"
        dut._log.info(f"  Rate {rate:2d}: {status} "
                      f"(detect@{r['frame_detect_cycle']}, phase_inc={r['phase_inc']})")

    dut._log.info(f"All-rates with CFO: {pass_count}/8 pass")


@cocotb.test()
async def diag_cfo_threshold_sweep(dut):
    """Sweep cfo_threshold values at multiple rates with 5 kHz CFO.

    Finds the optimal threshold for live mode. Tests each threshold
    against all 8 rates with realistic cable-loopback conditions
    (5 kHz CFO + 35 dB SNR + pre-noise).
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    thresholds = [64, 128, 256, 512, 1024]
    rates = [6, 9, 12, 18, 24, 36, 48, 54]

    dut._log.info(f"CFO threshold sweep: {len(thresholds)} thresholds × {len(rates)} rates")
    dut._log.info(f"Conditions: 5 kHz CFO, 35 dB SNR, live mode (1-per-5), pre_noise=500")
    dut._log.info(f"---")

    for thresh in thresholds:
        pass_count = 0
        pass_rates = []
        for rate in rates:
            await reset_dut(dut)
            iq = load_waveform_float(rate)
            iq = add_cfo(iq, cfo_hz=5000, sample_rate=SAMPLE_RATE)
            iq = add_awgn(iq, snr_db=35, seed=rate)
            samples = quantize_12bit(iq)
            r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500)
            passed = r['tag_valid'] and r['tag_fcs_ok']
            if passed:
                pass_count += 1
                pass_rates.append(rate)
        dut._log.info(f"  threshold={thresh:4d}: {pass_count}/8 pass  "
                      f"(phase_inc examples: see per-rate detail)")
        dut._log.info(f"    passing: {pass_rates}")

    dut._log.info(f"---")
    dut._log.info(f"Threshold sweep complete")


@cocotb.test()
async def diag_cfo_threshold_multi_cfo(dut):
    """Sweep cfo_threshold at multiple CFO values — rate 24 (16-QAM, most sensitive).

    Tests threshold 64 and 128 across a range of CFO values to find where
    correction starts being accurate enough for 16-QAM decode.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    thresholds = [64, 128]
    cfo_values = [1000, 2000, 3000, 5000, 7500, 10000, 15000]

    dut._log.info(f"Threshold × CFO sweep: rate 24, 35 dB SNR")
    dut._log.info(f"---")

    for thresh in thresholds:
        pass_count = 0
        for cfo_hz in cfo_values:
            await reset_dut(dut)
            iq = load_waveform_float(24)
            iq = add_cfo(iq, cfo_hz=cfo_hz, sample_rate=SAMPLE_RATE)
            iq = add_awgn(iq, snr_db=35, seed=cfo_hz)
            samples = quantize_12bit(iq)
            r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500)
            passed = r['tag_valid'] and r['tag_fcs_ok']
            if passed:
                pass_count += 1
            status = "PASS" if passed else "FAIL"
            dut._log.info(f"  thresh={thresh:4d} CFO={cfo_hz:5d} Hz: {status} "
                          f"(phase_inc={r['phase_inc']})")
        dut._log.info(f"  → threshold={thresh}: {pass_count}/{len(cfo_values)} CFO values pass")
        dut._log.info(f"  ---")


@cocotb.test()
async def diag_all_rates_live_threshold64(dut):
    """All 8 rates with 5 kHz CFO, threshold=64 — the target configuration.

    This is what cable loopback SHOULD achieve with configurable threshold.
    Compare results to diag_all_rates_with_cfo (threshold=1024).
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    rates = [6, 9, 12, 18, 24, 36, 48, 54]
    pass_count = 0
    pass_rates = []
    fail_rates = []

    dut._log.info(f"All-rates with CFO + threshold=64 (live mode target)")
    dut._log.info(f"Conditions: 5 kHz CFO, 35 dB SNR, pre_noise=500")
    dut._log.info(f"---")

    for rate in rates:
        await reset_dut(dut)
        iq = load_waveform_float(rate)
        iq = add_cfo(iq, cfo_hz=5000, sample_rate=SAMPLE_RATE)
        iq = add_awgn(iq, snr_db=35, seed=rate)
        samples = quantize_12bit(iq)
        r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500)
        passed = r['tag_valid'] and r['tag_fcs_ok']
        if passed:
            pass_count += 1
            pass_rates.append(rate)
        else:
            fail_rates.append(rate)
        status = "PASS" if passed else "FAIL"
        dut._log.info(f"  Rate {rate:2d}: {status} "
                      f"(phase_inc={r['phase_inc']}, len={r['parsed_length']})")

    dut._log.info(f"---")
    dut._log.info(f"Result: {pass_count}/8 pass (threshold=64, CFO=5kHz)")
    dut._log.info(f"  Passing: {pass_rates}")
    if fail_rates:
        dut._log.info(f"  Failing: {fail_rates}")

# =========================================================
# Noise and channel characterization
# =========================================================

@cocotb.test()
async def diag_noise_sensitivity(dut):
    """Rate 6 and 24 at various SNR levels — finds decode threshold."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    snr_values = [40, 35, 30, 25, 20, 15, 10]

    for rate in [6, 24]:
        dut._log.info(f"  --- Rate {rate} ---")
        for snr_db in snr_values:
            await reset_dut(dut)
            iq = load_waveform_float(rate)
            iq = add_awgn(iq, snr_db=snr_db, seed=snr_db * 100 + rate)
            samples = quantize_12bit(iq)
            r = await run_frontend_decode_live(dut, samples, pre_noise_samples=200)
            status = "PASS" if (r['tag_valid'] and r['tag_fcs_ok']) else "FAIL"
            dut._log.info(f"    SNR={snr_db:2d} dB: {status}")

    dut._log.info("Noise sensitivity sweep complete")


@cocotb.test()
async def diag_cable_conditions(dut):
    """All 8 rates with cable-like impairments: 30 dB SNR + DC offset.

    Models a pessimistic cable loopback scenario.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    rates = [6, 9, 12, 18, 24, 36, 48, 54]
    pass_count = 0

    for rate in rates:
        await reset_dut(dut)
        iq = load_waveform_float(rate)
        iq = add_awgn(iq, snr_db=30, seed=rate * 7)
        iq = add_dc_offset(iq, fraction=0.02)
        samples = quantize_12bit(iq)
        r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500)
        passed = r['tag_valid'] and r['tag_fcs_ok']
        if passed:
            pass_count += 1
        status = "PASS" if passed else "FAIL"
        dut._log.info(f"  Rate {rate:2d}: {status}")

    dut._log.info(f"Cable conditions: {pass_count}/8 pass")


# =========================================================
# Timing characterization
# =========================================================

@cocotb.test()
async def diag_realistic_timing_all_rates(dut):
    """All 8 rates with realistic 1-per-5 timing + 5 kHz CFO.

    Uses ltf_skip=97 (calibrated for realistic timing mode).
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    rates = [6, 9, 12, 18, 24, 36, 48, 54]
    pass_count = 0

    for rate in rates:
        await reset_dut(dut)
        dut.ltf_skip.value = 97
        iq = load_waveform_float(rate)
        iq = add_cfo(iq, cfo_hz=5000, sample_rate=SAMPLE_RATE)
        samples = quantize_12bit(iq)

        r = await run_frontend_decode_realistic(dut, samples, timeout_cycles=10000000)
        passed = r['tag_valid'] and r['tag_fcs_ok']
        if passed:
            pass_count += 1
        status = "PASS" if passed else "FAIL"
        dut._log.info(f"  Rate {rate:2d}: {status}")

    dut._log.info(f"Realistic timing: {pass_count}/8 rates pass")


@cocotb.test()
async def diag_stf_end_skip_sweep(dut):
    """Sweep stf_end_skip values at rate 6 — finds working range."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    skip_values = [0, 1, 2, 3, 4, 5, 6, 8, 10, 12, 16]
    dut._log.info(f"stf_end_skip sweep: rate 6, {len(skip_values)} values")

    for skip in skip_values:
        await reset_dut(dut)
        dut.stf_end_skip.value = skip
        iq = load_waveform_float(6)
        samples = quantize_12bit(iq)
        r = await run_frontend_decode(dut, samples)
        status = "PASS" if (r['tag_valid'] and r['tag_fcs_ok']) else "FAIL"
        dut._log.info(f"  skip={skip:2d}: {status} (stf_end@{r['stf_end_cycle']})")

    dut._log.info("stf_end_skip sweep complete")


@cocotb.test()
async def diag_pre_noise_sweep(dut):
    """Rate 6 with varying pre-noise lengths — finds if noise duration matters."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    noise_lengths = [0, 50, 100, 200, 500, 1000, 2000]

    for pre_n in noise_lengths:
        await reset_dut(dut)
        iq = load_waveform_float(6)
        iq = add_cfo(iq, cfo_hz=5000, sample_rate=SAMPLE_RATE)
        samples = quantize_12bit(iq)

        r = await run_frontend_decode_live(dut, samples, pre_noise_samples=pre_n)
        status = "PASS" if (r['tag_valid'] and r['tag_fcs_ok']) else "FAIL"
        dut._log.info(f"  pre_noise={pre_n:4d}: {status} (phase_inc={r['phase_inc']})")

    dut._log.info("Pre-noise sweep complete")


@cocotb.test()
async def diag_ltf_skip_sweep(dut):
    """Sweep ltf_skip values in live mode — find working range for FFT alignment."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    skip_values = [1, 50, 97, 100, 150, 192]

    for skip in skip_values:
        await reset_dut(dut)
        dut.ltf_skip.value = skip
        iq = load_waveform_float(6)
        iq = add_cfo(iq, cfo_hz=5000, sample_rate=SAMPLE_RATE)
        samples = quantize_12bit(iq)

        r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500)
        status = "PASS" if (r['tag_valid'] and r['tag_fcs_ok']) else "FAIL"
        dut._log.info(f"  ltf_skip={skip:3d}: {status} (phase_inc={r['phase_inc']})")

    dut._log.info("ltf_skip sweep complete")


# =========================================================
# Back-to-back / recovery
# =========================================================

@cocotb.test()
async def diag_back_to_back(dut):
    """Two frames back-to-back without DUT reset — tests recovery between frames.

    Frame 1: rate 6, CFO 3 kHz
    Frame 2: rate 24, CFO 7 kHz (no reset between frames)
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    # --- Frame 1: rate 6, CFO 3 kHz ---
    await reset_dut(dut)
    iq1 = load_waveform_float(6)
    iq1 = add_cfo(iq1, cfo_hz=3000, sample_rate=SAMPLE_RATE)
    samples1 = quantize_12bit(iq1)
    dut._log.info(f"Frame 1: rate 6, CFO=3kHz, {len(samples1)} samples")

    r1 = await run_frontend_decode_live(dut, samples1, pre_noise_samples=500)
    dut._log.info(f"  Frame 1: tag_valid={r1['tag_valid']}, fcs_ok={r1['tag_fcs_ok']}, "
                  f"phase_inc={r1['phase_inc']}")

    # --- Frame 2: rate 24, CFO 7 kHz — NO reset ---
    iq2 = load_waveform_float(24)
    iq2 = add_cfo(iq2, cfo_hz=7000, sample_rate=SAMPLE_RATE)
    samples2 = quantize_12bit(iq2)
    dut._log.info(f"Frame 2 (no reset): rate 24, CFO=7kHz, {len(samples2)} samples")

    r2 = await run_frontend_decode_live(dut, samples2, pre_noise_samples=300)
    dut._log.info(f"  Frame 2: tag_valid={r2['tag_valid']}, fcs_ok={r2['tag_fcs_ok']}, "
                  f"phase_inc={r2['phase_inc']}")

    dut._log.info(f"Back-to-back: frame1={'PASS' if r1['tag_fcs_ok'] else 'FAIL'}, "
                  f"frame2={'PASS' if r2['tag_fcs_ok'] else 'FAIL'}")


# =========================================================
# Pilot track diagnostics
# =========================================================

@cocotb.test()
async def diag_pilot_track_rate24(dut):
    """Rate 24 with 5 kHz CFO — detailed pilot_track characterization.

    Logs per-symbol pilot phase if accessible via DUT signals.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    iq = load_waveform_float(24)
    iq = add_cfo(iq, cfo_hz=5000, sample_rate=SAMPLE_RATE)
    iq = add_awgn(iq, snr_db=35, seed=24)
    samples = quantize_12bit(iq)
    dut._log.info(f"Pilot track diag: rate 24, CFO=5kHz, SNR=35dB")

    r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500)

    dut._log.info(f"  Result: tag_valid={r['tag_valid']}, fcs_ok={r['tag_fcs_ok']}")
    dut._log.info(f"  phase_inc={r['phase_inc']}")
    if r['tag_valid'] and not r['tag_fcs_ok']:
        dut._log.info("  FCS fail with correct SIGNAL — pilot_track residual likely cause")


@cocotb.test()
async def diag_pilot_track_correction_magnitude(dut):
    """Probe pilot_track CPE and correction for all 8 rates with threshold=64.

    Reports per-symbol CPE magnitude and whether pilot_track applied correction
    (S_EMIT) or bypassed (S_EMIT_BYPASS). This reveals whether the pilot_track
    dead-zone is absorbing residual CFO that should be corrected.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    rates = [6, 9, 12, 18, 24, 36, 48, 54]
    cfo_hz = 5000

    dut._log.info(f"Pilot track correction probe: threshold=64, CFO={cfo_hz}Hz, 35dB SNR")
    dut._log.info(f"---")

    for rate in rates:
        await reset_dut(dut)
        iq = load_waveform_float(rate)
        iq = add_cfo(iq, cfo_hz=cfo_hz, sample_rate=SAMPLE_RATE)
        iq = add_awgn(iq, snr_db=35, seed=rate)
        samples = quantize_12bit(iq)

        # Run decode while probing pilot_track internals
        r = await _run_with_pilot_probe(dut, samples, pre_noise_samples=500)
        status = "PASS" if r['fcs_ok'] else "FAIL"
        dut._log.info(f"  Rate {rate:2d}: {status}  phase_inc={r['phase_inc']:+4d}  "
                      f"symbols={r['n_symbols']}  "
                      f"corrected={r['n_corrected']}/{r['n_symbols']}  "
                      f"bypassed={r['n_bypassed']}/{r['n_symbols']}  "
                      f"max_cpe={r['max_cpe']}")

    dut._log.info(f"---")
    dut._log.info(f"Pilot track probe complete")


@cocotb.test()
async def diag_zero_cfo_with_threshold64(dut):
    """All 8 rates with ZERO CFO but threshold=64 — spurious correction safety check.

    If threshold=64 causes golden vectors (zero true CFO) to trigger spurious
    phase correction from quantization noise, this test will reveal it.
    Must pass 8/8 to confirm threshold=64 is safe for cable loopback.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    rates = [6, 9, 12, 18, 24, 36, 48, 54]
    pass_count = 0
    fail_rates = []

    dut._log.info(f"Zero-CFO safety check: threshold=64, 0 Hz CFO, 35 dB SNR")
    dut._log.info(f"---")

    for rate in rates:
        await reset_dut(dut)
        iq = load_waveform_float(rate)
        iq = add_awgn(iq, snr_db=35, seed=rate)
        samples = quantize_12bit(iq)
        r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500)
        passed = r['tag_valid'] and r['tag_fcs_ok']
        if passed:
            pass_count += 1
        else:
            fail_rates.append(rate)
        status = "PASS" if passed else "FAIL"
        dut._log.info(f"  Rate {rate:2d}: {status} (phase_inc={r['phase_inc']})")

    dut._log.info(f"---")
    dut._log.info(f"Zero-CFO + threshold=64: {pass_count}/8 pass")
    if fail_rates:
        dut._log.info(f"  SPURIOUS CORRECTION detected at rates: {fail_rates}")
    else:
        dut._log.info(f"  SAFE: no spurious correction from quantization noise")


async def _run_with_pilot_probe(dut, iq_samples, pre_noise_samples=500):
    """Run frontend decode probing pilot_track state transitions."""
    # Access pilot_track internals
    try:
        pt = dut.u_rx_pipeline.u_pilot_track
    except AttributeError:
        return {'fcs_ok': False, 'phase_inc': 0, 'n_symbols': 0,
                'n_corrected': 0, 'n_bypassed': 0, 'max_cpe': 0}

    # Build noise + frame stream
    sig_rms = np.sqrt(np.mean([r**2 + i**2 for r, i in iq_samples]))
    noise_std = sig_rms / (10**(35 / 20))
    rng = np.random.default_rng(seed=9999)
    noise_i = np.clip(np.round(rng.normal(0, noise_std, pre_noise_samples)),
                      -2048, 2047).astype(int)
    noise_q = np.clip(np.round(rng.normal(0, noise_std, pre_noise_samples)),
                      -2048, 2047).astype(int)
    noise_samples = list(zip(noise_i.tolist(), noise_q.tolist()))
    all_samples = noise_samples + list(iq_samples)
    n_samples = len(all_samples)
    post_zero_count = 2000

    result = {
        'fcs_ok': False,
        'phase_inc': 0,
        'n_symbols': 0,
        'n_corrected': 0,
        'n_bypassed': 0,
        'max_cpe': 0,
    }

    sample_idx = 0
    valid_counter = 0
    prev_state = 0
    cfo_done_seen = False

    # State constants matching pilot_track.v
    S_EMIT = 4
    S_EMIT_BYPASS = 7
    S_DONE = 5

    for cycle in range(10000000):
        await RisingEdge(dut.clk)

        # Feed at 1-per-5
        if valid_counter == 0:
            if sample_idx < n_samples:
                re_q, im_q = all_samples[sample_idx]
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = s12_to_unsigned(re_q)
                dut.iq_q_in.value = s12_to_unsigned(im_q)
                sample_idx += 1
            elif sample_idx < n_samples + post_zero_count:
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = 0
                dut.iq_q_in.value = 0
                sample_idx += 1
            else:
                dut.iq_valid_in.value = 0
        else:
            dut.iq_valid_in.value = 0
        valid_counter = (valid_counter + 1) % 5

        # Probe pilot_track state
        try:
            cur_state = int(pt.state.value)
            # Detect S_DONE transitions (symbol completed)
            if cur_state == S_DONE and prev_state != S_DONE:
                result['n_symbols'] += 1
            # Detect entry into S_EMIT (correction applied)
            if cur_state == S_EMIT and prev_state != S_EMIT:
                result['n_corrected'] += 1
                # Read CPE magnitude
                try:
                    cpe_val = int(pt.cpe_raw.value)
                    if cpe_val >= 32768:
                        cpe_val -= 65536
                    cpe_abs = abs(cpe_val)
                    if cpe_abs > result['max_cpe']:
                        result['max_cpe'] = cpe_abs
                except (ValueError, AttributeError):
                    pass
            # Detect entry into S_EMIT_BYPASS (dead-zone bypass)
            if cur_state == S_EMIT_BYPASS and prev_state != S_EMIT_BYPASS:
                result['n_bypassed'] += 1
            prev_state = cur_state
        except (ValueError, AttributeError):
            pass

        # CFO done
        try:
            if int(dut.cfo_done.value) == 1 and not cfo_done_seen:
                cfo_done_seen = True
                pi = int(dut.phase_inc.value)
                if pi >= 32768:
                    pi -= 65536
                result['phase_inc'] = pi
        except (ValueError, AttributeError):
            pass

        # Tag output
        try:
            if int(dut.tag_valid.value) == 1:
                result['fcs_ok'] = bool(int(dut.tag_fcs_ok.value))
                break
        except (ValueError, AttributeError):
            pass

        # Seq done (early exit)
        try:
            if int(dut.seq_done.value) == 1:
                break
        except (ValueError, AttributeError):
            pass

    return result


# =========================================================
# ADC replay diagnostics
# =========================================================

@cocotb.test()
async def diag_adc_replay(dut):
    """Replay captured ADC IQ through front-end — hardware failure reproducer.

    Set ADC_CAPTURE_FILE env var to the capture JSON path.
    Reports decode result without asserting pass.
    """
    capture_file = os.environ.get('ADC_CAPTURE_FILE', '')
    if not capture_file:
        # Try default location
        default_path = os.path.join(os.path.dirname(__file__), '..', '..', 'captures', 'cable_6mbps.json')
        if os.path.exists(default_path):
            capture_file = default_path
        else:
            cocotb.log.info("ADC_CAPTURE_FILE not set and no default capture — skipping")
            return

    if not os.path.exists(capture_file):
        cocotb.log.warning(f"Capture file not found: {capture_file}")
        return

    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    dut._log.info(f"Loading ADC capture: {capture_file}")
    samples = load_adc_capture(capture_file)
    dut._log.info(f"  Samples: {len(samples)}, duration: {len(samples)/SAMPLE_RATE*1000:.2f} ms")

    r = await run_frontend_decode(dut, samples, timeout_cycles=5000000)

    dut._log.info(f"  === ADC Replay Results ===")
    fd_detail = f" @ cycle {r['frame_detect_cycle']}" if r['frame_detect_seen'] else ''
    dut._log.info(f"  frame_detect: {'YES' if r['frame_detect_seen'] else 'NO'}{fd_detail}")
    se_detail = f" @ cycle {r['stf_end_cycle']}" if r['stf_end_seen'] else ''
    dut._log.info(f"  stf_end:      {'YES' if r['stf_end_seen'] else 'NO'}{se_detail}")
    dut._log.info(f"  cfo_done:     {'YES' if r['cfo_done_seen'] else 'NO'}"
                  f" (phase_inc={r['phase_inc']})")
    sig_detail = f" rate=0b{r['parsed_rate']:04b} len={r['parsed_length']}" if r['signal_valid'] else ''
    dut._log.info(f"  signal_valid: {'YES' if r['signal_valid'] else 'NO'}{sig_detail}")
    dut._log.info(f"  tag_valid:    {'YES' if r['tag_valid'] else 'NO'}")
    if r['tag_valid']:
        dut._log.info(f"  tag_fcs_ok:   {r['tag_fcs_ok']}")
        dut._log.info(f"  tag_rate:     0b{r['tag_rate']:04b}")
        dut._log.info(f"  tag_length:   {r['tag_length']}")
    dut._log.info(f"  total_cycles: {r['total_cycles']}")

    if r['tag_valid'] and r['tag_fcs_ok']:
        dut._log.info(f"  DECODE SUCCESS")
    elif r['tag_valid']:
        dut._log.info(f"  DECODE FAIL — FCS bad (debug: check pilot/EQ)")
    elif r['signal_valid']:
        dut._log.info(f"  SIGNAL OK but DATA failed — pipeline stalled or Viterbi error")
    elif r['frame_detect_seen']:
        dut._log.info(f"  STF detected but decode never completed — timing/alignment issue")
    else:
        dut._log.info(f"  NO DETECTION — STF never fired")


@cocotb.test()
async def diag_adc_replay_multi(dut):
    """Replay all captures from captures/ directory — batch diagnostic."""
    captures_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'captures')
    if not os.path.isdir(captures_dir):
        cocotb.log.info("No captures/ directory — skipping")
        return

    import glob as globmod
    capture_files = sorted(globmod.glob(os.path.join(captures_dir, '*.json')))
    if not capture_files:
        cocotb.log.info("No capture files found — skipping")
        return

    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    results = []
    for filepath in capture_files:
        fname = os.path.basename(filepath)
        await reset_dut(dut)

        dut._log.info(f"--- Replay: {fname} ---")
        samples = load_adc_capture(filepath)
        dut._log.info(f"  {len(samples)} samples ({len(samples)/SAMPLE_RATE*1000:.1f} ms)")

        r = await run_frontend_decode(dut, samples, timeout_cycles=5000000)

        status = "PASS" if (r['tag_valid'] and r['tag_fcs_ok']) else \
                 "FCS_FAIL" if r['tag_valid'] else \
                 "NO_TAG" if r['signal_valid'] else \
                 "NO_DETECT"
        results.append((fname, status, r))
        dut._log.info(f"  Result: {status}")

    dut._log.info(f"  ---")
    dut._log.info(f"=== Multi-Replay Summary ===")
    for fname, status, r in results:
        phase = r['phase_inc'] if r['cfo_done_seen'] else '?'
        dut._log.info(f"  {fname:40s}  {status:10s}  phase_inc={phase}")


# =========================================================
# EVM Analysis — Per-symbol characterization from ADC replay
# =========================================================

@cocotb.test()
async def diag_ltf_peak_pos(dut):
    """Report ltf_peak peak_sample_pos and ltf1_offset for all 8 rates.

    Isolates the peak_pos bug: 0 CFO, 0 noise, live mode with pre-noise.
    Probes internal signals: u_rx_pipeline.u_ltf_peak.peak_sample_pos
    and u_rx_pipeline.u_decode_engine.ltf1_offset.

    The preamble is identical for all rates — peak_pos SHOULD be identical.
    Any variation proves the peak tracker is influenced by post-LTF DATA content.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    rates = [6, 9, 12, 18, 24, 36, 48, 54]
    results = []

    dut._log.info("=== ltf_peak peak_sample_pos characterization ===")
    dut._log.info("Conditions: 0 CFO, 0 noise, live mode (1-per-5), pre_noise=500")
    dut._log.info("---")

    for rate in rates:
        await reset_dut(dut)
        iq = load_waveform_float(rate)
        samples = quantize_12bit(iq)

        # Use run_frontend_decode_live but monitor peak_pos internally
        r = await _run_with_peak_probe(dut, samples, pre_noise_samples=500)
        results.append(r)

        dut._log.info(
            f"  Rate {rate:2d}: peak_pos={r['peak_pos']:4d}  "
            f"ltf1_offset={r['ltf1_offset']:4d}  "
            f"fcs={'OK' if r['fcs_ok'] else 'FAIL'}  "
            f"done={'Y' if r['ltf_done'] else 'N'}"
        )

    # Summary
    peak_positions = [r['peak_pos'] for r in results]
    unique_peaks = set(peak_positions)
    dut._log.info(f"  ---")
    dut._log.info(f"  Peak positions: {peak_positions}")
    dut._log.info(f"  Unique values: {sorted(unique_peaks)} (want exactly 1)")
    if len(unique_peaks) > 1:
        dut._log.info(f"  BUG CONFIRMED: peak_pos varies by rate despite identical preamble")
    else:
        dut._log.info(f"  CONSISTENT: all rates produce peak_pos={peak_positions[0]}")

    pass_count = sum(1 for r in results if r['fcs_ok'])
    dut._log.info(f"  FCS pass: {pass_count}/8")


async def _run_with_peak_probe(dut, iq_samples, pre_noise_samples=500):
    """Run frontend decode in live mode, probing ltf_peak peak_sample_pos internally."""
    # Access internal signals (D19: rx_top/fft_wrap removed; the LTF peak
    # detector is u_ltf_peak, the latched offset is in u_decode_engine)
    de = dut.u_rx_pipeline.u_decode_engine
    peak = dut.u_rx_pipeline.u_ltf_peak

    # Compute signal RMS for noise generation
    sig_rms = np.sqrt(np.mean([r**2 + i**2 for r, i in iq_samples]))
    noise_std = sig_rms / (10**(35 / 20))  # 35 dB SNR
    rng = np.random.default_rng(seed=9999)
    noise_i = np.clip(np.round(rng.normal(0, noise_std, pre_noise_samples)),
                      -2048, 2047).astype(int)
    noise_q = np.clip(np.round(rng.normal(0, noise_std, pre_noise_samples)),
                      -2048, 2047).astype(int)
    noise_samples = list(zip(noise_i.tolist(), noise_q.tolist()))

    all_samples = noise_samples + list(iq_samples)
    n_samples = len(all_samples)
    post_zero_count = 2000

    result = {
        'peak_pos': 0,
        'ltf1_offset': 0,
        'ltf_done': False,
        'fcs_ok': False,
        'tag_valid': False,
    }

    sample_idx = 0
    valid_counter = 0

    for cycle in range(10000000):
        await RisingEdge(dut.clk)

        # Feed at 1-per-5
        if valid_counter == 0:
            if sample_idx < n_samples:
                re_q, im_q = all_samples[sample_idx]
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = s12_to_unsigned(re_q)
                dut.iq_q_in.value = s12_to_unsigned(im_q)
                sample_idx += 1
            elif sample_idx < n_samples + post_zero_count:
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = 0
                dut.iq_q_in.value = 0
                sample_idx += 1
            else:
                dut.iq_valid_in.value = 0
        else:
            dut.iq_valid_in.value = 0

        valid_counter = (valid_counter + 1) % 5

        # Probe ltf_peak_detect
        try:
            if int(peak.peak_found.value) == 1 and not result['ltf_done']:
                result['ltf_done'] = True
                result['peak_pos'] = int(peak.peak_sample_pos.value)
        except (ValueError, AttributeError):
            pass

        # Check tag output
        try:
            if int(dut.tag_valid.value) == 1:
                result['tag_valid'] = True
                result['fcs_ok'] = bool(int(dut.tag_fcs_ok.value))
                # Read ltf1_offset from decode_engine
                try:
                    result['ltf1_offset'] = int(de.ltf1_offset.value)
                except (ValueError, AttributeError):
                    pass
                break
        except (ValueError, AttributeError):
            pass

        # Also check seq_done for early exit
        try:
            if int(dut.seq_done.value) == 1 and not result['tag_valid']:
                try:
                    result['ltf1_offset'] = int(de.ltf1_offset.value)
                except (ValueError, AttributeError):
                    pass
                break
        except (ValueError, AttributeError):
            pass

    return result


# =========================================================
# EVM Analysis — Per-symbol characterization from ADC replay
# =========================================================

# Rate → modulation order (bits per subcarrier)
_RATE_CODE_TO_MOD = {
    0b1011: 1,  # 6M  BPSK
    0b1111: 1,  # 9M  BPSK
    0b1010: 2,  # 12M QPSK
    0b1110: 2,  # 18M QPSK
    0b1001: 4,  # 24M 16-QAM
    0b1101: 4,  # 36M 16-QAM
    0b1000: 6,  # 48M 64-QAM
    0b1100: 6,  # 54M 64-QAM
}

_RATE_CODE_TO_MBPS = {
    0b1011: 6,  0b1111: 9,  0b1010: 12, 0b1110: 18,
    0b1001: 24, 0b1101: 36, 0b1000: 48, 0b1100: 54,
}


async def _run_evm_analysis(dut, iq_samples, timeout_cycles=5000000):
    """Feed IQ through pipeline and collect per-symbol EQ output for EVM analysis.

    Probes internal signals:
      - dut.u_rx_pipeline.eq_data_valid / eq_data_re / eq_data_im / eq_data_idx
      - dut.u_rx_pipeline.u_pilot_track.cpe_raw
      - dut.u_rx_pipeline.u_pilot_track.smoothed_slope
      - dut.u_rx_pipeline.u_pilot_track.pilot_mag
      - dut.u_rx_pipeline.u_pilot_track.state (to detect symbol boundaries)
      - dut.u_rx_pipeline.u_decode_engine.symbol_idx_out

    Returns dict with:
      - Standard decode result fields
      - 'eq_symbols': list of arrays (48 complex per symbol, post-EQ pre-pilot_track)
      - 'pt_symbols': list of arrays (48 complex per symbol, post-pilot_track)
      - 'pilots': list of arrays (4 complex per symbol, equalized pilots)
      - 'cpe_per_symbol': list of raw CPE values
      - 'slope_per_symbol': list of smoothed slope values
      - 'mod_order': detected modulation order
    """
    n_samples = len(iq_samples)
    post_zero_count = 8192
    total_feed = n_samples + post_zero_count

    result = {
        'tag_valid': False, 'tag_rate': 0, 'tag_length': 0, 'tag_fcs_ok': 0,
        'signal_valid': False, 'parsed_rate': 0, 'parsed_length': 0,
        'frame_detect_seen': False, 'frame_detect_cycle': 0,
        'stf_end_seen': False, 'stf_end_cycle': 0,
        'cfo_done_seen': False, 'phase_inc': 0,
        'total_cycles': 0,
        'eq_symbols': [], 'pt_symbols': [], 'pilots': [],
        'cpe_per_symbol': [], 'slope_per_symbol': [],
        'mod_order': 0,
    }

    # Accumulators for current symbol
    cur_eq_re = [0] * 48
    cur_eq_im = [0] * 48
    eq_count = 0
    cur_pilot_re = [0] * 4
    cur_pilot_im = [0] * 4
    pilot_count = 0

    # Post-pilot_track accumulators
    cur_pt_re = [0] * 48
    cur_pt_im = [0] * 48
    pt_count = 0

    sample_idx = 0
    last_symbol_idx = -1
    signal_seen = False
    valid_counter = 0  # 1-per-5 clock IQ rate (matches hardware 100MHz/20MSPS)

    for cycle in range(timeout_cycles):
        await RisingEdge(dut.clk)

        # Feed IQ at 1-per-5 clock rate (matching hardware ADC/fabric ratio)
        if valid_counter == 0:
            if sample_idx < n_samples:
                re_q, im_q = iq_samples[sample_idx]
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = s12_to_unsigned(re_q)
                dut.iq_q_in.value = s12_to_unsigned(im_q)
                sample_idx += 1
            elif sample_idx < total_feed:
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = 0
                dut.iq_q_in.value = 0
                sample_idx += 1
            else:
                dut.iq_valid_in.value = 0
        else:
            dut.iq_valid_in.value = 0

        valid_counter = (valid_counter + 1) % 5

        # Front-end events
        try:
            if int(dut.frame_detect.value) == 1 and not result['frame_detect_seen']:
                result['frame_detect_seen'] = True
                result['frame_detect_cycle'] = cycle
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.stf_end.value) == 1 and not result['stf_end_seen']:
                result['stf_end_seen'] = True
                result['stf_end_cycle'] = cycle
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.cfo_done.value) == 1 and not result['cfo_done_seen']:
                result['cfo_done_seen'] = True
                pi = int(dut.phase_inc.value)
                if pi >= 32768:
                    pi -= 65536
                result['phase_inc'] = pi
        except (ValueError, AttributeError):
            pass

        # SIGNAL valid
        try:
            if int(dut.signal_valid.value) == 1 and not result['signal_valid']:
                result['signal_valid'] = True
                result['parsed_rate'] = int(dut.parsed_rate.value)
                result['parsed_length'] = int(dut.parsed_length.value)
                result['mod_order'] = _RATE_CODE_TO_MOD.get(result['parsed_rate'], 0)
                signal_seen = True
        except (ValueError, AttributeError):
            pass

        # Probe EQ output (post-equalizer, pre-pilot_track)
        if signal_seen:
            try:
                eq_valid = int(dut.u_rx_pipeline.eq_data_valid.value)
                if eq_valid:
                    idx = int(dut.u_rx_pipeline.eq_data_idx.value)
                    re_val = int(dut.u_rx_pipeline.eq_data_re.value)
                    im_val = int(dut.u_rx_pipeline.eq_data_im.value)
                    # Sign extend 16-bit
                    if re_val >= 32768: re_val -= 65536
                    if im_val >= 32768: im_val -= 65536
                    if 0 <= idx < 48:
                        cur_eq_re[idx] = re_val
                        cur_eq_im[idx] = im_val
                        eq_count += 1
                    # When we have all 48 data subcarriers, finalize symbol
                    if eq_count == 48:
                        result['eq_symbols'].append(
                            np.array([complex(cur_eq_re[k], cur_eq_im[k]) for k in range(48)])
                        )
                        cur_eq_re = [0] * 48
                        cur_eq_im = [0] * 48
                        eq_count = 0
            except (ValueError, AttributeError):
                pass

            # Probe pilot output
            try:
                pilot_valid = int(dut.u_rx_pipeline.eq_pilot_valid.value)
                if pilot_valid:
                    pidx = int(dut.u_rx_pipeline.eq_pilot_idx.value)
                    pre = int(dut.u_rx_pipeline.eq_pilot_re.value)
                    pim = int(dut.u_rx_pipeline.eq_pilot_im.value)
                    if pre >= 32768: pre -= 65536
                    if pim >= 32768: pim -= 65536
                    if 0 <= pidx < 4:
                        cur_pilot_re[pidx] = pre
                        cur_pilot_im[pidx] = pim
                        pilot_count += 1
                    if pilot_count == 4:
                        result['pilots'].append(
                            np.array([complex(cur_pilot_re[k], cur_pilot_im[k]) for k in range(4)])
                        )
                        cur_pilot_re = [0] * 4
                        cur_pilot_im = [0] * 4
                        pilot_count = 0
            except (ValueError, AttributeError):
                pass

            # Probe pilot_track output (post-correction)
            try:
                pt_valid = int(dut.u_rx_pipeline.pt_data_valid.value)
                if pt_valid:
                    pt_re = int(dut.u_rx_pipeline.pt_data_re.value)
                    pt_im = int(dut.u_rx_pipeline.pt_data_im.value)
                    if pt_re >= 32768: pt_re -= 65536
                    if pt_im >= 32768: pt_im -= 65536
                    cur_pt_re[pt_count] = pt_re
                    cur_pt_im[pt_count] = pt_im
                    pt_count += 1
                    if pt_count == 48:
                        result['pt_symbols'].append(
                            np.array([complex(cur_pt_re[k], cur_pt_im[k]) for k in range(48)])
                        )
                        cur_pt_re = [0] * 48
                        cur_pt_im = [0] * 48
                        pt_count = 0
            except (ValueError, AttributeError):
                pass

            # Probe CPE/slope at symbol boundaries (pilot_track done)
            try:
                pt_state = int(dut.u_rx_pipeline.u_pilot_track.state.value)
                # S_DONE = 5 — symbol processing complete
                if pt_state == 5:
                    cpe = int(dut.u_rx_pipeline.u_pilot_track.cpe_raw.value)
                    slope = int(dut.u_rx_pipeline.u_pilot_track.smoothed_slope.value)
                    if cpe >= 32768: cpe -= 65536
                    if slope >= 32768: slope -= 65536
                    result['cpe_per_symbol'].append(cpe)
                    result['slope_per_symbol'].append(slope)
                    # Also probe pilot_mag and norm_shift for debugging
                    pmag = int(dut.u_rx_pipeline.u_pilot_track.pilot_mag.value)
                    nshift = int(dut.u_rx_pipeline.u_pilot_track.norm_shift.value)
                    if 'pilot_mag_per_symbol' not in result:
                        result['pilot_mag_per_symbol'] = []
                        result['norm_shift_per_symbol'] = []
                    result['pilot_mag_per_symbol'].append(pmag)
                    result['norm_shift_per_symbol'].append(nshift)
            except (ValueError, AttributeError):
                pass

        # Tag output
        try:
            if int(dut.tag_valid.value) == 1:
                result['tag_valid'] = True
                result['tag_rate'] = int(dut.tag_rate.value)
                result['tag_length'] = int(dut.tag_length.value)
                result['tag_fcs_ok'] = int(dut.tag_fcs_ok.value)
                result['total_cycles'] = cycle
                break
        except (ValueError, AttributeError):
            pass

        # seq_done without tag
        try:
            if int(dut.seq_done.value) == 1 and not result['tag_valid']:
                result['total_cycles'] = cycle
                break
        except (ValueError, AttributeError):
            pass

    return result


@cocotb.test()
async def diag_evm_analysis(dut):
    """EVM characterization from ADC capture — the primary debug instrument.

    Set ADC_CAPTURE_FILE env var to the capture JSON path.
    Set CFO_THRESHOLD env var to override threshold (default: 64 for live mode).

    Produces per-symbol EVM breakdown showing:
      - Where in the frame errors accumulate
      - Whether the dominant error is phase drift (fixable) or noise (margin)
      - CPE and slope tracking behavior
      - Pre/post pilot_track comparison

    This is the tool that answers: "WHY does this rate fail on hardware?"
    """
    capture_file = os.environ.get('ADC_CAPTURE_FILE', '')
    if not capture_file:
        captures_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'captures')
        # Try to find any capture
        import glob as globmod
        candidates = sorted(globmod.glob(os.path.join(captures_dir, '*.json')))
        if candidates:
            capture_file = candidates[0]
        else:
            cocotb.log.info("No ADC_CAPTURE_FILE set and no captures found — skipping")
            return

    if not os.path.exists(capture_file):
        cocotb.log.warning(f"Capture file not found: {capture_file}")
        return

    cfo_threshold = int(os.environ.get('CFO_THRESHOLD', '64'))

    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    dut._log.info(f"=== EVM Analysis: {os.path.basename(capture_file)} ===")
    dut._log.info(f"  cfo_threshold={cfo_threshold}")

    samples = load_adc_capture(capture_file)
    dut._log.info(f"  Samples: {len(samples)} ({len(samples)/SAMPLE_RATE*1000:.2f} ms)")

    r = await _run_evm_analysis(dut, samples)

    # Report basic decode status
    if not r['frame_detect_seen']:
        dut._log.info(f"  RESULT: NO DETECTION — STF never fired")
        return
    if not r['signal_valid']:
        dut._log.info(f"  RESULT: SIGNAL not parsed — timing/alignment issue")
        return

    rate_mbps = _RATE_CODE_TO_MBPS.get(r['parsed_rate'], '?')
    mod_order = r['mod_order']
    dut._log.info(f"  Rate: {rate_mbps}M (mod_order={mod_order})")
    dut._log.info(f"  Length: {r['parsed_length']} bytes")
    dut._log.info(f"  phase_inc: {r['phase_inc']}")
    dut._log.info(f"  FCS: {'OK' if r['tag_fcs_ok'] else 'FAIL'}")
    dut._log.info(f"  Symbols collected (EQ): {len(r['eq_symbols'])}")
    dut._log.info(f"  Symbols collected (PT): {len(r['pt_symbols'])}")
    dut._log.info(f"  CPE readings: {len(r['cpe_per_symbol'])}")

    if not r['eq_symbols'] or mod_order == 0:
        dut._log.info(f"  No EQ data collected — cannot compute EVM")
        return

    # Compute per-symbol EVM (post-EQ, pre-pilot_track)
    dut._log.info(f"")
    dut._log.info(f"  --- Per-Symbol EVM (post-EQ, pre-pilot_track) ---")

    # Normalize: EQ output is 16-bit fixed point, scale to unit constellation
    # The equalizer applies H_inv with EQ_SCALE=6 shift, so output magnitude
    # varies. Normalize by dividing by the RMS of the first symbol (which
    # was equalized with a fresh H estimate).
    eq_syms = r['eq_symbols']
    if len(eq_syms) > 0:
        # Use first DATA symbol's RMS as normalization reference
        ref_rms = np.sqrt(np.mean(np.abs(eq_syms[0])**2))
        if ref_rms == 0:
            ref_rms = 1.0

        # Constellation reference grids (same as lib80211)
        grids = {
            1: np.array([-1.0, 1.0]),
            2: np.array([-1.0, 1.0]) / math.sqrt(2.0),
            4: np.array([-3.0, -1.0, 1.0, 3.0]) / math.sqrt(10.0),
            6: np.array([-7.0, -5.0, -3.0, -1.0, 1.0, 3.0, 5.0, 7.0]) / math.sqrt(42.0),
        }
        grid = grids.get(mod_order)
        if grid is None:
            dut._log.info(f"  Unknown mod_order={mod_order}, cannot compute EVM")
            return

        # Expected RMS for this modulation (ideal constellation power)
        if mod_order == 1:
            ideal_rms = 1.0
        else:
            # For QAM: E[|s|²] = 1 by normalization convention
            ideal_rms = 1.0

        # Scale factor: map fixed-point EQ output to unit constellation
        scale = ideal_rms / ref_rms

        evm_per_sym_eq = []
        evm_per_sym_pt = []
        phase_per_sym = []

        for sym_idx, sym in enumerate(eq_syms):
            # Normalize to unit constellation
            sym_norm = sym * scale

            # Compute mean phase rotation of this symbol
            if mod_order == 1:
                # BPSK: phase = mean angle of (sign-corrected symbols)
                signs = np.sign(sym_norm.real)
                rotated = sym_norm * signs  # all should be near +1
                mean_phase = np.angle(np.mean(rotated))
            else:
                # QAM: use pilots if available, else estimate from data
                mean_phase = 0.0  # approximate — pilots are better

            phase_per_sym.append(np.degrees(mean_phase))

            # Hard-slice and compute EVM
            if mod_order == 1:
                ref_pts = np.where(sym_norm.real >= 0, 1.0+0j, -1.0+0j)
            else:
                i_ref = grid[np.argmin(np.abs(sym_norm.real[:, None] - grid[None, :]), axis=1)]
                q_ref = grid[np.argmin(np.abs(sym_norm.imag[:, None] - grid[None, :]), axis=1)]
                ref_pts = i_ref + 1j * q_ref

            err = sym_norm - ref_pts
            evm_rms = np.sqrt(np.mean(np.abs(err)**2))
            evm_db = 20 * np.log10(evm_rms) if evm_rms > 0 else -60.0
            evm_per_sym_eq.append(evm_db)

            # Log per symbol
            cpe_str = ""
            if sym_idx < len(r['cpe_per_symbol']):
                cpe_str = f" CPE={r['cpe_per_symbol'][sym_idx]:+5d}"
            slope_str = ""
            if sym_idx < len(r['slope_per_symbol']):
                slope_str = f" slope={r['slope_per_symbol'][sym_idx]:+5d}"
            mag_str = ""
            if 'pilot_mag_per_symbol' in r and sym_idx < len(r['pilot_mag_per_symbol']):
                mag_str = f" pmag={r['pilot_mag_per_symbol'][sym_idx]:4d} ns={r['norm_shift_per_symbol'][sym_idx]}"
            dut._log.info(f"  sym {sym_idx:3d}: EVM={evm_db:+6.1f} dB"
                          f"  phase={phase_per_sym[-1]:+5.1f}°{cpe_str}{slope_str}{mag_str}")

        # Summary
        dut._log.info(f"")
        dut._log.info(f"  --- EVM Summary (post-EQ) ---")
        dut._log.info(f"  First symbol:  {evm_per_sym_eq[0]:+.1f} dB")
        if len(evm_per_sym_eq) > 1:
            dut._log.info(f"  Last symbol:   {evm_per_sym_eq[-1]:+.1f} dB")
            dut._log.info(f"  Mean:          {np.mean(evm_per_sym_eq):+.1f} dB")
            dut._log.info(f"  Worst:         {max(evm_per_sym_eq):+.1f} dB")
            drift = evm_per_sym_eq[-1] - evm_per_sym_eq[0]
            dut._log.info(f"  Drift (last-first): {drift:+.1f} dB")

            # Classify failure mode
            if drift > 3.0 and len(evm_per_sym_eq) > 3:
                dut._log.info(f"  DIAGNOSIS: Progressive degradation — channel drift (H update needed)")
            elif np.mean(evm_per_sym_eq) > -15.0 and drift < 2.0:
                dut._log.info(f"  DIAGNOSIS: Uniformly high EVM — noise floor or H estimation error")
            elif max(evm_per_sym_eq) > -10.0 and np.mean(evm_per_sym_eq) < -20.0:
                dut._log.info(f"  DIAGNOSIS: Isolated bad symbol — possible timing glitch")
            else:
                dut._log.info(f"  DIAGNOSIS: Mixed — inspect per-symbol detail above")

        # EVM thresholds for decode success (approximate)
        # BPSK: ~-8 dB, QPSK: ~-13 dB, 16-QAM: ~-19 dB, 64-QAM: ~-25 dB
        evm_limits = {1: -8, 2: -13, 4: -19, 6: -25}
        limit = evm_limits.get(mod_order, -20)
        dut._log.info(f"  Required EVM for {mod_order}-bit decode: < {limit} dB (approx)")

    # Post-pilot_track EVM (shows whether correction helps)
    pt_syms = r['pt_symbols']
    if pt_syms and len(pt_syms) > 0 and mod_order > 0:
        dut._log.info(f"")
        dut._log.info(f"  --- Per-Symbol EVM (post-pilot_track) ---")
        pt_ref_rms = np.sqrt(np.mean(np.abs(pt_syms[0])**2)) if len(pt_syms) > 0 else 1.0
        if pt_ref_rms == 0:
            pt_ref_rms = 1.0
        pt_scale = ideal_rms / pt_ref_rms
        for sym_idx, sym in enumerate(pt_syms):
            sym_norm = sym * pt_scale
            # Compute mean phase of this symbol
            if mod_order >= 4:
                # For QAM: estimate phase by looking at outer-ring points
                # Simple: compute angle of mean(constellation * conj(nearest_ref))
                i_ref = grid[np.argmin(np.abs(sym_norm.real[:, None] - grid[None, :]), axis=1)]
                q_ref = grid[np.argmin(np.abs(sym_norm.imag[:, None] - grid[None, :]), axis=1)]
                ref_pts = i_ref + 1j * q_ref
                # Phase error = angle of (received / reference)
                phase_err = np.angle(sym_norm / ref_pts)
                mean_phase_deg = np.degrees(np.mean(phase_err))
            else:
                i_ref = grid[np.argmin(np.abs(sym_norm.real[:, None] - grid[None, :]), axis=1)]
                q_ref = grid[np.argmin(np.abs(sym_norm.imag[:, None] - grid[None, :]), axis=1)]
                ref_pts = i_ref + 1j * q_ref
                mean_phase_deg = 0.0
            err = sym_norm - ref_pts
            evm_rms = np.sqrt(np.mean(np.abs(err)**2))
            evm_db = 20 * np.log10(evm_rms) if evm_rms > 0 else -60.0
            evm_per_sym_pt.append(evm_db)
            dut._log.info(f"  sym {sym_idx:3d}: EVM={evm_db:+6.1f} dB  phase={mean_phase_deg:+6.2f}° (post-PT)")
        dut._log.info(f"  PT Mean EVM: {np.mean(evm_per_sym_pt):+.1f} dB")
        dut._log.info(f"  PT Worst:    {max(evm_per_sym_pt):+.1f} dB")

    # Pilot analysis
    if r['pilots']:
        dut._log.info(f"")
        dut._log.info(f"  --- Pilot Analysis ---")
        for sym_idx, pilots in enumerate(r['pilots']):
            # Pilots should be ±1 after equalization (magnitude ~= ref_rms)
            pilot_phases = np.degrees(np.angle(pilots))
            pilot_mags = np.abs(pilots) / ref_rms if ref_rms > 0 else np.abs(pilots)
            dut._log.info(f"  sym {sym_idx:3d}: phases=[{pilot_phases[0]:+5.1f}° "
                          f"{pilot_phases[1]:+5.1f}° {pilot_phases[2]:+5.1f}° "
                          f"{pilot_phases[3]:+5.1f}°]  "
                          f"mags=[{pilot_mags[0]:.2f} {pilot_mags[1]:.2f} "
                          f"{pilot_mags[2]:.2f} {pilot_mags[3]:.2f}]")
            if sym_idx > 8:
                dut._log.info(f"  ... (truncated, {len(r['pilots'])} total)")
                break

    dut._log.info(f"")
    dut._log.info(f"  === EVM analysis complete ===")


@cocotb.test()
async def diag_evm_decomposition(dut):
    """Decompose EVM into rotational (phase) vs radial (amplitude) components.

    For each symbol, computes:
      - Total EVM (RMS)
      - Phase EVM: error component tangential to reference point (phase drift)
      - Amplitude EVM: error component radial from origin (gain/quantization)
      - Per-subcarrier EVM distribution (identifies edge-vs-center frequency pattern)

    Compares rate 36 vs rate 24 ADC captures to isolate rate-dependent effects.
    Falls back to golden vector with injected CFO if no captures available.

    Set ADC_CAPTURE_FILE to analyze a specific capture.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    capture_file = os.environ.get('ADC_CAPTURE_FILE', '')
    if not capture_file:
        captures_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'captures')
        import glob as globmod
        candidates = sorted(globmod.glob(os.path.join(captures_dir, 'cable_36m*.json')))
        if candidates:
            capture_file = candidates[0]
        else:
            cocotb.log.info("No ADC_CAPTURE_FILE set and no rate 36 captures found")
            return

    if not os.path.exists(capture_file):
        cocotb.log.warning(f"Capture file not found: {capture_file}")
        return

    cfo_threshold = int(os.environ.get('CFO_THRESHOLD', '64'))

    dut._log.info(f"=== EVM Decomposition: {os.path.basename(capture_file)} ===")

    samples = load_adc_capture(capture_file)
    r = await _run_evm_analysis(dut, samples)

    if not r['frame_detect_seen'] or not r['signal_valid']:
        dut._log.info(f"  No valid frame detected — cannot analyze")
        return

    rate_mbps = _RATE_CODE_TO_MBPS.get(r['parsed_rate'], '?')
    mod_order = r['mod_order']
    dut._log.info(f"  Rate: {rate_mbps}M  mod_order={mod_order}  FCS={'OK' if r['tag_fcs_ok'] else 'FAIL'}")

    if mod_order == 0 or not r['pt_symbols']:
        dut._log.info(f"  No demodulatable data — cannot decompose")
        return

    # Constellation reference grid
    grids = {
        1: np.array([-1.0, 1.0]),
        2: np.array([-1.0, 1.0]) / math.sqrt(2.0),
        4: np.array([-3.0, -1.0, 1.0, 3.0]) / math.sqrt(10.0),
        6: np.array([-7.0, -5.0, -3.0, -1.0, 1.0, 3.0, 5.0, 7.0]) / math.sqrt(42.0),
    }
    grid = grids.get(mod_order)
    if grid is None:
        dut._log.info(f"  Unknown mod_order={mod_order}")
        return

    pt_syms = r['pt_symbols']
    ref_rms = np.sqrt(np.mean(np.abs(pt_syms[0])**2))
    if ref_rms == 0:
        ref_rms = 1.0
    scale = 1.0 / ref_rms

    dut._log.info(f"")
    dut._log.info(f"  --- Per-Symbol Error Decomposition (post-pilot_track) ---")
    dut._log.info(f"  sym | EVM_total | EVM_phase | EVM_ampl | mean_phase_err | max_sc_evm")
    dut._log.info(f"  ----+-----------+-----------+----------+----------------+-----------")

    all_phase_evm = []
    all_ampl_evm = []
    all_total_evm = []
    per_sc_evm_accum = np.zeros(48)  # per-subcarrier EVM accumulator
    n_symbols = 0

    for sym_idx, sym in enumerate(pt_syms):
        sym_norm = sym * scale

        # Hard-slice to nearest constellation point
        if mod_order == 1:
            ref_pts = np.where(sym_norm.real >= 0, 1.0+0j, -1.0+0j)
        else:
            i_ref = grid[np.argmin(np.abs(sym_norm.real[:, None] - grid[None, :]), axis=1)]
            q_ref = grid[np.argmin(np.abs(sym_norm.imag[:, None] - grid[None, :]), axis=1)]
            ref_pts = i_ref + 1j * q_ref

        # Error vector
        err = sym_norm - ref_pts

        # Decompose error into radial (amplitude) and tangential (phase) components
        # For each subcarrier:
        #   - radial error = |received| - |reference| (projected along reference direction)
        #   - phase error = |reference| * sin(angle_between)
        ref_mag = np.abs(ref_pts)
        rx_mag = np.abs(sym_norm)
        # Angle between received and reference
        angle_err = np.angle(sym_norm) - np.angle(ref_pts)
        # Wrap to [-pi, pi]
        angle_err = (angle_err + np.pi) % (2 * np.pi) - np.pi

        # Phase (tangential) error: perpendicular to ref direction
        phase_err_component = ref_mag * np.sin(angle_err)
        # Amplitude (radial) error: along ref direction
        ampl_err_component = rx_mag * np.cos(angle_err) - ref_mag

        # RMS values
        evm_total = np.sqrt(np.mean(np.abs(err)**2))
        evm_phase = np.sqrt(np.mean(phase_err_component**2))
        evm_ampl = np.sqrt(np.mean(ampl_err_component**2))
        mean_phase_deg = np.degrees(np.mean(angle_err))

        evm_total_db = 20 * np.log10(evm_total) if evm_total > 0 else -60
        evm_phase_db = 20 * np.log10(evm_phase) if evm_phase > 0 else -60
        evm_ampl_db = 20 * np.log10(evm_ampl) if evm_ampl > 0 else -60

        # Per-subcarrier EVM
        sc_evm = np.abs(err)
        max_sc_evm_db = 20 * np.log10(np.max(sc_evm)) if np.max(sc_evm) > 0 else -60
        per_sc_evm_accum += sc_evm**2
        n_symbols += 1

        all_total_evm.append(evm_total_db)
        all_phase_evm.append(evm_phase_db)
        all_ampl_evm.append(evm_ampl_db)

        dut._log.info(f"  {sym_idx:3d} | {evm_total_db:+6.1f} dB | {evm_phase_db:+6.1f} dB "
                      f"| {evm_ampl_db:+5.1f} dB | {mean_phase_deg:+6.2f}°         "
                      f"| {max_sc_evm_db:+5.1f} dB")

    # Summary
    dut._log.info(f"")
    dut._log.info(f"  --- Summary ---")
    dut._log.info(f"  Mean EVM total:     {np.mean(all_total_evm):+.1f} dB")
    dut._log.info(f"  Mean EVM phase:     {np.mean(all_phase_evm):+.1f} dB")
    dut._log.info(f"  Mean EVM amplitude: {np.mean(all_ampl_evm):+.1f} dB")
    dut._log.info(f"")

    # Determine which dominates
    phase_power = 10**(np.mean(all_phase_evm)/10)
    ampl_power = 10**(np.mean(all_ampl_evm)/10)
    total_power = phase_power + ampl_power
    phase_pct = 100 * phase_power / total_power if total_power > 0 else 0
    ampl_pct = 100 * ampl_power / total_power if total_power > 0 else 0
    dut._log.info(f"  Dominant error: phase={phase_pct:.0f}%  amplitude={ampl_pct:.0f}%")

    if phase_pct > 70:
        dut._log.info(f"  DIAGNOSIS: Phase-dominated — better pilot tracking or SFO correction needed")
    elif ampl_pct > 70:
        dut._log.info(f"  DIAGNOSIS: Amplitude-dominated — quantization noise or channel estimation error")
    else:
        dut._log.info(f"  DIAGNOSIS: Mixed phase/amplitude — multiple error sources")

    # Per-subcarrier EVM profile (averaged over all symbols)
    dut._log.info(f"")
    dut._log.info(f"  --- Per-Subcarrier EVM Profile (averaged, 48 data subcarriers) ---")
    if n_symbols > 0:
        per_sc_evm_rms = np.sqrt(per_sc_evm_accum / n_symbols)
        per_sc_evm_db = 20 * np.log10(per_sc_evm_rms + 1e-10)

        # Group into 8 groups of 6 for readability
        dut._log.info(f"  (subcarrier index 0=lowest freq data SC, 47=highest)")
        for g in range(8):
            start = g * 6
            end = min(start + 6, 48)
            vals = per_sc_evm_db[start:end]
            bar = " ".join(f"{v:+5.1f}" for v in vals)
            dut._log.info(f"  SC[{start:2d}-{end-1:2d}]: {bar}")

        # Edge vs center comparison
        edge_evm = np.mean(np.concatenate([per_sc_evm_db[:6], per_sc_evm_db[-6:]]))
        center_evm = np.mean(per_sc_evm_db[18:30])
        dut._log.info(f"")
        dut._log.info(f"  Edge SC mean EVM:   {edge_evm:+.1f} dB")
        dut._log.info(f"  Center SC mean EVM: {center_evm:+.1f} dB")
        if edge_evm - center_evm > 2.0:
            dut._log.info(f"  NOTE: Edge subcarriers {edge_evm - center_evm:.1f} dB worse — "
                          f"suggests SFO or filter roll-off")

    dut._log.info(f"  === EVM decomposition complete ===")


@cocotb.test()
async def diag_rate36_cfo_sweep(dut):
    """Sweep CFO at rate 36 with per-symbol EVM measurement.

    Injects CFO from 0 to 5000 Hz into the rate 36 golden vector (no noise)
    and measures post-pilot_track EVM at each point. This isolates the pilot
    tracker's ability to handle residual CFO at 16-QAM modulation.

    Answers: "What CFO level causes rate 36 to fail, and is the failure
    from PLL lag or from something else?"
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    cfo_values = [0, 200, 500, 1000, 1500, 2000, 3000, 4000, 5000]
    dut._log.info(f"=== Rate 36 CFO Sweep — EVM vs residual CFO ===")
    dut._log.info(f"  CFO values: {cfo_values}")
    dut._log.info(f"  No noise (golden vector + CFO only)")
    dut._log.info(f"")
    dut._log.info(f"  CFO(Hz) | FCS | phase_inc | PT Mean EVM | PT Worst | drift°/sym")
    dut._log.info(f"  --------+-----+-----------+-------------+----------+-----------")

    for cfo_hz in cfo_values:
        await reset_dut(dut)
        iq = load_waveform_float(36)
        if cfo_hz > 0:
            iq = add_cfo(iq, cfo_hz=cfo_hz, sample_rate=SAMPLE_RATE)
        samples = quantize_12bit(iq)

        r = await _run_evm_analysis(dut, samples)

        fcs_str = "OK" if r['tag_fcs_ok'] else "FAIL"
        phase_inc = r.get('phase_inc', 0)

        # Compute post-PT EVM
        pt_syms = r.get('pt_symbols', [])
        if pt_syms and r['mod_order'] > 0:
            grid = np.array([-3.0, -1.0, 1.0, 3.0]) / math.sqrt(10.0)
            ref_rms = np.sqrt(np.mean(np.abs(pt_syms[0])**2)) if len(pt_syms) > 0 else 1.0
            if ref_rms == 0:
                ref_rms = 1.0
            pt_scale = 1.0 / ref_rms

            evm_per_sym = []
            for sym in pt_syms:
                sym_norm = sym * pt_scale
                i_ref = grid[np.argmin(np.abs(sym_norm.real[:, None] - grid[None, :]), axis=1)]
                q_ref = grid[np.argmin(np.abs(sym_norm.imag[:, None] - grid[None, :]), axis=1)]
                ref_pts = i_ref + 1j * q_ref
                err = sym_norm - ref_pts
                evm_rms = np.sqrt(np.mean(np.abs(err)**2))
                evm_db = 20 * np.log10(evm_rms) if evm_rms > 0 else -60.0
                evm_per_sym.append(evm_db)

            mean_evm = np.mean(evm_per_sym)
            worst_evm = max(evm_per_sym)
            # Drift: EVM degradation across symbols
            if len(evm_per_sym) > 1:
                drift = (evm_per_sym[-1] - evm_per_sym[0]) / max(1, len(evm_per_sym) - 1)
            else:
                drift = 0
        else:
            mean_evm = 0
            worst_evm = 0
            drift = 0

        dut._log.info(f"  {cfo_hz:5d}   | {fcs_str:4s}|    {phase_inc:+4d}   "
                      f"| {mean_evm:+6.1f} dB   | {worst_evm:+5.1f} dB"
                      f"| {drift:+5.2f}")

    dut._log.info(f"")
    dut._log.info(f"  === Rate 36 CFO sweep complete ===")


@cocotb.test()
async def diag_chan_est_quality(dut):
    """Probe channel estimate H and H_inv quality from an ADC capture.

    Dumps per-bin H magnitude, H_inv magnitude, and the expected vs actual
    equalization gain. Identifies whether the EVM floor comes from:
      - H quantization (LTF averaging truncation)
      - H_inv division precision
      - shift_val / EQ_SCALE interaction
    """
    capture_file = os.environ.get('ADC_CAPTURE_FILE', '')
    if not capture_file:
        captures_dir = os.path.join(os.path.dirname(__file__), '..', 'captures')
        import glob as globmod
        candidates = sorted(globmod.glob(os.path.join(captures_dir, '*.json')))
        if candidates:
            capture_file = candidates[0]
        else:
            cocotb.log.info("No ADC_CAPTURE_FILE set — skipping")
            return

    if not os.path.exists(capture_file):
        cocotb.log.warning(f"Capture file not found: {capture_file}")
        return

    cfo_threshold = int(os.environ.get('CFO_THRESHOLD', '64'))

    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    dut._log.info(f"=== Chan Est Quality: {os.path.basename(capture_file)} ===")

    samples = load_adc_capture(capture_file)
    dut._log.info(f"  Samples: {len(samples)}")

    n_samples = len(samples)
    post_zero_count = 8192
    total_feed = n_samples + post_zero_count

    # Feed samples and wait for chan_est done
    chan_est_done_seen = False
    ltf_peak_seen = False
    ltf_peak_pos = None
    ltf1_offset_val = None
    sample_idx = 0

    for cycle in range(5000000):
        await RisingEdge(dut.clk)

        # Feed IQ
        if sample_idx < n_samples:
            re_q, im_q = samples[sample_idx]
            dut.iq_valid_in.value = 1
            dut.iq_i_in.value = s12_to_unsigned(re_q)
            dut.iq_q_in.value = s12_to_unsigned(im_q)
            sample_idx += 1
        elif sample_idx < total_feed:
            dut.iq_valid_in.value = 1
            dut.iq_i_in.value = 0
            dut.iq_q_in.value = 0
            sample_idx += 1
        else:
            dut.iq_valid_in.value = 0

        # Probe ltf_peak_detect
        if not ltf_peak_seen:
            try:
                if int(dut.u_rx_pipeline.u_ltf_peak.peak_found.value) == 1:
                    ltf_peak_seen = True
                    ltf_peak_pos = int(dut.u_rx_pipeline.u_ltf_peak.peak_sample_pos.value)
                    ltf1_offset_val = int(dut.u_rx_pipeline.u_decode_engine.ltf1_offset.value)
                    dut._log.info(f"  ltf_peak found: peak_sample_pos={ltf_peak_pos}")
            except (ValueError, AttributeError):
                pass

        # Wait for chan_est done
        try:
            if int(dut.u_rx_pipeline.u_chan_est.done.value) == 1:
                chan_est_done_seen = True
                # Read ltf1_offset (latched by FSM)
                try:
                    ltf1_offset_val = int(dut.u_rx_pipeline.u_decode_engine.ltf1_offset.value)
                except (ValueError, AttributeError):
                    pass
                dut._log.info(f"  ltf1_offset (latched) = {ltf1_offset_val}")
                # Wait a few more cycles for BRAM writes to settle
                await ClockCycles(dut.clk, 5)
                break
        except (ValueError, AttributeError):
            pass

    if not chan_est_done_seen:
        dut._log.info("  chan_est done never asserted — no frame detected?")
        return

    # Read shift_val
    shift_val = int(dut.u_rx_pipeline.u_chan_est.shift_val.value)
    dut._log.info(f"  shift_val = {shift_val}")

    # Read H[k] and H_inv[k] for all 64 bins
    active_bins = list(range(1, 27)) + list(range(38, 64))
    pilot_bins = [7, 21, 43, 57]

    h_mags = []
    hinv_mags = []
    eq_gains = []

    dut._log.info(f"")
    dut._log.info(f"  --- Per-Bin Channel Estimate ---")
    dut._log.info(f"  {'bin':>4s} {'H_re':>7s} {'H_im':>7s} {'|H|':>7s} "
                  f"{'Hinv_re':>8s} {'Hinv_im':>8s} {'|Hinv|':>8s} {'EQ gain':>8s} {'type':>6s}")

    for k in active_bins:
        # Read H[k]
        h_re = int(dut.u_rx_pipeline.u_chan_est.h_mem_re[k].value)
        h_im = int(dut.u_rx_pipeline.u_chan_est.h_mem_im[k].value)
        if h_re >= 32768: h_re -= 65536
        if h_im >= 32768: h_im -= 65536

        # Read H_inv[k]
        hinv_re = int(dut.u_rx_pipeline.u_chan_est.hinv_mem_re[k].value)
        hinv_im = int(dut.u_rx_pipeline.u_chan_est.hinv_mem_im[k].value)
        if hinv_re >= 32768: hinv_re -= 65536
        if hinv_im >= 32768: hinv_im -= 65536

        h_mag = math.sqrt(h_re**2 + h_im**2)
        hinv_mag = math.sqrt(hinv_re**2 + hinv_im**2)

        # Expected EQ gain: H * H_inv >> (shift_val - EQ_SCALE) should = 1.0
        # |EQ gain| = |H| * |H_inv| / 2^(shift_val - EQ_SCALE)
        eff_shift = max(shift_val - 6, 0)  # EQ_SCALE = 6
        eq_gain = (h_mag * hinv_mag) / (2**eff_shift) if eff_shift > 0 else h_mag * hinv_mag

        h_mags.append(h_mag)
        hinv_mags.append(hinv_mag)
        eq_gains.append(eq_gain)

        bin_type = "PILOT" if k in pilot_bins else "data"
        dut._log.info(f"  {k:4d} {h_re:7d} {h_im:7d} {h_mag:7.1f} "
                      f"{hinv_re:8d} {hinv_im:8d} {hinv_mag:8.1f} {eq_gain:8.2f} {bin_type:>6s}")

    # Summary statistics
    dut._log.info(f"")
    dut._log.info(f"  --- Summary ---")
    h_mag_arr = np.array(h_mags)
    hinv_mag_arr = np.array(hinv_mags)
    eq_gain_arr = np.array(eq_gains)

    dut._log.info(f"  |H| range: {h_mag_arr.min():.1f} to {h_mag_arr.max():.1f} "
                  f"(spread: {h_mag_arr.max()/h_mag_arr.min():.2f}x)")
    dut._log.info(f"  |H_inv| range: {hinv_mag_arr.min():.1f} to {hinv_mag_arr.max():.1f} "
                  f"(spread: {hinv_mag_arr.max()/hinv_mag_arr.min():.2f}x)")
    dut._log.info(f"  EQ gain range: {eq_gain_arr.min():.2f} to {eq_gain_arr.max():.2f} "
                  f"(spread: {eq_gain_arr.max()/eq_gain_arr.min():.2f}x, ideal=64.0)")
    dut._log.info(f"  EQ gain mean: {eq_gain_arr.mean():.2f}, std: {eq_gain_arr.std():.2f}")
    dut._log.info(f"  EQ gain CV (std/mean): {eq_gain_arr.std()/eq_gain_arr.mean()*100:.1f}%")

    # The EVM floor from EQ gain variation alone:
    # If the mean is M and std is S, EVM ≈ S/M
    evm_from_gain = eq_gain_arr.std() / eq_gain_arr.mean()
    evm_db = 20 * math.log10(evm_from_gain) if evm_from_gain > 0 else -60
    dut._log.info(f"  Predicted EVM floor from gain spread: {evm_db:.1f} dB")

    # Check flatness of H (cable should be flat)
    h_flatness_db = 20 * math.log10(h_mag_arr.max() / h_mag_arr.min())
    dut._log.info(f"  Channel flatness (|H| max/min): {h_flatness_db:.1f} dB")

    dut._log.info(f"")
    dut._log.info(f"  === Chan est quality analysis complete ===")


@cocotb.test()
async def diag_fft_ltf_vs_data(dut):
    """Compare FFT input/output for LTF1 vs DATA1 — isolates phase corruption source.

    Captures:
      - 64 time-domain samples fed to the FFT for LTF1 and DATA1
      - 64 frequency-domain bins output by the FFT for LTF1 and DATA1
      - Python reference FFT of the same time-domain samples

    If RTL FFT output matches Python FFT of the same input → corruption is in
    the buffer data (samples are wrong before FFT).
    If RTL FFT output differs from Python FFT → fft64_sdf has a bug.

    This directly answers: "Is the phase corruption in the FFT input or output?"
    """
    capture_file = os.environ.get('ADC_CAPTURE_FILE', '')
    if not capture_file:
        captures_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'captures')
        import glob as globmod
        candidates = sorted(globmod.glob(os.path.join(captures_dir, '**', '24m*.json'), recursive=True))
        if not candidates:
            candidates = sorted(globmod.glob(os.path.join(captures_dir, '**', '*.json'), recursive=True))
        if candidates:
            capture_file = candidates[0]
        else:
            cocotb.log.info("No ADC_CAPTURE_FILE set and no captures found — skipping")
            return

    if not os.path.exists(capture_file):
        cocotb.log.warning(f"Capture file not found: {capture_file}")
        return

    cfo_threshold = int(os.environ.get('CFO_THRESHOLD', '64'))

    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    dut._log.info(f"=== FFT LTF vs DATA Comparison: {os.path.basename(capture_file)} ===")

    samples = load_adc_capture(capture_file)
    dut._log.info(f"  Samples: {len(samples)}")

    n_samples = len(samples)
    post_zero_count = 8192
    total_feed = n_samples + post_zero_count

    # Signal hierarchy (D19: rx_top/fft_wrap were removed — the FFT is driven
    # by decode_engine and instantiated as u_fft):
    #   dut.u_rx_pipeline.u_decode_engine.state      — FSM state
    #   dut.u_rx_pipeline.u_decode_engine.fft_phase  — 0=LTF1 1=LTF2 2=SIG 3=DATA
    #   dut.u_rx_pipeline.u_fft.din_valid/re/im      — FFT input
    #   dut.u_rx_pipeline.u_decode_engine.fft_bin_*  — natural-order bin output

    S_FFT_FEED = 1
    PHASE_LTF1 = 0
    PHASE_DATA = 3

    # Capture arrays
    ltf1_input_re = []
    ltf1_input_im = []
    ltf1_output = [None] * 64  # indexed by bin

    data1_input_re = []
    data1_input_im = []
    data1_output = [None] * 64

    sample_idx = 0

    def _s16(v):
        v = int(v)
        return v - 65536 if v >= 32768 else v

    for cycle in range(5000000):
        await RisingEdge(dut.clk)

        # Feed IQ
        if sample_idx < n_samples:
            re_q, im_q = samples[sample_idx]
            dut.iq_valid_in.value = 1
            dut.iq_i_in.value = s12_to_unsigned(re_q)
            dut.iq_q_in.value = s12_to_unsigned(im_q)
            sample_idx += 1
        elif sample_idx < total_feed:
            dut.iq_valid_in.value = 1
            dut.iq_i_in.value = 0
            dut.iq_q_in.value = 0
            sample_idx += 1
        else:
            dut.iq_valid_in.value = 0

        # Which FFT symbol is being fed / read out right now. Internal
        # signal reads work because Verilator is built with --public-flat-rw.
        phase = None
        try:
            de = dut.u_rx_pipeline.u_decode_engine
            if int(de.state.value) == S_FFT_FEED:
                ph = int(de.fft_phase.value)
                if ph == PHASE_LTF1:
                    phase = 'ltf1'
                elif ph == PHASE_DATA and int(de.symbol_idx_out.value) == 1:
                    phase = 'data1'   # first DATA symbol only
        except (ValueError, AttributeError):
            pass

        # FFT input samples (fft64_sdf instance ports)
        if phase is not None:
            try:
                if int(dut.u_rx_pipeline.u_fft.din_valid.value):
                    din_re = _s16(dut.u_rx_pipeline.u_fft.din_re.value)
                    din_im = _s16(dut.u_rx_pipeline.u_fft.din_im.value)
                    if phase == 'ltf1' and len(ltf1_input_re) < 64:
                        ltf1_input_re.append(din_re)
                        ltf1_input_im.append(din_im)
                    elif phase == 'data1' and len(data1_input_re) < 64:
                        data1_input_re.append(din_re)
                        data1_input_im.append(din_im)
            except (ValueError, AttributeError):
                pass

        # FFT output bins (decode_engine's natural-order registered output).
        # Outputs lag inputs but stay within the same fft_phase, so phase
        # attribution is valid.
        if phase is not None:
            try:
                de = dut.u_rx_pipeline.u_decode_engine
                if int(de.fft_bin_valid.value):
                    bidx = int(de.fft_bin_idx.value)
                    if 0 <= bidx < 64:
                        bre = _s16(de.fft_bin_re.value)
                        bim = _s16(de.fft_bin_im.value)
                        if phase == 'ltf1':
                            ltf1_output[bidx] = complex(bre, bim)
                        elif phase == 'data1':
                            data1_output[bidx] = complex(bre, bim)
            except (ValueError, AttributeError):
                pass

        # Early exit if we have everything
        if (len(data1_input_re) >= 64 and
            all(x is not None for x in ltf1_output) and
            all(x is not None for x in data1_output)):
            break

    # --- Analysis ---
    dut._log.info(f"  LTF1 input samples captured: {len(ltf1_input_re)}")
    dut._log.info(f"  DATA1 input samples captured: {len(data1_input_re)}")
    ltf1_out_count = sum(1 for x in ltf1_output if x is not None)
    data1_out_count = sum(1 for x in data1_output if x is not None)
    dut._log.info(f"  LTF1 output bins captured: {ltf1_out_count}")
    dut._log.info(f"  DATA1 output bins captured: {data1_out_count}")

    if len(ltf1_input_re) < 64 or len(data1_input_re) < 64:
        dut._log.info(f"  INCOMPLETE — not enough data captured. Aborting analysis.")
        return

    if ltf1_out_count < 64 or data1_out_count < 64:
        dut._log.info(f"  INCOMPLETE — FFT output not fully captured. Aborting analysis.")
        return

    # Compute Python reference FFT on the captured inputs
    ltf1_td = np.array(ltf1_input_re) + 1j * np.array(ltf1_input_im)
    data1_td = np.array(data1_input_re) + 1j * np.array(data1_input_im)

    ltf1_fft_ref = np.fft.fft(ltf1_td)
    data1_fft_ref = np.fft.fft(data1_td)

    ltf1_rtl = np.array(ltf1_output)
    data1_rtl = np.array(data1_output)

    # Compare RTL vs Python for LTF1
    dut._log.info(f"  --- LTF1: RTL vs Python FFT ---")
    ltf1_err = ltf1_rtl - ltf1_fft_ref
    ltf1_mag = np.abs(ltf1_fft_ref)
    ltf1_mag_safe = np.where(ltf1_mag > 1, ltf1_mag, 1)
    ltf1_phase_err = np.degrees(np.angle(ltf1_rtl / np.where(ltf1_fft_ref != 0, ltf1_fft_ref, 1)))

    # Print pilot bins (7, 21, 43, 57) and a few data bins
    for bidx in [7, 21, 43, 57, 1, 38, 63]:
        if ltf1_fft_ref[bidx] != 0:
            dut._log.info(f"  bin {bidx:2d}: RTL=({ltf1_rtl[bidx].real:+7.0f},{ltf1_rtl[bidx].imag:+7.0f})"
                         f"  Py=({ltf1_fft_ref[bidx].real:+7.0f},{ltf1_fft_ref[bidx].imag:+7.0f})"
                         f"  phase_err={ltf1_phase_err[bidx]:+5.1f}°")

    ltf1_active = [i for i in range(64) if i != 0 and i != 32 and ltf1_mag[i] > 10]
    if ltf1_active:
        ltf1_pe_active = ltf1_phase_err[ltf1_active]
        dut._log.info(f"  LTF1 phase error (active bins): mean={np.mean(ltf1_pe_active):+.1f}°"
                     f"  std={np.std(ltf1_pe_active):.1f}°  max={np.max(np.abs(ltf1_pe_active)):.1f}°")

    # Compare RTL vs Python for DATA1
    dut._log.info(f"  --- DATA1: RTL vs Python FFT ---")
    data1_err = data1_rtl - data1_fft_ref
    data1_mag = np.abs(data1_fft_ref)
    data1_phase_err = np.degrees(np.angle(data1_rtl / np.where(data1_fft_ref != 0, data1_fft_ref, 1)))

    for bidx in [7, 21, 43, 57, 1, 38, 63]:
        if data1_fft_ref[bidx] != 0:
            dut._log.info(f"  bin {bidx:2d}: RTL=({data1_rtl[bidx].real:+7.0f},{data1_rtl[bidx].imag:+7.0f})"
                         f"  Py=({data1_fft_ref[bidx].real:+7.0f},{data1_fft_ref[bidx].imag:+7.0f})"
                         f"  phase_err={data1_phase_err[bidx]:+5.1f}°")

    data1_active = [i for i in range(64) if i != 0 and i != 32 and data1_mag[i] > 10]
    if data1_active:
        data1_pe_active = data1_phase_err[data1_active]
        dut._log.info(f"  DATA1 phase error (active bins): mean={np.mean(data1_pe_active):+.1f}°"
                     f"  std={np.std(data1_pe_active):.1f}°  max={np.max(np.abs(data1_pe_active)):.1f}°")

    # Key comparison: positive vs negative frequency bins
    pos_bins = [i for i in range(1, 27)]  # bins 1-26
    neg_bins = [i for i in range(38, 64)]  # bins 38-63

    dut._log.info(f"  --- Phase Error Split: Positive vs Negative Freq ---")
    if ltf1_active:
        ltf1_pos_err = [ltf1_phase_err[i] for i in pos_bins if ltf1_mag[i] > 10]
        ltf1_neg_err = [ltf1_phase_err[i] for i in neg_bins if ltf1_mag[i] > 10]
        if ltf1_pos_err and ltf1_neg_err:
            dut._log.info(f"  LTF1 pos-freq: mean={np.mean(ltf1_pos_err):+.1f}° std={np.std(ltf1_pos_err):.1f}°")
            dut._log.info(f"  LTF1 neg-freq: mean={np.mean(ltf1_neg_err):+.1f}° std={np.std(ltf1_neg_err):.1f}°")

    if data1_active:
        data1_pos_err = [data1_phase_err[i] for i in pos_bins if data1_mag[i] > 10]
        data1_neg_err = [data1_phase_err[i] for i in neg_bins if data1_mag[i] > 10]
        if data1_pos_err and data1_neg_err:
            dut._log.info(f"  DATA1 pos-freq: mean={np.mean(data1_pos_err):+.1f}° std={np.std(data1_pos_err):.1f}°")
            dut._log.info(f"  DATA1 neg-freq: mean={np.mean(data1_neg_err):+.1f}° std={np.std(data1_neg_err):.1f}°")

    # Time-domain comparison: power profile of inputs
    ltf1_power = np.abs(ltf1_td)**2
    data1_power = np.abs(data1_td)**2
    dut._log.info(f"  --- Time-Domain Input Stats ---")
    dut._log.info(f"  LTF1:  mean_power={np.mean(ltf1_power):.0f}  peak={np.max(ltf1_power):.0f}")
    dut._log.info(f"  DATA1: mean_power={np.mean(data1_power):.0f}  peak={np.max(data1_power):.0f}")

    # Check if inputs themselves show the phase anomaly by computing
    # the ratio DATA1_FFT / LTF1_FFT (this is essentially what EQ does)
    dut._log.info(f"  --- EQ-equivalent: DATA1_FFT / LTF1_FFT (Python) ---")
    for bidx in [7, 21, 43, 57]:
        if ltf1_fft_ref[bidx] != 0:
            ratio = data1_fft_ref[bidx] / ltf1_fft_ref[bidx]
            dut._log.info(f"  pilot bin {bidx:2d}: mag={np.abs(ratio):.3f}  phase={np.degrees(np.angle(ratio)):+.1f}°")

    dut._log.info(f"  === FFT LTF vs DATA comparison complete ===")
