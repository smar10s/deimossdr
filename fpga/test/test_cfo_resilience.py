"""
test_cfo_resilience.py — Gate test: decode must succeed with realistic residual CFO.

THE PROBLEM THIS CATCHES:
  The coarse CFO estimator quantizes to integer phase_inc values. With same-crystal
  cable loopback (CFO ≈ 0), the estimator often outputs phase_inc=0. But on real
  traffic (or noisy loopback), the estimate varies frame-to-frame: phase_inc in
  [-6, +6] is normal. A residual of ±5 corresponds to ~1.5 kHz residual CFO,
  producing ~2.2°/symbol phase drift that pilot_track must correct.

  If pilot_track can't handle this drift (e.g., dead-zone blocks correction),
  long frames fail and the hardware shows high variance (45-95% pass rate).

WHAT THIS TESTS:
  Golden vectors with synthetic CFO injected, covering the residual range hardware
  actually sees. Each rate at each CFO must decode with FCS OK.

INTEGRATION:
  This is a gate test (test_*). It runs in sim.sh alongside test_rx_frontend.
  Uses rx_frontend DUT (no dead-zone — pilot PLL handles residual).
"""

import os
import sys

import numpy as np
import cocotb
from cocotb.clock import Clock

from frontend_helpers import (
    load_waveform_float, quantize_12bit, s12_to_unsigned,
    reset_dut, run_frontend_decode_live,
    RATE_CODES, EXPECTED_PSDU_LEN, SAMPLE_RATE,
)

# Add lib80211 for impairments
LIB80211_PYTHON = os.path.join(os.path.dirname(__file__), '..', '..', 'extern', 'lib80211', 'python')
sys.path.insert(0, LIB80211_PYTHON)
from py80211.impairments import add_cfo, add_awgn


def phase_inc_to_cfo_hz(phase_inc):
    """Convert integer phase_inc to CFO in Hz (matches RTL conversion)."""
    return phase_inc * SAMPLE_RATE / 65536


# Residual CFO values to test: these are the phase_inc values we observe on hardware.
# phase_inc=-5 → ~1.5 kHz residual → ~2.2°/symbol drift (the failure case).
# We test symmetric positive/negative and a few magnitudes.
RESIDUAL_PHASE_INCS = [-6, -4, -2, 0, 2, 4, 6]

# Rates to test: all 8 legacy rates.
# Lower-order modulations (BPSK/QPSK) are more tolerant but have longer frames.
# Higher-order (16/64-QAM) are less tolerant but have shorter frames.
RATES = [6, 9, 12, 18, 24, 36, 48, 54]

# Fixed seed for the AWGN noise — gate runs must be reproducible.
# (Unseeded, a marginal-case flake is possible even though 35 dB is robust.)
AWGN_SEED = 1701


@cocotb.test()
async def test_cfo_resilience_rate6(dut):
    """Rate 6 (BPSK 1/2) must decode at all residual CFO values.

    Rate 6 is critical: longest frames (37 DATA symbols for 100-byte payload),
    most vulnerable to progressive phase drift. This is the rate that shows
    45-95% variance on hardware when pilot_track doesn't correct properly.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    iq = load_waveform_float(6)
    failures = []

    for pi in RESIDUAL_PHASE_INCS:
        await reset_dut(dut)

        # Inject CFO corresponding to this phase_inc residual
        cfo_hz = phase_inc_to_cfo_hz(pi)
        iq_cfo = add_cfo(iq, cfo_hz)
        iq_noisy = add_awgn(iq_cfo, snr_db=35, seed=AWGN_SEED)
        samples = quantize_12bit(iq_noisy)

        r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500, snr_db=35)

        passed = r['tag_valid'] and r['tag_fcs_ok']
        status = "PASS" if passed else "FAIL"
        dut._log.info(f"  Rate 6, phase_inc={pi:+d} ({cfo_hz:+.0f} Hz): "
                      f"{status} (actual pi={r['phase_inc']})")
        if not passed:
            failures.append(pi)

    assert not failures, \
        f"Rate 6 failed at residual CFO phase_inc={failures}. " \
        f"pilot_track must handle ±6 phase_inc (~1.8 kHz) without FCS failure."


@cocotb.test()
async def test_cfo_resilience_rate24(dut):
    """Rate 24 (16-QAM 1/2) must decode at all residual CFO values.

    Rate 24 is the EAPOL rate — critical for the project goal. 16-QAM is
    more sensitive to phase error than BPSK but frames are shorter.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    iq = load_waveform_float(24)
    failures = []

    for pi in RESIDUAL_PHASE_INCS:
        await reset_dut(dut)

        cfo_hz = phase_inc_to_cfo_hz(pi)
        iq_cfo = add_cfo(iq, cfo_hz)
        iq_noisy = add_awgn(iq_cfo, snr_db=35, seed=AWGN_SEED)
        samples = quantize_12bit(iq_noisy)

        r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500, snr_db=35)

        passed = r['tag_valid'] and r['tag_fcs_ok']
        status = "PASS" if passed else "FAIL"
        dut._log.info(f"  Rate 24, phase_inc={pi:+d} ({cfo_hz:+.0f} Hz): "
                      f"{status} (actual pi={r['phase_inc']})")
        if not passed:
            failures.append(pi)

    assert not failures, \
        f"Rate 24 failed at residual CFO phase_inc={failures}. " \
        f"pilot_track must handle ±6 phase_inc (~1.8 kHz) without FCS failure."


@cocotb.test()
async def test_cfo_resilience_all_rates(dut):
    """All 8 rates must decode at phase_inc=±4 (the most common non-zero residual).

    This is the primary gate: ±4 phase_inc (~1.2 kHz) is within normal
    estimator variance on cable loopback. Every rate must handle it.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    TEST_PHASE_INCS = [-4, 4]
    failures = []

    for rate in RATES:
        iq = load_waveform_float(rate)
        for pi in TEST_PHASE_INCS:
            await reset_dut(dut)

            cfo_hz = phase_inc_to_cfo_hz(pi)
            iq_cfo = add_cfo(iq, cfo_hz)
            iq_noisy = add_awgn(iq_cfo, snr_db=35, seed=AWGN_SEED)
            samples = quantize_12bit(iq_noisy)

            r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500, snr_db=35)

            passed = r['tag_valid'] and r['tag_fcs_ok']
            status = "PASS" if passed else "FAIL"
            dut._log.info(f"  Rate {rate:2d}, pi={pi:+d}: {status} (actual pi={r['phase_inc']})")
            if not passed:
                failures.append((rate, pi))

    dut._log.info(f"CFO resilience: {len(RATES)*len(TEST_PHASE_INCS) - len(failures)}"
                  f"/{len(RATES)*len(TEST_PHASE_INCS)} pass")

    assert not failures, \
        f"CFO resilience failures: {failures}. " \
        f"All rates must decode with ±4 phase_inc residual (~1.2 kHz)."
