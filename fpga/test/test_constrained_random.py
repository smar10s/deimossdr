"""
test_constrained_random.py — Constrained-random integration test.

Exercises the full pipeline with randomized impairments that model the
range of real-world conditions. Each trial applies a unique combination
of CFO, SNR, multipath, and SFO to a golden vector, then asserts FCS
pass.

This catches corner-case interactions between impairments that fixed
test vectors cannot reach. The parametric ranges are tuned to match
what cable loopback and OTA conditions actually produce.

GATE: Per-rate pass rate must meet threshold. Thresholds are set
conservatively so that legitimate failures (not test noise) are caught.

REPRODUCIBILITY: All randomness is seeded. A failure on seed N is
reproducible by re-running with the same seed.
"""

import os
import sys

import numpy as np
import cocotb
from cocotb.clock import Clock

from frontend_helpers import (
    load_waveform_float, quantize_12bit,
    reset_dut, run_frontend_decode_live,
    RATE_CODES, SAMPLE_RATE,
)

# Add lib80211 for impairments
LIB80211_PYTHON = os.path.join(os.path.dirname(__file__), '..', '..', 'extern', 'lib80211', 'python')
sys.path.insert(0, LIB80211_PYTHON)
from py80211.impairments import add_cfo, add_awgn, add_sfo, apply_multipath


# =========================================================
# Impairment parameter ranges
#
# These model the conditions seen in cable loopback and OTA:
#   CFO:       ±5 kHz (residual after coarse correction)
#   SNR:       20-35 dB (cable: >30; OTA: 20-30 typical)
#   SFO:       0-5 ppm (crystal mismatch between TX and RX)
#   Multipath: 1-3 taps, delays 0-15 samples, gains -0.5 to +0.5
# =========================================================

TRIALS_PER_RATE = 30
BASE_SEED = 2026_07_03  # Date-based for this session; never change

# Pass rate thresholds per rate group.
# Lower rates (BPSK/QPSK) are more robust; higher rates need more SNR headroom.
THRESHOLDS = {
    6:  0.90,   # BPSK 1/2 — most robust
    9:  0.90,   # BPSK 3/4
    12: 0.90,   # QPSK 1/2
    18: 0.90,   # QPSK 3/4
    24: 0.90,   # 16-QAM 1/2
    36: 0.85,   # 16-QAM 3/4 — tighter EVM budget
    48: 0.75,   # 64-QAM 2/3 — narrow margin
    54: 0.70,   # 64-QAM 3/4 — narrowest margin
}

# Rates to test (all 8)
RATES = [6, 9, 12, 18, 24, 36, 48, 54]


def generate_impairments(rng, rate):
    """Generate a random impairment set appropriate for the rate.

    Higher-order modulations get more conservative impairment ranges
    (matching reality: 64-QAM is only used at high SNR).
    """
    # SNR: higher rates need better SNR to decode
    if rate <= 18:
        snr_db = rng.uniform(22, 35)
    elif rate <= 36:
        snr_db = rng.uniform(25, 35)
    else:
        snr_db = rng.uniform(28, 38)

    # CFO: residual after coarse correction (±5 kHz)
    cfo_hz = rng.uniform(-5000, 5000)

    # SFO: crystal mismatch (0-5 ppm)
    sfo_ppm = rng.uniform(0, 5)

    # Multipath: random 1-3 taps
    n_taps = rng.integers(1, 4)  # 1 to 3 extra taps
    taps = [(0, 1.0 + 0j)]  # LOS always present
    for _ in range(n_taps):
        delay = int(rng.integers(1, 12))
        # Gain: weaker for higher rates (less ISI tolerance)
        max_gain = 0.3 if rate >= 48 else 0.4
        gain_mag = rng.uniform(0.05, max_gain)
        gain_phase = rng.uniform(0, 2 * np.pi)
        gain = gain_mag * np.exp(1j * gain_phase)
        taps.append((delay, gain))

    return {
        'snr_db': snr_db,
        'cfo_hz': cfo_hz,
        'sfo_ppm': sfo_ppm,
        'multipath_taps': taps,
    }


def apply_impairments(iq, params, rng):
    """Apply the impairment set to an IQ waveform."""
    result = iq.copy()

    # Multipath first (before CFO, since channel is independent of oscillator)
    result = apply_multipath(result, params['multipath_taps'])

    # SFO (resampling)
    if params['sfo_ppm'] > 0.1:
        result = add_sfo(result, params['sfo_ppm'], SAMPLE_RATE)

    # CFO (oscillator offset)
    result = add_cfo(result, params['cfo_hz'], SAMPLE_RATE)

    # AWGN last (additive, independent of signal)
    seed = int(rng.integers(0, 2**31))
    result = add_awgn(result, params['snr_db'], seed=seed)

    return result


@cocotb.test()
async def test_constrained_random_all_rates(dut):
    """Constrained-random: all 8 rates with randomized impairments.

    Runs TRIALS_PER_RATE trials per rate with random CFO, SNR, multipath,
    and SFO. Asserts per-rate pass rate meets threshold.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    overall_results = {}

    for rate in RATES:
        iq_clean = load_waveform_float(rate)
        pass_count = 0
        fail_details = []

        for trial in range(TRIALS_PER_RATE):
            seed = BASE_SEED * 1000 + rate * 100 + trial
            rng = np.random.default_rng(seed)

            params = generate_impairments(rng, rate)
            iq_impaired = apply_impairments(iq_clean, params, rng)
            samples = quantize_12bit(iq_impaired)

            await reset_dut(dut)
            r = await run_frontend_decode_live(dut, samples, pre_noise_samples=300)

            passed = r['tag_valid'] and r['tag_fcs_ok']
            if passed:
                pass_count += 1
            else:
                fail_details.append({
                    'trial': trial,
                    'seed': seed,
                    'snr_db': params['snr_db'],
                    'cfo_hz': params['cfo_hz'],
                    'sfo_ppm': params['sfo_ppm'],
                    'n_taps': len(params['multipath_taps']),
                    'tag_valid': r['tag_valid'],
                })

        pass_rate = pass_count / TRIALS_PER_RATE
        overall_results[rate] = {
            'pass_count': pass_count,
            'total': TRIALS_PER_RATE,
            'pass_rate': pass_rate,
            'threshold': THRESHOLDS[rate],
            'failures': fail_details,
        }

        dut._log.info(
            f"Rate {rate:2d}: {pass_count}/{TRIALS_PER_RATE} "
            f"({pass_rate*100:.0f}%) [threshold: {THRESHOLDS[rate]*100:.0f}%]"
        )
        if fail_details:
            # Log first 3 failures for debugging
            for fd in fail_details[:3]:
                dut._log.info(
                    f"  FAIL trial {fd['trial']} (seed={fd['seed']}): "
                    f"SNR={fd['snr_db']:.1f}dB CFO={fd['cfo_hz']:.0f}Hz "
                    f"SFO={fd['sfo_ppm']:.1f}ppm taps={fd['n_taps']} "
                    f"tag_valid={fd['tag_valid']}"
                )

    # Summary
    dut._log.info("=== Constrained-Random Summary ===")
    all_pass = True
    for rate in RATES:
        r = overall_results[rate]
        status = "PASS" if r['pass_rate'] >= r['threshold'] else "FAIL"
        dut._log.info(
            f"  Rate {rate:2d}: {r['pass_count']}/{r['total']} "
            f"({r['pass_rate']*100:.0f}%) [{status}]"
        )
        if r['pass_rate'] < r['threshold']:
            all_pass = False

    # Assert all rates meet threshold
    failing_rates = [
        rate for rate in RATES
        if overall_results[rate]['pass_rate'] < overall_results[rate]['threshold']
    ]
    assert not failing_rates, (
        f"Rates below threshold: "
        + ", ".join(
            f"{r} ({overall_results[r]['pass_rate']*100:.0f}% < {overall_results[r]['threshold']*100:.0f}%)"
            for r in failing_rates
        )
    )
