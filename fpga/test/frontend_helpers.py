"""
frontend_helpers.py — Shared utilities for rx_frontend tests and diagnostics.

Contains waveform loading, quantization, DUT reset, and IQ feed routines.
Used by both test_rx_frontend.py (gate tests) and diag_rx_frontend.py
(diagnostic tools).
"""

import os
import sys
import json
import math

import numpy as np
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer

# Add lib80211 Python path for impairment functions
LIB80211_PYTHON = os.path.join(os.path.dirname(__file__), '..', '..', 'extern', 'lib80211', 'python')
sys.path.insert(0, LIB80211_PYTHON)

from py80211.impairments import add_cfo, add_awgn, add_dc_offset

# Path to golden vectors
VECTORS_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'extern', 'lib80211', 'vectors')

# Rate code table
RATE_CODES = {
    6:  0b1011, 9:  0b1111, 12: 0b1010, 18: 0b1110,
    24: 0b1001, 36: 0b1101, 48: 0b1000, 54: 0b1100,
}
EXPECTED_PSDU_LEN = 100
SAMPLE_RATE = 20_000_000  # 20 MSPS

# Expected emitted PSDU payload for the golden legacy vectors.
# All legacy_*mbps_waveform.json vectors carry the IEEE 802.11 Annex I.1
# PSDU (100 octets incl FCS); psdu_packer emits psdu_len - 4 = 96 payload
# bytes (SERVICE skipped, FCS not emitted). Source of truth:
#   extern/lib80211/vectors/annex_i1_psdu.json
#   extern/lib80211/scripts/gen_legacy_vectors.py
with open(os.path.join(VECTORS_DIR, 'annex_i1_psdu.json')) as _f:
    _annex = json.load(_f)
EXPECTED_PSDU_PAYLOAD = bytes(int(h, 16) for h in _annex['octets_hex'][:-4])


def verify_psdu_path(result, expect_content=True, expected_payload=None,
                     expected_count=None):
    """Check the psdu_packer byte path from a run_frontend_decode* result.

    Covers the psdu_packer → tag_fifo_axi byte stream that deimos_rx_poll
    depends on in hardware (and that sim previously never exercised):
      - psdu_frame_done must be observed
      - emitted byte count must equal expected_count
      - collected bytes must equal that count
      - if expect_content: bytes must equal expected_payload

    Defaults match the Annex I.1 golden vectors (100-octet PSDU, 96 payload
    bytes emitted). Pass explicit values for stimulus/capture gates.

    Returns (ok: bool, reason: str). Callers decide assert vs report.
    """
    if expected_payload is None:
        expected_payload = EXPECTED_PSDU_PAYLOAD
    if expected_count is None:
        expected_count = EXPECTED_PSDU_LEN - 4
    if not result.get('psdu_frame_done_seen'):
        return False, "psdu_frame_done never observed"
    n = result.get('psdu_byte_count_final', 0)
    if n != expected_count:
        return False, f"psdu_byte_count={n}, expected {expected_count}"
    got = bytes(result.get('psdu_bytes', []))
    if len(got) != n:
        return False, f"collected {len(got)} bytes but byte_count={n}"
    if expect_content and got != expected_payload:
        return False, "emitted bytes differ from expected payload"
    return True, "ok"


def load_waveform_float(rate_mbps):
    """Load golden vector waveform as complex float numpy array."""
    path = os.path.join(VECTORS_DIR, f'legacy_{rate_mbps}mbps_waveform.json')
    with open(path) as f:
        data = json.load(f)
    return np.array(data['real'], dtype=np.float64) + 1j * np.array(data['imag'], dtype=np.float64)


def quantize_12bit(iq_complex):
    """Quantize complex float IQ to 12-bit signed samples.

    Uses 80% of dynamic range (matching test_rx_pipeline convention).
    Returns list of (re_q, im_q) tuples, signed 12-bit.
    """
    re = iq_complex.real
    im = iq_complex.imag
    peak = max(np.abs(re).max(), np.abs(im).max())
    if peak == 0:
        peak = 1.0
    scale = 2047.0 / peak

    re_q = np.clip(np.round(re * scale), -2048, 2047).astype(int)
    im_q = np.clip(np.round(im * scale), -2048, 2047).astype(int)
    return list(zip(re_q.tolist(), im_q.tolist()))


def s12_to_unsigned(v):
    """Convert signed 12-bit to unsigned for DUT input."""
    if v < 0:
        v += 4096
    return v & 0xFFF


async def reset_dut(dut):
    """Reset DUT and set default configuration."""
    dut.rst_n.value = 0
    dut.iq_valid_in.value = 0
    dut.iq_i_in.value = 0
    dut.iq_q_in.value = 0
    dut.playback_start.value = 0
    dut.stf_end_skip.value = 4       # legacy (no longer used for skip shortening)
    dut.ltf_skip.value = 1           # live mode: capture starts immediately after trigger
    dut.stf_threshold.value = 0      # default: shift=0, standard 0.36 threshold
    await ClockCycles(dut.clk, 20)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    # Pulse playback_start to trigger BRAM clearing FSM in stf_detect.
    # Without this, stale BRAM data from prior tests corrupts autocorrelation
    # during the first ~82 samples, which can misalign detection timing.
    dut.playback_start.value = 1
    await ClockCycles(dut.clk, 1)
    dut.playback_start.value = 0
    # Wait for BRAM clearing FSM to complete (DEPTH=82 clocks)
    await ClockCycles(dut.clk, 90)


async def measure_symbol_cycle(dut, rate, max_cycles=500000):
    """Feed golden waveform at live rate, measure DATA-symbol cycle count.

    Returns (avg, deltas, decoded) where:
      avg     - average clocks between consecutive symbol_start events
                for DATA symbols (excludes SIGNAL→DATA1 transition)
      deltas  - list of all measured deltas (for min/max reporting)
      decoded - True if tag_valid fired (frame decoded to completion)

    Wire-speed reference: 400 clocks/symbol (80 samples × 5 clocks/sample
    at 100 MHz fabric / 20 MSPS ADC). avg <= 400 means the rate streams
    in real time without backpressure stalls.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    iq_float = load_waveform_float(rate)
    iq_samples = quantize_12bit(iq_float)
    n_samples = len(iq_samples)

    # Collect symbol_start cycle numbers
    symbol_start_cycles = []
    sample_idx = 0
    VALID_SPACING = 5  # live-mode: 1 sample per 5 clocks (20 MSPS @ 100 MHz)
    decoded = False

    for cycle in range(max_cycles):
        await RisingEdge(dut.clk)

        # Feed IQ at live-mode rate
        if sample_idx < n_samples and (cycle % VALID_SPACING) == 0:
            re_q, im_q = iq_samples[sample_idx]
            dut.iq_valid_in.value = 1
            dut.iq_i_in.value = s12_to_unsigned(re_q)
            dut.iq_q_in.value = s12_to_unsigned(im_q)
            sample_idx += 1
        else:
            dut.iq_valid_in.value = 0

        # Monitor symbol_start_out
        try:
            sym_start = int(dut.u_rx_pipeline.u_decode_engine.symbol_start_out.value)
            if sym_start:
                symbol_start_cycles.append(cycle)
        except (ValueError, AttributeError):
            pass

        # Stop at tag_valid (frame decode complete)
        try:
            if int(dut.tag_valid.value) == 1:
                decoded = True
                break
        except (ValueError, AttributeError):
            pass

    if len(symbol_start_cycles) < 3:
        return None, [], decoded

    # Compute DATA-to-DATA deltas (skip first delta which is SIGNAL→DATA1)
    all_deltas = [
        symbol_start_cycles[i] - symbol_start_cycles[i - 1]
        for i in range(1, len(symbol_start_cycles))
    ]
    data_deltas = all_deltas[1:]  # skip SIGNAL→DATA1

    if not data_deltas:
        return None, [], decoded

    avg = sum(data_deltas) / len(data_deltas)
    return avg, data_deltas, decoded


async def measure_compute_window(dut, rate, max_cycles=500000):
    """Measure the feed-independent per-symbol compute window.

    Returns (avg, windows, decoded) where window = deint_done(N) - symbol_start(N).

    Once the pipeline is faster than the live feed, the symbol_start *period*
    (measure_symbol_cycle) is clamped near 400 and hides the true per-symbol
    cost. This window is measured entirely inside one symbol's decode, so it is
    unaffected by feed spacing and is the correct latency-regression signal.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    iq_float = load_waveform_float(rate)
    iq_samples = quantize_12bit(iq_float)
    n_samples = len(iq_samples)

    symbol_starts = []
    deint_dones = []
    sample_idx = 0
    VALID_SPACING = 5  # live-mode: 1 sample per 5 clocks (20 MSPS @ 100 MHz)
    decoded = False

    for cycle in range(max_cycles):
        await RisingEdge(dut.clk)

        if sample_idx < n_samples and (cycle % VALID_SPACING) == 0:
            re_q, im_q = iq_samples[sample_idx]
            dut.iq_valid_in.value = 1
            dut.iq_i_in.value = s12_to_unsigned(re_q)
            dut.iq_q_in.value = s12_to_unsigned(im_q)
            sample_idx += 1
        else:
            dut.iq_valid_in.value = 0

        try:
            if int(dut.u_rx_pipeline.u_decode_engine.symbol_start_out.value):
                symbol_starts.append(cycle)
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.u_rx_pipeline.deint_done.value):
                deint_dones.append(cycle)
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.tag_valid.value) == 1:
                decoded = True
                break
        except (ValueError, AttributeError):
            pass

    # deint_done(N) is the first deint_done at/after symbol_start(N) (the FSM
    # advances on deint_done, so symbol_start(N+1) always follows it).
    windows = []
    for scyc in symbol_starts:
        for dcyc in deint_dones:
            if dcyc >= scyc:
                windows.append(dcyc - scyc)
                break

    data_windows = windows[1:]  # skip SIGNAL→DATA1
    if not data_windows:
        return None, [], decoded
    return sum(data_windows) / len(data_windows), data_windows, decoded


async def run_frontend_decode(dut, iq_samples, timeout_cycles=3000000):
    """Feed IQ samples at hardware rate (1-per-5 clocks) and wait for decode.

    Unlike test_rx_pipeline's run_decode(), this does NOT pulse trigger.
    STF detection fires naturally from the waveform content.
    After all frame samples are fed, continues feeding zeros to model
    continuous ADC operation (prevents premature capture exit).

    Uses 1-per-5 clock valid spacing to match real hardware timing
    (100 MHz fabric / 20 MSPS ADC = 5 clocks per sample).

    Returns dict with decode results and front-end observations.
    """
    n_samples = len(iq_samples)
    # Post-frame zeros: enough to fill the 8K buffer + flush pipeline
    post_zero_count = 8192
    total_feed = n_samples + post_zero_count
    result = {
        'tag_valid': False,
        'tag_rate': 0,
        'tag_length': 0,
        'tag_fcs_ok': 0,
        'signal_valid': False,
        'parsed_rate': 0,
        'parsed_length': 0,
        'frame_detect_seen': False,
        'frame_detect_cycle': 0,
        'stf_end_seen': False,
        'stf_end_cycle': 0,
        'cfo_done_seen': False,
        'phase_inc': 0,
        'total_cycles': 0,
        'psdu_bytes': [],
        'psdu_frame_done_seen': False,
        'psdu_frame_done_cycle': 0,
        'psdu_byte_count_final': 0,
    }

    sample_idx = 0
    valid_counter = 0

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
                # Post-frame: feed zeros (model continuous ADC)
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = 0
                dut.iq_q_in.value = 0
                sample_idx += 1
            else:
                dut.iq_valid_in.value = 0
        else:
            dut.iq_valid_in.value = 0

        valid_counter = (valid_counter + 1) % 5

        # Observe front-end events
        try:
            if int(dut.frame_detect.value) == 1 and not result['frame_detect_seen']:
                result['frame_detect_seen'] = True
                result['frame_detect_cycle'] = cycle
                dut._log.info(f"  frame_detect @ cycle {cycle} (sample ~{sample_idx})")
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.stf_end.value) == 1 and not result['stf_end_seen']:
                result['stf_end_seen'] = True
                result['stf_end_cycle'] = cycle
                dut._log.info(f"  stf_end @ cycle {cycle} (sample ~{sample_idx})")
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.cfo_done.value) == 1 and not result['cfo_done_seen']:
                result['cfo_done_seen'] = True
                pi = int(dut.phase_inc.value)
                # Sign-extend 16-bit
                if pi >= 32768:
                    pi -= 65536
                result['phase_inc'] = pi
                dut._log.info(f"  cfo_done: phase_inc={pi}")
        except (ValueError, AttributeError):
            pass

        # Check for signal_valid
        try:
            if int(dut.signal_valid.value) == 1 and not result['signal_valid']:
                result['signal_valid'] = True
                result['parsed_rate'] = int(dut.parsed_rate.value)
                result['parsed_length'] = int(dut.parsed_length.value)
                dut._log.info(f"  SIGNAL parsed: rate=0b{result['parsed_rate']:04b}, "
                              f"length={result['parsed_length']}")
        except (ValueError, AttributeError):
            pass

        # Check for tag output
        try:
            if int(dut.tag_valid.value) == 1:
                result['tag_valid'] = True
                result['tag_rate'] = int(dut.tag_rate.value)
                result['tag_length'] = int(dut.tag_length.value)
                result['tag_fcs_ok'] = int(dut.tag_fcs_ok.value)
                result['total_cycles'] = cycle
                dut._log.info(f"  TAG: rate=0b{result['tag_rate']:04b}, "
                              f"length={result['tag_length']}, fcs_ok={result['tag_fcs_ok']}")
                break
        except (ValueError, AttributeError):
            pass

        # Collect PSDU bytes (psdu_packer byte stream, deimos_rx_poll path)
        try:
            if int(dut.psdu_byte_valid.value) == 1:
                result['psdu_bytes'].append(int(dut.psdu_byte_out.value))
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.psdu_frame_done.value) == 1:
                result['psdu_frame_done_seen'] = True
                result['psdu_frame_done_cycle'] = cycle
                result['psdu_byte_count_final'] = int(dut.psdu_byte_count.value)
        except (ValueError, AttributeError):
            pass

        # Check for early abort
        try:
            if int(dut.seq_done.value) == 1 and not result['tag_valid']:
                result['total_cycles'] = cycle
                dut._log.warning(f"  seq_done without tag_valid at cycle {cycle}")
                break
        except (ValueError, AttributeError):
            pass

        # Progress logging
        if cycle > 0 and cycle % 500000 == 0:
            try:
                st = int(dut.state.value)
                dut._log.info(f"  cycle {cycle}: state={st}, sample={sample_idx}/{n_samples}")
            except (ValueError, AttributeError):
                pass

    return result


async def run_frontend_decode_realistic(dut, iq_samples, timeout_cycles=10000000):
    """Feed IQ samples at 1-per-5 clocks (models 20 MSPS / 100 MHz hardware).

    This is critical for validating stf_end_skip calibration. With valid every
    clock, the mixer has 14 SAMPLES of delay. With valid every 5 clocks, the
    mixer has 14 CLOCKS = ~2.8 SAMPLES of delay. This changes when
    stf_end_pending is consumed relative to the mixer output stream.
    """
    n_samples = len(iq_samples)
    result = {
        'tag_valid': False,
        'tag_rate': 0,
        'tag_length': 0,
        'tag_fcs_ok': 0,
        'signal_valid': False,
        'parsed_rate': 0,
        'parsed_length': 0,
        'frame_detect_seen': False,
        'frame_detect_cycle': 0,
        'stf_end_seen': False,
        'stf_end_cycle': 0,
        'cfo_done_seen': False,
        'phase_inc': 0,
        'total_cycles': 0,
        'psdu_bytes': [],
        'psdu_frame_done_seen': False,
        'psdu_frame_done_cycle': 0,
        'psdu_byte_count_final': 0,
    }

    sample_idx = 0
    valid_counter = 0  # counts 0..4, valid on 0

    for cycle in range(timeout_cycles):
        await RisingEdge(dut.clk)

        # Feed IQ at 1 valid per 5 clocks (realistic ADC rate)
        if sample_idx < n_samples and valid_counter == 0:
            re_q, im_q = iq_samples[sample_idx]
            dut.iq_valid_in.value = 1
            dut.iq_i_in.value = s12_to_unsigned(re_q)
            dut.iq_q_in.value = s12_to_unsigned(im_q)
            sample_idx += 1
        else:
            dut.iq_valid_in.value = 0

        valid_counter = (valid_counter + 1) % 5

        # Observe front-end events
        try:
            if int(dut.frame_detect.value) == 1 and not result['frame_detect_seen']:
                result['frame_detect_seen'] = True
                result['frame_detect_cycle'] = cycle
                dut._log.info(f"  frame_detect @ cycle {cycle} (sample ~{sample_idx})")
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.stf_end.value) == 1 and not result['stf_end_seen']:
                result['stf_end_seen'] = True
                result['stf_end_cycle'] = cycle
                dut._log.info(f"  stf_end @ cycle {cycle} (sample ~{sample_idx})")
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.cfo_done.value) == 1 and not result['cfo_done_seen']:
                result['cfo_done_seen'] = True
                pi = int(dut.phase_inc.value)
                if pi >= 32768:
                    pi -= 65536
                result['phase_inc'] = pi
                dut._log.info(f"  cfo_done: phase_inc={pi}")
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.signal_valid.value) == 1 and not result['signal_valid']:
                result['signal_valid'] = True
                result['parsed_rate'] = int(dut.parsed_rate.value)
                result['parsed_length'] = int(dut.parsed_length.value)
                dut._log.info(f"  SIGNAL parsed: rate=0b{result['parsed_rate']:04b}, "
                              f"length={result['parsed_length']}")
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.tag_valid.value) == 1:
                result['tag_valid'] = True
                result['tag_rate'] = int(dut.tag_rate.value)
                result['tag_length'] = int(dut.tag_length.value)
                result['tag_fcs_ok'] = int(dut.tag_fcs_ok.value)
                result['total_cycles'] = cycle
                dut._log.info(f"  TAG: rate=0b{result['tag_rate']:04b}, "
                              f"length={result['tag_length']}, fcs_ok={result['tag_fcs_ok']}")
                break
        except (ValueError, AttributeError):
            pass

        # Collect PSDU bytes (psdu_packer byte stream, deimos_rx_poll path)
        try:
            if int(dut.psdu_byte_valid.value) == 1:
                result['psdu_bytes'].append(int(dut.psdu_byte_out.value))
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.psdu_frame_done.value) == 1:
                result['psdu_frame_done_seen'] = True
                result['psdu_frame_done_cycle'] = cycle
                result['psdu_byte_count_final'] = int(dut.psdu_byte_count.value)
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.seq_done.value) == 1 and not result['tag_valid']:
                result['total_cycles'] = cycle
                dut._log.warning(f"  seq_done without tag_valid at cycle {cycle}")
                break
        except (ValueError, AttributeError):
            pass

    return result


async def run_frontend_decode_live(dut, iq_samples, pre_noise_samples=500,
                                   snr_db=35, timeout_cycles=10000000):
    """Feed pre-frame noise then IQ samples at 1-per-5 clocks (live hardware model).

    Models live cable-loopback conditions where:
      - ADC runs continuously at 20 MSPS (1 valid per 5 sys_clk cycles)
      - Delay lines are full of noise/prior-frame data before the frame arrives
      - There is no reset between frames

    After all frame samples are fed, continues feeding zeros (ADC never stops).

    Args:
        dut: cocotb DUT handle
        iq_samples: list of (re_q, im_q) integer tuples (already quantized 12-bit)
        pre_noise_samples: number of Gaussian noise samples before the frame
        snr_db: SNR for noise amplitude relative to signal RMS
        timeout_cycles: maximum simulation cycles

    Returns:
        Same result dict as run_frontend_decode_realistic()
    """
    # Compute signal RMS from quantized samples
    sig_rms = np.sqrt(np.mean([r**2 + i**2 for r, i in iq_samples]))
    # Noise std for given SNR
    noise_std = sig_rms / (10**(snr_db / 20))
    # Generate integer noise samples with fixed seed for reproducibility
    rng = np.random.default_rng(seed=9999)
    noise_i = np.clip(np.round(rng.normal(0, noise_std, pre_noise_samples)),
                      -2048, 2047).astype(int)
    noise_q = np.clip(np.round(rng.normal(0, noise_std, pre_noise_samples)),
                      -2048, 2047).astype(int)
    noise_samples = list(zip(noise_i.tolist(), noise_q.tolist()))

    # Build full sample stream: noise + frame
    all_samples = noise_samples + list(iq_samples)
    n_samples = len(all_samples)

    # Post-frame: continue feeding zeros for at least enough cycles to finish decode
    # We handle this in the loop below by feeding (0,0) after all_samples are exhausted
    post_zero_count = 2000  # enough zeros to flush pipeline

    result = {
        'tag_valid': False,
        'tag_rate': 0,
        'tag_length': 0,
        'tag_fcs_ok': 0,
        'signal_valid': False,
        'parsed_rate': 0,
        'parsed_length': 0,
        'frame_detect_seen': False,
        'frame_detect_cycle': 0,
        'stf_end_seen': False,
        'stf_end_cycle': 0,
        'cfo_done_seen': False,
        'phase_inc': 0,
        'total_cycles': 0,
        'psdu_bytes': [],
        'psdu_frame_done_seen': False,
        'psdu_frame_done_cycle': 0,
        'psdu_byte_count_final': 0,
    }

    sample_idx = 0
    valid_counter = 0

    for cycle in range(timeout_cycles):
        await RisingEdge(dut.clk)

        # Feed at 1-per-5 clocks
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

        # Observe front-end events
        try:
            if int(dut.frame_detect.value) == 1 and not result['frame_detect_seen']:
                result['frame_detect_seen'] = True
                result['frame_detect_cycle'] = cycle
                dut._log.info(f"  [live] frame_detect @ cycle {cycle} (sample ~{sample_idx})")
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.stf_end.value) == 1 and not result['stf_end_seen']:
                result['stf_end_seen'] = True
                result['stf_end_cycle'] = cycle
                dut._log.info(f"  [live] stf_end @ cycle {cycle} (sample ~{sample_idx})")
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.cfo_done.value) == 1 and not result['cfo_done_seen']:
                result['cfo_done_seen'] = True
                pi = int(dut.phase_inc.value)
                if pi >= 32768:
                    pi -= 65536
                result['phase_inc'] = pi
                dut._log.info(f"  [live] cfo_done: phase_inc={pi}")
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.signal_valid.value) == 1 and not result['signal_valid']:
                result['signal_valid'] = True
                result['parsed_rate'] = int(dut.parsed_rate.value)
                result['parsed_length'] = int(dut.parsed_length.value)
                dut._log.info(f"  [live] SIGNAL parsed: rate=0b{result['parsed_rate']:04b}, "
                              f"length={result['parsed_length']}")
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.tag_valid.value) == 1:
                result['tag_valid'] = True
                result['tag_rate'] = int(dut.tag_rate.value)
                result['tag_length'] = int(dut.tag_length.value)
                result['tag_fcs_ok'] = int(dut.tag_fcs_ok.value)
                result['total_cycles'] = cycle
                dut._log.info(f"  [live] TAG: rate=0b{result['tag_rate']:04b}, "
                              f"length={result['tag_length']}, fcs_ok={result['tag_fcs_ok']}")
                break
        except (ValueError, AttributeError):
            pass

        # Collect PSDU bytes (psdu_packer byte stream, deimos_rx_poll path)
        try:
            if int(dut.psdu_byte_valid.value) == 1:
                result['psdu_bytes'].append(int(dut.psdu_byte_out.value))
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.psdu_frame_done.value) == 1:
                result['psdu_frame_done_seen'] = True
                result['psdu_frame_done_cycle'] = cycle
                result['psdu_byte_count_final'] = int(dut.psdu_byte_count.value)
        except (ValueError, AttributeError):
            pass

        try:
            # Only check seq_done after frame_detect has fired (avoids
            # catching stale seq_done from a previous frame in back-to-back)
            if (int(dut.seq_done.value) == 1 and not result['tag_valid']
                    and result['frame_detect_seen']):
                result['total_cycles'] = cycle
                dut._log.warning(f"  [live] seq_done without tag_valid at cycle {cycle}")
                break
        except (ValueError, AttributeError):
            pass

    return result


def load_adc_capture(filepath):
    """Load ADC capture JSON and return quantized 12-bit IQ samples.

    The capture file contains normalized floats (raw_12bit / 2047).
    We rescale to fill 80% of 12-bit dynamic range (matching golden vector
    convention) to ensure consistent quantization behavior in sim.
    """
    with open(filepath) as f:
        data = json.load(f)
    re = np.array(data['real'], dtype=np.float64)
    im = np.array(data['imag'], dtype=np.float64)
    iq = re + 1j * im
    return quantize_12bit(iq)
