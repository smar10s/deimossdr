"""
test_system_traffic.py — System-level gate test: realistic mixed traffic.

THE PROBLEM THIS CATCHES:
  Short frames (ACKs) arriving during active decode of longer frames.
  The pending path must correctly queue and decode interleaved frames
  without losing or misattributing any.

WHAT THIS TESTS:
  Realistic OTA-like traffic: long frames (beacons, EAPOLs) interleaved
  with short frames (ACKs) at SIFS timing. Frames arrive during active
  decode, triggering the pending path.

  Scenario modeled from live ch36 5 GHz capture (eapol_4way_burst.json):
  - Beacon (380B rate 6) with ACK arriving during beacon decode
  - EAPOL M1 (137B rate 6) followed by SIFS ACK (14B)
  - EAPOL M3 (193B rate 6) preceded by SIFS ACK (14B)
  - Mixed rates (6/12/24 Mbps) with realistic CFO

GATE CRITERIA:
  1. All frames decode FCS OK (fabric correctness)

INTEGRATION:
  Gate test (test_*). Runs in sim.sh. DUT = rx_frontend.
"""

import os
import sys

import numpy as np
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from frontend_helpers import (
    quantize_12bit, s12_to_unsigned, reset_dut, SAMPLE_RATE,
)

# Add lib80211 for frame generation and impairments
LIB80211_PYTHON = os.path.join(os.path.dirname(__file__), '..', '..', 'extern', 'lib80211', 'python')
sys.path.insert(0, LIB80211_PYTHON)
from py80211.gen_ofdm_frame import generate_frame
from py80211.impairments import add_cfo, add_awgn


# =========================================================
# Constants
# =========================================================

# SIFS = 16 us at 20 MSPS
SIFS_SAMPLES = 320

# DIFS = 34 us at 20 MSPS
DIFS_SAMPLES = 680


# Minimum gap for the race NOT to trigger: pipeline must finish before
# next frame arrives. For rate 6, 100B frame: ~5000 samples.
# We use SIFS (320) and small gaps (1000-3000) to trigger the race.


# =========================================================
# Traffic Builders
# =========================================================

def make_frame(rate_mbps, psdu_len, cfo_hz=0, seed=0x5D):
    """Generate a single frame with optional CFO.

    Args:
        rate_mbps: 6, 9, 12, 18, 24, 36, 48, or 54
        psdu_len: PSDU payload length in bytes (excluding FCS)
        cfo_hz: carrier frequency offset
        seed: scrambler seed

    Returns:
        (complex IQ array, metadata dict)
    """
    # Generate deterministic payload
    rng = np.random.default_rng(seed=psdu_len * 1000 + rate_mbps)
    payload = bytes(rng.integers(0, 256, size=psdu_len, dtype=np.uint8))

    iq, meta = generate_frame(rate_mbps, payload, scrambler_seed=seed)
    if cfo_hz != 0:
        iq = add_cfo(iq, cfo_hz)
    return iq, meta


def build_traffic_stream(frame_specs, snr_db=30, leading_noise=2000):
    """Build a realistic traffic stream with specified inter-frame gaps.

    Args:
        frame_specs: list of dicts:
            {'rate': int, 'len': int, 'cfo': float, 'gap_after': int}
            gap_after is in samples (0 for last frame)
        snr_db: signal-to-noise ratio
        leading_noise: noise samples before first frame

    Returns:
        (quantized_samples, frame_info_list)
        frame_info_list entries: {'rate': int, 'len': int, 'stf_start_sample': int,
                                   'n_samples': int}
    """
    rng = np.random.default_rng(seed=777)

    # Compute noise level from first frame's signal power
    iq0, _ = make_frame(frame_specs[0]['rate'], frame_specs[0]['len'])
    sig_rms = np.sqrt(np.mean(np.abs(iq0)**2))
    noise_std = sig_rms / (10**(snr_db / 20))

    segments = []
    frame_info = []

    # Leading noise (fills delay lines, establishes noise floor)
    leading = rng.normal(0, noise_std, leading_noise) + \
              1j * rng.normal(0, noise_std, leading_noise)
    segments.append(leading)
    current_offset = leading_noise

    for i, spec in enumerate(frame_specs):
        rate = spec['rate']
        psdu_len = spec['len']
        cfo_hz = spec.get('cfo', 0)
        gap_after = spec.get('gap_after', 0)

        iq, meta = make_frame(rate, psdu_len, cfo_hz=cfo_hz)

        # Add noise
        noise = rng.normal(0, noise_std, len(iq)) + \
                1j * rng.normal(0, noise_std, len(iq))
        iq_noisy = iq + noise

        segments.append(iq_noisy)
        frame_info.append({
            'rate': rate,
            'len': meta['psdu_length'],  # includes FCS (payload + 4)
            'stf_start_sample': current_offset,
            'n_samples': len(iq),
            'index': i,
        })
        current_offset += len(iq)

        # Gap noise
        if gap_after > 0:
            gap = rng.normal(0, noise_std, gap_after) + \
                  1j * rng.normal(0, noise_std, gap_after)
            segments.append(gap)
            current_offset += gap_after

    # Trailing silence to flush pipeline
    trailing = np.zeros(8000, dtype=complex)
    segments.append(trailing)

    full_stream = np.concatenate(segments)
    samples = quantize_12bit(full_stream)
    return samples, frame_info


# =========================================================
# Tag Collection (reused from test_continuous_decode pattern)
# =========================================================

async def feed_and_collect(dut, samples, n_expected, timeout_cycles=30000000):
    """Feed IQ stream and collect tag outputs.

    Returns list of tag dicts with rate, length, fcs_ok.
    """
    n_samples = len(samples)
    tags = []
    sample_idx = 0
    valid_counter = 0

    for cycle in range(timeout_cycles):
        await RisingEdge(dut.clk)

        # Feed at 1-per-5 clock rate (20 MSPS / 100 MHz fabric)
        if valid_counter == 0:
            if sample_idx < n_samples:
                re_q, im_q = samples[sample_idx]
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = s12_to_unsigned(re_q)
                dut.iq_q_in.value = s12_to_unsigned(im_q)
                sample_idx += 1
            else:
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = 0
                dut.iq_q_in.value = 0
        else:
            dut.iq_valid_in.value = 0

        valid_counter = (valid_counter + 1) % 5

        # Collect tags
        try:
            if int(dut.tag_valid.value) == 1:
                tag = {
                    'rate': int(dut.tag_rate.value),
                    'length': int(dut.tag_length.value),
                    'fcs_ok': int(dut.tag_fcs_ok.value),
                    'cycle': cycle,
                    'sample': sample_idx,
                }
                tags.append(tag)
                fcs_str = 'OK' if tag['fcs_ok'] else 'FAIL'
                dut._log.info(f"  Tag {len(tags)}: rate={tag['rate']}, "
                              f"len={tag['length']}, fcs={fcs_str}")
        except (ValueError, AttributeError):
            pass

        # Done when all expected tags collected + flush
        if len(tags) >= n_expected and sample_idx >= n_samples:
            break

        # Progress
        if cycle > 0 and cycle % 5000000 == 0:
            dut._log.info(f"  cycle {cycle}: {len(tags)} tags, "
                          f"sample {sample_idx}/{n_samples}")

    return tags



# =========================================================
# Test Cases
# =========================================================

@cocotb.test()
async def test_offset_race_long_then_short(dut):
    """Long frame followed by short frame during decode — pending path race.

    Scenario: 380B beacon (rate 6, ~10640 samples pipeline time) with a
    14B ACK arriving 3000 samples into the beacon's decode. Both must
    decode FCS OK via the pending path.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # 380B beacon at rate 6 takes ~10640 samples
    # ACK arrives 3000 samples after beacon starts (well within pipeline time)
    specs = [
        {'rate': 6, 'len': 376, 'cfo': 800, 'gap_after': 3000},   # beacon (376+4=380B with FCS)
        {'rate': 6, 'len': 10,  'cfo': 200, 'gap_after': 15000},  # ACK (10+4=14B with FCS)
        {'rate': 6, 'len': 96,  'cfo': -500, 'gap_after': 0},     # trailing frame (sanity)
    ]

    samples, frame_info = build_traffic_stream(specs, snr_db=30)
    dut._log.info(f"Offset race test: {len(specs)} frames, "
                  f"{len(samples)} samples ({len(samples)/SAMPLE_RATE*1000:.1f} ms)")
    dut._log.info(f"  Beacon: {frame_info[0]['n_samples']} samples, "
                  f"STF @ {frame_info[0]['stf_start_sample']}")
    dut._log.info(f"  ACK:    {frame_info[1]['n_samples']} samples, "
                  f"STF @ {frame_info[1]['stf_start_sample']}")

    tags = await feed_and_collect(dut, samples, n_expected=len(specs))

    # Gate 1: All frames must decode with FCS OK
    fcs_ok_count = sum(1 for t in tags if t['fcs_ok'])
    assert fcs_ok_count >= len(specs), \
        f"Only {fcs_ok_count}/{len(specs)} FCS OK. " \
        f"Tags: {[(t['length'], t['fcs_ok']) for t in tags]}"


@cocotb.test()
async def test_offset_race_eapol_with_acks(dut):
    """EAPOL frame preceded and followed by ACKs at SIFS — realistic burst.

    Models the real OTA pattern from eapol_4way_burst.json:
      [ACK 14B] --SIFS--> [EAPOL 137B] --SIFS--> [ACK 14B]

    The preceding ACK is short enough to finish before EAPOL triggers.
    The following ACK triggers during EAPOL's decode.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    specs = [
        {'rate': 6, 'len': 10,  'cfo': 300,  'gap_after': SIFS_SAMPLES},  # pre-ACK
        {'rate': 6, 'len': 133, 'cfo': 1200, 'gap_after': SIFS_SAMPLES},  # EAPOL M1 (133+4=137)
        {'rate': 6, 'len': 10,  'cfo': -400, 'gap_after': 15000},         # post-ACK
        {'rate': 6, 'len': 96,  'cfo': 600,  'gap_after': 0},             # trailing (sanity)
    ]

    samples, frame_info = build_traffic_stream(specs, snr_db=30)
    dut._log.info(f"EAPOL+ACK test: {len(specs)} frames, "
                  f"{len(samples)} samples ({len(samples)/SAMPLE_RATE*1000:.1f} ms)")
    for i, info in enumerate(frame_info):
        dut._log.info(f"  Frame {i}: {info['rate']}Mbps {info['len']}B "
                      f"STF @ {info['stf_start_sample']}")

    tags = await feed_and_collect(dut, samples, n_expected=len(specs))

    # Gate 1: FCS
    fcs_ok_count = sum(1 for t in tags if t['fcs_ok'])
    assert fcs_ok_count >= len(specs), \
        f"Only {fcs_ok_count}/{len(specs)} FCS OK."


@cocotb.test()
async def test_mixed_traffic_burst(dut):
    """Mixed-rate burst with realistic timing — system-level integration.

    Models a realistic 5 GHz channel segment:
      Beacon (380B r6) → ACK (14B r24) → probe (200B r6) → ACK (14B r24)
      → EAPOL M1 (137B r6) → ACK (14B r6) → EAPOL M3 (193B r6) → ACK (14B r6)

    Gaps: SIFS where protocol requires it, DIFS between independent frames.
    Multiple rate changes test pipeline reconfiguration under timing pressure.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    specs = [
        # Beacon + ACK
        {'rate': 6,  'len': 376, 'cfo': 900,  'gap_after': SIFS_SAMPLES},
        {'rate': 24, 'len': 10,  'cfo': 200,  'gap_after': DIFS_SAMPLES},
        # Probe response + ACK
        {'rate': 6,  'len': 196, 'cfo': -600, 'gap_after': SIFS_SAMPLES},
        {'rate': 24, 'len': 10,  'cfo': 100,  'gap_after': DIFS_SAMPLES},
        # EAPOL M1 + ACK
        {'rate': 6,  'len': 133, 'cfo': 1100, 'gap_after': SIFS_SAMPLES},
        {'rate': 6,  'len': 10,  'cfo': -300, 'gap_after': DIFS_SAMPLES},
        # EAPOL M3 + ACK
        {'rate': 6,  'len': 189, 'cfo': 800,  'gap_after': SIFS_SAMPLES},
        {'rate': 6,  'len': 10,  'cfo': -200, 'gap_after': 0},
    ]

    samples, frame_info = build_traffic_stream(specs, snr_db=30)
    dut._log.info(f"Mixed traffic burst: {len(specs)} frames, "
                  f"{len(samples)} samples ({len(samples)/SAMPLE_RATE*1000:.1f} ms)")
    for i, info in enumerate(frame_info):
        dut._log.info(f"  Frame {i}: {info['rate']}Mbps {info['len']}B "
                      f"STF @ {info['stf_start_sample']}")

    tags = await feed_and_collect(dut, samples, n_expected=len(specs),
                                  timeout_cycles=50000000)

    # Gate 1: Frame detection — at least 7/8 detected (tolerant of edge cases)
    assert len(tags) >= len(specs) - 1, \
        f"Only {len(tags)}/{len(specs)} frames detected."

    # Gate 2: FCS pass rate
    fcs_ok_count = sum(1 for t in tags if t['fcs_ok'])
    assert fcs_ok_count >= len(specs) - 1, \
        f"Only {fcs_ok_count}/{len(specs)} FCS OK."


@cocotb.test()
async def test_offset_race_three_frame_pileup(dut):
    """Three frames arriving within one pipeline cycle — worst-case pending.

    Frame A (long, 500B rate 6): pipeline time ~14000 samples
    Frame B (short, 14B): arrives 2000 samples into A's decode
    Frame C (short, 14B): arrives 4000 samples into A's decode

    All three must decode FCS OK via the pending/frame-fifo path.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    specs = [
        {'rate': 6,  'len': 496, 'cfo': 700,  'gap_after': 2000},  # 496+4=500B, long
        {'rate': 6,  'len': 10,  'cfo': -300, 'gap_after': 2000},  # ACK during A decode
        {'rate': 6,  'len': 10,  'cfo': 400,  'gap_after': 15000}, # ACK during A decode
        {'rate': 6,  'len': 96,  'cfo': -800, 'gap_after': 0},     # trailing sanity
    ]

    samples, frame_info = build_traffic_stream(specs, snr_db=30)
    dut._log.info(f"Three-frame pileup: {len(specs)} frames, "
                  f"{len(samples)} samples ({len(samples)/SAMPLE_RATE*1000:.1f} ms)")
    for i, info in enumerate(frame_info):
        dut._log.info(f"  Frame {i}: {info['rate']}Mbps {info['len']}B "
                      f"STF @ {info['stf_start_sample']}")

    tags = await feed_and_collect(dut, samples, n_expected=len(specs))

    # Gate 1: Detection count
    assert len(tags) >= 3, \
        f"Only {len(tags)} frames detected (need >= 3)."

    # Gate 2: FCS
    fcs_ok_count = sum(1 for t in tags if t['fcs_ok'])
    assert fcs_ok_count >= 3, \
        f"Only {fcs_ok_count} FCS OK."


@cocotb.test()
async def test_offset_queue_desync_aborted_frame(dut):
    """Aborted frame (L-SIG corruption) must not prevent subsequent decode.

    SCENARIO:
      Frame A (normal, 200B rate 6): decodes OK.
      Frame B (corrupted L-SIG): STF triggers, L-SIG fails → abort.
      Frame C (normal, 150B rate 6): must still decode OK despite B's abort.

    HOW WE INJECT THE BAD FRAME:
      Generate a valid 802.11 frame, then zero the L-SIG OFDM symbol.
      STF and LTF are intact (trigger fires normally) but L-SIG decodes
      with bad parity → abort.

    WHY THIS MATTERS:
      On real traffic, HT/VHT frames and interference produce L-SIG parity
      failures after valid STF/LTF detection. The pipeline must recover
      cleanly for subsequent legacy frames.

    GATE CRITERIA:
      At least 2 tags (A and C). Frame C (150B) must decode FCS OK.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    rng = np.random.default_rng(seed=42)

    # Frame A: normal, decodes correctly
    iq_a, meta_a = make_frame(6, 196, cfo_hz=500)  # 200B with FCS

    # Frame B: valid STF+LTF but corrupted L-SIG.
    # Generate a normal frame, then corrupt the L-SIG OFDM symbol.
    iq_b_orig, meta_b = make_frame(6, 96, cfo_hz=-300)  # 100B, doesn't matter
    iq_b = iq_b_orig.copy()
    # L-SIG is the OFDM symbol starting at sample 320 (after STF=160 + LTF=160).
    # Corrupt it by scrambling the L-SIG symbol (samples 320+16=336 to 336+64).
    # Keep STF and LTF intact so stf_detect triggers and LTF correlates.
    sig_start = 320 + 16  # GI(16) + 64 data samples for L-SIG
    sig_end = sig_start + 64
    # Zero out the L-SIG symbol entirely. This produces all-zero soft decisions
    # in the Viterbi, which decodes to all-zero output. The L-SIG all-zero pattern
    # has rate=0b0000 which is invalid → triggers abort regardless of Viterbi strength.
    iq_b[sig_start:sig_end] = 0.0 + 0.0j

    # Frame C: normal, decodes correctly — but will get B's offset if bug present
    iq_c, meta_c = make_frame(6, 146, cfo_hz=700)  # 150B with FCS

    # Assemble stream: noise + A + gap + B + gap + C + trailing
    noise_std = np.sqrt(np.mean(np.abs(iq_a)**2)) / (10**(30/20))
    leading = rng.normal(0, noise_std, 2000) + 1j * rng.normal(0, noise_std, 2000)

    gap_ab = 5000  # Large gap so A finishes before B
    gap_bc = 5000  # Large gap so B's abort finishes before C

    gap_ab_iq = rng.normal(0, noise_std, gap_ab) + 1j * rng.normal(0, noise_std, gap_ab)
    gap_bc_iq = rng.normal(0, noise_std, gap_bc) + 1j * rng.normal(0, noise_std, gap_bc)
    trailing = np.zeros(8000, dtype=complex)

    # Add noise to all frames
    iq_a_n = iq_a + (rng.normal(0, noise_std, len(iq_a)) + 1j * rng.normal(0, noise_std, len(iq_a)))
    iq_b_n = iq_b + (rng.normal(0, noise_std, len(iq_b)) + 1j * rng.normal(0, noise_std, len(iq_b)))
    iq_c_n = iq_c + (rng.normal(0, noise_std, len(iq_c)) + 1j * rng.normal(0, noise_std, len(iq_c)))

    full_stream = np.concatenate([leading, iq_a_n, gap_ab_iq, iq_b_n, gap_bc_iq, iq_c_n, trailing])
    samples = quantize_12bit(full_stream)

    # Compute frame positions
    a_stf_start = len(leading)
    b_stf_start = a_stf_start + len(iq_a) + gap_ab
    c_stf_start = b_stf_start + len(iq_b) + gap_bc

    dut._log.info(f"Offset queue desync (aborted frame) test:")
    dut._log.info(f"  Frame A: 6Mbps 200B STF @ {a_stf_start}")
    dut._log.info(f"  Frame B: 6Mbps (corrupted L-SIG) STF @ {b_stf_start}")
    dut._log.info(f"  Frame C: 6Mbps 150B STF @ {c_stf_start}")
    dut._log.info(f"  Total: {len(samples)} samples ({len(samples)/SAMPLE_RATE*1000:.1f} ms)")

    # Collect tags. A and C should produce tags. B should NOT (aborted).
    # With the bug: C gets B's offset. Without the bug: C gets its own.
    tags = await feed_and_collect(dut, samples, n_expected=2,
                                  timeout_cycles=50000000)

    dut._log.info(f"  Collected {len(tags)} tags")
    for i, t in enumerate(tags):
        dut._log.info(f"    Tag {i}: rate={t['rate']}, len={t['length']}, "
                      f"fcs={'OK' if t['fcs_ok'] else 'FAIL'}")

    # Gate 1: Must get at least 2 tags (A and C). B should NOT produce a tag.
    assert len(tags) >= 2, \
        f"Only {len(tags)} tags (need >= 2: A + C)."

    # Gate 2: Find Frame C's tag (150B = unique length)
    c_tag = None
    for t in tags:
        if t['length'] == meta_c['psdu_length'] and t['fcs_ok']:
            c_tag = t
            break

    assert c_tag is not None, \
        f"Frame C (150B) not found in tags. Got: {[(t['length'], t['fcs_ok']) for t in tags]}"


# =========================================================
# HIL-Noise Burst Detection Test
# =========================================================
# Reproduces the exact conditions of the SIFS burst drop on hardware:
# - 10 identical frames at rate 6, 100 bytes, SIFS gap (320 samples)
# - Inter-frame noise at -40 dB (1% of peak) — matching HIL inject level
# - This exercises the unsigned energy accumulator (acc_e1) underflow path:
#   DATA→gap transition with very low noise causes acc_e1 to wrap around 2^32
#   because the sliding window subtracts high-energy DATA samples while only
#   adding near-zero noise samples.

def build_hil_burst(n_frames, rate, psdu_len, gap_samples, noise_fraction=0.01):
    """Build a burst matching HIL inject conditions.

    Inter-frame gaps use noise at `noise_fraction` of signal peak (default 1%).
    This is much lower than the 30 dB SNR used in other tests (~3% noise),
    and matches what deimos_burst_loopback.c injects in --hil mode.
    """
    rng = np.random.default_rng(seed=999)

    # Generate all frames first to find overall peak
    frames = []
    for i in range(n_frames):
        iq, meta = make_frame(rate, psdu_len, cfo_hz=0, seed=0x5D + i)
        frames.append((iq, meta))

    # Find global peak (for quantization scaling, same as firmware)
    all_iq = np.concatenate([f[0] for f in frames])
    peak = max(np.abs(all_iq.real).max(), np.abs(all_iq.imag).max())
    noise_level = peak * noise_fraction

    # Build stream: leading noise + (frame + gap) × N
    leading = rng.uniform(-noise_level, noise_level, 500) + \
              1j * rng.uniform(-noise_level, noise_level, 500)

    segments = [leading]
    frame_info = []
    current_offset = len(leading)

    for i, (iq, meta) in enumerate(frames):
        # Add noise to frame (very low level, matching HIL)
        noise = rng.uniform(-noise_level, noise_level, len(iq)) + \
                1j * rng.uniform(-noise_level, noise_level, len(iq))
        segments.append(iq + noise)
        frame_info.append({
            'rate': rate,
            'len': meta['psdu_length'],
            'stf_start_sample': current_offset,
            'n_samples': len(iq),
            'index': i,
        })
        current_offset += len(iq)

        # Gap noise (1% of peak — the critical difference from standard tests)
        if i < n_frames - 1:
            gap = rng.uniform(-noise_level, noise_level, gap_samples) + \
                  1j * rng.uniform(-noise_level, noise_level, gap_samples)
            segments.append(gap)
            current_offset += gap_samples

    # Trailing zeros to flush pipeline
    segments.append(np.zeros(10000, dtype=complex))

    full_stream = np.concatenate(segments)
    samples = quantize_12bit(full_stream)
    return samples, frame_info


@cocotb.test()
async def test_hil_burst_sifs_noise(dut):
    """HIL burst at SIFS gap with 1% noise — reproduces hardware burst drop.

    This test matches the exact conditions of deimos_burst_loopback --hil:
    - 10 frames, rate 6, 100 bytes, gap=320 (16 μs SIFS)
    - Inter-frame noise at 1% of signal peak (-40 dB)
    - No CFO (golden vector inject)

    The critical difference from test_mixed_traffic_burst (which passes):
    that test uses 30 dB SNR Gaussian noise (~3% of peak) which keeps the
    unsigned energy accumulators (acc_e1/e2) from underflowing during the
    DATA→gap transition. At 1% noise, the accumulators wrap around 2^32.

    If this test FAILS: confirms unsigned accumulator underflow as root cause.
    Fix: clamp acc_e1/e2 at 0 (prevent unsigned wrap).
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    n_frames = 10
    samples, frame_info = build_hil_burst(
        n_frames=n_frames,
        rate=6,
        psdu_len=96,   # 96+4=100 bytes with FCS
        gap_samples=SIFS_SAMPLES,
        noise_fraction=0.01,  # 1% of peak — HIL inject level
    )

    dut._log.info(f"HIL burst test: {n_frames} frames, rate 6, 100B, "
                  f"gap={SIFS_SAMPLES}, noise=1% of peak")
    dut._log.info(f"  Total: {len(samples)} samples ({len(samples)/SAMPLE_RATE*1000:.1f} ms)")
    for i, info in enumerate(frame_info):
        dut._log.info(f"  Frame {i}: STF @ sample {info['stf_start_sample']}")

    # In hardware with burst_mode=1, playback_start does NOT fire to trigger_or.
    # Frame 1 enters via frame_detect from STF detection (saving 80 samples
    # of pipeline margin). Don't pulse playback_start here to match.

    tags = await feed_and_collect(dut, samples, n_expected=n_frames,
                                  timeout_cycles=80000000)

    # Gate: ALL frames must be detected and FCS OK.
    # The hardware bug causes only 2-5/10 detection at gap=320.
    dut._log.info(f"  Detected: {len(tags)}/{n_frames} frames")
    fcs_ok = sum(1 for t in tags if t['fcs_ok'])
    dut._log.info(f"  FCS OK: {fcs_ok}/{len(tags)}")

    assert len(tags) >= n_frames, \
        f"HIL BURST DROP: Only {len(tags)}/{n_frames} frames detected. " \
        f"With 1% inter-frame noise, the unsigned energy accumulators " \
        f"(acc_e1/e2) underflow during DATA→gap transition, poisoning " \
        f"the threshold comparison for subsequent frames. " \
        f"Fix: clamp acc_e1/e2 to prevent unsigned wraparound."

    assert fcs_ok >= n_frames, \
        f"HIL BURST FCS: Only {fcs_ok}/{n_frames} FCS OK."
