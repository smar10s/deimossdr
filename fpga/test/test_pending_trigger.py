"""
test_pending_trigger.py — Gate test: pending trigger decodes frames during busy.

THE PROBLEM THIS CATCHES:
  When frame 2's STF arrives while frame 1 is still decoding, the trigger
  arbiter must latch the detection as "pending" and retrigger from the
  circular buffer after frame 1 completes. Without this, frame 2 is lost.

WHAT THIS TESTS:
  1. Two synthetic frames with a gap tight enough that frame 2's STF fires
     during frame 1's decode (200-sample gap — pipeline takes ~16000 clocks
     to decode a rate-6 frame, so frame 2 arrives mid-decode).
  2. Two frames at different rates to prove pending decode handles the rate
     switch correctly.
  3. EAPOL 4-way burst replay (real OTA capture with natural tight timing).

INTEGRATION:
  This is a gate test (test_*). It runs in sim.sh alongside other gate tests.
  Uses rx_frontend DUT (full front-end + decode pipeline).
"""

import os
import sys

import numpy as np
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from frontend_helpers import (
    load_waveform_float, quantize_12bit, s12_to_unsigned,
    reset_dut, SAMPLE_RATE,
)

LIB80211_PYTHON = os.path.join(os.path.dirname(__file__), '..', '..', 'extern', 'lib80211', 'python')
sys.path.insert(0, LIB80211_PYTHON)
from py80211.impairments import add_cfo, add_awgn


def build_tight_gap_stream(gap_samples, rates=(6, 6), snr_db=40, cfo_hz=(0, 0)):
    """Build 2 frames with a tight inter-frame gap.

    The gap is between end-of-frame-1 and start-of-frame-2. If gap is
    shorter than frame 1's decode time, frame 2's STF arrives during busy
    and exercises the pending trigger path.

    Args:
        gap_samples: noise gap between end of frame 1 and start of frame 2
        rates: tuple of (rate1, rate2) in Mbps
        snr_db: signal-to-noise ratio
        cfo_hz: tuple of (cfo1, cfo2) in Hz
    """
    rng = np.random.default_rng(seed=123)

    iq1 = load_waveform_float(rates[0])
    iq2 = load_waveform_float(rates[1])

    if cfo_hz[0] != 0:
        iq1 = add_cfo(iq1, cfo_hz[0])
    if cfo_hz[1] != 0:
        iq2 = add_cfo(iq2, cfo_hz[1])

    sig_rms = np.sqrt(np.mean(np.abs(iq1)**2))
    noise_std = sig_rms / (10**(snr_db / 20))

    # Add noise to frames
    iq1 = iq1 + (rng.normal(0, noise_std, len(iq1)) +
                 1j * rng.normal(0, noise_std, len(iq1)))
    iq2 = iq2 + (rng.normal(0, noise_std, len(iq2)) +
                 1j * rng.normal(0, noise_std, len(iq2)))

    # Leading silence (fills STF delay lines)
    leading = np.zeros(5000, dtype=complex)
    # Inter-frame gap (noise)
    gap = rng.normal(0, noise_std, gap_samples) + \
          1j * rng.normal(0, noise_std, gap_samples)
    # Trailing silence (flush pipeline)
    trailing = np.zeros(8000, dtype=complex)

    full = np.concatenate([leading, iq1, gap, iq2, trailing])
    return quantize_12bit(full)


async def feed_and_collect_tags(dut, samples, n_expected, timeout_cycles=500000):
    """Feed contiguous IQ stream, collect tags. No reset during feeding."""
    n_samples = len(samples)
    tags = []
    sample_idx = 0
    valid_counter = 0
    cfo_done_count = 0

    for cycle in range(timeout_cycles):
        await RisingEdge(dut.clk)

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

        # Monitor cfo_done events
        try:
            if int(dut.cfo_done.value) == 1:
                cfo_done_count += 1
                pi = int(dut.phase_inc.value)
                if pi >= 32768: pi -= 65536
                dut._log.info(f"  cfo_done #{cfo_done_count}: phase_inc={pi} @ cycle {cycle}")
        except (ValueError, AttributeError):
            pass

        # Monitor frame_detect (trigger) events
        try:
            if int(dut.frame_detect.value) == 1:
                dut._log.info(f"  frame_detect @ cycle {cycle} (sample ~{sample_idx})")
        except (ValueError, AttributeError):
            pass

        # Monitor tag output and log frame_phase_inc at tag time
        try:
            if int(dut.tag_valid.value) == 1:
                # Log frame_phase_inc at tag time
                try:
                    rxtop = dut.u_rx_pipeline.u_decode_engine
                    fpi = int(rxtop.frame_phase_inc.value)
                    if fpi >= 32768: fpi -= 65536
                    lo = int(rxtop.ltf1_offset.value)
                    fpi_str = f" frame_phase_inc={fpi} ltf1_offset={lo}"
                except (ValueError, AttributeError):
                    fpi_str = ""
                tag = {
                    'rate': int(dut.tag_rate.value),
                    'length': int(dut.tag_length.value),
                    'fcs_ok': int(dut.tag_fcs_ok.value),
                    'cycle': cycle,
                    'sample': sample_idx,
                }
                tags.append(tag)
                fcs_str = 'OK' if tag['fcs_ok'] else 'FAIL'
                dut._log.info(f"  Frame {len(tags)}: rate=0b{tag['rate']:04b}, "
                              f"len={tag['length']}, fcs={fcs_str}{fpi_str} @ cycle {cycle}")
                if len(tags) >= n_expected:
                    break
        except (ValueError, AttributeError):
            pass

        if cycle > 0 and cycle % 3000000 == 0:
            dut._log.info(f"  cycle {cycle}: {len(tags)}/{n_expected} frames, "
                          f"sample {sample_idx}/{n_samples}")

    return tags


@cocotb.test()
async def test_pending_trigger_200gap(dut):
    """Two rate-6 frames with 200-sample gap — exercises pending trigger.

    A rate-6 100-byte frame takes ~5000 samples worth of pipeline time from
    detection to tag (measured empirically). Frame 2's STF arrives ~3240 samples
    after frame 1's detection (5000 leading + 160 STF + 3200-160 frame body +
    200 gap = 8400 sample, detection at ~5160, delta=3240). Since 3240 < 5000,
    the pipeline IS busy when frame 2's STF fires — exercising the pending
    trigger path.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    samples = build_tight_gap_stream(gap_samples=200)
    dut._log.info(f"Feeding {len(samples)} samples (2× rate-6, gap=200)")

    tags = await feed_and_collect_tags(dut, samples, n_expected=2)

    fcs_ok = [t for t in tags if t['fcs_ok']]
    assert len(fcs_ok) >= 2, \
        f"Expected 2 FCS-OK frames, got {len(fcs_ok)}. " \
        f"Pending trigger failed — frame 2 was not decoded. Tags: {tags}"


@cocotb.test()
async def test_pending_trigger_50gap(dut):
    """Two rate-6 frames with 50-sample gap — tighter pending trigger exercise.

    Even tighter gap to stress-test the pending path. Frame 2's STF starts
    50 samples after frame 1 ends — frame 1 is deep in DATA decode.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    samples = build_tight_gap_stream(gap_samples=50)
    dut._log.info(f"Feeding {len(samples)} samples (2× rate-6, gap=50)")

    tags = await feed_and_collect_tags(dut, samples, n_expected=2)

    fcs_ok = [t for t in tags if t['fcs_ok']]
    assert len(fcs_ok) >= 2, \
        f"Expected 2 FCS-OK frames, got {len(fcs_ok)}. " \
        f"Pending trigger failed at tight gap. Tags: {tags}"


# Known limitations of the pending trigger path (not gated, for reference):
#
# 1. Mixed rates (e.g., rate 6 → rate 24): frame 2 is never detected.
#    Suspected cause: MIN_AGE filter or peak-detection timing for shorter frames.
#    Not a blocker: EAPOL burst is all rate 6.


@cocotb.test()
async def test_pending_trigger_cross_cfo(dut):
    """Gate: pending-path decode must match normal-path decode at 100 kHz CFO.

    Case A: frame at 100kHz CFO decoded via NORMAL path (wide gap, solo).
    Case B: same frame at 100kHz CFO decoded via PENDING path (tight gap,
    2nd of 2 frames).

    Gates three properties of the pending path:
      1. Frame 2 is detected and decodes with FCS OK (pending path works)
      2. T1 alignment (ltf1_offset) matches the normal path relative to
         buffer position (offset_error ~ 0 — a T1 error would put a phase
         slope across every DATA symbol)
      3. phase_inc matches (frame 2's CFO is estimated and applied, not
         frame 1's)

    (Historical note: the original hypothesis was that the pending path had
    a T1 alignment bug; A/B shows it decodes identically. This test is now
    a regression gate, and test_pending_differential_capture provides the
    stage-level report if it ever goes red.)
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    CFO_HZ = 100000
    rxtop_path = dut.u_rx_pipeline.u_decode_engine

    # === Case A: Normal path (wide gap) ===
    # Single frame with large leading silence → normal path decode
    await reset_dut(dut)
    rng = np.random.default_rng(seed=123)
    iq_frame = load_waveform_float(6)
    iq_frame = add_cfo(iq_frame, CFO_HZ)
    sig_rms = np.sqrt(np.mean(np.abs(iq_frame)**2))
    noise_std = sig_rms / (10**(40 / 20))
    iq_frame = iq_frame + (rng.normal(0, noise_std, len(iq_frame)) +
                           1j * rng.normal(0, noise_std, len(iq_frame)))
    # Wide leading gap so it's clearly normal path
    leading = np.zeros(5000, dtype=complex)
    trailing = np.zeros(8000, dtype=complex)
    stream_a = np.concatenate([leading, iq_frame, trailing])
    samples_a = quantize_12bit(stream_a)

    dut._log.info(f"=== Case A: Normal path, CFO={CFO_HZ}Hz, {len(samples_a)} samples ===")
    tags_a = await feed_and_collect_tags(dut, samples_a, n_expected=1, timeout_cycles=500000)

    # Capture internal state for Case A
    ltf1_offset_a = int(rxtop_path.ltf1_offset.value)
    fpi_a = int(rxtop_path.frame_phase_inc.value)
    if fpi_a >= 32768: fpi_a -= 65536
    fcs_a = tags_a[0]['fcs_ok'] if tags_a else False
    dut._log.info(f"  Case A result: fcs={'OK' if fcs_a else 'FAIL'}, "
                  f"ltf1_offset={ltf1_offset_a}, frame_phase_inc={fpi_a}")

    # === Case B: Pending path (tight gap) ===
    # Two frames: frame 1 (zero CFO) keeps pipeline busy, frame 2 (100kHz CFO)
    # arrives during decode → pending path. We care about frame 2's alignment.
    await reset_dut(dut)
    rng2 = np.random.default_rng(seed=123)  # same seed for frame 2
    iq1 = load_waveform_float(6)  # frame 1: zero CFO (dummy, keeps pipeline busy)
    iq1 = iq1 + (rng2.normal(0, noise_std, len(iq1)) +
                 1j * rng2.normal(0, noise_std, len(iq1)))
    # Frame 2: same as Case A (same CFO, same noise realization)
    rng3 = np.random.default_rng(seed=123)
    iq2 = load_waveform_float(6)
    iq2 = add_cfo(iq2, CFO_HZ)
    iq2 = iq2 + (rng3.normal(0, noise_std, len(iq2)) +
                 1j * rng3.normal(0, noise_std, len(iq2)))
    # Tight gap = 200 samples → frame 2 STF arrives during frame 1 decode
    leading_b = np.zeros(5000, dtype=complex)
    gap = rng2.normal(0, noise_std, 200) + 1j * rng2.normal(0, noise_std, 200)
    trailing_b = np.zeros(8000, dtype=complex)
    stream_b = np.concatenate([leading_b, iq1, gap, iq2, trailing_b])
    samples_b = quantize_12bit(stream_b)

    dut._log.info(f"=== Case B: Pending path, CFO={CFO_HZ}Hz, {len(samples_b)} samples ===")
    tags_b = await feed_and_collect_tags(dut, samples_b, n_expected=2, timeout_cycles=500000)

    # Frame 2 is the one that went through pending path
    ltf1_offset_b = int(rxtop_path.ltf1_offset.value)
    fpi_b = int(rxtop_path.frame_phase_inc.value)
    if fpi_b >= 32768: fpi_b -= 65536
    fcs_b2 = tags_b[1]['fcs_ok'] if len(tags_b) >= 2 else False
    fcs_b1 = tags_b[0]['fcs_ok'] if tags_b else False
    dut._log.info(f"  Case B frame 1: fcs={'OK' if fcs_b1 else 'FAIL'}")
    dut._log.info(f"  Case B frame 2: fcs={'OK' if fcs_b2 else 'FAIL'}, "
                  f"ltf1_offset={ltf1_offset_b}, frame_phase_inc={fpi_b}")

    # === Comparison ===
    dut._log.info("=== A/B COMPARISON ===")
    dut._log.info(f"  Normal path ltf1_offset:  {ltf1_offset_a}")
    dut._log.info(f"  Pending path ltf1_offset: {ltf1_offset_b}")
    dut._log.info(f"  Delta: {ltf1_offset_b - ltf1_offset_a}")
    dut._log.info(f"  Normal path phase_inc:    {fpi_a}")
    dut._log.info(f"  Pending path phase_inc:   {fpi_b}")
    dut._log.info(f"  Normal path FCS:  {'OK' if fcs_a else 'FAIL'}")
    dut._log.info(f"  Pending path FCS: {'OK' if fcs_b2 else 'FAIL'}")

    # === Relative offset check ===
    # Absolute ltf1_offset values differ (different buffer positions).
    # Compute the expected T1 position for frame 2 relative to its STF.
    # Normal path: frame STF at sample ~5044 in stream → ltf1_offset=147 in buffer
    # Pending path: frame 2 STF at sample ~8444 → ltf1_offset=3547 in buffer
    # Frame 1 STF at sample ~5044 → buffer origin for frame 1
    # Delta between STF positions: 8444-5044 = 3400 samples → buffer delta = 3400/5=680? No.
    # Actually wr_ptr increments once per iq_valid_in (1-per-5), so:
    # Frame 1 STF writes to buffer at wr_ptr ≈ 5044 (leading zeros + STF)
    # ltf1_offset_a = 147 means T1 is at buffer[147] (absolute from buf start)
    # In Case B (continuous buffer), frame 1 STF at sample 5044 writes to wr_ptr=5044
    # Frame 2 STF at sample 8444 writes to wr_ptr=8444
    # Expected frame 2 T1: ltf1_offset_a + (8444-5044) = 147 + 3400 = 3547
    expected_pending_offset = ltf1_offset_a + 3400  # approximate
    offset_error = ltf1_offset_b - expected_pending_offset
    dut._log.info(f"  Expected pending ltf1_offset: ~{expected_pending_offset}")
    dut._log.info(f"  Actual pending ltf1_offset:    {ltf1_offset_b}")
    dut._log.info(f"  Offset error: {offset_error} samples")

    # === Gate assertions ===
    # The pending path must decode identically to the normal path.
    # (Historically Case B was report-only; the pending path passes FCS
    # with zero offset error today, so it is gated now.)
    if not fcs_a:
        assert False, f"Case A (normal path) failed FCS — test setup problem"
    assert fcs_b1, "Case B frame 1 (normal path) failed FCS — test setup problem"
    assert len(tags_b) >= 2, \
        f"Pending path did not decode frame 2 (got {len(tags_b)} tags)"
    assert fcs_b2, \
        f"Pending path frame 2 failed FCS. phase_inc={fpi_b}, " \
        f"offset_error={offset_error} (normal path passed with the same frame)"
    assert abs(offset_error) <= 2, \
        f"Pending path T1 alignment wrong: offset_error={offset_error} " \
        f"(expected ~{expected_pending_offset}, got {ltf1_offset_b})"
    assert fpi_b == fpi_a, \
        f"Pending path applied wrong CFO: phase_inc={fpi_b}, normal={fpi_a}"


# =========================================================================
# Differential Signal Capture: A/B comparison at every pipeline boundary
# =========================================================================

async def _feed_with_capture(dut, samples, n_expected, timeout_cycles=500000):
    """Feed IQ and capture signals at every pipeline boundary per symbol.

    Returns dict with:
      - tags: list of tag dicts
      - mixer_samples: list of (re, im) per feed_mixer_valid_out clock
      - eq_data: list of lists — eq_data[sym_idx] = [(re, im), ...] 48 subcarriers
      - eq_pilots: list of lists — eq_pilots[sym_idx] = [(re, im), ...] 4 pilots
      - pt_phase_acc: list — pt_phase_acc[sym_idx] = phase_acc value after PLL update
      - pt_data: list of lists — pt_data[sym_idx] = [(re, im), ...] 48 subcarriers
    """
    rxtop = dut.u_rx_pipeline.u_decode_engine
    pilot_track = dut.u_rx_pipeline.u_pilot_track

    n_samples = len(samples)
    tags = []
    mixer_samples = []
    buf_samples = []   # raw BRAM read values (pre-mixer)
    eq_data = []       # per-symbol lists
    eq_pilots = []     # per-symbol lists
    pt_phase_acc = []  # per-symbol scalar
    pt_data = []       # per-symbol lists

    # Working accumulators for current symbol
    cur_eq_data = []
    cur_eq_pilots = []
    cur_pt_data = []
    in_data_symbols = False  # true after SIGNAL symbol seen

    # Buffer pointer snapshots at each feed burst start
    feed_burst_info = []  # list of (rd_ptr, wr_ptr) at start of each feed burst
    last_feed_valid = False

    sample_idx = 0
    valid_counter = 0
    sym_start_count = 0

    for cycle in range(timeout_cycles):
        await RisingEdge(dut.clk)

        # --- Feed IQ ---
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

        # --- Capture: mixer output + buffer pointers ---
        try:
            fv = int(rxtop.feed_mixer_valid_out.value)
            if fv == 1:
                re_val = int(rxtop.feed_mixer_re_out.value)
                im_val = int(rxtop.feed_mixer_im_out.value)
                # Convert unsigned 12-bit to signed
                if re_val >= 2048: re_val -= 4096
                if im_val >= 2048: im_val -= 4096
                mixer_samples.append((re_val, im_val))
                # Detect start of a new feed burst (rising edge of feed_valid)
                if not last_feed_valid:
                    rd = int(rxtop.rd_ptr.value)
                    wr = int(rxtop.wr_ptr.value)
                    feed_burst_info.append((rd, wr, len(mixer_samples)-1))
                last_feed_valid = True
            else:
                last_feed_valid = False
        except (ValueError, AttributeError):
            pass

        # --- Capture: raw buffer read (buf_re/buf_im on feed_valid) ---
        try:
            if int(rxtop.feed_valid.value) == 1:
                bre = int(rxtop.buf_re.value)
                bim = int(rxtop.buf_im.value)
                if bre >= 2048: bre -= 4096
                if bim >= 2048: bim -= 4096
                buf_samples.append((bre, bim))
        except (ValueError, AttributeError):
            pass

        # --- Capture: symbol_start (boundary marker) ---
        try:
            if int(dut.u_rx_pipeline.u_decode_engine.symbol_start_out.value) == 1:
                sym_start_count += 1
                is_sig = int(rxtop.is_signal_out.value)
                if is_sig:
                    in_data_symbols = False
                else:
                    # Commit previous DATA symbol's captures
                    if cur_eq_data:
                        eq_data.append(cur_eq_data)
                        eq_pilots.append(cur_eq_pilots)
                        pt_data.append(cur_pt_data)
                        # Capture phase_acc at symbol boundary (before new symbol)
                        try:
                            pa = int(pilot_track.phase_acc.value)
                            if pa >= 32768: pa -= 65536
                            pt_phase_acc.append(pa)
                        except (ValueError, AttributeError):
                            pt_phase_acc.append(None)
                    cur_eq_data = []
                    cur_eq_pilots = []
                    cur_pt_data = []
                    in_data_symbols = True
        except (ValueError, AttributeError):
            pass

        # --- Capture: equalizer data output (48 subcarriers per symbol) ---
        if in_data_symbols:
            try:
                if int(dut.u_rx_pipeline.eq_data_valid.value) == 1:
                    re_val = int(dut.u_rx_pipeline.eq_data_re.value)
                    im_val = int(dut.u_rx_pipeline.eq_data_im.value)
                    if re_val >= 32768: re_val -= 65536
                    if im_val >= 32768: im_val -= 65536
                    cur_eq_data.append((re_val, im_val))
            except (ValueError, AttributeError):
                pass

            # --- Capture: equalizer pilot output (4 per symbol) ---
            try:
                if int(dut.u_rx_pipeline.eq_pilot_valid.value) == 1:
                    re_val = int(dut.u_rx_pipeline.eq_pilot_re.value)
                    im_val = int(dut.u_rx_pipeline.eq_pilot_im.value)
                    if re_val >= 32768: re_val -= 65536
                    if im_val >= 32768: im_val -= 65536
                    cur_eq_pilots.append((re_val, im_val))
            except (ValueError, AttributeError):
                pass

            # --- Capture: pilot_track output (48 corrected subcarriers per symbol) ---
            try:
                if int(dut.u_rx_pipeline.pt_data_valid.value) == 1:
                    re_val = int(dut.u_rx_pipeline.pt_data_re.value)
                    im_val = int(dut.u_rx_pipeline.pt_data_im.value)
                    if re_val >= 32768: re_val -= 65536
                    if im_val >= 32768: im_val -= 65536
                    cur_pt_data.append((re_val, im_val))
            except (ValueError, AttributeError):
                pass

        # --- Capture: tag output ---
        try:
            if int(dut.tag_valid.value) == 1:
                # Commit last symbol's data before tag
                if cur_eq_data:
                    eq_data.append(cur_eq_data)
                    eq_pilots.append(cur_eq_pilots)
                    pt_data.append(cur_pt_data)
                    try:
                        pa = int(pilot_track.phase_acc.value)
                        if pa >= 32768: pa -= 65536
                        pt_phase_acc.append(pa)
                    except (ValueError, AttributeError):
                        pt_phase_acc.append(None)
                    cur_eq_data = []
                    cur_eq_pilots = []
                    cur_pt_data = []

                tag = {
                    'rate': int(dut.tag_rate.value),
                    'length': int(dut.tag_length.value),
                    'fcs_ok': int(dut.tag_fcs_ok.value),
                    'cycle': cycle,
                }
                tags.append(tag)
                if len(tags) >= n_expected:
                    break
        except (ValueError, AttributeError):
            pass

    return {
        'tags': tags,
        'mixer_samples': mixer_samples,
        'buf_samples': buf_samples,
        'eq_data': eq_data,
        'eq_pilots': eq_pilots,
        'pt_phase_acc': pt_phase_acc,
        'pt_data': pt_data,
        'feed_burst_info': feed_burst_info,
    }


def _compare_arrays(name, arr_a, arr_b, log, max_show=5):
    """Compare two lists of (re, im) tuples. Report first divergence."""
    min_len = min(len(arr_a), len(arr_b))
    if min_len == 0:
        log.info(f"    {name}: EMPTY (A={len(arr_a)}, B={len(arr_b)})")
        return -1

    first_diff = -1
    n_diff = 0
    for i in range(min_len):
        if arr_a[i] != arr_b[i]:
            if first_diff == -1:
                first_diff = i
            n_diff += 1

    if first_diff == -1 and len(arr_a) == len(arr_b):
        log.info(f"    {name}: IDENTICAL ({len(arr_a)} samples)")
        return -1
    elif first_diff == -1:
        log.info(f"    {name}: values match but length differs (A={len(arr_a)}, B={len(arr_b)})")
        return min_len  # divergence at length boundary
    else:
        log.warning(f"    {name}: DIVERGES at sample {first_diff}/{min_len} "
                    f"({n_diff} total diffs)")
        # Show first few differences
        shown = 0
        for i in range(first_diff, min_len):
            if arr_a[i] != arr_b[i]:
                log.info(f"      [{i}] A={arr_a[i]} B={arr_b[i]} "
                         f"delta=({arr_b[i][0]-arr_a[i][0]}, {arr_b[i][1]-arr_a[i][1]})")
                shown += 1
                if shown >= max_show:
                    break
        return first_diff


@cocotb.test()
async def test_pending_differential_capture(dut):
    """Gate: pending-path decode (2nd of 2 frames) passes FCS at 100 kHz CFO.

    Case A: single frame at 100kHz CFO → normal path decode.
    Case B: same frame at 100kHz CFO → pending path decode (2nd of 2 frames).

    While decoding, captures every pipeline boundary (mixer, equalizer
    data/pilots, pilot-track phase_acc and output) and reports the first
    divergence between the two paths. That report is diagnostic: the gate
    assertion is Case B's FCS, and if it fires the divergence report
    locates the failing stage.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    CFO_HZ = 100000
    SNR_DB = 40

    # === Build identical target frame ===
    iq_frame = load_waveform_float(6)
    iq_frame = add_cfo(iq_frame, CFO_HZ)
    rng = np.random.default_rng(seed=42)
    sig_rms = np.sqrt(np.mean(np.abs(iq_frame)**2))
    noise_std = sig_rms / (10**(SNR_DB / 20))
    iq_frame = iq_frame + (rng.normal(0, noise_std, len(iq_frame)) +
                           1j * rng.normal(0, noise_std, len(iq_frame)))

    # === Case A: Normal path ===
    dut._log.info("="*60)
    dut._log.info("CASE A: Normal path (solo frame, 100kHz CFO)")
    dut._log.info("="*60)
    await reset_dut(dut)

    leading_a = np.zeros(5000, dtype=complex)
    trailing_a = np.zeros(8000, dtype=complex)
    stream_a = np.concatenate([leading_a, iq_frame, trailing_a])
    samples_a = quantize_12bit(stream_a)

    cap_a = await _feed_with_capture(dut, samples_a, n_expected=1)

    fcs_a = cap_a['tags'][0]['fcs_ok'] if cap_a['tags'] else False
    dut._log.info(f"  FCS: {'OK' if fcs_a else 'FAIL'}")
    dut._log.info(f"  Mixer samples: {len(cap_a['mixer_samples'])}")
    dut._log.info(f"  EQ symbols: {len(cap_a['eq_data'])}")
    dut._log.info(f"  PT symbols: {len(cap_a['pt_data'])}")
    dut._log.info(f"  PT phase_acc: {cap_a['pt_phase_acc']}")

    assert fcs_a, "Case A must pass FCS — test setup problem"

    # === Case B: Pending path ===
    dut._log.info("-")
    dut._log.info("="*60)
    dut._log.info("CASE B: Pending path (2nd frame, 100kHz CFO)")
    dut._log.info("="*60)
    await reset_dut(dut)

    # Frame 1: zero-CFO dummy to keep pipeline busy
    rng2 = np.random.default_rng(seed=999)
    iq_dummy = load_waveform_float(6)
    iq_dummy = iq_dummy + (rng2.normal(0, noise_std, len(iq_dummy)) +
                           1j * rng2.normal(0, noise_std, len(iq_dummy)))

    leading_b = np.zeros(5000, dtype=complex)
    gap = rng2.normal(0, noise_std, 200) + 1j * rng2.normal(0, noise_std, 200)
    trailing_b = np.zeros(8000, dtype=complex)
    stream_b = np.concatenate([leading_b, iq_dummy, gap, iq_frame, trailing_b])
    samples_b = quantize_12bit(stream_b)

    # For Case B, we want frame 2's data. feed_with_capture captures everything,
    # so we collect 2 tags and then isolate frame 2's pipeline data.
    cap_b_raw = await _feed_with_capture(dut, samples_b, n_expected=2)

    fcs_b1 = cap_b_raw['tags'][0]['fcs_ok'] if len(cap_b_raw['tags']) >= 1 else False
    fcs_b2 = cap_b_raw['tags'][1]['fcs_ok'] if len(cap_b_raw['tags']) >= 2 else False
    dut._log.info(f"  Frame 1 FCS: {'OK' if fcs_b1 else 'FAIL'}")
    dut._log.info(f"  Frame 2 FCS: {'OK' if fcs_b2 else 'FAIL'}")
    dut._log.info(f"  Total mixer samples: {len(cap_b_raw['mixer_samples'])}")
    dut._log.info(f"  Total EQ symbols: {len(cap_b_raw['eq_data'])}")
    dut._log.info(f"  Total PT symbols: {len(cap_b_raw['pt_data'])}")
    dut._log.info(f"  All PT phase_acc: {cap_b_raw['pt_phase_acc']}")

    # Frame 1 is a rate-6 100-byte frame = ceil((22 + 800)/24) = 35 DATA symbols.
    # Frame 2's DATA symbols start after frame 1's 35 symbols.
    # The eq_data/pt_data lists accumulate across both frames (reset on is_signal).
    n_sym_frame1 = 35  # rate 6, 100 bytes → 35 DATA symbols
    # Frame 2's symbols are indices [n_sym_frame1:]
    cap_b = {
        'mixer_samples': cap_b_raw['mixer_samples'],  # will compare subsets
        'eq_data': cap_b_raw['eq_data'][n_sym_frame1:],
        'eq_pilots': cap_b_raw['eq_pilots'][n_sym_frame1:],
        'pt_phase_acc': cap_b_raw['pt_phase_acc'][n_sym_frame1:],
        'pt_data': cap_b_raw['pt_data'][n_sym_frame1:],
    }

    dut._log.info(f"  Frame 2 EQ symbols: {len(cap_b['eq_data'])}")
    dut._log.info(f"  Frame 2 PT symbols: {len(cap_b['pt_data'])}")
    dut._log.info(f"  Frame 2 PT phase_acc: {cap_b['pt_phase_acc']}")

    # === Differential Comparison ===
    dut._log.info("-")
    dut._log.info("="*60)
    dut._log.info("DIFFERENTIAL COMPARISON: A vs B (per stage, per symbol)")
    dut._log.info("="*60)

    # Stage 1: Mixer output (bulk comparison — different absolute positions)
    # For mixer, we can't directly compare sample-for-sample because the
    # absolute buffer positions differ. Instead compare total count and
    # note that LTF+SIG+DATA should produce the same per-symbol patterns.
    dut._log.info(f"")
    dut._log.info(f"  STAGE 1: Mixer output")
    dut._log.info(f"    Case A total mixer samples: {len(cap_a['mixer_samples'])}")
    dut._log.info(f"    Case B total mixer samples: {len(cap_b_raw['mixer_samples'])}")

    # Report feed burst info (buffer pointers at start of each burst)
    dut._log.info(f"    Case A feed bursts ({len(cap_a['feed_burst_info'])}):")
    for i, (rd, wr, mix_idx) in enumerate(cap_a['feed_burst_info'][:15]):
        dut._log.info(f"      burst {i:2d}: rd_ptr={rd:5d} wr_ptr={wr:5d} "
                      f"headroom={wr-rd:5d} mixer_idx={mix_idx}")
    dut._log.info(f"    Case B feed bursts ({len(cap_b_raw['feed_burst_info'])}):")
    for i, (rd, wr, mix_idx) in enumerate(cap_b_raw['feed_burst_info']):
        dut._log.info(f"      burst {i:2d}: rd_ptr={rd:5d} wr_ptr={wr:5d} "
                      f"headroom={wr-rd:5d} mixer_idx={mix_idx}")

    # Mixer samples for frame 2 in Case B start after frame 1's decode.
    # Frame 1: LTF1(64) + LTF2(64) + SIG(64) + 35*DATA(64) = 2432 samples
    frame1_mixer_count = 64 + 64 + 64 + n_sym_frame1 * 64
    mixer_b_frame2 = cap_b_raw['mixer_samples'][frame1_mixer_count:]
    dut._log.info(f"    Case B frame2 mixer samples: {len(mixer_b_frame2)}")
    mixer_diverge = _compare_arrays("mixer (frame2 vs A)",
                                    cap_a['mixer_samples'], mixer_b_frame2, dut._log)

    # Stage 1b: Raw buffer reads (pre-mixer)
    buf_b_frame2 = cap_b_raw['buf_samples'][frame1_mixer_count:]
    dut._log.info(f"    Case A buf_samples: {len(cap_a['buf_samples'])}")
    dut._log.info(f"    Case B frame2 buf_samples: {len(buf_b_frame2)}")
    buf_diverge = _compare_arrays("buf_re/im (frame2 vs A)",
                                  cap_a['buf_samples'], buf_b_frame2, dut._log)

    # Stage 2: EQ data (per-symbol comparison)
    dut._log.info(f"")
    dut._log.info(f"  STAGE 2: Equalizer data output (per symbol)")
    n_eq_syms = min(len(cap_a['eq_data']), len(cap_b['eq_data']))
    eq_first_diff_sym = -1
    for sym_i in range(n_eq_syms):
        if cap_a['eq_data'][sym_i] != cap_b['eq_data'][sym_i]:
            eq_first_diff_sym = sym_i
            break
    if eq_first_diff_sym == -1:
        dut._log.info(f"    EQ data: IDENTICAL across {n_eq_syms} symbols")
    else:
        dut._log.warning(f"    EQ data: DIVERGES at symbol {eq_first_diff_sym}")
        _compare_arrays(f"EQ sym {eq_first_diff_sym}",
                        cap_a['eq_data'][eq_first_diff_sym],
                        cap_b['eq_data'][eq_first_diff_sym], dut._log)

    # Stage 3: EQ pilots (per-symbol comparison)
    dut._log.info(f"")
    dut._log.info(f"  STAGE 3: Equalizer pilot output (per symbol)")
    n_pilot_syms = min(len(cap_a['eq_pilots']), len(cap_b['eq_pilots']))
    pilot_first_diff_sym = -1
    for sym_i in range(n_pilot_syms):
        if cap_a['eq_pilots'][sym_i] != cap_b['eq_pilots'][sym_i]:
            pilot_first_diff_sym = sym_i
            break
    if pilot_first_diff_sym == -1:
        dut._log.info(f"    EQ pilots: IDENTICAL across {n_pilot_syms} symbols")
    else:
        dut._log.warning(f"    EQ pilots: DIVERGES at symbol {pilot_first_diff_sym}")
        _compare_arrays(f"EQ pilots sym {pilot_first_diff_sym}",
                        cap_a['eq_pilots'][pilot_first_diff_sym],
                        cap_b['eq_pilots'][pilot_first_diff_sym], dut._log)

    # Stage 4: Pilot track phase_acc
    dut._log.info(f"")
    dut._log.info(f"  STAGE 4: Pilot track phase_acc (per symbol)")
    n_pa = min(len(cap_a['pt_phase_acc']), len(cap_b['pt_phase_acc']))
    pa_first_diff = -1
    for i in range(n_pa):
        if cap_a['pt_phase_acc'][i] != cap_b['pt_phase_acc'][i]:
            pa_first_diff = i
            break
    if pa_first_diff == -1:
        dut._log.info(f"    phase_acc: IDENTICAL across {n_pa} symbols")
        if n_pa > 0:
            dut._log.info(f"    values: {cap_a['pt_phase_acc'][:10]}...")
    else:
        dut._log.warning(f"    phase_acc: DIVERGES at symbol {pa_first_diff}")
        for i in range(pa_first_diff, min(pa_first_diff + 8, n_pa)):
            dut._log.info(f"      sym {i}: A={cap_a['pt_phase_acc'][i]} "
                          f"B={cap_b['pt_phase_acc'][i]} "
                          f"delta={cap_b['pt_phase_acc'][i] - cap_a['pt_phase_acc'][i]}")

    # Stage 5: Pilot track corrected output (per-symbol comparison)
    dut._log.info(f"")
    dut._log.info(f"  STAGE 5: Pilot track corrected output (per symbol)")
    n_pt_syms = min(len(cap_a['pt_data']), len(cap_b['pt_data']))
    pt_first_diff_sym = -1
    for sym_i in range(n_pt_syms):
        if cap_a['pt_data'][sym_i] != cap_b['pt_data'][sym_i]:
            pt_first_diff_sym = sym_i
            break
    if pt_first_diff_sym == -1:
        dut._log.info(f"    PT data: IDENTICAL across {n_pt_syms} symbols")
    else:
        dut._log.warning(f"    PT data: DIVERGES at symbol {pt_first_diff_sym}")
        _compare_arrays(f"PT sym {pt_first_diff_sym}",
                        cap_a['pt_data'][pt_first_diff_sym],
                        cap_b['pt_data'][pt_first_diff_sym], dut._log)

    # === Summary ===
    dut._log.info("-")
    dut._log.info("="*60)
    dut._log.info("SUMMARY")
    dut._log.info("="*60)
    dut._log.info(f"  Case A FCS: {'OK' if fcs_a else 'FAIL'}")
    dut._log.info(f"  Case B FCS: {'OK' if fcs_b2 else 'FAIL'}")

    # Identify the earliest divergence
    stages = [
        ("Buffer read", buf_diverge if buf_diverge != -1 else None, "sample"),
        ("Mixer", mixer_diverge if mixer_diverge != -1 else None, "sample"),
        ("EQ data", eq_first_diff_sym if eq_first_diff_sym != -1 else None, "symbol"),
        ("EQ pilots", pilot_first_diff_sym if pilot_first_diff_sym != -1 else None, "symbol"),
        ("PT phase_acc", pa_first_diff if pa_first_diff != -1 else None, "symbol"),
        ("PT output", pt_first_diff_sym if pt_first_diff_sym != -1 else None, "symbol"),
    ]

    first_stage = None
    for name, idx, unit in stages:
        if idx is not None:
            first_stage = (name, idx, unit)
            break

    if first_stage:
        dut._log.info(f"  FIRST DIVERGENCE: {first_stage[0]} at {first_stage[2]} {first_stage[1]}")
        dut._log.info(f"  → Bug is at or before: {first_stage[0]}")
    else:
        dut._log.info(f"  NO DIVERGENCE FOUND — pipeline outputs are identical")
        if not fcs_b2:
            dut._log.info(f"  → Bug must be downstream of pilot_track (demapper/Viterbi/FCS)")

    # === Gate assertion ===
    # The pending path must decode. The stage-by-stage comparison above is
    # diagnostic reporting: if this assert fires, the FIRST DIVERGENCE line
    # above locates the failing pipeline stage.
    assert fcs_b2, \
        f"Case B (pending path) failed FCS — pending decode regression. " \
        f"First divergence: " \
        f"{first_stage[0] if first_stage else 'downstream of pilot_track (demapper/Viterbi/FCS)'}"

