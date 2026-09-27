"""
test_adc_replay.py — Gate tests using real ADC captures from hardware.

THESE ARE TESTS. Every capture in captures/passing/ MUST decode with FCS OK.
A failing test means a regression — the RTL no longer decodes a waveform
that previously decoded correctly on hardware.

The test suite grows monotonically:
  1. Capture ADC on hardware (deimos_adc_capture)
  2. Fix RTL until sim replay passes
  3. Move capture to captures/passing/
  4. It becomes a permanent regression gate

Naming convention: captures/passing/<rate>_<description>.json
  e.g., captures/passing/6m_cable_ch149.json
        captures/passing/24m_cable_post_cfo_fix.json
        captures/passing/12m_cable_session21.json

Stimulus files (stimulus/passing/) are generated equivalents — see
stimulus/README.md. test_stimulus_replay_passing gates the single-frame
ones; the 4-way handshake burst is gated by test_eapol_burst_continuous
in this file.

This test uses the rx_frontend DUT (full front-end + decode pipeline),
matching hardware loopback config (no dead-zone, pilot PLL handles residual).
"""

import os
import json
import glob as globmod

import numpy as np
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from frontend_helpers import (
    reset_dut, run_frontend_decode, load_adc_capture,
    verify_psdu_path, s12_to_unsigned, SAMPLE_RATE,
)


CAPTURES_PASSING = os.path.join(os.path.dirname(__file__), '..', '..', 'captures', 'passing')
STIMULUS_PASSING = os.path.join(os.path.dirname(__file__), '..', '..', 'stimulus', 'passing')


@cocotb.test()
async def test_adc_replay_passing(dut):
    """All captures in captures/passing/ must decode with FCS OK.

    This is the hardware regression gate. Each file represents a real
    ADC capture that the RTL successfully decoded at some point. If any
    regresses, the code is broken.

    Skip (not fail) if no captures exist yet.
    """
    if not os.path.isdir(CAPTURES_PASSING):
        cocotb.log.info("captures/passing/ does not exist — skipping (no regression vectors yet)")
        return

    capture_files = sorted(globmod.glob(os.path.join(CAPTURES_PASSING, '*.json')))
    if not capture_files:
        cocotb.log.info("No captures in captures/passing/ — skipping (add captures to enable this gate)")
        return

    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    pass_count = 0
    fail_count = 0
    failures = []

    for filepath in capture_files:
        fname = os.path.basename(filepath)
        await reset_dut(dut)

        samples = load_adc_capture(filepath)
        dut._log.info(f"  Replay: {fname} ({len(samples)} samples)")

        r = await run_frontend_decode(dut, samples, timeout_cycles=5000000)

        if r['tag_valid'] and r['tag_fcs_ok']:
            pass_count += 1
            dut._log.info(f"    PASS (rate=0b{r['tag_rate']:04b}, len={r['tag_length']})")
            # PSDU byte-path invariant (payload unknown for real captures):
            # psdu_packer must emit length-4 payload bytes and frame_done.
            psdu_ok, psdu_reason = verify_psdu_path(
                r, expect_content=False,
                expected_count=r['tag_length'] - 4)
            if not psdu_ok:
                fail_count += 1
                failures.append((fname, f"PSDU: {psdu_reason}"))
                dut._log.info(f"    FAIL: PSDU byte path — {psdu_reason}")
        else:
            fail_count += 1
            status = "FCS_FAIL" if r['tag_valid'] else \
                     "NO_TAG" if r['signal_valid'] else \
                     "NO_DETECT"
            failures.append((fname, status))
            dut._log.info(f"    FAIL: {status}")

    dut._log.info(f"ADC Replay Gate: {pass_count}/{pass_count + fail_count} pass")

    assert fail_count == 0, \
        f"ADC replay regression: {fail_count} capture(s) failed: {failures}. " \
        f"These previously decoded on hardware — this is a regression."


@cocotb.test()
async def test_stimulus_replay_passing(dut):
    """All single-frame files in stimulus/passing/ must decode FCS-OK.

    Stimulus files are generated (not captured) — see stimulus/README.md.
    Multi-frame files (n_frames > 1) are gated by dedicated tests
    (test_eapol_burst_continuous), not here.
    """
    if not os.path.isdir(STIMULUS_PASSING):
        cocotb.log.info("stimulus/passing/ does not exist — skipping")
        return

    capture_files = sorted(globmod.glob(os.path.join(STIMULUS_PASSING, '*.json')))
    if not capture_files:
        cocotb.log.info("No stimulus files — skipping")
        return

    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    pass_count = 0
    fail_count = 0
    failures = []

    for filepath in capture_files:
        fname = os.path.basename(filepath)
        with open(filepath) as f:
            meta = json.load(f)
        if meta.get('n_frames', 1) > 1:
            dut._log.info(f"  Skip: {fname} (multi-frame — dedicated burst gate)")
            continue

        await reset_dut(dut)
        samples = load_adc_capture(filepath)
        dut._log.info(f"  Replay: {fname} ({len(samples)} samples)")

        r = await run_frontend_decode(dut, samples, timeout_cycles=5000000)

        if r['tag_valid'] and r['tag_fcs_ok']:
            pass_count += 1
            dut._log.info(f"    PASS (rate=0b{r['tag_rate']:04b}, len={r['tag_length']})")
            # PSDU byte-path gate: generated stimuli carry the known payload
            # (psdu_hex, WITHOUT FCS) — verify the emitted byte stream
            # matches it exactly. Closes the psdu_packer → tag_fifo_axi
            # coverage gap that let the TAG_HI bug live undetected.
            ef = meta['expected_frames'][0]
            psdu_ok, psdu_reason = verify_psdu_path(
                r,
                expected_payload=bytes.fromhex(ef['psdu_hex']),
                expected_count=ef['length'] - 4)
            if not psdu_ok:
                fail_count += 1
                failures.append((fname, f"PSDU: {psdu_reason}"))
                dut._log.info(f"    FAIL: PSDU byte path — {psdu_reason}")
        else:
            fail_count += 1
            status = "FCS_FAIL" if r['tag_valid'] else \
                     "NO_TAG" if r['signal_valid'] else \
                     "NO_DETECT"
            failures.append((fname, status))
            dut._log.info(f"    FAIL: {status}")

    dut._log.info(f"Stimulus Replay Gate: {pass_count}/{pass_count + fail_count} pass")
    assert fail_count == 0, \
        f"Stimulus replay regression: {fail_count} file(s) failed: {failures}."


@cocotb.test()
async def test_eapol_burst_continuous(dut):
    """Replay the synthetic EAPOL 4-way burst — all 14 decodable frames FCS-OK.

    Structure reproduced from the removed OTA capture: 14 legacy frames
    (all 6 Mbps) with the measured inter-frame STF deltas — EAPOLs
    7.2/42.8/1.2 ms apart (NOT SIFS), while M2/M4 ride tight ACK chains
    and every ACK follows its data at whatever spacing the original
    channel had. A 15th frame models the HT/VHT signal that correctly
    fails: valid STF/LTF, corrupted L-SIG -> decode abort -> NO tag (the
    original OTA capture also produced 14 tags, 0 FCS-fail in sim).

    This is the closest sim approximation to OTA EAPOL capture: coherent
    channel impairments, real inter-frame timing structure, and an
    undecodable frame mid-burst (offset-queue desync stress).

    If this passes in sim but fails OTA, the problem is STF sensitivity or
    ARM-side readout — not the decode pipeline.
    """
    burst_path = os.path.join(STIMULUS_PASSING, 'eapol_4way_burst.json')
    if not os.path.exists(burst_path):
        cocotb.log.info("eapol_4way_burst.json not found — skipping")
        return

    with open(burst_path) as f:
        meta = json.load(f)

    n_frames = meta.get('n_frames', 1)
    if n_frames < 2:
        cocotb.log.info("eapol_4way_burst.json has n_frames < 2 — skipping multi-frame test")
        return

    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Load and quantize the full burst
    samples = load_adc_capture(burst_path)
    n_samples = len(samples)
    dut._log.info(f"EAPOL burst: {n_samples} samples ({n_samples/SAMPLE_RATE*1000:.1f} ms), "
                  f"expecting >= {n_frames} EAPOL frames (plus other OTA traffic)")

    # Feed all samples continuously and collect ALL tags
    tags = []
    sample_idx = 0
    valid_counter = 0
    post_zero_count = 8192  # flush pipeline after last frame

    # Budget: all samples * 5 clk/sample + post_zeros + pipeline latency
    timeout_cycles = (n_samples + post_zero_count) * 5 + 2000000

    for cycle in range(timeout_cycles):
        await RisingEdge(dut.clk)

        # Feed IQ at 1-per-5 clock rate
        if valid_counter == 0:
            if sample_idx < n_samples:
                re_q, im_q = samples[sample_idx]
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = s12_to_unsigned(re_q)
                dut.iq_q_in.value = s12_to_unsigned(im_q)
                sample_idx += 1
            elif sample_idx < n_samples + post_zero_count:
                # Post-stream zeros (continuous ADC, flush pipeline)
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = 0
                dut.iq_q_in.value = 0
                sample_idx += 1
            else:
                dut.iq_valid_in.value = 0
        else:
            dut.iq_valid_in.value = 0

        valid_counter = (valid_counter + 1) % 5

        # Collect tag outputs (don't stop early — collect all frames in the burst)
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
                dut._log.info(f"  Frame {len(tags)}: "
                              f"rate=0b{tag['rate']:04b}, len={tag['length']}, "
                              f"fcs={fcs_str} @ sample ~{sample_idx}")
        except (ValueError, AttributeError):
            pass

        # Exit once all data fed + pipeline flush complete
        if sample_idx >= n_samples + post_zero_count:
            break

        # Progress logging
        if cycle > 0 and cycle % 2000000 == 0:
            dut._log.info(f"  cycle {cycle}: {len(tags)} frames decoded, "
                          f"sample {sample_idx}/{n_samples}")

    # --- Analysis ---
    fcs_ok_tags = [t for t in tags if t['fcs_ok']]
    fcs_fail_tags = [t for t in tags if not t['fcs_ok']]

    dut._log.info(f"EAPOL Burst Summary:")
    dut._log.info(f"  Total frames decoded: {len(tags)}")
    dut._log.info(f"  FCS OK: {len(fcs_ok_tags)}, FCS FAIL: {len(fcs_fail_tags)}")

    # --- Gate 1: FCS rate ratchet ---
    # Baseline: 14/15 FCS OK from this capture. If total decode count drops
    # or FCS rate degrades, it's a regression even if EAPOLs still decode.
    MIN_FCS_OK = 14
    assert len(fcs_ok_tags) >= MIN_FCS_OK, \
        f"FCS rate regression: {len(fcs_ok_tags)} FCS OK (need >= {MIN_FCS_OK}). " \
        f"Total: {len(tags)}, FCS FAIL: {len(fcs_fail_tags)}."

    # --- Gate 2: EAPOL identification by STF proximity + length ---
    # Each EAPOL frame must appear near its expected STF offset AND have the
    # correct PSDU length. This prevents false positives from non-EAPOL frames
    # that happen to share a length value.
    #
    # The EAPOL M1-M4 frames are at known positions within the burst.
    # stf_offsets may list ALL frames (not just EAPOLs), so we match by length.
    stf_offsets = meta.get('stf_offsets', [])
    expected_frames = meta.get('expected_frames', [])

    # Expected EAPOL PSDU lengths in order: M1, M2, M3, M4
    expected_eapol_lengths = [137, 159, 193, 137]

    # Find the EAPOL frames' STF offsets from expected_frames metadata
    # (match by length — EAPOL lengths are unique enough in this burst)
    eapol_stf_offsets = []
    used_indices = set()
    for exp_len in expected_eapol_lengths:
        for idx, (off, ef) in enumerate(zip(stf_offsets, expected_frames)):
            if idx not in used_indices and ef.get('length') == exp_len:
                eapol_stf_offsets.append(off)
                used_indices.add(idx)
                break
        else:
            # Fallback: no expected_frames, use stf_offsets directly (legacy)
            if not expected_frames:
                break

    # If we couldn't find EAPOL offsets from expected_frames, fall back to
    # first N stf_offsets (legacy behavior for files without expected_frames)
    if len(eapol_stf_offsets) != len(expected_eapol_lengths):
        eapol_stf_offsets = stf_offsets[:len(expected_eapol_lengths)]

    # Tolerance: frame detection sample should be within STF_PROXIMITY of the
    # expected STF offset. Accounts for STF/LTF/pipeline processing (~7000 samples).
    STF_PROXIMITY = 10000

    matched_eapols = []
    for i, (stf_off, exp_len) in enumerate(zip(eapol_stf_offsets, expected_eapol_lengths)):
        # Find FCS-OK frame with correct length near expected STF offset
        candidates = [
            t for t in fcs_ok_tags
            if t['length'] == exp_len
            and abs(t['sample'] - stf_off) < STF_PROXIMITY
        ]
        if candidates:
            best = min(candidates, key=lambda t: abs(t['sample'] - stf_off))
            matched_eapols.append(best)
            dut._log.info(f"    EAPOL M{i+1}: len={best['length']}, "
                          f"detected @ sample ~{best['sample']} "
                          f"(STF expected @ {stf_off}, delta={best['sample'] - stf_off})")
        else:
            dut._log.info(f"    EAPOL M{i+1}: NOT FOUND "
                          f"(expected len={exp_len} near STF @ {stf_off})")

    # Gate: all 4 EAPOL frames must be identified by position AND length
    assert len(matched_eapols) == len(expected_eapol_lengths), \
        f"Only {len(matched_eapols)}/{len(expected_eapol_lengths)} EAPOL frames matched by STF proximity + length. " \
        f"Pending trigger regression? Total: {len(tags)}, FCS OK: {len(fcs_ok_tags)}."

    # --- Gate 3: the HT/VHT stand-in must NOT decode (correct failure) ---
    # The synthetic burst contains one frame with corrupted L-SIG at a
    # known offset. It must produce NO FCS-OK tag near that offset.
    # (Original OTA behavior: 14 tags, 0 FCS-fail — the HT/VHT frame
    # never yields a decodable tag.)
    abort_offsets = [a.get('stf_offset') for a in meta.get('abort_frames', [])]
    for off in abort_offsets:
        near = [t for t in fcs_ok_tags
                if abs(t['sample'] - off) < STF_PROXIMITY]
        assert not near, \
            f"HT/VHT stand-in @ {off} decoded FCS-OK ({near}) — " \
            f"the undecodable frame must not decode."
        dut._log.info(f"    HT/VHT stand-in @ {off}: correctly produced no FCS-OK tag")
