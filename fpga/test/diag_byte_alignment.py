"""
diag_byte_alignment.py — Diagnostic: verify byte output matches tag_length per frame.

THIS IS A DIAGNOSTIC (never asserts, always exits 0). Run it to characterize
whether the pipeline's byte output count matches what tag_length reports.

WHY THIS EXISTS:
  On hardware, the byte FIFO alignment bug causes deimos_frame_read to read
  the wrong number of bytes for a frame, desynchronizing subsequent frames.
  This diagnostic checks the upstream question: does the pipeline itself emit
  the correct number of bytes per frame? If yes, the bug is firmware-only.
  If no, the bug is in psdu_packer or the pipeline byte emission logic.

WHAT IT DOES:
  Replays multi-frame captures (e.g., eapol_4way_burst.json) through
  rx_frontend, and for each frame:
    - Records tag_length from tag_valid
    - Counts psdu_byte_valid pulses between tag_valid events
    - Reports whether byte_count == tag_length - 4 (FCS not in byte stream)

RUNNING:
  ./scripts/sim.sh diag_byte_alignment
"""

import os
import json

import numpy as np
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from frontend_helpers import (
    reset_dut, s12_to_unsigned, quantize_12bit, SAMPLE_RATE,
)


CAPTURES_PASSING = os.path.join(os.path.dirname(__file__), '..', '..', 'captures', 'passing')
STIMULUS_PASSING = os.path.join(os.path.dirname(__file__), '..', '..', 'stimulus', 'passing')


def load_burst_capture(filepath):
    """Load a multi-frame burst capture and return quantized samples + metadata."""
    with open(filepath) as f:
        data = json.load(f)
    re = np.array(data['real'], dtype=np.float64)
    im = np.array(data['imag'], dtype=np.float64)
    iq = re + 1j * im
    samples = quantize_12bit(iq)
    return samples, data


async def feed_and_track_bytes(dut, samples, n_expected_frames, timeout_cycles=20000000):
    """Feed samples and track both tag outputs and byte counts per frame.

    Returns list of dicts with tag info and byte count for each frame.
    """
    n_samples = len(samples)
    frames = []
    sample_idx = 0
    valid_counter = 0
    current_byte_count = 0
    in_frame = False

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
            else:
                # Post-stream: keep feeding zeros
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = 0
                dut.iq_q_in.value = 0
        else:
            dut.iq_valid_in.value = 0

        valid_counter = (valid_counter + 1) % 5

        # Count byte pulses
        try:
            if int(dut.psdu_byte_valid.value) == 1:
                current_byte_count += 1
        except (ValueError, AttributeError):
            pass

        # Check for tag output (signals frame complete)
        try:
            if int(dut.tag_valid.value) == 1:
                tag_rate = int(dut.tag_rate.value)
                tag_length = int(dut.tag_length.value)
                tag_fcs_ok = int(dut.tag_fcs_ok.value)

                # Expected bytes = tag_length - 4 (FCS is checked but not emitted)
                expected_bytes = tag_length - 4 if tag_length > 4 else 0

                frame = {
                    'index': len(frames),
                    'tag_rate': tag_rate,
                    'tag_length': tag_length,
                    'tag_fcs_ok': tag_fcs_ok,
                    'byte_count': current_byte_count,
                    'expected_bytes': expected_bytes,
                    'aligned': current_byte_count == expected_bytes,
                    'delta': current_byte_count - expected_bytes,
                    'cycle': cycle,
                }
                frames.append(frame)

                dut._log.info(
                    f"  Frame {frame['index']}: rate=0b{tag_rate:04b} "
                    f"len={tag_length} fcs={'OK' if tag_fcs_ok else 'FAIL'} "
                    f"bytes={current_byte_count}/{expected_bytes} "
                    f"{'ALIGNED' if frame['aligned'] else 'MISALIGNED (delta=' + str(frame['delta']) + ')'}"
                )

                # Reset byte counter for next frame
                current_byte_count = 0

                if len(frames) >= n_expected_frames:
                    break
        except (ValueError, AttributeError):
            pass

        # Progress logging
        if cycle > 0 and cycle % 3000000 == 0:
            dut._log.info(f"  cycle {cycle}: {len(frames)}/{n_expected_frames} frames, "
                          f"sample {sample_idx}/{n_samples}, pending_bytes={current_byte_count}")

    return frames


@cocotb.test()
async def diag_burst_byte_alignment(dut):
    """Replay eapol_4way_burst.json and report byte alignment per frame.

    This is the primary diagnostic for the byte FIFO alignment bug.
    Reports whether the pipeline emits exactly (tag_length - 4) bytes per frame.
    """
    burst_path = os.path.join(STIMULUS_PASSING, 'eapol_4way_burst.json')
    if not os.path.exists(burst_path):
        dut._log.info("eapol_4way_burst.json not found — skipping")
        return

    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    samples, metadata = load_burst_capture(burst_path)
    n_frames = metadata.get('n_frames', 4)
    dut._log.info(f"Replaying eapol_4way_burst.json: {len(samples)} samples, "
                  f"{n_frames} expected frames")

    frames = await feed_and_track_bytes(dut, samples, n_frames, timeout_cycles=30000000)

    # Summary
    dut._log.info(f"\n{'='*60}")
    dut._log.info(f"BYTE ALIGNMENT DIAGNOSTIC RESULTS")
    dut._log.info(f"{'='*60}")
    dut._log.info(f"  Frames detected: {len(frames)}/{n_frames}")

    aligned_count = sum(1 for f in frames if f['aligned'])
    misaligned = [f for f in frames if not f['aligned']]

    dut._log.info(f"  Aligned:         {aligned_count}/{len(frames)}")
    if misaligned:
        dut._log.info(f"  MISALIGNED:      {len(misaligned)} frames")
        for f in misaligned:
            dut._log.info(f"    Frame {f['index']}: expected {f['expected_bytes']} bytes, "
                          f"got {f['byte_count']} (delta={f['delta']})")
        dut._log.info(f"\n  VERDICT: BYTE MISALIGNMENT DETECTED IN PIPELINE")
        dut._log.info(f"  This means psdu_packer emits wrong byte count — RTL bug.")
    else:
        dut._log.info(f"\n  VERDICT: PIPELINE BYTE OUTPUT IS CORRECT")
        dut._log.info(f"  If hardware shows misalignment, bug is in firmware byte consumption.")

    fcs_results = [f for f in frames if f['tag_fcs_ok']]
    dut._log.info(f"  FCS OK:          {len(fcs_results)}/{len(frames)}")
    dut._log.info(f"{'='*60}")


@cocotb.test()
async def diag_single_frame_byte_counts(dut):
    """Replay each single-frame capture and report byte alignment.

    Cross-checks that single-frame captures also have correct byte counts.
    """
    if not os.path.isdir(CAPTURES_PASSING):
        dut._log.info("captures/passing/ not found — skipping")
        return

    import glob as globmod
    capture_files = sorted(globmod.glob(os.path.join(CAPTURES_PASSING, '*.json')))
    # Exclude burst captures (n_frames > 1)
    single_frame_files = []
    for fp in capture_files:
        with open(fp) as f:
            data = json.load(f)
        if data.get('n_frames', 1) == 1:
            single_frame_files.append(fp)

    if not single_frame_files:
        dut._log.info("No single-frame captures — skipping")
        return

    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    results = []
    for filepath in single_frame_files:
        fname = os.path.basename(filepath)
        await reset_dut(dut)

        samples, metadata = load_burst_capture(filepath)
        frames = await feed_and_track_bytes(dut, samples, n_expected_frames=1,
                                            timeout_cycles=5000000)

        if frames:
            f = frames[0]
            results.append({'file': fname, **f})
            status = 'ALIGNED' if f['aligned'] else f'MISALIGNED (delta={f["delta"]})'
            dut._log.info(f"  {fname}: len={f['tag_length']} bytes={f['byte_count']} {status}")
        else:
            results.append({'file': fname, 'aligned': False, 'tag_length': 0, 'byte_count': 0})
            dut._log.info(f"  {fname}: NO TAG OUTPUT")

    # Summary
    aligned = sum(1 for r in results if r.get('aligned', False))
    dut._log.info(f"\n  Single-frame byte alignment: {aligned}/{len(results)} correct")
