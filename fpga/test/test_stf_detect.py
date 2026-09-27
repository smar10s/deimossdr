"""
Test STF (Short Training Field) detector.

Tests:
1. Feed 6 Mbps golden waveform → assert frame_detect fires
2. Feed pure noise → assert frame_detect does NOT fire
3. Feed waveform with noise prepended → assert correct offset
4. Feed back-to-back frames → detect both

The STF detector uses:
- Lag-16, window-64 sliding normalized autocorrelation
- Squared threshold: |P|^2 >= 0.36 * E1 * E2
- Duration check: 8 consecutive periods above threshold
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer

import json
import os
import random
import math

VECTORS_DIR = os.path.join(os.path.dirname(__file__), "../../extern/lib80211/vectors")


def load_6mbps_waveform_quantized(scale=6000):
    """Load the 6 Mbps golden waveform, quantize to 12-bit signed."""
    path = os.path.join(VECTORS_DIR, "legacy_6mbps_waveform.json")
    with open(path) as f:
        d = json.load(f)
    re_f = d["real"]
    im_f = d["imag"]
    # Quantize to 12-bit signed [-2048, 2047]
    re = [max(-2048, min(2047, int(round(x * scale)))) for x in re_f]
    im = [max(-2048, min(2047, int(round(x * scale)))) for x in im_f]
    return re, im


def reference_stf_detect(re, im):
    """Python reference implementation matching the RTL algorithm.
    Returns sample index where frame_detect fires, or -1."""
    LAG = 16
    WINDOW = 64
    THRESH_SQ_NUM = 36      # 0.36 scaled by 100
    THRESH_SQ_DEN = 100
    DUR_THRESH_SQ = 0.1764  # 0.42^2
    MIN_PERIODS = 8
    n = len(re)

    def duration_check(pos):
        consec = 0
        nn = pos
        while nn + LAG + LAG <= n:
            pr, pi_acc, e1, e2 = 0, 0, 0, 0
            for k in range(LAG):
                r1, i1 = re[nn + k], im[nn + k]
                r2, i2 = re[nn + k + LAG], im[nn + k + LAG]
                pr += r1 * r2 + i1 * i2
                pi_acc += i1 * r2 - r1 * i2
                e1 += r1 * r1 + i1 * i1
                e2 += r2 * r2 + i2 * i2
            p_sq = pr * pr + pi_acc * pi_acc
            e_prod = e1 * e2
            if e_prod > 0 and p_sq * 100 >= 18 * e_prod:  # 0.1764 ≈ 18/100 (approx)
                consec += 1
                if consec >= MIN_PERIODS:
                    return True
            else:
                break
            nn += LAG
        return False

    max_pos = n - WINDOW - LAG
    if max_pos < 0:
        return -1

    # Init sliding accumulators
    pr, pi_acc, e1, e2 = 0, 0, 0, 0
    for k in range(WINDOW):
        r1, i1 = re[k], im[k]
        r2, i2 = re[k + LAG], im[k + LAG]
        pr += r1 * r2 + i1 * i2
        pi_acc += i1 * r2 - r1 * i2
        e1 += r1 * r1 + i1 * i1
        e2 += r2 * r2 + i2 * i2

    for nn in range(max_pos + 1):
        p_sq = pr * pr + pi_acc * pi_acc
        e_prod = e1 * e2
        if e_prod > 0 and p_sq * THRESH_SQ_DEN >= THRESH_SQ_NUM * e_prod:
            if duration_check(nn):
                return nn

        if nn < max_pos:
            r1_old, i1_old = re[nn], im[nn]
            r2_old, i2_old = re[nn + LAG], im[nn + LAG]
            pr -= r1_old * r2_old + i1_old * i2_old
            pi_acc -= i1_old * r2_old - r1_old * i2_old
            e1 -= r1_old * r1_old + i1_old * i1_old
            e2 -= r2_old * r2_old + i2_old * i2_old
            new_idx = nn + WINDOW
            r1_new, i1_new = re[new_idx], im[new_idx]
            r2_new, i2_new = re[new_idx + LAG], im[new_idx + LAG]
            pr += r1_new * r2_new + i1_new * i2_new
            pi_acc += i1_new * r2_new - r1_new * i2_new
            e1 += r1_new * r1_new + i1_new * i1_new
            e2 += r2_new * r2_new + i2_new * i2_new

    return -1


async def reset(dut):
    """Assert reset for a few cycles."""
    dut.rst_n.value = 0
    dut.enable.value = 1
    dut.clear.value = 0
    dut.iq_valid.value = 0
    dut.iq_i.value = 0
    dut.iq_q.value = 0
    dut.threshold.value = 0  # default: shift=0, standard 0.36 threshold
    dut.pipeline_ack.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


def to_twos_complement_12(val):
    """Convert signed int to 12-bit two's complement."""
    if val < 0:
        return val + 4096
    return val


async def feed_iq_stream(dut, re_samples, im_samples):
    """Feed IQ samples at 1-in-5 spacing (matching real ADC-to-fabric rate).

    Returns list of (sample_idx, offset) for detections and sample indices
    where stf_end fired. The 1-in-5 spacing means iq_valid is asserted for
    1 clock then deasserted for 4 clocks, matching the 20 MSPS ADC rate within
    the 100 MHz fabric clock.

    Detection outputs (frame_detect, stf_end) are gated by iq_valid in RTL,
    so they fire on the iq_valid clock. We check after the rising edge where
    iq_valid was asserted (the RTL registers update at that posedge).

    Automatically pulses pipeline_ack after frame_detect fires (simulates
    pipeline accepting the trigger — required for free-running correlator
    to eventually clear detected_latch).
    """
    detections = []
    stf_ends = []
    for i in range(len(re_samples)):
        dut.iq_i.value = to_twos_complement_12(re_samples[i])
        dut.iq_q.value = to_twos_complement_12(im_samples[i])
        dut.iq_valid.value = 1
        await RisingEdge(dut.clk)
        dut.iq_valid.value = 0
        await Timer(1, units="ns")  # Let combinational settle
        detected = int(dut.frame_detect.value) == 1
        if detected:
            offset = int(dut.frame_offset.value)
            detections.append((i, offset))
        if int(dut.stf_end.value) == 1:
            stf_ends.append(i)
        if detected:
            # Simulate pipeline accepting the detection (1-cycle pulse)
            dut.pipeline_ack.value = 1
            await RisingEdge(dut.clk)
            dut.pipeline_ack.value = 0
            # 3 remaining idle clocks (already consumed 1 for ack)
            await ClockCycles(dut.clk, 3)
        else:
            # 4 idle clocks (1-in-5 spacing)
            await ClockCycles(dut.clk, 4)
    return detections, stf_ends


@cocotb.test()
async def test_detect_6mbps_waveform(dut):
    """Feed 6 Mbps golden waveform → frame_detect fires in STF region."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    re, im = load_6mbps_waveform_quantized()

    # Only feed the first 400 samples (STF is 160, LTF starts at 160)
    # Detection should happen within the STF region
    n_feed = 400
    detections, stf_ends = await feed_iq_stream(dut, re[:n_feed], im[:n_feed])

    assert len(detections) >= 1, \
        f"Expected at least one detection, got {len(detections)}"

    # Detection should happen within the STF region (samples 0-160)
    first_cycle, first_offset = detections[0]
    # The detector needs to accumulate lag+window = 80 samples minimum,
    # plus 16 periods for persistence check.
    # So detection fires somewhere around sample 80-210 (allowing pipeline latency)
    assert first_offset <= 220, \
        f"Detection too late: offset={first_offset}, expected within STF+margin"

    dut._log.info(f"Detection at cycle={first_cycle}, offset={first_offset}")


@cocotb.test()
async def test_no_false_detect_noise(dut):
    """Pure random noise → no frame_detect."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    random.seed(42)
    n_samples = 1000
    # Noise: uniformly random in [-500, 500] (moderate power, no correlation)
    re_noise = [random.randint(-500, 500) for _ in range(n_samples)]
    im_noise = [random.randint(-500, 500) for _ in range(n_samples)]

    detections, stf_ends = await feed_iq_stream(dut, re_noise, im_noise)

    assert len(detections) == 0, \
        f"False detection in noise: got {len(detections)} detections"


@cocotb.test()
async def test_detect_with_noise_prefix(dut):
    """Noise prefix + waveform → detect at correct offset."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    random.seed(123)
    n_prefix = 500
    noise_re = [random.randint(-200, 200) for _ in range(n_prefix)]
    noise_im = [random.randint(-200, 200) for _ in range(n_prefix)]

    re, im = load_6mbps_waveform_quantized()

    # Concatenate noise + waveform (just need first 400 of waveform)
    full_re = noise_re + re[:400]
    full_im = noise_im + im[:400]

    detections, stf_ends = await feed_iq_stream(dut, full_re, full_im)

    assert len(detections) >= 1, \
        f"Expected detection after noise prefix, got {len(detections)}"

    first_cycle, first_offset = detections[0]
    # Offset should be within the STF onset region.
    # The sliding window (64 samples) means detection can fire up to ~64
    # samples before the actual STF start (partial window overlap).
    # Allow detection from (n_prefix - 64) to (n_prefix + 160).
    assert first_offset >= n_prefix - 64, \
        f"Detection offset {first_offset} too early (expected >= {n_prefix - 64})"
    assert first_offset < n_prefix + 160, \
        f"Detection offset {first_offset} too late (expected < {n_prefix + 160})"

    dut._log.info(f"Detection at offset={first_offset} (prefix={n_prefix})")


@cocotb.test()
async def test_back_to_back_frames(dut):
    """Two frames separated by a gap → detect both."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    re, im = load_6mbps_waveform_quantized()

    # Gap of 200 samples of silence between frames
    gap = 200
    silence_re = [0] * gap
    silence_im = [0] * gap

    # Feed first frame (first 400 samples), then gap, then second frame
    full_re = re[:400] + silence_re + re[:400]
    full_im = im[:400] + silence_im + im[:400]

    detections, stf_ends = await feed_iq_stream(dut, full_re, full_im)

    assert len(detections) >= 2, \
        f"Expected 2 detections for back-to-back frames, got {len(detections)}"

    # Second detection should be offset by ~600 (400 + 200 gap) from first
    off1 = detections[0][1]
    off2 = detections[1][1]
    separation = off2 - off1
    assert separation > 300, \
        f"Detections too close: off1={off1}, off2={off2}, sep={separation}"

    dut._log.info(f"Frame 1 at offset={off1}, Frame 2 at offset={off2}")


@cocotb.test()
async def test_stf_end_timing(dut):
    """STF end pulse fires near the STF/GI2 boundary (~sample 160)."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    re, im = load_6mbps_waveform_quantized()

    # Feed enough samples to cover STF(160) + GI2(32) + some margin
    n_feed = 300
    detections, stf_ends = await feed_iq_stream(dut, re[:n_feed], im[:n_feed])

    assert len(detections) >= 1, \
        f"Expected frame_detect, got {len(detections)}"
    assert len(stf_ends) >= 1, \
        f"Expected stf_end pulse, got {len(stf_ends)}"

    detect_cycle = detections[0][0]
    end_cycle = stf_ends[0]

    # stf_end must come AFTER frame_detect
    assert end_cycle > detect_cycle, \
        f"stf_end ({end_cycle}) should come after frame_detect ({detect_cycle})"

    # STF boundary is at sample 160. With threshold pipeline delay (~5 cycles)
    # and 8-sample persistence for end detection, expect stf_end around 160-180.
    # Allow wider margin for pipeline effects: 155-200.
    assert 155 <= end_cycle <= 200, \
        f"stf_end at cycle {end_cycle}, expected 155-200 (STF boundary ~160)"

    dut._log.info(f"frame_detect at cycle {detect_cycle}, stf_end at cycle {end_cycle}, "
                  f"delta={end_cycle - detect_cycle} samples")


@cocotb.test()
async def test_dc_offset_false_trigger(dut):
    """DC offset in IQ stream → triggers false frame_detect.

    Reproduces the cold-start bug: AD9361 DC offset before calibration
    settles creates perfect lag-16 autocorrelation (metric=1.0), causing
    continuous false STF detections. Just 8 LSBs of DC offset (0.39% FS)
    is enough to exceed the 0.36 threshold with persistence.

    This test proves the vulnerability exists — the fix (DC-removal HPF)
    should make this test pass by preventing false detection.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    # DC offset of 20 LSBs (well within AD9361 uncalibrated range of 10-50)
    # with small noise floor (RMS ~5 LSBs)
    random.seed(77)
    dc_i, dc_q = 20, -15  # Typical uncalibrated DC offset
    n_dc_samples = 200  # Enough for fill (82) + persistence (16) + margin

    dc_re = [dc_i + random.randint(-5, 5) for _ in range(n_dc_samples)]
    dc_im = [dc_q + random.randint(-5, 5) for _ in range(n_dc_samples)]

    detections, _ = await feed_iq_stream(dut, dc_re, dc_im)

    # BUG: DC offset SHOULD NOT trigger detection, but currently DOES.
    # After the HPF fix, this assert should pass (no detections).
    # For now, this test documents the failure mode.
    assert len(detections) == 0, \
        f"DC offset caused {len(detections)} false detection(s) — " \
        f"STF detector is not DC-immune"


@cocotb.test()
async def test_dc_offset_blocks_real_frame(dut):
    """DC offset triggers false detect → real frame after clear is missed.

    Demonstrates the operational impact: DC offset causes detected_latch=1,
    and without an external clear, subsequent real frames cannot be detected.
    Even with the watchdog fix (external clear after timeout), the frame
    must arrive in the brief window between clear and re-trigger.

    After the HPF fix, DC should be rejected entirely and the frame detected.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    random.seed(88)
    dc_i, dc_q = 20, -15

    # Phase 1: DC offset triggers false detection (200 samples)
    n_dc = 200
    dc_re = [dc_i + random.randint(-5, 5) for _ in range(n_dc)]
    dc_im = [dc_q + random.randint(-5, 5) for _ in range(n_dc)]

    # Phase 2: DC goes away, then a real frame arrives
    # (simulates radio settling + real TX arriving)
    n_quiet = 100  # Gap with no DC (radio settled)
    quiet_re = [random.randint(-5, 5) for _ in range(n_quiet)]
    quiet_im = [random.randint(-5, 5) for _ in range(n_quiet)]

    # Phase 3: Real 6 Mbps frame
    frame_re, frame_im = load_6mbps_waveform_quantized()

    # Concatenate: DC → quiet → frame
    full_re = dc_re + quiet_re + frame_re[:400]
    full_im = dc_im + quiet_im + frame_im[:400]

    detections, _ = await feed_iq_stream(dut, full_re, full_im)

    # We expect the REAL frame to be detected (after DC goes away).
    # The frame starts at sample n_dc + n_quiet = 300.
    real_frame_detections = [
        (cyc, off) for cyc, off in detections
        if off >= n_dc + n_quiet - 64  # Frame region (allow window overlap)
    ]

    assert len(real_frame_detections) >= 1, \
        f"Real frame not detected after DC offset cleared — " \
        f"got detections at offsets {[d[1] for d in detections]}, " \
        f"expected detection near sample {n_dc + n_quiet}"

    dut._log.info(f"Frame detected at offset {real_frame_detections[0][1]} "
                   f"(frame starts at sample {n_dc + n_quiet})")


@cocotb.test()
async def test_detected_latch_lockup_continuous_signal(dut):
    """Verifies rearm timeout prevents STF lockup under continuous high-correlation signal.

    ORIGINAL BUG (stf_detect.v line 425): detected_latch only cleared when BOTH:
      1. pipeline_ack_latch is set (pipeline accepted the detection)
      2. !threshold_met (autocorrelation metric dropped below threshold)

    In dense traffic (e.g., EAPOL handshake burst with surrounding traffic),
    the autocorrelation can stay above threshold continuously across multiple
    frames. Once detected_latch gets set, it never clears → no new frame_detect
    pulses → fabric appears dead.

    FIX: rearm timeout (REARM_TIMEOUT=128 samples). After pipeline acks and
    128 samples elapse, force-clear detected_latch regardless of threshold_met.
    This allows the detector to rearm even when the channel has continuous signal.

    Test structure:
      Phase 1: Real STF (160 samples) → triggers detection
      Phase 2: Continuous STF-like signal (500 samples) — keeps metric high.
               Rearm timeout fires periodically (this is expected/correct).
      Phase 3: Noise gap (50 samples) — metric briefly drops
      Phase 4: Second real STF (160 samples) → MUST trigger detection

    Gate: detection occurs in the Phase 4 STF region. Without the fix,
    the latch stays stuck from Phase 1 and Phase 4 is invisible.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    re, im = load_6mbps_waveform_quantized()

    # Phase 1: Feed real STF to trigger first detection (first 400 samples)
    # The STF is 10 repetitions of a 16-sample pattern — high autocorrelation.
    stf_re = re[:160]
    stf_im = im[:160]

    # Phase 2: Continuous high-correlation "traffic" — just repeat the 16-sample
    # STF pattern endlessly. This keeps |P|^2 / (E1*E2) above threshold,
    # preventing detected_latch from clearing (the original bug).
    # Use the first 16 samples of the STF (one period of the repeating pattern).
    pattern_re = re[:16]
    pattern_im = im[:16]

    # 500 samples of continuous STF-like signal (well beyond the persist window)
    continuous_len = 500
    continuous_re = [pattern_re[i % 16] for i in range(continuous_len)]
    continuous_im = [pattern_im[i % 16] for i in range(continuous_len)]

    # Phase 3: Brief noise gap (50 samples) — simulates a tiny inter-frame
    # moment. The metric may or may not fully drop in 50 samples depending
    # on the sliding window (64 samples). This makes the test realistic:
    # on a busy channel, you get brief sub-threshold dips between frames.
    rng = random.Random(42)
    noise_len = 50
    noise_re = [rng.randint(-100, 100) for _ in range(noise_len)]
    noise_im = [rng.randint(-100, 100) for _ in range(noise_len)]

    # Phase 4: Second real STF — must be detected
    second_stf_re = re[:160]
    second_stf_im = im[:160]

    # Assemble the full stream
    # Phase boundaries: [0:160] [160:660] [660:710] [710:870]
    full_re = stf_re + continuous_re + noise_re + second_stf_re
    full_im = stf_im + continuous_im + noise_im + second_stf_im

    phase4_start = len(stf_re) + continuous_len + noise_len  # = 710

    # Custom feed loop so we can control pipeline_ack precisely
    detections = []
    for i in range(len(full_re)):
        dut.iq_i.value = to_twos_complement_12(full_re[i])
        dut.iq_q.value = to_twos_complement_12(full_im[i])
        dut.iq_valid.value = 1
        await RisingEdge(dut.clk)
        dut.iq_valid.value = 0
        await Timer(1, units="ns")  # Let combinational settle
        detected = int(dut.frame_detect.value) == 1
        if detected:
            offset = int(dut.frame_offset.value)
            detections.append((i, offset))
            dut._log.info(f"  Detection #{len(detections)} at sample {i}, offset={offset}")
            # Pulse pipeline_ack (pipeline accepted)
            dut.pipeline_ack.value = 1
            await RisingEdge(dut.clk)
            dut.pipeline_ack.value = 0
            await ClockCycles(dut.clk, 3)
        else:
            await ClockCycles(dut.clk, 4)

    dut._log.info(f"Total detections: {len(detections)}")
    dut._log.info(f"  All sample indices: {[d[0] for d in detections]}")
    dut._log.info(f"  Phase 4 starts at sample {phase4_start}")

    # Check detected_latch state at end
    await RisingEdge(dut.clk)
    latch_state = int(dut.detected_latch.value)
    ack_latch_state = int(dut.pipeline_ack_latch.value)
    dut._log.info(f"  End state: detected_latch={latch_state}, "
                   f"pipeline_ack_latch={ack_latch_state}")

    # GATE: at least one detection must occur in the Phase 4 region (sample >= 710).
    # Without the rearm timeout fix, the latch stays stuck from Phase 1 and
    # nothing in Phase 4 is detected — even after the noise gap.
    phase4_detections = [d for d in detections if d[0] >= phase4_start]
    assert len(phase4_detections) >= 1, \
        f"No detection in Phase 4 (second STF at sample >= {phase4_start}). " \
        f"Got detections only at: {[d[0] for d in detections]}. " \
        f"Lockup bug: detected_latch not clearing under continuous signal."

    # Also verify we got multiple detections total (rearm is working)
    assert len(detections) >= 2, \
        f"Expected >= 2 total detections, got {len(detections)}."

    dut._log.info(f"PASS: Rearm timeout works. Phase 4 detection at sample "
                   f"{phase4_detections[0][0]} (second STF region).")


@cocotb.test()
async def test_hil_burst_two_frames_continuous_valid(dut):
    """Two frames with continuous iq_valid (HIL mode) — both must be detected.

    This reproduces the hardware HIL burst failure: 1-per-clock iq_valid
    (vs 1-per-5 in live mode). If the STF detector's rearm logic depends
    on the idle gaps between iq_valid pulses, HIL burst will fail.

    Structure:
      - 500 samples noise (pre-frame silence)
      - Frame 1: full 6 Mbps waveform (STF + LTF + SIGNAL + DATA)
      - Gap: 5000 samples of 1% noise (matches firmware HIL burst gen)
      - Frame 2: full 6 Mbps waveform
      - 200 samples noise (post)

    Gate: both frames must produce frame_detect.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    re, im = load_6mbps_waveform_quantized()
    frame_len = len(re)

    # Build the burst waveform
    rng = random.Random(99)
    noise_amp = 60  # 1% of typical 6000 scale = 60 (matches firmware 0.01 * peak)

    pre_len = 500
    gap_len = 5000
    post_len = 200

    pre_re = [rng.randint(-noise_amp, noise_amp) for _ in range(pre_len)]
    pre_im = [rng.randint(-noise_amp, noise_amp) for _ in range(pre_len)]
    gap_re = [rng.randint(-noise_amp, noise_amp) for _ in range(gap_len)]
    gap_im = [rng.randint(-noise_amp, noise_amp) for _ in range(gap_len)]
    post_re = [rng.randint(-noise_amp, noise_amp) for _ in range(post_len)]
    post_im = [rng.randint(-noise_amp, noise_amp) for _ in range(post_len)]

    full_re = pre_re + re + gap_re + re + post_re
    full_im = pre_im + im + gap_im + im + post_im

    frame1_start = pre_len
    frame2_start = pre_len + frame_len + gap_len

    dut._log.info(f"Burst: {len(full_re)} samples, frame1@{frame1_start}, "
                  f"frame2@{frame2_start}, gap={gap_len}")

    # Feed with CONTINUOUS iq_valid (1 per clock — HIL mode)
    detections = []
    for i in range(len(full_re)):
        dut.iq_i.value = to_twos_complement_12(full_re[i])
        dut.iq_q.value = to_twos_complement_12(full_im[i])
        dut.iq_valid.value = 1
        await RisingEdge(dut.clk)
        # Check frame_detect (registered output, valid after posedge)
        await Timer(1, units="ns")
        detected = int(dut.frame_detect.value) == 1
        if detected:
            offset = int(dut.frame_offset.value)
            detections.append((i, offset))
            dut._log.info(f"  Detection #{len(detections)} at sample {i}, offset={offset}")
            # Pulse pipeline_ack (same-cycle feedback as BD wiring)
            dut.pipeline_ack.value = 1
            await RisingEdge(dut.clk)
            dut.pipeline_ack.value = 0

    dut._log.info(f"Total detections: {len(detections)}")
    dut._log.info(f"  Sample indices: {[d[0] for d in detections]}")

    # Gate: at least 2 detections (one per frame)
    assert len(detections) >= 2, \
        f"Expected >= 2 detections (one per frame), got {len(detections)}. " \
        f"Detected at samples: {[d[0] for d in detections]}. " \
        f"Frame 1 starts at {frame1_start}, frame 2 at {frame2_start}. " \
        f"HIL burst rearm failure: only first frame detected."

    # Verify detections are in the right regions
    frame1_dets = [d for d in detections if frame1_start <= d[0] < frame1_start + 200]
    frame2_dets = [d for d in detections if frame2_start <= d[0] < frame2_start + 200]
    assert len(frame1_dets) >= 1, \
        f"No detection in frame 1 region [{frame1_start}, {frame1_start+200})"
    assert len(frame2_dets) >= 1, \
        f"No detection in frame 2 region [{frame2_start}, {frame2_start+200}). " \
        f"All detections at: {[d[0] for d in detections]}"

    dut._log.info(f"PASS: Both frames detected in HIL-mode continuous iq_valid.")


@cocotb.test()
async def test_detect_low_amplitude(dut):
    """Verify detection at low signal levels (OTA weak-STA scenario).

    The energy floor must not block detection for signals above RMS=16.
    This test sweeps amplitude from scale=20 (RMS~14) to scale=80 (RMS~57)
    and verifies detection fires for scale >= 25 (RMS~18, safely above
    the 2^14 energy floor which requires RMS >= 16).

    This is the regression gate for the energy floor fix: prevents the
    floor from being raised back to a level that blocks weak OTA signals.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())

    # Test at multiple amplitude levels
    # Normalized STF has RMS ≈ 0.1127, so scale=S gives ADC RMS ≈ 0.1127*S.
    # E_window = 64 * (0.1127*S)^2 = 0.813 * S^2
    # Floor at 2^14=16384 requires S >= 142.
    # Floor at 2^17=131072 required S >= 402 (the old broken floor).
    test_cases = [
        (6000, True, "full scale (default test level)"),
        (1000, True, "strong OTA signal"),
        (500, True, "typical STA at gain=50 (RMS~56)"),
        (300, True, "weak STA at gain=50 (RMS~34, was failing with 2^17 floor)"),
        (200, True, "minimum guaranteed detection (RMS~23, E~33k > 2^14)"),
        (100, False, "below floor — detection not required (RMS~11, E~8k < 2^14)"),
    ]

    results = []
    for scale, expect_detect, desc in test_cases:
        await reset(dut)

        re, im = load_6mbps_waveform_quantized(scale=scale)

        # Prepend 200 samples of low noise (RMS ~2, like real ADC thermal)
        random.seed(99)
        noise_len = 200
        noise_re = [random.randint(-3, 3) for _ in range(noise_len)]
        noise_im = [random.randint(-3, 3) for _ in range(noise_len)]

        full_re = noise_re + re[:400]
        full_im = noise_im + im[:400]

        detections, stf_ends = await feed_iq_stream(dut, full_re, full_im)

        detected = len(detections) >= 1
        results.append((scale, detected, expect_detect, desc))

        if expect_detect:
            assert detected, \
                f"FAILED: scale={scale} ({desc}) — expected detection but got none. " \
                f"Energy floor may be too high."
        # For scale=20 (below floor), we don't assert failure — it may or may not detect

        status = "PASS" if (detected == expect_detect) else (
            "OK (bonus)" if detected and not expect_detect else "FAIL")
        dut._log.info(f"  scale={scale:3d} ({desc}): detected={detected} {status}")

    dut._log.info("Low-amplitude sensitivity sweep complete.")
    for scale, detected, expected, desc in results:
        dut._log.info(f"  scale={scale}: detected={detected} (expected={expected})")


@cocotb.test()
async def test_detect_power_step_frames(dut):
    """Two frames at different amplitudes (AP strong, STA weak) → detect both.

    Models the real OTA failure: AP frame at high power followed by STA frame
    at -6dB (half amplitude), separated by a realistic noise gap.
    This exercises the energy floor AND the accumulator transition behavior.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    re_raw, im_raw = load_6mbps_waveform_quantized(scale=6000)

    # Frame 1: AP at full strength (scale 6000 → RMS~676)
    # Frame 2: STA at -6dB (scale 3000 → RMS~338)
    # The STA level (scale=3000, E=7.3M) is well above the floor,
    # but we also test a harder case: scale=300 (RMS~34, E=73k)
    # which was below the OLD floor (2^17=131072) but above the new (2^14=16384).
    ap_re = re_raw[:400]
    ap_im = im_raw[:400]

    re_weak, im_weak = load_6mbps_waveform_quantized(scale=300)
    sta_re = re_weak[:400]
    sta_im = im_weak[:400]

    # Gap: 320 samples (SIFS = 16μs at 20 MSPS) of thermal noise (RMS ~2)
    random.seed(77)
    gap_len = 320
    gap_re = [random.randint(-3, 3) for _ in range(gap_len)]
    gap_im = [random.randint(-3, 3) for _ in range(gap_len)]

    # Prefix noise
    pre_len = 200
    pre_re = [random.randint(-3, 3) for _ in range(pre_len)]
    pre_im = [random.randint(-3, 3) for _ in range(pre_len)]

    full_re = pre_re + ap_re + gap_re + sta_re
    full_im = pre_im + ap_im + gap_im + sta_im

    detections, stf_ends = await feed_iq_stream(dut, full_re, full_im)

    dut._log.info(f"Power-step burst: {len(detections)} detections")
    for i, (cyc, off) in enumerate(detections):
        dut._log.info(f"  Detection {i}: cycle={cyc}, offset={off}")

    # Must detect both frames
    ap_start = pre_len
    sta_start = pre_len + 400 + gap_len

    ap_dets = [d for d in detections if ap_start <= d[0] < ap_start + 300]
    sta_dets = [d for d in detections if sta_start <= d[0] < sta_start + 300]

    assert len(ap_dets) >= 1, \
        f"AP frame (scale=6000) not detected! Detections at: {[d[0] for d in detections]}"
    assert len(sta_dets) >= 1, \
        f"STA frame (scale=300) not detected after AP frame! " \
        f"Power-step rearm failure. Detections at: {[d[0] for d in detections]}. " \
        f"AP@{ap_start}, STA@{sta_start}"

    dut._log.info(f"PASS: Both AP (scale=6000) and STA (scale=300) detected.")
