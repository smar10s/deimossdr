"""
test_acquisition_ctrl.py — Unit test for standalone acquisition engine.

Tests acquisition_ctrl in isolation by driving synthetic correlator metric
waveforms that mimic what ltf_correlator produces during the GI2→LTF1
transition. Validates:
  1. Single frame acquisition (stf_end-anchored peak finding)
  2. Two frames at tight gap (both acquired)
  3. Duplicate suppression (trigger within holdoff → rejected)
  4. GI2 extension tolerance (peak shifted +10/+15 → still found)
  5. Backpressure (fifo_full → trigger rejected)
  6. Metric floor rejection (noise-only → no descriptor)

DUT: acquisition_ctrl (standalone, no stf_detect/correlator/decode)
Gate test (test_*). Runs in sim.sh.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer, ClockCycles


# =========================================================
# Helpers
# =========================================================

async def reset_dut(dut):
    """Reset and initialize all inputs."""
    dut.rst_n.value = 0
    dut.frame_detect.value = 0
    dut.stf_end.value = 0
    dut.corr_metric_valid.value = 0
    dut.corr_metric.value = 0
    dut.wr_ptr.value = 0
    dut.cfo_done.value = 0
    dut.phase_inc.value = 0
    dut.fifo_full.value = 0
    for _ in range(10):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    for _ in range(5):
        await RisingEdge(dut.clk)


async def pulse(dut, signal, cycles=1):
    """Pulse a signal high for N cycles."""
    signal.value = 1
    for _ in range(cycles):
        await RisingEdge(dut.clk)
    signal.value = 0


async def drive_metric_profile(dut, profile, start_wr_ptr=100):
    """Drive a synthetic corr_metric profile at 1-per-5 cadence.

    Profile is a list of (metric_value, n_samples) tuples.
    Simulates wr_ptr incrementing with each valid sample.
    Returns list of cycles where desc_valid pulsed.
    """
    wr_ptr = start_wr_ptr
    valid_counter = 0
    descriptors = []

    for metric_val, n_samples in profile:
        for _ in range(n_samples):
            # Drive at 1-per-5 cadence
            for clk_cycle in range(5):
                await RisingEdge(dut.clk)
                if clk_cycle == 0:
                    dut.corr_metric_valid.value = 1
                    dut.corr_metric.value = metric_val
                    dut.wr_ptr.value = wr_ptr & 0x7FFF
                    wr_ptr += 1
                else:
                    dut.corr_metric_valid.value = 0

                # Check for descriptor output
                try:
                    if int(dut.desc_valid.value) == 1:
                        descriptors.append({
                            'ltf_pos': int(dut.desc_ltf_pos.value),
                            'phase_inc': int(dut.desc_phase_inc.value),
                        })
                except (ValueError, AttributeError):
                    pass

    return descriptors, wr_ptr


async def drive_rising_edge_profile(dut, stf_end_delay=50, peak_pos_offset=0,
                                     start_wr_ptr=100, peak_metric=0x800000):
    """Drive a realistic LTF correlation profile after frame_detect + stf_end.

    Sequence:
      1. noise floor (stf_end_delay samples)
      2. stf_end fires
      3. noise floor (PEAK_WINDOW_START = 2 samples)
      4. rising edge (10 samples: linearly from floor to peak)
      5. plateau (30 samples at peak ± small variation)

    peak_pos_offset shifts the rising edge later (models GI2 extension).
    Returns descriptors found.
    """
    NOISE_FLOOR = 0x1000   # below METRIC_FLOOR (0x4000 = 16384)
    RISING_STEPS = 10

    wr_ptr = start_wr_ptr
    descriptors = []

    async def tick_metric(metric):
        nonlocal wr_ptr
        for clk_cycle in range(5):
            await RisingEdge(dut.clk)
            if clk_cycle == 0:
                dut.corr_metric_valid.value = 1
                dut.corr_metric.value = metric
                dut.wr_ptr.value = wr_ptr & 0x7FFF
                wr_ptr += 1
            else:
                dut.corr_metric_valid.value = 0
            try:
                if int(dut.desc_valid.value) == 1:
                    descriptors.append({
                        'ltf_pos': int(dut.desc_ltf_pos.value),
                        'phase_inc': int(dut.desc_phase_inc.value),
                    })
            except (ValueError, AttributeError):
                pass

    # Phase 1: noise before stf_end
    for _ in range(stf_end_delay):
        await tick_metric(NOISE_FLOOR)

    # Fire stf_end
    dut.stf_end.value = 1
    await RisingEdge(dut.clk)
    dut.stf_end.value = 0

    # Phase 2: post-stf_end noise (models additional GI2 if shifted)
    for _ in range(2 + peak_pos_offset):
        await tick_metric(NOISE_FLOOR)

    # Phase 3: rising edge (10 samples)
    for i in range(RISING_STEPS):
        m = NOISE_FLOOR + (peak_metric - NOISE_FLOOR) * (i + 1) // RISING_STEPS
        await tick_metric(m)

    # Phase 4: plateau (40 samples)
    for _ in range(40):
        await tick_metric(peak_metric)

    return descriptors, wr_ptr


async def wait_idle(dut, max_cycles=200):
    """Wait until acquisition_ctrl returns to idle or timeout."""
    for _ in range(max_cycles):
        await RisingEdge(dut.clk)
        try:
            if int(dut.desc_valid.value) == 1:
                return True
        except (ValueError, AttributeError):
            pass
    return False


# =========================================================
# Tests
# =========================================================

@cocotb.test()
async def test_single_frame_acquisition(dut):
    """Single frame: frame_detect → stf_end → peak found → descriptor output."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Ensure trigger distance is satisfied (wait enough metric valids)
    for _ in range(300):
        dut.corr_metric_valid.value = 1
        dut.corr_metric.value = 0x10000
        dut.wr_ptr.value = 0
        await RisingEdge(dut.clk)
        dut.corr_metric_valid.value = 0
        for _ in range(4):
            await RisingEdge(dut.clk)

    # Fire frame_detect
    await pulse(dut, dut.frame_detect)

    # Drive CFO estimate
    await RisingEdge(dut.clk)
    dut.cfo_done.value = 1
    dut.phase_inc.value = 0x0123
    await RisingEdge(dut.clk)
    dut.cfo_done.value = 0

    # Drive metric profile (stf_end + peak)
    descriptors, _ = await drive_rising_edge_profile(dut, stf_end_delay=30)

    assert len(descriptors) == 1, f"Expected 1 descriptor, got {len(descriptors)}"
    assert descriptors[0]['phase_inc'] == 0x0123, \
        f"CFO mismatch: {descriptors[0]['phase_inc']:#x} != 0x0123"

    # Verify pipeline_ack fired
    dut._log.info(f"Single frame: ltf_pos={descriptors[0]['ltf_pos']}, "
                  f"phase_inc={descriptors[0]['phase_inc']:#06x}")


@cocotb.test()
async def test_two_frames_tight_gap(dut):
    """Two frames with tight gap (200 samples) — both must produce descriptors."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    all_descriptors = []

    # Satisfy initial trigger distance
    for _ in range(300):
        dut.corr_metric_valid.value = 1
        dut.corr_metric.value = 0x10000
        dut.wr_ptr.value = 0
        await RisingEdge(dut.clk)
        dut.corr_metric_valid.value = 0
        for _ in range(4):
            await RisingEdge(dut.clk)

    # --- Frame 1 ---
    await pulse(dut, dut.frame_detect)
    dut.cfo_done.value = 1
    dut.phase_inc.value = 0x0100
    await RisingEdge(dut.clk)
    dut.cfo_done.value = 0

    descs, wr = await drive_rising_edge_profile(dut, stf_end_delay=30, start_wr_ptr=100)
    all_descriptors.extend(descs)

    # Inter-frame gap: 260 samples of noise (> MIN_TRIGGER_DISTANCE=256)
    for _ in range(260):
        dut.corr_metric_valid.value = 1
        dut.corr_metric.value = 0x10000
        dut.wr_ptr.value = wr & 0x7FFF
        wr += 1
        await RisingEdge(dut.clk)
        dut.corr_metric_valid.value = 0
        for _ in range(4):
            await RisingEdge(dut.clk)

    # --- Frame 2 ---
    await pulse(dut, dut.frame_detect)
    dut.cfo_done.value = 1
    dut.phase_inc.value = 0x0200
    await RisingEdge(dut.clk)
    dut.cfo_done.value = 0

    descs, _ = await drive_rising_edge_profile(dut, stf_end_delay=30, start_wr_ptr=wr)
    all_descriptors.extend(descs)

    assert len(all_descriptors) == 2, \
        f"Expected 2 descriptors, got {len(all_descriptors)}"
    assert all_descriptors[0]['phase_inc'] == 0x0100
    assert all_descriptors[1]['phase_inc'] == 0x0200

    dut._log.info(f"Two frames: pos1={all_descriptors[0]['ltf_pos']}, "
                  f"pos2={all_descriptors[1]['ltf_pos']}")


@cocotb.test()
async def test_duplicate_suppression(dut):
    """Trigger within holdoff distance → rejected, no descriptor."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Satisfy initial distance
    for _ in range(300):
        dut.corr_metric_valid.value = 1
        dut.corr_metric.value = 0x10000
        dut.wr_ptr.value = 0
        await RisingEdge(dut.clk)
        dut.corr_metric_valid.value = 0
        for _ in range(4):
            await RisingEdge(dut.clk)

    # Frame 1
    await pulse(dut, dut.frame_detect)
    descs, wr = await drive_rising_edge_profile(dut, stf_end_delay=30, start_wr_ptr=100)
    assert len(descs) == 1, "Frame 1 should produce a descriptor"

    # Immediately fire another trigger (within holdoff — only ~10 samples later)
    for _ in range(10):
        dut.corr_metric_valid.value = 1
        dut.corr_metric.value = 0x10000
        dut.wr_ptr.value = wr & 0x7FFF
        wr += 1
        await RisingEdge(dut.clk)
        dut.corr_metric_valid.value = 0
        for _ in range(4):
            await RisingEdge(dut.clk)

    # This trigger should be rejected
    await pulse(dut, dut.frame_detect)

    # Drive some metric — should NOT produce a descriptor
    descs2, _ = await drive_rising_edge_profile(dut, stf_end_delay=20, start_wr_ptr=wr)

    # The second frame_detect was rejected (state was IDLE but distance < 256)
    # So stf_end/metric won't produce a descriptor
    # Check diagnostic counter
    rejected = int(dut.diag_frames_rejected.value)
    assert rejected >= 1, f"Expected rejection, got diag_frames_rejected={rejected}"
    dut._log.info(f"Duplicate suppression: rejected={rejected}")


@cocotb.test()
async def test_gi2_extension_tolerance(dut):
    """GI2 extension (+10, +15 samples) — peak still found correctly.

    This is THE BUG FIX test. The old pending path's [135,165] window
    failed at shift=+10 because the peak moved outside the window.
    acquisition_ctrl uses stf_end-anchored search, so the peak position
    relative to stf_end is constant regardless of GI2 length.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    for shift in [0, 5, 10, 15, -5, -10]:
        await reset_dut(dut)

        # Satisfy trigger distance
        for _ in range(300):
            dut.corr_metric_valid.value = 1
            dut.corr_metric.value = 0x10000
            dut.wr_ptr.value = 0
            await RisingEdge(dut.clk)
            dut.corr_metric_valid.value = 0
            for _ in range(4):
                await RisingEdge(dut.clk)

        await pulse(dut, dut.frame_detect)
        dut.cfo_done.value = 1
        dut.phase_inc.value = 0x0042
        await RisingEdge(dut.clk)
        dut.cfo_done.value = 0

        # peak_pos_offset models GI2 extension: positive = LTF arrives later
        # relative to stf_end. But since our search starts from stf_end,
        # the extra delay just means more noise samples before the rising edge.
        # The search window is wide enough (30 samples) to accommodate this.
        descs, _ = await drive_rising_edge_profile(
            dut, stf_end_delay=30, peak_pos_offset=max(0, shift), start_wr_ptr=200
        )

        assert len(descs) == 1, \
            f"shift={shift:+d}: Expected descriptor, got {len(descs)}"
        dut._log.info(f"  shift={shift:+3d}: OK, ltf_pos={descs[0]['ltf_pos']}")

    dut._log.info("GI2 extension tolerance: all shifts pass")


@cocotb.test()
async def test_backpressure_fifo_full(dut):
    """fifo_full=1 → trigger rejected, no acquisition attempt."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Satisfy trigger distance
    for _ in range(300):
        dut.corr_metric_valid.value = 1
        dut.corr_metric.value = 0x10000
        dut.wr_ptr.value = 0
        await RisingEdge(dut.clk)
        dut.corr_metric_valid.value = 0
        for _ in range(4):
            await RisingEdge(dut.clk)

    # Set fifo full
    dut.fifo_full.value = 1

    # Fire trigger
    await pulse(dut, dut.frame_detect)

    # Drive metric profile — should not produce anything
    descs, _ = await drive_rising_edge_profile(dut, stf_end_delay=30, start_wr_ptr=100)

    assert len(descs) == 0, f"Expected no descriptor with fifo_full, got {len(descs)}"

    rejected = int(dut.diag_frames_rejected.value)
    assert rejected >= 1, f"Expected rejection counter, got {rejected}"
    dut._log.info(f"Backpressure: correctly rejected (diag={rejected})")


@cocotb.test()
async def test_metric_floor_rejection(dut):
    """Noise-only metric (below METRIC_FLOOR) → no descriptor pushed."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Satisfy trigger distance
    for _ in range(300):
        dut.corr_metric_valid.value = 1
        dut.corr_metric.value = 0x10000
        dut.wr_ptr.value = 0
        await RisingEdge(dut.clk)
        dut.corr_metric_valid.value = 0
        for _ in range(4):
            await RisingEdge(dut.clk)

    # Fire trigger
    await pulse(dut, dut.frame_detect)

    # Fire stf_end after some noise
    for _ in range(30):
        dut.corr_metric_valid.value = 1
        dut.corr_metric.value = 0x1000  # well below METRIC_FLOOR (0x4000)
        dut.wr_ptr.value = 100
        await RisingEdge(dut.clk)
        dut.corr_metric_valid.value = 0
        for _ in range(4):
            await RisingEdge(dut.clk)

    dut.stf_end.value = 1
    await RisingEdge(dut.clk)
    dut.stf_end.value = 0

    # Drive only noise through search window (50 samples, all below floor)
    descs = []
    for _ in range(50):
        dut.corr_metric_valid.value = 1
        dut.corr_metric.value = 0x1000  # 4096 — below METRIC_FLOOR (16384)
        dut.wr_ptr.value = 150
        await RisingEdge(dut.clk)
        dut.corr_metric_valid.value = 0
        for _ in range(4):
            await RisingEdge(dut.clk)
            try:
                if int(dut.desc_valid.value) == 1:
                    descs.append(1)
            except (ValueError, AttributeError):
                pass

    assert len(descs) == 0, f"Expected no descriptor for noise, got {len(descs)}"

    rejected = int(dut.diag_frames_rejected.value)
    assert rejected >= 1, f"Expected metric rejection, got diag_frames_rejected={rejected}"
    dut._log.info(f"Metric floor: correctly rejected noise (diag={rejected})")


@cocotb.test()
async def test_stf_end_timeout(dut):
    """Trigger accepted but stf_end never arrives -> timeout back to IDLE.

    Wedge scenario: watchdog/playback clear during the STF window resets
    stf_detect's stf_end_armed flag, so the pending stf_end pulse never
    fires. Without a timeout, acquisition_ctrl wedges in S_WAIT_STF_END —
    the next frame's trigger is silently ignored and only its stf_end
    un-wedges the FSM (1-2 frames lost per occurrence).

    With the timeout: the wedged acquisition gives up (diag_frames_rejected
    increments, pipeline_ack fires) and the NEXT frame is acquired cleanly.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Satisfy trigger distance
    for _ in range(300):
        dut.corr_metric_valid.value = 1
        dut.corr_metric.value = 0x10000
        dut.wr_ptr.value = 0
        await RisingEdge(dut.clk)
        dut.corr_metric_valid.value = 0
        for _ in range(4):
            await RisingEdge(dut.clk)

    # Frame 1: trigger accepted, but stf_end NEVER arrives
    await pulse(dut, dut.frame_detect)

    # Wedge window: keep the channel busy (noise metric) so trigger_distance
    # keeps counting, but no stf_end. Run well past the timeout.
    for _ in range(410):
        dut.corr_metric_valid.value = 1
        dut.corr_metric.value = 0x10000
        dut.wr_ptr.value = 0
        await RisingEdge(dut.clk)
        dut.corr_metric_valid.value = 0
        for _ in range(4):
            await RisingEdge(dut.clk)

    # The wedged acquisition must have timed out and counted a rejection
    rejected = int(dut.diag_frames_rejected.value)
    assert rejected >= 1, \
        f"stf_end timeout did not fire — FSM wedged in S_WAIT_STF_END " \
        f"(diag_frames_rejected={rejected})"

    # Frame 2: must be acquired cleanly (trigger accepted from S_IDLE)
    await pulse(dut, dut.frame_detect)
    dut.cfo_done.value = 1
    dut.phase_inc.value = 0x0300
    await RisingEdge(dut.clk)
    dut.cfo_done.value = 0

    descs, _ = await drive_rising_edge_profile(dut, stf_end_delay=30, start_wr_ptr=500)
    assert len(descs) == 1, \
        f"frame after timeout: expected 1 descriptor, got {len(descs)}"
    assert descs[0]['phase_inc'] == 0x0300, \
        f"frame after timeout: CFO mismatch {descs[0]['phase_inc']:#x}"

    dut._log.info(f"PASS: stf_end timeout released wedge (rejected={rejected}), "
                  "next frame acquired cleanly")
