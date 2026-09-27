"""
test_hil_burst_replay.py — Gate test: hardware-exact replay of the layer-8
multipath_mod burst (the exact vector hil_test.sh layer 8 injects) must
decode >= 5/6 frames with FCS OK and 0 FCS fail — the same gate as hardware.

Regression guard for the rotation-sensitive |Re|+|Im| correlator metric:
with the NCO phase at 0 (hardware-deterministic after the BD phase_reset
tie-down), multipath_mod previously produced 3/6. The squared-magnitude
metric makes peak selection rotation-invariant.

Self-contained: generates the burst in-memory via gen_impaired_burst
(FRAME_SPECS, seed 42, MULTIPATH_PRESETS['moderate']), quantized 90% FS
exactly as hardware plays it, with the 2% noise floor in silence regions.
"""
import os
import sys

import cocotb
import numpy as np
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from frontend_helpers import reset_dut, s12_to_unsigned
from diag_hil_burst_replay import apply_hil_noise_floor

SCRIPTS_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "scripts")
sys.path.insert(0, os.path.abspath(SCRIPTS_DIR))

from gen_impaired_burst import build_burst, FRAME_SPECS, SCENARIOS  # noqa: E402
from py80211.impairments import apply_multipath, MULTIPATH_PRESETS  # noqa: E402


def build_scenario_waveform(scenario_name):
    """Build + impair + quantize one layer-8 scenario in-memory, exactly
    as hardware plays it (90% FS, 2% noise floor in silence)."""
    cfg = SCENARIOS[scenario_name]
    burst_iq, expected_frames = build_burst(cfg["specs"])
    impaired = cfg["impairments"](burst_iq.copy())

    re = np.real(impaired).astype(np.float64)
    im = np.imag(impaired).astype(np.float64)
    peak = max(np.abs(re).max(), np.abs(im).max())
    scale = (2047.0 * 0.9) / peak if peak > 0 else 1.0
    re_q = np.clip(np.round(re * scale), -2048, 2047).astype(int)
    im_q = np.clip(np.round(im * scale), -2048, 2047).astype(int)

    samples = list(zip(re_q.tolist(), im_q.tolist()))
    samples = apply_hil_noise_floor(samples, noise_fraction=0.02, seed=1)
    return samples, expected_frames


def build_multipath_mod_waveform():
    """Back-compat: the layer-8 multipath_mod burst."""
    return build_scenario_waveform("multipath_mod")


async def run_scenario_replay(dut, scenario_name):
    """Feed one impaired burst through the DUT; gate = hardware layer 8
    criterion (6 tags, 0 FCS fail, >=5 FCS OK)."""
    samples, expected_frames = build_scenario_waveform(scenario_name)
    n_samples = len(samples)
    dut._log.info(f"Replaying layer-8 {scenario_name} burst: "
                  f"{n_samples} samples, {len(expected_frames)} frames")

    tags = []
    sample_idx = 0
    valid_counter = 0
    post_zero_count = 8192
    timeout_cycles = (n_samples + post_zero_count) * 5 + 2000000

    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)

        if valid_counter == 0:
            if sample_idx < n_samples:
                re_q, im_q = samples[sample_idx]
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

        if int(dut.tag_valid.value) == 1:
            tag = {
                "rate": int(dut.tag_rate.value),
                "length": int(dut.tag_length.value),
                "fcs_ok": int(dut.tag_fcs_ok.value),
                "sample": sample_idx,
            }
            tags.append(tag)
            fcs_str = "OK" if tag["fcs_ok"] else "FAIL"
            dut._log.info(f"  Frame {len(tags)}: rate=0b{tag['rate']:04b}, "
                          f"len={tag['length']}, fcs={fcs_str} @ sample ~{sample_idx}")

        if sample_idx >= n_samples + post_zero_count:
            break

    fcs_ok = [t for t in tags if t["fcs_ok"]]
    fcs_fail = [t for t in tags if not t["fcs_ok"]]
    dut._log.info(f"REPLAY SUMMARY: {len(tags)} tags, "
                  f"{len(fcs_ok)} FCS OK, {len(fcs_fail)} FCS FAIL")

    assert len(tags) == 6, f"Expected 6 tags, got {len(tags)}"
    assert len(fcs_fail) == 0, f"{len(fcs_fail)} FCS FAIL: {fcs_fail}"
    assert len(fcs_ok) >= 5, f"Only {len(fcs_ok)}/6 FCS OK (gate: >=5)"


@cocotb.test()
async def test_multipath_mod_burst(dut):
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    await run_scenario_replay(dut, "multipath_mod")


@cocotb.test()
async def test_multipath_mod_54m_burst(dut):
    """The layer-8 multipath_mod scenario at 54 Mbps — closes the gap
    that no 48/54M frame existed in the impairment gates."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    await run_scenario_replay(dut, "multipath_mod_54m")


@cocotb.test()
async def test_combined_54m_burst(dut):
    """CFO 3kHz + SFO 8ppm + multipath + 30 dB AWGN at 54 Mbps."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    await run_scenario_replay(dut, "combined_54m")


async def run_single_rate_burst(dut, rate_mbps):
    """Six impaired frames at one rate, all six must decode.

    Holding the rate fixed across a burst is what exposes puncture-pattern
    state that survives a frame boundary. The mixed-rate scenarios above
    hide it: they are all code rate 1/2, where the depuncturer is a
    passthrough and has no pattern position to carry over.

    Uses the same payload mix and impairment as multipath_mod so the only
    variable against that scenario is the rate.
    """
    specs = [(rate_mbps, n) for n in (137, 14, 193, 100, 14, 371)]
    burst_iq, _ = build_burst(specs)
    impaired = apply_multipath(burst_iq.copy(), MULTIPATH_PRESETS["moderate"])

    re = np.real(impaired).astype(np.float64)
    im = np.imag(impaired).astype(np.float64)
    peak = max(np.abs(re).max(), np.abs(im).max())
    scale = (2047.0 * 0.9) / peak if peak > 0 else 1.0
    re_q = np.clip(np.round(re * scale), -2048, 2047).astype(int)
    im_q = np.clip(np.round(im * scale), -2048, 2047).astype(int)
    samples = list(zip(re_q.tolist(), im_q.tolist()))
    samples = apply_hil_noise_floor(samples, noise_fraction=0.02, seed=1)

    await feed_burst(dut, samples, rate_mbps)


async def feed_burst(dut, samples, rate_mbps):
    n_samples = len(samples)
    tags = []
    sample_idx = 0
    valid_counter = 0
    post_zero_count = 8192

    for _ in range((n_samples + post_zero_count) * 5 + 2000000):
        await RisingEdge(dut.clk)

        if valid_counter == 0:
            if sample_idx < n_samples:
                re_q, im_q = samples[sample_idx]
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

        if int(dut.tag_valid.value) == 1:
            tags.append({
                "length": int(dut.tag_length.value),
                "fcs_ok": int(dut.tag_fcs_ok.value),
            })
            ok = tags[-1]["fcs_ok"]
            dut._log.info(f"  Frame {len(tags)}: len={tags[-1]['length']} "
                          f"fcs={'OK' if ok else 'FAIL'}")

        if sample_idx >= n_samples + post_zero_count:
            break

    pattern = "".join("O" if t["fcs_ok"] else "X" for t in tags)
    n_ok = sum(t["fcs_ok"] for t in tags)
    dut._log.info(f"RATE {rate_mbps}M: {len(tags)} tags, {n_ok} OK "
                  f"pattern={pattern}")

    assert len(tags) == 6, f"rate {rate_mbps}M: expected 6 tags, got {len(tags)}"
    assert n_ok == 6, (
        f"rate {rate_mbps}M: {n_ok}/6 FCS OK (pattern {pattern}). "
        f"A first-frame-only pass means per-frame decode state leaked."
    )


@cocotb.test()
async def test_single_rate_burst_36m(dut):
    """Six 36 Mbps frames — code rate 3/4, 16-QAM."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    await run_single_rate_burst(dut, 36)


@cocotb.test()
async def test_single_rate_burst_48m(dut):
    """Six 48 Mbps frames — code rate 2/3, the only rate using that pattern."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    await run_single_rate_burst(dut, 48)


@cocotb.test()
async def test_single_rate_burst_54m(dut):
    """Six 54 Mbps frames — code rate 3/4 at the tightest symbol budget."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    await run_single_rate_burst(dut, 54)
