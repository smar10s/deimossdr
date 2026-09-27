"""
diag_hil_burst_replay.py — Replay a captured HIL burst waveform through the
rx_frontend sim DUT and report every decoded tag (rate/length/fcs).

DIAGNOSTIC (never asserts). Used to characterize intermittent HIL layer-6b
failures: replay the exact waveform the hardware played (dumped via
deimos_burst_loopback --dump-hil) and compare sim tags vs hardware tags.

Usage:
  REPLAY_FILE=/tmp/simreplay/fail_seed4748.json \
    make sim SIM_BUILD=sim_build_rx_frontend TOPLEVEL=rx_frontend \
    VERILOG_SOURCES="$(make -s print_rx_frontend_srcs)" \
    COCOTB_TEST_MODULES=diag_hil_burst_replay
"""
import json
import os

import cocotb
import numpy as np
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from frontend_helpers import reset_dut, load_adc_capture, s12_to_unsigned


def apply_hil_noise_floor(samples, noise_fraction=0.02, seed=1):
    """Emulate deimos_burst_loopback --file --hil waveform preprocessing.

    Hardware replaces near-silence samples with noise at `noise_fraction`
    of signal peak so the STF correlator's energy denominator can reset
    between frames (deimos_burst_loopback.c:971-977). The JSON vectors
    contain exact zeros in pre-pad/gap regions; hardware never plays
    those. Returns list of (re, im) integer tuples.
    """
    re = np.array([s[0] for s in samples], dtype=np.float64)
    im = np.array([s[1] for s in samples], dtype=np.float64)
    peak = max(np.abs(re).max(), np.abs(im).max())
    noise_level = peak * noise_fraction
    rng = np.random.default_rng(seed)
    near_silence = (np.abs(re) < noise_level) & (np.abs(im) < noise_level)
    re[near_silence] = noise_level * rng.uniform(-1.0, 1.0, near_silence.sum())
    im[near_silence] = noise_level * rng.uniform(-1.0, 1.0, near_silence.sum())
    return list(zip(re.round().astype(int).tolist(), im.round().astype(int).tolist()))


def load_hil_waveform(path, noise_fraction=0.02, seed=1):
    """Load a gen_impaired_burst JSON exactly as hardware plays it.

    The JSON 'real'/'imag' arrays are 12-bit integers quantized at 90% FS
    by gen_impaired_burst.py. deimos_burst_loopback --file --hil re-scales
    to 90% FS (≈identity for these arrays) and adds a noise floor to
    near-silence samples. Sim feeds the integers directly — bit-identical
    to hardware (modulo noise RNG values).
    """
    with open(path) as f:
        data = json.load(f)
    samples = list(zip(data['real'], data['imag']))
    if noise_fraction > 0:
        samples = apply_hil_noise_floor(samples, noise_fraction, seed)
    return samples


@cocotb.test()
async def replay_hil_burst(dut):
    path = os.environ.get("REPLAY_FILE")
    if not path:
        dut._log.warning("REPLAY_FILE not set — skipping replay")
        return
    with open(path) as f:
        meta = json.load(f)

    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Diagnostic hook: preset the cfo_mixer NCO phase accumulator to model
    # hardware's unreset NCO (BD leaves phase_reset floating). The
    # correlator metric |Re|+|Im| is rotation-sensitive, so peak position
    # depends on this residue.
    nco_deg = os.environ.get("HIL_NCO_PHASE_DEG")
    if nco_deg is not None:
        try:
            phase_acc = int(round(float(nco_deg) / 360.0 * 65536)) & 0xFFFF
            dut.u_rx_pipeline.u_cfo_mixer.phase_acc.value = phase_acc
            dut._log.info(f"  NCO phase preset: {nco_deg} deg "
                          f"(phase_acc=0x{phase_acc:04X})")
        except (ValueError, AttributeError) as e:
            dut._log.warning(f"  NCO phase preset failed: {e}")

    if os.environ.get("HIL_WAVEFORM", "1") == "1":
        seed = int(os.environ.get("HIL_NOISE_SEED", "1"))
        samples = load_hil_waveform(path, seed=seed)
        dut._log.info(f"  Waveform: firmware-exact (JSON ints + 2% noise floor, seed={seed})")
    else:
        samples = load_adc_capture(path)
        if os.environ.get("HIL_NOISE_FLOOR", "1") == "1":
            samples = apply_hil_noise_floor(samples)
            dut._log.info("  HIL noise floor: ON (2% of peak in silence regions)")
        else:
            dut._log.info("  HIL noise floor: OFF (raw JSON zeros)")
    n_samples = len(samples)
    dut._log.info(f"Replaying {path}: {n_samples} samples")

    tags = []
    acq_events = []
    sample_idx = 0
    valid_counter = 0
    post_zero_count = 8192
    timeout_cycles = (n_samples + post_zero_count) * 5 + 2000000

    pending_acq = {}
    last_desc = {}
    VERBOSE = os.environ.get("HIL_REPLAY_VERBOSE", "0") == "1"

    def _sign16(v):
        v = int(v)
        return v - 65536 if v >= 32768 else v

    for cycle in range(timeout_cycles):
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

        # Acquisition instrumentation: frame_detect, stf_end, cfo_done,
        # and acquisition_ctrl descriptor (desc_ltf_pos / desc_phase_inc).
        try:
            if int(dut.frame_detect.value) == 1:
                pending_acq = {'fd_sample': sample_idx}
                if VERBOSE:
                    dut._log.info(f"  [acq] frame_detect @ cycle {cycle} sample ~{sample_idx}")
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.stf_end.value) == 1 and 'se_sample' not in pending_acq:
                pending_acq['se_sample'] = sample_idx
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.cfo_done.value) == 1:
                pending_acq['phase_inc'] = _sign16(dut.phase_inc.value)
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.u_rx_pipeline.u_acquisition_ctrl.desc_valid.value) == 1:
                last_desc = {
                    'ltf_pos': int(dut.u_rx_pipeline.u_acquisition_ctrl.desc_ltf_pos.value),
                    'phase_inc': _sign16(dut.u_rx_pipeline.u_acquisition_ctrl.desc_phase_inc.value),
                    'max_metric': int(dut.u_rx_pipeline.u_acquisition_ctrl.max_metric.value),
                }
                pending_acq.update(last_desc)
                acq_events.append(dict(pending_acq))
                if VERBOSE:
                    dut._log.info(
                        f"  [acq] desc: ltf_pos={last_desc['ltf_pos']} "
                        f"phase_inc={last_desc['phase_inc']} "
                        f"max_metric={last_desc['max_metric']} "
                        f"fd={pending_acq.get('fd_sample')} "
                        f"stf_end={pending_acq.get('se_sample')}")
        except (ValueError, AttributeError):
            pass

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

        if sample_idx >= n_samples + post_zero_count:
            break

    if VERBOSE:
        dut._log.info("ACQ EVENTS (fd_sample, stf_end_sample, phase_inc, ltf_pos, max_metric):")
        for i, ev in enumerate(acq_events):
            dut._log.info(
                f"  acq[{i}]: fd={ev.get('fd_sample')} stf_end={ev.get('se_sample')} "
                f"phase_inc={ev.get('phase_inc')} ltf_pos={ev.get('ltf_pos')} "
                f"max_metric={ev.get('max_metric')}")

    fcs_ok = [t for t in tags if t['fcs_ok']]
    fcs_fail = [t for t in tags if not t['fcs_ok']]
    dut._log.info(f"REPLAY SUMMARY: {len(tags)} tags, "
                  f"{len(fcs_ok)} FCS OK, {len(fcs_fail)} FCS FAIL")
    for i, t in enumerate(fcs_fail):
        dut._log.info(f"  FAIL tag {i}: rate=0b{t['rate']:04b} "
                      f"len={t['length']} @ sample ~{t['sample']}")
