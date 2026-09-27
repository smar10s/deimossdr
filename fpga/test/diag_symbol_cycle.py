"""
diag_symbol_cycle.py — Full-pipeline symbol-to-symbol cycle measurement.

THE key metric for latency budget work. Measures how many clock cycles
elapse between consecutive symbol_start events in the full rx_frontend
pipeline (STF → CFO → FFT → EQ → pilot → demod → Viterbi → FCS).

Context:
  - 802.11a symbol period: 4 μs = 400 clocks at 100 MHz
  - Current measured: ~700 clocks/symbol (causes buffer accumulation)
  - Target after FFT overlap: ≤ 400 clocks (zero net accumulation)

When the ≤400 target is achieved, this diagnostic gets promoted to
test_symbol_cycle with a hard assertion.

Reports:
  - Per-symbol cycle count (symbol_start[N+1] - symbol_start[N])
  - Per-stage breakdown within each symbol
  - Buffer level proxy (cumulative excess over 400-clock budget)
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer

from frontend_helpers import (
    load_waveform_float, quantize_12bit, s12_to_unsigned, reset_dut
)


@cocotb.test()
async def diag_symbol_cycle_rate6(dut):
    """Full-pipeline symbol cycle measurement at rate 6 (BPSK, most DATA symbols)."""
    await _run_symbol_cycle(dut, rate=6)


@cocotb.test()
async def diag_symbol_cycle_rate24(dut):
    """Full-pipeline symbol cycle measurement at rate 24 (16-QAM)."""
    await _run_symbol_cycle(dut, rate=24)


@cocotb.test()
async def diag_symbol_cycle_rate12(dut):
    """Full-pipeline symbol cycle measurement at rate 12 (QPSK)."""
    await _run_symbol_cycle(dut, rate=12)


async def _run_symbol_cycle(dut, rate):
    """Core measurement: feed golden vector, instrument symbol-to-symbol timing."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    iq_float = load_waveform_float(rate)
    iq_samples = quantize_12bit(iq_float)
    n_samples = len(iq_samples)

    dut._log.info("=" * 70)
    dut._log.info(f"SYMBOL CYCLE DIAGNOSTIC — Rate {rate} Mbps")
    dut._log.info(f"  Waveform: {n_samples} IQ samples")
    dut._log.info(f"  Budget target: 400 clocks/symbol (zero net accumulation)")
    dut._log.info("=" * 70)

    # Timing collectors
    symbol_start_events = []   # (cycle, symbol_idx)
    eq_valid_events = []       # (cycle, symbol_idx) — first eq output per symbol
    deint_done_events = []     # (cycle, symbol_idx) — per-symbol compute-window end
    tag_cycle = None

    # State tracking
    current_symbol_idx = -1
    eq_seen_this_sym = False
    prev_sym_start_seen = False

    # Feed IQ samples (1-per-5 valid to match live mode)
    sample_idx = 0
    VALID_SPACING = 5

    for cycle in range(500000):
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

        # Monitor symbol_start_out (from decode_engine inside rx_pipeline)
        try:
            sym_start = int(dut.u_rx_pipeline.u_decode_engine.symbol_start_out.value)
            sym_idx = int(dut.u_rx_pipeline.u_decode_engine.symbol_idx_out.value)
            if sym_start:
                symbol_start_events.append((cycle, sym_idx))
                current_symbol_idx = sym_idx
                eq_seen_this_sym = False
        except (ValueError, AttributeError):
            pass

        # Monitor eq output
        try:
            eq_dv = int(dut.u_rx_pipeline.u_equalizer.data_valid.value)
            if eq_dv and not eq_seen_this_sym:
                eq_valid_events.append((cycle, current_symbol_idx))
                eq_seen_this_sym = True
        except (ValueError, AttributeError):
            pass

        # Monitor deint_done — the end of a symbol's decode work. The
        # symbol_start -> deint_done window is the true, feed-independent
        # per-symbol compute cost (the symbol_start *period* is clamped at
        # 400 once the pipeline is faster than the live feed).
        try:
            if int(dut.u_rx_pipeline.deint_done.value) == 1:
                deint_done_events.append((cycle, current_symbol_idx))
        except (ValueError, AttributeError):
            pass

        # Check for tag (end of decode)
        try:
            if int(dut.tag_valid.value) == 1:
                tag_cycle = cycle
                break
        except (ValueError, AttributeError):
            pass

    # =========================================================
    # Analysis
    # =========================================================

    if not symbol_start_events:
        dut._log.error("  NO symbol_start events detected!")
        return

    dut._log.info(" ")
    dut._log.info(f"  Frame detected. Tag at cycle {tag_cycle}.")
    dut._log.info(f"  Symbol starts: {len(symbol_start_events)}")
    dut._log.info(" ")

    # Per-symbol cycle gaps
    dut._log.info("Per-symbol timing (symbol_start[N+1] - symbol_start[N]):")
    dut._log.info(f"  {'Sym':>4} {'Cycle':>7} {'Delta':>6} {'EQ→':>5} "
                  f"{'Accum':>6}")
    dut._log.info(f"  {'----':>4} {'-----':>7} {'-----':>6} {'---':>5} "
                  f"{'-----':>6}")

    cumulative_excess = 0
    symbol_deltas = []

    for i, (cyc, idx) in enumerate(symbol_start_events):
        delta = None
        if i > 0:
            delta = cyc - symbol_start_events[i-1][0]
            excess = delta - 400
            cumulative_excess += excess
            symbol_deltas.append(delta)

        # Find matching eq_valid
        eq_delta = None
        for (ec, ei) in eq_valid_events:
            if ei == idx and ec >= cyc:
                eq_delta = ec - cyc
                break

        sym_type = "SIG" if idx == 0 else f"D{idx}"
        delta_str = str(delta) if delta else "-"
        eq_str = str(eq_delta) if eq_delta else "-"
        accum_str = f"{cumulative_excess:+d}" if i > 0 else "-"

        dut._log.info(f"  {sym_type:>4} {cyc:>7} {delta_str:>6} "
                      f"{eq_str:>5} {accum_str:>6}")

    # Summary
    if symbol_deltas:
        # Separate SIGNAL→DATA1 from DATA→DATA (first delta includes LTF processing)
        data_deltas = symbol_deltas[1:] if len(symbol_deltas) > 1 else symbol_deltas

        dut._log.info(" ")
        dut._log.info("SUMMARY (DATA symbols only, excluding SIGNAL→DATA1):")
        if data_deltas:
            avg = sum(data_deltas) / len(data_deltas)
            dut._log.info(f"  Average: {avg:.1f} clocks/symbol")
            dut._log.info(f"  Min: {min(data_deltas)}, Max: {max(data_deltas)}")
            dut._log.info(f"  Target: 400 clocks")
            dut._log.info(f"  Over budget by: {avg - 400:.0f} clocks/symbol (average)")
            dut._log.info(f"  Cumulative excess after {len(data_deltas)} DATA symbols: "
                          f"{cumulative_excess:+d} clocks")
            dut._log.info(f"  Buffer entries accumulated: ~{cumulative_excess // 5}")
            dut._log.info(" ")
            if avg <= 400:
                dut._log.info("  *** TARGET MET — ready to promote to test_symbol_cycle ***")
            else:
                dut._log.info(f"  OVER BUDGET — FFT overlap needed to reduce by "
                              f"~{avg - 400:.0f} clocks")

    # =========================================================
    # Feed-independent capability: symbol_start -> deint_done
    # =========================================================
    # Once the pipeline is faster than the live feed, the symbol_start
    # *period* above is clamped at ~400 and hides the true cost. This window
    # is measured entirely inside one symbol's decode, so it is unaffected by
    # feed spacing.
    cap_windows = []
    for (scyc, sidx) in symbol_start_events:
        for (dcyc, didx) in deint_done_events:
            if didx == sidx and dcyc >= scyc:
                cap_windows.append(dcyc - scyc)
                break

    if len(cap_windows) > 1:
        data_caps = cap_windows[1:]  # exclude SIGNAL→DATA1
        cap_avg = sum(data_caps) / len(data_caps)
        dut._log.info(" ")
        dut._log.info("COMPUTE WINDOW (deint_done - symbol_start, feed-independent):")
        dut._log.info(f"  Average: {cap_avg:.1f} clocks/symbol")
        dut._log.info(f"  Min: {min(data_caps)}, Max: {max(data_caps)}")
        if cap_avg <= 400:
            dut._log.info("  *** CAPABILITY MEETS WIRE SPEED (<=400) ***")
        else:
            dut._log.info(f"  Over budget by: {cap_avg - 400:.0f} clocks/symbol")
