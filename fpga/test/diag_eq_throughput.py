"""
diag_eq_throughput.py — Equalizer per-symbol cycle budget diagnostic.

Measures clock cycles from the first fft_valid input to the last data_valid
output for each OFDM symbol. The equalizer receives 64 FFT bins and emits
48 data subcarriers + 4 pilots.

Pipeline: bin arrives → BRAM read (1 clk) → complex multiply → output
Expected: ~67 clocks (64 bins in + 3 pipeline flush)

This is a DIAGNOSTIC — reports timing, never asserts.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

import math


# IEEE 802.11a data subcarrier indices (as used by equalizer)
# The equalizer extracts these from the 64 FFT bins
DATA_BINS = [
    1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 22,
    23, 24, 25, 26, 38, 39, 40, 41, 42, 44, 45, 46, 47, 48, 49, 50, 51, 52,
    53, 54, 55, 56, 58, 59, 60, 61, 62, 63
]

PILOT_BINS = [7, 21, 43, 57]


def to_s16(val):
    """Convert to signed 16-bit representation."""
    val = int(round(val))
    val = max(-32768, min(32767, val))
    return val & 0xFFFF


async def reset_dut(dut):
    dut.rst_n.value = 0
    dut.start.value = 0
    dut.shift_val.value = 15
    dut.fft_valid.value = 0
    dut.fft_bin.value = 0
    dut.fft_re.value = 0
    dut.fft_im.value = 0
    dut.hinv_re.value = 0
    dut.hinv_im.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


@cocotb.test()
async def diag_eq_symbol_timing(dut):
    """Measure equalizer input-to-output latency per symbol.

    Feeds 64 FFT bins sequentially (one per clock) and measures:
    - First bin in → first data_valid out
    - First bin in → last data_valid out (all 48 data subcarriers)
    - Total symbol processing time
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    N_SYMBOLS = 8

    # Set up H_inv as unity (re=16384, im=0 for Q1.15 format, shift=15)
    # This way equalized output ≈ input
    HINV_UNITY_RE = 16384  # 0.5 in Q1.15 (with shift_val=14 → unity)
    HINV_UNITY_IM = 0
    dut.shift_val.value = 14  # shift=14 with hinv=0.5 → effective unity

    results = []

    dut._log.info("=" * 70)
    dut._log.info("EQUALIZER THROUGHPUT DIAGNOSTIC — per-symbol cycle budget")
    dut._log.info(f"  Symbols: {N_SYMBOLS}, feeding 64 bins/symbol (1/clock)")
    dut._log.info("=" * 70)

    for sym_idx in range(N_SYMBOLS):
        # Pulse start
        dut.start.value = 1
        await RisingEdge(dut.clk)
        dut.start.value = 0
        await RisingEdge(dut.clk)

        # Feed 64 FFT bins (one per clock, continuous)
        first_bin_cycle = None
        cycle = 0

        for bin_idx in range(64):
            dut.fft_valid.value = 1
            dut.fft_bin.value = bin_idx
            # Simple test pattern: known constellation point per bin
            dut.fft_re.value = to_s16(1000 * math.cos(bin_idx * 0.3))
            dut.fft_im.value = to_s16(1000 * math.sin(bin_idx * 0.3))
            # Provide H_inv (normally chan_est responds to rd_addr)
            dut.hinv_re.value = to_s16(HINV_UNITY_RE)
            dut.hinv_im.value = to_s16(HINV_UNITY_IM)
            await RisingEdge(dut.clk)
            cycle += 1
            if first_bin_cycle is None:
                first_bin_cycle = cycle

        dut.fft_valid.value = 0

        # Count data_valid outputs and find timing
        first_data_cycle = None
        last_data_cycle = None
        data_count = 0
        pilot_count = 0

        for extra in range(200):  # generous timeout
            await RisingEdge(dut.clk)
            cycle += 1

            try:
                dv = int(dut.data_valid.value)
                if dv:
                    if first_data_cycle is None:
                        first_data_cycle = cycle
                    last_data_cycle = cycle
                    data_count += 1
            except (ValueError, AttributeError):
                pass

            try:
                pv = int(dut.pilot_valid.value)
                if pv:
                    pilot_count += 1
            except (ValueError, AttributeError):
                pass

            # Done when we have all 48 data outputs
            if data_count >= 48:
                break

        results.append({
            'symbol': sym_idx,
            'first_in': first_bin_cycle,
            'first_data_out': first_data_cycle,
            'last_data_out': last_data_cycle,
            'latency': (first_data_cycle - first_bin_cycle) if first_data_cycle else None,
            'span': (last_data_cycle - first_bin_cycle) if last_data_cycle else None,
            'data_count': data_count,
            'pilot_count': pilot_count,
            'total_clocks': cycle,
        })

    # Report
    dut._log.info(" ")
    dut._log.info("Per-symbol results:")
    dut._log.info(f"  {'Sym':>3} {'Latency':>8} {'Span':>6} {'Data':>5} {'Pilot':>6} {'Total':>6}")
    dut._log.info(f"  {'---':>3} {'-------':>8} {'----':>6} {'----':>5} {'-----':>6} {'-----':>6}")
    for r in results:
        lat_str = str(r['latency']) if r['latency'] else "N/A"
        span_str = str(r['span']) if r['span'] else "N/A"
        dut._log.info(f"  {r['symbol']:>3} {lat_str:>8} {span_str:>6} "
                      f"{r['data_count']:>5} {r['pilot_count']:>6} {r['total_clocks']:>6}")

    if results:
        spans = [r['span'] for r in results if r['span']]
        if spans:
            avg = sum(spans) / len(spans)
            dut._log.info(" ")
            dut._log.info(f"  Average input-to-last-output span: {avg:.1f} clocks")
            dut._log.info(f"  Min: {min(spans)}, Max: {max(spans)}")
            dut._log.info(" ")
            dut._log.info("  NOTE: Equalizer is purely combinational/pipelined.")
            dut._log.info("  Cycle count = 64 (bin input) + pipeline latency (~3).")
            dut._log.info("  This module is NOT the throughput bottleneck.")


@cocotb.test()
async def diag_eq_back_to_back(dut):
    """Measure gap between consecutive symbol outputs (inter-symbol dead time).

    In the real pipeline, decode_engine re-arms the FFT between symbols. The
    equalizer itself has no inter-symbol gap — it processes whatever bins
    arrive. This test measures the equalizer's inherent turnaround.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    HINV_RE = 16384
    dut.shift_val.value = 14

    N_SYMBOLS = 5
    GAP_CLOCKS = 10  # clocks between symbols (simulates decode_engine re-arm)

    dut._log.info("=" * 70)
    dut._log.info("EQUALIZER BACK-TO-BACK — inter-symbol turnaround")
    dut._log.info(f"  {N_SYMBOLS} symbols, {GAP_CLOCKS}-clock gap between")
    dut._log.info("=" * 70)

    last_data_out_cycle = 0
    global_cycle = 0
    results = []

    for sym_idx in range(N_SYMBOLS):
        sym_start_cycle = global_cycle

        # Pulse start
        dut.start.value = 1
        await RisingEdge(dut.clk)
        global_cycle += 1
        dut.start.value = 0
        await RisingEdge(dut.clk)
        global_cycle += 1

        # Feed 64 bins
        for bin_idx in range(64):
            dut.fft_valid.value = 1
            dut.fft_bin.value = bin_idx
            dut.fft_re.value = to_s16(800)
            dut.fft_im.value = to_s16(400)
            dut.hinv_re.value = to_s16(HINV_RE)
            dut.hinv_im.value = to_s16(0)
            await RisingEdge(dut.clk)
            global_cycle += 1

        dut.fft_valid.value = 0

        # Drain outputs
        data_count = 0
        first_out = None
        for _ in range(200):
            await RisingEdge(dut.clk)
            global_cycle += 1
            try:
                if int(dut.data_valid.value):
                    data_count += 1
                    if first_out is None:
                        first_out = global_cycle
                    if data_count >= 48:
                        last_data_out_cycle = global_cycle
                        break
            except (ValueError, AttributeError):
                pass

        gap_from_prev = sym_start_cycle - (results[-1]['last_out'] if results else 0)
        results.append({
            'symbol': sym_idx,
            'start': sym_start_cycle,
            'first_out': first_out,
            'last_out': last_data_out_cycle,
            'processing': last_data_out_cycle - sym_start_cycle,
            'gap_from_prev': gap_from_prev if sym_idx > 0 else None,
        })

        # Inter-symbol gap
        for _ in range(GAP_CLOCKS):
            await RisingEdge(dut.clk)
            global_cycle += 1

    dut._log.info(" ")
    dut._log.info(f"  {'Sym':>3} {'Start':>6} {'1st Out':>8} {'Last Out':>9} "
                  f"{'Process':>8} {'Gap':>5}")
    for r in results:
        gap_str = str(r['gap_from_prev']) if r['gap_from_prev'] else "-"
        dut._log.info(f"  {r['symbol']:>3} {r['start']:>6} {r['first_out']:>8} "
                      f"{r['last_out']:>9} {r['processing']:>8} {gap_str:>5}")
