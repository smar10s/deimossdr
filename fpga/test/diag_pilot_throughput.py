"""
diag_pilot_throughput.py — Pilot tracking per-symbol cycle budget diagnostic.

Measures clock cycles consumed by pilot_track for each symbol:
  - symbol_start → first data_valid_out (pipeline latency)
  - symbol_start → symbol_done (total per-symbol time)
  - Breakdown: collect(~52 clk) + atan2(~18 clk) + emit(48+14 ≈ 62 clk)

Expected total: ~132 clocks/symbol (from module header comments).
This module is NOT the bottleneck — it processes data received from the
equalizer and emits to demapper. The bottleneck is the FFT feed
(u_fft + decode_engine).

This is a DIAGNOSTIC — reports timing, never asserts.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

import math

# Pilot polarity sequence (IEEE 802.11a Table 17-6)
PILOT_POLARITY = [
    1, 1, 1, 1, -1, -1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1,
    -1, -1, 1, 1, -1, 1, 1, -1, 1, 1, 1, 1, 1, 1, -1, 1,
    1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, 1, -1, -1, -1, 1,
    -1, 1, -1, -1, 1, -1, -1, 1, 1, 1, 1, 1, -1, -1, 1, 1,
    -1, -1, 1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, -1, -1,
    -1, 1, -1, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, -1, 1,
    -1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, 1, 1, -1,
    -1, 1, -1, -1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1
]


def to_s16(val):
    val = int(round(val))
    val = max(-32768, min(32767, val))
    return val & 0xFFFF


async def reset_dut(dut):
    dut.rst_n.value = 0
    dut.symbol_idx.value = 0
    dut.symbol_start.value = 0
    dut.pilot_valid.value = 0
    dut.pilot_re.value = 0
    dut.pilot_im.value = 0
    dut.pilot_idx.value = 0
    dut.data_valid_in.value = 0
    dut.data_re_in.value = 0
    dut.data_im_in.value = 0
    dut.data_idx_in.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


@cocotb.test()
async def diag_pilot_symbol_timing(dut):
    """Measure pilot_track per-symbol processing time.

    Feeds symbol data matching what the equalizer produces:
    - 4 pilot subcarriers (pilot_valid pulses)
    - 48 data subcarriers (data_valid_in pulses)
    Then measures time to symbol_done.

    Simulates realistic interleaved arrival: pilots and data come from
    the equalizer as bins are processed (pilots at their natural positions
    among the 64 bins).
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    N_SYMBOLS = 10
    PILOT_MAGNITUDE = 1000  # Typical equalized pilot magnitude

    results = []

    dut._log.info("=" * 70)
    dut._log.info("PILOT_TRACK THROUGHPUT DIAGNOSTIC — per-symbol cycle budget")
    dut._log.info(f"  Symbols: {N_SYMBOLS}")
    dut._log.info("=" * 70)

    for sym_idx in range(N_SYMBOLS):
        # Use symbol_idx >= 1 for DATA symbols (idx 0 = SIGNAL, bypasses pilot_track)
        actual_sym_idx = sym_idx + 1  # DATA symbols start at 1
        polarity = PILOT_POLARITY[actual_sym_idx % len(PILOT_POLARITY)]

        # Pulse symbol_start
        dut.symbol_idx.value = actual_sym_idx
        dut.symbol_start.value = 1
        await RisingEdge(dut.clk)
        dut.symbol_start.value = 0
        await RisingEdge(dut.clk)

        cycle = 0
        first_data_out = None
        data_out_count = 0
        done_cycle = None

        # Feed pilots (4 consecutive clocks, as equalizer emits them)
        for p_idx in range(4):
            dut.pilot_valid.value = 1
            # Pilot with known phase (zero CPE for clean measurement)
            dut.pilot_re.value = to_s16(PILOT_MAGNITUDE * polarity)
            dut.pilot_im.value = to_s16(0)
            dut.pilot_idx.value = p_idx
            await RisingEdge(dut.clk)
            cycle += 1

        dut.pilot_valid.value = 0

        # Feed 48 data subcarriers (consecutive, as equalizer emits them)
        for d_idx in range(48):
            dut.data_valid_in.value = 1
            # Simple data pattern
            dut.data_re_in.value = to_s16(500 * math.cos(d_idx * 0.2))
            dut.data_im_in.value = to_s16(500 * math.sin(d_idx * 0.2))
            dut.data_idx_in.value = d_idx
            await RisingEdge(dut.clk)
            cycle += 1

            # Check for output while feeding
            try:
                if int(dut.data_valid_out.value):
                    if first_data_out is None:
                        first_data_out = cycle
                    data_out_count += 1
            except (ValueError, AttributeError):
                pass

        dut.data_valid_in.value = 0

        # Wait for remaining outputs and symbol_done
        for _ in range(300):
            await RisingEdge(dut.clk)
            cycle += 1

            try:
                if int(dut.data_valid_out.value):
                    if first_data_out is None:
                        first_data_out = cycle
                    data_out_count += 1
            except (ValueError, AttributeError):
                pass

            try:
                if int(dut.symbol_done.value):
                    done_cycle = cycle
                    break
            except (ValueError, AttributeError):
                pass

        results.append({
            'symbol': actual_sym_idx,
            'first_out': first_data_out,
            'done': done_cycle,
            'data_out_count': data_out_count,
            'total_clocks': done_cycle if done_cycle else cycle,
        })

    # Report
    dut._log.info(" ")
    dut._log.info("Per-symbol results (clocks from symbol_start):")
    dut._log.info(f"  {'Sym':>3} {'1st Out':>8} {'Done':>6} {'N_data':>7} {'Total':>6}")
    dut._log.info(f"  {'---':>3} {'-------':>8} {'----':>6} {'------':>7} {'-----':>6}")
    for r in results:
        first_str = str(r['first_out']) if r['first_out'] else "N/A"
        done_str = str(r['done']) if r['done'] else "TIMEOUT"
        dut._log.info(f"  {r['symbol']:>3} {first_str:>8} {done_str:>6} "
                      f"{r['data_out_count']:>7} {r['total_clocks']:>6}")

    if results:
        dones = [r['done'] for r in results if r['done']]
        if dones:
            avg = sum(dones) / len(dones)
            dut._log.info(" ")
            dut._log.info(f"  Average symbol_start → symbol_done: {avg:.1f} clocks")
            dut._log.info(f"  Min: {min(dones)}, Max: {max(dones)}")
            dut._log.info(f"  Expected: ~132 clocks (collect + atan2 + emit)")
            dut._log.info(" ")
            dut._log.info("  NOTE: pilot_track is NOT the throughput bottleneck.")
            dut._log.info("  It processes data as fast as the equalizer emits it.")
            dut._log.info("  The real budget is dominated by the FFT (u_fft + decode_engine feed).")
