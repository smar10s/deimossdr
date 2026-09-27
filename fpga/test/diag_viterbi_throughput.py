"""
diag_viterbi_throughput.py — Viterbi decoder per-symbol cycle budget diagnostic.

Measures clock cycles consumed by viterbi_k7 in streaming mode (DATA symbols):
  - Input acceptance rate (stalls during traceback)
  - Traceback + output time per TB_DEPTH block
  - Effective bits-per-clock throughput

Viterbi architecture (streaming DATA mode):
  - TB_DEPTH=48 (was 40): accepts 48 soft-bit pairs (ACS), then stalls input
  - Traceback from best state: ~48 clocks
  - Output 48 decoded bits: ~48 clocks
  - Total per block: 48 + 48 + 48 = 144 clocks (for 48 decoded bits)

Per symbol at rate 6 (BPSK, 48 coded bits/symbol → 48 soft pairs):
  - Exactly 1 traceback block per symbol
  - Total: ~144 clocks/symbol

Per symbol at rate 24 (16-QAM, 192 coded bits/symbol → 192 soft pairs):
  - 4 traceback blocks per symbol
  - Total: ~576 clocks/symbol (but input arrives over 48 data_valid pulses,
    each carrying 4 soft bits, so input is spread across the equalizer output time)

This is a DIAGNOSTIC — reports timing, never asserts.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

import random


async def reset_dut(dut):
    dut.rst_n.value = 0
    dut.frame_start.value = 0
    dut.flush.value = 0
    dut.streaming_mode.value = 0
    dut.valid_in.value = 0
    dut.soft0.value = 0
    dut.soft1.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


def encode_bit(bit, g0=0o133, g1=0o171, state=0):
    """Encode one data bit with rate-1/2 K=7 convolutional code."""
    reg = (bit << 6) | state
    c0 = bin(reg & g0).count('1') % 2
    c1 = bin(reg & g1).count('1') % 2
    new_state = reg >> 1
    return c0, c1, new_state & 0x3F


def make_soft(coded_bit, noise=0):
    """Convert coded bit to soft value (positive = likely 0)."""
    # 0 → +64, 1 → -64 (with optional noise)
    val = 64 if coded_bit == 0 else -64
    val += random.randint(-noise, noise)
    return max(-127, min(127, val)) & 0xFF


@cocotb.test()
async def diag_viterbi_streaming_throughput(dut):
    """Measure Viterbi streaming-mode timing for DATA symbol decode.

    Feeds multiple symbols worth of soft bits (rate 1/2: 48 pairs per BPSK
    symbol) and measures:
    - Clock cycles per traceback block
    - Stall duration (input not accepted during traceback)
    - Output burst timing
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    N_SYMBOLS = 8  # 8 BPSK symbols = 8 × 48 = 384 coded bit pairs
    BITS_PER_SYMBOL = 48  # Rate 6 BPSK: 48 data bits per symbol

    # Generate random data and encode
    random.seed(42)
    data_bits = [random.randint(0, 1) for _ in range(N_SYMBOLS * BITS_PER_SYMBOL)]

    # Convolutional encode
    state = 0
    soft_pairs = []
    for bit in data_bits:
        c0, c1, state = encode_bit(bit, state=state)
        soft_pairs.append((make_soft(c0), make_soft(c1)))

    # Add tail bits (6 zeros to flush encoder)
    for _ in range(6):
        c0, c1, state = encode_bit(0, state=state)
        soft_pairs.append((make_soft(c0), make_soft(c1)))

    dut._log.info("=" * 70)
    dut._log.info("VITERBI THROUGHPUT DIAGNOSTIC — streaming mode")
    dut._log.info(f"  Symbols: {N_SYMBOLS} (BPSK, 48 pairs/symbol)")
    dut._log.info(f"  Total soft pairs: {len(soft_pairs)}")
    dut._log.info("=" * 70)

    # Start frame in streaming mode
    dut.frame_start.value = 1
    dut.streaming_mode.value = 1
    await RisingEdge(dut.clk)
    dut.frame_start.value = 0
    await RisingEdge(dut.clk)

    # Feed soft pairs and track timing
    pair_idx = 0
    total_pairs = len(soft_pairs)
    cycle = 0
    output_bits = 0
    stall_cycles = 0
    active_cycles = 0

    # Track per-block timing
    block_events = []  # (start_cycle, end_cycle, bits_out)
    current_block_start = 0
    block_bits = 0
    in_stall = False
    stall_start = None

    # Stall tracking
    stall_periods = []

    while pair_idx < total_pairs or output_bits < N_SYMBOLS * BITS_PER_SYMBOL:
        await RisingEdge(dut.clk)
        cycle += 1

        # Try to feed input
        if pair_idx < total_pairs:
            dut.valid_in.value = 1
            dut.soft0.value = soft_pairs[pair_idx][0]
            dut.soft1.value = soft_pairs[pair_idx][1]
        else:
            dut.valid_in.value = 0

        # Check if input was accepted (busy signal)
        try:
            busy = int(dut.busy.value)
            if busy:
                stall_cycles += 1
                if not in_stall:
                    in_stall = True
                    stall_start = cycle
            else:
                if in_stall:
                    stall_periods.append((stall_start, cycle - 1, cycle - stall_start))
                    in_stall = False
                if pair_idx < total_pairs:
                    pair_idx += 1
                    active_cycles += 1
        except (ValueError, AttributeError):
            if pair_idx < total_pairs:
                pair_idx += 1
                active_cycles += 1

        # Check output
        try:
            if int(dut.valid_out.value):
                output_bits += 1
                block_bits += 1
        except (ValueError, AttributeError):
            pass

        # Safety timeout
        if cycle > 5000:
            dut._log.warning(f"  TIMEOUT at cycle {cycle}, output_bits={output_bits}")
            break

    # Final flush
    dut.valid_in.value = 0
    dut.flush.value = 1
    await RisingEdge(dut.clk)
    dut.flush.value = 0

    flush_bits = 0
    flush_start = cycle
    for _ in range(500):
        await RisingEdge(dut.clk)
        cycle += 1
        try:
            if int(dut.valid_out.value):
                output_bits += 1
                flush_bits += 1
        except (ValueError, AttributeError):
            pass
        # Done when output stops
        try:
            if not int(dut.busy.value) and flush_bits > 0:
                # Wait a few more clocks for stragglers
                await ClockCycles(dut.clk, 10)
                cycle += 10
                break
        except (ValueError, AttributeError):
            pass

    # Report
    dut._log.info(" ")
    dut._log.info(f"  Total cycles: {cycle}")
    dut._log.info(f"  Input pairs fed: {pair_idx}/{total_pairs}")
    dut._log.info(f"  Output bits: {output_bits} (expected ~{N_SYMBOLS * BITS_PER_SYMBOL})")
    dut._log.info(f"  Active input cycles: {active_cycles}")
    dut._log.info(f"  Stall cycles: {stall_cycles}")
    dut._log.info(f"  Stall ratio: {stall_cycles/(active_cycles+stall_cycles)*100:.1f}%")

    if stall_periods:
        dut._log.info(f"  Stall events: {len(stall_periods)}")
        stall_durations = [s[2] for s in stall_periods]
        dut._log.info(f"  Stall durations: min={min(stall_durations)}, "
                      f"max={max(stall_durations)}, avg={sum(stall_durations)/len(stall_durations):.1f}")

    # Per-symbol equivalent
    if output_bits > 0:
        clocks_per_bit = cycle / output_bits
        clocks_per_symbol = clocks_per_bit * BITS_PER_SYMBOL
        dut._log.info(" ")
        dut._log.info(f"  Effective clocks/bit: {clocks_per_bit:.2f}")
        dut._log.info(f"  Effective clocks/symbol (48 bits): {clocks_per_symbol:.1f}")
        dut._log.info(f"  Budget impact: Viterbi consumes ~{clocks_per_symbol:.0f} clocks "
                      f"of the 400-clock symbol budget")
        dut._log.info(" ")
        dut._log.info("  NOTE: In the real pipeline, Viterbi processes IN PARALLEL with")
        dut._log.info("  the next symbol's FFT. The critical path is FFT, not Viterbi.")
        dut._log.info("  Viterbi stalls only matter if they back-pressure the depuncturer")
        dut._log.info("  faster than new symbols arrive.")


@cocotb.test()
async def diag_viterbi_input_rate_sweep(dut):
    """Measure Viterbi throughput at different input rates.

    Tests the decoder with varying input spacing to find the point where
    back-pressure (busy) engages. This tells us the maximum input rate
    the Viterbi can sustain without stalling upstream.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    random.seed(123)

    # One symbol worth of data (48 pairs for BPSK)
    data_bits = [random.randint(0, 1) for _ in range(96)]  # 2 symbols
    state = 0
    soft_pairs = []
    for bit in data_bits:
        c0, c1, state = encode_bit(bit, state=state)
        soft_pairs.append((make_soft(c0), make_soft(c1)))

    spacings = [1, 2, 3, 4, 5]  # clocks between valid inputs

    dut._log.info("=" * 70)
    dut._log.info("VITERBI INPUT RATE SWEEP — find back-pressure threshold")
    dut._log.info("=" * 70)

    for spacing in spacings:
        await reset_dut(dut)

        dut.frame_start.value = 1
        dut.streaming_mode.value = 1
        await RisingEdge(dut.clk)
        dut.frame_start.value = 0
        await RisingEdge(dut.clk)

        pair_idx = 0
        cycle = 0
        stalls = 0
        clk_in_spacing = 0

        while pair_idx < len(soft_pairs):
            await RisingEdge(dut.clk)
            cycle += 1
            clk_in_spacing += 1

            try:
                busy = int(dut.busy.value)
            except (ValueError, AttributeError):
                busy = 0

            if busy:
                stalls += 1
                dut.valid_in.value = 0
            elif clk_in_spacing >= spacing:
                dut.valid_in.value = 1
                dut.soft0.value = soft_pairs[pair_idx][0]
                dut.soft1.value = soft_pairs[pair_idx][1]
                pair_idx += 1
                clk_in_spacing = 0
            else:
                dut.valid_in.value = 0

            if cycle > 3000:
                break

        dut.valid_in.value = 0

        # Wait for outputs to drain
        output_count = 0
        for _ in range(500):
            await RisingEdge(dut.clk)
            cycle += 1
            try:
                if int(dut.valid_out.value):
                    output_count += 1
            except (ValueError, AttributeError):
                pass
            if output_count >= 48:  # at least one full block
                break

        dut._log.info(f"  Spacing={spacing}: {cycle} total cycles, "
                      f"{stalls} stalls ({stalls*100/cycle:.1f}%), "
                      f"{output_count} bits out")
