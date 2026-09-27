"""
Test depuncturer -- 802.11a depuncturing for convolutional decoder.

Inserts erasure (zero) soft bits at punctured positions to restore the
rate-1/2 coded stream expected by the Viterbi decoder.

Puncture patterns (from IEEE 802.11-2020, 17.3.5.6):
  Rate 1/2: passthrough (no puncturing applied)
  Rate 2/3: pattern [1,1,1,0] — 3 input → 4 output (insert 1 erasure per group)
  Rate 3/4: pattern [1,1,1,0,0,1] — 4 input → 6 output (insert 2 erasures per group)

Interface:
  code_rate[1:0]: 0=1/2 (passthrough), 1=2/3, 2=3/4
  valid_in + soft_in0/soft_in1[7:0]: deinterleaved soft bits (pair per clock;
    bit 2j on soft_in0, bit 2j+1 on soft_in1)
  symbol_start: pulse at first bit of each OFDM symbol (resets pattern counter)
  valid_out + soft_out[7:0]: depunctured soft bits (one per clock)
    - At punctured positions, outputs 0x00 (erasure = no confidence)
    - At kept positions, outputs the input soft bit unchanged

Output is bursty: for rates 2/3 and 3/4, more bits come out than went in.
The elastic pair FIFO absorbs input bursts (2 bits/clk in, 1 bit/clk out).

Tests:
  1. Rate 1/2 passthrough: input = output, no insertions
  2. Rate 3/4 with golden vector (Annex I.1 DATA, rate 36)
  3. Rate 2/3 with synthetic pattern
  4. Output count verification: N_in * expansion_factor = N_out
  5. symbol_start resets pattern mid-stream
  6. Back-to-back symbols
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer
import json
import os

VECTORS = os.path.join(os.path.dirname(__file__), '..', '..', 'extern', 'lib80211', 'vectors')

# Code rate modes
RATE_1_2 = 0  # passthrough
RATE_2_3 = 1  # pattern [1,1,1,0]
RATE_3_4 = 2  # pattern [1,1,1,0,0,1]

# Puncture patterns: 1 = kept (input bit), 0 = punctured (insert erasure)
PATTERNS = {
    RATE_2_3: [1, 1, 1, 0],
    RATE_3_4: [1, 1, 1, 0, 0, 1],
}

# Kept bits per pattern period
KEPT = {
    RATE_2_3: 3,  # 3 of 4 positions are kept
    RATE_3_4: 4,  # 4 of 6 positions are kept
}


def depuncture_reference(soft_in, code_rate):
    """Reference depuncture implementation.

    For rate 1/2: passthrough.
    For rate 2/3 and 3/4: walk the pattern, inserting 0 at punctured positions.
    """
    if code_rate == RATE_1_2:
        return list(soft_in)

    pattern = PATTERNS[code_rate]
    pat_len = len(pattern)
    kept_per_group = KEPT[code_rate]

    # Number of complete groups
    n_groups = len(soft_in) // kept_per_group
    assert len(soft_in) % kept_per_group == 0, \
        f"Input length {len(soft_in)} not divisible by {kept_per_group}"

    output = []
    in_idx = 0
    for _ in range(n_groups):
        for p in range(pat_len):
            if pattern[p] == 1:
                output.append(soft_in[in_idx])
                in_idx += 1
            else:
                output.append(0)  # erasure

    assert in_idx == len(soft_in)
    return output


def to_s8(v):
    """Interpret 8-bit value as signed."""
    v = int(v) & 0xFF
    return v - 0x100 if v >= 0x80 else v


def s8_to_bits(v):
    """Convert signed int to 8-bit unsigned representation."""
    if v < 0:
        v = v + 0x100
    return v & 0xFF


async def reset_dut(dut):
    """Reset the DUT and initialize inputs."""
    dut.rst_n.value = 0
    dut.valid_in.value = 0
    dut.soft_in0.value = 0
    dut.soft_in1.value = 0
    dut.code_rate.value = 0
    dut.symbol_start.value = 0
    dut.stall_in.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def feed_and_collect(dut, soft_in_list, code_rate, n_expected):
    """Feed soft bits to depuncturer and collect outputs.

    Drives valid_in=1 for each input bit. The module may hold back input
    (ready=0) while emitting erasure insertions. We wait for all expected
    outputs.

    Args:
        soft_in_list: list of signed 8-bit soft values
        code_rate: RATE_1_2, RATE_2_3, or RATE_3_4
        n_expected: expected number of output soft bits

    Returns:
        list of signed 8-bit soft values (depunctured)
    """
    dut.code_rate.value = code_rate

    # Pulse symbol_start to reset pattern counter
    dut.symbol_start.value = 1
    await RisingEdge(dut.clk)
    dut.symbol_start.value = 0

    soft_out = []
    in_idx = 0

    # Feed inputs and collect outputs simultaneously
    # The module may emit more outputs than inputs (erasure insertions)
    timeout = len(soft_in_list) * 3 + 50  # generous timeout
    cycles = 0

    while len(soft_out) < n_expected and cycles < timeout:
        # Drive input pairs (2 bits per valid_in)
        if in_idx < len(soft_in_list):
            dut.valid_in.value = 1
            dut.soft_in0.value = s8_to_bits(soft_in_list[in_idx])
            dut.soft_in1.value = s8_to_bits(soft_in_list[in_idx + 1])
            in_idx += 2
        else:
            dut.valid_in.value = 0
            dut.soft_in0.value = 0
            dut.soft_in1.value = 0

        await RisingEdge(dut.clk)

        # Collect output
        if int(dut.valid_out.value) == 1:
            soft_out.append(to_s8(int(dut.soft_out.value)))

        cycles += 1

    # Keep collecting any remaining outputs after all inputs sent
    for _ in range(50):
        await RisingEdge(dut.clk)
        if int(dut.valid_out.value) == 1:
            soft_out.append(to_s8(int(dut.soft_out.value)))

    return soft_out


@cocotb.test()
async def test_rate_half_passthrough(dut):
    """Rate 1/2: passthrough — output = input with no insertions.

    Feed 48 soft bits (SIGNAL field size). Expect 48 soft bits out,
    identical to input.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # 48 soft bits with various magnitudes
    soft_in = [((i * 7 + 3) % 251) - 125 for i in range(48)]
    expected = depuncture_reference(soft_in, RATE_1_2)

    assert expected == soft_in, "Rate 1/2 reference should be identity"

    soft_out = await feed_and_collect(dut, soft_in, RATE_1_2, n_expected=48)

    assert len(soft_out) == 48, f"Expected 48 outputs, got {len(soft_out)}"

    errors = 0
    for i in range(48):
        if soft_out[i] != expected[i]:
            dut._log.warning(f"Pos {i}: got {soft_out[i]}, expected {expected[i]}")
            errors += 1

    assert errors == 0, f"Rate 1/2 passthrough: {errors}/48 errors"
    dut._log.info("Rate 1/2 passthrough test passed (48/48 correct)")


@cocotb.test()
async def test_rate_34_golden_vector(dut):
    """Rate 3/4: Annex I.1 DATA first symbol — depuncture matches reference.

    Annex I.1 is rate 36 (16-QAM, coding rate 3/4).
    Input: 192 deinterleaved bits (one OFDM symbol worth).
    After depuncture: 192 * 6/4 = 288 bits (rate-1/2 equivalent).

    Pattern [1,1,1,0,0,1]: every group of 4 input bits becomes 6 output bits,
    with erasures at positions 3 and 4 in each group.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Load golden vector (post-puncture encoded data)
    with open(os.path.join(VECTORS, 'annex_i1_data_encoded.json')) as f:
        data_enc = json.load(f)

    # First DATA symbol = 192 bits (N_CBPS for 16-QAM)
    encoded_bits = data_enc['data'][:192]

    # Convert hard bits to soft LLRs for test: bit=1 → +100, bit=0 → -100
    soft_in = [100 if b == 1 else -100 for b in encoded_bits]

    # Expected depuncture output: 192 * 6/4 = 288 soft bits
    expected = depuncture_reference(soft_in, RATE_3_4)
    n_expected = 288

    assert len(expected) == n_expected, \
        f"Reference output should be {n_expected}, got {len(expected)}"

    soft_out = await feed_and_collect(dut, soft_in, RATE_3_4, n_expected=n_expected)

    assert len(soft_out) == n_expected, \
        f"Expected {n_expected} outputs, got {len(soft_out)}"

    errors = 0
    for i in range(n_expected):
        if soft_out[i] != expected[i]:
            dut._log.warning(
                f"Pos {i}: got {soft_out[i]}, expected {expected[i]} "
                f"(pattern pos {i % 6})")
            errors += 1
            if errors > 10:
                dut._log.warning("... (truncating)")
                break

    assert errors == 0, f"Rate 3/4 depuncture: {errors}/{n_expected} errors"
    dut._log.info(f"Rate 3/4 golden vector test passed ({n_expected}/{n_expected} correct)")


@cocotb.test()
async def test_rate_23_synthetic(dut):
    """Rate 2/3: pattern [1,1,1,0] — 3 input → 4 output per group.

    Feed 48 soft bits (16 groups of 3). Expect 64 output bits (16 groups of 4).
    Every 4th output position should be 0 (erasure).
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # 48 input soft bits (must be divisible by 3 for rate 2/3)
    soft_in = [((i * 11 + 7) % 251) - 125 for i in range(48)]

    expected = depuncture_reference(soft_in, RATE_2_3)
    n_expected = 64  # 48 * 4/3 = 64

    assert len(expected) == n_expected

    # Verify reference has erasures at correct positions
    for g in range(16):
        # Pattern [1,1,1,0]: position 3 in each group is erasure
        assert expected[g * 4 + 3] == 0, \
            f"Group {g} pos 3 should be erasure, got {expected[g * 4 + 3]}"
        # Positions 0,1,2 should be original values
        assert expected[g * 4 + 0] == soft_in[g * 3 + 0]
        assert expected[g * 4 + 1] == soft_in[g * 3 + 1]
        assert expected[g * 4 + 2] == soft_in[g * 3 + 2]

    soft_out = await feed_and_collect(dut, soft_in, RATE_2_3, n_expected=n_expected)

    assert len(soft_out) == n_expected, \
        f"Expected {n_expected} outputs, got {len(soft_out)}"

    errors = 0
    for i in range(n_expected):
        if soft_out[i] != expected[i]:
            dut._log.warning(
                f"Pos {i}: got {soft_out[i]}, expected {expected[i]} "
                f"(pattern pos {i % 4}, {'ERASURE' if PATTERNS[RATE_2_3][i % 4] == 0 else 'kept'})")
            errors += 1

    assert errors == 0, f"Rate 2/3 depuncture: {errors}/{n_expected} errors"
    dut._log.info(f"Rate 2/3 synthetic test passed ({n_expected}/{n_expected} correct)")


@cocotb.test()
async def test_output_count_all_rates(dut):
    """Output count: verify expansion factor for all code rates.

    Rate 1/2: 96 in → 96 out (×1)
    Rate 2/3: 96 in → 128 out (×4/3)
    Rate 3/4: 96 in → 144 out (×6/4 = ×3/2)
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    test_cases = [
        (RATE_1_2, 96, 96),
        (RATE_2_3, 96, 128),   # 96/3 = 32 groups × 4 = 128
        (RATE_3_4, 96, 144),   # 96/4 = 24 groups × 6 = 144
    ]

    for code_rate, n_in, n_expected in test_cases:
        await reset_dut(dut)
        soft_in = [50 if i % 2 == 0 else -50 for i in range(n_in)]

        soft_out = await feed_and_collect(dut, soft_in, code_rate, n_expected=n_expected)

        assert len(soft_out) == n_expected, \
            f"Rate {code_rate}: {n_in} in → expected {n_expected} out, got {len(soft_out)}"

        dut._log.info(f"Rate {code_rate}: {n_in} → {len(soft_out)} outputs OK")

    dut._log.info("Output count test passed for all rates")


@cocotb.test()
async def test_symbol_start_resets_pattern(dut):
    """symbol_start resets the pattern counter mid-stream.

    Feed partial symbol at rate 3/4 (not aligned to pattern boundary),
    then pulse symbol_start, feed a complete symbol. Verify second symbol
    is correct (pattern counter was reset).
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    dut.code_rate.value = RATE_3_4

    # Pulse symbol_start
    dut.symbol_start.value = 1
    await RisingEdge(dut.clk)
    dut.symbol_start.value = 0

    # Feed 3 pairs (6 bits). Draining crosses the rate 3/4 erasure
    # positions and wraps the 6-bit pattern period, leaving pat_pos
    # mid-pattern (non-zero), so the symbol_start below clears a
    # genuinely partial pattern state.
    for i in range(3):
        dut.valid_in.value = 1
        dut.soft_in0.value = s8_to_bits(50)
        dut.soft_in1.value = s8_to_bits(50)
        await RisingEdge(dut.clk)

    dut.valid_in.value = 0
    await ClockCycles(dut.clk, 5)

    # Now pulse symbol_start again to reset
    dut.symbol_start.value = 1
    await RisingEdge(dut.clk)
    dut.symbol_start.value = 0

    # Drain any leftover outputs
    await ClockCycles(dut.clk, 10)

    # Feed a complete aligned symbol (12 bits = 3 groups of 4 for rate 3/4)
    soft_in = [((i * 13 + 1) % 251) - 125 for i in range(12)]
    expected = depuncture_reference(soft_in, RATE_3_4)
    n_expected = 18  # 12 * 6/4 = 18

    soft_out = []
    in_idx = 0
    timeout = 100

    for _ in range(timeout):
        if in_idx < len(soft_in):
            dut.valid_in.value = 1
            dut.soft_in0.value = s8_to_bits(soft_in[in_idx])
            dut.soft_in1.value = s8_to_bits(soft_in[in_idx + 1])
            in_idx += 2
        else:
            dut.valid_in.value = 0

        await RisingEdge(dut.clk)

        if int(dut.valid_out.value) == 1:
            soft_out.append(to_s8(int(dut.soft_out.value)))

        if len(soft_out) >= n_expected and in_idx >= len(soft_in):
            break

    # Collect stragglers
    for _ in range(20):
        await RisingEdge(dut.clk)
        if int(dut.valid_out.value) == 1:
            soft_out.append(to_s8(int(dut.soft_out.value)))

    # We only check the last n_expected outputs (after reset)
    # There may be partial outputs from before the reset
    final_out = soft_out[-n_expected:] if len(soft_out) >= n_expected else soft_out

    assert len(final_out) == n_expected, \
        f"Expected {n_expected} outputs after reset, got {len(final_out)}"

    errors = 0
    for i in range(n_expected):
        if final_out[i] != expected[i]:
            dut._log.warning(f"Pos {i}: got {final_out[i]}, expected {expected[i]}")
            errors += 1

    assert errors == 0, f"Symbol start reset: {errors}/{n_expected} errors"
    dut._log.info("symbol_start reset test passed")


@cocotb.test()
async def test_back_to_back_symbols_rate_34(dut):
    """Back-to-back: two consecutive rate 3/4 symbols without reset.

    Verify both are depunctured correctly. Second symbol should start
    fresh pattern (triggered by symbol_start between them).
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    dut.code_rate.value = RATE_3_4

    # Symbol 1: 24 input bits → 36 output bits
    soft_in_1 = [((i * 7 + 3) % 251) - 125 for i in range(24)]
    expected_1 = depuncture_reference(soft_in_1, RATE_3_4)

    # Symbol 2: 24 input bits → 36 output bits
    soft_in_2 = [((i * 11 + 9) % 251) - 125 for i in range(24)]
    expected_2 = depuncture_reference(soft_in_2, RATE_3_4)

    # --- Feed symbol 1 ---
    dut.symbol_start.value = 1
    await RisingEdge(dut.clk)
    dut.symbol_start.value = 0

    out_1 = []
    in_idx = 0
    timeout = 0
    while (len(out_1) < 36 or in_idx < 24) and timeout < 200:
        if in_idx < 24:
            dut.valid_in.value = 1
            dut.soft_in0.value = s8_to_bits(soft_in_1[in_idx])
            dut.soft_in1.value = s8_to_bits(soft_in_1[in_idx + 1])
            in_idx += 2
        else:
            dut.valid_in.value = 0

        await RisingEdge(dut.clk)

        if int(dut.valid_out.value) == 1:
            out_1.append(to_s8(int(dut.soft_out.value)))
        timeout += 1

    # Drain remaining
    for _ in range(20):
        await RisingEdge(dut.clk)
        if int(dut.valid_out.value) == 1:
            out_1.append(to_s8(int(dut.soft_out.value)))

    assert len(out_1) == 36, f"Symbol 1: expected 36 outputs, got {len(out_1)}"

    # --- Feed symbol 2 ---
    dut.symbol_start.value = 1
    await RisingEdge(dut.clk)
    dut.symbol_start.value = 0

    out_2 = []
    in_idx = 0
    timeout = 0
    while (len(out_2) < 36 or in_idx < 24) and timeout < 200:
        if in_idx < 24:
            dut.valid_in.value = 1
            dut.soft_in0.value = s8_to_bits(soft_in_2[in_idx])
            dut.soft_in1.value = s8_to_bits(soft_in_2[in_idx + 1])
            in_idx += 2
        else:
            dut.valid_in.value = 0

        await RisingEdge(dut.clk)

        if int(dut.valid_out.value) == 1:
            out_2.append(to_s8(int(dut.soft_out.value)))
        timeout += 1

    # Drain remaining
    for _ in range(20):
        await RisingEdge(dut.clk)
        if int(dut.valid_out.value) == 1:
            out_2.append(to_s8(int(dut.soft_out.value)))

    assert len(out_2) == 36, f"Symbol 2: expected 36 outputs, got {len(out_2)}"

    # Verify both symbols
    errors = 0
    for i in range(36):
        if out_1[i] != expected_1[i]:
            dut._log.warning(f"Sym1 pos {i}: got {out_1[i]}, expected {expected_1[i]}")
            errors += 1
        if out_2[i] != expected_2[i]:
            dut._log.warning(f"Sym2 pos {i}: got {out_2[i]}, expected {expected_2[i]}")
            errors += 1

    assert errors == 0, f"Back-to-back rate 3/4: {errors}/72 errors"
    dut._log.info("Back-to-back rate 3/4 test passed")


# =========================================================
# Test 7: Continuous multi-symbol streaming (rate 3/4, 3 symbols)
# =========================================================
@cocotb.test()
async def test_continuous_three_symbols_rate_34(dut):
    """Three consecutive rate 3/4 symbols with symbol_start between each.

    Exercises the FIFO under sustained load: 3 × 192 = 576 input bits,
    producing 3 × 288 = 864 output bits. Each symbol_start fires only after
    the previous symbol's outputs are collected (since symbol_start resets the FIFO).
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    dut.code_rate.value = RATE_3_4

    N_SYMBOLS = 3
    BITS_PER_SYM = 192
    OUT_PER_SYM = 288

    all_soft_in = []
    all_expected = []
    for sym in range(N_SYMBOLS):
        soft_in = [((i * (7 + sym * 3) + sym * 11) % 251) - 125 for i in range(BITS_PER_SYM)]
        all_soft_in.append(soft_in)
        all_expected.append(depuncture_reference(soft_in, RATE_3_4))

    # Process each symbol: feed all inputs, then collect all outputs before next
    all_out = []
    for sym in range(N_SYMBOLS):
        sym_out = await feed_and_collect(
            dut, all_soft_in[sym], RATE_3_4, n_expected=OUT_PER_SYM)
        all_out.extend(sym_out)

    assert len(all_out) == N_SYMBOLS * OUT_PER_SYM, \
        f"Expected {N_SYMBOLS * OUT_PER_SYM} total outputs, got {len(all_out)}"

    # Verify each symbol's output matches reference
    errors = 0
    for sym in range(N_SYMBOLS):
        start = sym * OUT_PER_SYM
        for i in range(OUT_PER_SYM):
            if all_out[start + i] != all_expected[sym][i]:
                errors += 1
                if errors <= 5:
                    dut._log.warning(
                        f"Sym {sym} pos {i}: got {all_out[start + i]}, "
                        f"expected {all_expected[sym][i]}")

    assert errors == 0, f"Continuous 3-symbol rate 3/4: {errors}/{N_SYMBOLS * OUT_PER_SYM} errors"
    dut._log.info(f"Continuous 3-symbol rate 3/4 passed ({N_SYMBOLS * OUT_PER_SYM} outputs correct)")


# =========================================================
# Test 8: Rate 2/3 continuous (3 symbols, pattern boundary)
# =========================================================
@cocotb.test()
async def test_continuous_three_symbols_rate_23(dut):
    """Three consecutive rate 2/3 symbols — verifies pattern resets correctly.

    Pattern [1,1,1,0] must reset at each symbol_start. If it doesn't, the
    erasure positions drift and corrupt subsequent symbols.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    dut.code_rate.value = RATE_2_3

    N_SYMBOLS = 3
    BITS_PER_SYM = 96
    OUT_PER_SYM = 128

    all_soft_in = []
    all_expected = []
    for sym in range(N_SYMBOLS):
        soft_in = [((i * (11 + sym * 5) + sym * 7) % 251) - 125 for i in range(BITS_PER_SYM)]
        all_soft_in.append(soft_in)
        all_expected.append(depuncture_reference(soft_in, RATE_2_3))

    # Process each symbol sequentially
    all_out = []
    for sym in range(N_SYMBOLS):
        sym_out = await feed_and_collect(
            dut, all_soft_in[sym], RATE_2_3, n_expected=OUT_PER_SYM)
        all_out.extend(sym_out)

    assert len(all_out) == N_SYMBOLS * OUT_PER_SYM, \
        f"Expected {N_SYMBOLS * OUT_PER_SYM} total outputs, got {len(all_out)}"

    errors = 0
    for sym in range(N_SYMBOLS):
        start = sym * OUT_PER_SYM
        for i in range(OUT_PER_SYM):
            if all_out[start + i] != all_expected[sym][i]:
                errors += 1
                if errors <= 5:
                    dut._log.warning(
                        f"Sym {sym} pos {i}: got {all_out[start + i]}, "
                        f"expected {all_expected[sym][i]}")

    assert errors == 0, f"Continuous 3-symbol rate 2/3: {errors}/{N_SYMBOLS * OUT_PER_SYM} errors"
    dut._log.info(f"Continuous 3-symbol rate 2/3 passed ({N_SYMBOLS * OUT_PER_SYM} outputs correct)")


# =========================================================
# Test 9: Streaming without symbol_start (pattern free-runs)
# =========================================================
@cocotb.test()
async def test_no_symbol_start_between(dut):
    """Feed 2 symbols worth of data at rate 3/4 WITHOUT symbol_start between them.

    This tests what happens if the upstream module doesn't fire symbol_start
    at the boundary. The pattern counter should free-run, and if the input
    count happens to be aligned to the pattern period, it should still work.
    If misaligned, the second symbol will be wrong — that's expected, and
    proves symbol_start is required.

    For rate 3/4: pattern period = 6, kept per period = 4.
    If input_per_symbol = 192 (divisible by 4), then 192/4 = 48 complete groups.
    48 × 6 = 288 outputs. Pattern wraps cleanly. Without symbol_start,
    the second symbol SHOULD still decode correctly because 192 is exactly
    aligned to the pattern boundary.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    dut.code_rate.value = RATE_3_4

    BITS_PER_SYM = 192  # exactly divisible by 4 (kept_per_group)
    OUT_PER_SYM = 288

    # Two symbols concatenated — single initial symbol_start only
    soft_in_1 = [((i * 7 + 3) % 251) - 125 for i in range(BITS_PER_SYM)]
    soft_in_2 = [((i * 13 + 5) % 251) - 125 for i in range(BITS_PER_SYM)]
    all_soft = soft_in_1 + soft_in_2

    # With free-running pattern, the reference is just depuncturing the whole stream
    expected_all = depuncture_reference(all_soft, RATE_3_4)
    total_expected = len(expected_all)

    assert total_expected == 2 * OUT_PER_SYM

    # Feed everything with only ONE symbol_start at the beginning
    dut.symbol_start.value = 1
    await RisingEdge(dut.clk)
    dut.symbol_start.value = 0

    all_out = []
    in_idx = 0
    timeout = len(all_soft) * 3 + 200

    for _ in range(timeout):
        if in_idx < len(all_soft):
            dut.valid_in.value = 1
            dut.soft_in0.value = s8_to_bits(all_soft[in_idx])
            dut.soft_in1.value = s8_to_bits(all_soft[in_idx + 1])
            in_idx += 2
        else:
            dut.valid_in.value = 0

        await RisingEdge(dut.clk)

        if int(dut.valid_out.value) == 1:
            all_out.append(to_s8(int(dut.soft_out.value)))

        if len(all_out) >= total_expected and in_idx >= len(all_soft):
            break

    # Drain
    dut.valid_in.value = 0
    for _ in range(100):
        await RisingEdge(dut.clk)
        if int(dut.valid_out.value) == 1:
            all_out.append(to_s8(int(dut.soft_out.value)))

    assert len(all_out) == total_expected, \
        f"Expected {total_expected} outputs, got {len(all_out)}"

    # Because 192 is aligned to the pattern period (192/4=48 groups exactly),
    # the free-running pattern should produce correct output for both halves.
    errors = 0
    for i in range(total_expected):
        if all_out[i] != expected_all[i]:
            errors += 1
            if errors <= 5:
                dut._log.warning(f"Pos {i}: got {all_out[i]}, expected {expected_all[i]}")

    assert errors == 0, f"Free-running pattern (aligned): {errors}/{total_expected} errors"
    dut._log.info(f"No symbol_start between aligned symbols: {total_expected} outputs correct")


@cocotb.test()
async def test_stall_in_freezes_output(dut):
    """stall_in holds the current output bit; sequence preserved on release."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    dut.code_rate.value = RATE_3_4
    dut.stall_in.value = 0

    # 24 input bits (12 pairs) -> 24 * 6/4 = 36 output bits
    soft_in = [((i * 7) % 200) - 100 for i in range(24)]
    expected = depuncture_reference(soft_in, RATE_3_4)

    out = []
    stalled_once = False
    pair_idx = 0
    while pair_idx < len(soft_in) // 2:
        if pair_idx == 4 and not stalled_once:
            # Stall mid-stream. The bit visible now was already collected
            # at the previous edge -- only assert stability here.
            stalled_once = True
            dut.stall_in.value = 1
            held_valid = int(dut.valid_out.value)
            held_soft = int(dut.soft_out.value)
            for _ in range(8):
                await RisingEdge(dut.clk)
                await Timer(1, unit="ns")
                assert int(dut.valid_out.value) == held_valid, \
                    "valid_out changed during stall"
                if held_valid:
                    assert int(dut.soft_out.value) == held_soft, \
                        "soft_out changed during stall"
            dut.stall_in.value = 0
            continue
        dut.valid_in.value = 1
        dut.soft_in0.value = s8_to_bits(soft_in[2 * pair_idx])
        dut.soft_in1.value = s8_to_bits(soft_in[2 * pair_idx + 1])
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        dut.valid_in.value = 0
        pair_idx += 1
        if int(dut.valid_out.value) == 1:
            out.append(to_s8(int(dut.soft_out.value)))

    # Drain remaining outputs
    for _ in range(200):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        if int(dut.valid_out.value) == 1:
            out.append(to_s8(int(dut.soft_out.value)))

    assert len(out) == len(expected), f"expected {len(expected)} outputs, got {len(out)}"
    assert out == expected, f"sequence mismatch: {out} != {expected}"


async def drip_feed_collect(dut, soft_in, code_rate, gap, n_expected):
    """Feed pairs with idle gaps so the pair FIFO drains to empty mid-frame.

    The existing tests all keep the FIFO non-empty or deliberately overrun
    it. This helper covers the opposite regime: the emit FSM repeatedly
    catches up with the write pointer and sees fifo_empty while the
    puncture pattern sits mid-group.

    Returns the collected output bits in order.
    """
    out = []
    for i in range(0, len(soft_in), 2):
        dut.valid_in.value = 1
        dut.soft_in0.value = s8_to_bits(soft_in[i])
        dut.soft_in1.value = s8_to_bits(soft_in[i + 1])
        await RisingEdge(dut.clk)
        if int(dut.valid_out.value) == 1:
            out.append(to_s8(int(dut.soft_out.value)))

        # Idle gap: nothing offered, so the FSM drains the FIFO to empty.
        dut.valid_in.value = 0
        for _ in range(gap):
            await RisingEdge(dut.clk)
            if int(dut.valid_out.value) == 1:
                out.append(to_s8(int(dut.soft_out.value)))

    dut.valid_in.value = 0
    for _ in range(n_expected * 2 + 200):
        if len(out) >= n_expected:
            break
        await RisingEdge(dut.clk)
        if int(dut.valid_out.value) == 1:
            out.append(to_s8(int(dut.soft_out.value)))
    return out


@cocotb.test()
async def test_starved_input_rate_34(dut):
    """Rate 3/4 with a starved input: the FIFO empties mid-pattern-group.

    Guards the regime a continuous-flow Viterbi creates. When the decoder
    stops asserting backpressure, the pair FIFO runs dry instead of
    backing up, so the emit FSM decides erasure-vs-kept while empty.
    A phase slip here shifts every later bit of the frame, because this
    module free-runs its pattern with symbol_start tied low.

    Swept across several gap widths: a gap of 1 barely starves the FIFO,
    while a gap of 12 empties it inside every pattern group.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # 192 bits = 48 complete rate-3/4 groups, so the pattern ends aligned.
    N_BITS = 192
    soft_in = [((i * 7 + 3) % 251) - 125 for i in range(N_BITS)]
    expected = depuncture_reference(soft_in, RATE_3_4)

    for gap in (1, 2, 3, 5, 8, 12):
        await reset_dut(dut)
        dut.code_rate.value = RATE_3_4
        dut.symbol_start.value = 1
        await RisingEdge(dut.clk)
        dut.symbol_start.value = 0

        out = await drip_feed_collect(
            dut, soft_in, RATE_3_4, gap, n_expected=len(expected))

        assert len(out) == len(expected), (
            f"gap={gap}: expected {len(expected)} outputs, got {len(out)}")

        first_bad = next(
            (i for i in range(len(expected)) if out[i] != expected[i]), None)
        assert first_bad is None, (
            f"gap={gap}: starved-input phase slip at output bit {first_bad} "
            f"(got {out[first_bad]}, expected {expected[first_bad]}); "
            f"got {out[max(0, first_bad - 3):first_bad + 6]} vs "
            f"expected {expected[max(0, first_bad - 3):first_bad + 6]}")
        dut._log.info(f"gap={gap}: {len(out)} outputs correct under starvation")


@cocotb.test()
async def test_starved_input_rate_23(dut):
    """Rate 2/3 with a starved input — same guard, pattern [1,1,1,0].

    Rate 2/3 has one erasure per group instead of two, so it exercises a
    different residue of the pattern counter against the empty FIFO.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # 192 bits = 64 complete rate-2/3 groups.
    N_BITS = 192
    soft_in = [((i * 11 + 5) % 251) - 125 for i in range(N_BITS)]
    expected = depuncture_reference(soft_in, RATE_2_3)

    for gap in (1, 3, 8):
        await reset_dut(dut)
        dut.code_rate.value = RATE_2_3
        dut.symbol_start.value = 1
        await RisingEdge(dut.clk)
        dut.symbol_start.value = 0

        out = await drip_feed_collect(
            dut, soft_in, RATE_2_3, gap, n_expected=len(expected))

        assert len(out) == len(expected), (
            f"gap={gap}: expected {len(expected)} outputs, got {len(out)}")

        first_bad = next(
            (i for i in range(len(expected)) if out[i] != expected[i]), None)
        assert first_bad is None, (
            f"gap={gap}: starved-input phase slip at output bit {first_bad} "
            f"(got {out[first_bad]}, expected {expected[first_bad]})")
        dut._log.info(f"gap={gap}: {len(out)} outputs correct under starvation")


@cocotb.test()
async def test_fifo_full_backpressure_contract(dut):
    """fifo_full asserts when downstream stalls; held writes retried, not lost."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    dut.code_rate.value = RATE_3_4
    dut.stall_in.value = 1   # downstream never drains
    dut.valid_in.value = 0

    # In this harness, values read after RisingEdge are the pre-edge
    # values, so `fifo_full == 0` read after an edge means the write at
    # that edge was accepted (the FSM sampled the same pre-edge full).
    soft_in = [((i * 7) % 200) - 100 for i in range(600)]
    pair_idx = 0
    full_seen = False
    while pair_idx < len(soft_in) // 2:
        dut.valid_in.value = 1
        dut.soft_in0.value = s8_to_bits(soft_in[2 * pair_idx])
        dut.soft_in1.value = s8_to_bits(soft_in[2 * pair_idx + 1])
        await RisingEdge(dut.clk)
        if int(dut.fifo_full.value) == 0:
            pair_idx += 1
        else:
            full_seen = True
            break
    assert full_seen, "fifo_full never asserted with downstream stalled"

    # Hold the next pair while full: the write is skipped, not lost
    for _ in range(8):
        dut.valid_in.value = 1
        await RisingEdge(dut.clk)
        assert int(dut.fifo_full.value) == 1, "fifo_full dropped while holding"

    # Release downstream and retry the held pair: with valid_in held, the
    # write is retried each cycle until fifo_full drops (pre-edge), then
    # accepted exactly once. Feed the rest under the same handshake while
    # collecting outputs -- no loss, no duplication.
    dut.stall_in.value = 0
    out = []
    retry_cycles = 0
    while pair_idx < len(soft_in) // 2:
        dut.valid_in.value = 1
        dut.soft_in0.value = s8_to_bits(soft_in[2 * pair_idx])
        dut.soft_in1.value = s8_to_bits(soft_in[2 * pair_idx + 1])
        await RisingEdge(dut.clk)
        retry_cycles += 1
        assert retry_cycles < 10000, \
            "held write never accepted after release (retry contract broken)"
        if int(dut.fifo_full.value) == 0:
            pair_idx += 1
        # else: write skipped this cycle -- same pair retried next cycle
        if int(dut.valid_out.value) == 1:
            out.append(to_s8(int(dut.soft_out.value)))
    dut.valid_in.value = 0

    # Drain the remainder; everything fed must come out in order
    for _ in range(3000):
        await RisingEdge(dut.clk)
        if int(dut.valid_out.value) == 1:
            out.append(to_s8(int(dut.soft_out.value)))

    expected = depuncture_reference(soft_in, RATE_3_4)
    assert len(out) == len(expected), f"expected {len(expected)} outputs, got {len(out)}"
    assert out == expected, f"sequence mismatch: {out} != {expected}"
