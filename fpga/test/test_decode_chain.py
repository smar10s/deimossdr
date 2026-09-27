"""
Integration test: depuncturer → soft_pairer → vit_fifo → viterbi_k7

Tests the exact data path that fails on hardware at rate 1/2.
Rate 1/2: depuncturer is passthrough, soft_pairer pairs into (G0, G1),
vit_fifo buffers during Viterbi traceback stalls.

Multi-symbol input simulates what happens during DATA decode:
deinterleaver outputs N_CBPS bits per symbol with a gap between symbols.
The gap (valid=0) triggers depuncturer capture-then-emit.

Test approach:
1. Generate known data bits
2. Convolutional encode at rate 1/2
3. Feed through the chain symbol-by-symbol (48 bits per symbol for BPSK)
4. Verify decoded output matches original data bits
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer

# Generator polynomials (octal)
G0 = 0o133  # 0b1011011
G1 = 0o171  # 0b1111001


def conv_encode(bits):
    """Convolutional encode (rate 1/2, K=7)."""
    state = 0
    coded = []
    for b in bits:
        state = ((state >> 1) | (b << 6)) & 0x7F
        o0 = bin(state & G0).count('1') % 2
        o1 = bin(state & G1).count('1') % 2
        coded.append(o0)
        coded.append(o1)
    return coded


def to_soft(bit, mag=64):
    """Convert hard bit to soft LLR.

    Demapper convention: positive = likely 1 (before soft_pairer negation).
    bit=0 → constellation at +1 → demapper outputs positive Re.
    But wait — the soft_pairer negates. So we need to understand the
    actual convention used in the hardware.

    From the existing working system (SIGNAL decodes correctly):
    The demapper outputs sat16(Re) for BPSK. For a '0' bit, the TX puts +1,
    so received Re > 0 → demapper outputs positive value.
    soft_pairer negates: positive → negative for Viterbi.
    Viterbi convention: positive = likely 0.
    So after negation: negative (from demapper positive for bit=0) means
    the Viterbi sees negative = likely 1. That's WRONG for bit=0!

    Unless... the standard maps differently. Let me check:
    IEEE 802.11-2020 Table 17-6: BPSK I/Q output for bit 0 = -1, bit 1 = +1.
    Wait no — Table 17-6 says d_k=0 → I=-1, d_k=1 → I=+1 (for BPSK).

    So bit=0 → I=-1 → Re is negative → demapper outputs negative LLR.
    soft_pairer negates: negative → positive.
    Viterbi: positive = likely 0. ✓ That's correct!

    So: bit=0 → demapper output is NEGATIVE (since Re = -1 × norm)
        bit=1 → demapper output is POSITIVE (since Re = +1 × norm)

    The soft_pairer negates, giving Viterbi:
        bit=0 → positive (likely 0) ✓
        bit=1 → negative (likely 1) ✓

    For our test, we simulate the demapper output:
        coded_bit=0 → soft value = -mag (negative = the constellation was at -1)
        coded_bit=1 → soft value = +mag (positive = the constellation was at +1)
    """
    if bit == 0:
        return -mag  # transmitted as I=-1 → demapper outputs negative
    else:
        return mag   # transmitted as I=+1 → demapper outputs positive


def to_u8(val):
    """Convert signed 8-bit to unsigned for cocotb."""
    if val < 0:
        return val + 256
    return val & 0xFF


async def reset_dut(dut):
    """Reset the DUT."""
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


@cocotb.test()
async def test_rate12_multisymbol(dut):
    """Rate 1/2 multi-symbol decode through full chain.

    Simulates 5 BPSK symbols (rate 6): 48 coded bits per symbol with
    inter-symbol gap (like deinterleaver capture-then-emit behavior).
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Generate 120 data bits (5 symbols × 24 data bits/symbol)
    # Use pseudo-random but deterministic pattern
    import random
    random.seed(42)
    n_data_bits = 120
    data_bits = [random.randint(0, 1) for _ in range(n_data_bits)]

    # Encode at rate 1/2 (no puncturing)
    coded = conv_encode(data_bits)
    assert len(coded) == n_data_bits * 2  # 240 coded bits

    # Split into symbols of 48 coded bits each
    n_sym = len(coded) // 48
    assert n_sym == 5

    # Configure for rate 1/2
    dut.code_rate.value = 0  # rate 1/2 (passthrough)
    dut.streaming_mode.value = 1  # streaming DATA mode

    # Pulse frame_start to reset Viterbi and soft_pairer
    dut.frame_start.value = 1
    dut.valid_in.value = 0
    dut.soft_in0.value = 0
    dut.soft_in1.value = 0
    dut.flush_in.value = 0
    await RisingEdge(dut.clk)
    dut.frame_start.value = 0
    await ClockCycles(dut.clk, 5)

    # Feed symbols with inter-symbol gaps
    decoded = []
    sym_offset = 0

    for sym_idx in range(n_sym):
        sym_bits = coded[sym_offset:sym_offset + 48]
        sym_offset += 48

        # Feed 48 soft bits as 24 pairs (simulating deinterleaver output)
        for i in range(0, len(sym_bits), 2):
            dut.valid_in.value = 1
            dut.soft_in0.value = to_u8(to_soft(sym_bits[i]))
            dut.soft_in1.value = to_u8(to_soft(sym_bits[i + 1]))
            await RisingEdge(dut.clk)

            # Capture any outputs
            if int(dut.valid_out.value) == 1:
                decoded.append(int(dut.bit_out.value))

        # Inter-symbol gap (deinterleaver is in capture phase for next symbol)
        # Gap must be long enough for: depunct emit (~50 cycles) + vit_fifo latency
        # + Viterbi traceback (64 find_best + TB_DEPTH trace + TB_DEPTH output ~150 cycles)
        # Total: ~250 cycles needed when last symbol pair fills the window exactly.
        dut.valid_in.value = 0
        for _ in range(300):
            await RisingEdge(dut.clk)
            if int(dut.valid_out.value) == 1:
                decoded.append(int(dut.bit_out.value))

    # Flush to get remaining bits
    dut.flush_in.value = 1
    await RisingEdge(dut.clk)
    dut.flush_in.value = 0

    # Wait for flush to complete
    for _ in range(500):
        await RisingEdge(dut.clk)
        if int(dut.valid_out.value) == 1:
            decoded.append(int(dut.bit_out.value))

    # Compare
    dut._log.info(f"Decoded {len(decoded)} bits, expected {n_data_bits}")

    errors = sum(1 for a, b in zip(decoded, data_bits) if a != b)
    dut._log.info(f"Errors: {errors}/{min(len(decoded), n_data_bits)}")

    if errors > 0:
        # Show first few mismatches
        for i, (a, b) in enumerate(zip(decoded, data_bits)):
            if a != b:
                dut._log.info(f"  Mismatch at bit {i}: got {a}, expected {b}")
                if i > 10:
                    break

    assert len(decoded) >= n_data_bits, \
        f"Too few decoded bits: {len(decoded)} < {n_data_bits}"
    assert errors <= 2, \
        f"Rate 1/2 multi-symbol decode errors: {errors}/{n_data_bits}"

    dut._log.info(f"PASS: Rate 1/2 multi-symbol ({n_sym} symbols, {errors} errors)")


@cocotb.test()
async def test_rate12_long_frame(dut):
    """Rate 1/2 long frame (35 symbols, like rate 6 with 100 bytes).

    840 Viterbi pairs = 21 traceback windows at TB_DEPTH=40.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # 35 symbols × 24 data bits = 840 data bits
    # (matches rate 6, 100-byte PSDU: ceil((16+800+6)/24) = 35 symbols)
    import random
    random.seed(100)
    n_data_bits = 840
    data_bits = [random.randint(0, 1) for _ in range(n_data_bits)]

    coded = conv_encode(data_bits)
    assert len(coded) == 1680  # 840 * 2
    n_sym = len(coded) // 48
    assert n_sym == 35

    # Configure
    dut.code_rate.value = 0
    dut.streaming_mode.value = 1

    dut.frame_start.value = 1
    dut.valid_in.value = 0
    dut.soft_in0.value = 0
    dut.soft_in1.value = 0
    dut.flush_in.value = 0
    await RisingEdge(dut.clk)
    dut.frame_start.value = 0
    await ClockCycles(dut.clk, 5)

    decoded = []
    sym_offset = 0

    for sym_idx in range(n_sym):
        sym_bits = coded[sym_offset:sym_offset + 48]
        sym_offset += 48

        for i in range(0, len(sym_bits), 2):
            dut.valid_in.value = 1
            dut.soft_in0.value = to_u8(to_soft(sym_bits[i]))
            dut.soft_in1.value = to_u8(to_soft(sym_bits[i + 1]))
            await RisingEdge(dut.clk)
            if int(dut.valid_out.value) == 1:
                decoded.append(int(dut.bit_out.value))

        # Inter-symbol gap (must cover depunct emit + Viterbi traceback)
        dut.valid_in.value = 0
        for _ in range(300):
            await RisingEdge(dut.clk)
            if int(dut.valid_out.value) == 1:
                decoded.append(int(dut.bit_out.value))

    # Flush
    dut.flush_in.value = 1
    await RisingEdge(dut.clk)
    dut.flush_in.value = 0

    for _ in range(500):
        await RisingEdge(dut.clk)
        if int(dut.valid_out.value) == 1:
            decoded.append(int(dut.bit_out.value))

    dut._log.info(f"Decoded {len(decoded)} bits, expected {n_data_bits}")
    errors = sum(1 for a, b in zip(decoded, data_bits) if a != b)
    dut._log.info(f"Errors: {errors}/{min(len(decoded), n_data_bits)}")

    if errors > 0:
        first_err = next(i for i, (a, b) in enumerate(zip(decoded, data_bits)) if a != b)
        dut._log.info(f"  First error at bit {first_err}")

    assert len(decoded) >= n_data_bits, \
        f"Too few decoded bits: {len(decoded)} < {n_data_bits}"
    # With TB_DEPTH=48 and clean input, expect at most a few errors near
    # the flush boundary (last window has less convergence depth).
    assert errors <= 5, \
        f"Rate 1/2 long frame decode errors: {errors}/{n_data_bits}"

    dut._log.info(f"PASS: Rate 1/2 long frame ({n_sym} symbols, {errors} errors)")


def puncture_34(coded_bits):
    """Puncture at rate 3/4: pattern [1,1,1,0,0,1] applied to coded bits.

    Input: rate-1/2 coded bits (pairs: G0_0, G1_0, G0_1, G1_1, ...)
    Output: punctured bits (4 kept out of every 6)
    """
    pattern = [1, 1, 1, 0, 0, 1]
    pat_len = 6
    result = []
    for i, bit in enumerate(coded_bits):
        if pattern[i % pat_len]:
            result.append(bit)
    return result


@cocotb.test()
async def test_rate34_multisymbol(dut):
    """Rate 3/4 multi-symbol decode (simulates rate 9 BPSK 3/4).

    Same as rate 6 test but with puncturing.
    48 coded bits per symbol (after puncturing) → depuncture → 72 → Viterbi → 36 data bits.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Rate 9: n_dbps=36, 23 symbols for 100-byte PSDU
    # Use actual frame parameters: 23 symbols × 36 = 828 data bits
    # The last 6 must be tail zeros (as in real 802.11)
    import random
    random.seed(99)
    n_data_bits = 828
    data_bits = [random.randint(0, 1) for _ in range(n_data_bits)]
    # Force last 6 bits to 0 (convolutional tail — drives encoder to state 0)
    for i in range(6):
        data_bits[n_data_bits - 6 + i] = 0

    # Encode at rate 1/2 first
    coded_r12 = conv_encode(data_bits)
    assert len(coded_r12) == n_data_bits * 2  # 360 coded bits

    # Puncture to rate 3/4: keep 4 out of every 6
    punctured = puncture_34(coded_r12)
    # Expected: 1656 * 4/6 = 1104 punctured bits
    assert len(punctured) == 1104, f"Expected 1104 punctured bits, got {len(punctured)}"

    # Split into symbols of 48 coded bits (BPSK: 48 subcarriers × 1 bit)
    n_sym = len(punctured) // 48
    assert n_sym == 23

    # Configure for rate 3/4
    dut.code_rate.value = 2  # rate 3/4
    dut.streaming_mode.value = 1

    # Pulse frame_start
    dut.frame_start.value = 1
    dut.valid_in.value = 0
    dut.soft_in0.value = 0
    dut.soft_in1.value = 0
    dut.flush_in.value = 0
    await RisingEdge(dut.clk)
    dut.frame_start.value = 0
    await ClockCycles(dut.clk, 5)

    # Feed symbols with inter-symbol gaps
    decoded = []
    sym_offset = 0

    for sym_idx in range(n_sym):
        sym_bits = punctured[sym_offset:sym_offset + 48]
        sym_offset += 48

        # Feed 48 soft bits as 24 pairs
        for i in range(0, len(sym_bits), 2):
            dut.valid_in.value = 1
            dut.soft_in0.value = to_u8(to_soft(sym_bits[i]))
            dut.soft_in1.value = to_u8(to_soft(sym_bits[i + 1]))
            await RisingEdge(dut.clk)
            if int(dut.valid_out.value) == 1:
                decoded.append(int(dut.bit_out.value))

        # Inter-symbol gap
        dut.valid_in.value = 0
        for _ in range(300):
            await RisingEdge(dut.clk)
            if int(dut.valid_out.value) == 1:
                decoded.append(int(dut.bit_out.value))

    # Flush
    dut.flush_in.value = 1
    await RisingEdge(dut.clk)
    dut.flush_in.value = 0

    for _ in range(500):
        await RisingEdge(dut.clk)
        if int(dut.valid_out.value) == 1:
            decoded.append(int(dut.bit_out.value))

    dut._log.info(f"Decoded {len(decoded)} bits, expected {n_data_bits}")
    errors = sum(1 for a, b in zip(decoded, data_bits) if a != b)
    dut._log.info(f"Errors: {errors}/{min(len(decoded), n_data_bits)}")

    if errors > 0:
        first_err = next(i for i, (a, b) in enumerate(zip(decoded, data_bits)) if a != b)
        dut._log.info(f"  First error at bit {first_err}")
        # Show first 5 errors
        err_count = 0
        for i, (a, b) in enumerate(zip(decoded, data_bits)):
            if a != b:
                dut._log.info(f"  bit {i}: got {a}, expected {b}")
                err_count += 1
                if err_count >= 5:
                    break

    assert len(decoded) >= n_data_bits, \
        f"Too few decoded bits: {len(decoded)} < {n_data_bits}"
    assert errors <= 2, \
        f"Rate 3/4 multi-symbol decode errors: {errors}/{n_data_bits}"

    dut._log.info(f"PASS: Rate 3/4 multi-symbol ({n_sym} symbols, {errors} errors)")


@cocotb.test()
async def test_rate34_after_signal(dut):
    """Rate 3/4 decode after SIGNAL decode — mimics full pipeline flow.

    Simulates:
    1. SIGNAL at code_rate=0 (48 bits, flush mode)
    2. frame_start reset
    3. DATA at code_rate=2 (23 symbols × 48 bits, streaming mode)

    If this passes but full pipeline fails, bug is in equalizer/demapper output.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    import random
    random.seed(200)

    # --- SIGNAL phase (24 data bits, rate 1/2) ---
    sig_data = [random.randint(0, 1) for _ in range(24)]
    sig_coded = conv_encode(sig_data)  # 48 coded bits
    assert len(sig_coded) == 48

    # Configure for SIGNAL (rate 1/2, flush mode)
    dut.code_rate.value = 0
    dut.streaming_mode.value = 0  # flush mode

    dut.frame_start.value = 1
    dut.valid_in.value = 0
    dut.soft_in0.value = 0
    dut.soft_in1.value = 0
    dut.flush_in.value = 0
    await RisingEdge(dut.clk)
    dut.frame_start.value = 0
    await ClockCycles(dut.clk, 5)

    # Feed SIGNAL bits as 24 pairs
    for i in range(0, len(sig_coded), 2):
        dut.valid_in.value = 1
        dut.soft_in0.value = to_u8(to_soft(sig_coded[i]))
        dut.soft_in1.value = to_u8(to_soft(sig_coded[i + 1]))
        await RisingEdge(dut.clk)

    dut.valid_in.value = 0
    await ClockCycles(dut.clk, 100)

    # Flush SIGNAL
    dut.flush_in.value = 1
    await RisingEdge(dut.clk)
    dut.flush_in.value = 0
    await ClockCycles(dut.clk, 200)

    # --- DATA phase (rate 3/4, streaming) ---
    # Reset for DATA
    dut.code_rate.value = 2  # rate 3/4
    dut.streaming_mode.value = 1  # streaming mode

    dut.frame_start.value = 1
    await RisingEdge(dut.clk)
    dut.frame_start.value = 0
    await ClockCycles(dut.clk, 5)

    # Generate DATA: 828 data bits with tail
    n_data_bits = 828
    data_bits = [random.randint(0, 1) for _ in range(n_data_bits)]
    for i in range(6):
        data_bits[n_data_bits - 6 + i] = 0

    coded_r12 = conv_encode(data_bits)
    punctured = puncture_34(coded_r12)
    assert len(punctured) == 1104
    n_sym = 23

    decoded = []
    sym_offset = 0

    for sym_idx in range(n_sym):
        sym_bits = punctured[sym_offset:sym_offset + 48]
        sym_offset += 48

        for i in range(0, len(sym_bits), 2):
            dut.valid_in.value = 1
            dut.soft_in0.value = to_u8(to_soft(sym_bits[i]))
            dut.soft_in1.value = to_u8(to_soft(sym_bits[i + 1]))
            await RisingEdge(dut.clk)
            if int(dut.valid_out.value) == 1:
                decoded.append(int(dut.bit_out.value))

        dut.valid_in.value = 0
        for _ in range(300):
            await RisingEdge(dut.clk)
            if int(dut.valid_out.value) == 1:
                decoded.append(int(dut.bit_out.value))

    # Flush
    dut.flush_in.value = 1
    await RisingEdge(dut.clk)
    dut.flush_in.value = 0

    for _ in range(500):
        await RisingEdge(dut.clk)
        if int(dut.valid_out.value) == 1:
            decoded.append(int(dut.bit_out.value))

    dut._log.info(f"Decoded {len(decoded)} bits (DATA), expected {n_data_bits}")
    errors = sum(1 for a, b in zip(decoded, data_bits) if a != b)
    dut._log.info(f"Errors: {errors}/{min(len(decoded), n_data_bits)}")

    assert len(decoded) >= n_data_bits, \
        f"Too few decoded bits: {len(decoded)} < {n_data_bits}"
    assert errors <= 2, \
        f"Rate 3/4 after SIGNAL decode errors: {errors}/{n_data_bits}"

    dut._log.info(f"PASS: Rate 3/4 after SIGNAL ({errors} errors)")


@cocotb.test()
async def test_multiframe_rate34_no_reset_between(dut):
    """Six rate-3/4 frames back-to-back with only frame_start between them.

    A burst gives the chain frame_start per frame but never rst_n. The
    depuncturer is the one module in the chain with no per-frame reset:
    symbol_start is tied low (decode_chain.v:49, rx_pipeline.v:587), so
    its pair FIFO, pat_pos and group_active carry over between frames.

    That makes any residue left by frame N a decode error in frame N+1.
    This is the mechanism the 54M burst regression is attributed to, so
    it needs a gate independent of the Viterbi's backpressure behaviour.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    import random
    random.seed(4242)

    dut.code_rate.value = 2  # rate 3/4
    dut.streaming_mode.value = 1
    dut.valid_in.value = 0
    dut.soft_in0.value = 0
    dut.soft_in1.value = 0
    dut.flush_in.value = 0

    N_FRAMES = 6
    n_data_bits = 300
    per_frame_errors = []

    for frame in range(N_FRAMES):
        data_bits = [random.randint(0, 1) for _ in range(n_data_bits)]
        for i in range(6):
            data_bits[n_data_bits - 6 + i] = 0
        punct = puncture_34(conv_encode(data_bits))

        # Per-frame reset only -- deliberately no rst_n between frames.
        dut.frame_start.value = 1
        await RisingEdge(dut.clk)
        dut.frame_start.value = 0
        await ClockCycles(dut.clk, 5)

        decoded = []
        for i in range(0, len(punct) - 1, 2):
            while int(dut.upstream_full.value) == 1:
                dut.valid_in.value = 0
                await RisingEdge(dut.clk)
                await Timer(1, unit="ns")
                if int(dut.valid_out.value) == 1:
                    decoded.append(int(dut.bit_out.value))
            dut.valid_in.value = 1
            dut.soft_in0.value = to_u8(to_soft(punct[i]))
            dut.soft_in1.value = to_u8(to_soft(punct[i + 1]))
            await RisingEdge(dut.clk)
            await Timer(1, unit="ns")
            dut.valid_in.value = 0
            if int(dut.valid_out.value) == 1:
                decoded.append(int(dut.bit_out.value))

        dut.flush_in.value = 1
        await RisingEdge(dut.clk)
        dut.flush_in.value = 0
        for _ in range(4000):
            await RisingEdge(dut.clk)
            if int(dut.valid_out.value) == 1:
                decoded.append(int(dut.bit_out.value))
            if len(decoded) >= n_data_bits:
                break

        assert len(decoded) >= n_data_bits, (
            f"frame {frame}: too few decoded bits ({len(decoded)} < {n_data_bits}); "
            f"state leaked from the previous frame")
        errors = sum(1 for a, b in zip(decoded, data_bits) if a != b)
        per_frame_errors.append(errors)
        dut._log.info(f"frame {frame}: {errors}/{n_data_bits} errors")

    assert all(e <= 2 for e in per_frame_errors), (
        f"per-frame decode errors across a burst: {per_frame_errors} "
        f"(frame 0 clean but later frames dirty = depuncturer state leak)")
    dut._log.info(f"PASS: {N_FRAMES} back-to-back rate-3/4 frames, "
                  f"errors {per_frame_errors}")


@cocotb.test()
async def test_stall_stress_rate34_long_frame(dut):
    """Long 3/4 frame at max feed rate: backpressure engages, decode correct.

    The Viterbi's own traceback stalls (~160 clocks per TB window) force
    vit_fifo full -> the stall chain must hold the stream without losing
    a single pair. The test asserts the backpressure actually engaged.

    Backpressure handshake uses post-edge reads (Timer(1ns)) so the
    upstream_full check matches what the depuncturer samples at the next
    edge — pre-edge reads lose one pair per FIFO-full episode.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    import random
    random.seed(7)
    n_data_bits = 1000
    data_bits = [random.randint(0, 1) for _ in range(n_data_bits)]
    coded = conv_encode(data_bits)
    punct = puncture_34(coded)
    assert len(punct) % 2 == 0

    dut.code_rate.value = 2  # rate 3/4
    dut.streaming_mode.value = 1
    dut.frame_start.value = 1
    dut.valid_in.value = 0
    dut.soft_in0.value = 0
    dut.soft_in1.value = 0
    dut.flush_in.value = 0
    await RisingEdge(dut.clk)
    dut.frame_start.value = 0
    await ClockCycles(dut.clk, 5)

    decoded = []
    stall_cycles = 0
    for i in range(0, len(punct), 2):
        while int(dut.upstream_full.value) == 1:
            dut.valid_in.value = 0
            stall_cycles += 1
            await RisingEdge(dut.clk)
            await Timer(1, unit="ns")
            if int(dut.valid_out.value) == 1:
                decoded.append(int(dut.bit_out.value))
        dut.valid_in.value = 1
        dut.soft_in0.value = to_u8(to_soft(punct[i]))
        dut.soft_in1.value = to_u8(to_soft(punct[i + 1]))
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        dut.valid_in.value = 0
        if int(dut.valid_out.value) == 1:
            decoded.append(int(dut.bit_out.value))

    dut.flush_in.value = 1
    await RisingEdge(dut.clk)
    dut.flush_in.value = 0
    for _ in range(4000):
        await RisingEdge(dut.clk)
        if int(dut.valid_out.value) == 1:
            decoded.append(int(dut.bit_out.value))
        if len(decoded) >= n_data_bits:
            break

    assert stall_cycles > 0, "backpressure never engaged -- stress test is vacuous"
    dut._log.info(f"backpressure engaged: {stall_cycles} stall cycles")
    errors = sum(1 for a, b in zip(decoded, data_bits) if a != b)
    assert len(decoded) >= n_data_bits, f"too few decoded bits: {len(decoded)}"
    assert errors <= 10, f"decode errors under backpressure: {errors}/{n_data_bits}"
    dut._log.info(f"PASS: stall stress rate 3/4 ({errors} errors, "
                  f"{stall_cycles} stall cycles)")
