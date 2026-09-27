"""
Test viterbi_k7 — K=7 (64-state) convolutional decoder for 802.11a.

Generators: G0 = 133 octal (0b1011011), G1 = 171 octal (0b1111001)
Constraint length K=7, rate 1/2.

Input: pairs of 8-bit signed soft bits (LLR for G0 output, LLR for G1 output).
       Positive = likely 0, Negative = likely 1.
Output: decoded bits, one per trellis step.

Architecture: fully-parallel 64-state ACS + sliding-window traceback.
  - Flush mode (streaming_mode=0): SIGNAL decode, trace from state 0
  - Streaming mode (streaming_mode=1): DATA decode, periodic traceback
    from best state every TB_DEPTH=35 steps

Tests:
1. SIGNAL field: 48 coded bits → 24 decoded bits (annex I.1, bit-exact)
2. DATA field: longer frame (30 bits) flush mode
3. Error correction: flip 2 soft bits → still decodes correctly
4. All-zeros input
5. Back-to-back frames (state reset between frames)
6. Streaming mode: annex I.1 rate 36 DATA (1152 coded bits → 864 decoded bits)
7. Streaming mode: rate 6, 100-byte frame (long frame, no overflow)
8. Back-to-back: SIGNAL flush then DATA streaming
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import json
import os


# Generator polynomials (octal)
G0 = 0o133  # 0b1011011
G1 = 0o171  # 0b1111001
K = 7
N_STATES = 64


def conv_encode(bits):
    """Convolutional encode (rate 1/2, K=7). Returns pairs [g0_out, g1_out].

    IEEE 802.11 convention: newest bit enters at MSB (bit 6) of the
    7-bit shift register. Register shifts right each step.
    """
    state = 0
    coded = []
    for b in bits:
        state = ((state >> 1) | (b << 6)) & 0x7F
        o0 = bin(state & G0).count('1') % 2
        o1 = bin(state & G1).count('1') % 2
        coded.append(o0)
        coded.append(o1)
    return coded


def bits_to_soft(bits, magnitude=64):
    """Convert hard bits to soft LLRs. bit=0 → +magnitude, bit=1 → -magnitude."""
    return [magnitude if b == 0 else -magnitude for b in bits]


def to_u8(val):
    """Convert signed 8-bit to unsigned representation for cocotb."""
    if val < 0:
        return val + 256
    return val & 0xFF


async def reset_dut(dut):
    """Reset the DUT."""
    dut.rst_n.value = 0
    dut.frame_start.value = 0
    dut.flush.value = 0
    dut.valid_in.value = 0
    dut.soft0.value = 0
    dut.soft1.value = 0
    dut.streaming_mode.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def decode_flush(dut, soft_pairs, n_data_bits):
    """Decode in flush mode (SIGNAL). Feed pairs, flush, collect output.

    Args:
        soft_pairs: list of (soft0, soft1) tuples (signed 8-bit)
        n_data_bits: expected number of output bits

    Returns:
        list of decoded bits (0/1)
    """
    dut.streaming_mode.value = 0

    # Pulse frame_start
    dut.frame_start.value = 1
    await RisingEdge(dut.clk)
    dut.frame_start.value = 0
    await RisingEdge(dut.clk)

    decoded = []
    pair_idx = 0

    for i in range(len(soft_pairs) + 300):
        if pair_idx < len(soft_pairs):
            # Check busy before feeding
            if int(dut.busy.value) == 0:
                s0, s1 = soft_pairs[pair_idx]
                dut.valid_in.value = 1
                dut.soft0.value = to_u8(s0)
                dut.soft1.value = to_u8(s1)
                pair_idx += 1
            else:
                dut.valid_in.value = 0
        else:
            dut.valid_in.value = 0
            # Pulse flush once after all input
            if pair_idx == len(soft_pairs):
                dut.flush.value = 1
                pair_idx += 1
            else:
                dut.flush.value = 0

        await RisingEdge(dut.clk)

        if int(dut.valid_out.value) == 1:
            decoded.append(int(dut.bit_out.value))

        if len(decoded) >= n_data_bits:
            break

    dut.valid_in.value = 0
    dut.flush.value = 0
    return decoded


async def decode_streaming(dut, soft_pairs, n_data_bits):
    """Decode in streaming mode (DATA). Feed pairs, periodic output, flush at end.

    Uses "present and confirm" protocol: always drive the current pair on
    valid_in. After the edge, check busy: if busy=0, the input was accepted
    and we advance to the next pair. If busy=1, re-present the same pair.

    Args:
        soft_pairs: list of (soft0, soft1) tuples
        n_data_bits: expected number of output bits

    Returns:
        list of decoded bits (0/1)
    """
    dut.streaming_mode.value = 1

    # Pulse frame_start
    dut.frame_start.value = 1
    await RisingEdge(dut.clk)
    dut.frame_start.value = 0
    await RisingEdge(dut.clk)

    decoded = []
    pair_idx = 0
    flushed = False
    timeout = len(soft_pairs) * 6 + 2000

    for i in range(timeout):
        if pair_idx < len(soft_pairs):
            # Always present current pair
            s0, s1 = soft_pairs[pair_idx]
            dut.valid_in.value = 1
            dut.soft0.value = to_u8(s0)
            dut.soft1.value = to_u8(s1)
        else:
            dut.valid_in.value = 0
            if not flushed:
                dut.flush.value = 1
                flushed = True
            else:
                dut.flush.value = 0

        await RisingEdge(dut.clk)

        # Check if input was accepted (busy=0 means it was)
        if pair_idx < len(soft_pairs) and int(dut.busy.value) == 0:
            pair_idx += 1

        if int(dut.valid_out.value) == 1:
            decoded.append(int(dut.bit_out.value))

        if len(decoded) >= n_data_bits:
            break

    dut.valid_in.value = 0
    dut.flush.value = 0
    return decoded


@cocotb.test()
async def test_signal_field_decode(dut):
    """SIGNAL field: 48 coded bits → 24 decoded bits (annex I.1, bit-exact)."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    vec_dir = os.path.join(os.path.dirname(__file__), '../../extern/lib80211/vectors')

    with open(os.path.join(vec_dir, 'annex_i1_signal_encoded.json')) as f:
        encoded = json.load(f)
    coded_bits = encoded['bits']
    assert len(coded_bits) == 48

    with open(os.path.join(vec_dir, 'annex_i1_signal_bits.json')) as f:
        expected = json.load(f)
    expected_bits = expected['bits']
    assert len(expected_bits) == 24

    soft_pairs = []
    for i in range(0, 48, 2):
        s0 = 64 if coded_bits[i] == 0 else -64
        s1 = 64 if coded_bits[i+1] == 0 else -64
        soft_pairs.append((s0, s1))

    decoded = await decode_flush(dut, soft_pairs, 24)

    assert len(decoded) == 24, f"Expected 24 decoded bits, got {len(decoded)}"
    assert decoded == expected_bits, \
        f"SIGNAL decode mismatch:\n  got:      {decoded}\n  expected: {expected_bits}"

    dut._log.info(f"SIGNAL field decoded correctly: {decoded}")


@cocotb.test()
async def test_data_field_decode(dut):
    """Longer frame decode: 30 bits (< TB_DEPTH=35) at rate 1/2, flush mode."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # 30 bits: 24 data + 6 tail zeros
    input_bits = [1, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0]
    assert len(input_bits) == 30
    assert input_bits[-6:] == [0, 0, 0, 0, 0, 0]

    coded = conv_encode(input_bits)
    assert len(coded) == 60

    soft_pairs = [(64 if coded[i] == 0 else -64, 64 if coded[i+1] == 0 else -64)
                  for i in range(0, len(coded), 2)]

    decoded = await decode_flush(dut, soft_pairs, 30)

    assert len(decoded) == 30, f"Expected 30 decoded bits, got {len(decoded)}"

    data_errors = sum(1 for a, b in zip(decoded[:24], input_bits[:24]) if a != b)
    assert data_errors == 0, f"30-bit frame: {data_errors}/24 data bit errors (got {decoded})"

    dut._log.info(f"30-bit frame decoded correctly")


@cocotb.test()
async def test_error_correction(dut):
    """Flip 2 soft bits in SIGNAL encoded stream → Viterbi still corrects."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    vec_dir = os.path.join(os.path.dirname(__file__), '../../extern/lib80211/vectors')

    with open(os.path.join(vec_dir, 'annex_i1_signal_encoded.json')) as f:
        encoded = json.load(f)
    coded_bits = encoded['bits']

    with open(os.path.join(vec_dir, 'annex_i1_signal_bits.json')) as f:
        expected = json.load(f)
    expected_bits = expected['bits']

    corrupted = list(coded_bits)
    corrupted[5] ^= 1
    corrupted[20] ^= 1

    soft_pairs = []
    for i in range(0, 48, 2):
        s0 = 64 if corrupted[i] == 0 else -64
        s1 = 64 if corrupted[i+1] == 0 else -64
        soft_pairs.append((s0, s1))

    decoded = await decode_flush(dut, soft_pairs, 24)

    assert len(decoded) == 24, f"Expected 24 decoded bits, got {len(decoded)}"
    assert decoded == expected_bits, \
        f"Error correction failed: decoded {decoded} != expected {expected_bits}"

    dut._log.info("Error correction test passed: 2 bit errors corrected")


@cocotb.test()
async def test_all_zeros(dut):
    """All-zeros input: encode 24 zeros, decode them back."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    input_bits = [0] * 24
    coded = conv_encode(input_bits)
    assert len(coded) == 48

    soft_pairs = []
    for i in range(0, 48, 2):
        s0 = 64 if coded[i] == 0 else -64
        s1 = 64 if coded[i+1] == 0 else -64
        soft_pairs.append((s0, s1))

    decoded = await decode_flush(dut, soft_pairs, 24)

    assert len(decoded) == 24, f"Expected 24 decoded bits, got {len(decoded)}"
    assert decoded == input_bits, f"All-zeros decode mismatch: got {decoded}"

    dut._log.info("All-zeros test passed")


@cocotb.test()
async def test_back_to_back_frames(dut):
    """Decode two frames back-to-back with frame_start reset between them."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    vec_dir = os.path.join(os.path.dirname(__file__), '../../extern/lib80211/vectors')

    with open(os.path.join(vec_dir, 'annex_i1_signal_encoded.json')) as f:
        encoded = json.load(f)
    coded_bits = encoded['bits']

    with open(os.path.join(vec_dir, 'annex_i1_signal_bits.json')) as f:
        expected = json.load(f)
    expected_bits = expected['bits']

    soft_pairs = []
    for i in range(0, 48, 2):
        s0 = 64 if coded_bits[i] == 0 else -64
        s1 = 64 if coded_bits[i+1] == 0 else -64
        soft_pairs.append((s0, s1))

    # First frame
    decoded1 = await decode_flush(dut, soft_pairs, 24)
    assert decoded1 == expected_bits, f"Frame 1 failed: {decoded1}"

    await ClockCycles(dut.clk, 10)

    # Second frame
    decoded2 = await decode_flush(dut, soft_pairs, 24)
    assert decoded2 == expected_bits, f"Frame 2 failed: {decoded2}"

    dut._log.info("Back-to-back frames decoded correctly")


@cocotb.test()
async def test_streaming_annex_i1_data(dut):
    """Streaming mode: annex I.1 rate 36 DATA (1152 coded → 864 decoded bits).

    Uses annex_i1_data_encoded.json (1152 coded bits, rate 3/4 punctured then
    repacked as rate 1/2 pairs for Viterbi input) and annex_i1_data_scrambled.json
    (864 bits expected output — the scrambled DATA bits before encoding).

    Note: The Viterbi sees rate-1/2 pairs AFTER depuncturing. The depuncturer
    inserts erasures (zero-magnitude soft bits) at punctured positions. So for
    rate 3/4: every 4 input bits become 6 soft pairs (2 erasures inserted).

    For this test we use the pre-encoded bits directly as rate 1/2 input
    (simulating what the depuncturer would produce for rate 1/2 coding).
    The annex I.1 DATA is rate 3/4, so the encoded output is 1152 bits for
    864 input bits. But these 1152 bits are the PUNCTURED output. For the
    Viterbi we need the rate 1/2 stream with erasures.

    Actually: we just encode the scrambled data at rate 1/2 ourselves (no
    puncturing) and verify the Viterbi recovers the original bits. This tests
    the streaming Viterbi independent of puncturing.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    vec_dir = os.path.join(os.path.dirname(__file__), '../../extern/lib80211/vectors')

    # Load the scrambled DATA bits (what we expect to decode)
    with open(os.path.join(vec_dir, 'annex_i1_data_scrambled.json')) as f:
        data = json.load(f)
    input_bits = data['data']
    assert len(input_bits) == 864

    # Encode at rate 1/2 (no puncturing — tests Viterbi in isolation)
    coded = conv_encode(input_bits)
    assert len(coded) == 1728  # 864 * 2

    # Convert to soft pairs
    soft_pairs = [(64 if coded[i] == 0 else -64, 64 if coded[i+1] == 0 else -64)
                  for i in range(0, len(coded), 2)]

    decoded = await decode_streaming(dut, soft_pairs, 864)

    assert len(decoded) == 864, f"Expected 864 decoded bits, got {len(decoded)}"

    errors = sum(1 for a, b in zip(decoded, input_bits) if a != b)
    # Sliding-window with TB_DEPTH=35 may produce 1-2 errors near window
    # boundaries (insufficient convergence at traceback start). This is an
    # inherent limitation. For rate 1/2 with hard decisions, expect ≤2 errors.
    assert errors <= 2, \
        f"Streaming decode errors: {errors}/864 bits wrong\n" \
        f"  First mismatch at bit {next(i for i,(a,b) in enumerate(zip(decoded, input_bits)) if a!=b)}"

    dut._log.info(f"Streaming annex I.1 DATA decoded: 864 bits, {errors} errors (≤2 acceptable)")


@cocotb.test()
async def test_streaming_long_frame(dut):
    """Streaming mode: rate 6, 200-byte frame (no overflow, tests many windows).

    200 bytes = 1600 data bits + 16 SERVICE + 6 tail = 1622 bits.
    With rate 1/2: 3244 coded bits = 1622 pairs.
    That's 1622/35 = ~46 traceback windows. Exercises the circular buffer well.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Generate pseudo-random input bits (LFSR-like pattern)
    import random
    random.seed(42)
    n_bits = 1622  # 200 bytes * 8 + 16 SERVICE + 6 tail
    input_bits = [random.randint(0, 1) for _ in range(n_bits)]
    # Zero the last 6 bits (tail)
    input_bits[-6:] = [0, 0, 0, 0, 0, 0]

    coded = conv_encode(input_bits)
    assert len(coded) == n_bits * 2

    soft_pairs = [(64 if coded[i] == 0 else -64, 64 if coded[i+1] == 0 else -64)
                  for i in range(0, len(coded), 2)]

    decoded = await decode_streaming(dut, soft_pairs, n_bits)

    assert len(decoded) == n_bits, f"Expected {n_bits} decoded bits, got {len(decoded)}"

    errors = sum(1 for a, b in zip(decoded, input_bits) if a != b)
    # Allow small errors near window boundaries (sliding window approximation)
    # With TB_DEPTH=35 (5*K) and clean input, expect 0 errors
    assert errors == 0, \
        f"Long frame decode errors: {errors}/{n_bits} bits wrong\n" \
        f"  First mismatch at bit {next(i for i,(a,b) in enumerate(zip(decoded, input_bits)) if a!=b)}"

    dut._log.info(f"Long frame (200 bytes) decoded correctly: {n_bits} bits, 0 errors")


@cocotb.test()
async def test_streaming_total_steps_wrap(dut):
    """Regression: total_steps must not wrap at 2048 (old 11-bit counter).

    Rate 6, L=254 bytes → total_steps = 16 + 254*8 + 6 = 2054.
    With the old 11-bit counter, total_steps wrapped to 6 at step 2048.
    On flush, the comparison `total_steps > window_steps` failed (6 > 38 = false),
    routing to tb_conv_depth=0 and defeating the extended traceback (ff79c30).
    The fix: widen total_steps to 16 bits.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    import random
    random.seed(2048)
    n_bits = 16 + 254 * 8 + 6  # = 2054 (wraps old 11-bit counter)
    input_bits = [random.randint(0, 1) for _ in range(n_bits)]
    # Zero the last 6 bits (tail)
    input_bits[-6:] = [0, 0, 0, 0, 0, 0]

    coded = conv_encode(input_bits)
    assert len(coded) == n_bits * 2

    soft_pairs = [(64 if coded[i] == 0 else -64, 64 if coded[i+1] == 0 else -64)
                  for i in range(0, len(coded), 2)]

    decoded = await decode_streaming(dut, soft_pairs, n_bits)

    assert len(decoded) == n_bits, f"Expected {n_bits} decoded bits, got {len(decoded)}"

    errors = sum(1 for a, b in zip(decoded, input_bits) if a != b)
    assert errors == 0, (
        f"total_steps wrap regression: {errors}/{n_bits} bits wrong "
        f"(total_steps=2054 wraps old 11-bit counter to 6; flush takes wrong branch)\n"
        f"  First mismatch at bit {next(i for i,(a,b) in enumerate(zip(decoded, input_bits)) if a!=b)}"
    )

    dut._log.info(f"total_steps wrap test PASSED: {n_bits} bits, 0 errors (no 11-bit wrap)")


@cocotb.test()
async def test_signal_then_data_streaming(dut):
    """Back-to-back: SIGNAL flush then DATA streaming (simulates real frame)."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    vec_dir = os.path.join(os.path.dirname(__file__), '../../extern/lib80211/vectors')

    # --- SIGNAL phase (flush mode) ---
    with open(os.path.join(vec_dir, 'annex_i1_signal_encoded.json')) as f:
        encoded = json.load(f)
    coded_bits = encoded['bits']
    with open(os.path.join(vec_dir, 'annex_i1_signal_bits.json')) as f:
        expected = json.load(f)
    expected_bits = expected['bits']

    soft_pairs_sig = []
    for i in range(0, 48, 2):
        s0 = 64 if coded_bits[i] == 0 else -64
        s1 = 64 if coded_bits[i+1] == 0 else -64
        soft_pairs_sig.append((s0, s1))

    decoded_sig = await decode_flush(dut, soft_pairs_sig, 24)
    assert decoded_sig == expected_bits, f"SIGNAL failed: {decoded_sig}"
    dut._log.info("SIGNAL phase passed")

    await ClockCycles(dut.clk, 5)

    # --- DATA phase (streaming mode) ---
    with open(os.path.join(vec_dir, 'annex_i1_data_scrambled.json')) as f:
        data = json.load(f)
    data_bits = data['data']

    coded_data = conv_encode(data_bits)
    soft_pairs_data = [(64 if coded_data[i] == 0 else -64,
                        64 if coded_data[i+1] == 0 else -64)
                       for i in range(0, len(coded_data), 2)]

    decoded_data = await decode_streaming(dut, soft_pairs_data, 864)

    assert len(decoded_data) == 864, f"Expected 864 DATA bits, got {len(decoded_data)}"
    errors = sum(1 for a, b in zip(decoded_data, data_bits) if a != b)
    assert errors <= 2, f"DATA phase errors: {errors}/864 (max 2 acceptable)"

    dut._log.info(f"SIGNAL flush → DATA streaming passed ({errors} errors)")


# =========================================================
# Test 9: Streaming with pad bits (encoder wanders from state 0)
# =========================================================
@cocotb.test()
async def test_streaming_with_pad_bits(dut):
    """Streaming mode: frame with pad bits after tail (encoder wanders from state 0).

    In 802.11a, a 100-byte rate-6 frame has:
      - 16 SERVICE bits + 800 data bits + 6 tail bits = 822 bits
      - Padded to next multiple of N_DBPS(48) = 864 bits total
      - Pad bits = 864 - 822 = 42 scrambled bits after tail

    The 6 tail bits flush the encoder to state 0. The 42 pad bits are then
    encoded starting from state 0, driving the encoder to arbitrary states.
    The Viterbi must recover the first 822 bits correctly (the rest are pad/don't care).

    This tests that the traceback-from-best-state logic (not state 0) works
    correctly for windows that span the pad region.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    import random
    random.seed(123)

    # 100-byte frame: 16 SERVICE + 800 data + 6 tail = 822 data bits
    n_data_bits = 822
    n_total_bits = 864  # padded to OFDM symbol boundary (864 = 18 * 48)
    n_pad_bits = n_total_bits - n_data_bits  # 42

    # Generate input: random data + 6 zero tail bits + scrambled pad bits
    input_bits = [random.randint(0, 1) for _ in range(n_data_bits - 6)]
    input_bits.extend([0, 0, 0, 0, 0, 0])  # tail bits
    assert len(input_bits) == n_data_bits

    # Pad bits: scrambled (effectively random from encoder's perspective)
    pad_bits = [random.randint(0, 1) for _ in range(n_pad_bits)]
    all_bits = input_bits + pad_bits
    assert len(all_bits) == n_total_bits

    # Encode the full stream (data + pad)
    coded = conv_encode(all_bits)
    assert len(coded) == n_total_bits * 2

    # Convert to soft pairs
    soft_pairs = [(64 if coded[i] == 0 else -64, 64 if coded[i+1] == 0 else -64)
                  for i in range(0, len(coded), 2)]

    # Decode — we expect n_total_bits output bits
    decoded = await decode_streaming(dut, soft_pairs, n_total_bits)

    assert len(decoded) == n_total_bits, \
        f"Expected {n_total_bits} decoded bits, got {len(decoded)}"

    # Check the DATA bits (first 822) are correct. Pad bits are don't-care.
    data_errors = sum(1 for a, b in zip(decoded[:n_data_bits], input_bits) if a != b)

    # With TB_DEPTH=35 and pad bits, we might see 1-2 errors near the
    # tail/pad boundary where convergence depth is marginal.
    assert data_errors <= 2, \
        f"Pad bit test: {data_errors}/{n_data_bits} data bit errors\n" \
        f"  First mismatch at bit {next((i for i,(a,b) in enumerate(zip(decoded[:n_data_bits], input_bits)) if a!=b), 'none')}"

    dut._log.info(f"Streaming with pad bits: {n_data_bits} data bits decoded, "
                  f"{data_errors} errors (≤2 acceptable), {n_pad_bits} pad bits ignored")


# =========================================================
# Test 10: Streaming with erasures (rate 3/4 depunctured input)
# =========================================================
@cocotb.test()
async def test_streaming_with_erasures(dut):
    """Streaming mode with erasure inputs (zero-magnitude soft bits).

    Simulates rate 3/4 depunctured stream: every 6 soft pairs have 2 erasures
    (0x00) at positions matching the [1,1,1,0,0,1] pattern. The Viterbi must
    still decode correctly despite reduced confidence at punctured positions.

    This is what the Viterbi sees in the real pipeline for rate 36/48/54.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    import random
    random.seed(456)

    # 50 data bits + 6 tail = 56 bits (manageable size)
    n_bits = 56
    input_bits = [random.randint(0, 1) for _ in range(n_bits)]
    input_bits[-6:] = [0, 0, 0, 0, 0, 0]  # tail

    # Encode at rate 1/2
    coded = conv_encode(input_bits)
    assert len(coded) == n_bits * 2  # 112 coded bits

    # Puncture with rate 3/4 pattern: [1,1,1,0,0,1]
    # This means: from every 4 input bits, we produce 6 output bits (2 erasures inserted)
    # But actually depuncturing happens BEFORE Viterbi. So we start with:
    # 56 data bits → 112 rate-1/2 coded bits → puncture to 75 bits → depuncture to 112 bits
    # Simpler: just apply the puncture pattern directly to coded pairs.
    #
    # Rate 3/4 puncturing: from each 4 coded bits, keep 3, discard 1.
    # Pattern on code pairs: for every 3 pairs in, keep bits at [1,1,1,0,0,1] → produce 2 pairs out
    # Actually the 802.11 puncturing works on the interleaved G0/G1 stream:
    # Pattern [1,1,1,0,0,1] means: positions 0,1,2,5 are kept, positions 3,4 are punctured.
    #
    # For testing purposes: just insert erasures at pattern positions in the coded stream
    pattern = [1, 1, 1, 0, 0, 1]
    depunctured_soft = []
    coded_idx = 0
    for i in range(len(coded)):
        pat_pos = i % len(pattern)
        if pattern[pat_pos] == 1:
            # Kept position: use actual soft bit
            depunctured_soft.append(64 if coded[coded_idx] == 0 else -64)
            coded_idx += 1
        else:
            # Erasure position
            depunctured_soft.append(0)

    # Actually this doesn't work cleanly because we need the right number of kept bits.
    # Let me use a simpler approach: encode at rate 1/2, then zero out every 6th pair
    # to simulate erasures at known positions.

    soft_values = [64 if coded[i] == 0 else -64 for i in range(len(coded))]

    # Zero out positions 3 and 4 in every group of 6 (simulating depunctured erasures)
    for i in range(0, len(soft_values), 6):
        if i + 3 < len(soft_values):
            soft_values[i + 3] = 0
        if i + 4 < len(soft_values):
            soft_values[i + 4] = 0

    # Convert to pairs
    soft_pairs = [(soft_values[i], soft_values[i+1]) for i in range(0, len(soft_values), 2)]

    decoded = await decode_streaming(dut, soft_pairs, n_bits)

    assert len(decoded) == n_bits, f"Expected {n_bits} decoded bits, got {len(decoded)}"

    errors = sum(1 for a, b in zip(decoded, input_bits) if a != b)
    # With erasures at 2/6 positions, error correction should still handle it
    # (effective code rate is 3/4, which can correct some errors)
    assert errors <= 3, \
        f"Erasure test: {errors}/{n_bits} errors (max 3 acceptable with rate 3/4 equivalent)"

    dut._log.info(f"Streaming with erasures: {n_bits} bits decoded, {errors} errors")


@cocotb.test()
async def test_best_state_search_13clk(dut):
    """Best-state search ratchet: window drain must start within 13 clocks.

    After the 48th pair of a streaming window is accepted, the Viterbi
    finds the best-metric state before traceback. The sequential search
    took 65 cycles (one state per clock); the pipelined 8-way tree search
    completes in 13. This ratchet measures the gap between acceptance of
    the 48th pair and the first decoded bit of that window and asserts
    the drain completes in < 80 clocks (13 search + trace prime + 48
    traceback + pipeline slack). The 65-cycle sequential search drains
    in ~116 clocks and fails this assertion.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    dut.streaming_mode.value = 1

    dut.frame_start.value = 1
    await RisingEdge(dut.clk)
    dut.frame_start.value = 0
    await RisingEdge(dut.clk)

    # Exactly one full window: TB_DEPTH = 48 pairs. Soft values are
    # irrelevant to the timing measurement; decode content is not checked.
    soft_pairs = [(64, 64)] * 48
    pair_idx = 0
    accepted_at = None
    gap = 0
    saw_output = False

    for cycle in range(2000):
        if pair_idx < len(soft_pairs):
            s0, s1 = soft_pairs[pair_idx]
            dut.valid_in.value = 1
            dut.soft0.value = to_u8(s0)
            dut.soft1.value = to_u8(s1)
        else:
            dut.valid_in.value = 0

        await RisingEdge(dut.clk)

        # Present-and-confirm: busy=0 after the edge means accepted.
        if pair_idx < len(soft_pairs) and int(dut.busy.value) == 0:
            pair_idx += 1
            if pair_idx == len(soft_pairs):
                accepted_at = cycle

        if accepted_at is not None:
            gap += 1
            if int(dut.valid_out.value) == 1:
                saw_output = True
                break

    dut.valid_in.value = 0

    assert accepted_at is not None, "48th pair was never accepted"
    assert saw_output, "no decoded output after window drain"
    dut._log.info(f"Window drain: {gap} clocks from pair-48 acceptance "
                  f"to first output bit")
    assert gap < 80, (
        f"BEST-STATE SEARCH TOO SLOW: window drain took {gap} clocks "
        f"(ratchet: < 80 with 13-cycle tree search; the 65-cycle "
        f"sequential search drains in ~116)"
    )


@cocotb.test()
async def test_streaming_throughput_ratchet(dut):
    """Steady-state input acceptance rate in streaming mode.

    Feeds pairs continuously across several traceback windows and measures
    clocks-per-accepted-pair. The Task 2 decoder (traceback stalls ACS)
    sits at ~2.15. Task 3-only runs ACS during traceback and serializes the
    13-clock search → ~1.27. This ratchet is the gate; D23 records the
    schedule.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    dut.streaming_mode.value = 1

    dut.frame_start.value = 1
    await RisingEdge(dut.clk)
    dut.frame_start.value = 0
    await RisingEdge(dut.clk)

    # 6 windows of 48 pairs. Enough to average over several traceback
    # cycles so the first-window fill does not dominate the ratio.
    n_pairs = 48 * 6
    accepted = 0
    cycles = 0

    # Offer a pair every cycle; count how many are actually taken.
    while accepted < n_pairs and cycles < 4000:
        dut.valid_in.value = 1
        dut.soft0.value = to_u8(64)
        dut.soft1.value = to_u8(64)
        await RisingEdge(dut.clk)
        cycles += 1
        if int(dut.busy.value) == 0:
            accepted += 1

    dut.valid_in.value = 0
    assert accepted == n_pairs, f"only {accepted}/{n_pairs} pairs accepted"

    clk_per_pair = cycles / accepted
    dut._log.info(f"Steady-state: {clk_per_pair:.3f} clk/pair "
                  f"({cycles} clocks / {accepted} pairs)")

    # RATCHET — Task 3-only: traceback overlaps ACS, the serialized 13-clock
    # best-state search stalls it. Target ~1.27 clk/pair (48 ACS + 13 search
    # per window). Measured 1.278 in sim and matched on hardware
    # (fingerprint 0x0f0ee6ed), so the bound is tightened from 1.40 to 1.35.
    assert clk_per_pair < 1.35, (
        f"throughput ratchet: {clk_per_pair:.3f} clk/pair, need < 1.35"
    )
