"""
Test descrambler -- 802.11 LFSR descrambler (x^7 + x^4 + 1).

Self-synchronizing: first 7 input bits (scrambled SERVICE zeros) are used
to initialize the LFSR. From bit 7 onward, normal LFSR feedback is used.

Algorithm:
  - Bits 0-6: feedback = input_bit (self-sync), output = 0
  - Bits 7+:  feedback = state[6] ^ state[3], output = input ^ feedback
  - State always: {state[5:0], feedback}

Interface:
  frame_start: pulse to reset for new frame
  valid_in + bit_in: serial decoded bits from Viterbi (one per clock)
  valid_out + bit_out: descrambled bits (one per clock)
  seed_detected[6:0]: LFSR state after 7 bits (valid when seed_valid=1)

Tests:
  1. Annex I.1 golden vector: 864 scrambled bits → 16 SERVICE zeros + 800 PSDU bits
  2. Seed detection: verify seed_detected matches known seed state
  3. All-zeros input (edge case)
  4. Back-to-back frames (frame_start resets properly)
  5. Different scrambler seed (synthetic vector)
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import json
import os

VECTORS = os.path.join(os.path.dirname(__file__), '..', '..', 'extern', 'lib80211', 'vectors')


def descramble_reference(scrambled_bits, seed=None):
    """Reference descrambler matching the self-sync HW implementation.

    If seed is None, uses self-sync (first 7 bits as feedback).
    Returns (descrambled_bits, detected_state_after_7).
    """
    state = 0  # initial state doesn't matter for self-sync
    descrambled = []

    for i in range(len(scrambled_bits)):
        if i < 7:
            feedback = scrambled_bits[i]
        else:
            feedback = ((state >> 6) ^ (state >> 3)) & 1

        descrambled.append(scrambled_bits[i] ^ feedback)
        state = ((state << 1) | feedback) & 0x7F

    # State after 7 bits of self-sync
    detected_state = 0
    for i in range(7):
        detected_state = ((detected_state << 1) | scrambled_bits[i]) & 0x7F

    return descrambled, detected_state


def scramble_with_seed(data_bits, seed):
    """Scramble bits with known seed (for generating test vectors)."""
    state = seed & 0x7F
    scrambled = []
    for bit in data_bits:
        feedback = ((state >> 6) ^ (state >> 3)) & 1
        scrambled.append(bit ^ feedback)
        state = ((state << 1) | feedback) & 0x7F
    return scrambled


async def reset_dut(dut):
    """Reset the DUT."""
    dut.rst_n.value = 0
    dut.valid_in.value = 0
    dut.bit_in.value = 0
    dut.frame_start.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def feed_and_collect(dut, bits_in, use_frame_start=True):
    """Feed bits to descrambler and collect outputs.

    Returns (output_bits, seed_detected, seed_valid_seen).
    """
    if use_frame_start:
        dut.frame_start.value = 1
        await RisingEdge(dut.clk)
        dut.frame_start.value = 0
        await RisingEdge(dut.clk)

    output_bits = []
    seed_detected = None
    seed_valid_seen = False

    for bit in bits_in:
        dut.valid_in.value = 1
        dut.bit_in.value = bit
        await RisingEdge(dut.clk)

        # Check outputs (1 cycle latency due to registered output)
        if int(dut.valid_out.value) == 1:
            output_bits.append(int(dut.bit_out.value))

        if int(dut.seed_valid.value) == 1 and not seed_valid_seen:
            seed_detected = int(dut.seed_detected.value)
            seed_valid_seen = True

    # Stop driving and collect remaining outputs
    dut.valid_in.value = 0
    for _ in range(10):
        await RisingEdge(dut.clk)
        if int(dut.valid_out.value) == 1:
            output_bits.append(int(dut.bit_out.value))
        if int(dut.seed_valid.value) == 1 and not seed_valid_seen:
            seed_detected = int(dut.seed_detected.value)
            seed_valid_seen = True

    return output_bits, seed_detected, seed_valid_seen


@cocotb.test()
async def test_annex_i1_golden_vector(dut):
    """Annex I.1: 864 scrambled bits → SERVICE zeros + PSDU match.

    Golden vector: annex_i1_data_scrambled.json (seed 0x5D = 1011101)
    Expected: first 16 bits = 0, bits 16-815 = PSDU from annex_i1_psdu.json
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Load scrambled data
    with open(os.path.join(VECTORS, 'annex_i1_data_scrambled.json')) as f:
        vec = json.load(f)
    scrambled = vec['data']
    assert len(scrambled) == 864

    # Load expected PSDU
    with open(os.path.join(VECTORS, 'annex_i1_psdu.json')) as f:
        psdu_vec = json.load(f)
    psdu_hex = psdu_vec['octets_hex']
    psdu_bits = []
    for h in psdu_hex:
        byte = int(h, 16)
        for bit in range(8):
            psdu_bits.append((byte >> bit) & 1)
    assert len(psdu_bits) == 800

    # Run through DUT
    output_bits, seed_det, seed_valid = await feed_and_collect(dut, scrambled)

    assert len(output_bits) == 864, \
        f"Expected 864 output bits, got {len(output_bits)}"

    # Check SERVICE field (bits 0-15 should be all zeros)
    service = output_bits[:16]
    assert all(b == 0 for b in service), \
        f"SERVICE field not all zeros: {service}"
    dut._log.info("SERVICE field: all 16 bits = 0 (correct)")

    # Check PSDU (bits 16-815)
    psdu_out = output_bits[16:816]
    errors = 0
    first_error = -1
    for i in range(800):
        if psdu_out[i] != psdu_bits[i]:
            if first_error < 0:
                first_error = i
            errors += 1

    assert errors == 0, \
        f"PSDU mismatch: {errors}/800 errors, first at bit {first_error}"
    dut._log.info("PSDU: 800/800 bits correct (100 bytes match annex_i1_psdu.json)")

    # Verify reference implementation agrees
    ref_out, ref_seed = descramble_reference(scrambled)
    assert ref_out[:16] == [0]*16
    assert ref_out[16:816] == psdu_bits


@cocotb.test()
async def test_seed_detection(dut):
    """Seed detection: verify seed_detected matches expected LFSR state.

    For Annex I.1 (seed 0x5D = 1011101):
    First 7 scrambled bits = [0,1,1,0,1,1,0]
    LFSR state after self-sync = 0b0110110 = 0x36
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    with open(os.path.join(VECTORS, 'annex_i1_data_scrambled.json')) as f:
        scrambled = json.load(f)['data']

    # Expected state: first 7 bits shifted into register
    # state = (state << 1) | bit for each of first 7 bits
    expected_state = 0
    for i in range(7):
        expected_state = ((expected_state << 1) | scrambled[i]) & 0x7F
    assert expected_state == 0x36, f"Expected 0x36, computed 0x{expected_state:02X}"

    output_bits, seed_det, seed_valid = await feed_and_collect(dut, scrambled)

    assert seed_valid, "seed_valid never asserted"
    assert seed_det == expected_state, \
        f"seed_detected = 0x{seed_det:02X}, expected 0x{expected_state:02X}"
    dut._log.info(f"Seed detected: 0x{seed_det:02X} (correct, matches self-sync state)")


@cocotb.test()
async def test_different_seed(dut):
    """Different scrambler seed: verify descrambler self-syncs correctly.

    Generate a synthetic scrambled vector with seed 0x7F (all ones) and
    verify the descrambler recovers the original data.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    seed = 0x7F  # 1111111
    # Data: 16 SERVICE zeros + 80 pseudo-random data bits + 6 tail zeros
    data_bits = [0] * 16 + [(i * 7 + 3) % 2 for i in range(80)] + [0] * 6
    scrambled = scramble_with_seed(data_bits, seed)

    output_bits, seed_det, seed_valid = await feed_and_collect(dut, scrambled)

    assert len(output_bits) == len(data_bits), \
        f"Expected {len(data_bits)} outputs, got {len(output_bits)}"

    # SERVICE field should be all zeros
    service = output_bits[:16]
    assert all(b == 0 for b in service), f"SERVICE not zero: {service}"

    # Data should match original
    errors = 0
    for i in range(len(data_bits)):
        if output_bits[i] != data_bits[i]:
            errors += 1

    assert errors == 0, f"Seed 0x7F: {errors}/{len(data_bits)} errors"

    # Verify seed detection
    expected_state = 0
    for i in range(7):
        expected_state = ((expected_state << 1) | scrambled[i]) & 0x7F

    assert seed_valid, "seed_valid not asserted"
    assert seed_det == expected_state, \
        f"seed_detected 0x{seed_det:02X} != expected 0x{expected_state:02X}"
    dut._log.info(f"Seed 0x7F test passed, detected state 0x{seed_det:02X}")


@cocotb.test()
async def test_back_to_back_frames(dut):
    """Back-to-back frames: frame_start resets state correctly.

    Process two frames with different seeds back-to-back.
    Both should descramble correctly.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Frame 1: seed 0x5D (Annex I.1)
    seed1 = 0x5D
    data1 = [0] * 16 + [1, 0, 1, 1, 0, 0, 1, 0] * 5 + [0] * 6  # 62 bits
    scrambled1 = scramble_with_seed(data1, seed1)

    # Frame 2: seed 0x2A
    seed2 = 0x2A
    data2 = [0] * 16 + [0, 1, 0, 1, 1, 1, 0, 1] * 5 + [0] * 6  # 62 bits
    scrambled2 = scramble_with_seed(data2, seed2)

    # Process frame 1
    out1, _, _ = await feed_and_collect(dut, scrambled1, use_frame_start=True)
    assert len(out1) == len(data1), f"Frame 1: got {len(out1)} bits, expected {len(data1)}"

    errors1 = sum(1 for i in range(len(data1)) if out1[i] != data1[i])
    assert errors1 == 0, f"Frame 1: {errors1} errors"

    # Process frame 2 (frame_start should reset state)
    out2, _, _ = await feed_and_collect(dut, scrambled2, use_frame_start=True)
    assert len(out2) == len(data2), f"Frame 2: got {len(out2)} bits, expected {len(data2)}"

    errors2 = sum(1 for i in range(len(data2)) if out2[i] != data2[i])
    assert errors2 == 0, f"Frame 2: {errors2} errors"

    dut._log.info("Back-to-back frames: both descrambled correctly")


@cocotb.test()
async def test_all_zeros_input(dut):
    """Edge case: all-zeros input (as if seed produced all-zero feedback).

    With input = [0, 0, 0, 0, 0, 0, 0, ...]:
    - Self-sync loads state = 0000000
    - From bit 7 onward: feedback = 0^0 = 0, output = 0^0 = 0
    - The LFSR stays at 0 (degenerate case)
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    bits_in = [0] * 50
    output_bits, seed_det, seed_valid = await feed_and_collect(dut, bits_in)

    assert len(output_bits) == 50, f"Expected 50, got {len(output_bits)}"
    assert all(b == 0 for b in output_bits), "All-zero input should give all-zero output"
    assert seed_valid
    assert seed_det == 0, f"All-zero seed should be 0, got 0x{seed_det:02X}"
    dut._log.info("All-zeros edge case passed (degenerate LFSR state)")


@cocotb.test()
async def test_valid_gating(dut):
    """Valid gating: gaps in valid_in don't corrupt state.

    Feed bits with gaps (valid_in deasserted between bits).
    Result should be identical to continuous feeding.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    seed = 0x3C
    data_bits = [0] * 16 + [1, 0, 1, 1, 0, 1, 0, 0] * 4 + [0] * 6  # 54 bits
    scrambled = scramble_with_seed(data_bits, seed)

    # Feed with gaps (2 idle cycles between each bit)
    dut.frame_start.value = 1
    await RisingEdge(dut.clk)
    dut.frame_start.value = 0
    await RisingEdge(dut.clk)

    output_bits = []
    for bit in scrambled:
        dut.valid_in.value = 1
        dut.bit_in.value = bit
        await RisingEdge(dut.clk)

        if int(dut.valid_out.value) == 1:
            output_bits.append(int(dut.bit_out.value))

        # Gap: 2 idle cycles
        dut.valid_in.value = 0
        for _ in range(2):
            await RisingEdge(dut.clk)
            if int(dut.valid_out.value) == 1:
                output_bits.append(int(dut.bit_out.value))

    # Collect stragglers
    dut.valid_in.value = 0
    for _ in range(10):
        await RisingEdge(dut.clk)
        if int(dut.valid_out.value) == 1:
            output_bits.append(int(dut.bit_out.value))

    assert len(output_bits) == len(data_bits), \
        f"Expected {len(data_bits)}, got {len(output_bits)}"

    errors = sum(1 for i in range(len(data_bits)) if output_bits[i] != data_bits[i])
    assert errors == 0, f"Valid gating: {errors} errors"
    dut._log.info("Valid gating test passed (gaps don't corrupt state)")
