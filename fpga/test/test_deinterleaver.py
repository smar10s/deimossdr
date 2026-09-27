"""
Test deinterleaver -- 802.11a bit deinterleaver.

Permutes soft bits from interleaved order back to coded order using
pre-computed ROM lookup tables. Undoes the two-stage IEEE 802.11
interleave permutation so the Viterbi decoder receives coded bits in
the correct sequence.

Architecture: Capture-then-emit (mirrors demapper pattern).
  Phase 1 (CAPTURE): Accepts N_CBPS soft bits at 1/clock into RAM.
  Phase 2 (EMIT): Reads out in permuted order at 1/clock.

Rate modes (determines N_CBPS):
  0 = BPSK:   N_CBPS=48,  N_BPSC=1
  1 = QPSK:   N_CBPS=96,  N_BPSC=2
  2 = 16-QAM: N_CBPS=192, N_BPSC=4
  3 = 64-QAM: N_CBPS=288, N_BPSC=6

Tests:
  1. BPSK: golden vector (SIGNAL field) — deinterleaved matches encoded
  2. 16-QAM: golden vector (DATA symbol 1) — deinterleaved matches encoded
  3. 64-QAM: round-trip with known permutation
  4. Back-to-back symbols without reset
  5. Timing: N_CBPS bits in → N_CBPS bits out
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer
import json
import os

VECTORS = os.path.join(os.path.dirname(__file__), '..', '..', 'extern', 'lib80211', 'vectors')

# Rate modes
BPSK = 0
QPSK = 1
QAM16 = 2
QAM64 = 3

# N_CBPS per rate
NCBPS = {BPSK: 48, QPSK: 96, QAM16: 192, QAM64: 288}

# N_BPSC per rate (LLRs per subcarrier / wide-word lanes)
NBPSC = {BPSK: 1, QPSK: 2, QAM16: 4, QAM64: 6}

# Pre-computed deinterleave permutation tables (from lib80211 interleaver.c)
# Semantics: out[k] = in[perm[k]]
PERM_48 = [
    0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45,
    1, 4, 7, 10, 13, 16, 19, 22, 25, 28, 31, 34, 37, 40, 43, 46,
    2, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 38, 41, 44, 47,
]

PERM_96 = [
    0, 6, 12, 18, 24, 30, 36, 42, 48, 54, 60, 66, 72, 78, 84, 90,
    1, 7, 13, 19, 25, 31, 37, 43, 49, 55, 61, 67, 73, 79, 85, 91,
    2, 8, 14, 20, 26, 32, 38, 44, 50, 56, 62, 68, 74, 80, 86, 92,
    3, 9, 15, 21, 27, 33, 39, 45, 51, 57, 63, 69, 75, 81, 87, 93,
    4, 10, 16, 22, 28, 34, 40, 46, 52, 58, 64, 70, 76, 82, 88, 94,
    5, 11, 17, 23, 29, 35, 41, 47, 53, 59, 65, 71, 77, 83, 89, 95,
]

PERM_192 = [
    0, 13, 24, 37, 48, 61, 72, 85, 96, 109, 120, 133, 144, 157, 168, 181,
    1, 12, 25, 36, 49, 60, 73, 84, 97, 108, 121, 132, 145, 156, 169, 180,
    2, 15, 26, 39, 50, 63, 74, 87, 98, 111, 122, 135, 146, 159, 170, 183,
    3, 14, 27, 38, 51, 62, 75, 86, 99, 110, 123, 134, 147, 158, 171, 182,
    4, 17, 28, 41, 52, 65, 76, 89, 100, 113, 124, 137, 148, 161, 172, 185,
    5, 16, 29, 40, 53, 64, 77, 88, 101, 112, 125, 136, 149, 160, 173, 184,
    6, 19, 30, 43, 54, 67, 78, 91, 102, 115, 126, 139, 150, 163, 174, 187,
    7, 18, 31, 42, 55, 66, 79, 90, 103, 114, 127, 138, 151, 162, 175, 186,
    8, 21, 32, 45, 56, 69, 80, 93, 104, 117, 128, 141, 152, 165, 176, 189,
    9, 20, 33, 44, 57, 68, 81, 92, 105, 116, 129, 140, 153, 164, 177, 188,
    10, 23, 34, 47, 58, 71, 82, 95, 106, 119, 130, 143, 154, 167, 178, 191,
    11, 22, 35, 46, 59, 70, 83, 94, 107, 118, 131, 142, 155, 166, 179, 190,
]

PERM_288 = [
    0, 20, 37, 54, 74, 91, 108, 128, 145, 162, 182, 199, 216, 236, 253, 270,
    1, 18, 38, 55, 72, 92, 109, 126, 146, 163, 180, 200, 217, 234, 254, 271,
    2, 19, 36, 56, 73, 90, 110, 127, 144, 164, 181, 198, 218, 235, 252, 272,
    3, 23, 40, 57, 77, 94, 111, 131, 148, 165, 185, 202, 219, 239, 256, 273,
    4, 21, 41, 58, 75, 95, 112, 129, 149, 166, 183, 203, 220, 237, 257, 274,
    5, 22, 39, 59, 76, 93, 113, 130, 147, 167, 184, 201, 221, 238, 255, 275,
    6, 26, 43, 60, 80, 97, 114, 134, 151, 168, 188, 205, 222, 242, 259, 276,
    7, 24, 44, 61, 78, 98, 115, 132, 152, 169, 186, 206, 223, 240, 260, 277,
    8, 25, 42, 62, 79, 96, 116, 133, 150, 170, 187, 204, 224, 241, 258, 278,
    9, 29, 46, 63, 83, 100, 117, 137, 154, 171, 191, 208, 225, 245, 262, 279,
    10, 27, 47, 64, 81, 101, 118, 135, 155, 172, 189, 209, 226, 243, 263, 280,
    11, 28, 45, 65, 82, 99, 119, 136, 153, 173, 190, 207, 227, 244, 261, 281,
    12, 32, 49, 66, 86, 103, 120, 140, 157, 174, 194, 211, 228, 248, 265, 282,
    13, 30, 50, 67, 84, 104, 121, 138, 158, 175, 192, 212, 229, 246, 266, 283,
    14, 31, 48, 68, 85, 102, 122, 139, 156, 176, 193, 210, 230, 247, 264, 284,
    15, 35, 52, 69, 89, 106, 123, 143, 160, 177, 197, 214, 231, 251, 268, 285,
    16, 33, 53, 70, 87, 107, 124, 141, 161, 178, 195, 215, 232, 249, 269, 286,
    17, 34, 51, 71, 88, 105, 125, 142, 159, 179, 196, 213, 233, 250, 267, 287,
]

PERMS = {BPSK: PERM_48, QPSK: PERM_96, QAM16: PERM_192, QAM64: PERM_288}


def to_s8(v):
    """Interpret 8-bit value as signed."""
    v = int(v) & 0xFF
    return v - 0x100 if v >= 0x80 else v


def s8_to_bits(v):
    """Convert signed int to 8-bit unsigned representation."""
    if v < 0:
        v = v + 0x100
    return v & 0xFF


def hard_decision(soft):
    """Convert signed soft bit to hard decision (1 if positive, 0 if negative/zero)."""
    return 1 if soft > 0 else 0


def deinterleave_reference(soft_in, rate_mode):
    """Reference deinterleave: out[k] = in[perm[k]]."""
    perm = PERMS[rate_mode]
    return [soft_in[perm[k]] for k in range(len(perm))]


async def reset_dut(dut):
    """Reset the DUT and initialize inputs."""
    dut.rst_n.value = 0
    dut.valid_in.value = 0
    for j in range(6):
        getattr(dut, f"soft_in{j}").value = 0
    dut.rate_mode.value = 0
    dut.stall_in.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def feed_and_collect(dut, soft_in_list, rate_mode):
    """Feed 48 wide subcarrier words and collect deinterleaved outputs.

    soft_in_list is in arrival (interleaved) order, N_CBPS entries. Subcarrier
    s is fed as lanes 0..n_bpsc-1 = soft_in_list[s*n_bpsc : (s+1)*n_bpsc].
    All rates emit 2 bits/clk (one pair per valid_out pulse). Returns a flat
    list of signed 8-bit values in deinterleaved order.
    """
    n_cbps = NCBPS[rate_mode]
    nb = NBPSC[rate_mode]
    assert len(soft_in_list) == n_cbps, \
        f"Expected {n_cbps} input soft bits, got {len(soft_in_list)}"

    dut.rate_mode.value = rate_mode

    soft_out = []

    for s in range(48):
        dut.valid_in.value = 1
        for j in range(6):
            lane = soft_in_list[s * nb + j] if j < nb else 0
            getattr(dut, f"soft_in{j}").value = s8_to_bits(lane)
        await RisingEdge(dut.clk)

    dut.valid_in.value = 0
    for j in range(6):
        getattr(dut, f"soft_in{j}").value = 0

    # Collect outputs (capture-then-emit; pairs per valid pulse)
    for _ in range(n_cbps + 50):
        await RisingEdge(dut.clk)
        if int(dut.valid_out.value) == 1:
            soft_out.append(to_s8(int(dut.soft_out0.value)))
            soft_out.append(to_s8(int(dut.soft_out1.value)))

    return soft_out


@cocotb.test()
async def test_bpsk_signal_golden(dut):
    """BPSK: Annex I.1 SIGNAL field deinterleave matches golden encoded bits.

    Input:  annex_i1_signal_interleaved.json (48 bits as soft LLRs)
    Expected: annex_i1_signal_encoded.json (48 bits)
    Verify: hard decisions on output match encoded bits exactly.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    with open(os.path.join(VECTORS, 'annex_i1_signal_interleaved.json')) as f:
        sig_intl = json.load(f)
    with open(os.path.join(VECTORS, 'annex_i1_signal_encoded.json')) as f:
        sig_enc = json.load(f)

    interleaved_bits = sig_intl['bits']
    expected_encoded = sig_enc['bits']

    # Convert hard bits to soft LLRs: bit=1 → +100, bit=0 → -100
    soft_in = [100 if b == 1 else -100 for b in interleaved_bits]

    soft_out = await feed_and_collect(dut, soft_in, BPSK)

    assert len(soft_out) == 48, f"Expected 48 output soft bits, got {len(soft_out)}"

    # Hard decisions must match encoded bits exactly
    errors = 0
    for i in range(48):
        hd = hard_decision(soft_out[i])
        if hd != expected_encoded[i]:
            dut._log.warning(
                f"Bit {i}: soft_out={soft_out[i]}, hard={hd}, "
                f"expected={expected_encoded[i]}")
            errors += 1

    assert errors == 0, f"BPSK SIGNAL deinterleave: {errors}/48 bit errors"
    dut._log.info("BPSK SIGNAL golden vector test passed (48/48 bits correct)")


@cocotb.test()
async def test_16qam_data_golden(dut):
    """16-QAM: Annex I.1 DATA first symbol deinterleave matches golden encoded bits.

    Input:  annex_i1_data_interleaved.json first 192 bits (as soft LLRs)
    Expected: annex_i1_data_encoded.json first 192 bits
    Verify: hard decisions on output match encoded bits exactly.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    with open(os.path.join(VECTORS, 'annex_i1_data_interleaved.json')) as f:
        data_intl = json.load(f)
    with open(os.path.join(VECTORS, 'annex_i1_data_encoded.json')) as f:
        data_enc = json.load(f)

    # First DATA symbol = first 192 interleaved bits (N_CBPS=192 for 16-QAM)
    interleaved_bits = data_intl['data'][:192]
    expected_encoded = data_enc['data'][:192]

    # Convert to soft LLRs
    soft_in = [100 if b == 1 else -100 for b in interleaved_bits]

    soft_out = await feed_and_collect(dut, soft_in, QAM16)

    assert len(soft_out) == 192, f"Expected 192 output soft bits, got {len(soft_out)}"

    errors = 0
    for i in range(192):
        hd = hard_decision(soft_out[i])
        if hd != expected_encoded[i]:
            dut._log.warning(
                f"Bit {i}: soft_out={soft_out[i]}, hard={hd}, "
                f"expected={expected_encoded[i]}")
            errors += 1

    assert errors == 0, f"16-QAM DATA deinterleave: {errors}/192 bit errors"
    dut._log.info("16-QAM DATA golden vector test passed (192/192 bits correct)")


@cocotb.test()
async def test_64qam_permutation(dut):
    """64-QAM: Verify correct permutation with known soft values.

    Feed 288 soft values (indices as values so we can verify permutation).
    Expected output: out[k] = in[perm_288[k]] for all k.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Use unique values: input[i] = (i * 7 + 3) mod 251 - 125
    # This gives unique signed 8-bit values we can track through the permutation
    soft_in = [((i * 7 + 3) % 251) - 125 for i in range(288)]

    # Expected output via reference deinterleave
    expected = deinterleave_reference(soft_in, QAM64)

    soft_out = await feed_and_collect(dut, soft_in, QAM64)

    assert len(soft_out) == 288, f"Expected 288 output soft bits, got {len(soft_out)}"

    errors = 0
    for i in range(288):
        if soft_out[i] != expected[i]:
            dut._log.warning(
                f"Position {i}: got {soft_out[i]}, expected {expected[i]} "
                f"(perm[{i}]={PERM_288[i]}, in[{PERM_288[i]}]={soft_in[PERM_288[i]]})")
            errors += 1
            if errors > 10:
                dut._log.warning("... (truncating)")
                break

    assert errors == 0, f"64-QAM permutation: {errors}/288 errors"
    dut._log.info("64-QAM permutation test passed (288/288 correct)")


@cocotb.test()
async def test_qpsk_permutation(dut):
    """QPSK: Verify correct permutation with known soft values.

    Feed 96 unique soft values. Verify output order matches PERM_96 table.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Unique values per position
    soft_in = [((i * 11 + 5) % 251) - 125 for i in range(96)]
    expected = deinterleave_reference(soft_in, QPSK)

    soft_out = await feed_and_collect(dut, soft_in, QPSK)

    assert len(soft_out) == 96, f"Expected 96 output soft bits, got {len(soft_out)}"

    errors = 0
    for i in range(96):
        if soft_out[i] != expected[i]:
            dut._log.warning(f"Position {i}: got {soft_out[i]}, expected {expected[i]}")
            errors += 1

    assert errors == 0, f"QPSK permutation: {errors}/96 errors"
    dut._log.info("QPSK permutation test passed (96/96 correct)")


@cocotb.test()
async def test_back_to_back_symbols(dut):
    """Back-to-back: feed two consecutive BPSK symbols without reset.

    Verify both are deinterleaved correctly and independently.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # First symbol: all positive
    soft_in_1 = [((i * 3 + 1) % 127) + 1 for i in range(48)]  # all positive
    expected_1 = deinterleave_reference(soft_in_1, BPSK)

    # Second symbol: alternating signs
    soft_in_2 = [50 if i % 2 == 0 else -50 for i in range(48)]
    expected_2 = deinterleave_reference(soft_in_2, BPSK)

    dut.rate_mode.value = BPSK

    # Feed first symbol
    for i in range(48):
        dut.valid_in.value = 1
        dut.soft_in0.value = s8_to_bits(soft_in_1[i])
        await RisingEdge(dut.clk)

    dut.valid_in.value = 0

    # Collect first symbol output
    out_1 = []
    timeout = 0
    while len(out_1) < 48 and timeout < 120:
        await RisingEdge(dut.clk)
        if int(dut.valid_out.value) == 1:
            out_1.append(to_s8(int(dut.soft_out0.value)))
            out_1.append(to_s8(int(dut.soft_out1.value)))
        timeout += 1

    assert len(out_1) == 48, f"First symbol: expected 48, got {len(out_1)}"

    # Wait a couple clocks for FSM to return to idle
    await ClockCycles(dut.clk, 2)

    # Feed second symbol
    for i in range(48):
        dut.valid_in.value = 1
        dut.soft_in0.value = s8_to_bits(soft_in_2[i])
        await RisingEdge(dut.clk)

    dut.valid_in.value = 0

    # Collect second symbol output
    out_2 = []
    timeout = 0
    while len(out_2) < 48 and timeout < 120:
        await RisingEdge(dut.clk)
        if int(dut.valid_out.value) == 1:
            out_2.append(to_s8(int(dut.soft_out0.value)))
            out_2.append(to_s8(int(dut.soft_out1.value)))
        timeout += 1

    assert len(out_2) == 48, f"Second symbol: expected 48, got {len(out_2)}"

    # Verify both symbols
    errors = 0
    for i in range(48):
        if out_1[i] != expected_1[i]:
            errors += 1
        if out_2[i] != expected_2[i]:
            errors += 1

    assert errors == 0, f"Back-to-back: {errors} errors across 96 bits"
    dut._log.info("Back-to-back symbols test passed")


@cocotb.test()
async def test_timing_output_count(dut):
    """Timing: N_CBPS soft bits in → exactly N_CBPS soft bits out for each rate.

    Also verifies no extra or missing outputs.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    for rate, n_cbps in [(BPSK, 48), (QPSK, 96), (QAM16, 192), (QAM64, 288)]:
        await reset_dut(dut)
        soft_in = [((i * 13) % 251) - 125 for i in range(n_cbps)]
        soft_out = await feed_and_collect(dut, soft_in, rate)

        assert len(soft_out) == n_cbps, \
            f"Rate {rate} (N_CBPS={n_cbps}): expected {n_cbps} outputs, got {len(soft_out)}"

        dut._log.info(f"Rate {rate} (N_CBPS={n_cbps}): {len(soft_out)} outputs OK")

    dut._log.info("Timing/output count test passed for all 4 rates")


@cocotb.test()
async def test_stall_mid_emit_holds_output(dut):
    """stall_in during emit holds valid_out high and preserves bit order."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    dut.stall_in.value = 0

    # QPSK symbol: 96 bits, captured at 2/clk
    soft_in = [((i * 7) % 200) - 100 for i in range(96)]
    expected = deinterleave_reference(soft_in, QPSK)

    dut.rate_mode.value = QPSK
    for i in range(0, 96, 2):
        dut.valid_in.value = 1
        dut.soft_in0.value = s8_to_bits(soft_in[i])
        dut.soft_in1.value = s8_to_bits(soft_in[i + 1])
        await RisingEdge(dut.clk)
    dut.valid_in.value = 0
    dut.soft_in0.value = 0
    dut.soft_in1.value = 0

    # Emit: collect pairs, stalling once mid-emit.
    # This harness reads PRE-edge values after RisingEdge; Timer(1) gives
    # post-edge samples (repo convention, cf. test_soft_pairer). With the
    # emit pipeline fully frozen, the pair captured at the trigger IS the
    # pair held through the stall and every released pair is a genuine
    # uncounted pair, so no skip is needed — the sequence gate below
    # (exact match, no loss/dup/reorder) is the brief's.
    out = []
    stall_todo = True
    for _ in range(300):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        if int(dut.valid_out.value) == 1:
            out.append(to_s8(int(dut.soft_out0.value)))
            out.append(to_s8(int(dut.soft_out1.value)))
            if stall_todo and len(out) >= 20:
                stall_todo = False
                dut.stall_in.value = 1
                held0 = int(dut.soft_out0.value)
                held1 = int(dut.soft_out1.value)
                for _ in range(8):
                    await RisingEdge(dut.clk)
                    await Timer(1, unit="ns")
                    assert int(dut.valid_out.value) == 1, \
                        "valid_out deasserted during stall"
                    assert int(dut.soft_out0.value) == held0, \
                        "soft_out0 changed during stall"
                    assert int(dut.soft_out1.value) == held1, \
                        "soft_out1 changed during stall"
                dut.stall_in.value = 0
        if len(out) >= 96:
            break

    assert out == expected, f"sequence mismatch after stall: {out} != {expected}"


@cocotb.test()
async def test_stall_during_last_pair_holds_output(dut):
    """stall_in during the LAST pair holds valid_out high until release.

    Regression for the lever 2a-prime layer-6 corruption: the old FSM
    transitioned to S_IDLE in the same edge the last pair landed on the
    output registers, so valid_out dropped one cycle later regardless of
    stall_in. If the downstream elastic FIFO was full during that single
    presentation cycle, the pair was skipped and never retried (lost
    pair -> stream desync -> FCS fail). The S_TAIL state must hold the
    last pair while stalled and present it for exactly one non-stall
    cycle on release.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    dut.stall_in.value = 0

    # QPSK symbol: 96 bits -> 48 pairs
    soft_in = [((i * 7) % 200) - 100 for i in range(96)]
    expected = deinterleave_reference(soft_in, QPSK)

    dut.rate_mode.value = QPSK
    for i in range(0, 96, 2):
        dut.valid_in.value = 1
        dut.soft_in0.value = s8_to_bits(soft_in[i])
        dut.soft_in1.value = s8_to_bits(soft_in[i + 1])
        await RisingEdge(dut.clk)
    dut.valid_in.value = 0
    dut.soft_in0.value = 0
    dut.soft_in1.value = 0

    out = []
    stall_at_next_valid = False
    last_pair_stall_done = False
    for _ in range(400):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        if int(dut.valid_out.value) == 1:
            if stall_at_next_valid and not last_pair_stall_done:
                # This is the last pair on the wire. Engage the stall and
                # verify it is HELD (valid_out stays high, values frozen).
                last_pair_stall_done = True
                held0 = int(dut.soft_out0.value)
                held1 = int(dut.soft_out1.value)
                dut.stall_in.value = 1
                for _ in range(8):
                    await RisingEdge(dut.clk)
                    await Timer(1, unit="ns")
                    assert int(dut.valid_out.value) == 1, \
                        "valid_out dropped while stalled during last pair"
                    assert int(dut.soft_out0.value) == held0, \
                        "soft_out0 changed during last-pair stall"
                    assert int(dut.soft_out1.value) == held1, \
                        "soft_out1 changed during last-pair stall"
                dut.stall_in.value = 0
                out.append(to_s8(int(dut.soft_out0.value)))
                out.append(to_s8(int(dut.soft_out1.value)))
            else:
                out.append(to_s8(int(dut.soft_out0.value)))
                out.append(to_s8(int(dut.soft_out1.value)))
                if len(out) >= 94:
                    # 47 pairs collected — the next valid cycle presents
                    # the LAST pair
                    stall_at_next_valid = True
        if last_pair_stall_done and int(dut.valid_out.value) == 0:
            break

    assert len(out) == 96, f"expected 96 bits, got {len(out)}"
    assert out == expected, \
        f"sequence mismatch after last-pair stall: {out} != {expected}"
