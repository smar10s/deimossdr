"""
Test psdu_packer -- serial bit-to-byte packer for PSDU recovery.

Receives descrambled bits serially (one per clock when valid_in), skips 16
SERVICE bits, packs remaining bits into bytes LSB-first, emits byte_valid +
byte_out for each completed byte, and stops after payload_len bytes
(psdu_len - 4, excluding FCS).

Tests:
  1. test_basic_packing — Known bit pattern, verify byte values and count
  2. test_frame_reset — Mid-frame reset via frame_start, verify clean restart
  3. test_various_lengths — Multiple payload sizes (1, 20, 96 bytes)
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles


def bytes_to_bits(data):
    """Convert bytes to bit list (LSB first per byte, per 802.11)."""
    bits = []
    for byte in data:
        for bit in range(8):
            bits.append((byte >> bit) & 1)
    return bits


async def reset_dut(dut):
    """Reset the DUT."""
    dut.rst_n.value = 0
    dut.valid_in.value = 0
    dut.bit_in.value = 0
    dut.frame_start.value = 0
    dut.psdu_len.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def feed_frame(dut, payload_bytes, psdu_len, continuous=True):
    """Feed SERVICE + payload + FCS bits to the packer.

    Args:
        dut: DUT handle
        payload_bytes: payload data (what we expect to be emitted)
        psdu_len: total PSDU length including 4-byte FCS
        continuous: if False, insert gaps between bytes

    Returns:
        list of emitted bytes, final byte_count, done_seen, active went low
    """
    # Build the full bit stream: 16 SERVICE + payload + 32 FCS dummy bits
    service_bits = [0] * 16
    payload_bits = bytes_to_bits(payload_bytes)
    fcs_bits = [0] * 32  # FCS content doesn't matter (packer ignores it)
    all_bits = service_bits + payload_bits + fcs_bits

    # Start frame
    dut.psdu_len.value = psdu_len
    dut.frame_start.value = 1
    await RisingEdge(dut.clk)
    dut.frame_start.value = 0
    await RisingEdge(dut.clk)

    emitted_bytes = []
    done_seen = False

    for i, bit in enumerate(all_bits):
        dut.valid_in.value = 1
        dut.bit_in.value = bit
        await RisingEdge(dut.clk)

        if int(dut.byte_valid.value) == 1:
            emitted_bytes.append(int(dut.byte_out.value))
        if int(dut.done.value) == 1:
            done_seen = True

        if not continuous and (i + 1) % 8 == 0:
            dut.valid_in.value = 0
            await RisingEdge(dut.clk)
            if int(dut.byte_valid.value) == 1:
                emitted_bytes.append(int(dut.byte_out.value))
            if int(dut.done.value) == 1:
                done_seen = True

    # Drain
    dut.valid_in.value = 0
    for _ in range(10):
        await RisingEdge(dut.clk)
        if int(dut.byte_valid.value) == 1:
            emitted_bytes.append(int(dut.byte_out.value))
        if int(dut.done.value) == 1:
            done_seen = True

    byte_count = int(dut.byte_count.value)
    active_low = int(dut.active.value) == 0

    return emitted_bytes, byte_count, done_seen, active_low


@cocotb.test()
async def test_basic_packing(dut):
    """Feed known bit pattern (16 SERVICE + N*8 data + 32 FCS bits).

    Verify:
      - Exactly psdu_len - 4 bytes emitted
      - Each byte matches expected value (bits packed LSB-first)
      - No bytes emitted during SERVICE or FCS
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # 10 payload bytes with known pattern
    payload = bytes([0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB])
    psdu_len = len(payload) + 4  # 14

    emitted, count, done_seen, active_low = await feed_frame(
        dut, payload, psdu_len)

    assert len(emitted) == len(payload), \
        f"Expected {len(payload)} bytes, got {len(emitted)}"
    assert count == len(payload), \
        f"byte_count {count} != expected {len(payload)}"
    assert done_seen, "done not asserted"
    assert active_low, "active should be low after completion"

    for i, (got, exp) in enumerate(zip(emitted, payload)):
        assert got == exp, \
            f"Byte {i}: got 0x{got:02X}, expected 0x{exp:02X}"

    dut._log.info(f"Basic packing: {len(payload)} bytes correct")


@cocotb.test()
async def test_frame_reset(dut):
    """Start a frame, feed partial data, pulse frame_start again.

    Verify counters reset and new frame starts clean.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Start first frame (psdu_len=24 → 20 payload bytes)
    psdu_len_1 = 24
    dut.psdu_len.value = psdu_len_1
    dut.frame_start.value = 1
    await RisingEdge(dut.clk)
    dut.frame_start.value = 0
    await RisingEdge(dut.clk)

    # Feed 16 SERVICE + 32 data bits (4 bytes worth) — partial frame
    for _ in range(16 + 32):
        dut.valid_in.value = 1
        dut.bit_in.value = 1
        await RisingEdge(dut.clk)
    dut.valid_in.value = 0
    await ClockCycles(dut.clk, 5)

    # Verify some bytes were emitted
    partial_count = int(dut.byte_count.value)
    assert partial_count == 4, f"Expected 4 partial bytes, got {partial_count}"
    assert int(dut.active.value) == 1, "Should still be active"

    # Now reset with a new frame
    payload_2 = bytes([0x55, 0xAA, 0x55, 0xAA, 0x55])
    psdu_len_2 = len(payload_2) + 4  # 9

    emitted, count, done_seen, active_low = await feed_frame(
        dut, payload_2, psdu_len_2)

    assert len(emitted) == len(payload_2), \
        f"After reset: expected {len(payload_2)} bytes, got {len(emitted)}"
    assert count == len(payload_2), \
        f"byte_count {count} != {len(payload_2)}"
    assert done_seen, "done not asserted after second frame"

    for i, (got, exp) in enumerate(zip(emitted, payload_2)):
        assert got == exp, \
            f"Byte {i} after reset: got 0x{got:02X}, expected 0x{exp:02X}"

    dut._log.info("Frame reset: counters properly reset, second frame correct")


@cocotb.test()
async def test_various_lengths(dut):
    """Test with various psdu_len values.

    psdu_len=5  → 1 payload byte
    psdu_len=24 → 20 payload bytes
    psdu_len=100 → 96 payload bytes

    Verify byte_count matches expected for each.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    test_cases = [
        (5, 1),    # 1 payload byte
        (24, 20),  # 20 payload bytes
        (100, 96), # 96 payload bytes
    ]

    for psdu_len, expected_payload in test_cases:
        # Generate payload with incrementing pattern
        payload = bytes([i & 0xFF for i in range(expected_payload)])

        emitted, count, done_seen, active_low = await feed_frame(
            dut, payload, psdu_len)

        assert len(emitted) == expected_payload, \
            f"psdu_len={psdu_len}: expected {expected_payload} bytes, got {len(emitted)}"
        assert count == expected_payload, \
            f"psdu_len={psdu_len}: byte_count={count} != {expected_payload}"
        assert done_seen, \
            f"psdu_len={psdu_len}: done not asserted"
        assert active_low, \
            f"psdu_len={psdu_len}: active should be low after completion"

        for i, (got, exp) in enumerate(zip(emitted, payload)):
            assert got == exp, \
                f"psdu_len={psdu_len}, byte {i}: got 0x{got:02X}, expected 0x{exp:02X}"

        dut._log.info(f"psdu_len={psdu_len}: {expected_payload} bytes correct")

    dut._log.info("All length variants passed")
