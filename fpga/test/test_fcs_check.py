"""
Test fcs_check -- 802.11 CRC-32 FCS checker.

Receives serial bits (LSB first per byte), computes running CRC-32,
and checks the residual after all PSDU bytes (including FCS) are consumed.

CRC-32: polynomial 0xEDB88320 (reflected), init 0xFFFFFFFF.
Magic residual after valid frame: 0xDEBB20E3.

Interface:
  frame_start: pulse to reset
  psdu_len[11:0]: total PSDU bytes including 4-byte FCS
  valid_in + bit_in: serial PSDU bits (after SERVICE, LSB first per byte)
  fcs_valid: pulse when CRC residual matches (frame good)
  fcs_fail: pulse when CRC residual doesn't match (frame bad)
  frame_done: pulse when all bytes processed

Tests:
  1. Annex I.1 golden vector: 100-byte PSDU → fcs_valid
  2. Corrupted byte: flip one bit → fcs_fail
  3. Short frame (14 bytes, minimal ACK-sized)
  4. Different PSDU content with valid CRC
  5. Valid gating (gaps in input)
  6. Back-to-back frames
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import json
import os
import struct

VECTORS = os.path.join(os.path.dirname(__file__), '..', '..', 'extern', 'lib80211', 'vectors')

CRC_RESIDUAL = 0xDEBB20E3


def crc32_802(data):
    """CRC-32 as used in 802.11 FCS (same as Ethernet).

    Reflected polynomial 0xEDB88320, init 0xFFFFFFFF, no final XOR
    (caller must XOR with 0xFFFFFFFF for the transmitted FCS value).
    """
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    return crc


def make_frame_with_fcs(payload_bytes):
    """Create a frame with valid FCS appended."""
    crc = crc32_802(payload_bytes)
    fcs = struct.pack('<I', crc ^ 0xFFFFFFFF)
    return payload_bytes + fcs


def bytes_to_bits(data):
    """Convert bytes to bit array (LSB first per byte, per 802.11)."""
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


async def feed_frame(dut, psdu_bytes, continuous=True):
    """Feed a frame (bytes including FCS) to the DUT.

    Returns (fcs_valid_seen, fcs_fail_seen, frame_done_seen).
    """
    psdu_len = len(psdu_bytes)
    bits = bytes_to_bits(psdu_bytes)

    # Start frame
    dut.psdu_len.value = psdu_len
    dut.frame_start.value = 1
    await RisingEdge(dut.clk)
    dut.frame_start.value = 0
    await RisingEdge(dut.clk)

    fcs_valid_seen = False
    fcs_fail_seen = False
    frame_done_seen = False

    # Feed 16 SERVICE bits (zeros, will be skipped by fcs_check)
    for _ in range(16):
        dut.valid_in.value = 1
        dut.bit_in.value = 0
        await RisingEdge(dut.clk)

    for i, bit in enumerate(bits):
        dut.valid_in.value = 1
        dut.bit_in.value = bit
        await RisingEdge(dut.clk)

        if int(dut.fcs_valid.value) == 1:
            fcs_valid_seen = True
        if int(dut.fcs_fail.value) == 1:
            fcs_fail_seen = True
        if int(dut.frame_done.value) == 1:
            frame_done_seen = True

        if not continuous and i % 8 == 7:
            # Insert gap between bytes
            dut.valid_in.value = 0
            await RisingEdge(dut.clk)

            if int(dut.fcs_valid.value) == 1:
                fcs_valid_seen = True
            if int(dut.fcs_fail.value) == 1:
                fcs_fail_seen = True
            if int(dut.frame_done.value) == 1:
                frame_done_seen = True

    # Collect remaining outputs
    dut.valid_in.value = 0
    for _ in range(10):
        await RisingEdge(dut.clk)
        if int(dut.fcs_valid.value) == 1:
            fcs_valid_seen = True
        if int(dut.fcs_fail.value) == 1:
            fcs_fail_seen = True
        if int(dut.frame_done.value) == 1:
            frame_done_seen = True

    return fcs_valid_seen, fcs_fail_seen, frame_done_seen


@cocotb.test()
async def test_annex_i1_golden_vector(dut):
    """Annex I.1: 100-byte PSDU with valid FCS → fcs_valid asserted.

    The PSDU includes the 4-byte FCS at the end. CRC-32 over all 100 bytes
    should produce the magic residual 0xDEBB20E3.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Load golden PSDU (already includes FCS)
    with open(os.path.join(VECTORS, 'annex_i1_psdu.json')) as f:
        psdu_vec = json.load(f)
    psdu_bytes = bytes(int(h, 16) for h in psdu_vec['octets_hex'])
    assert len(psdu_bytes) == 100

    # Verify CRC in software first
    residual = crc32_802(psdu_bytes)
    assert residual == CRC_RESIDUAL, \
        f"Software CRC residual 0x{residual:08X} != 0x{CRC_RESIDUAL:08X}"

    fcs_valid, fcs_fail, frame_done = await feed_frame(dut, psdu_bytes)

    assert frame_done, "frame_done not asserted"
    assert fcs_valid, "fcs_valid not asserted for valid frame"
    assert not fcs_fail, "fcs_fail should not assert for valid frame"
    dut._log.info("Annex I.1: 100-byte PSDU, FCS valid (correct)")


@cocotb.test()
async def test_corrupted_byte(dut):
    """Corrupt one byte → fcs_fail asserted.

    Flip bit 0 of byte 50 in the Annex I.1 PSDU. CRC should fail.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    with open(os.path.join(VECTORS, 'annex_i1_psdu.json')) as f:
        psdu_vec = json.load(f)
    psdu_bytes = bytearray(int(h, 16) for h in psdu_vec['octets_hex'])

    # Corrupt byte 50
    psdu_bytes[50] ^= 0x01

    # Verify CRC fails in software
    residual = crc32_802(psdu_bytes)
    assert residual != CRC_RESIDUAL

    fcs_valid, fcs_fail, frame_done = await feed_frame(dut, bytes(psdu_bytes))

    assert frame_done, "frame_done not asserted"
    assert fcs_fail, "fcs_fail not asserted for corrupted frame"
    assert not fcs_valid, "fcs_valid should not assert for corrupted frame"
    dut._log.info("Corrupted frame: fcs_fail asserted (correct)")


@cocotb.test()
async def test_short_frame(dut):
    """Short frame: 14 bytes (10 data + 4 FCS) — minimal ACK-sized.

    Verifies the module works with short payloads.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # 10 bytes of payload
    payload = bytes([0x04, 0x02, 0x00, 0x2E, 0x00, 0x60, 0x08, 0xCD, 0x37, 0xA6])
    frame = make_frame_with_fcs(payload)
    assert len(frame) == 14

    # Verify in software
    assert crc32_802(frame) == CRC_RESIDUAL

    fcs_valid, fcs_fail, frame_done = await feed_frame(dut, frame)

    assert frame_done, "frame_done not asserted"
    assert fcs_valid, "fcs_valid not asserted for valid short frame"
    assert not fcs_fail, "fcs_fail should not assert"
    dut._log.info("Short frame (14 bytes): FCS valid (correct)")


@cocotb.test()
async def test_different_content(dut):
    """Different PSDU content: all-0xFF payload + valid FCS.

    Tests that CRC works for non-golden-vector data.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    payload = bytes([0xFF] * 200)
    frame = make_frame_with_fcs(payload)
    assert len(frame) == 204

    assert crc32_802(frame) == CRC_RESIDUAL

    fcs_valid, fcs_fail, frame_done = await feed_frame(dut, frame)

    assert frame_done, "frame_done not asserted"
    assert fcs_valid, "fcs_valid not asserted"
    assert not fcs_fail, "fcs_fail should not assert"
    dut._log.info("All-0xFF 200-byte frame: FCS valid (correct)")


@cocotb.test()
async def test_valid_gating(dut):
    """Valid gating: gaps between bytes don't corrupt CRC state.

    Feed the Annex I.1 PSDU with 1-cycle gaps between each byte.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    with open(os.path.join(VECTORS, 'annex_i1_psdu.json')) as f:
        psdu_vec = json.load(f)
    psdu_bytes = bytes(int(h, 16) for h in psdu_vec['octets_hex'])

    fcs_valid, fcs_fail, frame_done = await feed_frame(
        dut, psdu_bytes, continuous=False)

    assert frame_done, "frame_done not asserted"
    assert fcs_valid, "fcs_valid not asserted (gated feed)"
    assert not fcs_fail, "fcs_fail should not assert"
    dut._log.info("Valid gating: FCS still passes with gaps between bytes")


@cocotb.test()
async def test_back_to_back_frames(dut):
    """Back-to-back: first frame valid, second frame corrupted.

    Verifies frame_start properly resets CRC state.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Frame 1: valid
    payload1 = bytes([0xAA, 0xBB, 0xCC, 0xDD] * 10)
    frame1 = make_frame_with_fcs(payload1)

    # Frame 2: corrupted (flip last data byte before FCS)
    payload2 = bytes([0x11, 0x22, 0x33, 0x44] * 10)
    frame2 = bytearray(make_frame_with_fcs(payload2))
    frame2[39] ^= 0x80  # corrupt last payload byte

    # Feed frame 1
    v1, f1, d1 = await feed_frame(dut, frame1)
    assert d1, "Frame 1: frame_done not asserted"
    assert v1, "Frame 1: should be valid"
    assert not f1, "Frame 1: should not fail"

    # Feed frame 2
    v2, f2, d2 = await feed_frame(dut, bytes(frame2))
    assert d2, "Frame 2: frame_done not asserted"
    assert f2, "Frame 2: should fail (corrupted)"
    assert not v2, "Frame 2: should not be valid"

    dut._log.info("Back-to-back: frame 1 valid, frame 2 fail (correct)")
