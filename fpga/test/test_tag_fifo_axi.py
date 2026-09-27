"""Cocotb tests for tag_fifo_axi — descriptor + addressed BRAM architecture.

Core test loop (every test exercises this): write frame bytes via psdu_packer
interface, write tag via tag_wr_valid, read tag via AXI, read PSDU bytes via
AXI cursor — verify byte-for-byte match.
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
from axi_lite_driver import AxiLiteMaster

# Register offsets
REG_STATUS    = 0x00
REG_TAG_LO    = 0x04
REG_TAG_HI    = 0x08
REG_CONTROL   = 0x0C
REG_FRAME_CNT = 0x10
REG_DROP_CNT  = 0x14
REG_PSDU_DATA = 0x28

# STATUS bits
STATUS_TAG_EMPTY = 1 << 0
STATUS_TAG_FULL  = 1 << 1


async def reset(dut):
    """Drive rst=1 for 5 cycles, deassert, wait 3 cycles."""
    dut.tag_wr_valid.value = 0
    dut.tag_wr_rate.value = 0
    dut.tag_wr_length.value = 0
    dut.tag_wr_fcs_ok.value = 0
    dut.psdu_byte_in.value = 0
    dut.psdu_byte_valid.value = 0
    dut.psdu_frame_start.value = 0
    dut.psdu_frame_done.value = 0
    dut.tag_abort_in.value = 0
    dut.rst.value = 1
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 3)


async def setup(dut):
    """Start clock, reset, return AXI master."""
    clock = Clock(dut.clk, 10, units='ns')
    cocotb.start_soon(clock.start())
    axi = AxiLiteMaster(dut, prefix="s_axi_", clk=dut.clk)
    await axi._init_signals()
    await reset(dut)
    return axi


async def write_psdu_frame(dut, payload, rate=6, length=None, fcs_ok=True):
    """Write a complete frame: psdu_frame_start, bytes, psdu_frame_done, tag_wr_valid.

    Args:
        dut: DUT handle
        payload: list/bytes of PSDU payload bytes (excluding FCS)
        rate: rate field for tag (4-bit)
        length: PSDU length including FCS (default: len(payload) + 4)
        fcs_ok: whether FCS passed

    The timing mirrors real hardware:
      - psdu_frame_start pulse
      - bytes stream in (one per cycle)
      - psdu_frame_done pulse
      - ~5 cycles gap (simulates FCS check time)
      - tag_wr_valid pulse
    """
    if length is None:
        length = len(payload) + 4

    # Frame start
    dut.psdu_frame_start.value = 1
    await RisingEdge(dut.clk)
    dut.psdu_frame_start.value = 0

    # Stream bytes
    for b in payload:
        dut.psdu_byte_in.value = b
        dut.psdu_byte_valid.value = 1
        await RisingEdge(dut.clk)
    dut.psdu_byte_valid.value = 0
    dut.psdu_byte_in.value = 0

    # Frame done
    dut.psdu_frame_done.value = 1
    await RisingEdge(dut.clk)
    dut.psdu_frame_done.value = 0

    # Gap (simulates FCS computation completing)
    await ClockCycles(dut.clk, 5)

    # Tag write (commit)
    dut.tag_wr_valid.value = 1
    dut.tag_wr_rate.value = rate
    dut.tag_wr_length.value = length
    dut.tag_wr_fcs_ok.value = 1 if fcs_ok else 0
    await RisingEdge(dut.clk)
    dut.tag_wr_valid.value = 0
    dut.tag_wr_fcs_ok.value = 0
    await RisingEdge(dut.clk)


async def read_psdu_frame(axi, expected_payload, expected_rate=None, expected_fcs_ok=True):
    """Read a frame from AXI: TAG_LO, TAG_HI (sets cursor), PSDU_DATA × N.

    Asserts byte-for-byte match against expected_payload.
    Returns (tag_lo, tag_hi, read_bytes).
    """
    # Read TAG_LO (peek)
    tag_lo, _ = await axi.read(REG_TAG_LO)

    # Verify tag fields
    fcs_ok = (tag_lo >> 23) & 1
    rate = (tag_lo >> 19) & 0xF
    length = (tag_lo >> 4) & 0xFFF

    assert fcs_ok == (1 if expected_fcs_ok else 0), \
        f"fcs_ok mismatch: got {fcs_ok}, expected {1 if expected_fcs_ok else 0}"
    if expected_rate is not None:
        assert rate == expected_rate, \
            f"rate mismatch: got {rate}, expected {expected_rate}"

    # Read TAG_HI (pops tag, sets cursor)
    tag_hi, _ = await axi.read(REG_TAG_HI)
    psdu_addr = tag_hi & 0x3FFF

    # Read PSDU bytes
    byte_count = length - 4  # PSDU length minus FCS
    read_bytes = []
    for i in range(byte_count):
        data, _ = await axi.read(REG_PSDU_DATA)
        read_bytes.append(data & 0xFF)

    # Verify byte-for-byte
    assert len(read_bytes) == len(expected_payload), \
        f"length mismatch: read {len(read_bytes)}, expected {len(expected_payload)}"
    for i, (got, exp) in enumerate(zip(read_bytes, expected_payload)):
        assert got == exp, \
            f"byte {i} mismatch: got 0x{got:02x}, expected 0x{exp:02x}"

    return tag_lo, tag_hi, read_bytes


# =============================================================================
# Tests
# =============================================================================


@cocotb.test()
async def test_single_frame_write_read(dut):
    """Write one frame, read it back byte-for-byte via AXI."""
    axi = await setup(dut)

    payload = list(range(20))  # 20 bytes: 0x00..0x13
    await write_psdu_frame(dut, payload, rate=6, fcs_ok=True)

    # Verify not empty
    status, _ = await axi.read(REG_STATUS)
    assert not (status & STATUS_TAG_EMPTY), "Tag FIFO should not be empty"

    # Full round-trip: read tag + PSDU
    await read_psdu_frame(axi, payload, expected_rate=6, expected_fcs_ok=True)

    # Should now be empty
    status, _ = await axi.read(REG_STATUS)
    assert status & STATUS_TAG_EMPTY, "Tag FIFO should be empty after pop"


@cocotb.test()
async def test_fcs_fail_no_bytes(dut):
    """FCS-fail frame: tag is emitted but PSDU addr=0, no bytes to read."""
    axi = await setup(dut)

    payload = [0xAA] * 50
    await write_psdu_frame(dut, payload, rate=12, fcs_ok=False)

    # Tag should exist
    status, _ = await axi.read(REG_STATUS)
    assert not (status & STATUS_TAG_EMPTY)

    # Read TAG_LO — fcs_ok should be 0
    tag_lo, _ = await axi.read(REG_TAG_LO)
    fcs_ok = (tag_lo >> 23) & 1
    assert fcs_ok == 0, "Expected fcs_ok=0 for failed frame"

    # Read TAG_HI — addr should be 0 (no valid PSDU)
    tag_hi, _ = await axi.read(REG_TAG_HI)
    psdu_addr = tag_hi & 0x3FFF
    assert psdu_addr == 0, f"Expected psdu_addr=0 for FCS-fail, got {psdu_addr}"

    # FIFO should be empty now
    status, _ = await axi.read(REG_STATUS)
    assert status & STATUS_TAG_EMPTY


@cocotb.test()
async def test_multi_frame_sequential(dut):
    """Three frames back-to-back, each read correctly."""
    axi = await setup(dut)

    frames = [
        (list(range(10)), 0xB),       # 10 bytes, rate code 0xB (6 Mbps)
        (list(range(100, 150)), 0xF), # 50 bytes, rate code 0xF (9 Mbps)
        ([0xFF] * 200, 0x9),          # 200 bytes, rate code 0x9 (24 Mbps)
    ]

    # Write all three
    for payload, rate_code in frames:
        await write_psdu_frame(dut, payload, rate=rate_code, fcs_ok=True)

    # Read all three — verify each is correct
    for payload, rate_code in frames:
        await read_psdu_frame(axi, payload, expected_rate=rate_code, expected_fcs_ok=True)

    # Should be empty
    status, _ = await axi.read(REG_STATUS)
    assert status & STATUS_TAG_EMPTY


@cocotb.test()
async def test_cursor_abandon(dut):
    """Pop a tag but don't read all bytes, then pop next tag — cursor resets."""
    axi = await setup(dut)

    payload1 = list(range(30))
    payload2 = list(range(50, 80))

    await write_psdu_frame(dut, payload1, rate=6, fcs_ok=True)
    await write_psdu_frame(dut, payload2, rate=12, fcs_ok=True)

    # Pop first tag, read only 5 of 30 bytes (abandon the rest)
    tag_lo, _ = await axi.read(REG_TAG_LO)
    tag_hi, _ = await axi.read(REG_TAG_HI)
    for i in range(5):
        data, _ = await axi.read(REG_PSDU_DATA)
        assert (data & 0xFF) == payload1[i]

    # Now pop second tag — cursor should reset to frame 2's base
    await read_psdu_frame(axi, payload2, expected_rate=12, expected_fcs_ok=True)


@cocotb.test()
async def test_fcs_fail_between_good_frames(dut):
    """Good frame, FCS-fail frame, good frame — bytes don't get corrupted."""
    axi = await setup(dut)

    payload1 = [0x11] * 25
    payload_bad = [0xDE, 0xAD] * 50  # 100 bytes that should be reclaimed
    payload2 = [0x22] * 40

    await write_psdu_frame(dut, payload1, rate=0xB, fcs_ok=True)
    await write_psdu_frame(dut, payload_bad, rate=0xB, fcs_ok=False)
    await write_psdu_frame(dut, payload2, rate=0x9, fcs_ok=True)

    # Read frame 1 — good
    await read_psdu_frame(axi, payload1, expected_rate=0xB, expected_fcs_ok=True)

    # Read frame 2 — FCS fail, no bytes
    tag_lo, _ = await axi.read(REG_TAG_LO)
    assert ((tag_lo >> 23) & 1) == 0  # fcs_ok = 0
    tag_hi, _ = await axi.read(REG_TAG_HI)  # pop it

    # Read frame 3 — good, bytes must be correct (not contaminated by frame 2)
    await read_psdu_frame(axi, payload2, expected_rate=0x9, expected_fcs_ok=True)


@cocotb.test()
async def test_abort_rewinds_bram(dut):
    """Frame abort (watchdog/L-SIG fail) rewinds BRAM, next frame is correct."""
    axi = await setup(dut)

    # Start a frame that will be aborted
    dut.psdu_frame_start.value = 1
    await RisingEdge(dut.clk)
    dut.psdu_frame_start.value = 0

    # Write some bytes (partial frame)
    for b in [0xDE, 0xAD, 0xBE, 0xEF]:
        dut.psdu_byte_in.value = b
        dut.psdu_byte_valid.value = 1
        await RisingEdge(dut.clk)
    dut.psdu_byte_valid.value = 0

    # Abort
    dut.tag_abort_in.value = 1
    await RisingEdge(dut.clk)
    dut.tag_abort_in.value = 0
    await ClockCycles(dut.clk, 3)

    # Now write a good frame — it should occupy the same BRAM space
    payload = [0x42] * 15
    await write_psdu_frame(dut, payload, rate=9, fcs_ok=True)

    # Read it back
    await read_psdu_frame(axi, payload, expected_rate=9, expected_fcs_ok=True)


@cocotb.test()
async def test_tag_fifo_overflow(dut):
    """Fill 16 tag entries, 17th is dropped. First 16 are readable."""
    axi = await setup(dut)

    # Write 16 frames (fills tag FIFO)
    for i in range(16):
        payload = [i] * 5
        await write_psdu_frame(dut, payload, rate=6, fcs_ok=True)

    # Verify full
    status, _ = await axi.read(REG_STATUS)
    assert status & STATUS_TAG_FULL, "Tag FIFO should be full after 16 entries"

    # Write 17th — should be dropped
    payload_17 = [0xFF] * 10
    await write_psdu_frame(dut, payload_17, rate=6, fcs_ok=True)

    # Check drop count
    drop_cnt, _ = await axi.read(REG_DROP_CNT)
    assert drop_cnt == 1, f"Expected drop_cnt=1, got {drop_cnt}"

    # Read first frame — should still be correct
    await read_psdu_frame(axi, [0] * 5, expected_rate=6, expected_fcs_ok=True)


@cocotb.test()
async def test_bram_wrap(dut):
    """Write enough frames to wrap the 16KB BRAM boundary — reads stay correct."""
    axi = await setup(dut)

    # Write and immediately drain frames to advance wr_ptr near boundary.
    # Each frame is 1000 bytes → ~16 frames to wrap 16384 bytes.
    for i in range(17):
        payload = [(i * 7 + j) & 0xFF for j in range(1000)]
        await write_psdu_frame(dut, payload, rate=0x9, fcs_ok=True)
        # Drain immediately so tag FIFO doesn't overflow
        await read_psdu_frame(axi, payload, expected_rate=0x9, expected_fcs_ok=True)

    # One more after wrap
    payload_final = [0xCA, 0xFE] * 50  # 100 bytes
    await write_psdu_frame(dut, payload_final, rate=6, fcs_ok=True)
    await read_psdu_frame(axi, payload_final, expected_rate=6, expected_fcs_ok=True)


@cocotb.test()
async def test_flush_resets_everything(dut):
    """Flush register clears tag FIFO and BRAM pointers."""
    axi = await setup(dut)

    # Write a frame
    payload = [0xAB] * 30
    await write_psdu_frame(dut, payload, rate=6, fcs_ok=True)

    # Verify not empty
    status, _ = await axi.read(REG_STATUS)
    assert not (status & STATUS_TAG_EMPTY)

    # Flush
    await axi.write(REG_CONTROL, 0x01)
    await ClockCycles(dut.clk, 3)

    # Should be empty
    status, _ = await axi.read(REG_STATUS)
    assert status & STATUS_TAG_EMPTY

    # Write a new frame — should work fine (pointers reset)
    payload2 = [0xCD] * 10
    await write_psdu_frame(dut, payload2, rate=9, fcs_ok=True)
    await read_psdu_frame(axi, payload2, expected_rate=9, expected_fcs_ok=True)


@cocotb.test()
async def test_frame_count(dut):
    """FRAME_CNT increments for every tag_wr_valid (even drops)."""
    axi = await setup(dut)

    for i in range(5):
        await write_psdu_frame(dut, [i] * 3, rate=6, fcs_ok=True)

    frame_cnt, _ = await axi.read(REG_FRAME_CNT)
    assert frame_cnt == 5, f"Expected frame_cnt=5, got {frame_cnt}"


@cocotb.test()
async def test_ring_wrap_guard_large_frame(dut):
    """A frame that would wrap the 16KB ring must be suppressed, not clobber.

    Fill the ring to 13999 bytes used (all fcs_ok, all unread), then push a
    4091-byte frame (max legal PSDU). 13999 + 4091 > 16384, so admitting it
    wraps into live unread frames. The overflow guard must suppress the
    writes and the first frame's bytes must remain intact.

    Bug: threshold 14000 admitted this frame (13999 < 14000) and it
    clobbered the oldest unread frame's bytes. Threshold must be
    16384 - 4091 = 12293.
    """
    axi = await setup(dut)

    # Frames summing to 13999 bytes: 4091*3 + 1726
    sizes = [4091, 4091, 4091, 1726]
    frames = []
    for i, n in enumerate(sizes):
        payload = [(i * 0x40 + j) & 0xFF for j in range(n)]
        frames.append(payload)
        await write_psdu_frame(dut, payload, rate=0x9, fcs_ok=True)

    # Attempt a max-size frame at space_used=13999
    big = [0xC3] * 4091
    await write_psdu_frame(dut, big, rate=0x9, fcs_ok=True, length=4095)

    # Pop tags for the four fill frames (their bytes must be intact) and the
    # oversize frame. The safety property: no wrap-clobber of live frames.
    for i, payload in enumerate(frames):
        tag_lo, _ = await axi.read(REG_TAG_LO)
        tag_hi, _ = await axi.read(REG_TAG_HI)
        for j in range(len(payload)):
            data, _ = await axi.read(REG_PSDU_DATA)
            assert (data & 0xFF) == payload[j], \
                f"frame {i} byte {j} clobbered: got 0x{data & 0xFF:02x}, " \
                f"expected 0x{payload[j]:02x}"

    # Oversize frame's tag still exists (fcs_ok=1, suppressed writes)
    tag_lo, _ = await axi.read(REG_TAG_LO)
    tag_hi, _ = await axi.read(REG_TAG_HI)

    dut._log.info("PASS: oversize frame writes suppressed, all live frames intact")


@cocotb.test()
async def test_fail_frame_base_accounting(dut):
    """FCS-fail frames must store their frame base (not 0) for space accounting.

    space_used = wr_ptr - oldest_addr. Storing 0 for a fail frame makes
    space_used balloon once the fail frame becomes the oldest unread tag,
    suppressing frames long before the ring is actually full.

    Bug: after popping the frames before a fail frame, oldest_addr=0 while
    the true live region starts at the fail frame's base — subsequent good
    frames get suppressed even though the ring has plenty of room.
    """
    axi = await setup(dut)

    # Two good frames, then a fail frame (base lands at 8182)
    p1 = [0x10 + j & 0xFF for j in range(4091)]
    p2 = [0x20 + j & 0xFF for j in range(4091)]
    p_fail = [0xB5] * 100
    await write_psdu_frame(dut, p1, rate=0x9, fcs_ok=True)
    await write_psdu_frame(dut, p2, rate=0x9, fcs_ok=True)
    await write_psdu_frame(dut, p_fail, rate=0x9, fcs_ok=False)

    # Pop the two good frames; the fail frame is now the oldest unread tag.
    for _ in range(2):
        await axi.read(REG_TAG_LO)
        await axi.read(REG_TAG_HI)

    # Fail frame's tag: fcs_ok=0, no bytes read by ARM; pop it later.

    # Now push good frames. True live region is small (fail bytes rewound),
    # so these must all be admitted and stored at/after the fail frame base.
    p4 = [0x40 + j & 0xFF for j in range(4091)]
    p5 = [0x50 + j & 0xFF for j in range(4091)]
    p6 = [0x60 + j & 0xFF for j in range(500)]
    await write_psdu_frame(dut, p4, rate=0x9, fcs_ok=True)
    await write_psdu_frame(dut, p5, rate=0x9, fcs_ok=True)
    await write_psdu_frame(dut, p6, rate=0x9, fcs_ok=True)

    # Pop fail frame tag
    tag_lo, _ = await axi.read(REG_TAG_LO)
    assert ((tag_lo >> 23) & 1) == 0, "fail frame tag must have fcs_ok=0"
    await axi.read(REG_TAG_HI)

    # Frames after the fail frame must be readable byte-for-byte.
    await read_psdu_frame(axi, p4, expected_rate=0x9, expected_fcs_ok=True)
    await read_psdu_frame(axi, p5, expected_rate=0x9, expected_fcs_ok=True)
    await read_psdu_frame(axi, p6, expected_rate=0x9, expected_fcs_ok=True)

    dut._log.info("PASS: fail frame base accounting keeps ring capacity")
