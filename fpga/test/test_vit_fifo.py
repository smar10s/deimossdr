"""Test for vit_fifo — backpressure FIFO between soft_pairer and Viterbi.

This FIFO has a registered output (1-cycle latency from internal read to
rd_valid assertion). This breaks the combinational timing path from
Viterbi busy → FIFO read → valid_in.
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles


async def reset(dut):
    dut.rst_n.value = 0
    dut.frame_start.value = 0
    dut.wr_valid.value = 0
    dut.wr_soft0.value = 0
    dut.wr_soft1.value = 0
    dut.vit_busy.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


@cocotb.test()
async def test_passthrough(dut):
    """Data passes through with small latency when vit_busy=0."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    # Write 10 pairs, busy=0
    received = []
    for i in range(10):
        dut.wr_valid.value = 1
        dut.wr_soft0.value = i & 0xFF
        dut.wr_soft1.value = (i + 100) & 0xFF
        await RisingEdge(dut.clk)
        if dut.rd_valid.value == 1:
            received.append((int(dut.rd_soft0.value), int(dut.rd_soft1.value)))

    dut.wr_valid.value = 0
    # Drain remaining (BRAM read adds 1 extra cycle latency vs dist RAM)
    for _ in range(15):
        await RisingEdge(dut.clk)
        if dut.rd_valid.value == 1:
            received.append((int(dut.rd_soft0.value), int(dut.rd_soft1.value)))

    assert len(received) == 10, f"Expected 10 pairs, got {len(received)}"
    for i, (s0, s1) in enumerate(received):
        assert s0 == (i & 0xFF), f"Pair {i}: soft0={s0}, expected {i & 0xFF}"
        assert s1 == ((i + 100) & 0xFF), f"Pair {i}: soft1={s1}, expected {(i+100) & 0xFF}"


@cocotb.test()
async def test_backpressure(dut):
    """When vit_busy=1, data is buffered and delivered when busy deasserts."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    # Write 20 pairs while busy
    dut.vit_busy.value = 1
    for i in range(20):
        dut.wr_valid.value = 1
        dut.wr_soft0.value = i & 0xFF
        dut.wr_soft1.value = (i * 2) & 0xFF
        await RisingEdge(dut.clk)

    dut.wr_valid.value = 0
    await ClockCycles(dut.clk, 3)

    # rd_valid may be asserted (output reg loaded) but data won't advance
    # because busy is high — the "consumed" condition is false.
    # Actually with registered output: the output register can be loaded
    # even while busy (it loads from FIFO, but won't be consumed).
    # Let's just verify correct delivery after busy deasserts.

    # Deassert busy — data should drain
    dut.vit_busy.value = 0
    received = []
    for _ in range(40):
        await RisingEdge(dut.clk)
        if dut.rd_valid.value == 1:
            received.append((int(dut.rd_soft0.value), int(dut.rd_soft1.value)))

    assert len(received) == 20, f"Expected 20 pairs, got {len(received)}"
    for i, (s0, s1) in enumerate(received):
        assert s0 == (i & 0xFF), f"Pair {i}: soft0={s0}, expected {i & 0xFF}"
        assert s1 == ((i * 2) & 0xFF), f"Pair {i}: soft1={s1}, expected {(i*2) & 0xFF}"


@cocotb.test()
async def test_stall_burst(dut):
    """Simulate Viterbi stall pattern: accept 35, then 135-cycle stall."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    total_pairs = 100
    received = []

    # Feed pairs at 1 per 2 clocks (mimicking soft_pairer rate)
    pair_idx = 0
    stall_counter = 0
    accepted_since_stall = 0
    stalling = False

    for _ in range(1200):
        await RisingEdge(dut.clk)

        # Collect output (Viterbi "accepts" when rd_valid && !busy)
        if dut.rd_valid.value == 1 and not stalling:
            received.append((int(dut.rd_soft0.value), int(dut.rd_soft1.value)))
            accepted_since_stall += 1
            # Simulate Viterbi going busy after 35 accepted pairs
            if accepted_since_stall >= 35:
                stalling = True
                stall_counter = 135
                dut.vit_busy.value = 1
                accepted_since_stall = 0

        # Manage stall
        if stalling:
            stall_counter -= 1
            if stall_counter <= 0:
                stalling = False
                dut.vit_busy.value = 0

        # Feed input at 1 pair every 2 clocks
        if pair_idx < total_pairs and (_ % 2 == 0):
            dut.wr_valid.value = 1
            dut.wr_soft0.value = pair_idx & 0xFF
            dut.wr_soft1.value = (pair_idx + 50) & 0xFF
            pair_idx += 1
        else:
            dut.wr_valid.value = 0

    # Drain any remaining
    dut.wr_valid.value = 0
    dut.vit_busy.value = 0
    for _ in range(300):
        await RisingEdge(dut.clk)
        if dut.rd_valid.value == 1:
            received.append((int(dut.rd_soft0.value), int(dut.rd_soft1.value)))

    assert len(received) == total_pairs, f"Expected {total_pairs} pairs, got {len(received)}"
    # Verify ordering preserved
    for i, (s0, s1) in enumerate(received):
        assert s0 == (i & 0xFF), f"Pair {i}: soft0={s0}, expected {i & 0xFF}"
        assert s1 == ((i + 50) & 0xFF), f"Pair {i}: soft1={s1}, expected {(i+50) & 0xFF}"


@cocotb.test()
async def test_frame_start_flush(dut):
    """frame_start resets FIFO state."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    # Fill with some data while busy (so data stays in FIFO)
    dut.vit_busy.value = 1
    for i in range(10):
        dut.wr_valid.value = 1
        dut.wr_soft0.value = 0xAA
        dut.wr_soft1.value = 0xBB
        await RisingEdge(dut.clk)
    dut.wr_valid.value = 0
    await ClockCycles(dut.clk, 2)

    # Pulse frame_start
    dut.frame_start.value = 1
    await RisingEdge(dut.clk)
    dut.frame_start.value = 0
    dut.vit_busy.value = 0
    await ClockCycles(dut.clk, 3)

    # FIFO should be empty now — no output
    for _ in range(5):
        await RisingEdge(dut.clk)
        assert dut.rd_valid.value == 0, "FIFO should be empty after frame_start"


@cocotb.test()
async def test_no_overflow_128(dut):
    """128 entries can be stored without overflow."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    dut.vit_busy.value = 1
    overflow_seen = False
    for i in range(128):
        dut.wr_valid.value = 1
        dut.wr_soft0.value = i & 0xFF
        dut.wr_soft1.value = 0
        await RisingEdge(dut.clk)
        if dut.overflow.value == 1:
            overflow_seen = True

    # The output register may have consumed one entry (loaded from FIFO
    # even while busy, since it fills the reg). So effective capacity might
    # be 128 + 1 (output reg). Let's just verify no overflow at 128 writes.
    # With registered output and busy=1: the output reg CAN still be loaded
    # (consumed = rd_valid && !vit_busy = 0, so the output reg stays loaded
    # once filled, and rd_ptr advances once). Check:
    # Actually no: consumed = rd_valid && !vit_busy. If busy=1, consumed=0.
    # So the output reg loads once (!rd_valid case), then stalls. rd_ptr
    # advances by 1. Effective FIFO holds 127 more = 128 total with output reg.
    # So 128 writes with one consumed by output reg = 127 in FIFO + 1 in reg = ok.

    assert not overflow_seen, "Should not overflow at depth 128"

    # Next write should overflow (129th entry)
    dut.wr_valid.value = 1
    dut.wr_soft0.value = 0xFF
    dut.wr_soft1.value = 0xFF
    await RisingEdge(dut.clk)
    # After 128 writes: wr_ptr=128, rd_ptr=1 (output reg consumed one).
    # full check: wr_ptr[7]!=rd_ptr[7] && wr_ptr[6:0]==rd_ptr[6:0]
    # wr_ptr=10000000, rd_ptr=00000001 → not full yet (128 vs 1, diff=127)
    # Actually this means we can store MORE than 128. Let me just verify
    # no overflow seen during 128 writes.
    dut.wr_valid.value = 0


@cocotb.test()
async def test_no_overflow_250(dut):
    """250 entries can be stored without overflow (near-full stress test)."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    dut.vit_busy.value = 1
    overflow_seen = False
    for i in range(250):
        dut.wr_valid.value = 1
        dut.wr_soft0.value = i & 0xFF
        dut.wr_soft1.value = (i >> 8) & 0xFF
        await RisingEdge(dut.clk)
        if dut.overflow.value == 1:
            overflow_seen = True
            break

    dut.wr_valid.value = 0
    assert not overflow_seen, "Should not overflow at 250 entries (depth=256)"

    # Now drain and verify data integrity
    dut.vit_busy.value = 0
    received = []
    for _ in range(600):
        await RisingEdge(dut.clk)
        if dut.rd_valid.value == 1:
            s0 = int(dut.rd_soft0.value)
            s1 = int(dut.rd_soft1.value)
            received.append((s0, s1))

    # Verify all 250 entries came back in order
    assert len(received) == 250, f"Expected 250 pairs, got {len(received)}"
    for i in range(250):
        exp_s0 = i & 0xFF
        exp_s1 = (i >> 8) & 0xFF
        assert received[i] == (exp_s0, exp_s1), \
            f"Pair {i}: got {received[i]}, expected ({exp_s0}, {exp_s1})"


@cocotb.test()
async def test_full_signal_held_write(dut):
    """full asserts under sustained backpressure; a held write is not lost."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    # Viterbi stuck busy -> nothing consumed -> FIFO fills
    dut.vit_busy.value = 1

    written = 0
    full_seen = False
    received = []
    for _ in range(300):
        dut.wr_valid.value = 1
        dut.wr_soft0.value = written & 0xFF
        dut.wr_soft1.value = (written + 100) & 0xFF
        await RisingEdge(dut.clk)
        if dut.rd_valid.value == 1 and int(dut.vit_busy.value) == 0:
            received.append((int(dut.rd_soft0.value), int(dut.rd_soft1.value)))
        if int(dut.full.value) == 0:
            written += 1
        else:
            full_seen = True
            break
    assert full_seen, "FIFO never reported full with vit_busy held"
    assert written >= 250, f"full asserted implausibly early ({written} writes)"

    # While full, the upstream holds its output pair (soft_pairer freezes
    # on stall_in): wr_valid stays high and the data is frozen — the pair
    # on the bus may be pending (its write was skipped while full) and
    # must not be replaced. The vit_fifo stall assertion enforces exactly
    # this (an upstream that advances while full would drop pairs).
    for _ in range(8):
        await RisingEdge(dut.clk)
        assert int(dut.full.value) == 1, "full dropped while holding wr_valid"

    # Release busy while still holding wr_valid, data unchanged. The first
    # full==0 cycle is the pending pair's write-back: full is 0 for the
    # whole cycle, so the write lands at its edge. Drop wr_valid in that
    # same cycle so the pair is written exactly once.
    dut.vit_busy.value = 0
    cleared = False
    for _ in range(600):
        await RisingEdge(dut.clk)
        if dut.rd_valid.value == 1:
            received.append((int(dut.rd_soft0.value), int(dut.rd_soft1.value)))
        if int(dut.full.value) == 0:
            cleared = True
            break
    assert cleared, "full never cleared after releasing vit_busy"
    dut.wr_valid.value = 0

    # Wait for the FIFO to drain completely (viterbi consumes ~1/clk).
    for _ in range(600):
        await RisingEdge(dut.clk)
        if dut.rd_valid.value == 1:
            received.append((int(dut.rd_soft0.value), int(dut.rd_soft1.value)))

    # Present a distinct pair for exactly one write cycle into the empty
    # FIFO: it must land intact, after everything else.
    dut.wr_soft0.value = 0xAB
    dut.wr_soft1.value = 0xCD
    dut.wr_valid.value = 1
    await RisingEdge(dut.clk)
    if dut.rd_valid.value == 1:
        received.append((int(dut.rd_soft0.value), int(dut.rd_soft1.value)))
    dut.wr_valid.value = 0

    # Drain the last entry and verify no loss, distinct pair last
    for _ in range(200):
        await RisingEdge(dut.clk)
        if dut.rd_valid.value == 1:
            received.append((int(dut.rd_soft0.value), int(dut.rd_soft1.value)))

    expected = written + 2
    assert len(received) == expected, f"expected {expected} pairs, got {len(received)}"
    assert received[-1] == (0xAB, 0xCD), f"held pair corrupted: {received[-1]}"
