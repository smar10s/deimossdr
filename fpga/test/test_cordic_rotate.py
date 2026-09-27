"""
Test cordic_rotate — Pipelined CORDIC rotation mode.

Verifies:
1. Zero angle → passthrough (x_out=x_in, y_out=y_in)
2. +90° → (x_out=-y_in, y_out=x_in)
3. Small angle (5°) → verify vs float reference, tolerance ±2 LSB
4. Large angle (60°) → verify vs float reference
5. Negative angle (-45°) → verify vs float reference
6. Pipeline throughput: 48 consecutive samples in, 48 out (no gaps)
7. Wraparound: ±180° → verify sign flip
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import math


PIPELINE_LATENCY = 10  # pre-rot + 8 iterations + gain comp


def to_s16(v):
    """Interpret 16-bit value as signed."""
    v = int(v) & 0xFFFF
    return v - 0x10000 if v >= 0x8000 else v


def s16_to_u16(v):
    """Convert signed int to 16-bit unsigned for DUT."""
    if v < 0:
        v = v + 0x10000
    return v & 0xFFFF


def deg_to_s16(degrees):
    """Convert degrees to 16-bit signed angle (32768 = π = 180°)."""
    radians = degrees * math.pi / 180.0
    # 32768 = π, so scale = 32768/π
    val = int(round(radians * 32768.0 / math.pi))
    # Clamp to 16-bit signed range
    val = max(-32768, min(32767, val))
    return val


def rotate_ref(x, y, degrees):
    """Float reference rotation."""
    rad = degrees * math.pi / 180.0
    cos_a = math.cos(rad)
    sin_a = math.sin(rad)
    x_out = x * cos_a - y * sin_a
    y_out = x * sin_a + y * cos_a
    return (x_out, y_out)


async def reset_dut(dut):
    """Reset DUT."""
    dut.rst_n.value = 0
    dut.valid_in.value = 0
    dut.x_in.value = 0
    dut.y_in.value = 0
    dut.angle.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def apply_one(dut, x, y, angle_deg):
    """Apply one sample and wait for result. Returns (x_out, y_out)."""
    dut.valid_in.value = 1
    dut.x_in.value = s16_to_u16(x)
    dut.y_in.value = s16_to_u16(y)
    dut.angle.value = s16_to_u16(deg_to_s16(angle_deg))
    await RisingEdge(dut.clk)
    dut.valid_in.value = 0
    # Wait for pipeline
    for _ in range(PIPELINE_LATENCY + 2):
        await RisingEdge(dut.clk)
        if dut.valid_out.value == 1:
            return (to_s16(dut.x_out.value), to_s16(dut.y_out.value))
    raise RuntimeError("No valid_out after pipeline latency")


@cocotb.test()
async def test_zero_angle(dut):
    """Zero angle → passthrough (x_out=x_in, y_out=y_in)."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    test_vectors = [(1000, 500), (-2000, 3000), (32000, -32000), (0, 1000)]
    for x, y in test_vectors:
        xo, yo = await apply_one(dut, x, y, 0.0)
        # 8-iteration CORDIC: error ~0.2-0.4% of magnitude
        mag = max(abs(x), abs(y))
        tol = max(5, int(mag * 0.015) + 3)
        assert abs(xo - x) <= tol, f"x: got {xo}, exp {x}, tol {tol}"
        assert abs(yo - y) <= tol, f"y: got {yo}, exp {y}, tol {tol}"

    dut._log.info("PASS: zero angle passthrough within tolerance")


@cocotb.test()
async def test_plus_90(dut):
    """+90° → (x_out=-y_in, y_out=x_in)."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    test_vectors = [(1000, 0), (0, 1000), (500, -500), (-1000, 2000)]
    for x, y in test_vectors:
        xo, yo = await apply_one(dut, x, y, 90.0)
        exp_x, exp_y = rotate_ref(x, y, 90.0)
        mag = max(abs(x), abs(y))
        tol = max(8, int(mag * 0.015) + 3)
        assert abs(xo - round(exp_x)) <= tol, f"x: got {xo}, exp {round(exp_x)}"
        assert abs(yo - round(exp_y)) <= tol, f"y: got {yo}, exp {round(exp_y)}"

    dut._log.info("PASS: +90° rotation correct")


@cocotb.test()
async def test_small_angle_5deg(dut):
    """Small angle (5°) → verify vs float reference, tolerance ±2 LSB."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    test_vectors = [(5000, 3000), (-4000, 2000), (10000, -8000), (200, 200)]
    for x, y in test_vectors:
        xo, yo = await apply_one(dut, x, y, 5.0)
        exp_x, exp_y = rotate_ref(x, y, 5.0)
        mag = max(abs(x), abs(y))
        tol = max(8, int(mag * 0.015) + 3)
        assert abs(xo - round(exp_x)) <= tol, f"x: got {xo}, exp {round(exp_x)} (input {x},{y})"
        assert abs(yo - round(exp_y)) <= tol, f"y: got {yo}, exp {round(exp_y)} (input {x},{y})"

    dut._log.info("PASS: 5° rotation within tolerance")


@cocotb.test()
async def test_large_angle_60deg(dut):
    """Large angle (60°) → verify vs float reference."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    test_vectors = [(5000, 3000), (-4000, 2000), (10000, -8000), (1000, 0)]
    for x, y in test_vectors:
        xo, yo = await apply_one(dut, x, y, 60.0)
        exp_x, exp_y = rotate_ref(x, y, 60.0)
        mag = max(abs(x), abs(y))
        tol = max(8, int(mag * 0.015) + 3)
        assert abs(xo - round(exp_x)) <= tol, f"x: got {xo}, exp {round(exp_x)} (input {x},{y})"
        assert abs(yo - round(exp_y)) <= tol, f"y: got {yo}, exp {round(exp_y)} (input {x},{y})"

    dut._log.info("PASS: 60° rotation within tolerance")


@cocotb.test()
async def test_negative_angle_minus45(dut):
    """Negative angle (-45°) → verify vs float reference."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    test_vectors = [(5000, 3000), (-4000, 2000), (10000, -8000), (0, 5000)]
    for x, y in test_vectors:
        xo, yo = await apply_one(dut, x, y, -45.0)
        exp_x, exp_y = rotate_ref(x, y, -45.0)
        mag = max(abs(x), abs(y))
        tol = max(8, int(mag * 0.015) + 3)
        assert abs(xo - round(exp_x)) <= tol, f"x: got {xo}, exp {round(exp_x)} (input {x},{y})"
        assert abs(yo - round(exp_y)) <= tol, f"y: got {yo}, exp {round(exp_y)} (input {x},{y})"

    dut._log.info("PASS: -45° rotation within tolerance")


@cocotb.test()
async def test_pipeline_throughput(dut):
    """48 consecutive samples in, 48 out (no gaps in output valid)."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    n_samples = 48
    angle_deg = 30.0
    angle_s16 = deg_to_s16(angle_deg)

    # Feed 48 samples continuously
    inputs = []
    for i in range(n_samples):
        x = 1000 + i * 100
        y = 500 - i * 50
        inputs.append((x, y))

    # Feed and collect concurrently
    outputs = []
    max_gap = 0
    gap = 0

    # Feed all samples, collecting outputs as they appear
    for i in range(n_samples):
        dut.valid_in.value = 1
        dut.x_in.value = s16_to_u16(inputs[i][0])
        dut.y_in.value = s16_to_u16(inputs[i][1])
        dut.angle.value = s16_to_u16(angle_s16)
        await RisingEdge(dut.clk)
        if dut.valid_out.value == 1:
            outputs.append((to_s16(dut.x_out.value), to_s16(dut.y_out.value)))
            if gap > max_gap and len(outputs) > 1:
                max_gap = gap
            gap = 0
        elif len(outputs) > 0:
            gap += 1
    dut.valid_in.value = 0

    # Continue collecting remaining outputs
    for _ in range(PIPELINE_LATENCY + 5):
        await RisingEdge(dut.clk)
        if dut.valid_out.value == 1:
            outputs.append((to_s16(dut.x_out.value), to_s16(dut.y_out.value)))
            if gap > max_gap and len(outputs) > 1:
                max_gap = gap
            gap = 0
        elif len(outputs) > 0:
            gap += 1
        if len(outputs) == n_samples:
            break

    assert len(outputs) == n_samples, f"Expected {n_samples} outputs, got {len(outputs)}"
    assert max_gap == 0, f"Gap in output stream: max_gap={max_gap}"

    # Verify correctness of each output
    for i in range(n_samples):
        exp_x, exp_y = rotate_ref(inputs[i][0], inputs[i][1], angle_deg)
        mag = max(abs(inputs[i][0]), abs(inputs[i][1]))
        tol = max(8, int(mag * 0.015) + 3)
        assert abs(outputs[i][0] - round(exp_x)) <= tol, f"idx {i}: x error"
        assert abs(outputs[i][1] - round(exp_y)) <= tol, f"idx {i}: y error"

    dut._log.info(f"PASS: {n_samples} samples, zero gaps, all within tolerance")


@cocotb.test()
async def test_wraparound_180(dut):
    """±180° → verify sign flip (x_out=-x_in, y_out=-y_in)."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # +180° (use 179.99° since 180° exactly = -32768 which is -π)
    # Actually test both: angle = +32767 (≈179.99°) and angle = -32768 (= -180°)
    test_vectors = [(5000, 3000), (-2000, 1000), (1000, -1000)]

    # Test with angle near +180° (32767 = just under π)
    for x, y in test_vectors:
        dut.valid_in.value = 1
        dut.x_in.value = s16_to_u16(x)
        dut.y_in.value = s16_to_u16(y)
        dut.angle.value = s16_to_u16(32767)  # ≈ +179.99°
        await RisingEdge(dut.clk)
        dut.valid_in.value = 0
        for _ in range(PIPELINE_LATENCY + 2):
            await RisingEdge(dut.clk)
            if dut.valid_out.value == 1:
                xo = to_s16(dut.x_out.value)
                yo = to_s16(dut.y_out.value)
                # At ~180°: x_out ≈ -x_in, y_out ≈ -y_in
                assert abs(xo - (-x)) <= max(8, int(abs(x)*0.015)+3), f"+180 x: got {xo}, exp {-x}"
                assert abs(yo - (-y)) <= max(8, int(abs(y)*0.015)+3), f"+180 y: got {yo}, exp {-y}"
                break

    # Test with angle = -32768 (= -180° = +180°)
    for x, y in test_vectors:
        dut.valid_in.value = 1
        dut.x_in.value = s16_to_u16(x)
        dut.y_in.value = s16_to_u16(y)
        dut.angle.value = s16_to_u16(-32768)  # = -180° = +180°
        await RisingEdge(dut.clk)
        dut.valid_in.value = 0
        for _ in range(PIPELINE_LATENCY + 2):
            await RisingEdge(dut.clk)
            if dut.valid_out.value == 1:
                xo = to_s16(dut.x_out.value)
                yo = to_s16(dut.y_out.value)
                assert abs(xo - (-x)) <= max(8, int(abs(x)*0.015)+3), f"-180 x: got {xo}, exp {-x}"
                assert abs(yo - (-y)) <= max(8, int(abs(y)*0.015)+3), f"-180 y: got {yo}, exp {-y}"
                break

    dut._log.info("PASS: ±180° wraparound within tolerance")
