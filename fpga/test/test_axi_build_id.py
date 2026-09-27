"""Cocotb tests for axi_build_id — AXI4-Lite read-only build fingerprint register.

Registers:
    Offset 0x00: BUILD_ID    — content-addressed source fingerprint
    Offset 0x04: PROJECT_ID  — ASCII project magic (e.g. "WIFI" = 0x57494649)
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
from axi_lite_driver import AxiLiteMaster

BUILD_ID_DEFAULT   = 0xDEADBEEF
PROJECT_ID_DEFAULT = 0x57494649


async def reset(dut):
    """Drive aresetn=0 for 5 cycles, then deassert (active-low reset)."""
    dut.s_axi_aresetn.value = 0
    await ClockCycles(dut.s_axi_aclk, 5)
    dut.s_axi_aresetn.value = 1
    await ClockCycles(dut.s_axi_aclk, 3)


async def setup(dut):
    """Start clock, reset, return AxiLiteMaster."""
    clock = Clock(dut.s_axi_aclk, 10, unit='ns')
    cocotb.start_soon(clock.start())

    axi = AxiLiteMaster(dut, prefix="s_axi_", clk=dut.s_axi_aclk)
    await axi._init_signals()
    await reset(dut)
    return axi


@cocotb.test()
async def test_read_returns_build_id(dut):
    """Read offset 0 returns BUILD_ID parameter (default 0xDEADBEEF)."""
    axi = await setup(dut)

    rdata, rresp = await axi.read(0x00)
    assert rresp == 0, f"Expected RRESP=OKAY (0), got {rresp}"
    assert rdata == BUILD_ID_DEFAULT, \
        f"Expected BUILD_ID=0x{BUILD_ID_DEFAULT:08X}, got 0x{rdata:08X}"


@cocotb.test()
async def test_read_returns_project_id(dut):
    """Read offset 0x04 returns PROJECT_ID parameter (0x57494649 = "WIFI")."""
    axi = await setup(dut)

    rdata, rresp = await axi.read(0x04)
    assert rresp == 0, f"Expected RRESP=OKAY (0), got {rresp}"
    assert rdata == PROJECT_ID_DEFAULT, \
        f"Expected PROJECT_ID=0x{PROJECT_ID_DEFAULT:08X}, got 0x{rdata:08X}"


@cocotb.test()
async def test_project_id_unchanged_after_write(dut):
    """Write to offset 0x04, read back — PROJECT_ID unchanged (read-only)."""
    axi = await setup(dut)

    bresp = await axi.write(0x04, 0xCAFEBABE)
    assert bresp == 0, f"Write failed with BRESP={bresp}"

    rdata, rresp = await axi.read(0x04)
    assert rresp == 0, f"Expected RRESP=OKAY (0), got {rresp}"
    assert rdata == PROJECT_ID_DEFAULT, \
        f"Write modified PROJECT_ID! Expected 0x{PROJECT_ID_DEFAULT:08X}, got 0x{rdata:08X}"


@cocotb.test()
async def test_write_accepted_no_error(dut):
    """Write to offset 0 completes with BRESP=OKAY (no bus hang)."""
    axi = await setup(dut)

    bresp = await axi.write(0x00, 0x12345678)
    assert bresp == 0, f"Expected BRESP=OKAY (0), got {bresp}"


@cocotb.test()
async def test_read_after_write_unchanged(dut):
    """Write a value, read back — BUILD_ID remains unchanged (read-only)."""
    axi = await setup(dut)

    # Write an arbitrary value
    bresp = await axi.write(0x00, 0xCAFEBABE)
    assert bresp == 0, f"Write failed with BRESP={bresp}"

    # Read back — should still be BUILD_ID
    rdata, rresp = await axi.read(0x00)
    assert rresp == 0, f"Expected RRESP=OKAY (0), got {rresp}"
    assert rdata == BUILD_ID_DEFAULT, \
        f"Expected BUILD_ID=0x{BUILD_ID_DEFAULT:08X} after write, got 0x{rdata:08X}"
