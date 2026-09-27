"""Test deimos_regs_axi -- AXI-Lite register read/write."""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge


async def axi_write(dut, addr, data):
    """Single AXI-Lite write transaction."""
    dut.s_axi_awaddr.value = addr
    dut.s_axi_awvalid.value = 1
    dut.s_axi_wdata.value = data
    dut.s_axi_wstrb.value = 0xF
    dut.s_axi_wvalid.value = 1
    dut.s_axi_bready.value = 1

    for _ in range(20):
        await RisingEdge(dut.s_axi_aclk)
        if dut.s_axi_bvalid.value:
            break

    dut.s_axi_awvalid.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_bready.value = 0
    await RisingEdge(dut.s_axi_aclk)


async def axi_read(dut, addr):
    """Single AXI-Lite read transaction. Returns 32-bit value."""
    dut.s_axi_araddr.value = addr
    dut.s_axi_arvalid.value = 1
    dut.s_axi_rready.value = 1

    for _ in range(20):
        await RisingEdge(dut.s_axi_aclk)
        if dut.s_axi_rvalid.value:
            break

    val = int(dut.s_axi_rdata.value)
    dut.s_axi_arvalid.value = 0
    await RisingEdge(dut.s_axi_aclk)
    return val


async def reset_dut(dut):
    dut.s_axi_aresetn.value = 0
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_bready.value = 0
    dut.s_axi_rready.value = 0
    dut.phase_inc_in.value = 0
    dut.diag_frames_found_in.value = 0
    dut.diag_frames_rejected_in.value = 0
    dut.diag_drop_cnt_in.value = 0
    dut.diag_clip_cnt_in.value = 0
    await ClockCycles(dut.s_axi_aclk, 5)
    dut.s_axi_aresetn.value = 1
    await ClockCycles(dut.s_axi_aclk, 2)


@cocotb.test()
async def test_defaults(dut):
    """RW registers read back their default values after reset."""
    cocotb.start_soon(Clock(dut.s_axi_aclk, 10, units="ns").start())
    await reset_dut(dut)

    assert await axi_read(dut, 0x00) == 0    # STF_THRESH
    assert await axi_read(dut, 0x04) == 0    # STF_ENABLE
    assert await axi_read(dut, 0x08) == 0    # DIAG_ACQ (inputs zeroed)
    assert await axi_read(dut, 0x0C) == 0    # SNAP_MODE
    assert await axi_read(dut, 0x10) == 0    # DIAG_DROP_CNT (input zeroed)
    assert await axi_read(dut, 0x14) == 0    # DIAG_CLIP_CNT (input zeroed)


@cocotb.test()
async def test_write_read_rw_regs(dut):
    """Write values to RW registers and read them back."""
    cocotb.start_soon(Clock(dut.s_axi_aclk, 10, units="ns").start())
    await reset_dut(dut)

    await axi_write(dut, 0x00, 64)
    assert await axi_read(dut, 0x00) == 64
    assert int(dut.stf_threshold_out.value) == 64

    await axi_write(dut, 0x04, 1)
    assert await axi_read(dut, 0x04) == 1
    assert int(dut.stf_enable_out.value) == 1

    await axi_write(dut, 0x0C, 5)
    assert await axi_read(dut, 0x0C) == 5
    assert int(dut.snap_mode_out.value) == 5


@cocotb.test()
async def test_ro_slots_ignore_writes(dut):
    """Writes to read-only diagnostic slots are silently ignored."""
    cocotb.start_soon(Clock(dut.s_axi_aclk, 10, units="ns").start())
    await reset_dut(dut)

    # Set known input values
    dut.diag_frames_found_in.value = 0x12
    dut.diag_frames_rejected_in.value = 0x34
    dut.diag_drop_cnt_in.value = 0xABCD
    dut.diag_clip_cnt_in.value = 0x5678
    await ClockCycles(dut.s_axi_aclk, 2)

    # Attempt writes (should be silently ignored)
    await axi_write(dut, 0x08, 0xFFFF)
    await axi_write(dut, 0x10, 0xFFFF)
    await axi_write(dut, 0x14, 0xFFFF)

    # Reads should still reflect the input ports, not the written values
    assert await axi_read(dut, 0x08) == 0x1234  # {found[7:0], rejected[7:0]}
    assert await axi_read(dut, 0x10) == 0xABCD
    assert await axi_read(dut, 0x14) == 0x5678


@cocotb.test()
async def test_diag_acq_readback(dut):
    """DIAG_ACQ register packs found[15:8] and rejected[7:0]."""
    cocotb.start_soon(Clock(dut.s_axi_aclk, 10, units="ns").start())
    await reset_dut(dut)

    dut.diag_frames_found_in.value = 7
    dut.diag_frames_rejected_in.value = 3
    await ClockCycles(dut.s_axi_aclk, 2)
    val = await axi_read(dut, 0x08)
    assert val == (7 << 8) | 3, f"Expected {(7<<8)|3}, got {val}"

    dut.diag_frames_found_in.value = 255
    dut.diag_frames_rejected_in.value = 255
    await ClockCycles(dut.s_axi_aclk, 2)
    val = await axi_read(dut, 0x08)
    assert val == 0xFFFF


@cocotb.test()
async def test_diag_counters_readback(dut):
    """DIAG_DROP_CNT and DIAG_CLIP_CNT reflect input ports."""
    cocotb.start_soon(Clock(dut.s_axi_aclk, 10, units="ns").start())
    await reset_dut(dut)

    dut.diag_drop_cnt_in.value = 42
    dut.diag_clip_cnt_in.value = 100
    await ClockCycles(dut.s_axi_aclk, 2)
    assert await axi_read(dut, 0x10) == 42
    assert await axi_read(dut, 0x14) == 100

    dut.diag_drop_cnt_in.value = 0xFFFF
    dut.diag_clip_cnt_in.value = 0xFFFF
    await ClockCycles(dut.s_axi_aclk, 2)
    assert await axi_read(dut, 0x10) == 0xFFFF
    assert await axi_read(dut, 0x14) == 0xFFFF


@cocotb.test()
async def test_phase_inc_readback(dut):
    """PHASE_INC register reflects input port (sign-extended)."""
    cocotb.start_soon(Clock(dut.s_axi_aclk, 10, units="ns").start())
    await reset_dut(dut)

    dut.phase_inc_in.value = 0x1234
    await ClockCycles(dut.s_axi_aclk, 2)
    assert await axi_read(dut, 0x18) == 0x00001234

    dut.phase_inc_in.value = 0xF000
    await ClockCycles(dut.s_axi_aclk, 2)
    assert await axi_read(dut, 0x18) == 0xFFFFF000


@cocotb.test()
async def test_version_readback(dut):
    """VERSION register reads back the parameter value."""
    cocotb.start_soon(Clock(dut.s_axi_aclk, 10, units="ns").start())
    await reset_dut(dut)

    val = await axi_read(dut, 0x1C)
    assert val == 0x00010000


@cocotb.test()
async def test_back_to_back_writes(dut):
    """Multiple sequential writes to RW registers."""
    cocotb.start_soon(Clock(dut.s_axi_aclk, 10, units="ns").start())
    await reset_dut(dut)

    await axi_write(dut, 0x00, 128)
    await axi_write(dut, 0x0C, 7)

    assert await axi_read(dut, 0x00) == 128
    assert await axi_read(dut, 0x0C) == 7
    assert int(dut.stf_threshold_out.value) == 128
    assert int(dut.snap_mode_out.value) == 7


@cocotb.test()
async def test_reset_restores_defaults(dut):
    """Reset restores all RW registers to defaults."""
    cocotb.start_soon(Clock(dut.s_axi_aclk, 10, units="ns").start())
    await reset_dut(dut)

    await axi_write(dut, 0x00, 255)
    await axi_write(dut, 0x04, 1)
    await axi_write(dut, 0x0C, 3)
    assert await axi_read(dut, 0x00) == 255

    dut.s_axi_aresetn.value = 0
    await ClockCycles(dut.s_axi_aclk, 3)
    dut.s_axi_aresetn.value = 1
    await ClockCycles(dut.s_axi_aclk, 2)

    assert await axi_read(dut, 0x00) == 0
    assert await axi_read(dut, 0x04) == 0
    assert await axi_read(dut, 0x0C) == 0
