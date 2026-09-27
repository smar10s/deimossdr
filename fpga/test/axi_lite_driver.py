"""Reusable cocotb AXI4-Lite master driver.

Performs single-beat AXI4-Lite read and write transactions against a DUT.
Drives AW/W/B channels for writes and AR/R channels for reads.

Usage:
    axi = AxiLiteMaster(dut, prefix="s_axi_", clk=dut.clk)
    await axi.write(0x00, 0x01)      # write 0x01 to register at offset 0
    val = await axi.read(0x04)       # read register at offset 4
"""

import cocotb
from cocotb.triggers import RisingEdge


class AxiLiteMaster:
    """AXI4-Lite master for cocotb testbenches.

    All signal names are derived from prefix + standard AXI suffix.
    E.g., prefix="s_axi_" gives s_axi_awaddr, s_axi_awvalid, etc.
    """

    def __init__(self, dut, prefix="s_axi_", clk=None):
        self.dut = dut
        self.clk = clk if clk is not None else dut.clk
        self._p = prefix

    def _sig(self, name):
        return getattr(self.dut, self._p + name)

    async def _init_signals(self):
        """Deassert all master-driven signals."""
        self._sig("awaddr").value = 0
        self._sig("awvalid").value = 0
        self._sig("wdata").value = 0
        self._sig("wstrb").value = 0
        self._sig("wvalid").value = 0
        self._sig("bready").value = 0
        self._sig("araddr").value = 0
        self._sig("arvalid").value = 0
        self._sig("rready").value = 0

    async def write(self, addr, data, strb=0xF):
        """Perform a single AXI4-Lite write transaction.

        Args:
            addr: Register byte address (low bits only).
            data: 32-bit write data.
            strb: Write byte strobes (default: all bytes).

        Returns:
            bresp: 2-bit write response (0 = OKAY).
        """
        # Drive AW and W channels simultaneously
        self._sig("awaddr").value = addr
        self._sig("awvalid").value = 1
        self._sig("wdata").value = data
        self._sig("wstrb").value = strb
        self._sig("wvalid").value = 1
        self._sig("bready").value = 1

        # Wait for AW handshake
        while True:
            await RisingEdge(self.clk)
            if self._sig("awready").value:
                break
        self._sig("awvalid").value = 0
        self._sig("awaddr").value = 0

        # Wait for W handshake (may already be done)
        if not self._sig("wready").value:
            while True:
                await RisingEdge(self.clk)
                if self._sig("wready").value:
                    break
        self._sig("wvalid").value = 0
        self._sig("wdata").value = 0
        self._sig("wstrb").value = 0

        # Wait for B response
        while True:
            await RisingEdge(self.clk)
            if self._sig("bvalid").value:
                break
        bresp = int(self._sig("bresp").value)
        self._sig("bready").value = 0

        # Let bvalid deassert
        await RisingEdge(self.clk)

        return bresp

    async def read(self, addr):
        """Perform a single AXI4-Lite read transaction.

        Args:
            addr: Register byte address (low bits only).

        Returns:
            (rdata, rresp): 32-bit read data and 2-bit response.
        """
        self._sig("araddr").value = addr
        self._sig("arvalid").value = 1
        self._sig("rready").value = 1

        # Wait for AR handshake
        while True:
            await RisingEdge(self.clk)
            if self._sig("arready").value:
                break
        self._sig("arvalid").value = 0
        self._sig("araddr").value = 0

        # Wait for R response
        while True:
            await RisingEdge(self.clk)
            if self._sig("rvalid").value:
                break
        rdata = int(self._sig("rdata").value)
        rresp = int(self._sig("rresp").value)
        self._sig("rready").value = 0

        # Let rvalid deassert
        await RisingEdge(self.clk)

        return rdata, rresp
