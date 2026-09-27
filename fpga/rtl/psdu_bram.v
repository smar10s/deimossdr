// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/psdu_bram.v — 16 KB simple dual-port BRAM for PSDU storage
//
// Synthesis: 8 RAMB18E1 primitives in 16384x1 simple dual-port configuration.
// Simulation: behavioral reg array (Verilator compatible via `ifdef SIM).

module psdu_bram (
    input  wire        clk,
    input  wire        rst,
    input  wire [13:0] waddr,
    input  wire [7:0]  wdata,
    input  wire        we,
    input  wire [13:0] raddr,
    output wire [7:0]  rdata
);

    // 16 KB = 8 RAMB18E1s, one bit plane each
    localparam PSDU_MEM_DEPTH = 16384;
    localparam PSDU_BRAM_COUNT = 8;

`ifdef SIM
    reg [7:0] mem [0:PSDU_MEM_DEPTH-1];

    // NOTE: No reset zeroing — matches hardware RAMB18E1 behavior, which
    // only resets the output register (RSTRAMARSTRAM), not memory contents.
    // Stale bytes survive reset in hardware; sim must behave the same to
    // prevent tests from accidentally relying on post-reset zeros.
    always @(posedge clk) begin
        if (we) begin
            mem[waddr] <= wdata;
        end
    end

    assign rdata = mem[raddr];
`else
    wire [7:0] bram_dout;

    genvar g;
    generate
        for (g = 0; g < PSDU_BRAM_COUNT; g = g + 1) begin : gen_bram
            wire [15:0] doado;

            RAMB18E1 #(
                .RDADDR_COLLISION_HWCONFIG("DELAYED_WRITE"),
                .READ_WIDTH_A(1),
                .WRITE_WIDTH_B(1),
                .WRITE_MODE_A("READ_FIRST"),
                .RSTREG_PRIORITY_A("REGCE"),
                .SIM_COLLISION_CHECK("ALL"),
                .INIT_A(18'h00000),
                .SRVAL_A(18'h00000)
            ) bram_inst (
                .DOADO(doado),
                .DOBDO(),
                .DOPADOP(),
                .DOPBDOP(),
                .ADDRARDADDR(raddr),
                .ADDRBWRADDR(waddr),
                .CLKARDCLK(clk),
                .CLKBWRCLK(clk),
                .DIADI(16'h0000),
                .DIBDI({15'b0, wdata[g]}),
                .DIPADIP(2'b00),
                .DIPBDIP(2'b00),
                .ENARDEN(1'b1),
                .ENBWREN(we),
                .REGCEAREGCE(1'b0),
                .REGCEB(1'b0),
                .RSTRAMARSTRAM(rst),
                .RSTRAMB(rst),
                .RSTREGARSTREG(1'b0),
                .RSTREGB(1'b0),
                .WEA(2'b00),
                .WEBWE({3'b000, we})
            );

            assign bram_dout[g] = doado[0];
        end
    endgenerate

    assign rdata = bram_dout;
`endif

endmodule
