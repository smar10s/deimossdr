// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/frame_fifo.v — Frame descriptor FIFO (acquisition → decode handoff)
//
// 4-deep registered FIFO holding {ltf_pos[15:0], phase_inc[15:0]} descriptors.
// Pure registers (no BRAM). Synchronous push/pop with full/empty flags.
//
// Resource: ~60 LUTs (4 × 31-bit entries + pointer logic)

module frame_fifo #(
    parameter DEPTH = 4
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        flush,        // clear all entries

    // Push side (from acquisition_ctrl)
    input  wire        push,
    input  wire [15:0] push_ltf_pos,
    input  wire [15:0] push_phase_inc,

    // Pop side (to decode_engine)
    input  wire        pop,
    output wire [15:0] pop_ltf_pos,
    output wire [15:0] pop_phase_inc,

    // Status
    output wire        empty,
    output wire        full,
    output wire [2:0]  count
);

    // =========================================================
    // Storage
    // =========================================================
    localparam PTR_W = (DEPTH <= 4) ? 2 : 3;  // pointer width

    reg [15:0] mem_ltf_pos  [0:DEPTH-1];
    reg [15:0] mem_phase_inc [0:DEPTH-1];
    reg [PTR_W:0] wr_ptr;  // extra bit for full/empty disambiguation
    reg [PTR_W:0] rd_ptr;

    // =========================================================
    // Status
    // =========================================================
    wire [PTR_W:0] used = wr_ptr - rd_ptr;
    assign empty = (wr_ptr == rd_ptr);
    assign full  = (used == DEPTH[PTR_W:0]);
    assign count = used[2:0];

    // =========================================================
    // Read port (combinational — valid when !empty)
    // =========================================================
    assign pop_ltf_pos   = mem_ltf_pos[rd_ptr[PTR_W-1:0]];
    assign pop_phase_inc = mem_phase_inc[rd_ptr[PTR_W-1:0]];

    // =========================================================
    // Push/Pop Logic
    // =========================================================
    integer i;
    always @(posedge clk) begin
        if (!rst_n || flush) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            for (i = 0; i < DEPTH; i = i + 1) begin
                mem_ltf_pos[i]   <= 0;
                mem_phase_inc[i] <= 0;
            end
        end else begin
            // Push (ignored if full)
            if (push && !full) begin
                mem_ltf_pos[wr_ptr[PTR_W-1:0]]   <= push_ltf_pos;
                mem_phase_inc[wr_ptr[PTR_W-1:0]] <= push_phase_inc;
                wr_ptr <= wr_ptr + 1;
            end

            // Pop (ignored if empty)
            if (pop && !empty) begin
                rd_ptr <= rd_ptr + 1;
            end
        end
    end

    // =========================================================
    // Simulation assertions (not synthesized)
    // =========================================================
    `ifdef SIM
    always @(posedge clk) begin
        if (push && full && !(!rst_n || flush))
            $warning("[frame_fifo] push while full (entry dropped)");
        if (pop && empty && !(!rst_n || flush))
            $warning("[frame_fifo] pop while empty (no effect)");
    end
    `endif

endmodule
