// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/decode_chain.v — Integration wrapper for cocotb testing
//
// Wires: depuncturer → soft_pairer → vit_fifo → viterbi_k7
// Exposes the same interface as if feeding the depuncturer directly
// and reading decoded bits from the Viterbi output.

module decode_chain (
    input  wire        clk,
    input  wire        rst_n,

    // Frame control
    input  wire        frame_start,
    input  wire        flush_in,        // final flush (routes through vit_fifo)
    input  wire        streaming_mode,  // 0=SIGNAL, 1=DATA

    // Depuncturer input (simulates deinterleaver output — pairs)
    input  wire [1:0]  code_rate,
    input  wire        valid_in,
    input  wire [7:0]  soft_in0,
    input  wire [7:0]  soft_in1,

    // Decoded output (from Viterbi)
    output wire        valid_out,
    output wire        bit_out,

    // Debug
    output wire        vit_busy,
    output wire        fifo_overflow,
    // Backpressure: depuncturer pair FIFO full -- testbench must hold
    // valid_in while asserted (lever 2a-prime).
    output wire        upstream_full
);

    // --- Backpressure (lever 2a-prime) ---
    wire        depunct_full;
    wire        vitf_full;

    // --- Depuncturer ---
    wire        depunct_valid;
    wire [7:0]  depunct_soft;

    depuncturer u_depunct (
        .clk(clk),
        .rst_n(rst_n),
        .code_rate(code_rate),
        .symbol_start(1'b0),     // tied low (same as system_bd.tcl)
        .stall_in(vitf_full),
        .valid_in(valid_in),
        .soft_in0(soft_in0),
        .soft_in1(soft_in1),
        .valid_out(depunct_valid),
        .soft_out(depunct_soft),
        .fifo_full(depunct_full)
    );

    // --- Soft Pairer ---
    wire        pair_valid;
    wire [7:0]  pair_soft0;
    wire [7:0]  pair_soft1;

    soft_pairer u_pairer (
        .clk(clk),
        .rst_n(rst_n),
        .frame_start(frame_start),
        .stall_in(vitf_full),
        .valid_in(depunct_valid),
        .soft_in(depunct_soft),
        .valid_out(pair_valid),
        .soft0(pair_soft0),
        .soft1(pair_soft1)
    );

    // --- Vit FIFO ---
    wire        fifo_rd_valid;
    wire [7:0]  fifo_rd_soft0;
    wire [7:0]  fifo_rd_soft1;
    wire        fifo_flush_out;

    vit_fifo u_fifo (
        .clk(clk),
        .rst_n(rst_n),
        .frame_start(frame_start),
        .wr_valid(pair_valid),
        .wr_soft0(pair_soft0),
        .wr_soft1(pair_soft1),
        .rd_valid(fifo_rd_valid),
        .rd_soft0(fifo_rd_soft0),
        .rd_soft1(fifo_rd_soft1),
        .vit_busy(vit_busy),
        .flush_in(flush_in),
        .flush_out(fifo_flush_out),
        .overflow(fifo_overflow),
        .full(vitf_full)
    );

    // --- Viterbi K=7 ---
    viterbi_k7 u_viterbi (
        .clk(clk),
        .rst_n(rst_n),
        .frame_start(frame_start),
        .flush(fifo_flush_out),
        .streaming_mode(streaming_mode),
        .valid_in(fifo_rd_valid),
        .soft0(fifo_rd_soft0),
        .soft1(fifo_rd_soft1),
        .valid_out(valid_out),
        .bit_out(bit_out),
        .busy(vit_busy)
    );

    // fifo_overflow is already connected via .overflow(fifo_overflow) above.
    // A redundant `assign fifo_overflow = u_fifo.overflow;` was removed here
    // (double-drive, masked by -Wno-MULTIDRIVEN — cleanup sweep G3).
    assign upstream_full = depunct_full;

endmodule
