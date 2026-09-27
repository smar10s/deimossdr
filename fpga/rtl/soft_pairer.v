// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/soft_pairer.v — Pairs sequential soft bits into (soft0, soft1) for Viterbi
//
// The depuncturer emits one soft bit per clock (rate-1/2 coded stream:
// G0 output, G1 output, G0, G1, ...). The Viterbi expects simultaneous
// (soft0, soft1) pairs.
//
// This module accumulates two consecutive soft bits and presents them
// together as a single valid pair with one clock of latency.

module soft_pairer (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        frame_start,    // reset pairing state

    // Backpressure: hold the current output pair and stop consuming
    // input while asserted (lever 2a-prime).
    input  wire        stall_in,

    // Input: sequential soft bits from depuncturer
    input  wire        valid_in,
    input  wire [7:0]  soft_in,        // signed 8-bit LLR

    // Output: paired soft bits for Viterbi
    output reg         valid_out,
    output reg  [7:0]  soft0,          // first of pair (G0)
    output reg  [7:0]  soft1           // second of pair (G1)
);

    reg       have_first /* verilator public */;
    reg [7:0] first_saved;

    always @(posedge clk) begin
        if (!rst_n || frame_start) begin
            have_first <= 0;
            first_saved <= 0;
            valid_out <= 0;
            soft0 <= 0;
            soft1 <= 0;
        end else if (!stall_in) begin
            valid_out <= 0;

            if (valid_in) begin
                if (!have_first) begin
                    // Save first soft bit (negated: demapper convention
                    // is positive=likely-1, Viterbi expects positive=likely-0)
                    first_saved <= -soft_in;
                    have_first <= 1;
                end else begin
                    // Emit pair (second also negated)
                    soft0 <= first_saved;
                    soft1 <= -soft_in;
                    valid_out <= 1;
                    have_first <= 0;
                end
            end
        end
        // else: stalled -- hold valid_out/soft0/soft1/have_first unchanged
    end

endmodule
