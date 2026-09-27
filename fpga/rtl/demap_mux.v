// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/demap_mux.v — Demapper input mux (SIGNAL bypass for pilot_track)
//
// SIGNAL symbol (is_signal==1): equalizer data → demapper directly (zero latency).
// DATA symbols (is_signal==0): pilot_track corrected output → demapper.
//
// This matches real receiver architectures where pilot tracking only
// applies to DATA symbols. SIGNAL uses raw channel estimates from LTF.
//
// Select is is_signal (decoder FSM state), NOT symbol_idx: the 8-bit DATA
// symbol counter wraps at the 256th DATA symbol (6M >= 763 B), which would
// re-assert the SIGNAL path mid-frame.

module demap_mux (
    input  wire        is_signal,

    // Path A: equalizer direct (for SIGNAL)
    input  wire        eq_data_valid,
    input  wire signed [15:0] eq_data_re,
    input  wire signed [15:0] eq_data_im,

    // Path B: pilot_track output (for DATA)
    input  wire        pt_data_valid,
    input  wire signed [15:0] pt_data_re,
    input  wire signed [15:0] pt_data_im,

    // Output to demapper
    output wire        valid_out,
    output wire signed [15:0] re_out,
    output wire signed [15:0] im_out
);

    assign valid_out = is_signal ? eq_data_valid : pt_data_valid;
    assign re_out    = is_signal ? eq_data_re    : pt_data_re;
    assign im_out    = is_signal ? eq_data_im    : pt_data_im;

endmodule
