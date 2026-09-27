// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/demapper.v — 802.11a soft decision demapper
//
// Converts equalized constellation points to soft LLR bits for the Viterbi decoder.
//
// Rate modes:
//   0 = BPSK:   1 bit/symbol  → LLR(b0) = Re
//   1 = QPSK:   2 bits/symbol → LLR(b0) = Re, LLR(b1) = Im
//   2 = 16-QAM: 4 bits/symbol → LLR = [Re, 2*norm-|Re|, Im, 2*norm-|Im|]
//   3 = 64-QAM: 6 bits/symbol → LLR = [Re, 4*norm-|Re|, 2*norm-||Re|-4*norm|,
//                                        Im, 4*norm-|Im|, 2*norm-||Im|-4*norm|]
//
// Architecture (lever C, Task 4): pure 3-stage streaming LLR pipeline.
//   No symbol buffer, no capture/prime/emit FSM. Every valid_in subcarrier
//   flows input-register -> LLR arithmetic -> wide output, an exact 3-clock
//   latency:
//     Stage 1: register the sample; abs + norm-scaled terms
//     Stage 2: select and register all six candidate LLRs (all computed in
//              parallel regardless of rate)
//     Stage 3: drive soft_wide0..5
//
// Output: one subcarrier per clock on soft_wide0..5 (8-bit signed, saturated
// to ±127). Lane j is coded bit j of that subcarrier; lanes >= n_bpsc are
// defined don't-care. wide_valid is valid_in delayed exactly 3 clocks, so a
// contiguous 48-clock input produces a contiguous 48-word output.
//
// Resource estimate: 0 DSP, ~150 LUTs (pipeline arithmetic), 0 BRAM

module demapper (
    input  wire        clk,
    input  wire        rst_n,

    // Configuration
    input  wire [1:0]  rate_mode,      // 0=BPSK, 1=QPSK, 2=16QAM, 3=64QAM
    input  wire signed [15:0] norm,    // expected innermost constellation magnitude

    // Input: equalized symbol (one per clock when valid_in=1)
    input  wire        valid_in,
    input  wire signed [15:0] sym_re,
    input  wire signed [15:0] sym_im,

    // Output (lever C): one subcarrier's LLRs in coded-bit order when
    // wide_valid, one subcarrier per clock for 48 clocks. Lanes 0..n_bpsc-1
    // are meaningful; higher lanes are DEFINED don't-care (never X — an X lane
    // once propagated into the shared soft path and regressed test_adc_replay).
    // The former 2-lane serial output is gone; the deinterleaver consumes this
    // wide word directly.
    output reg         wide_valid,
    output reg  signed [7:0] soft_wide0,
    output reg  signed [7:0] soft_wide1,
    output reg  signed [7:0] soft_wide2,
    output reg  signed [7:0] soft_wide3,
    output reg  signed [7:0] soft_wide4,
    output reg  signed [7:0] soft_wide5
);

    // =========================================================
    // Stage 1 input (combinational)
    // =========================================================

    // Absolute values (combinational)
    wire [15:0] abs_re_comb = (sym_re < 0) ? -sym_re : sym_re;
    wire [15:0] abs_im_comb = (sym_im < 0) ? -sym_im : sym_im;

    // Precompute 64-QAM first subtraction (combinational, registered in stage 1)
    wire signed [16:0] diff_re_4n_comb = {1'b0, abs_re_comb} - {norm[13:0], 2'b0};
    wire signed [16:0] diff_im_4n_comb = {1'b0, abs_im_comb} - {norm[13:0], 2'b0};

    // ---------------------------------------------------------
    // Stage 1 registers: raw sample + derived terms
    // ---------------------------------------------------------
    reg signed [15:0] s1_re;       // raw Re
    reg signed [15:0] s1_im;       // raw Im
    reg [15:0]        s1_abs_re;   // |Re|
    reg [15:0]        s1_abs_im;   // |Im|
    reg signed [16:0] s1_norm_x2;  // 2 * norm
    reg signed [16:0] s1_norm_x4;  // 4 * norm
    reg signed [16:0] s1_diff_re_4n;  // |Re| - 4*norm (signed)
    reg signed [16:0] s1_diff_im_4n;  // |Im| - 4*norm (signed)
    reg [1:0]         s1_mode;     // rate_mode of the stage-1 sample
    reg               v1;          // stage-1 valid

    // =========================================================
    // LLR computation (combinational from stage 1 → stage 2 registers)
    // =========================================================
    // All LLR candidates computed combinationally, then REGISTERED.
    // This confines the arithmetic (subtractions, abs, scale, saturation)
    // to a single clock period ending at the s2_llr_* registers.

    // --- Intermediate combinational signals ---

    // 16-QAM: 2*norm - |x|
    wire signed [16:0] llr_16q_b1 = s1_norm_x2 - {1'b0, s1_abs_re};
    wire signed [16:0] llr_16q_b3 = s1_norm_x2 - {1'b0, s1_abs_im};

    // 64-QAM: ||x| - 4*norm|
    wire [15:0] abs_diff_re_4n = s1_diff_re_4n[16] ? -s1_diff_re_4n[15:0] : s1_diff_re_4n[15:0];
    wire [15:0] abs_diff_im_4n = s1_diff_im_4n[16] ? -s1_diff_im_4n[15:0] : s1_diff_im_4n[15:0];

    // 64-QAM LLRs (17-bit signed)
    wire signed [16:0] llr_64q_b1 = s1_norm_x4 - {1'b0, s1_abs_re};
    wire signed [16:0] llr_64q_b2 = s1_norm_x2 - {1'b0, abs_diff_re_4n};
    wire signed [16:0] llr_64q_b4 = s1_norm_x4 - {1'b0, s1_abs_im};
    wire signed [16:0] llr_64q_b5 = s1_norm_x2 - {1'b0, abs_diff_im_4n};

    // --- Saturation functions ---
    function signed [7:0] sat16;
        input signed [15:0] val;
        begin
            if (val > 16'sd127)
                sat16 = 8'sd127;
            else if (val < -16'sd127)
                sat16 = -8'sd127;
            else
                sat16 = val[7:0];
        end
    endfunction

    // 64-QAM LLR scaling: multiply by 4 (shift left 2) then saturate to [-127,+127]
    function signed [7:0] scale4_sat;
        input signed [16:0] val;
        reg signed [18:0] scaled;
        begin
            scaled = {val[16], val[16], val} << 2;  // ×4 with sign extension
            if (scaled > 19'sd127)
                scale4_sat = 8'sd127;
            else if (scaled < -19'sd127)
                scale4_sat = -8'sd127;
            else
                scale4_sat = scaled[7:0];
        end
    endfunction

    function signed [7:0] scale4_sat16;
        input signed [15:0] val;
        reg signed [17:0] scaled;
        begin
            scaled = {val[15], val[15], val} << 2;  // ×4 with sign extension
            if (scaled > 18'sd127)
                scale4_sat16 = 8'sd127;
            else if (scaled < -18'sd127)
                scale4_sat16 = -8'sd127;
            else
                scale4_sat16 = scaled[7:0];
        end
    endfunction

    // 16-QAM LLR scaling: multiply by 2 (shift left 1) then saturate to [-127,+127]
    // Matches 64-QAM design pattern. Without scaling, 16-QAM b1 (±20) quantizes
    // to ±1 in the Viterbi (÷8), providing essentially no soft information.
    // With 2× scaling: b1 = ±40 → Viterbi quant ±5 (meaningful soft margin).
    function signed [7:0] scale2_sat16;
        input signed [15:0] val;
        reg signed [16:0] scaled;
        begin
            scaled = {val[15], val} << 1;  // ×2 with sign extension
            if (scaled > 17'sd127)
                scale2_sat16 = 8'sd127;
            else if (scaled < -17'sd127)
                scale2_sat16 = -8'sd127;
            else
                scale2_sat16 = scaled[7:0];
        end
    endfunction

    function signed [7:0] scale2_sat17;
        input signed [16:0] val;
        reg signed [17:0] scaled;
        begin
            scaled = {val[16], val} << 1;  // ×2 with sign extension
            if (scaled > 18'sd127)
                scale2_sat17 = 8'sd127;
            else if (scaled < -18'sd127)
                scale2_sat17 = -8'sd127;
            else
                scale2_sat17 = scaled[7:0];
        end
    endfunction

    // --- Pre-saturated LLR candidates (combinational, all computed in parallel) ---
    // These are the values that will be REGISTERED into s2_llr_* at posedge clk.
    // 16-QAM scaling note: all 16-QAM LLRs scaled ×2 for Viterbi dynamic range.
    // 64-QAM scaling note: all 64-QAM LLRs scaled ×4 for Viterbi dynamic range.
    wire signed [7:0] llr_cand_re  = sat16(s1_re);          // BPSK/QPSK b0
    wire signed [7:0] llr_cand_im  = sat16(s1_im);          // QPSK b1
    wire signed [7:0] llr_cand_16re = scale2_sat16(s1_re);  // 16QAM b0
    wire signed [7:0] llr_cand_16im = scale2_sat16(s1_im);  // 16QAM b2
    wire signed [7:0] llr_cand_16b1 = scale2_sat17(llr_16q_b1);    // 16QAM b1
    wire signed [7:0] llr_cand_16b3 = scale2_sat17(llr_16q_b3);    // 16QAM b3
    wire signed [7:0] llr_cand_64re = scale4_sat16(s1_re);  // 64QAM b0
    wire signed [7:0] llr_cand_64im = scale4_sat16(s1_im);  // 64QAM b3
    wire signed [7:0] llr_cand_64b1 = scale4_sat(llr_64q_b1); // 64QAM b1
    wire signed [7:0] llr_cand_64b2 = scale4_sat(llr_64q_b2); // 64QAM b2
    wire signed [7:0] llr_cand_64b4 = scale4_sat(llr_64q_b4); // 64QAM b4
    wire signed [7:0] llr_cand_64b5 = scale4_sat(llr_64q_b5); // 64QAM b5

    // ---------------------------------------------------------
    // Stage 2 registers: selected candidates for the stage-1 sample
    // ---------------------------------------------------------
    // The mode select uses s1_mode (registered alongside s1_*), so it is the
    // mode of the sample that produced these candidates. Lanes >= n_bpsc hold
    // defined values (never X).
    reg signed [7:0] s2_llr_b0;    // bit 0: Re or scaled Re
    reg signed [7:0] s2_llr_b1;    // bit 1: Im, 16q_b1, or 64q_b1
    reg signed [7:0] s2_llr_b2;    // bit 2: 16q_im, or 64q_b2
    reg signed [7:0] s2_llr_b3;    // bit 3: 16q_b3, or 64q_im
    reg signed [7:0] s2_llr_b4;    // bit 4: 64q_b4
    reg signed [7:0] s2_llr_b5;    // bit 5: 64q_b5
    reg              v2;           // stage-2 valid

    // =========================================================
    // Pipeline
    // =========================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            v1         <= 0;
            v2         <= 0;
            wide_valid <= 0;
            s1_re      <= 0;
            s1_im      <= 0;
            s1_abs_re  <= 0;
            s1_abs_im  <= 0;
            s1_norm_x2 <= 0;
            s1_norm_x4 <= 0;
            s1_diff_re_4n <= 0;
            s1_diff_im_4n <= 0;
            s1_mode    <= 0;
            s2_llr_b0  <= 0;
            s2_llr_b1  <= 0;
            s2_llr_b2  <= 0;
            s2_llr_b3  <= 0;
            s2_llr_b4  <= 0;
            s2_llr_b5  <= 0;
            soft_wide0 <= 0;
            soft_wide1 <= 0;
            soft_wide2 <= 0;
            soft_wide3 <= 0;
            soft_wide4 <= 0;
            soft_wide5 <= 0;
        end else begin
            // Stage 1: register the incoming subcarrier
            s1_re      <= sym_re;
            s1_im      <= sym_im;
            s1_abs_re  <= abs_re_comb;
            s1_abs_im  <= abs_im_comb;
            s1_norm_x2 <= {norm[15], norm[14:0], 1'b0};
            s1_norm_x4 <= {norm[15], norm[13:0], 2'b0};
            s1_diff_re_4n <= diff_re_4n_comb;
            s1_diff_im_4n <= diff_im_4n_comb;
            s1_mode    <= rate_mode;
            v1         <= valid_in;

            // Stage 2: select the LLR candidates for the stage-1 subcarrier
            s2_llr_b0 <= (s1_mode == 2'd3) ? llr_cand_64re :
                         (s1_mode == 2'd2) ? llr_cand_16re : llr_cand_re;
            s2_llr_b1 <= (s1_mode == 2'd3) ? llr_cand_64b1 :
                         (s1_mode == 2'd2) ? llr_cand_16b1 : llr_cand_im;
            s2_llr_b2 <= (s1_mode == 2'd3) ? llr_cand_64b2 :
                         (s1_mode == 2'd2) ? llr_cand_16im : llr_cand_im;
            s2_llr_b3 <= (s1_mode == 2'd3) ? llr_cand_64im : llr_cand_16b3;
            s2_llr_b4 <= llr_cand_64b4;
            s2_llr_b5 <= llr_cand_64b5;
            v2        <= v1;

            // Stage 3: output the wide word (valid_in delayed 3 clocks)
            wide_valid <= v2;
            soft_wide0 <= s2_llr_b0;
            soft_wide1 <= s2_llr_b1;
            soft_wide2 <= s2_llr_b2;
            soft_wide3 <= s2_llr_b3;
            soft_wide4 <= s2_llr_b4;
            soft_wide5 <= s2_llr_b5;
        end
    end

endmodule
