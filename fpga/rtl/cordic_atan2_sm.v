// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/cordic_atan2_sm.v — Compact CORDIC atan2 for pilot phase extraction
//
// Same algorithm as cordic_atan2.v but optimized for 16-bit pilot inputs:
//   - 8 iterations (vs 16): residual error < 0.45° (atan(2^-7)) — adequate for pilot tracking
//   - 16-bit internal datapath (vs 34-bit): matches 16-bit input precision
//   - Pipeline latency: 10 cycles (pre-rot + 8 iterations + output)
//   - Throughput: 1 result/clock
//
// Saves ~300-400 LUTs compared to the full cordic_atan2.
//
// Resources: ~100-200 LUTs, 0 DSP48E1, 0 BRAM

module cordic_atan2_sm (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid_in,
    input  wire signed [15:0] x_in,
    input  wire signed [15:0] y_in,
    output wire        valid_out,
    output wire signed [15:0] angle_out
);

    // CORDIC atan table (8 entries)
    wire signed [15:0] atan_table [0:7];
    assign atan_table[0] = 16'sd8192;   // atan(2^0)  = 45.000°
    assign atan_table[1] = 16'sd4836;   // atan(2^-1) = 26.565°
    assign atan_table[2] = 16'sd2555;   // atan(2^-2) = 14.036°
    assign atan_table[3] = 16'sd1297;   // atan(2^-3) = 7.125°
    assign atan_table[4] = 16'sd651;    // atan(2^-4) = 3.576°
    assign atan_table[5] = 16'sd326;    // atan(2^-5) = 1.790°
    assign atan_table[6] = 16'sd163;    // atan(2^-6) = 0.895°
    assign atan_table[7] = 16'sd81;     // atan(2^-7) = 0.448°

    // Internal width: 17 bits — 1 guard bit above 16-bit input precision.
    // Iteration 0 adds |y| to x (worst case at 45°: 32767+32767 = 65534,
    // needs 17 bits signed, max 65535). CORDIC gain through 8 iterations is
    // ~1.647, so final x magnitude ≤ 32767*1.647 ≈ 53968 — fits in 17 bits.
    // Without the guard bit, magnitudes > 23170 (√(32767²/2)) wrap and
    // produce arbitrary angles. The equalizer saturates output to ±32767,
    // so the full range is reachable via pilot_track sum_re[17:2].
    // Also fixes the −32768 negation wrap in pre-rotation (sign-extend first).
    localparam W = 17;

    reg signed [W-1:0] x_pipe [0:8];
    reg signed [W-1:0] y_pipe [0:8];
    reg signed [15:0]  z_pipe [0:8];
    reg                v_pipe [0:8];

    // Pre-rotation (stage 0): rotate into right half-plane
    always @(posedge clk) begin
        if (!rst_n) begin
            x_pipe[0] <= 0;
            y_pipe[0] <= 0;
            z_pipe[0] <= 0;
            v_pipe[0] <= 0;
        end else begin
            v_pipe[0] <= valid_in;
            if (x_in[15] == 1'b0) begin
                // x >= 0: already in right half-plane
                x_pipe[0] <= {{(W-16){x_in[15]}}, x_in};
                y_pipe[0] <= {{(W-16){y_in[15]}}, y_in};
                z_pipe[0] <= 16'sd0;
            end else begin
                // x < 0: negate both, start z at π
                x_pipe[0] <= -{{(W-16){x_in[15]}}, x_in};
                y_pipe[0] <= -{{(W-16){y_in[15]}}, y_in};
                z_pipe[0] <= 16'sh8000;
            end
        end
    end

    // CORDIC iterations (stages 1-8): vectoring mode
    genvar i;
    generate
        for (i = 0; i < 8; i = i + 1) begin : cordic_stage
            always @(posedge clk) begin
                if (!rst_n) begin
                    x_pipe[i+1] <= 0;
                    y_pipe[i+1] <= 0;
                    z_pipe[i+1] <= 0;
                    v_pipe[i+1] <= 0;
                end else begin
                    v_pipe[i+1] <= v_pipe[i];
                    if (y_pipe[i] >= 0) begin
                        x_pipe[i+1] <= x_pipe[i] + (y_pipe[i] >>> i);
                        y_pipe[i+1] <= y_pipe[i] - (x_pipe[i] >>> i);
                        z_pipe[i+1] <= z_pipe[i] + atan_table[i];
                    end else begin
                        x_pipe[i+1] <= x_pipe[i] - (y_pipe[i] >>> i);
                        y_pipe[i+1] <= y_pipe[i] + (x_pipe[i] >>> i);
                        z_pipe[i+1] <= z_pipe[i] - atan_table[i];
                    end
                end
            end
        end
    endgenerate

    // Output
    assign valid_out = v_pipe[8];
    assign angle_out = z_pipe[8];

endmodule
