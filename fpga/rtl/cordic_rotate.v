// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/cordic_rotate.v — Pipelined CORDIC rotation mode
//
// Rotates vector (x_in, y_in) by 'angle' radians (16-bit, ±π = ±32768).
// Output: gain-compensated (x_out, y_out), 16-bit signed.
//
// Architecture:
//   - Quadrant pre-rotation for full ±π range (stage 0)
//   - 8 CORDIC iterations (stages 1-8)
//   - Gain compensation: ×39797 >>> 16 (= 1/K ≈ 0.6073) (stage 9)
//   - Output saturation to 16-bit signed (combinational)
//
// Latency: 10 cycles (pre-rot + 8 iterations + gain comp)
// Throughput: 1 sample/clock after pipeline fill
//
// Internal width: 18 bits (16-bit input + 2 guard bits)
// Residual angle error: < 0.45° (atan(2^-7), the last of 8 iterations)
//
// Resources: ~200-300 LUTs, 0-1 DSP48E1 (gain comp), 0 BRAM

module cordic_rotate (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid_in,
    input  wire signed [15:0] x_in,    // data Re
    input  wire signed [15:0] y_in,    // data Im
    input  wire signed [15:0] angle,   // rotation angle (16-bit, ±π = ±32768)
    output wire        valid_out,
    output wire signed [15:0] x_out,   // rotated Re
    output wire signed [15:0] y_out    // rotated Im
);

    // =========================================================
    // CORDIC atan table (8 entries)
    // =========================================================
    wire signed [15:0] atan_table [0:7];
    assign atan_table[0]  = 16'sd8192;   // atan(2^0)  = 45.000°
    assign atan_table[1]  = 16'sd4836;   // atan(2^-1) = 26.565°
    assign atan_table[2]  = 16'sd2555;   // atan(2^-2) = 14.036°
    assign atan_table[3]  = 16'sd1297;   // atan(2^-3) = 7.125°
    assign atan_table[4]  = 16'sd651;    // atan(2^-4) = 3.576°
    assign atan_table[5]  = 16'sd326;    // atan(2^-5) = 1.790°
    assign atan_table[6]  = 16'sd163;    // atan(2^-6) = 0.895°
    assign atan_table[7]  = 16'sd81;     // atan(2^-7) = 0.448°

    // Internal pipeline width: 18 bits (16-bit + 2 guard)
    localparam W = 18;

    // Pipeline registers
    reg signed [W-1:0] x_pipe [0:8];
    reg signed [W-1:0] y_pipe [0:8];
    reg signed [15:0]  z_pipe [0:8];
    reg                v_pipe [0:8];

    // =========================================================
    // Pre-rotation stage (stage 0): map angle into [-π/2, +π/2]
    // =========================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            x_pipe[0] <= 0;
            y_pipe[0] <= 0;
            z_pipe[0] <= 0;
            v_pipe[0] <= 0;
        end else begin
            v_pipe[0] <= valid_in;
            if (valid_in) begin
                if (angle >= 16'sd16384) begin
                    // Angle in [π/2, π): pre-rotate input by +π/2
                    // (x,y) -> (-y, x), subtract π/2 from angle
                    x_pipe[0] <= -{{(W-16){y_in[15]}}, y_in};
                    y_pipe[0] <=  {{(W-16){x_in[15]}}, x_in};
                    z_pipe[0] <= angle - 16'sd16384;
                end else if (angle < -16'sd16383) begin
                    // Angle in [-π, -π/2): pre-rotate input by -π/2
                    // (x,y) -> (y, -x), add π/2 to angle
                    x_pipe[0] <=  {{(W-16){y_in[15]}}, y_in};
                    y_pipe[0] <= -{{(W-16){x_in[15]}}, x_in};
                    z_pipe[0] <= angle + 16'sd16384;
                end else begin
                    // Angle in [-π/2, π/2): no pre-rotation
                    x_pipe[0] <= {{(W-16){x_in[15]}}, x_in};
                    y_pipe[0] <= {{(W-16){y_in[15]}}, y_in};
                    z_pipe[0] <= angle;
                end
            end else begin
                x_pipe[0] <= 0;
                y_pipe[0] <= 0;
                z_pipe[0] <= 0;
            end
        end
    end

    // =========================================================
    // CORDIC iterations (stages 1-8): rotation mode
    // =========================================================
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
                    if (z_pipe[i] >= 0) begin
                        x_pipe[i+1] <= x_pipe[i] - (y_pipe[i] >>> i);
                        y_pipe[i+1] <= y_pipe[i] + (x_pipe[i] >>> i);
                        z_pipe[i+1] <= z_pipe[i] - atan_table[i];
                    end else begin
                        x_pipe[i+1] <= x_pipe[i] + (y_pipe[i] >>> i);
                        y_pipe[i+1] <= y_pipe[i] - (x_pipe[i] >>> i);
                        z_pipe[i+1] <= z_pipe[i] + atan_table[i];
                    end
                end
            end
        end
    endgenerate

    // =========================================================
    // Gain compensation (stage 9): multiply by 1/K ≈ 0.6073
    // =========================================================
    // K(8) ≈ 1.6468
    localparam signed [16:0] GAIN_COMP = 17'sd39797;

    reg signed [W-1:0] x_comp;
    reg signed [W-1:0] y_comp;
    reg                v_comp;

    wire signed [W+16:0] x_product = x_pipe[8] * GAIN_COMP;
    wire signed [W+16:0] y_product = y_pipe[8] * GAIN_COMP;

    always @(posedge clk) begin
        if (!rst_n) begin
            x_comp <= 0;
            y_comp <= 0;
            v_comp <= 0;
        end else begin
            v_comp <= v_pipe[8];
            x_comp <= x_product[W+15:16];
            y_comp <= y_product[W+15:16];
        end
    end

    // =========================================================
    // Output saturation to 16-bit signed [-32768, +32767]
    // =========================================================
    wire x_overflow  = (~x_comp[W-1] && |x_comp[W-2:15]);
    wire x_underflow = ( x_comp[W-1] && ~(&x_comp[W-2:15]));
    wire y_overflow  = (~y_comp[W-1] && |y_comp[W-2:15]);
    wire y_underflow = ( y_comp[W-1] && ~(&y_comp[W-2:15]));

    assign x_out = x_overflow  ? 16'sh7FFF :
                   x_underflow ? 16'sh8000 :
                   x_comp[15:0];
    assign y_out = y_overflow  ? 16'sh7FFF :
                   y_underflow ? 16'sh8000 :
                   y_comp[15:0];
    assign valid_out = v_comp;

endmodule
