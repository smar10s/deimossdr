// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/cordic_atan2.v — CORDIC atan2 (vectoring mode)
//
// Computes atan2(y, x) using 16 iterations of CORDIC vectoring.
// Output: 16-bit signed angle representing radians as a fraction of 2π.
//   angle = atan2(y,x) / (2π) × 2^16
//   Range: [-32768, +32767] maps to [-π, +π)
//
// Pipeline: 16 stages + 2 (pre-rotation + output register) = 18 cycles latency
// Throughput: 1 result per clock after pipeline fills
//
// Interface:
//   - Start: assert valid_in with x_in, y_in
//   - Result: valid_out asserts 18 cycles later with angle_out
//
// Resources: ~200-400 LUTs, 0 DSP48E1, 0 BRAM

module cordic_atan2 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid_in,
    input  wire signed [31:0] x_in,   // autocorrelation real part
    input  wire signed [31:0] y_in,   // autocorrelation imag part
    output wire        valid_out,
    output wire signed [15:0] angle_out  // atan2(y,x) / (2π) × 2^16
);

    // CORDIC atan table: atan(2^-i) in units of (2π) × 2^16
    // atan(2^-i) / (2π) × 65536
    // atan(1)       = π/4     → 65536/8 = 8192
    // atan(0.5)     = 0.4636  → 4836
    // atan(0.25)    = 0.2450  → 2555
    // etc.
    //
    // Computed as: round(atan(2^-i) / (2*pi) * 65536)

    wire signed [15:0] atan_table [0:15];
    assign atan_table[0]  = 16'sd8192;   // atan(2^0)  = 45.000°
    assign atan_table[1]  = 16'sd4836;   // atan(2^-1) = 26.565°
    assign atan_table[2]  = 16'sd2555;   // atan(2^-2) = 14.036°
    assign atan_table[3]  = 16'sd1297;   // atan(2^-3) = 7.125°
    assign atan_table[4]  = 16'sd651;    // atan(2^-4) = 3.576°
    assign atan_table[5]  = 16'sd326;    // atan(2^-5) = 1.790°
    assign atan_table[6]  = 16'sd163;    // atan(2^-6) = 0.895°
    assign atan_table[7]  = 16'sd81;     // atan(2^-7) = 0.448°
    assign atan_table[8]  = 16'sd41;     // atan(2^-8) = 0.224°
    assign atan_table[9]  = 16'sd20;     // atan(2^-9) = 0.112°
    assign atan_table[10] = 16'sd10;     // atan(2^-10)
    assign atan_table[11] = 16'sd5;      // atan(2^-11)
    assign atan_table[12] = 16'sd3;      // atan(2^-12)
    assign atan_table[13] = 16'sd1;      // atan(2^-13)
    assign atan_table[14] = 16'sd1;      // atan(2^-14)
    assign atan_table[15] = 16'sd0;      // atan(2^-15)

    // Pipeline registers (fully pipelined: one stage per clock)
    // Width: 34 bits for x/y (32-bit input + 2 guard bits for growth)
    localparam W = 34;

    reg signed [W-1:0] x_pipe [0:16];
    reg signed [W-1:0] y_pipe [0:16];
    reg signed [15:0]  z_pipe [0:16];
    reg                v_pipe [0:16];

    // Pre-rotation (stage 0): rotate into right half-plane (|angle| <= π/2)
    // If x < 0: negate both x and y, add π to angle accumulator.
    // Since 16'sh8000 = -32768 represents both +π and -π (they're the same
    // angle mod 2π), we use it uniformly. The CORDIC iterations then refine
    // from this starting point using signed wrapping arithmetic.
    always @(posedge clk) begin
        if (!rst_n) begin
            x_pipe[0] <= 0;
            y_pipe[0] <= 0;
            z_pipe[0] <= 0;
            v_pipe[0] <= 0;
        end else begin
            v_pipe[0] <= valid_in;
            if (x_in[31] == 1'b0) begin
                // x >= 0: already in right half-plane
                x_pipe[0] <= {{(W-32){x_in[31]}}, x_in};
                y_pipe[0] <= {{(W-32){y_in[31]}}, y_in};
                z_pipe[0] <= 16'sd0;
            end else begin
                // x < 0: negate both, start z at π
                // After negation, the point is in the right half-plane.
                // The CORDIC vectoring will add corrections to z_pipe[0].
                // For y>=0 (Q2): true angle is in (π/2, π] → z starts at +π, CORDIC subtracts
                // For y<0 (Q3): true angle is in [-π, -π/2) → z starts at -π, CORDIC adds
                // Both use 16'sh8000 because +π = -π mod 2π.
                x_pipe[0] <= -{{(W-32){x_in[31]}}, x_in};
                y_pipe[0] <= -{{(W-32){y_in[31]}}, y_in};
                z_pipe[0] <= 16'sh8000;
            end
        end
    end

    // CORDIC iterations (stages 1-16)
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : cordic_stage
            always @(posedge clk) begin
                if (!rst_n) begin
                    x_pipe[i+1] <= 0;
                    y_pipe[i+1] <= 0;
                    z_pipe[i+1] <= 0;
                    v_pipe[i+1] <= 0;
                end else begin
                    v_pipe[i+1] <= v_pipe[i];
                    // Vectoring mode: drive y toward 0
                    if (y_pipe[i] >= 0) begin
                        // y positive: rotate clockwise (subtract angle)
                        x_pipe[i+1] <= x_pipe[i] + (y_pipe[i] >>> i);
                        y_pipe[i+1] <= y_pipe[i] - (x_pipe[i] >>> i);
                        z_pipe[i+1] <= z_pipe[i] + atan_table[i];
                    end else begin
                        // y negative: rotate counter-clockwise (add angle)
                        x_pipe[i+1] <= x_pipe[i] - (y_pipe[i] >>> i);
                        y_pipe[i+1] <= y_pipe[i] + (x_pipe[i] >>> i);
                        z_pipe[i+1] <= z_pipe[i] - atan_table[i];
                    end
                end
            end
        end
    endgenerate

    // Output
    assign valid_out = v_pipe[16];
    assign angle_out = z_pipe[16];

endmodule
