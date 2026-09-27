// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/cfo_est.v — Coarse CFO Estimator
//
// After STF detected, computes coarse CFO from lag-16 autocorrelation angle.
//
// Algorithm (matches lib80211 sync.c:172-179):
//   1. On `start` pulse, begin accumulating complex autocorrelation at lag-16
//      over 96 IQ samples (the caller provides IQ starting from STF+48)
//   2. After 96 samples accumulated, feed (P_re, P_im) into CORDIC atan2
//   3. CORDIC output = atan2(P_im, P_re) in 16-bit angle units (fraction of 2π)
//   4. CFO = -angle / 16 (divide by lag)
//      In fixed-point: phase_inc = -cordic_out >>> 4 (arithmetic right-shift by 4)
//   5. Assert `done` with `phase_inc` output
//
// Pipeline:
//   Stage 1 (on iq_valid): multiply dl[15] × current sample (DSP48E1 M-reg)
//   Stage 2 (next cycle):  add products to accumulator (DSP48E1 P-reg or fabric)
//   This 2-stage MAC breaks the critical path that caused timing failure.
//
// Resources: ~200-400 LUTs (CORDIC), 2-4 DSP48E1 (pipelined MAC), 0 BRAM

module cfo_est (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,          // pulse: begin CFO estimation
    input  wire [11:0] iq_i,          // signed 12-bit I
    input  wire [11:0] iq_q,          // signed 12-bit Q
    input  wire        iq_valid,
    output reg         done,           // pulse: estimation complete
    output reg  [15:0] phase_inc       // signed: CFO as phase increment per sample
);

    localparam WINDOW = 32;  // Must stay within STF (160 samples from stream start).
                             // Detection fires at ~sample 98, fill takes 16 more,
                             // so accumulation spans samples ~114-146. STF ends at 160.
                             // 32 lag-16 products of the STF's strong periodic structure
                             // is adequate for coarse CFO (accuracy ~±1 kHz).

    // =========================================================
    // State Machine
    // =========================================================
    localparam S_IDLE   = 2'd0;
    localparam S_ACCUM  = 2'd1;
    localparam S_CORDIC = 2'd2;
    localparam S_DONE   = 2'd3;

    reg [1:0] state;

    // =========================================================
    // Delay Line — 16 entries for lag-16
    // =========================================================
    reg signed [11:0] dl_i [0:15];
    reg signed [11:0] dl_q [0:15];
    integer j;

    // =========================================================
    // Pipelined MAC
    // =========================================================
    // Stage 1: multiply (registered product outputs)
    // P_re contribution = dl_i[15]*iq_i + dl_q[15]*iq_q
    // P_im contribution = dl_q[15]*iq_i - dl_i[15]*iq_q
    //
    // Pipelined MAC (12×12 products → DSP48E1 to relieve LUT pressure)
    reg signed [23:0] prod_ii;   // dl_i[15] * iq_i
    reg signed [23:0] prod_qq;   // dl_q[15] * iq_q
    reg signed [23:0] prod_qi;   // dl_q[15] * iq_i
    reg signed [23:0] prod_iq;   // dl_i[15] * iq_q
    reg               prod_valid; // Stage 1 output valid

    // Stage 2: accumulate
    reg signed [31:0] acc_p_re;
    reg signed [31:0] acc_p_im;

    // Sample and pipeline counters
    reg [6:0] sample_cnt;  // counts input samples in accumulation (0..WINDOW-1)
    reg [6:0] accum_cnt;   // counts accumulated products (pipeline stage 2)
    reg [4:0] fill_cnt;    // 0-16 for delay line fill

    // =========================================================
    // CORDIC Interface
    // =========================================================
    reg         cordic_start;
    wire        cordic_done;
    wire signed [15:0] cordic_angle;

    cordic_atan2 u_cordic (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (cordic_start),
        .x_in      (acc_p_re),
        .y_in      (acc_p_im),
        .valid_out (cordic_done),
        .angle_out (cordic_angle)
    );

    // =========================================================
    // Stage 1: Multiply (pipelined)
    // =========================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            prod_ii    <= 0;
            prod_qq    <= 0;
            prod_qi    <= 0;
            prod_iq    <= 0;
            prod_valid <= 0;
        end else begin
            prod_valid <= 0;
            if (state == S_ACCUM && iq_valid && fill_cnt >= 5'd16) begin
                // Delay line is full — compute products
                prod_ii    <= dl_i[15] * $signed(iq_i);
                prod_qq    <= dl_q[15] * $signed(iq_q);
                prod_qi    <= dl_q[15] * $signed(iq_i);
                prod_iq    <= dl_i[15] * $signed(iq_q);
                prod_valid <= 1'b1;
            end
        end
    end

    // =========================================================
    // Stage 2: Accumulate
    // =========================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            acc_p_re  <= 0;
            acc_p_im  <= 0;
            accum_cnt <= 0;
        end else if (start) begin
            // Clear accumulators on any start (supports restart from any state)
            acc_p_re  <= 0;
            acc_p_im  <= 0;
            accum_cnt <= 0;
        end else if (prod_valid) begin
            // P_re += prod_ii + prod_qq
            // P_im += prod_qi - prod_iq
            acc_p_re <= acc_p_re + {{8{prod_ii[23]}}, prod_ii} + {{8{prod_qq[23]}}, prod_qq};
            acc_p_im <= acc_p_im + {{8{prod_qi[23]}}, prod_qi} - {{8{prod_iq[23]}}, prod_iq};
            accum_cnt <= accum_cnt + 1;
        end
    end

    // =========================================================
    // Main Control Logic
    // =========================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            sample_cnt   <= 0;
            fill_cnt     <= 0;
            done         <= 0;
            phase_inc    <= 0;
            cordic_start <= 0;
            for (j = 0; j < 16; j = j + 1) begin
                dl_i[j] <= 0;
                dl_q[j] <= 0;
            end
        end else begin
            done         <= 1'b0;
            cordic_start <= 1'b0;

            // Re-startable: accept start in ANY state. Aborts the current
            // estimation and begins fresh. The CORDIC pipeline may still
            // produce a stale valid_out from the old computation — harmless
            // because only S_DONE checks cordic_done, and we'll be in
            // S_ACCUM by then. Accumulators are cleared in Stage 2 block.
            if (start) begin
                state      <= S_ACCUM;
                sample_cnt <= 0;
                fill_cnt   <= 0;
                for (j = 0; j < 16; j = j + 1) begin
                    dl_i[j] <= 0;
                    dl_q[j] <= 0;
                end
            end else case (state)
                S_IDLE: begin
                    // Nothing — wait for start (handled above)
                end

                S_ACCUM: begin
                    if (iq_valid) begin
                        // Shift delay line
                        for (j = 15; j >= 1; j = j - 1) begin
                            dl_i[j] <= dl_i[j-1];
                            dl_q[j] <= dl_q[j-1];
                        end
                        dl_i[0] <= $signed(iq_i);
                        dl_q[0] <= $signed(iq_q);

                        if (fill_cnt < 5'd16) begin
                            fill_cnt <= fill_cnt + 1;
                        end else begin
                            // Count input samples during accumulation phase
                            if (sample_cnt == WINDOW - 1) begin
                                // Last sample submitted to pipeline.
                                // Wait one more cycle for the product to be
                                // accumulated (stage 2), then fire CORDIC.
                                state <= S_CORDIC;
                            end else begin
                                sample_cnt <= sample_cnt + 1;
                            end
                        end
                    end
                end

                S_CORDIC: begin
                    // Wait for the last product to be accumulated
                    // (prod_valid from the last sample arrives this cycle
                    //  and is accumulated in the acc logic above)
                    if (accum_cnt == WINDOW) begin
                        cordic_start <= 1'b1;
                        state        <= S_DONE;
                    end
                end

                S_DONE: begin
                    // Wait for CORDIC to produce result
                    if (cordic_done) begin
                        // CFO = -atan2(P_im, P_re) / 16
                        // phase_inc = -(cordic_angle >>> 4)
                        //
                        // Always apply: the pilot PLL handles any residual,
                        // and channel estimation absorbs the mixer offset.
                        // No dead-zone needed.
                        phase_inc <= -{{4{cordic_angle[15]}}, cordic_angle[15:4]};
                        done      <= 1'b1;
                        state     <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
