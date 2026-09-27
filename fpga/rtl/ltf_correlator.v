// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/ltf_correlator.v — Sliding LTF cross-correlator
//
// 4 parallel complex MAC engines, 16 taps, one metric per ADC sample.
// Processes 4 taps per clock across 4 stages (4 clocks) + 1 clock for
// metric output = 5 clocks per sample (matches 1-per-5 ADC/fabric ratio).
//
// Reduced from 6 engines / 24 taps: empirically identical peak-finding
// on all OTA and cable captures (see docs/correlator-reduction-analysis.md).
//
// CE merging: sig_re/sig_im and accumulator use always-write with data MUX
// instead of stage-gated CE, reducing control set count by ~8.
//
// DSP usage: 16 DSP48E1 (all 4 engines × 4 products each).
// Critical path broken by registering stage_sum before accumulator.

module ltf_correlator (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        iq_valid,          // 1-per-5 sample strobe
    input  wire signed [11:0] iq_re,
    input  wire signed [11:0] iq_im,
    output reg         metric_valid,      // pulses once per sample (after pipeline fill)
    output reg  [28:0] metric             // (Re>>11)^2 + (Im>>11)^2, rotation-invariant
);

    // =========================================================
    // 16-entry shift register (stores last 16 IQ samples)
    // =========================================================
    reg signed [11:0] sr_re [0:15];
    reg signed [11:0] sr_im [0:15];

    integer i;
    always @(posedge clk) begin
        if (iq_valid) begin
            for (i = 15; i > 0; i = i - 1) begin
                sr_re[i] <= sr_re[i-1];
                sr_im[i] <= sr_im[i-1];
            end
            sr_re[0] <= iq_re;
            sr_im[0] <= iq_im;
        end
    end

    // =========================================================
    // Stage counter: counts 0-4 between iq_valid pulses
    // =========================================================
    reg [2:0] stage;
    reg [4:0] prime_cnt;     // counts 16 samples for pipeline fill
    wire      primed = (prime_cnt >= 5'd16);

    always @(posedge clk) begin
        if (!rst_n) begin
            stage     <= 3'd4;  // idle until first iq_valid
            prime_cnt <= 0;
        end else if (iq_valid) begin
            stage <= 3'd0;
            if (!primed)
                prime_cnt <= prime_cnt + 1;
        end else if (stage < 3'd4) begin
            stage <= stage + 1;
        end
    end

    // =========================================================
    // Per-stage signal tap registers — always-write with data MUX
    //
    // 16 taps / 4 stages = 4 taps per stage.
    // Ordering: ref[k] correlates against the (15-k)-th newest sample.
    // sr[0] = newest, sr[15] = oldest.
    // Stage 0: ref[0..3]   pairs with sr[15..12] (oldest 4)
    // Stage 1: ref[4..7]   pairs with sr[11..8]
    // Stage 2: ref[8..11]  pairs with sr[7..4]
    // Stage 3: ref[12..15] pairs with sr[3..0] (newest 4)
    //
    // On iq_valid, pre-load stage 0 taps (post-shift sr[15..12] =
    // pre-shift sr[14..11]). Subsequent stages load from current sr.
    //
    // Always-write: register updates every cycle (single CE = clk),
    // data source selected by MUX. Eliminates per-stage CE signals.
    // =========================================================
    reg signed [11:0] sig_re [0:3];
    reg signed [11:0] sig_im [0:3];

    // MUX selects data source based on current state
    reg signed [11:0] sig_re_next [0:3];
    reg signed [11:0] sig_im_next [0:3];

    always @(*) begin
        if (iq_valid) begin
            // Stage 0 taps: post-shift sr[15..12] = pre-shift sr[14..11]
            sig_re_next[0] = sr_re[14]; sig_re_next[1] = sr_re[13];
            sig_re_next[2] = sr_re[12]; sig_re_next[3] = sr_re[11];
            sig_im_next[0] = sr_im[14]; sig_im_next[1] = sr_im[13];
            sig_im_next[2] = sr_im[12]; sig_im_next[3] = sr_im[11];
        end else begin
            case (stage)
                3'd0: begin
                    // Stage 1 taps: sr[11..8]
                    sig_re_next[0] = sr_re[11]; sig_re_next[1] = sr_re[10];
                    sig_re_next[2] = sr_re[9];  sig_re_next[3] = sr_re[8];
                    sig_im_next[0] = sr_im[11]; sig_im_next[1] = sr_im[10];
                    sig_im_next[2] = sr_im[9];  sig_im_next[3] = sr_im[8];
                end
                3'd1: begin
                    // Stage 2 taps: sr[7..4]
                    sig_re_next[0] = sr_re[7];  sig_re_next[1] = sr_re[6];
                    sig_re_next[2] = sr_re[5];  sig_re_next[3] = sr_re[4];
                    sig_im_next[0] = sr_im[7];  sig_im_next[1] = sr_im[6];
                    sig_im_next[2] = sr_im[5];  sig_im_next[3] = sr_im[4];
                end
                3'd2: begin
                    // Stage 3 taps: sr[3..0]
                    sig_re_next[0] = sr_re[3];  sig_re_next[1] = sr_re[2];
                    sig_re_next[2] = sr_re[1];  sig_re_next[3] = sr_re[0];
                    sig_im_next[0] = sr_im[3];  sig_im_next[1] = sr_im[2];
                    sig_im_next[2] = sr_im[1];  sig_im_next[3] = sr_im[0];
                end
                default: begin
                    // Stage 3,4: hold current values
                    sig_re_next[0] = sig_re[0]; sig_re_next[1] = sig_re[1];
                    sig_re_next[2] = sig_re[2]; sig_re_next[3] = sig_re[3];
                    sig_im_next[0] = sig_im[0]; sig_im_next[1] = sig_im[1];
                    sig_im_next[2] = sig_im[2]; sig_im_next[3] = sig_im[3];
                end
            endcase
        end
    end

    // Single always block — uniform CE (always active under rst_n)
    always @(posedge clk) begin
        if (!rst_n) begin
            sig_re[0] <= 0; sig_re[1] <= 0; sig_re[2] <= 0; sig_re[3] <= 0;
            sig_im[0] <= 0; sig_im[1] <= 0; sig_im[2] <= 0; sig_im[3] <= 0;
        end else begin
            sig_re[0] <= sig_re_next[0]; sig_re[1] <= sig_re_next[1];
            sig_re[2] <= sig_re_next[2]; sig_re[3] <= sig_re_next[3];
            sig_im[0] <= sig_im_next[0]; sig_im[1] <= sig_im_next[1];
            sig_im[2] <= sig_im_next[2]; sig_im[3] <= sig_im_next[3];
        end
    end

    // =========================================================
    // ROM reference values (4 lookups, addressed by stage)
    // Uses first 16 entries of ltf_rom32 (taps 0-15).
    // =========================================================
    wire [4:0] rom_base;
    assign rom_base = {1'b0, stage[1:0], 2'b00};  // stage * 4

    wire signed [7:0] ref_re [0:3];
    wire signed [7:0] ref_im [0:3];

    genvar g;
    generate
        for (g = 0; g < 4; g = g + 1) begin : rom_inst
            ltf_rom32 u_rom (
                .addr(rom_base + g[4:0]),
                .rom_re(ref_re[g]),
                .rom_im(ref_im[g])
            );
        end
    endgenerate

    // =========================================================
    // 4 complex multipliers — all DSP with registered outputs
    // Complex conjugate correlation: conj(ref) * sig
    //   re_part = ref_re * sig_re + ref_im * sig_im
    //   im_part = ref_re * sig_im - ref_im * sig_re
    //
    // All 4 engines use DSP48E1 (16 DSPs, 4 engines × 4 products).
    // Products are registered (tap_re_r/tap_im_r) to break the critical
    // path: stage_reg → MUX → DSP is one cycle, adder → stage_sum_r is next.
    // =========================================================
    wire signed [19:0] prod_re [0:3];   // ref_re * sig_re
    wire signed [19:0] prod_ii [0:3];   // ref_im * sig_im
    wire signed [19:0] prod_ri [0:3];   // ref_re * sig_im
    wire signed [19:0] prod_ir [0:3];   // ref_im * sig_re

    generate
        for (g = 0; g < 4; g = g + 1) begin : eng_dsp
            assign prod_re[g] = ref_re[g] * sig_re[g];
            assign prod_ii[g] = ref_im[g] * sig_im[g];
            assign prod_ri[g] = ref_re[g] * sig_im[g];
            assign prod_ir[g] = ref_im[g] * sig_re[g];
        end
    endgenerate

    // Register all multiply outputs — absorbs DSP MREG/PREG, breaks critical path.
    reg signed [20:0] tap_re_r [0:3];
    reg signed [20:0] tap_im_r [0:3];

    always @(posedge clk) begin
        if (!rst_n) begin
            for (i = 0; i < 4; i = i + 1) begin
                tap_re_r[i] <= 0;
                tap_im_r[i] <= 0;
            end
        end else begin
            for (i = 0; i < 4; i = i + 1) begin
                tap_re_r[i] <= {prod_re[i][19], prod_re[i]} + {prod_ii[i][19], prod_ii[i]};
                tap_im_r[i] <= {prod_ri[i][19], prod_ri[i]} - {prod_ir[i][19], prod_ir[i]};
            end
        end
    end

    // Per-stage partial sum (4 engines -> 23-bit, from registered taps)
    wire signed [22:0] stage_sum_re;
    wire signed [22:0] stage_sum_im;

    assign stage_sum_re = {{2{tap_re_r[0][20]}}, tap_re_r[0]} + {{2{tap_re_r[1][20]}}, tap_re_r[1]}
                        + {{2{tap_re_r[2][20]}}, tap_re_r[2]} + {{2{tap_re_r[3][20]}}, tap_re_r[3]};

    assign stage_sum_im = {{2{tap_im_r[0][20]}}, tap_im_r[0]} + {{2{tap_im_r[1][20]}}, tap_im_r[1]}
                        + {{2{tap_im_r[2][20]}}, tap_im_r[2]} + {{2{tap_im_r[3][20]}}, tap_im_r[3]};

    // =========================================================
    // Running accumulator across 4 stages — always-write with MUX
    // Single CE (always active under rst_n) eliminates per-stage CEs.
    // tap_re_r is 1 cycle behind stage, so accumulator uses stage_d1:
    //   stage_d1=0 → load first partial sum
    //   stage_d1=1,2,3 → accumulate subsequent partial sums
    //   stage_d1=4 → hold (complete)
    // =========================================================
    reg signed [24:0] acc_re, acc_im;
    reg [2:0] stage_d1;  // delayed stage for accumulator control

    always @(posedge clk) begin
        if (!rst_n)
            stage_d1 <= 3'd4;
        else
            stage_d1 <= stage;
    end

    wire signed [24:0] acc_re_load = {{2{stage_sum_re[22]}}, stage_sum_re};
    wire signed [24:0] acc_im_load = {{2{stage_sum_im[22]}}, stage_sum_im};

    // MUX: stage_d1 0 = fresh load, 1-3 = accumulate, 4 = hold
    wire signed [24:0] acc_re_next = (stage_d1 == 3'd0) ? acc_re_load :
                                     (stage_d1 <  3'd4) ? (acc_re + acc_re_load) :
                                                          acc_re;
    wire signed [24:0] acc_im_next = (stage_d1 == 3'd0) ? acc_im_load :
                                     (stage_d1 <  3'd4) ? (acc_im + acc_im_load) :
                                                          acc_im;

    always @(posedge clk) begin
        if (!rst_n) begin
            acc_re <= 0;
            acc_im <= 0;
        end else begin
            acc_re <= acc_re_next;
            acc_im <= acc_im_next;
        end
    end

    // =========================================================
    // Metric output: squared magnitude Re^2 + Im^2 (rotation-invariant)
    // Output when stage_d1=4: accumulation of all 4 partial sums complete.
    //
    // |Re|+|Im| (L1) varies +/-41% with the cfo_mixer NCO phase, which let a
    // multipath sidelobe (T1+10, ~15% weaker in magnitude) beat the true T1
    // peak and shift the FFT window +10 samples (layer-8 multipath_mod).
    // Re^2+Im^2 removes the NCO-phase dependence entirely.
    //
    // Fixed-point: metric = (|acc_re|>>11)^2 + (|acc_im|>>11)^2. The >>11
    // truncation keeps the squarers at 14x14 (~400 LUTs). use_dsp="no"
    // forces LUT multipliers — the 2 spare DSP48s stay free (design is
    // 78/80 without them; Vivado would otherwise consume both).
    // Monotone in the true Re^2+Im^2.
    //
    // 3-stage pipeline (abs -> square -> sum): the unregistered acc->metric
    // path through LUT multipliers violated timing (-3.6ns / -1.58ns).
    // metric_valid shifts with the pipeline: the pulse lands on the next
    // sample's iq_valid clock (stage_d1==4 + 3 registered stages). The
    // captured wr_ptr is unchanged (it updates only on sample boundaries,
    // registered), so acquisition_ctrl's T1_OFFSET stays valid.
    // =========================================================
    wire [24:0] abs_re = acc_re[24] ? (~acc_re + 1) : acc_re;
    wire [24:0] abs_im = acc_im[24] ? (~acc_im + 1) : acc_im;
    wire [13:0] mag_re = abs_re[24:11];   // |acc_re| >> 11, max 8191
    wire [13:0] mag_im = abs_im[24:11];

    reg [13:0] mag_re_r, mag_im_r;
    reg        val0;
    reg [27:0] sq_re_r, sq_im_r;
    reg        val1;
    reg [28:0] metric_r;
    reg        val2;

    (* use_dsp = "no" *) wire [27:0] sq_re = mag_re_r * mag_re_r;
    (* use_dsp = "no" *) wire [27:0] sq_im = mag_im_r * mag_im_r;

    always @(posedge clk) begin
        if (!rst_n) begin
            mag_re_r <= 14'd0; mag_im_r <= 14'd0; val0 <= 1'b0;
            sq_re_r  <= 28'd0; sq_im_r  <= 28'd0; val1 <= 1'b0;
            metric_r <= 29'd0; val2 <= 1'b0;
            metric_valid <= 1'b0;
            metric       <= 29'd0;
        end else begin
            mag_re_r <= mag_re;
            mag_im_r <= mag_im;
            val0     <= (stage_d1 == 3'd4) && primed;
            sq_re_r  <= sq_re;
            sq_im_r  <= sq_im;
            val1     <= val0;
            metric_r <= {1'b0, sq_re_r} + {1'b0, sq_im_r};  // max 2*8191^2 < 2^29
            val2     <= val1;
            metric_valid <= val2;
            if (val2)
                metric <= metric_r;
        end
    end

endmodule
