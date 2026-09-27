// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/stf_detect.v — STF (Short Training Field) detector
//
// Sliding lag-16, window-64 normalized autocorrelation on 12-bit IQ stream.
// Squared threshold: 25*|P|^2 >= 9*E1*E2 (integer form of |P|^2/E1E2 >= 0.36)
// Persistence check: metric above threshold for 16 consecutive samples
//
// DC Rejection:
//   A leaky-integrator DC estimator tracks and removes DC offset before
//   autocorrelation. This prevents false STF triggers from AD9361 DC offset
//   during power-on, frequency changes, or gain adjustments. The HPF 3dB
//   point is ~200 kHz (fs=20 MHz, alpha=15/16), below the 312.5 kHz
//   OFDM subcarrier spacing — no impact on real preamble detection.
//
// Architecture:
//   - DC-removal HPF (leaky integrator, ~4 LUTs + 2 regs per channel)
//   - BRAM circular buffer delay line (82 deep, 4 taps) via bram_delay_tap
//   - Sliding accumulation: add new pair/subtract old pair each iq_valid
//   - Four accumulators: P_re, P_im, E1, E2 (32-bit)
//   - Threshold: 25*|P|^2 >= 9*E1*E2 (integer equivalent of |P|^2/E1E2 >= 0.36)
//   - Persistence counter fires after 16 consecutive samples above threshold
//
// Requires 1-in-5 iq_valid spacing (20 MSPS ADC within 100 MHz fabric clock)
// so that BRAM read latency settles between valid samples.
//
// Resources: 2 BRAM18 + ~8-12 DSP48E1 + ~200-400 LUTs (vs 1200-1700 LUTs w/o BRAM)

module stf_detect (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,         // datapath enable (firmware-controlled)
                                       // 0 = held in reset (accumulators/counters zeroed)
                                       // 1 = normal operation
                                       // Firmware asserts after RF path is configured.
                                       // De-assert + re-assert = full accumulator reset.
    input  wire        clear,          // unconditional synchronous clear (watchdog only)
    input  wire        soft_clear,     // gated clear (playback_start): ignored when
                                       // stf_end_armed (mid-detection). Prevents HIL
                                       // re-arm from killing an in-progress frame.
    input  wire        iq_valid,
    input  wire [11:0] iq_i,        // signed 12-bit
    input  wire [11:0] iq_q,        // signed 12-bit
    input  wire [7:0]  threshold,   // detection sensitivity
                                    // Comparison: 25*p_sq >= 9*(e_prod >> shift)
                                    // shift = threshold[2:0] (0-7):
                                    //   0 = original 802.11 threshold 0.36 (most strict)
                                    //   1 = 2x more sensitive, 2 = 4x, 3 = 8x, etc.
                                    // Default register value: 0 (standard threshold)
    input  wire        pipeline_ack,   // pulse: pipeline accepted frame_detect
    output reg         frame_detect, // pulse: frame detected
    output reg  [24:0] frame_offset, // sample count at detection
    output reg         stf_end,      // pulse: STF plateau ended (GI2 boundary)
    output wire [31:0] diag_out,     // diagnostic: internal state for snap observation
    output wire [31:0] diag_thresh   // threshold comparator diagnostic for snap
);

    // =========================================================
    // DC-Removal High-Pass Filter (leaky integrator)
    // =========================================================
    // Tracks DC via exponential moving average, subtracts from input.
    // dc[n] = dc[n-1] + (x[n] - dc[n-1]) / 16
    // y[n] = x[n] - dc[n]
    //
    // 3dB cutoff ≈ fs*(1-alpha)/(2π) = 20e6/(16*2π) ≈ 199 kHz at 20 MSPS
    // Settling: ~80 samples (4 μs) to 99%
    //
    // Implementation: 16-bit signed accumulator (12.4 fixed-point).
    // Cost: ~12 LUTs, ~40 FFs
    localparam DC_FRAC = 4;
    localparam DC_W = 12 + DC_FRAC;  // 16 bits

    reg signed [DC_W-1:0] dc_i_acc;
    reg signed [DC_W-1:0] dc_q_acc;

    wire signed [11:0] in_i = $signed(iq_i);
    wire signed [11:0] in_q = $signed(iq_q);

    // Integer part of DC estimate (top 12 bits with rounding)
    wire signed [11:0] dc_i_int = dc_i_acc[DC_W-1 -: 12];
    wire signed [11:0] dc_q_int = dc_q_acc[DC_W-1 -: 12];

    // HPF output: input minus DC estimate, clamped to 12-bit
    wire signed [12:0] sub_i = {in_i[11], in_i} - {dc_i_int[11], dc_i_int};
    wire signed [12:0] sub_q = {in_q[11], in_q} - {dc_q_int[11], dc_q_int};
    wire signed [11:0] hpf_i = sub_i[12] == sub_i[11] ? sub_i[11:0] :
                                sub_i[12] ? -12'sd2048 : 12'sd2047;
    wire signed [11:0] hpf_q = sub_q[12] == sub_q[11] ? sub_q[11:0] :
                                sub_q[12] ? -12'sd2048 : 12'sd2047;

    // Update DC accumulator: dc += (in - dc_int) / 16
    // Division by 16 via fixed-point: accumulator has 4 fractional bits.
    //
    // err MUST be 13-bit: in and dc_int are each 12-bit signed, so their
    // difference spans [-4095, 4095] (13-bit). A 12-bit err wraps once
    // |dc_int| > 0 and |in| approaches full scale, which feeds the
    // accumulator the wrong sign and runs it away. On strong frames (OTA
    // M2, rms ~910) the DC estimate then saturates/wraps, corrupting the
    // HPF output, collapsing the STF autocorrelation early, and firing
    // stf_end ~64 samples early -> wrong LTF window -> lost frame.
    wire signed [12:0] err_i = {in_i[11], in_i} - {dc_i_int[11], dc_i_int};
    wire signed [12:0] err_q = {in_q[11], in_q} - {dc_q_int[11], dc_q_int};

    always @(posedge clk) begin
        if (!rst_n || !enable) begin
            dc_i_acc <= 0;
            dc_q_acc <= 0;
        end else if (iq_valid) begin
            // Add error/16 to accumulator. Since accumulator has 4 fractional
            // bits, adding the 12-bit error directly is equivalent to
            // adding error * 2^4 / 2^4 = error to the fractional part.
            // We want: dc_acc += err / 16 (in accumulator units = err)
            // So we just add the sign-extended error to the accumulator.
            dc_i_acc <= dc_i_acc + {{(DC_W-13){err_i[12]}}, err_i};
            dc_q_acc <= dc_q_acc + {{(DC_W-13){err_q[12]}}, err_q};
        end
    end

    // =========================================================
    // Delay Line — BRAM circular buffer (replaces LUT shift register)
    // =========================================================
    // 82-deep delay line with 4 taps, packed as {I[11:0], Q[11:0]}.
    // DEPTH=82 (not 81) eliminates BRAM read-during-write collision at TAP3.
    // With DEPTH=81: rd_addr3 = (wr_ptr + 2*81 - 1 - 80) mod 81 = wr_ptr.
    // With DEPTH=82: rd_addr3 = (wr_ptr + 2*82 - 1 - 80) mod 82 = wr_ptr+1.
    // The extra entry is just padding — tap offsets and window size unchanged.
    // Uses 2 BRAM18 + ~30 LUTs instead of ~800-1500 LUTs for shift regs.
    // Requires 1-in-5 iq_valid spacing (real hardware rate) so that BRAM
    // read latency (1 clock) settles before the next iq_valid arrives.
    wire [23:0] tap0_packed, tap1_packed, tap2_packed, tap3_packed;

    bram_delay_tap #(
        .DEPTH(82),
        .WIDTH(24),
        .TAP0(0),    // dl[0]  — newest sample (x2_new)
        .TAP1(16),   // dl[16] — x1_new (lag-16)
        .TAP2(64),   // dl[64] — x2_old (window-64 back)
        .TAP3(80)    // dl[80] — x1_old (oldest in window)
    ) u_delay_line (
        .clk      (clk),
        .rst_n    (rst_n),
        .clear    (clear || (soft_clear && !stf_end_armed)),
        .iq_valid (iq_valid),
        .din      ({hpf_i, hpf_q}),   // Feed DC-removed signal into delay line
        .tap0_out (tap0_packed),
        .tap1_out (tap1_packed),
        .tap2_out (tap2_packed),
        .tap3_out (tap3_packed)
    );

    // No iq_valid gating from BRAM clearing FSM. The FSM zeros entries in the
    // background (82 clocks, aborts on consecutive iq_valid in test mode).
    // Accumulators continue processing, and the sliding window self-corrects
    // within ~150 samples as zeroed BRAM entries replace stale values.
    // Gate with enable: when disabled, no samples are processed.
    wire iq_valid_gated = iq_valid & enable;

    // Unpack taps: {I[23:12], Q[11:0]}
    wire signed [11:0] x2_new_i = $signed(tap0_packed[23:12]);  // dl[0]  — x[n]
    wire signed [11:0] x2_new_q = $signed(tap0_packed[11:0]);
    wire signed [11:0] x1_new_i = $signed(tap1_packed[23:12]);  // dl[16] — x[n-16]
    wire signed [11:0] x1_new_q = $signed(tap1_packed[11:0]);
    wire signed [11:0] x2_old_i = $signed(tap2_packed[23:12]);  // dl[64] — x[n-64]
    wire signed [11:0] x2_old_q = $signed(tap2_packed[11:0]);
    wire signed [11:0] x1_old_i = $signed(tap3_packed[23:12]);  // dl[80] — x[n-80]
    wire signed [11:0] x1_old_q = $signed(tap3_packed[11:0]);

    // =========================================================
    // Tap input registers — break the BRAM tap → DSP input path.
    //
    // The tap outputs are registered BRAM reads (RAMB18 C2O ~2.5 ns)
    // feeding unregistered DSP48E1 A/B inputs. The DSP A→PCOUT
    // cascade arc is ~4 ns combinational, so the BRAM C2O → A → PCOUT
    // → PCIN path runs close to the 10 ns budget (0.439 ns slack on
    // 0x2b69dce3, VIOLATED after the fft16 retiming shifted global
    // placement). One uniform pipeline stage on all taps splits the
    // path; relative tap lags are unchanged and the +1 cycle latency
    // is absorbed by the running accumulator.
    // =========================================================
    reg signed [11:0] x2_new_i_r, x2_new_q_r;
    reg signed [11:0] x1_new_i_r, x1_new_q_r;
    reg signed [11:0] x2_old_i_r, x2_old_q_r;
    reg signed [11:0] x1_old_i_r, x1_old_q_r;

    always @(posedge clk) begin
        if (!rst_n || !enable) begin
            x2_new_i_r <= 0; x2_new_q_r <= 0;
            x1_new_i_r <= 0; x1_new_q_r <= 0;
            x2_old_i_r <= 0; x2_old_q_r <= 0;
            x1_old_i_r <= 0; x1_old_q_r <= 0;
        end else if (iq_valid_gated) begin
            x2_new_i_r <= x2_new_i; x2_new_q_r <= x2_new_q;
            x1_new_i_r <= x1_new_i; x1_new_q_r <= x1_new_q;
            x2_old_i_r <= x2_old_i; x2_old_q_r <= x2_old_q;
            x1_old_i_r <= x1_old_i; x1_old_q_r <= x1_old_q;
        end
    end

    // =========================================================
    // Product Computation
    // =========================================================
    // 12×12 multiplies (24-bit results).
    // All products are registered (lines 88-107) before accumulation,
    // so DSP48E1 output register latency is absorbed by the explicit
    // pipeline stage. No timing concern with DSP inference.
    // Energy products also use DSP. Total: ~8-12 DSP48E1 for this module.
    
    // P_add = x1_new * conj(x2_new): re = r1*r2+i1*i2, im = i1*r2-r1*i2
    wire signed [24:0] p_add_re = x1_new_i_r * x2_new_i_r + x1_new_q_r * x2_new_q_r;
    wire signed [24:0] p_add_im = x1_new_q_r * x2_new_i_r - x1_new_i_r * x2_new_q_r;
    // P_sub = x1_old * conj(x2_old)
    wire signed [24:0] p_sub_re = x1_old_i_r * x2_old_i_r + x1_old_q_r * x2_old_q_r;
    wire signed [24:0] p_sub_im = x1_old_q_r * x2_old_i_r - x1_old_i_r * x2_old_q_r;

    // Energy: |x|^2 = r^2 + i^2
    (* use_dsp = "yes" *) wire [23:0] e1_add_w = x1_new_i_r * x1_new_i_r + x1_new_q_r * x1_new_q_r;
    (* use_dsp = "yes" *) wire [23:0] e1_sub_w = x1_old_i_r * x1_old_i_r + x1_old_q_r * x1_old_q_r;
    (* use_dsp = "yes" *) wire [23:0] e2_add_w = x2_new_i_r * x2_new_i_r + x2_new_q_r * x2_new_q_r;
    (* use_dsp = "yes" *) wire [23:0] e2_sub_w = x2_old_i_r * x2_old_i_r + x2_old_q_r * x2_old_q_r;

    // =========================================================
    // Registered Products (pipeline stage to break DSP→accumulator path)
    // =========================================================
    reg signed [24:0] p_add_re_r, p_add_im_r;
    reg signed [24:0] p_sub_re_r, p_sub_im_r;
    reg [23:0] e1_add_r, e1_sub_r, e2_add_r, e2_sub_r;

    always @(posedge clk) begin
        if (!rst_n || !enable) begin
            p_add_re_r <= 0; p_add_im_r <= 0;
            p_sub_re_r <= 0; p_sub_im_r <= 0;
            e1_add_r <= 0; e1_sub_r <= 0;
            e2_add_r <= 0; e2_sub_r <= 0;
        end else if (iq_valid_gated) begin
            p_add_re_r <= p_add_re;
            p_add_im_r <= p_add_im;
            p_sub_re_r <= p_sub_re;
            p_sub_im_r <= p_sub_im;
            e1_add_r   <= e1_add_w;
            e1_sub_r   <= e1_sub_w;
            e2_add_r   <= e2_add_w;
            e2_sub_r   <= e2_sub_w;
        end
    end

    // =========================================================
    // Accumulators (registered)
    // =========================================================
    // Max values:
    //   P_re: 64 * 2048^2 = 268M → 29 bits. Signed → 30 bits.
    //   E1/E2: 64 * (2048^2 + 2048^2) = 537M → 30 bits unsigned.
    // Use 32-bit to have 2 guard bits for potential rounding drift.
    reg signed [31:0] acc_p_re;
    reg signed [31:0] acc_p_im;
    reg        [31:0] acc_e1;
    reg        [31:0] acc_e2;

    // =========================================================
    // Sample Counter
    // =========================================================
    reg [24:0] sample_cnt;

    // Fill phase: accumulate without subtraction (building initial window sum)
    // sample_cnt goes 0,1,2... On each iq_valid clock:
    //   - Sample enters dl[0]
    //   - After sample_cnt >= 16, dl[16] is valid (first entered at sample 0)
    //   - The first valid pair for P is: dl[0]=sample[16], dl[16]=sample[0]
    //     This happens when sample_cnt = 16 (after 17 samples written: 0..16)
    //   - We accumulate during fill: sample_cnt = 16..79 (64 pairs total)
    //   With product registers (1 extra cycle), valid products available 1 cycle later:
    //   - First valid registered product at sample_cnt=18
    //   - Fill: sample_cnt 18..81 (64 accumulations)
    //   - Window full at sample_cnt=82, start sliding
    wire       filling    = (sample_cnt < 25'd82);
    wire       can_accum  = (sample_cnt >= 25'd18);
    wire window_full = (sample_cnt >= 25'd82);

    // =========================================================
    // Threshold Comparison (registered for timing)
    // =========================================================
    // Compute on registered accumulators. Pipeline stages break critical path.
    // |P|^2 >= 0.36 * E1 * E2
    // → 25 * |P|^2 >= 9 * E1 * E2
    //
    // Widths with 32-bit accumulators:
    //   p_re_sq max: (268M)^2 = 7.2e16 → 57 bits. Fits 64-bit.
    //   p_sq max: 1.4e17 → 57 bits.
    //   e_prod max: (537M)^2 = 2.9e17 → 59 bits. Fits 64-bit.
    //   25*p_sq: 62 bits, 9*e_prod: 63 bits.
    //
    // Pipeline: 3 stages
    //   Stage 1: squares (32×32→64, each fits 2 DSP48E1) and e_prod
    //   Stage 2: sum + latch
    //   Stage 3: compare (shift+add)

    // Absolute value for unsigned squaring (registered — breaks critical path)
    // This adds 1 cycle of latency to threshold detection, which is
    // irrelevant for frame detection (8+ consecutive periods required).
    reg [31:0] abs_p_re;
    reg [31:0] abs_p_im;
    reg        abs_wfull;  // pipeline window_full alongside abs values

    // Stage 1 (was stage 0: abs → square)
    reg [63:0] pipe1_p_re_sq;
    reg [63:0] pipe1_p_im_sq;
    reg [63:0] pipe1_e_prod;
    reg        pipe1_wfull;

    // Stage 2
    reg [63:0] pipe2_p_sq;
    reg [63:0] pipe2_e_prod;
    reg        pipe2_wfull;

    // Stage 3
    reg        threshold_met;

    always @(posedge clk) begin
        if (!rst_n || !enable) begin
            abs_p_re      <= 0;
            abs_p_im      <= 0;
            abs_wfull     <= 0;
            pipe1_p_re_sq <= 0;
            pipe1_p_im_sq <= 0;
            pipe1_e_prod  <= 0;
            pipe1_wfull   <= 0;
            pipe2_p_sq    <= 0;
            pipe2_e_prod  <= 0;
            pipe2_wfull   <= 0;
            threshold_met <= 0;
        end else begin
            // Stage 0: absolute value (registered to break acc→DSP critical path)
            abs_p_re <= acc_p_re[31] ? (~acc_p_re + 1) : acc_p_re;
            abs_p_im <= acc_p_im[31] ? (~acc_p_im + 1) : acc_p_im;
            // Energy floor: force threshold_met=0 when window energy is below
            // MIN_ENERGY. Prevents degenerate ratio comparisons in silence
            // (HIL inter-frame gaps, or any near-zero energy region).
            // MIN_ENERGY = 2^14 = 16384. With 64-sample window this requires
            // average per-sample energy of 256, i.e. ~16 LSBs RMS per sample.
            // Real STF at gain=50 OTA weakest (STA, ~38 RMS) ≈ 92,000 — above.
            // HIL 1% noise (±20 LSBs) ≈ 51,200 — above (but metric << 0.36).
            // Live ADC thermal noise (~2 LSBs RMS) ≈ 256 — below floor.
            // Persistence counter (16 consecutive) prevents false triggers
            // from noise even without the floor (noise metric ≈ 1/64 << 0.36).
            abs_wfull <= window_full && (acc_e1[31:14] != 0) && (acc_e2[31:14] != 0);

            // Stage 1: squares and energy product (use DSPs — 32×32 multiplies)
            pipe1_p_re_sq <= abs_p_re * abs_p_re;
            pipe1_p_im_sq <= abs_p_im * abs_p_im;
            pipe1_e_prod  <= acc_e1 * acc_e2;
            pipe1_wfull   <= abs_wfull;

            // Stage 2: sum squares, pass through e_prod
            pipe2_p_sq    <= pipe1_p_re_sq + pipe1_p_im_sq;
            pipe2_e_prod  <= pipe1_e_prod;
            pipe2_wfull   <= pipe1_wfull;

            // Stage 3: threshold comparison (runtime-configurable via right-shift)
            // 25*p_sq >= 9*(e_prod >> shift)
            // shift = threshold[2:0] (0-7):
            //   0 = original threshold 0.36 (standard 802.11)
            //   1 = 0.18 (2× more sensitive)
            //   2 = 0.09 (4× more sensitive)
            //   3 = 0.045 (8× more sensitive) — useful for weak OTA signals
            //   7 = ~0.003 (128× more sensitive) — triggers on almost anything
            //
            // Right-shift is FREE in routing (no LUTs). Original shift-add
            // implementation preserved for 25× and 9× multipliers.
            threshold_met <= pipe2_wfull &&
                            (({3'b0, pipe2_p_sq[59:0], 4'b0} + {3'b0, pipe2_p_sq[60:0], 3'b0} + {3'b0, pipe2_p_sq}) >=
                             ({(pipe2_e_prod >> threshold[2:0]), 3'b0} + {3'b0, (pipe2_e_prod >> threshold[2:0])}));
        end
    end

    // =========================================================
    // Persistence Counter
    // =========================================================
    localparam [7:0] PERSIST_TARGET = 8'd16;   // 1 STF period of persistence

    reg [7:0]  persist_cnt;
    reg        detected_latch;
    reg [24:0] thresh_start_cnt;  // sample_cnt when persistence started

    // STF-end detection: after frame_detect, wait for metric to drop
    reg        stf_end_armed;     // set on frame_detect, cleared on stf_end
    reg [3:0]  stf_end_cnt;       // consecutive below-threshold samples

    // Pipeline acknowledgment tracking (free-running correlator)
    reg        pipeline_ack_latch;

    // Rearm timeout: prevents detected_latch lockup under continuous signal.
    // After pipeline acks, count samples. If metric hasn't dropped within
    // REARM_TIMEOUT samples, force-clear detected_latch anyway. This handles
    // dense traffic where autocorrelation stays above threshold across multiple
    // frames (busy WiFi channels, EAPOL bursts + surrounding traffic).
    //
    // Timing: detection fires ~100 samples into a 160-sample STF. The 64-sample
    // sliding window keeps the metric elevated until ~200 samples after detection
    // (window must fully slide past the STF). Normal clear fires at ~200 samples.
    // Timeout must be LONGER than this to avoid spurious re-trigger on the same
    // frame. 256 samples (12.8 μs) is safely past the STF-to-data transition
    // and well within SIFS timing (320 samples = 16 μs).
    localparam [8:0] REARM_TIMEOUT = 9'd256;
    reg [8:0]  rearm_cnt;

    // =========================================================
    // Main Logic
    // =========================================================
    always @(posedge clk) begin
        if (!rst_n || !enable) begin
            // Hard reset: everything zeros.
            // !enable holds the entire datapath at zero — firmware releases
            // after the RF front-end is configured and stable.
            acc_p_re         <= 0;
            acc_p_im         <= 0;
            acc_e1           <= 0;
            acc_e2           <= 0;
            sample_cnt       <= 0;
            persist_cnt      <= 0;
            detected_latch   <= 0;
            frame_detect     <= 0;
            frame_offset     <= 0;
            thresh_start_cnt <= 0;
            stf_end          <= 0;
            stf_end_armed    <= 0;
            stf_end_cnt      <= 0;
            pipeline_ack_latch <= 0;
            rearm_cnt        <= 0;
        end else begin
            frame_detect <= 1'b0;
            stf_end      <= 1'b0;

            // Clear logic: two paths with different semantics.
            //
            // clear (unconditional): watchdog timeout — acquisition_ctrl gave up
            // waiting for stf_end. Must force-reset everything so the detector
            // can rearm for the next frame. Always honored.
            //
            // soft_clear (gated): playback_start — HIL re-arm pulse. Ignored
            // when stf_end_armed (a frame is mid-detection between frame_detect
            // and stf_end). This prevents the race where an HIL trigger pulse
            // destroys an in-progress detection, causing a lost frame with no tag.
            //
            // Accumulators and sample_cnt MUST be cleared on either path. The BRAM
            // clearing FSM zeros the delay line, but if accumulators retain residual
            // energy from the prior frame, the sliding window never subtracts it
            // (old BRAM entries are now zero, not the original values). This inflates
            // E1*E2 on the next detection attempt, causing threshold failure.
            if (clear || (soft_clear && !stf_end_armed)) begin
                persist_cnt      <= 0;
                detected_latch   <= 0;
                stf_end_armed    <= 0;
                stf_end_cnt      <= 0;
                pipeline_ack_latch <= 0;
                rearm_cnt        <= 0;
                acc_p_re         <= 0;
                acc_p_im         <= 0;
                acc_e1           <= 0;
                acc_e2           <= 0;
                sample_cnt       <= 0;
            end

            // Pipeline ack tracking: set immediately at detection time (internal
            // self-ack). This eliminates dependency on the external BD self-loop
            // wiring (stf_detect/frame_detect → stf_detect/pipeline_ack) which
            // may have routing delay that prevents reliable latch capture.
            // The external pipeline_ack input is kept as a redundant secondary
            // path (still works if connected, harmless if not).
            if (pipeline_ack)
                pipeline_ack_latch <= 1'b1;

            if (iq_valid_gated && !clear && !(soft_clear && !stf_end_armed)) begin
                if (!sample_cnt[24])  // saturate at 2^24 to prevent overflow
                    sample_cnt <= sample_cnt + 1;

                // --- Accumulator Update ---
                if (filling && can_accum) begin
                    // Fill phase: add only (from registered products)
                    acc_p_re <= acc_p_re + {{7{p_add_re_r[24]}}, p_add_re_r};
                    acc_p_im <= acc_p_im + {{7{p_add_im_r[24]}}, p_add_im_r};
                    acc_e1   <= acc_e1 + {8'd0, e1_add_r};
                    acc_e2   <= acc_e2 + {8'd0, e2_add_r};
                end else if (window_full) begin
                    // Slide: add new, subtract old (from registered products)
                    acc_p_re <= acc_p_re + {{7{p_add_re_r[24]}}, p_add_re_r}
                                        - {{7{p_sub_re_r[24]}}, p_sub_re_r};
                    acc_p_im <= acc_p_im + {{7{p_add_im_r[24]}}, p_add_im_r}
                                        - {{7{p_sub_im_r[24]}}, p_sub_im_r};
                    // Energy accumulators: clamp at 0 to prevent unsigned
                    // underflow during DATA→gap transitions. Without clamping,
                    // the 32-bit unsigned value wraps to ~4 billion when sub > add,
                    // poisoning the threshold comparison (9*E1*E2 overflows 64-bit)
                    // for ~64 samples until the window fully fills with new STF.
                    // This is the root cause of the SIFS burst drop on hardware:
                    // sim recovers in time (deterministic noise), but hardware
                    // noise patterns push recovery past the STF boundary.
                    acc_e1   <= ({1'b0, acc_e1} + {9'd0, e1_add_r} < {9'd0, e1_sub_r})
                                ? 32'd0
                                : acc_e1 + {8'd0, e1_add_r} - {8'd0, e1_sub_r};
                    acc_e2   <= ({1'b0, acc_e2} + {9'd0, e2_add_r} < {9'd0, e2_sub_r})
                                ? 32'd0
                                : acc_e2 + {8'd0, e2_add_r} - {8'd0, e2_sub_r};
                end

                // --- Detection Logic ---
                if (window_full) begin
                    if (threshold_met && !detected_latch) begin
                        if (persist_cnt == 0)
                            thresh_start_cnt <= sample_cnt;
                        if (persist_cnt >= PERSIST_TARGET - 1) begin
                            frame_detect       <= 1'b1;
                            frame_offset       <= thresh_start_cnt - 25'd82;
                            detected_latch     <= 1'b1;
                            pipeline_ack_latch <= 1'b1;  // internal self-ack (immediate)
                            stf_end_armed      <= 1'b1;
                            persist_cnt        <= 0;
                            rearm_cnt          <= 0;
                        end else begin
                            persist_cnt <= persist_cnt + 1;
                        end
                    end else if (!threshold_met) begin
                        persist_cnt <= 0;
                        rearm_cnt   <= 0;
                        // Clear latch after pipeline acknowledged AND metric dropped.
                        // This prevents re-triggering on the same STF's tail.
                        if (pipeline_ack_latch)
                            detected_latch <= 1'b0;
                    end else if (detected_latch && pipeline_ack_latch) begin
                        // Rearm timeout: metric is still above threshold but
                        // pipeline has acked. Count samples. After REARM_TIMEOUT,
                        // force-clear the latch so we can detect the next frame.
                        // This prevents permanent lockup on busy channels where
                        // autocorrelation never drops between frames.
                        if (rearm_cnt >= REARM_TIMEOUT - 1) begin
                            detected_latch <= 1'b0;
                            rearm_cnt      <= 0;
                        end else begin
                            rearm_cnt <= rearm_cnt + 1;
                        end
                    end
                end

                // --- STF End Detection ---
                // After frame_detect, watch for autocorrelation to drop.
                // 8 consecutive below-threshold = STF/GI2 boundary.
                if (stf_end_armed) begin
                    if (!threshold_met) begin
                        if (stf_end_cnt >= 4'd7) begin
                            stf_end       <= 1'b1;
                            stf_end_armed <= 1'b0;
                            stf_end_cnt   <= 0;
                        end else begin
                            stf_end_cnt <= stf_end_cnt + 1;
                        end
                    end else begin
                        stf_end_cnt <= 0;
                    end
                end
            end
        end
    end

    // =========================================================
    // Diagnostic output (for snap_mode=3 observation)
    // =========================================================
    // Packs key internal state into 32 bits:
    //   [31]    = threshold_met
    //   [30]    = detected_latch
    //   [29]    = window_full
    //   [28:24] = persist_cnt[4:0]
    //   [23:12] = acc_e1[31:20]  (top 12 bits of energy accumulator)
    //   [11:0]  = acc_e2[31:20]  (top 12 bits of energy accumulator)
    assign diag_out = {threshold_met, detected_latch, window_full,
                       persist_cnt[4:0],
                       acc_e1[31:20], acc_e2[31:20]};

    // =========================================================
    // Threshold diagnostic output (for snap observation of comparator)
    // =========================================================
    // Packs threshold comparison operands into 32 bits:
    //   [31]    = threshold_met (redundant but convenient)
    //   [30]    = detected_latch
    //   [29]    = window_full
    //   [28:16] = pipe2_p_sq[60:48]   (top 13 bits of |P|^2)
    //   [15:3]  = pipe2_e_prod[60:48] (top 13 bits of E1*E2)
    //   [2:0]   = persist_cnt[2:0]
    //
    // Interpretation: if threshold_met=1 and pipe2_p_sq[60:48] are nonzero,
    // the correlation magnitude is genuinely large (not a rounding artifact).
    // If threshold_met=1 but pipe2_p_sq[60:48]=0, the decision is based on
    // low-order bits only — possible synthesis issue.
    assign diag_thresh = {threshold_met, detected_latch, window_full,
                          pipe2_p_sq[60:48],
                          pipe2_e_prod[60:48],
                          persist_cnt[2:0]};

endmodule
