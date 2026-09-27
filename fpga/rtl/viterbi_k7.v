// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/viterbi_k7.v — K=7 (64-state) Viterbi decoder for 802.11a BCC
//
// Generators: G0 = 133 octal (0b1011011), G1 = 171 octal (0b1111001)
// Rate 1/2, constraint length K=7, 64 states.
//
// Architecture (Task 3-only schedule; D23):
//   - Fully parallel ACS: 64 ACS units, one trellis step per clock
//   - Sliding-window traceback for streaming DATA decode
//   - Flush-mode traceback for SIGNAL decode (24 bits, traces from state 0)
//   - Decision memory: circular buffer, depth MEM_DEPTH = 128 entries
//     (power of 2: bare-decrement wrap, no comparator)
//   - Path metric: unsigned modulo arithmetic (no per-step normalization)
//     Correctness requires max PM spread < 2^(PM_WIDTH-1) = 1024.
//   - Soft input: 8-bit signed per coded bit (positive = likely 0)
//
// Dual mode operation:
//   FLUSH mode (SIGNAL): feed N pairs, pulse flush, get N decoded bits.
//     Traces back from state 0 (tail-biting). Same as legacy behavior.
//   STREAMING mode (DATA): after every TB_DEPTH=48 ACS steps, pauses input
//     for a 13-clock serialized best-state search, then traces back TB_DEPTH
//     steps while ACS concurrently accepts the NEXT window's pairs. The
//     traceback (49 clocks) hides inside the next window's cadence (48 ACS +
//     13 search = 61 clocks) → ~1.27 clk/pair. Continues until flush.
//
// Interface: frame_start resets. streaming_mode selects behavior.
//   - streaming_mode=0 (SIGNAL): flush triggers full traceback from state 0
//   - streaming_mode=1 (DATA): periodic traceback from best state; flush
//     triggers final partial traceback from the best state
//
// Timing:
//   SIGNAL: 24 ACS + 24 traceback + 24 output = 72 cycles
//   DATA streaming: input stalls only for the 13-clock best-state search;
//     traceback overlaps the next window's ACS.
//
// Resource: ~2-3K LUTs, 0 DSP, 1 RAMB36 (128*64 = 8192 bits)

module viterbi_k7 (
    input  wire        clk,
    input  wire        rst_n,

    // Frame control
    input  wire        frame_start,    // pulse: reset decoder state for new frame
    input  wire        flush,          // pulse: all input done, start final traceback
    input  wire        streaming_mode, // 0=flush mode (SIGNAL), 1=streaming (DATA)

    // Soft bit input (rate 1/2 pair)
    input  wire        valid_in,
    input  wire signed [7:0] soft0,   // LLR for G0 output (positive = likely 0)
    input  wire signed [7:0] soft1,   // LLR for G1 output (positive = likely 0)

    // Decoded bit output
    output reg         valid_out,
    output reg         bit_out,

    // Backpressure: when asserted, upstream must hold valid_in low
    output wire        busy
);

    // =========================================================
    // Parameters
    // =========================================================
    localparam N_STATES   = 64;
    localparam PM_WIDTH   = 11;       // unsigned path metric width, modulo normalization
    // Sized to PM_WIDTH on purpose: an unsized `1 << (PM_WIDTH-1)` is 32-bit,
    // which widens the (a-b) differences in the modulo comparisons and breaks
    // the wrap semantics (2026-09-03 sim regression, cleanup sweep).
    localparam [PM_WIDTH-1:0] PM_HALF = 1 << (PM_WIDTH-1); // modulo comparison threshold
    localparam TB_DEPTH   = 48;       // traceback window (~7*K). Chosen so that all rate-9 PSDU
                                      // bits fit in regular windows (17×48=816 ≥ 816 service+PSDU).
    localparam MEM_DEPTH  = 128;      // power of 2 for bare-decrement wrap (no comparator)
                                      // Span: oldest live traceback read to newest concurrent
                                      // ACS write = 2*TB_DEPTH+13 = 109 (< 128).
    localparam MEM_ADDR_W = 7;        // log2(MEM_DEPTH)
    localparam TB_ADDR_W  = 6;        // ceil(log2(TB_DEPTH+1))

    // =========================================================
    // Parity function
    // =========================================================
    function parity7;
        input [6:0] v;
        begin
            parity7 = ^v;
        end
    endfunction

    // =========================================================
    // Path metrics
    // =========================================================
    reg [PM_WIDTH-1:0] pm [0:N_STATES-1];
    reg [PM_WIDTH-1:0] pm_next [0:N_STATES-1];

    // =========================================================
    // Decision memory: circular buffer, 128 entries x 64 bits.
    // Depth raised 96 -> 128 (Task 3-only): ACS now writes the decision
    // memory while traceback reads it, so the span from the oldest live
    // traceback read to the newest ACS write is 2*TB_DEPTH+13 = 109.
    // 128 = one RAMB36 (8192 bits), and being a power of two lets circ_dec
    // be a bare decrement with no comparator.
    // =========================================================
    (* ram_style = "block" *)
    reg [N_STATES-1:0] decision_mem [0:MEM_DEPTH-1];
    reg [MEM_ADDR_W-1:0] wr_ptr;         // next write position (circular)

    // =========================================================
    // FSM
    // =========================================================
    localparam S_IDLE     = 3'd0;
    localparam S_FORWARD  = 3'd1;   // ACS processing input
    localparam S_FIND_BEST= 3'd2;   // best-state search (13-clock pipeline)
    localparam S_TRACE    = 3'd3;   // traceback; ACS runs concurrently (Task 3-only)
    reg [2:0] state;

    // Step counter: counts ACS steps within current window (0..TB_DEPTH-1)
    reg [TB_ADDR_W-1:0] window_steps;
    // Total steps since frame_start (for flush-mode traceback depth)
    // 16-bit: max legal frame is ~32782 steps (rate 6, LENGTH=4095).
    // Was 11-bit, wrapping at 2048 and defeating flush-depth logic for long frames.
    reg [15:0] total_steps;

    // Busy/backpressure: combinational. High when the Viterbi will NOT accept
    // valid_in on this clock edge.
    //
    // window_full: window_steps == TB_DEPTH. Triggers best-state search.
    // pipeline_will_fill: anticipatory — accounts for in-flight pipeline
    //   steps that will increment window_steps in 1-2 cycles.
    // S_FIND_BEST: the serialized 13-clock search stalls ACS.
    // S_TRACE is deliberately NOT in busy: traceback overlaps ACS (Task 3-only).
    //
    // flush_arriving: combinational signal indicating a flush is being received
    //   on this or the previous 2 cycles. Used to resolve the race between
    //   window_full and flush_pending when they coincide on the same clock
    //   (the last window of a frame arrives at exactly TB_DEPTH pairs).
    wire window_full = (streaming_mode && window_steps == TB_DEPTH);
    wire pipeline_will_fill = streaming_mode && (
        (window_steps == TB_DEPTH - 1 && (valid_in_r2 || valid_in_r)) ||
        (window_steps == TB_DEPTH - 2 && valid_in_r && valid_in_r2));
    wire flush_arriving = flush || flush_r || flush_r2;
    assign busy = (state == S_FIND_BEST || window_full || pipeline_will_fill);

    // Traceback-engine overrun canary. Task 3-only serializes the search with
    // traceback, so the next window may only fill once the previous trace has
    // drained. If window_full coincides with more than one trace step still
    // pending, the schedule has slipped (decision memory read/write span grows
    // past the 128-entry budget). Must stay silent.
    wire tb_engine_overrun = (state == S_TRACE) && window_full &&
                             (tb_remaining > 1);

    // =========================================================
    // Soft input pipeline register + quantization + branch metric
    //
    // Pipeline stage: register soft inputs and valid_in, compute quantized
    // values and branch metrics combinationally from the registered values.
    // This breaks the critical path between vit_fifo output register and
    // the ACS path metric computation.
    //
    // Round-to-nearest instead of truncate. Truncation (soft[7:3]) maps
    // small positive values (1..7) to 0, destroying soft information for
    // 64-QAM b2/b5 positions (max ±10). Rounding: (soft + 4)[7:3] maps
    // +10 → +1 (symmetric with -10 → -1). Capped at +15 to prevent
    // overflow when soft ∈ [120, 127] (which would wrap +16 → -16).
    //
    // 5-bit quantization (÷8): gives 16-QAM b1 LLR (±20) a quantized
    // value of ±3 instead of ±1, providing meaningful soft information.
    // Max branch metric: |q0|+|q1| = 15+15 = 30. With modulo normalization,
    // max metric spread ≈ 535 — well within PM_HALF = 1024.
    // =========================================================

    // Pipeline register for soft inputs
    reg signed [7:0] soft0_r, soft1_r;
    reg              valid_in_r;
    reg              flush_r;

    always @(posedge clk) begin
        if (!rst_n || frame_start) begin
            soft0_r    <= 0;
            soft1_r    <= 0;
            valid_in_r <= 0;
            flush_r    <= 0;
        end else begin
            soft0_r    <= soft0;
            soft1_r    <= soft1;
            valid_in_r <= valid_in && !busy;  // only register if accepted
            flush_r    <= flush;
        end
    end

    // Quantization from registered inputs (5-bit: ÷8 with rounding)
    wire signed [5:0] round0_6 = {soft0_r[7], soft0_r[7:3]} + {5'b0, soft0_r[2]};
    wire signed [4:0] q_soft0 = (round0_6 > 6'sd15) ? 5'sd15 : round0_6[4:0];
    wire signed [5:0] round1_6 = {soft1_r[7], soft1_r[7:3]} + {5'b0, soft1_r[2]};
    wire signed [4:0] q_soft1 = (round1_6 > 6'sd15) ? 5'sd15 : round1_6[4:0];

    // =========================================================
    // Branch metric pipeline register (stage 2)
    //
    // Registers the 4 branch metric values and valid_in. This splits the
    // critical path into three short stages:
    //   Stage 1 (soft_r): capture + quantize (2 LUT levels)
    //   Stage 2 (bm_r):   branch metric add/sub (2 LUT levels)
    //   Stage 3 (ACS):    pm[pred] + bm + compare → pm_next (CARRY4 chain)
    // =========================================================
    reg signed [PM_WIDTH-1:0] bm_r [0:3];
    reg                       valid_in_r2;
    reg                       flush_r2;

    always @(posedge clk) begin
        if (!rst_n || frame_start) begin
            bm_r[0]     <= 0;
            bm_r[1]     <= 0;
            bm_r[2]     <= 0;
            bm_r[3]     <= 0;
            valid_in_r2  <= 0;
            flush_r2     <= 0;
        end else begin
            bm_r[0]     <= {{(PM_WIDTH-5){q_soft0[4]}}, q_soft0} + {{(PM_WIDTH-5){q_soft1[4]}}, q_soft1};
            bm_r[1]     <= {{(PM_WIDTH-5){q_soft0[4]}}, q_soft0} - {{(PM_WIDTH-5){q_soft1[4]}}, q_soft1};
            bm_r[2]     <= -{{(PM_WIDTH-5){q_soft0[4]}}, q_soft0} + {{(PM_WIDTH-5){q_soft1[4]}}, q_soft1};
            bm_r[3]     <= -{{(PM_WIDTH-5){q_soft0[4]}}, q_soft0} - {{(PM_WIDTH-5){q_soft1[4]}}, q_soft1};
            valid_in_r2  <= valid_in_r;
            flush_r2     <= flush_r;
        end
    end

    // =========================================================
    // ACS computation (combinational, fully parallel)
    // Fed by registered branch metrics — critical path is only:
    // bm_r[idx] + pm[pred] + compare → pm_next (CARRY4 chain only)
    // =========================================================
    wire [PM_WIDTH-1:0] bm_table [0:3];
    assign bm_table[0] = bm_r[0];
    assign bm_table[1] = bm_r[1];
    assign bm_table[2] = bm_r[2];
    assign bm_table[3] = bm_r[3];

    // ACS for all 64 destination states
    integer d;
    reg [PM_WIDTH-1:0] candidate0, candidate1;
    reg [5:0] pred0_state, pred1_state;
    reg [6:0] full_reg0, full_reg1;
    reg c0_0, c1_0, c0_1, c1_1;
    reg [1:0] bm_idx0, bm_idx1;
    reg [5:0] d_vec;
    reg [N_STATES-1:0] decisions;

    always @(*) begin
        for (d = 0; d < N_STATES; d = d + 1) begin
            d_vec = d[5:0];

            pred0_state = {d_vec[4:0], 1'b0};
            pred1_state = {d_vec[4:0], 1'b1};

            full_reg0 = {d_vec[5], pred0_state};
            full_reg1 = {d_vec[5], pred1_state};

            c0_0 = parity7(full_reg0 & 7'o133);
            c1_0 = parity7(full_reg0 & 7'o171);
            c0_1 = parity7(full_reg1 & 7'o133);
            c1_1 = parity7(full_reg1 & 7'o171);

            bm_idx0 = {c0_0, c1_0};
            bm_idx1 = {c0_1, c1_1};

            candidate0 = pm[pred0_state] + bm_table[bm_idx0];
            candidate1 = pm[pred1_state] + bm_table[bm_idx1];

            // Modulo comparison: spread < PM_HALF = 1024
            if ((candidate0 - candidate1) < PM_HALF) begin
                pm_next[d] = candidate0;
                decisions[d] = 1'b0;
            end else begin
                pm_next[d] = candidate1;
                decisions[d] = 1'b1;
            end
        end
    end

    // =========================================================
    // Path metric normalization: modulo arithmetic.
    // PMs are unsigned, allowed to wrap at 2^PM_WIDTH. Signed
    // (two's-complement) comparison would invert at the wrap point,
    // so ACS uses modulo comparison ((a - b) < PM_HALF) instead.
    // Correctness requires max spread < PM_HALF = 1024, which holds
    // at measured Δ ≈ 535. No per-step subtraction needed.
    // =========================================================

    // =========================================================
    // Best-state finder: 8-way per-clock tree search.
    // pm_bank mirrors pm_next on every ACS step (banked for parallel read).
    // Tree: 7 comparators evaluate 8 states per clock, 8 clocks = 64 states.
    // Cost: ~120 LUTs (7 comparators + 8:1 read muxes), down from 65 clocks.
    // =========================================================
    reg [6:0] best_search_idx;  // pipeline group counter (0-12, 13 cycles total)
    reg [PM_WIDTH-1:0] best_metric;
    reg [5:0] best_state;

    // Banked snapshot: pm_bank[b][e] = pm_next[e*8 + b]
    // Rewritten on every ACS step; read during S_FIND_BEST.
    reg [PM_WIDTH-1:0] pm_bank [0:7][0:7];

    // Fully-pipelined 8-way max tree. Each comparison level is a separate
    // pipeline stage, so every stage path is 1 compare (CARRY4 chain + mux).
    // Stages: sel_val (bank mux) → l0 (4 pairs) → l1 (2 winners) → gmax
    // (final) → best-compare. All overlapped 1 cycle apart.
    wire [2:0] bs_grp = best_search_idx[2:0];
    reg [PM_WIDTH-1:0] sel_val [0:7];
    reg [2:0] bs_grp_d1, bs_grp_d2, bs_grp_d3;

    // Level 0 (combinational): 4 pairwise comparisons over sel_val
    wire cmp_l0_01 = (sel_val[0] - sel_val[1]) < PM_HALF;
    wire [PM_WIDTH-1:0] l0_val_01 = cmp_l0_01 ? sel_val[0] : sel_val[1];
    wire [2:0]          l0_idx_01 = cmp_l0_01 ? 3'd0 : 3'd1;
    wire cmp_l0_23 = (sel_val[2] - sel_val[3]) < PM_HALF;
    wire [PM_WIDTH-1:0] l0_val_23 = cmp_l0_23 ? sel_val[2] : sel_val[3];
    wire [2:0]          l0_idx_23 = cmp_l0_23 ? 3'd2 : 3'd3;
    wire cmp_l0_45 = (sel_val[4] - sel_val[5]) < PM_HALF;
    wire [PM_WIDTH-1:0] l0_val_45 = cmp_l0_45 ? sel_val[4] : sel_val[5];
    wire [2:0]          l0_idx_45 = cmp_l0_45 ? 3'd4 : 3'd5;
    wire cmp_l0_67 = (sel_val[6] - sel_val[7]) < PM_HALF;
    wire [PM_WIDTH-1:0] l0_val_67 = cmp_l0_67 ? sel_val[6] : sel_val[7];
    wire [2:0]          l0_idx_67 = cmp_l0_67 ? 3'd6 : 3'd7;
    // Pipeline stage after level 0
    reg [PM_WIDTH-1:0] l0_val_r [0:3];
    reg [2:0]          l0_idx_r [0:3];

    // Level 1 (combinational): 2 winners from registered level 0
    wire cmp_l1_01 = (l0_val_r[0] - l0_val_r[1]) < PM_HALF;
    wire [PM_WIDTH-1:0] l1_val_01 = cmp_l1_01 ? l0_val_r[0] : l0_val_r[1];
    wire [2:0]          l1_idx_01 = cmp_l1_01 ? l0_idx_r[0] : l0_idx_r[1];
    wire cmp_l1_23 = (l0_val_r[2] - l0_val_r[3]) < PM_HALF;
    wire [PM_WIDTH-1:0] l1_val_23 = cmp_l1_23 ? l0_val_r[2] : l0_val_r[3];
    wire [2:0]          l1_idx_23 = cmp_l1_23 ? l0_idx_r[2] : l0_idx_r[3];
    // Pipeline stage after level 1
    reg [PM_WIDTH-1:0] l1_val_r [0:1];
    reg [2:0]          l1_idx_r [0:1];

    // Level 2 (combinational): final winner
    wire cmp_final = (l1_val_r[0] - l1_val_r[1]) < PM_HALF;
    wire [PM_WIDTH-1:0] group_max       = cmp_final ? l1_val_r[0] : l1_val_r[1];
    wire [2:0]          group_max_bank  = cmp_final ? l1_idx_r[0] : l1_idx_r[1];
    wire [5:0]          group_max_state = {bs_grp_d3, group_max_bank};

    // Pipeline register between tree and best-compare. Keeps the tree
    // off the best_metric CE/D path.
    reg [PM_WIDTH-1:0] gmax_val_reg;
    reg [5:0]          gmax_state_reg;

    // =========================================================
    // Traceback logic
    // =========================================================
    reg [MEM_ADDR_W-1:0] tb_addr;
    reg [5:0]            tb_state;
    reg [MEM_ADDR_W-1:0] tb_remaining;   // wider: supports extended flush TB (up to 80)
    reg                  tb_primed;      // 1-cycle BRAM read latency absorbed

    // Decoded bits buffer, ping-ponged so window k's output overlaps
    // window k+1's ACS. wr_buf is written by traceback; rd_buf is drained
    // by the output stage. Traceback stores indices in descending order, so
    // it shifts left instead of indexing: after decode_len shifts,
    // decoded_bits[buf][k] holds bit k. A variable-index write would infer a
    // 96-way decoder. Forced to flip-flops: the array would otherwise infer
    // as LUTRAM (~99 LUTs), and the device is at its placement limit (D21).
    (* ram_style = "registers" *)
    reg [TB_DEPTH-1:0]   decoded_bits [0:1];
    reg                  wr_buf, rd_buf;
    reg                  out_active;
    reg [TB_ADDR_W-1:0]  decode_len;
    reg [TB_ADDR_W-1:0]  out_len;
    reg [TB_ADDR_W-1:0]  output_idx;

    // Extended flush traceback: when streaming mode flushes with window_steps < TB_DEPTH,
    // the flush traceback extends TB_DEPTH extra steps into the previous window's decision
    // memory for convergence. Only the first `decode_len` bits (nearest to frame end) are
    // stored and output; the extra steps provide convergence history only.
    // tb_conv_depth = number of convergence-only steps at the end of traceback.
    reg [MEM_ADDR_W-1:0] tb_conv_depth;

    // Flush-via-best-state flag: streaming-mode flush runs the best-state search
    // first (S_FIND_BEST) and then performs the extended traceback from the
    // best-metric state. The 802.11 pad bits (6 scrambled bits after the zeroed
    // tail) drive the encoder to a data-dependent end state, so tracing from
    // state 0 corrupts the last ~15 decoded bits of the final partial window
    // whenever the wrong-start path fails to merge before the stored region.
    reg flush_tb;

    // Decision memory read (registered — BRAM has 1-cycle read latency)
    reg [N_STATES-1:0] dec_rd_data;
    always @(posedge clk) begin
        dec_rd_data <= decision_mem[tb_addr];
    end

    // Circular buffer address arithmetic
    // MEM_DEPTH is a power of 2, so decrement wraps for free — no comparator,
    // shorter pointer path than (addr == 0 ? MEM_DEPTH-1 : addr-1).
    function [MEM_ADDR_W-1:0] circ_dec;
        input [MEM_ADDR_W-1:0] addr;
        begin
            circ_dec = addr - 1'b1;
        end
    endfunction

    // =========================================================
    // Sequential logic
    // =========================================================
    integer i;
    reg flush_pending;  // latch flush (captured in any state)

    always @(posedge clk) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            wr_ptr        <= 0;
            window_steps  <= 0;
            total_steps   <= 0;
            tb_addr       <= 0;
            tb_state      <= 0;
            tb_remaining  <= 0;
            tb_primed     <= 0;
            tb_conv_depth <= 0;
            decode_len    <= 0;
            out_len       <= 0;
            rd_buf        <= 0;
            wr_buf        <= 0;
            out_active    <= 0;
            flush_pending <= 0;
            flush_tb      <= 0;
            for (i = 0; i < N_STATES; i = i + 1)
                pm[i] <= {PM_WIDTH{1'b0}};
        end else if (frame_start) begin
            state         <= S_FORWARD;
            wr_ptr        <= 0;
            window_steps  <= 0;
            total_steps   <= 0;
            tb_addr       <= 0;
            tb_state      <= 0;
            tb_remaining  <= 0;
            tb_primed     <= 0;
            tb_conv_depth <= 0;
            decode_len    <= 0;
            out_len       <= 0;
            rd_buf        <= 0;
            wr_buf        <= 0;
            out_active    <= 0;
            flush_pending <= 0;
            flush_tb      <= 0;
            for (i = 0; i < N_STATES; i = i + 1)
                pm[i] <= {PM_WIDTH{1'b0}};
        end else begin
            // Clear the output-stage active flag once it has drained. The
            // drain itself is owned by the independent output process below.
            if (out_active && output_idx >= out_len)
                out_active <= 0;

            // Latch flush in ANY state (so it's never lost)
            // Respond to raw flush, pipelined flush_r, and double-pipelined flush_r2
            if (flush || flush_r || flush_r2)
                flush_pending <= 1;

            // =============================================================
            // ACS — shared by S_FORWARD and S_TRACE (Task 3-only schedule)
            //
            // Fires on double-pipelined valid_in_r2 (2 cycles after input is
            // accepted). In S_TRACE it runs concurrently with the traceback
            // engine, so the next window's decisions accumulate while the
            // previous window drains. S_FIND_BEST stalls it via busy.
            // Gated to S_FORWARD for the non-streaming (SIGNAL) path, whose
            // S_TRACE is a one-shot final traceback with no further input.
            // =============================================================
            if (valid_in_r2 &&
                (state == S_FORWARD || (streaming_mode && state == S_TRACE))) begin
                // Store decision and update path metrics (modulo, no per-step normalization)
                decision_mem[wr_ptr] <= decisions;
                for (i = 0; i < N_STATES; i = i + 1)
                    pm[i] <= pm_next[i];
                for (i = 0; i < 64; i = i + 1)
                    pm_bank[i[2:0]][i[5:3]] <= pm_next[i];
                // MEM_DEPTH is a power of 2, so the 7-bit pointer wraps for free.
                wr_ptr       <= wr_ptr + 1'b1;
                window_steps <= window_steps + 1;
                total_steps  <= total_steps + 1;
            end

            case (state)
                S_FORWARD: begin
                    // ACS runs in the shared block above.
                    //
                    // Window full: after TB_DEPTH steps collected, start traceback.
                    // Guard: do NOT enter S_FIND_BEST if a flush is arriving or pending.
                    // When total pairs divides exactly into TB_DEPTH (e.g. rate 36:
                    // 864/48=18), the vit_fifo's flush_out can fire on the same clock
                    // that window_steps reaches TB_DEPTH. Without the flush_arriving
                    // guard, S_FIND_BEST wins the race (flush_pending not yet latched).
                    // The Viterbi then processes the window normally (correct output)
                    // but on hardware, tight timing (WNS +0.085ns) on the busy →
                    // valid_in path can cause one extra pair to sneak through, corrupting
                    // the window count. The flush_arriving guard ensures that when flush
                    // and window_full coincide, we always take the flush path (from
                    // state 0), which is also more correct for the end-of-frame case.
                    //
                    // CORRECTION: 802.11 pad bits (after tail) are scrambled and cause
                    // the encoder to wander from state 0. The flush path (state 0
                    // traceback) is INCORRECT for full windows. When window_full fires,
                    // ALWAYS use S_FIND_BEST regardless of flush_pending. The flush is
                    // consumed on traceback handover (→ S_IDLE) after the window is correctly
                    // decoded from best_state. The flush path from state 0 is only
                    // correct for PARTIAL windows (window_steps < TB_DEPTH) where pad
                    // bits haven't had time to shift the encoder far from state 0.
                    if (window_full && !flush_arriving) begin
                        state <= S_FIND_BEST;
                        best_search_idx <= 0;
                    end else if (window_full && flush_arriving) begin
                        // Flush arrived exactly as window fills. Still use best-state
                        // traceback (pad bits mean state 0 is wrong for full windows).
                        // Latch flush_pending so traceback handover → S_IDLE handles it.
                        state <= S_FIND_BEST;
                        best_search_idx <= 0;
                    end else if (flush_pending) begin
                        if (!valid_in_r2 && !valid_in_r) begin
                            if (window_steps > 0) begin
                                if (streaming_mode) begin
                                    // Streaming flush: find the best-metric end
                                    // state first (802.11 pad bits after the zeroed
                                    // tail mean the encoder does NOT end at state 0),
                                    // then extended traceback from that state.
                                    // flush_pending stays latched: traceback handover
                                    // consumes it and returns to S_IDLE.
                                    state <= S_FIND_BEST;
                                    best_search_idx <= 0;
                                    flush_tb <= 1;
                                end else begin
                                    // Flush: traceback remaining steps from state 0.
                                    // SIGNAL: encoder is flushed to state 0 via its
                                    // 6 tail bits (no pad bits after them).
                                    flush_pending <= 0;
                                    tb_state     <= 6'd0;
                                    tb_addr      <= circ_dec(wr_ptr);
                                    decode_len   <= window_steps[TB_ADDR_W-1:0];
                                    tb_primed    <= 0;
                                    state        <= S_TRACE;
                                    tb_remaining <= {1'b0, window_steps[TB_ADDR_W-1:0]};
                                    tb_conv_depth <= 0;
                                    window_steps <= 0;
                                end
                            end else begin
                                flush_pending <= 0;
                                state <= S_IDLE;
                            end
                        end
                    end
                end

                S_FIND_BEST: begin
                    // Fully pipelined: sel_val → l0 → l1 → gmax → best-compare,
                    // overlapped 1 cycle apart. search_idx 0-11 process,
                    // 12 → done (13 cycles total). Each stage path is one
                    // 11-bit compare, keeping critical paths ~5 levels deep.
                    // Stage 1: latch selected bank values for current group
                    for (i = 0; i < 8; i = i + 1)
                        sel_val[i] <= pm_bank[i][bs_grp];
                    // Stage 2: latch level-0 winners (4 pairs)
                    l0_val_r[0] <= l0_val_01;
                    l0_idx_r[0] <= l0_idx_01;
                    l0_val_r[1] <= l0_val_23;
                    l0_idx_r[1] <= l0_idx_23;
                    l0_val_r[2] <= l0_val_45;
                    l0_idx_r[2] <= l0_idx_45;
                    l0_val_r[3] <= l0_val_67;
                    l0_idx_r[3] <= l0_idx_67;
                    // Stage 3: latch level-1 winners (2 pairs)
                    l1_val_r[0] <= l1_val_01;
                    l1_idx_r[0] <= l1_idx_01;
                    l1_val_r[1] <= l1_val_23;
                    l1_idx_r[1] <= l1_idx_23;
                    // Stage 4: latch final winner
                    gmax_val_reg   <= group_max;
                    gmax_state_reg <= group_max_state;
                    // Group-index delay chain (tree latency = 3)
                    bs_grp_d1 <= bs_grp;
                    bs_grp_d2 <= bs_grp_d1;
                    bs_grp_d3 <= bs_grp_d2;
                    // Stage 5: compare gmax (group idx-4) with running best
                    if (best_search_idx == 7'd4) begin
                        best_metric <= gmax_val_reg;
                        best_state  <= gmax_state_reg;
                    end else if ((best_search_idx > 7'd4) &&
                                 (best_search_idx <= 7'd11) &&
                                 (gmax_val_reg != best_metric) &&
                                 ((gmax_val_reg - best_metric) < PM_HALF)) begin
                        best_metric <= gmax_val_reg;
                        best_state  <= gmax_state_reg;
                    end
                    if (best_search_idx < 7'd12)
                        best_search_idx <= best_search_idx + 7'd1;
                    else begin
                        // Search complete — start traceback from best_state
                        tb_state      <= best_state;
                        tb_addr       <= circ_dec(wr_ptr);
                        tb_primed     <= 0;
                        if (flush_tb) begin
                            // Streaming flush variant: extended traceback from
                            // best_state covering window_steps + TB_DEPTH steps;
                            // store and output only the final window's bits.
                            flush_tb      <= 0;
                            decode_len    <= window_steps[TB_ADDR_W-1:0];
                            if (total_steps > {10'b0, window_steps}) begin
                                tb_remaining  <= window_steps[TB_ADDR_W-1:0] + TB_DEPTH[MEM_ADDR_W-1:0];
                                tb_conv_depth <= TB_DEPTH[MEM_ADDR_W-1:0];
                            end else begin
                                tb_remaining  <= {1'b0, window_steps[TB_ADDR_W-1:0]};
                                tb_conv_depth <= 0;
                            end
                        end else begin
                            tb_remaining  <= TB_DEPTH;
                            decode_len    <= TB_DEPTH;
                            tb_conv_depth <= 0;
                        end
                        // Restart the window counter so ACS (now enabled in
                        // S_TRACE) accumulates the next window during the
                        // upcoming traceback.
                        window_steps  <= 0;
                        state         <= S_TRACE;
                    end
                end

                S_TRACE: begin
                    if (!tb_primed) begin
                        // First cycle: BRAM read of tb_addr is in flight.
                        // Advance tb_addr for the next read (pipelined).
                        tb_primed <= 1;
                        tb_addr   <= circ_dec(tb_addr);
                    end else if (tb_remaining > 0) begin
                        // dec_rd_data is now valid (from address set 1 cycle ago).
                        // Store decoded bit only during data phase (past convergence).
                        // Convergence phase: tb_remaining <= tb_conv_depth (trace only)
                        // Data phase: tb_remaining > tb_conv_depth (trace + store)
                        if (tb_remaining > tb_conv_depth)
                            decoded_bits[wr_buf] <= {decoded_bits[wr_buf][TB_DEPTH-2:0], tb_state[5]};

                        // Reconstruct predecessor from decision
                        if (dec_rd_data[tb_state] == 1'b0)
                            tb_state <= {tb_state[4:0], 1'b0};
                        else
                            tb_state <= {tb_state[4:0], 1'b1};

                        tb_remaining <= tb_remaining - 1;
                        tb_addr      <= circ_dec(tb_addr);
                    end else begin
                        // Traceback complete. Hand the buffer to the output stage.
                        // Do NOT reset window_steps: in streaming mode ACS has been
                        // filling the next window concurrently with the traceback.
                        // A full next window goes straight to the serialized search;
                        // a partial window (or a pending flush) is handled by
                        // S_FORWARD's existing window-full / flush logic.
                        out_len      <= decode_len;
                        rd_buf       <= wr_buf;
                        wr_buf       <= ~wr_buf;
                        out_active   <= 1;
                        if (window_full) begin
                            state           <= S_FIND_BEST;
                            best_search_idx <= 0;
                        end else begin
                            state <= S_FORWARD;
                        end
                    end
                end

                default: begin
                    // S_IDLE: wait for frame_start
                end
            endcase
        end
    end

    // Output stage: drains one decoded bit per clock from the buffer the
    // traceback last completed. Runs concurrently with ACS.
    always @(posedge clk) begin
        if (!rst_n || frame_start) begin
            valid_out  <= 0;
            bit_out    <= 0;
            output_idx <= 0;
        end else if (out_active && output_idx < out_len) begin
            valid_out  <= 1;
            bit_out    <= decoded_bits[rd_buf][output_idx];
            output_idx <= output_idx + 1;
        end else begin
            valid_out  <= 0;
            output_idx <= 0;
        end
    end

    // =========================================================
    // Simulation assertions
    // =========================================================
    `ifdef SIM
    // Task 3-only schedule canary. The best-state search is serialized with
    // traceback, so a new window may only fill once the previous trace has
    // drained. If the schedule slips, the live decision-memory span exceeds
    // the 128-entry budget and the traceback reads overwritten decisions.
    // Must stay silent.
    always @(posedge clk) begin
        if (rst_n && !frame_start && tb_engine_overrun)
            $error("[viterbi_k7] TB ENGINE OVERRUN: window filled with traceback still active (tb_remaining=%0d) at %0t",
                   tb_remaining, $time);
    end
    `endif

endmodule
