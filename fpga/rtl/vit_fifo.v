// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/vit_fifo.v — Backpressure FIFO between soft_pairer and Viterbi
//
// Absorbs Viterbi traceback stalls (~160 cycles every TB_DEPTH=48 ACS steps).
// When Viterbi asserts busy, FIFO holds incoming pairs. When Viterbi
// is ready, FIFO drains at 1 pair/clock (no dead cycles).
//
// Depth: 256 entries. At rate 48/54 (64-QAM), the depuncturer produces
// 192-216 pairs per symbol at 1 pair/2 clocks while the Viterbi stalls for
// ~160 cycles per window. 256 entries provides ample margin for burst
// accumulation during stalls.
//
// Implementation: BRAM with prefetch register.
//   BRAM has 1-cycle read latency. A prefetch FSM issues reads
//   speculatively and holds the result in a register. The output
//   register is loaded from the prefetch register with zero dead
//   cycles during sustained drain.
//
// Resource: 0.5 BRAM18 + ~30 LUTs (control).

module vit_fifo (
    input  wire        clk,
    input  wire        rst_n,

    // Reset on new frame (flush FIFO)
    input  wire        frame_start,

    // Input from soft_pairer
    input  wire        wr_valid,
    input  wire [7:0]  wr_soft0,
    input  wire [7:0]  wr_soft1,

    // Output to Viterbi
    output reg         rd_valid,
    output reg  [7:0]  rd_soft0,
    output reg  [7:0]  rd_soft1,

    // Backpressure from Viterbi
    input  wire        vit_busy,

    // Backpressure: pointer-based full. Drives the upstream stall chain
    // (soft_pairer holds its output pair; the pair is retried when full
    // clears — never lost).
    output wire        full,

    // Flush pass-through: delays flush until FIFO is drained
    input  wire        flush_in,       // from decode_engine
    output reg         flush_out,      // to Viterbi (fires after FIFO empty)

    // Status
    output wire        overflow   // diagnostic: FIFO was full when write attempted
);

    localparam DEPTH   = 256;
    localparam ADDR_W  = 8;  // log2(256)

    // Storage — BRAM (registered read, 1-cycle latency).
    // 256 × 16 bits = 0.5 BRAM18K. Saves ~88 LUTs vs distributed RAM.
    (* ram_style = "block" *)
    reg [15:0] mem [0:DEPTH-1];

    // Pointers
    reg [ADDR_W:0] wr_ptr;  // extra bit for full/empty detection
    reg [ADDR_W:0] rd_ptr;

    wire [ADDR_W-1:0] wr_addr = wr_ptr[ADDR_W-1:0];
    wire [ADDR_W-1:0] rd_addr = rd_ptr[ADDR_W-1:0];

    // Status (based on pointer comparison — rd_ptr advances when prefetch issued)
    wire empty = (wr_ptr == rd_ptr);
    assign full = (wr_ptr[ADDR_W] != rd_ptr[ADDR_W]) &&
                  (wr_ptr[ADDR_W-1:0] == rd_ptr[ADDR_W-1:0]);

    // BRAM read port (registered output, 1-cycle latency)
    reg [15:0] bram_rd;
    always @(posedge clk) begin
        bram_rd <= mem[rd_addr];
    end

    // Write logic
    wire do_write = wr_valid && !full;
    assign overflow = wr_valid && full;

    // Prefetch register: holds one entry read from BRAM, ready for
    // immediate transfer to output register.
    reg        pf_valid;
    reg [15:0] pf_data;

    // consumed: Viterbi accepted the current output this cycle
    wire consumed = rd_valid && !vit_busy;

    // State: whether a BRAM read is in-flight (address presented,
    // data arrives next cycle)
    reg rd_in_flight;

    // =========================================================
    // Flush pass-through logic
    // =========================================================
    reg flush_pending /* verilator public */;
    reg [7:0] drain_idle_cnt /* verilator public */;
    reg       upstream_done /* verilator public */;
    wire fifo_fully_drained = upstream_done && empty && !rd_valid && !pf_valid && !rd_in_flight;

    always @(posedge clk) begin
        if (!rst_n || frame_start) begin
            wr_ptr         <= 0;
            rd_ptr         <= 0;
            rd_valid       <= 0;
            rd_soft0       <= 0;
            rd_soft1       <= 0;
            pf_valid       <= 0;
            pf_data        <= 0;
            rd_in_flight   <= 0;
            flush_pending  <= 0;
            flush_out      <= 0;
            drain_idle_cnt <= 0;
            upstream_done  <= 0;
        end else begin
            flush_out <= 0;  // default: pulse

            // Drain idle timer: after flush_in fires, wait for upstream
            // pipeline (deinterleaver → depuncturer → soft_pairer) to finish
            // producing soft pairs. By the time flush_in fires (S_WAIT_FCS
            // entry), the last DATA symbol's deinterleaver has ALREADY emitted.
            // Only depuncturer tail + soft_pairer remain. Empirically, 32
            // cycles is too aggressive for rate 3/4 on hardware (causes
            // ~15-25% FCS failures at rate 36). 64 cycles provides safe
            // margin while still saving 136 clocks vs the original 200.
            if (do_write || !flush_pending) begin
                drain_idle_cnt <= 0;
                upstream_done  <= 0;
            end else if (drain_idle_cnt < 8'd255) begin
                drain_idle_cnt <= drain_idle_cnt + 1;
                if (drain_idle_cnt == 8'd63)
                    upstream_done <= 1;
            end

            // Latch flush request
            if (flush_in)
                flush_pending <= 1;

            // Fire flush_out when fully drained
            if (flush_pending && fifo_fully_drained) begin
                flush_out     <= 1;
                flush_pending <= 0;
            end

            // Write
            if (do_write) begin
                mem[wr_addr] <= {wr_soft0, wr_soft1};
                wr_ptr <= wr_ptr + 1;
            end

            // =====================================================
            // BRAM prefetch pipeline
            //
            // Cycle N:   Issue read (present rd_addr), set rd_in_flight,
            //            advance rd_ptr
            // Cycle N+1: Capture bram_rd into pf_data, set pf_valid
            //
            // Output register loads from pf_data (zero extra latency
            // once prefetch is primed).
            // =====================================================

            // Capture in-flight read into prefetch register
            if (rd_in_flight) begin
                pf_valid <= 1;
                pf_data  <= bram_rd;
                rd_in_flight <= 0;
            end

            // Transfer prefetch → output register
            if (pf_valid && (consumed || !rd_valid)) begin
                rd_valid <= 1;
                rd_soft0 <= pf_data[15:8];
                rd_soft1 <= pf_data[7:0];
                pf_valid <= 0;
            end else if (consumed) begin
                // Consumed but no prefetch ready
                rd_valid <= 0;
            end

            // Issue new BRAM read if prefetch register is empty (or being
            // consumed this cycle) and data is available in FIFO
            if (!rd_in_flight && !empty) begin
                // Only issue if prefetch slot will be free
                if (!pf_valid || (pf_valid && (consumed || !rd_valid))) begin
                    rd_ptr <= rd_ptr + 1;
                    rd_in_flight <= 1;
                end
            end
        end
    end

    // =========================================================
    // Simulation assertions (not synthesized)
    // =========================================================
    // Contract (lever 2a-prime): wr_valid asserted while full is the
    // upstream stall hold — the write retries when full clears. No data
    // loss is possible; `overflow` remains as a stall-active diagnostic.
    //
    // Hold-contract assertion: mechanically detect an upstream that is
    // NOT actually wired to the stall chain (the 2026-08-23 deploy-cycle
    // class). soft_pairer freezes its output registers while stall_in is
    // asserted, so the write data MUST NOT change while full persists.
    // The one-cycle tolerance at full's rising edge covers the boundary
    // presentation: the pairer may present the next pair in the same
    // cycle the previous write fills the FIFO (that pair is then held and
    // retried when full clears). Data changing in a second consecutive
    // full cycle means the upstream advanced while stalled — pairs are
    // being dropped. (No wr_valid check: the pairer legitimately idles
    // with valid_out low while full.)
    `ifdef SIM
    reg        full_prev;
    reg [7:0]  prev_soft0, prev_soft1;
    always @(posedge clk) begin
        if (!rst_n || frame_start) begin
            full_prev <= 1'b0;
            prev_soft0 <= 8'd0;
            prev_soft1 <= 8'd0;
        end else begin
            if (full && full_prev) begin
                if (wr_soft0 !== prev_soft0 || wr_soft1 !== prev_soft1)
                    $error("[vit_fifo] STALL VIOLATION: wr data changed while full (%0h/%0h -> %0h/%0h) at %0t",
                           prev_soft0, prev_soft1, wr_soft0, wr_soft1, $time);
            end
            full_prev  <= full;
            prev_soft0 <= wr_soft0;
            prev_soft1 <= wr_soft1;
        end
    end
    `endif

endmodule
