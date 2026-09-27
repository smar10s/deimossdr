// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/depuncturer.v — 802.11a depuncturer (streaming)
//
// Inserts erasure (zero) soft bits at punctured positions to restore
// the rate-1/2 coded bit stream expected by the Viterbi decoder.
//
// Architecture: Streaming with small elastic pair FIFO.
//   The deinterleaver produces soft bits in pairs (2 bits/clk when
//   valid_in=1): bit 2j on soft_in0, bit 2j+1 on soft_in1. Pairs are
//   written into a 256×16-bit distributed-RAM FIFO. The output FSM
//   reads one bit per clock at "kept" pattern positions and emits
//   erasures (zero) at punctured positions. This avoids the
//   capture-then-emit architecture that dropped symbols in streaming
//   mode.
//
// Puncture patterns (IEEE 802.11-2020, 17.3.5.6):
//   Rate 1/2 (code_rate=0): passthrough, no insertions.
//   Rate 2/3 (code_rate=1): pattern [1,1,1,0] — 3 kept → 4 output per group.
//   Rate 3/4 (code_rate=2): pattern [1,1,1,0,0,1] — 4 kept → 6 output per group.
//
// symbol_start timing invariant: despite the name, this input must NOT be
// pulsed per symbol. A per-symbol reset would flush the elastic FIFO
// mid-stream and corrupt the coded bit order, because the FIFO legitimately
// carries bits across symbol boundaries. Symbol-level realignment needs no
// help: N_CBPS is a multiple of the puncture period (48/96/192/288/... bits)
// at every legacy rate, so pat_pos self-realigns at symbol boundaries, and
// the deinterleaver pads truncated captures to full N_CBPS. If either
// property changes (new rates, partial symbols), revisit this.
//
// It IS pulsed once per FRAME, at SIGNAL setup, to discard the previous
// frame's undrained tail and restart the pattern at position 0. Skipping
// that leaves pat_pos stranded mid-group and shifts the puncture phase for
// every later frame in a burst — see rx_pipeline.v where it is driven.
//
// Resource estimate: ~64 LUTs (control + pair FIFO), 0 BRAM.

module depuncturer (
    input  wire        clk,
    input  wire        rst_n,

    // Configuration
    input  wire [1:0]  code_rate,      // 0=1/2 (pass), 1=2/3, 2=3/4
    input  wire        symbol_start,   // pulse: reset pattern counter

    // Backpressure (lever 2a-prime): downstream hold freezes the output
    // FSM (valid_out held high); upstream must stop writing when
    // fifo_full asserts (writes while full are skipped and retried).
    input  wire        stall_in,

    // Input: deinterleaved soft bits, 2 per clock when valid_in=1
    // (pairs: bit 2j on soft_in0, bit 2j+1 on soft_in1)
    input  wire        valid_in,
    input  wire [7:0]  soft_in0,
    input  wire [7:0]  soft_in1,

    // Output: depunctured soft bits (one per clock when valid_out=1)
    output reg         valid_out,
    output reg  [7:0]  soft_out,       // signed 8-bit LLR (0x00 = erasure)

    // Backpressure to the deinterleaver: pair FIFO cannot accept a write
    output wire        fifo_full
);

    // =========================================================
    // Elastic pair FIFO (256 entries x 16 bits, distributed RAM)
    // Writes 1 pair/clk (from deinterleaver's 2/clk bursts).
    // Reads 1 bit/clk (lane select by rd_bits[0]).
    // Capacity: 510 bits. Worst-case peak occupancy: 257 bits (test 9:
    // 384 bits at 2/clk for 192 clocks, drain 4/6 per clock). Sized
    // above the brief's 128×16 because the 8-bit occupancy of that
    // design caps usable capacity at 254 bits, which the 384-bit
    // no-symbol_start streaming test exceeds by 3 bits. Entry count
    // MUST stay a power of 2: the occupancy arithmetic (2*wr_entries
    // mod 512 minus rd_bits) requires wr_entries to wrap at 256 so
    // 2*wr_entries wraps exactly at the 9-bit modulus. 256 entries
    // also covers multi-symbol carry-over at 54M.
    // =========================================================
    (* ram_style = "distributed" *)
    reg [15:0] fifo [0:255];
    reg [7:0] wr_entries /* verilator public */;  // 0..255 (write ptr, in pairs)
    reg [8:0] rd_bits /* verilator public */;     // 0..511 (read ptr, in bits)

    // 2*wr_entries mod 512 minus rd_bits, 9-bit wraparound = bits in FIFO
    /* verilator lint_off WIDTHTRUNC */
    wire [8:0] occupancy = ({1'b0, wr_entries, 1'b0}) - rd_bits;
    /* verilator lint_on WIDTHTRUNC */
    wire fifo_empty = (occupancy == 9'd0);
    assign fifo_full = (occupancy >= 9'd510);  // 2-bit write must fit (512-2)

    wire [15:0] fifo_rd_word = fifo[rd_bits[8:1]];
    wire [7:0]  fifo_rd_bit  = rd_bits[0] ? fifo_rd_word[7:0] : fifo_rd_word[15:8];

    always @(posedge clk) begin
        if (!rst_n || symbol_start)
            wr_entries <= 8'd0;
        else if (valid_in && !fifo_full) begin
            fifo[wr_entries] <= {soft_in0, soft_in1};
            wr_entries <= wr_entries + 8'd1;
        end
    end

    // =========================================================
    // Backpressure contract
    // =========================================================
    // Contract (lever 2a-prime): valid_in while fifo_full means the
    // upstream holder (deinterleaver) is retrying a held pair. The write
    // is skipped this cycle and retried when space frees -- never lost.
    //
    // Hold-contract assertions: mechanically detect an upstream that is
    // NOT actually wired to the stall chain (the 2026-08-23 deploy-cycle
    // class). While fifo_full, the held write must keep valid_in high and
    // its data unchanged — otherwise the write was dropped, not retried.
    `ifdef SIM
    reg        held_full;
    reg [7:0]  held_soft0, held_soft1;
    always @(posedge clk) begin
        if (!rst_n || symbol_start) begin
            held_full  <= 1'b0;
            held_soft0 <= 8'd0;
            held_soft1 <= 8'd0;
        end else begin
            if (held_full) begin
                if (!valid_in)
                    $error("[depuncturer] HOLD VIOLATION: valid_in dropped while fifo_full");
                if (soft_in0 !== held_soft0 || soft_in1 !== held_soft1)
                    $error("[depuncturer] HOLD VIOLATION: data changed while fifo_full");
            end
            held_full  <= valid_in && fifo_full;
            held_soft0 <= soft_in0;
            held_soft1 <= soft_in1;
        end
    end
    `endif

    // =========================================================
    // Pattern logic
    // =========================================================
    reg [2:0] pat_pos /* verilator public */;

    // Returns 1 if pattern position is "kept" (emit from FIFO)
    function pat_is_kept;
        input [1:0] rate;
        input [2:0] pos;
        begin
            case (rate)
                2'd1: // Rate 2/3: [1,1,1,0]
                    pat_is_kept = (pos != 3'd3);
                2'd2: // Rate 3/4: [1,1,1,0,0,1]
                    pat_is_kept = (pos != 3'd3) && (pos != 3'd4);
                default: // Rate 1/2: all kept
                    pat_is_kept = 1'b1;
            endcase
        end
    endfunction

    // Pattern length per rate
    function [2:0] pat_len;
        input [1:0] rate;
        begin
            case (rate)
                2'd1: pat_len = 3'd4;   // Rate 2/3
                2'd2: pat_len = 3'd6;   // Rate 3/4
                default: pat_len = 3'd1; // Rate 1/2 (always wrap)
            endcase
        end
    endfunction

    // =========================================================
    // Output FSM: emit from FIFO or erasure based on pattern
    //
    // Rate 1/2: direct passthrough (FIFO read whenever non-empty)
    // Rate 2/3, 3/4: at "kept" positions, read FIFO; at erasure
    // positions, emit zero without reading FIFO.
    //
    // The output runs whenever there is data to emit OR an erasure
    // is pending in the pattern sequence (between groups of kept bits).
    // =========================================================
    reg active /* verilator public */;  // we've seen data and are in an active emit sequence
    reg group_active /* verilator public */;  // we've read at least one kept bit in current pattern group

    // Parity of the pattern position each emitted bit came from
    // (simulation only). The emitted bit's global index is 6g+pat_pos at
    // rate 3/4 and 4g+pat_pos at rate 2/3; both group strides are even,
    // so this parity IS the coded-stream G0/G1 parity: even = G0.
    // rx_pipeline uses it to prove the pairing downstream stays aligned.
    `ifdef SIM
    reg emit_par /* verilator public */;
    `endif

    always @(posedge clk) begin
        if (!rst_n) begin
            rd_bits    <= 9'd0;
            pat_pos   <= 3'd0;
            valid_out <= 1'b0;
            soft_out  <= 8'd0;
            active    <= 1'b0;
            group_active <= 1'b0;
        end else if (symbol_start) begin
            rd_bits    <= 9'd0;
            pat_pos   <= 3'd0;
            active    <= 1'b0;
            valid_out <= 1'b0;
            group_active <= 1'b0;
        end else begin
            // Activate on incoming data even while stalled: resume after
            // a stall must not depend on a future valid_in.
            if (valid_in && !active) begin
                active   <= 1'b1;
            end

            if (!stall_in) begin
                valid_out <= 1'b0;

                if (active) begin
                    if (code_rate == 2'd0) begin
                        // Rate 1/2: direct passthrough from FIFO
                        if (!fifo_empty) begin
                            valid_out <= 1'b1;
                            soft_out  <= fifo_rd_bit;
                            rd_bits   <= rd_bits + 9'd1;
                            `ifdef SIM
                            emit_par <= rd_bits[0];
                            `endif
                        end
                        group_active <= 1'b0;
                    end else begin
                        // Rate 2/3 or 3/4: pattern-based emit
                        if (pat_is_kept(code_rate, pat_pos)) begin
                            if (!fifo_empty) begin
                                valid_out <= 1'b1;
                                soft_out  <= fifo_rd_bit;
                                rd_bits   <= rd_bits + 9'd1;
                                group_active <= 1'b1;
                                `ifdef SIM
                                emit_par <= pat_pos[0];
                                `endif
                                if (pat_pos == pat_len(code_rate) - 3'd1) begin
                                    pat_pos <= 3'd0;
                                    group_active <= 1'b0;
                                end else
                                    pat_pos <= pat_pos + 3'd1;
                            end
                        end else begin
                            if (group_active || !fifo_empty) begin
                                valid_out <= 1'b1;
                                soft_out  <= 8'h00;
                                `ifdef SIM
                                emit_par <= pat_pos[0];
                                `endif
                                if (pat_pos == pat_len(code_rate) - 3'd1) begin
                                    pat_pos <= 3'd0;
                                    group_active <= 1'b0;
                                end else
                                    pat_pos <= pat_pos + 3'd1;
                            end
                        end
                    end
                end
            end
            // else: stalled -- hold valid_out/soft_out; rd_bits/pat_pos/
            // group_active frozen (all updates live inside !stall_in).
        end
    end

endmodule
