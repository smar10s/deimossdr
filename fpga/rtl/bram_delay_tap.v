// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/bram_delay_tap.v — Multi-tap BRAM delay line (circular buffer)
//
// Replaces shift-register delay lines that consume thousands of LUTs.
// Uses block RAM with circular buffer addressing. Supports back-to-back
// iq_valid (no idle clocks required between writes).
//
// IMPLEMENTATION:
//   Uses 2 BRAM instances (port-replicated) to provide 4 simultaneous
//   read ports. Each BRAM stores the same data; they differ only in
//   which read address is presented. Xilinx TDP BRAM allows:
//     - Port A: write + read (1 address for write, read on next clock)
//     - Port B: independent read (different address)
//   BRAM instance 0: port A reads tap0, port B reads tap1
//   BRAM instance 1: port A reads tap2, port B reads tap3
//   Both instances write din on iq_valid to the same wr_ptr.
//
// TIMING:
//   Write at clock T → Read data available at clock T+1 (BRAM latency).
//   This matches the original shift register timing where non-blocking
//   updates mean taps reflect the PREVIOUS iq_valid's state.
//
// RESOURCE COST:
//   - 2 BRAM18 (81 × 24 bits = 1944 bits each, well within 18 Kbit)
//   - ~30-50 LUTs for modular address arithmetic
//
// PARAMETERS:
//   DEPTH   — delay line entries (e.g., 81)
//   WIDTH   — bits per entry (e.g., 24 for packed 12-bit I + 12-bit Q)
//   TAP0-3  — tap offsets from newest sample (0 = most recent write)

module bram_delay_tap #(
    parameter DEPTH = 81,
    parameter WIDTH = 24,
    parameter TAP0  = 0,
    parameter TAP1  = 16,
    parameter TAP2  = 64,
    parameter TAP3  = 80,
    // USE_DISTRIBUTED_RAM: forces LUT-based distributed RAM instead of BRAM.
    // Guarantees behavioral correctness (no BRAM read-during-write ambiguity)
    // at a cost of ~800 LUTs. Set to 1 for SIFS burst-drop debugging.
    // Set to 0 to restore BRAM inference with WRITE_FIRST attribute.
    // TESTED: distributed RAM does NOT fix SIFS burst drop (session 7).
    // BRAM port mode confirmed READ_FIRST/SDP — no collision possible.
    // Root cause is elsewhere (accumulator timing, not memory).
    parameter USE_DISTRIBUTED_RAM = 0
) (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              clear,       // triggers write-clear FSM (zeros all BRAM entries)
    input  wire              iq_valid,
    input  wire [WIDTH-1:0]  din,
    output reg  [WIDTH-1:0]  tap0_out,
    output reg  [WIDTH-1:0]  tap1_out,
    output reg  [WIDTH-1:0]  tap2_out,
    output reg  [WIDTH-1:0]  tap3_out
);

    localparam ADDR_W = $clog2(DEPTH);

    // =========================================================
    // Write-clear FSM — zeros all entries on clear assertion
    // =========================================================
    // Takes DEPTH clocks (82) to zero entire delay line. Runs at full clock
    // rate (no iq_valid gating) so it completes in 82 ns at 100 MHz.
    // Uses wr_ptr as the write address (same port, no mux on address).
    // Only the write data is muxed: zeros during clear, din otherwise.
    // After clearing: wr_ptr=0, all entries=0. Combined with sample_cnt=0
    // in stf_detect, the fill phase accumulates zero-valued products,
    // naturally zeroing accumulators without conditional reset.
    //
    // TEST MODE AUTO-DETECT: In HIL mode, iq_valid fires on consecutive
    // clocks (1-per-clock from DDR playback). In live mode, iq_valid is
    // 1-in-5 (never consecutive). If 2 consecutive iq_valid are seen during
    // clearing, the FSM aborts — test mode data should not be discarded.
    // The golden vector naturally refills the delay line within 82 samples.
    reg                clear_active;
    reg [ADDR_W-1:0]  clear_cnt;
    reg                iq_valid_prev;  // previous-clock iq_valid for consecutive detect

    wire               clearing;  // internal: active clearing (suppressed on test mode detect)
    assign clearing = clear_active & ~consecutive_iq;  // internal use only

    always @(posedge clk) begin
        if (!rst_n)
            iq_valid_prev <= 0;
        else
            iq_valid_prev <= iq_valid;
    end

    wire consecutive_iq = iq_valid && iq_valid_prev;  // 2 back-to-back = test mode

    always @(posedge clk) begin
        if (!rst_n) begin
            clear_active <= 0;
            clear_cnt    <= 0;
        end else begin
            if (clear && !clear_active) begin
                clear_active <= 1;
                clear_cnt    <= 0;
            end else if (clear_active) begin
                if (consecutive_iq) begin
                    // Abort: test mode detected (1-per-clock injection)
                    clear_active <= 0;
                    clear_cnt    <= 0;
                end else if (clear_cnt == DEPTH[ADDR_W-1:0] - 1) begin
                    clear_active <= 0;
                    clear_cnt    <= 0;
                end else begin
                    clear_cnt <= clear_cnt + 1'b1;
                end
            end
        end
    end

    // =========================================================
    // Write pointer — advances on iq_valid (normal) or every clock (clearing)
    // Reset to 0 on clear start. During clearing, advances every clock to
    // sweep all entries. Same pointer serves as write address always.
    // =========================================================
    reg [ADDR_W-1:0] wr_ptr;

    always @(posedge clk) begin
        if (!rst_n)
            wr_ptr <= 0;
        else if (clear && !clear_active)
            wr_ptr <= 0;  // Reset on clear start
        else if (clear_active)
            wr_ptr <= (wr_ptr == DEPTH[ADDR_W-1:0] - 1) ? {ADDR_W{1'b0}} : wr_ptr + 1'b1;
        else if (iq_valid)
            wr_ptr <= (wr_ptr == DEPTH[ADDR_W-1:0] - 1) ? {ADDR_W{1'b0}} : wr_ptr + 1'b1;
    end

    // =========================================================
    // Write data/enable — shared between both RAM implementations
    // =========================================================
    wire             wr_en   = clearing | iq_valid;
    wire [WIDTH-1:0] wr_data = (clearing && !iq_valid) ? {WIDTH{1'b0}} : din;

    // =========================================================
    // Read address computation
    // =========================================================
    // After previous iq_valid wrote sample S-1 at position P and advanced
    // wr_ptr to P+1, the read addresses during the idle phase settle to:
    //   tap at offset K reads (wr_ptr - 1 - K) mod DEPTH
    //
    // Timing: on the idle clock after wr_ptr advances, rd_addr is computed
    // from the new wr_ptr. Registered read delivers data one clock later.
    // By the next iq_valid (4 idle clocks away in live mode), tap_out is
    // stable and holds the sample written K+1 iq_valids ago = correct value.
    //
    // This matches shift register behavior: on an iq_valid clock, the
    // non-blocking `dl[K]` reads the value from K positions back in the
    // history (the state BEFORE the current write enters).

    wire [ADDR_W-1:0] rd_addr0, rd_addr1, rd_addr2, rd_addr3;

    // Modular subtraction: (wr_ptr - 1 - offset) mod DEPTH
    function [ADDR_W-1:0] mod_sub;
        input [ADDR_W-1:0] ptr;
        input integer       offset;
        integer raw;
        begin
            // Add 2*DEPTH to guarantee positive before modulo.
            // Max input: ptr=80, offset=0 → raw = 80 + 162 - 1 - 0 = 241
            // Min input: ptr=0, offset=80 → raw = 0 + 162 - 1 - 80 = 81
            raw = {25'b0, ptr} + 2 * DEPTH - 1 - offset;
            if (raw >= DEPTH)
                raw = raw - DEPTH;
            if (raw >= DEPTH)
                raw = raw - DEPTH;
            mod_sub = raw[ADDR_W-1:0];
        end
    endfunction

    assign rd_addr0 = mod_sub(wr_ptr, TAP0);
    assign rd_addr1 = mod_sub(wr_ptr, TAP1);
    assign rd_addr2 = mod_sub(wr_ptr, TAP2);
    assign rd_addr3 = mod_sub(wr_ptr, TAP3);

    // =========================================================
    // Memory instances + write/read logic (implementation-dependent)
    // =========================================================
    // When USE_DISTRIBUTED_RAM=1: Forces LUT-based distributed RAM.
    // Guarantees behavioral correctness (eliminates any BRAM read-during-write
    // or timing ambiguity) at ~800 LUTs cost. Diagnostic mode for SIFS debug.
    //
    // When USE_DISTRIBUTED_RAM=0: Uses block RAM with explicit WRITE_FIRST
    // coding style (write-then-read in same always block) to ensure
    // read-during-write returns new data. Attribute rw_addr_collision="yes"
    // tells Vivado to handle simultaneous cross-port access correctly.
    //
    // NOTE: Previous DONT_TOUCH attribute was ignored by Vivado 2025.2
    // (Synth 8-6026 warning). Removed — not needed for correct inference.

    generate if (USE_DISTRIBUTED_RAM) begin : gen_lutram
        // --- Distributed RAM (LUT-based, behaviorally identical to sim) ---
        (* ram_style = "distributed" *) reg [WIDTH-1:0] mem0 [0:DEPTH-1];
        (* ram_style = "distributed" *) reg [WIDTH-1:0] mem1 [0:DEPTH-1];

        always @(posedge clk) begin
            if (wr_en) begin
                mem0[wr_ptr] <= wr_data;
                mem1[wr_ptr] <= wr_data;
            end
        end

        always @(posedge clk) begin
            if (!rst_n) begin
                tap0_out <= 0;
                tap1_out <= 0;
                tap2_out <= 0;
                tap3_out <= 0;
            end else begin
                tap0_out <= mem0[rd_addr0];
                tap1_out <= mem0[rd_addr1];
                tap2_out <= mem1[rd_addr2];
                tap3_out <= mem1[rd_addr3];
            end
        end

        integer i;
        initial begin
            for (i = 0; i < DEPTH; i = i + 1) begin
                mem0[i] = {WIDTH{1'b0}};
                mem1[i] = {WIDTH{1'b0}};
            end
        end

    end else begin : gen_bram
        // --- Block RAM with WRITE_FIRST inference ---
        // Write and read in same always block: Xilinx UG901 "RAM HDL Coding
        // Techniques" specifies this pattern infers WRITE_FIRST mode (read
        // sees new data when read/write address collide on same port).
        // rw_addr_collision="yes" ensures defined behavior on cross-port
        // address collision (returns new write data, not undefined).
        (* ram_style = "block", rw_addr_collision = "yes" *) reg [WIDTH-1:0] mem0 [0:DEPTH-1];
        (* ram_style = "block", rw_addr_collision = "yes" *) reg [WIDTH-1:0] mem1 [0:DEPTH-1];

        always @(posedge clk) begin
            if (wr_en) begin
                mem0[wr_ptr] <= wr_data;
                mem1[wr_ptr] <= wr_data;
            end
            if (!rst_n) begin
                tap0_out <= 0;
                tap1_out <= 0;
                tap2_out <= 0;
                tap3_out <= 0;
            end else begin
                tap0_out <= mem0[rd_addr0];
                tap1_out <= mem0[rd_addr1];
                tap2_out <= mem1[rd_addr2];
                tap3_out <= mem1[rd_addr3];
            end
        end

        integer i;
        initial begin
            for (i = 0; i < DEPTH; i = i + 1) begin
                mem0[i] = {WIDTH{1'b0}};
                mem1[i] = {WIDTH{1'b0}};
            end
        end

    end endgenerate

endmodule
