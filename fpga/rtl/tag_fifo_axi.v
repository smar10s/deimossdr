// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/tag_fifo_axi.v — Tag FIFO + PSDU descriptor with AXI4-Lite interface
//
// Tag FIFO holds per-frame metadata (16 entries). Each tag entry includes
// a BRAM base address where that frame's PSDU bytes are stored. ARM reads
// bytes via a cursor register that is set on TAG_HI pop.
//
// The BRAM is addressed memory, not a FIFO. ARM reads from wherever the
// tag descriptor says. No independent read pointer to desync.
//
// Tag packing:
//   TAG_LO: {8'b0, fcs_ok, rate[3:0], 3'b0, length[11:0], 4'b0}
//   TAG_HI: {18'b0, psdu_addr[13:0]}  (also pops tag + sets cursor)
//
// Register map (base 0x7C510000):
//   0x00 STATUS     [0]=tag_empty, [1]=tag_full, [15:8]=tag_count
//   0x04 TAG_LO     Read (no pop): tag metadata (fcs, rate, length)
//   0x08 TAG_HI     Read-pop: BRAM base address, advances tag FIFO, sets PSDU cursor
//   0x0C CONTROL    [0]=flush (W1S)
//   0x10 FRAME_CNT  Total frames written (32-bit)
//   0x14 DROP_CNT   Frames dropped (tag full or BRAM full) (32-bit)
//   0x28 PSDU_DATA  Read: byte at cursor, advances cursor
//
// ARM protocol:
//   1. Read TAG_LO -> get fcs_ok, rate, length (peek, no pop)
//   2. Read TAG_HI -> get psdu_addr, pop tag, cursor set to psdu_addr
//   3. Read PSDU_DATA x (length-4) -> get PSDU bytes
//   4. Repeat from 1

module tag_fifo_axi (
    input  wire        clk,
    input  wire        rst,

    // Tag write interface (from decode_engine)
    input  wire        tag_wr_valid,
    input  wire [3:0]  tag_wr_rate,
    input  wire [11:0] tag_wr_length,
    input  wire        tag_wr_fcs_ok,

    // PSDU byte input (from psdu_packer)
    input  wire [7:0]  psdu_byte_in,
    input  wire        psdu_byte_valid,
    input  wire        psdu_frame_start,
    input  wire        psdu_frame_done,

    // Frame abort — L-SIG parity fail or watchdog timeout.
    input  wire        tag_abort_in,

    // AXI4-Lite slave interface
    input  wire [31:0] s_axi_awaddr,
    input  wire        s_axi_awvalid,
    output reg         s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output reg         s_axi_wready,
    output reg  [1:0]  s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [31:0] s_axi_araddr,
    input  wire        s_axi_arvalid,
    output reg         s_axi_arready,
    output reg  [31:0] s_axi_rdata,
    output reg  [1:0]  s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready
);

    // =========================================================
    // Tag FIFO storage (16 entries)
    // =========================================================
    (* ram_style = "registers" *) reg [31:0] tag_mem [0:15];
    (* ram_style = "registers" *) reg [13:0] psdu_addr_store [0:15];

    reg [4:0]  tag_wr_ptr;  // extra bit for full/empty detection
    reg [4:0]  tag_rd_ptr;

    wire [3:0] tag_wr_idx = tag_wr_ptr[3:0];
    wire [3:0] tag_rd_idx = tag_rd_ptr[3:0];

    wire tag_empty = (tag_wr_ptr == tag_rd_ptr);
    wire tag_full  = (tag_wr_ptr[4] != tag_rd_ptr[4]) &&
                     (tag_wr_ptr[3:0] == tag_rd_ptr[3:0]);
    wire [4:0] tag_count = tag_wr_ptr - tag_rd_ptr;

    // Counters
    reg [31:0] frame_cnt;
    reg [31:0] drop_cnt;

    // Tag packing
    wire [31:0] tag_packed = {8'b0, tag_wr_fcs_ok, tag_wr_rate, 3'b0, tag_wr_length, 4'b0};

    // =========================================================
    // PSDU BRAM — 16 KB via psdu_bram (8 RAMB18E1)
    // =========================================================
    reg  [13:0] bram_waddr;
    reg  [7:0]  bram_wdata;
    reg         bram_we;
    wire [13:0] bram_raddr;
    wire [7:0]  bram_rdata;

    psdu_bram u_psdu_bram (
        .clk   (clk),
        .rst   (rst),
        .waddr (bram_waddr),
        .wdata (bram_wdata),
        .we    (bram_we),
        .raddr (bram_raddr),
        .rdata (bram_rdata)
    );

    // =========================================================
    // PSDU write side — ring buffer
    //
    // Write flow:
    //   1. psdu_frame_start -> latch frame_base = wr_ptr, set active
    //   2. psdu_byte_valid  -> BRAM[wr_ptr++] = byte
    //   3. psdu_frame_done  -> frame complete, wait for tag
    //   4. tag_wr_valid     -> commit: store psdu_addr, or rewind on fcs_fail
    //
    // Overflow: if remaining space < 1536 at frame_start, set overflow
    //   flag — suppress BRAM writes, tag gets psdu_addr=0 (no bytes).
    // =========================================================
    reg [13:0] psdu_wr_ptr;        // next write address
    reg [13:0] psdu_frame_base;    // start of current frame's bytes
    reg        psdu_active;        // frame in progress
    reg        psdu_overflow;      // frame won't fit, suppress writes

    // Overflow detection: distance from wr_ptr to oldest unread entry.
    // If tag FIFO is empty, all space is free (ARM has consumed everything).
    // PSDU_SPACE_LIMIT = 16384 - 4091: a max-size frame (4095 LENGTH, 4091
    // stored bytes) starting at space_used=12293 ends exactly at the ring
    // end — any larger space_used could wrap and clobber unread frames.
    localparam [13:0] PSDU_SPACE_LIMIT = 14'd12293;
    wire [13:0] oldest_addr = psdu_addr_store[tag_rd_idx];
    wire [13:0] space_used = psdu_wr_ptr - oldest_addr;
    wire        bram_has_space = tag_empty || (space_used < PSDU_SPACE_LIMIT);

    // =========================================================
    // PSDU read side — cursor (set by TAG_HI pop)
    // =========================================================
    reg [13:0] psdu_cursor;

    assign bram_raddr = psdu_cursor;

    // =========================================================
    // FIFO + PSDU write logic
    // =========================================================
    reg flush_pulse;
    reg tag_pop;
    reg psdu_data_read;    // pulse from AXI read of PSDU_DATA
    reg psdu_cursor_load;  // pulse from AXI read of TAG_HI
    reg [13:0] psdu_cursor_load_val;

    always @(posedge clk) begin
        if (rst || flush_pulse) begin
            tag_wr_ptr      <= 0;
            tag_rd_ptr      <= 0;
            frame_cnt       <= 0;
            drop_cnt        <= 0;
            psdu_wr_ptr     <= 0;
            psdu_frame_base <= 0;
            psdu_active     <= 0;
            psdu_overflow   <= 0;
            bram_we         <= 0;
            psdu_cursor     <= 0;
        end else begin
            // Default: no BRAM write
            bram_we <= 0;

            // --- Tag FIFO pop (from AXI read path) ---
            if (tag_pop && !tag_empty) begin
                tag_rd_ptr <= tag_rd_ptr + 1;
            end

            // --- PSDU frame start ---
            if (psdu_frame_start) begin
                psdu_frame_base <= psdu_wr_ptr;
                psdu_active     <= 1;
                psdu_overflow   <= !bram_has_space;
            end

            // --- PSDU byte write ---
            if (psdu_byte_valid && psdu_active && !psdu_overflow) begin
                bram_waddr  <= psdu_wr_ptr;
                bram_wdata  <= psdu_byte_in;
                bram_we     <= 1;
                psdu_wr_ptr <= psdu_wr_ptr + 1;
            end

            // --- PSDU frame done (bytes complete, FCS still pending) ---
            if (psdu_frame_done && psdu_active) begin
                psdu_active <= 0;
            end

            // --- Frame abort (watchdog/L-SIG fail) ---
            if (tag_abort_in && psdu_active) begin
                psdu_wr_ptr <= psdu_frame_base;
                psdu_active <= 0;
            end

            // --- Tag write (commit point) ---
            if (tag_wr_valid) begin
                frame_cnt <= frame_cnt + 1;

                if (tag_full) begin
                    // Tag FIFO overflow: drop frame, rewind BRAM
                    drop_cnt    <= drop_cnt + 1;
                    psdu_wr_ptr <= psdu_frame_base;
                end else begin
                    tag_mem[tag_wr_idx] <= tag_packed;

                    if (tag_wr_fcs_ok && !psdu_overflow) begin
                        // FCS OK: store BRAM base address
                        psdu_addr_store[tag_wr_idx] <= psdu_frame_base;
                    end else begin
                        // FCS fail or overflow: rewind, store frame base.
                        // Base (not 0): space_used = wr_ptr - oldest_addr
                        // must stay correct when this entry becomes the
                        // oldest unread tag. Storing 0 makes space_used
                        // balloon and suppresss frames the ring can hold.
                        psdu_addr_store[tag_wr_idx] <= psdu_frame_base;
                        psdu_wr_ptr <= psdu_frame_base;
                    end

                    tag_wr_ptr <= tag_wr_ptr + 1;
                end

                psdu_overflow <= 0;
            end

            // --- PSDU cursor management ---
            // Load takes priority over advance (same-cycle TAG_HI + PSDU_DATA
            // read is impossible in practice, but load wins if it ever happens)
            if (psdu_cursor_load) begin
                psdu_cursor <= psdu_cursor_load_val;
            end else if (psdu_data_read) begin
                psdu_cursor <= psdu_cursor + 1;
            end
        end
    end

    // =========================================================
    // AXI4-Lite slave — write path
    // =========================================================
    reg        aw_done;
    reg [31:0] aw_addr;
    reg        w_done;
    reg [31:0] w_data;

    always @(posedge clk) begin
        if (rst) begin
            s_axi_awready  <= 0;
            s_axi_wready   <= 0;
            s_axi_bvalid   <= 0;
            s_axi_bresp    <= 0;
            aw_done        <= 0;
            w_done         <= 0;
            aw_addr        <= 0;
            w_data         <= 0;
            flush_pulse    <= 0;
        end else begin
            flush_pulse <= 0;

            // AW handshake
            if (s_axi_awvalid && !aw_done) begin
                s_axi_awready <= 1;
                aw_addr       <= s_axi_awaddr;
                aw_done       <= 1;
            end else begin
                s_axi_awready <= 0;
            end

            // W handshake
            if (s_axi_wvalid && !w_done) begin
                s_axi_wready <= 1;
                w_data       <= s_axi_wdata;
                w_done       <= 1;
            end else begin
                s_axi_wready <= 0;
            end

            // Complete write transaction
            if (aw_done && w_done && !s_axi_bvalid) begin
                case (aw_addr[7:0])
                    8'h0C: begin  // CONTROL register
                        if (w_data[0])
                            flush_pulse <= 1;
                    end
                endcase
                s_axi_bvalid <= 1;
                s_axi_bresp  <= 2'b00;
                aw_done      <= 0;
                w_done       <= 0;
            end

            // B channel handshake
            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 0;
            end
        end
    end

    // =========================================================
    // AXI4-Lite slave — read path
    // =========================================================
    reg        ar_done;
    reg [31:0] ar_addr;

    always @(posedge clk) begin
        if (rst) begin
            s_axi_arready      <= 0;
            s_axi_rvalid       <= 0;
            s_axi_rresp        <= 0;
            s_axi_rdata        <= 0;
            ar_done            <= 0;
            ar_addr            <= 0;
            tag_pop            <= 0;
            psdu_data_read     <= 0;
            psdu_cursor_load   <= 0;
            psdu_cursor_load_val <= 0;
        end else begin
            tag_pop            <= 0;
            psdu_data_read     <= 0;
            psdu_cursor_load   <= 0;

            // AR handshake
            if (s_axi_arvalid && !ar_done) begin
                s_axi_arready <= 1;
                ar_addr       <= s_axi_araddr;
                ar_done       <= 1;
            end else begin
                s_axi_arready <= 0;
            end

            // R response (one cycle after AR handshake)
            if (ar_done && !s_axi_rvalid) begin
                case (ar_addr[7:0])
                    8'h00: // STATUS
                        s_axi_rdata <= {16'b0, 3'b0, tag_count, 6'b0, tag_full, tag_empty};
                    8'h04: // TAG_LO (peek, no pop)
                        s_axi_rdata <= tag_empty ? 32'b0 : tag_mem[tag_rd_idx];
                    8'h08: begin // TAG_HI (pop tag, set cursor)
                        s_axi_rdata <= tag_empty ? 32'b0 :
                                       {18'b0, psdu_addr_store[tag_rd_idx]};
                        if (!tag_empty) begin
                            tag_pop              <= 1;
                            psdu_cursor_load     <= 1;
                            psdu_cursor_load_val <= psdu_addr_store[tag_rd_idx];
                        end
                    end
                    8'h0C: // CONTROL (reads back 0)
                        s_axi_rdata <= 32'b0;
                    8'h10: // FRAME_CNT
                        s_axi_rdata <= frame_cnt;
                    8'h14: // DROP_CNT
                        s_axi_rdata <= drop_cnt;
                    8'h28: begin // PSDU_DATA (byte at cursor, advance)
                        s_axi_rdata    <= {24'b0, bram_rdata};
                        psdu_data_read <= 1;
                    end
                    default:
                        s_axi_rdata <= 32'hDEADBEEF;
                endcase
                s_axi_rvalid <= 1;
                s_axi_rresp  <= 2'b00;
                ar_done      <= 0;
            end

            // R channel handshake complete
            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 0;
            end
        end
    end

    // =========================================================
    // Simulation assertions (not synthesized)
    // =========================================================
    `ifdef SIM
    always @(posedge clk) begin
        if (tag_wr_valid && tag_full && !rst)
            $warning("[tag_fifo_axi] TAG OVERFLOW: frame dropped");
        if (psdu_frame_start && !bram_has_space && !rst)
            $warning("[tag_fifo_axi] BRAM OVERFLOW: frame bytes will be suppressed");
    end
    `endif

endmodule
