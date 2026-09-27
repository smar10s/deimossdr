// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/deimos_regs_axi.v
//
// Deimos receiver control/status registers.
//
// Standalone AXI-Lite slave module at base address 0x7C520000.
// Provides runtime configuration for the receiver pipeline
// (STF threshold, CFO dead-zone, snap observation point, etc.)
// and read-back of pipeline status (CFO phase_inc estimate).
//
// Register map (all 32-bit word-aligned, 4-byte increments):
//   0x00  STF_THRESH    RW  [7:0]=stf_detect threshold. Default 0.
//   0x04  STF_ENABLE    RW  [0]=stf_detect enable. Default 0 (held in reset).
//   0x08  DIAG_ACQ      RO  [15:8]=diag_frames_found, [7:0]=diag_frames_rejected.
//   0x0C  SNAP_MODE     RW  [2:0]=snap observation point selector. Default 0.
//   0x10  DIAG_DROP_CNT RO  RESERVED — unimplemented (decode_engine counter
//                           hardwired 0; addresses kept for future wiring).
//   0x14  DIAG_CLIP_CNT RO  RESERVED — unimplemented (chan_est counter
//                           const-tied 0 in the BD; addresses kept).
//   0x18  PHASE_INC     RO  [15:0]=cfo_est phase_inc (sign-extended). Read-only.
//   0x1C  VERSION       RO  Firmware ID (parameter). Read-only.

(* X_INTERFACE_PARAMETER = "PROTOCOL AXI4LITE, ADDR_WIDTH 32, DATA_WIDTH 32" *)
module deimos_regs_axi #(
    parameter VERSION = 32'h0001_0000
) (
    input  wire         s_axi_aclk,
    input  wire         s_axi_aresetn,

    input  wire [31:0]  s_axi_awaddr,
    input  wire         s_axi_awvalid,
    output reg          s_axi_awready,
    input  wire [31:0]  s_axi_wdata,
    input  wire [3:0]   s_axi_wstrb,
    input  wire         s_axi_wvalid,
    output reg          s_axi_wready,
    output wire [1:0]   s_axi_bresp,
    output reg          s_axi_bvalid,
    input  wire         s_axi_bready,
    input  wire [31:0]  s_axi_araddr,
    input  wire         s_axi_arvalid,
    output reg          s_axi_arready,
    output reg  [31:0]  s_axi_rdata,
    output wire [1:0]   s_axi_rresp,
    output reg          s_axi_rvalid,
    input  wire         s_axi_rready,

    output wire [7:0]  stf_threshold_out,
    output wire        stf_enable_out,
    output wire [2:0]  snap_mode_out,

    input  wire [15:0] phase_inc_in,
    input  wire [7:0]  diag_frames_found_in,
    input  wire [7:0]  diag_frames_rejected_in,
    input  wire [15:0] diag_drop_cnt_in,
    input  wire [15:0] diag_clip_cnt_in
);

    assign s_axi_bresp = 2'b00;
    assign s_axi_rresp = 2'b00;

    reg [7:0]  reg_stf_thresh;
    reg        reg_stf_enable;
    reg [2:0]  reg_snap_mode;

    initial begin
        reg_stf_thresh   = 8'd0;
        reg_stf_enable   = 1'b0;
        reg_snap_mode    = 3'd0;
    end

    assign stf_threshold_out = reg_stf_thresh;
    assign stf_enable_out    = reg_stf_enable;
    assign snap_mode_out     = reg_snap_mode;

    reg aw_en;

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            aw_en          <= 1'b1;
            reg_stf_thresh   <= 8'd0;
            reg_stf_enable   <= 1'b0;
            reg_snap_mode    <= 3'd0;
        end else begin
            if (s_axi_awvalid && !s_axi_awready && s_axi_wvalid && aw_en) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;
                aw_en <= 1'b0;
                case (s_axi_awaddr[4:2])
                    3'd0: reg_stf_thresh   <= s_axi_wdata[7:0];
                    3'd1: reg_stf_enable   <= s_axi_wdata[0];
                    // Slots 2,4,5: read-only diagnostic counters (writes ignored)
                    3'd3: reg_snap_mode    <= s_axi_wdata[2:0];
                endcase
            end else begin
                s_axi_awready <= 1'b0;
                s_axi_wready  <= 1'b0;
            end

            if (s_axi_awready && s_axi_wready && !s_axi_bvalid) begin
                s_axi_bvalid <= 1'b1;
            end else if (s_axi_bready && s_axi_bvalid) begin
                s_axi_bvalid <= 1'b0;
                aw_en <= 1'b1;
            end
        end
    end

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= 32'd0;
        end else begin
            if (s_axi_arvalid && !s_axi_arready) begin
                s_axi_arready <= 1'b1;
                s_axi_rvalid  <= 1'b1;
                case (s_axi_araddr[4:2])
                    3'd0: s_axi_rdata <= {24'd0, reg_stf_thresh};
                    3'd1: s_axi_rdata <= {31'd0, reg_stf_enable};
                    3'd2: s_axi_rdata <= {16'd0, diag_frames_found_in, diag_frames_rejected_in};
                    3'd3: s_axi_rdata <= {29'd0, reg_snap_mode};
                    3'd4: s_axi_rdata <= {16'd0, diag_drop_cnt_in};
                    3'd5: s_axi_rdata <= {16'd0, diag_clip_cnt_in};
                    3'd6: s_axi_rdata <= {{16{phase_inc_in[15]}}, phase_inc_in};
                    3'd7: s_axi_rdata <= VERSION;
                endcase
            end else begin
                s_axi_arready <= 1'b0;
            end

            if (s_axi_rvalid && s_axi_rready)
                s_axi_rvalid <= 1'b0;
        end
    end

endmodule
