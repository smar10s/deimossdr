// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/ltf_rom32.v — First 32 samples of LTF time-domain waveform
//
// Combinational ROM: 32 entries, 8-bit signed I + 8-bit signed Q.
// Infers as distributed LUT ROM (no BRAM).

module ltf_rom32 (
    input  wire [4:0] addr,
    output reg signed [7:0] rom_re,
    output reg signed [7:0] rom_im
);

    always @(*) begin
        case (addr)
            5'd0:  begin rom_re =  8'sd124; rom_im =  8'sd0;    end
            5'd1:  begin rom_re = -8'sd4;   rom_im = -8'sd95;   end
            5'd2:  begin rom_re =  8'sd31;  rom_im = -8'sd88;   end
            5'd3:  begin rom_re =  8'sd77;  rom_im =  8'sd65;   end
            5'd4:  begin rom_re =  8'sd17;  rom_im =  8'sd22;   end
            5'd5:  begin rom_re =  8'sd47;  rom_im = -8'sd69;   end
            5'd6:  begin rom_re = -8'sd91;  rom_im = -8'sd44;   end
            5'd7:  begin rom_re = -8'sd30;  rom_im = -8'sd84;   end
            5'd8:  begin rom_re =  8'sd77;  rom_im = -8'sd20;   end
            5'd9:  begin rom_re =  8'sd42;  rom_im =  8'sd3;    end
            5'd10: begin rom_re =  8'sd1;   rom_im = -8'sd91;   end
            5'd11: begin rom_re = -8'sd108; rom_im = -8'sd37;   end
            5'd12: begin rom_re =  8'sd19;  rom_im = -8'sd46;   end
            5'd13: begin rom_re =  8'sd46;  rom_im = -8'sd12;   end
            5'd14: begin rom_re = -8'sd18;  rom_im =  8'sd127;  end
            5'd15: begin rom_re =  8'sd94;  rom_im = -8'sd3;    end
            5'd16: begin rom_re =  8'sd49;  rom_im = -8'sd49;   end
            5'd17: begin rom_re =  8'sd29;  rom_im =  8'sd78;   end
            5'd18: begin rom_re = -8'sd45;  rom_im =  8'sd31;   end
            5'd19: begin rom_re = -8'sd104; rom_im =  8'sd52;   end
            5'd20: begin rom_re =  8'sd65;  rom_im =  8'sd73;   end
            5'd21: begin rom_re =  8'sd55;  rom_im =  8'sd11;   end
            5'd22: begin rom_re = -8'sd48;  rom_im =  8'sd64;   end
            5'd23: begin rom_re = -8'sd45;  rom_im = -8'sd17;   end
            5'd24: begin rom_re = -8'sd28;  rom_im = -8'sd119;  end
            5'd25: begin rom_re = -8'sd96;  rom_im = -8'sd13;   end
            5'd26: begin rom_re = -8'sd101; rom_im = -8'sd16;   end
            5'd27: begin rom_re =  8'sd59;  rom_im = -8'sd59;   end
            5'd28: begin rom_re = -8'sd2;   rom_im =  8'sd43;   end
            5'd29: begin rom_re = -8'sd73;  rom_im =  8'sd91;   end
            5'd30: begin rom_re =  8'sd73;  rom_im =  8'sd84;   end
            5'd31: begin rom_re =  8'sd10;  rom_im =  8'sd77;   end
            default: begin rom_re = 8'sd0; rom_im = 8'sd0; end
        endcase
    end

endmodule
