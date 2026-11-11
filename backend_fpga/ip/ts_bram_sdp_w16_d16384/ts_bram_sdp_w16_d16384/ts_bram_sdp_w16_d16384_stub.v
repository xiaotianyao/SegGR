// Copyright 1986-2018 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2018.2 (lin64) Build 2258646 Thu Jun 14 20:02:38 MDT 2018
// Date        : Tue Aug 18 15:02:45 2026
// Host        : bn-ws running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode synth_stub -rename_top ts_bram_sdp_w16_d16384 -prefix
//               ts_bram_sdp_w16_d16384_ ts_bram_sdp_w16_d16384_stub.v
// Design      : ts_bram_sdp_w16_d16384
// Purpose     : Stub declaration of top-level module interface
// Device      : xcku095-ffva1156-1-i
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* x_core_info = "blk_mem_gen_v8_4_1,Vivado 2018.2" *)
module ts_bram_sdp_w16_d16384(clka, ena, wea, addra, dina, clkb, enb, addrb, doutb)
/* synthesis syn_black_box black_box_pad_pin="clka,ena,wea[0:0],addra[13:0],dina[15:0],clkb,enb,addrb[13:0],doutb[15:0]" */;
  input clka;
  input ena;
  input [0:0]wea;
  input [13:0]addra;
  input [15:0]dina;
  input clkb;
  input enb;
  input [13:0]addrb;
  output [15:0]doutb;
endmodule
