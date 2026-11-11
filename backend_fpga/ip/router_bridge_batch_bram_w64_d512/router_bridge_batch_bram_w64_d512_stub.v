// Copyright 1986-2018 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2018.2 (win64) Build 2258646 Thu Jun 14 20:03:12 MDT 2018
// Date        : Thu Jul 16 14:48:25 2026
// Host        : pc-axiao running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode synth_stub -rename_top router_bridge_batch_bram_w64_d512 -prefix
//               router_bridge_batch_bram_w64_d512_ router_bridge_batch_bram_w64_d512_stub.v
// Design      : router_bridge_batch_bram_w64_d512
// Purpose     : Stub declaration of top-level module interface
// Device      : xcku095-ffva1156-1-i
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* x_core_info = "blk_mem_gen_v8_4_1,Vivado 2018.2" *)
module router_bridge_batch_bram_w64_d512(clka, ena, wea, addra, dina, clkb, enb, addrb, doutb)
/* synthesis syn_black_box black_box_pad_pin="clka,ena,wea[0:0],addra[8:0],dina[63:0],clkb,enb,addrb[8:0],doutb[63:0]" */;
  input clka;
  input ena;
  input [0:0]wea;
  input [8:0]addra;
  input [63:0]dina;
  input clkb;
  input enb;
  input [8:0]addrb;
  output [63:0]doutb;
endmodule
