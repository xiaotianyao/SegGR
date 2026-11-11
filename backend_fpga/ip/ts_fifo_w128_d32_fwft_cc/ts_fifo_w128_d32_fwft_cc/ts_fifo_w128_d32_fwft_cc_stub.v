// Copyright 1986-2018 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2018.2 (win64) Build 2258646 Thu Jun 14 20:03:12 MDT 2018
// Date        : Tue Jul 14 13:48:15 2026
// Host        : pc-axiao running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode synth_stub -rename_top ts_fifo_w128_d32_fwft_cc -prefix
//               ts_fifo_w128_d32_fwft_cc_ ts_fifo_w128_d32_fwft_cc_stub.v
// Design      : ts_fifo_w128_d32_fwft_cc
// Purpose     : Stub declaration of top-level module interface
// Device      : xcku095-ffva1156-1-i
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* x_core_info = "fifo_generator_v13_2_2,Vivado 2018.2" *)
module ts_fifo_w128_d32_fwft_cc(clk, rst, din, wr_en, rd_en, dout, full, empty, valid, 
  wr_rst_busy, rd_rst_busy)
/* synthesis syn_black_box black_box_pad_pin="clk,rst,din[127:0],wr_en,rd_en,dout[127:0],full,empty,valid,wr_rst_busy,rd_rst_busy" */;
  input clk;
  input rst;
  input [127:0]din;
  input wr_en;
  input rd_en;
  output [127:0]dout;
  output full;
  output empty;
  output valid;
  output wr_rst_busy;
  output rd_rst_busy;
endmodule
