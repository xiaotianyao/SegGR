// Copyright 1986-2018 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2018.2 (lin64) Build 2258646 Thu Jun 14 20:02:38 MDT 2018
// Date        : Thu Jul  2 20:21:30 2026
// Host        : bn-ws running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode synth_stub -rename_top pcie_clk_wiz_0_0 -prefix
//               pcie_clk_wiz_0_0_ pcie_clk_wiz_0_0_stub.v
// Design      : pcie_clk_wiz_0_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xcku095-ffva1156-1-i
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
module pcie_clk_wiz_0_0(clk_out1, clk_in1_p, clk_in1_n)
/* synthesis syn_black_box black_box_pad_pin="clk_out1,clk_in1_p,clk_in1_n" */;
  output clk_out1;
  input clk_in1_p;
  input clk_in1_n;
endmodule
