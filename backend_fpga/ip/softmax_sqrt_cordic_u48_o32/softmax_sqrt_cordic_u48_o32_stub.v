// Copyright 1986-2018 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2018.2 (win64) Build 2258646 Thu Jun 14 20:03:12 MDT 2018
// Date        : Sun Jul 12 13:59:40 2026
// Host        : pc-axiao running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode synth_stub -rename_top softmax_sqrt_cordic_u48_o32 -prefix
//               softmax_sqrt_cordic_u48_o32_ softmax_sqrt_cordic_u48_o32_stub.v
// Design      : softmax_sqrt_cordic_u48_o32
// Purpose     : Stub declaration of top-level module interface
// Device      : xcku095-ffva1156-1-i
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* x_core_info = "cordic_v6_0_14,Vivado 2018.2" *)
module softmax_sqrt_cordic_u48_o32(aclk, aresetn, s_axis_cartesian_tvalid, 
  s_axis_cartesian_tready, s_axis_cartesian_tdata, m_axis_dout_tvalid, 
  m_axis_dout_tready, m_axis_dout_tdata)
/* synthesis syn_black_box black_box_pad_pin="aclk,aresetn,s_axis_cartesian_tvalid,s_axis_cartesian_tready,s_axis_cartesian_tdata[47:0],m_axis_dout_tvalid,m_axis_dout_tready,m_axis_dout_tdata[31:0]" */;
  input aclk;
  input aresetn;
  input s_axis_cartesian_tvalid;
  output s_axis_cartesian_tready;
  input [47:0]s_axis_cartesian_tdata;
  output m_axis_dout_tvalid;
  input m_axis_dout_tready;
  output [31:0]m_axis_dout_tdata;
endmodule
