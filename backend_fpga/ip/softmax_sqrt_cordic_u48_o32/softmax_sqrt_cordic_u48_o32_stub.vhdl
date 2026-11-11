-- Copyright 1986-2018 Xilinx, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2018.2 (win64) Build 2258646 Thu Jun 14 20:03:12 MDT 2018
-- Date        : Sun Jul 12 13:59:40 2026
-- Host        : pc-axiao running 64-bit major release  (build 9200)
-- Command     : write_vhdl -force -mode synth_stub -rename_top softmax_sqrt_cordic_u48_o32 -prefix
--               softmax_sqrt_cordic_u48_o32_ softmax_sqrt_cordic_u48_o32_stub.vhdl
-- Design      : softmax_sqrt_cordic_u48_o32
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xcku095-ffva1156-1-i
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity softmax_sqrt_cordic_u48_o32 is
  Port ( 
    aclk : in STD_LOGIC;
    aresetn : in STD_LOGIC;
    s_axis_cartesian_tvalid : in STD_LOGIC;
    s_axis_cartesian_tready : out STD_LOGIC;
    s_axis_cartesian_tdata : in STD_LOGIC_VECTOR ( 47 downto 0 );
    m_axis_dout_tvalid : out STD_LOGIC;
    m_axis_dout_tready : in STD_LOGIC;
    m_axis_dout_tdata : out STD_LOGIC_VECTOR ( 31 downto 0 )
  );

end softmax_sqrt_cordic_u48_o32;

architecture stub of softmax_sqrt_cordic_u48_o32 is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "aclk,aresetn,s_axis_cartesian_tvalid,s_axis_cartesian_tready,s_axis_cartesian_tdata[47:0],m_axis_dout_tvalid,m_axis_dout_tready,m_axis_dout_tdata[31:0]";
attribute x_core_info : string;
attribute x_core_info of stub : architecture is "cordic_v6_0_14,Vivado 2018.2";
begin
end;
