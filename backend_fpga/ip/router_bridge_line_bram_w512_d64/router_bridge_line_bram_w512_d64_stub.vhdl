-- Copyright 1986-2018 Xilinx, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2018.2 (win64) Build 2258646 Thu Jun 14 20:03:12 MDT 2018
-- Date        : Thu Jul 16 14:48:35 2026
-- Host        : pc-axiao running 64-bit major release  (build 9200)
-- Command     : write_vhdl -force -mode synth_stub -rename_top router_bridge_line_bram_w512_d64 -prefix
--               router_bridge_line_bram_w512_d64_ router_bridge_line_bram_w512_d64_stub.vhdl
-- Design      : router_bridge_line_bram_w512_d64
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xcku095-ffva1156-1-i
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity router_bridge_line_bram_w512_d64 is
  Port ( 
    clka : in STD_LOGIC;
    ena : in STD_LOGIC;
    wea : in STD_LOGIC_VECTOR ( 0 to 0 );
    addra : in STD_LOGIC_VECTOR ( 5 downto 0 );
    dina : in STD_LOGIC_VECTOR ( 511 downto 0 );
    clkb : in STD_LOGIC;
    enb : in STD_LOGIC;
    addrb : in STD_LOGIC_VECTOR ( 5 downto 0 );
    doutb : out STD_LOGIC_VECTOR ( 511 downto 0 )
  );

end router_bridge_line_bram_w512_d64;

architecture stub of router_bridge_line_bram_w512_d64 is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "clka,ena,wea[0:0],addra[5:0],dina[511:0],clkb,enb,addrb[5:0],doutb[511:0]";
attribute x_core_info : string;
attribute x_core_info of stub : architecture is "blk_mem_gen_v8_4_1,Vivado 2018.2";
begin
end;
