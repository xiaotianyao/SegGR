-- Copyright 1986-2018 Xilinx, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2018.2 (lin64) Build 2258646 Thu Jun 14 20:02:38 MDT 2018
-- Date        : Tue Aug 18 23:50:07 2026
-- Host        : bn-ws running 64-bit Ubuntu 22.04.5 LTS
-- Command     : write_vhdl -force -mode synth_stub -rename_top ts_bram_sdp_w16_d32768 -prefix
--               ts_bram_sdp_w16_d32768_ ts_bram_sdp_w16_d32768_stub.vhdl
-- Design      : ts_bram_sdp_w16_d32768
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xcku095-ffva1156-1-i
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity ts_bram_sdp_w16_d32768 is
  Port ( 
    clka : in STD_LOGIC;
    ena : in STD_LOGIC;
    wea : in STD_LOGIC_VECTOR ( 0 to 0 );
    addra : in STD_LOGIC_VECTOR ( 14 downto 0 );
    dina : in STD_LOGIC_VECTOR ( 15 downto 0 );
    clkb : in STD_LOGIC;
    enb : in STD_LOGIC;
    addrb : in STD_LOGIC_VECTOR ( 14 downto 0 );
    doutb : out STD_LOGIC_VECTOR ( 15 downto 0 )
  );

end ts_bram_sdp_w16_d32768;

architecture stub of ts_bram_sdp_w16_d32768 is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "clka,ena,wea[0:0],addra[14:0],dina[15:0],clkb,enb,addrb[14:0],doutb[15:0]";
attribute x_core_info : string;
attribute x_core_info of stub : architecture is "blk_mem_gen_v8_4_1,Vivado 2018.2";
begin
end;
