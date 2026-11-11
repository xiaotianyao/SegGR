set_property CFGBVS GND [current_design]
set_property CONFIG_VOLTAGE 1.8 [current_design]

# router_wrapper top-port note:
# - sys_rst_n is the board FPGA_RSETN push button from fpga/pins_button.txt.
# - ddr_ref_clk_i and ddr_sys_rst_i are generated inside router_wrapper using
#   a local PL_CLK0 IBUFDS/BUFG path and the startup-reset pattern from backup/test_dram.
# - router_busy_o and ddr_calib_done_o are exposed on LED1/LED2.

###############################################################################
# System Reset
###############################################################################

set_property PACKAGE_PIN N27 [get_ports sys_rst_n]
set_property IOSTANDARD LVCMOS18 [get_ports sys_rst_n]
set_false_path -from [get_ports sys_rst_n]

###############################################################################
# LEDs / Debug Outputs
###############################################################################

# LED 1
set_property PACKAGE_PIN E12 [get_ports router_busy_o]
# LED 2
set_property PACKAGE_PIN F12 [get_ports ddr_calib_done_o]

set_property IOSTANDARD LVCMOS18 [get_ports router_busy_o]
set_property IOSTANDARD LVCMOS18 [get_ports ddr_calib_done_o]
set_property DRIVE 8 [get_ports router_busy_o]
set_property DRIVE 8 [get_ports ddr_calib_done_o]
set_property SLEW SLOW [get_ports router_busy_o]
set_property SLEW SLOW [get_ports ddr_calib_done_o]

set_property PACKAGE_PIN E13 [get_ports test_led_sys]
set_property PACKAGE_PIN F13 [get_ports test_led_pcie]
set_property IOSTANDARD LVCMOS18 [get_ports test_led_sys]
set_property IOSTANDARD LVCMOS18 [get_ports test_led_pcie]
set_property DRIVE 8 [get_ports test_led_sys]
set_property DRIVE 8 [get_ports test_led_pcie]
set_property SLEW SLOW [get_ports test_led_sys]
set_property SLEW SLOW [get_ports test_led_pcie]

###############################################################################
# Clock Domain Groups
###############################################################################

set_property PACKAGE_PIN AB6 [get_ports clk_pcie_p]
set_property PACKAGE_PIN AB5 [get_ports clk_pcie_n]
create_clock -name clk_pcie_refclk -period 10.000 [get_ports clk_pcie_p]

set_property PACKAGE_PIN AK17 [get_ports {sys_clk_diff_i_clk_p[0]}]
set_property PACKAGE_PIN AK16 [get_ports {sys_clk_diff_i_clk_n[0]}]
set_property IOSTANDARD LVDS [get_ports {sys_clk_diff_i_clk_p[0]}]
set_property IOSTANDARD LVDS [get_ports {sys_clk_diff_i_clk_n[0]}]
create_clock -name clk_200m -period 5.000 [get_ports {sys_clk_diff_i_clk_p[0]}]
set_property CLOCK_DEDICATED_ROUTE BACKBONE [get_nets ddr_ref_clk]

create_generated_clock -name ddr_ref_clk -divide_by 1 \
  -source [get_pins u_ddr_refclk_bufg/I] [get_pins u_ddr_refclk_bufg/O]
create_generated_clock -name router_clk -divide_by 3 \
  -source [get_pins u_router_clk_div/I] [get_pins u_router_clk_div/O]

# Router-DDR/PCIe crossings must go through AXI CDC bridges or explicit
# single-bit synchronizers.
set_clock_groups -asynchronous \
    -group [get_clocks -include_generated_clocks ddr_ref_clk] \
    -group [get_clocks -include_generated_clocks router_clk] \
    -group [get_clocks {clk_pcie_refclk pcie_clk_axi}]

###############################################################################
# PCIe Gen3 x4
###############################################################################

set_property PACKAGE_PIN K22 [get_ports rstn_pcie_i]
set_property IOSTANDARD LVCMOS18 [get_ports rstn_pcie_i]

# ddr4_axi_0/par/example_design.xdc is intentionally neutralized because it
# contains board-invalid DDR pin placements. This board-level XDC owns DDR LOCs.

set_property PACKAGE_PIN AB1 [get_ports {pcie_7x_mgt_rtl_0_rxn[0]}]
set_property PACKAGE_PIN AB2 [get_ports {pcie_7x_mgt_rtl_0_rxp[0]}]
set_property PACKAGE_PIN AC3 [get_ports {pcie_7x_mgt_rtl_0_txn[0]}]
set_property PACKAGE_PIN AC4 [get_ports {pcie_7x_mgt_rtl_0_txp[0]}]

set_property PACKAGE_PIN AD1 [get_ports {pcie_7x_mgt_rtl_0_rxn[1]}]
set_property PACKAGE_PIN AD2 [get_ports {pcie_7x_mgt_rtl_0_rxp[1]}]
set_property PACKAGE_PIN AE3 [get_ports {pcie_7x_mgt_rtl_0_txn[1]}]
set_property PACKAGE_PIN AE4 [get_ports {pcie_7x_mgt_rtl_0_txp[1]}]

set_property PACKAGE_PIN AF1 [get_ports {pcie_7x_mgt_rtl_0_rxn[2]}]
set_property PACKAGE_PIN AF2 [get_ports {pcie_7x_mgt_rtl_0_rxp[2]}]
set_property PACKAGE_PIN AG3 [get_ports {pcie_7x_mgt_rtl_0_txn[2]}]
set_property PACKAGE_PIN AG4 [get_ports {pcie_7x_mgt_rtl_0_txp[2]}]

set_property PACKAGE_PIN AH1 [get_ports {pcie_7x_mgt_rtl_0_rxn[3]}]
set_property PACKAGE_PIN AH2 [get_ports {pcie_7x_mgt_rtl_0_rxp[3]}]
set_property PACKAGE_PIN AH5 [get_ports {pcie_7x_mgt_rtl_0_txn[3]}]
set_property PACKAGE_PIN AH6 [get_ports {pcie_7x_mgt_rtl_0_txp[3]}]

###############################################################################
# DDR4 Data Bus
###############################################################################

set_property PACKAGE_PIN AE20 [get_ports {c0_ddr4_dq[0]}]
set_property PACKAGE_PIN AG20 [get_ports {c0_ddr4_dq[1]}]
set_property PACKAGE_PIN AF20 [get_ports {c0_ddr4_dq[2]}]
set_property PACKAGE_PIN AE22 [get_ports {c0_ddr4_dq[3]}]
set_property PACKAGE_PIN AD20 [get_ports {c0_ddr4_dq[4]}]
set_property PACKAGE_PIN AG22 [get_ports {c0_ddr4_dq[5]}]
set_property PACKAGE_PIN AF22 [get_ports {c0_ddr4_dq[6]}]
set_property PACKAGE_PIN AE23 [get_ports {c0_ddr4_dq[7]}]
set_property PACKAGE_PIN AF24 [get_ports {c0_ddr4_dq[8]}]
set_property PACKAGE_PIN AJ23 [get_ports {c0_ddr4_dq[9]}]
set_property PACKAGE_PIN AF23 [get_ports {c0_ddr4_dq[10]}]
set_property PACKAGE_PIN AH23 [get_ports {c0_ddr4_dq[11]}]
set_property PACKAGE_PIN AG25 [get_ports {c0_ddr4_dq[12]}]
set_property PACKAGE_PIN AJ24 [get_ports {c0_ddr4_dq[13]}]
set_property PACKAGE_PIN AG24 [get_ports {c0_ddr4_dq[14]}]
set_property PACKAGE_PIN AH22 [get_ports {c0_ddr4_dq[15]}]
set_property PACKAGE_PIN AK22 [get_ports {c0_ddr4_dq[16]}]
set_property PACKAGE_PIN AL22 [get_ports {c0_ddr4_dq[17]}]
set_property PACKAGE_PIN AM20 [get_ports {c0_ddr4_dq[18]}]
set_property PACKAGE_PIN AL23 [get_ports {c0_ddr4_dq[19]}]
set_property PACKAGE_PIN AK23 [get_ports {c0_ddr4_dq[20]}]
set_property PACKAGE_PIN AL25 [get_ports {c0_ddr4_dq[21]}]
set_property PACKAGE_PIN AL20 [get_ports {c0_ddr4_dq[22]}]
set_property PACKAGE_PIN AL24 [get_ports {c0_ddr4_dq[23]}]
set_property PACKAGE_PIN AM22 [get_ports {c0_ddr4_dq[24]}]
set_property PACKAGE_PIN AP24 [get_ports {c0_ddr4_dq[25]}]
set_property PACKAGE_PIN AN22 [get_ports {c0_ddr4_dq[26]}]
set_property PACKAGE_PIN AN24 [get_ports {c0_ddr4_dq[27]}]
set_property PACKAGE_PIN AN23 [get_ports {c0_ddr4_dq[28]}]
set_property PACKAGE_PIN AP25 [get_ports {c0_ddr4_dq[29]}]
set_property PACKAGE_PIN AP23 [get_ports {c0_ddr4_dq[30]}]
set_property PACKAGE_PIN AM24 [get_ports {c0_ddr4_dq[31]}]
set_property PACKAGE_PIN AM26 [get_ports {c0_ddr4_dq[32]}]
set_property PACKAGE_PIN AJ28 [get_ports {c0_ddr4_dq[33]}]
set_property PACKAGE_PIN AM27 [get_ports {c0_ddr4_dq[34]}]
set_property PACKAGE_PIN AK28 [get_ports {c0_ddr4_dq[35]}]
set_property PACKAGE_PIN AH27 [get_ports {c0_ddr4_dq[36]}]
set_property PACKAGE_PIN AH28 [get_ports {c0_ddr4_dq[37]}]
set_property PACKAGE_PIN AK26 [get_ports {c0_ddr4_dq[38]}]
set_property PACKAGE_PIN AK27 [get_ports {c0_ddr4_dq[39]}]
set_property PACKAGE_PIN AN28 [get_ports {c0_ddr4_dq[40]}]
set_property PACKAGE_PIN AM30 [get_ports {c0_ddr4_dq[41]}]
set_property PACKAGE_PIN AP28 [get_ports {c0_ddr4_dq[42]}]
set_property PACKAGE_PIN AM29 [get_ports {c0_ddr4_dq[43]}]
set_property PACKAGE_PIN AN27 [get_ports {c0_ddr4_dq[44]}]
set_property PACKAGE_PIN AL30 [get_ports {c0_ddr4_dq[45]}]
set_property PACKAGE_PIN AL29 [get_ports {c0_ddr4_dq[46]}]
set_property PACKAGE_PIN AP29 [get_ports {c0_ddr4_dq[47]}]
set_property PACKAGE_PIN AK31 [get_ports {c0_ddr4_dq[48]}]
set_property PACKAGE_PIN AH34 [get_ports {c0_ddr4_dq[49]}]
set_property PACKAGE_PIN AK32 [get_ports {c0_ddr4_dq[50]}]
set_property PACKAGE_PIN AJ31 [get_ports {c0_ddr4_dq[51]}]
set_property PACKAGE_PIN AJ30 [get_ports {c0_ddr4_dq[52]}]
set_property PACKAGE_PIN AH31 [get_ports {c0_ddr4_dq[53]}]
set_property PACKAGE_PIN AJ34 [get_ports {c0_ddr4_dq[54]}]
set_property PACKAGE_PIN AH32 [get_ports {c0_ddr4_dq[55]}]
set_property PACKAGE_PIN AN31 [get_ports {c0_ddr4_dq[56]}]
set_property PACKAGE_PIN AL34 [get_ports {c0_ddr4_dq[57]}]
set_property PACKAGE_PIN AN32 [get_ports {c0_ddr4_dq[58]}]
set_property PACKAGE_PIN AN33 [get_ports {c0_ddr4_dq[59]}]
set_property PACKAGE_PIN AM32 [get_ports {c0_ddr4_dq[60]}]
set_property PACKAGE_PIN AM34 [get_ports {c0_ddr4_dq[61]}]
set_property PACKAGE_PIN AP31 [get_ports {c0_ddr4_dq[62]}]
set_property PACKAGE_PIN AP33 [get_ports {c0_ddr4_dq[63]}]

set_property PACKAGE_PIN AD21 [get_ports {c0_ddr4_dm_dbi_n[0]}]
set_property PACKAGE_PIN AE25 [get_ports {c0_ddr4_dm_dbi_n[1]}]
set_property PACKAGE_PIN AJ21 [get_ports {c0_ddr4_dm_dbi_n[2]}]
set_property PACKAGE_PIN AM21 [get_ports {c0_ddr4_dm_dbi_n[3]}]
set_property PACKAGE_PIN AH26 [get_ports {c0_ddr4_dm_dbi_n[4]}]
set_property PACKAGE_PIN AN26 [get_ports {c0_ddr4_dm_dbi_n[5]}]
set_property PACKAGE_PIN AJ29 [get_ports {c0_ddr4_dm_dbi_n[6]}]
set_property PACKAGE_PIN AL32 [get_ports {c0_ddr4_dm_dbi_n[7]}]

set_property PACKAGE_PIN AG21 [get_ports {c0_ddr4_dqs_t[0]}]
set_property PACKAGE_PIN AH21 [get_ports {c0_ddr4_dqs_c[0]}]
set_property PACKAGE_PIN AH24 [get_ports {c0_ddr4_dqs_t[1]}]
set_property PACKAGE_PIN AJ25 [get_ports {c0_ddr4_dqs_c[1]}]
set_property PACKAGE_PIN AJ20 [get_ports {c0_ddr4_dqs_t[2]}]
set_property PACKAGE_PIN AK20 [get_ports {c0_ddr4_dqs_c[2]}]
set_property PACKAGE_PIN AP20 [get_ports {c0_ddr4_dqs_t[3]}]
set_property PACKAGE_PIN AP21 [get_ports {c0_ddr4_dqs_c[3]}]
set_property PACKAGE_PIN AL27 [get_ports {c0_ddr4_dqs_t[4]}]
set_property PACKAGE_PIN AL28 [get_ports {c0_ddr4_dqs_c[4]}]
set_property PACKAGE_PIN AN29 [get_ports {c0_ddr4_dqs_t[5]}]
set_property PACKAGE_PIN AP30 [get_ports {c0_ddr4_dqs_c[5]}]
set_property PACKAGE_PIN AH33 [get_ports {c0_ddr4_dqs_t[6]}]
set_property PACKAGE_PIN AJ33 [get_ports {c0_ddr4_dqs_c[6]}]
set_property PACKAGE_PIN AN34 [get_ports {c0_ddr4_dqs_t[7]}]
set_property PACKAGE_PIN AP34 [get_ports {c0_ddr4_dqs_c[7]}]

###############################################################################
# DDR4 Address / Command
###############################################################################

set_property PACKAGE_PIN AG14 [get_ports {c0_ddr4_adr[0]}]
set_property PACKAGE_PIN AF17 [get_ports {c0_ddr4_adr[1]}]
set_property PACKAGE_PIN AF15 [get_ports {c0_ddr4_adr[2]}]
set_property PACKAGE_PIN AJ14 [get_ports {c0_ddr4_adr[3]}]
set_property PACKAGE_PIN AD18 [get_ports {c0_ddr4_adr[4]}]
set_property PACKAGE_PIN AG17 [get_ports {c0_ddr4_adr[5]}]
set_property PACKAGE_PIN AE17 [get_ports {c0_ddr4_adr[6]}]
set_property PACKAGE_PIN AK18 [get_ports {c0_ddr4_adr[7]}]
set_property PACKAGE_PIN AD16 [get_ports {c0_ddr4_adr[8]}]
set_property PACKAGE_PIN AH18 [get_ports {c0_ddr4_adr[9]}]
set_property PACKAGE_PIN AD19 [get_ports {c0_ddr4_adr[10]}]
set_property PACKAGE_PIN AD15 [get_ports {c0_ddr4_adr[11]}]
set_property PACKAGE_PIN AH16 [get_ports {c0_ddr4_adr[12]}]
set_property PACKAGE_PIN AL17 [get_ports {c0_ddr4_adr[13]}]
set_property PACKAGE_PIN AL15 [get_ports {c0_ddr4_adr[14]}]
set_property PACKAGE_PIN AL19 [get_ports {c0_ddr4_adr[15]}]
set_property PACKAGE_PIN AM19 [get_ports {c0_ddr4_adr[16]}]

set_property PACKAGE_PIN AG15 [get_ports {c0_ddr4_ba[0]}]
set_property PACKAGE_PIN AL18 [get_ports {c0_ddr4_ba[1]}]
set_property PACKAGE_PIN AJ15 [get_ports {c0_ddr4_bg[0]}]
set_property PACKAGE_PIN AJ16 [get_ports {c0_ddr4_cke[0]}]
set_property PACKAGE_PIN AF18 [get_ports c0_ddr4_act_n]
set_property PACKAGE_PIN AE15 [get_ports {c0_ddr4_ck_c[0]}]
set_property PACKAGE_PIN AE16 [get_ports {c0_ddr4_ck_t[0]}]
set_property PACKAGE_PIN AE18 [get_ports {c0_ddr4_cs_n[0]}]
set_property PACKAGE_PIN AG19 [get_ports {c0_ddr4_odt[0]}]
set_property PACKAGE_PIN AG16 [get_ports c0_ddr4_reset_n]

###############################################################################
# DDR4 Electrical / Interface Constraints
###############################################################################

set _ddr_all_ports [get_ports { \
  c0_ddr4_adr[*] c0_ddr4_act_n c0_ddr4_ba[*] c0_ddr4_bg[*] \
  c0_ddr4_cke[*] c0_ddr4_cs_n[*] c0_ddr4_odt[*] \
  c0_ddr4_ck_t[*] c0_ddr4_ck_c[*] \
  c0_ddr4_dq[*] c0_ddr4_dm_dbi_n[*] c0_ddr4_dqs_t[*] c0_ddr4_dqs_c[*] \
}]

set_property OUTPUT_IMPEDANCE RDRV_40_40 $_ddr_all_ports

set_property IOSTANDARD SSTL12_DCI [get_ports { \
  c0_ddr4_adr[*] c0_ddr4_act_n c0_ddr4_ba[*] c0_ddr4_bg[*] \
  c0_ddr4_cke[*] c0_ddr4_cs_n[*] c0_ddr4_odt[*] \
}]
set_property IOSTANDARD DIFF_SSTL12_DCI [get_ports {c0_ddr4_ck_t[*] c0_ddr4_ck_c[*]}]
set_property IOSTANDARD POD12_DCI [get_ports {c0_ddr4_dq[*] c0_ddr4_dm_dbi_n[*]}]
set_property IOSTANDARD DIFF_POD12_DCI [get_ports {c0_ddr4_dqs_t[*] c0_ddr4_dqs_c[*]}]
set_property IOSTANDARD LVCMOS12 [get_ports c0_ddr4_reset_n]
set_property DRIVE 8 [get_ports c0_ddr4_reset_n]

set_property SLEW FAST $_ddr_all_ports
set_property IBUF_LOW_PWR FALSE [get_ports {c0_ddr4_dq[*] c0_ddr4_dqs_t[*] c0_ddr4_dqs_c[*] c0_ddr4_dm_dbi_n[*]}]
set_property ODT RTT_40 [get_ports {c0_ddr4_dq[*] c0_ddr4_dqs_t[*] c0_ddr4_dqs_c[*] c0_ddr4_dm_dbi_n[*]}]
set_property EQUALIZATION EQ_LEVEL2 [get_ports {c0_ddr4_dq[*] c0_ddr4_dqs_t[*] c0_ddr4_dqs_c[*] c0_ddr4_dm_dbi_n[*]}]
set_property PRE_EMPHASIS RDRV_240 [get_ports {c0_ddr4_dq[*] c0_ddr4_dqs_t[*] c0_ddr4_dqs_c[*] c0_ddr4_dm_dbi_n[*]}]
set_property DATA_RATE SDR [get_ports { \
  c0_ddr4_adr[*] c0_ddr4_act_n c0_ddr4_ba[*] c0_ddr4_bg[*] \
  c0_ddr4_cke[*] c0_ddr4_cs_n[*] c0_ddr4_odt[*] \
}]
set_property DATA_RATE DDR [get_ports { \
  c0_ddr4_ck_t[*] c0_ddr4_ck_c[*] \
  c0_ddr4_dq[*] c0_ddr4_dm_dbi_n[*] c0_ddr4_dqs_t[*] c0_ddr4_dqs_c[*] \
}]

create_interface -quiet interface_ddr4_axi_0
set_property interface interface_ddr4_axi_0 [get_ports { \
  c0_ddr4_adr[*] c0_ddr4_act_n c0_ddr4_ba[*] c0_ddr4_bg[*] \
  c0_ddr4_cke[*] c0_ddr4_cs_n[*] c0_ddr4_odt[*] \
  c0_ddr4_ck_t[*] c0_ddr4_ck_c[*] c0_ddr4_reset_n \
  c0_ddr4_dq[*] c0_ddr4_dm_dbi_n[*] c0_ddr4_dqs_t[*] c0_ddr4_dqs_c[*] \
}]
