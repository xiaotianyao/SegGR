//Copyright 1986-2018 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2018.2 (lin64) Build 2258646 Thu Jun 14 20:02:38 MDT 2018
//Date        : Thu Oct  8 20:15:10 2026
//Host        : bn-ws running 64-bit Ubuntu 22.04.5 LTS
//Command     : generate_target pcie_wrapper.bd
//Design      : pcie_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module pcie_wrapper
   (addr_bram_i,
    clk_axi_o,
    clk_bram_i,
    clk_pcie_i_clk_n,
    clk_pcie_i_clk_p,
    clk_sys_buf_o,
    clk_sys_i_clk_n,
    clk_sys_i_clk_p,
    din_bram_i,
    dout_bram_o,
    en_bram_i,
    pcie_7x_mgt_rtl_rxn,
    pcie_7x_mgt_rtl_rxp,
    pcie_7x_mgt_rtl_txn,
    pcie_7x_mgt_rtl_txp,
    peripheral_aresetn_o,
    rst_bram_i,
    rstn_pcie_i,
    user_lnk_up_o,
    user_rstn_o,
    usr_irq_ack,
    usr_irq_req,
    we_bram_i);
  input [31:0]addr_bram_i;
  output clk_axi_o;
  input clk_bram_i;
  input [0:0]clk_pcie_i_clk_n;
  input [0:0]clk_pcie_i_clk_p;
  output clk_sys_buf_o;
  input clk_sys_i_clk_n;
  input clk_sys_i_clk_p;
  input [511:0]din_bram_i;
  output [511:0]dout_bram_o;
  input en_bram_i;
  input [3:0]pcie_7x_mgt_rtl_rxn;
  input [3:0]pcie_7x_mgt_rtl_rxp;
  output [3:0]pcie_7x_mgt_rtl_txn;
  output [3:0]pcie_7x_mgt_rtl_txp;
  output [0:0]peripheral_aresetn_o;
  input rst_bram_i;
  input rstn_pcie_i;
  output user_lnk_up_o;
  output user_rstn_o;
  output [0:0]usr_irq_ack;
  input usr_irq_req;
  input [63:0]we_bram_i;

  wire [31:0]addr_bram_i;
  wire clk_axi_o;
  wire clk_bram_i;
  wire [0:0]clk_pcie_i_clk_n;
  wire [0:0]clk_pcie_i_clk_p;
  wire clk_sys_buf_o;
  wire clk_sys_i_clk_n;
  wire clk_sys_i_clk_p;
  wire [511:0]din_bram_i;
  wire [511:0]dout_bram_o;
  wire en_bram_i;
  wire [3:0]pcie_7x_mgt_rtl_rxn;
  wire [3:0]pcie_7x_mgt_rtl_rxp;
  wire [3:0]pcie_7x_mgt_rtl_txn;
  wire [3:0]pcie_7x_mgt_rtl_txp;
  wire [0:0]peripheral_aresetn_o;
  wire rst_bram_i;
  wire rstn_pcie_i;
  wire user_lnk_up_o;
  wire user_rstn_o;
  wire [0:0]usr_irq_ack;
  wire usr_irq_req;
  wire [63:0]we_bram_i;

  pcie pcie_i
       (.addr_bram_i(addr_bram_i),
        .clk_axi_o(clk_axi_o),
        .clk_bram_i(clk_bram_i),
        .clk_pcie_i_clk_n(clk_pcie_i_clk_n),
        .clk_pcie_i_clk_p(clk_pcie_i_clk_p),
        .clk_sys_buf_o(clk_sys_buf_o),
        .clk_sys_i_clk_n(clk_sys_i_clk_n),
        .clk_sys_i_clk_p(clk_sys_i_clk_p),
        .din_bram_i(din_bram_i),
        .dout_bram_o(dout_bram_o),
        .en_bram_i(en_bram_i),
        .pcie_7x_mgt_rtl_rxn(pcie_7x_mgt_rtl_rxn),
        .pcie_7x_mgt_rtl_rxp(pcie_7x_mgt_rtl_rxp),
        .pcie_7x_mgt_rtl_txn(pcie_7x_mgt_rtl_txn),
        .pcie_7x_mgt_rtl_txp(pcie_7x_mgt_rtl_txp),
        .peripheral_aresetn_o(peripheral_aresetn_o),
        .rst_bram_i(rst_bram_i),
        .rstn_pcie_i(rstn_pcie_i),
        .user_lnk_up_o(user_lnk_up_o),
        .user_rstn_o(user_rstn_o),
        .usr_irq_ack(usr_irq_ack),
        .usr_irq_req(usr_irq_req),
        .we_bram_i(we_bram_i));
endmodule
