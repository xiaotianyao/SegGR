`ifndef FPGA_ROUTER_WRAPPER_SV
`define FPGA_ROUTER_WRAPPER_SV

`include "../common/segcore_pkg.sv"

module router_wrapper #(
  parameter int CORE_COUNT = 4,
  parameter int CHUNK_SIZE = 64,
  parameter int CORE_COUNT_PER_FIFO = 1,
  parameter int SEGMENT_PER_FIFO = 1,
  parameter int DATA_W = `SEGCORE_DATA_W,
  parameter int FRAC_W = `SEGCORE_FRAC_W,
  parameter int COEFF_W = `SEGCORE_COEFF_W,
  parameter int COEFF_FRAC_W = `SEGCORE_COEFF_FRAC_W,
  parameter int P_DATA_W = 16,
  parameter int P_FRAC_W = 15,
  parameter int ACC_W = 64,
  parameter int MEM_GRAD_P_W = 64,
  parameter int RMS_V_W = 64,
  parameter int RMS_V_FRAC_W = (2 * FRAC_W),
  parameter logic [DATA_W-1:0] RMSPROP_ALPHA_FXP = 32'd64881,
  parameter logic [DATA_W-1:0] RMSPROP_EPS_FXP = 32'd43,
  parameter int GROUP_FLAG_WORD_W = 32,
  parameter int BANK_DEPTH = 65536,
  // Blocked-GRADP v2: see router.sv / blocked_ctrl.sv
  // FPGA target builds blocked (v2); sim overrides via TB_* defines.
  parameter bit BLOCKED_GRADP = 1'b1,
  parameter bit BLOCKED_SCHED = 1'b1,
  parameter int WIN_ROWS = 16384,
  parameter int P_REGIONS = 64,
  parameter int DISPATCHER_FIFO_DEPTH = 512,
  parameter int GRAD_OVERFLOW_FIFO_DEPTH = 512,
  parameter int GRAD_WLVIA_FIFO_DEPTH = 512,
  parameter bit NATIVE_QUERY_ENABLE = 1'b1,
  parameter int QUERY_PREFETCH_FIFO_DEPTH = 16,
  parameter int QUERY_ACT_SCALE_FXP = (1 << (FRAC_W - 1)),
  parameter int SOFTMAX_GROUP_DESC_FIFO_DEPTH = 4096,
  parameter int ACT_GROUP = 8,
  parameter int ACT_PIPELINE = 2,
  parameter int PRE_ACT_IDLE_CYCLES = 0,
  parameter int POST_ACT_IDLE_CYCLES = ACT_PIPELINE + 2,
  parameter int SOFTMAX_AXI_BURST_LINES = 64,
  parameter int CMD_BRAM_DATA_W = 512,
  parameter int CMD_BRAM_DEPTH = 4096,
  parameter int CMD_BRAM_ADDR_W = 12,
  parameter int DDR_AXI_ID_W = 4,
  parameter int DDR_AXI_ADDR_W = 32,
  parameter int DDR_AXI_DATA_W = 512,
  parameter logic [DDR_AXI_ADDR_W-1:0] LOGITS_DRAM_BASE_ADDR = 32'h0000_0000,
  parameter logic [DDR_AXI_ADDR_W-1:0] RMS_DRAM_BASE_ADDR = 32'h0400_0000
) (
  input  logic                         sys_rst_n,

  input  logic                         rstn_pcie_i,
  input  logic                         clk_pcie_n,
  input  logic                         clk_pcie_p,
  input  logic [0:0]                   sys_clk_diff_i_clk_n,
  input  logic [0:0]                   sys_clk_diff_i_clk_p,
  input  logic [3:0]                   pcie_7x_mgt_rtl_0_rxn,
  input  logic [3:0]                   pcie_7x_mgt_rtl_0_rxp,
  output logic [3:0]                   pcie_7x_mgt_rtl_0_txn,
  output logic [3:0]                   pcie_7x_mgt_rtl_0_txp,

`ifdef COCOTB_SIM
  output logic                         sim_cmd_bram_en_o,
  output logic [(CMD_BRAM_DATA_W/8)-1:0] sim_cmd_bram_we_o,
  output logic [CMD_BRAM_ADDR_W-1:0]   sim_cmd_bram_addr_o,
  output logic [CMD_BRAM_DATA_W-1:0]   sim_cmd_bram_wdata_o,
  input  logic [CMD_BRAM_DATA_W-1:0]   sim_cmd_bram_rdata_i,
  output logic [0:0]                   sim_usr_irq_req_o,
  input  logic [0:0]                   sim_usr_irq_ack_i,
`endif

  output logic                         router_busy_o,
  output logic                         ddr_calib_done_o,
  output logic                         test_led_sys,
  output logic                         test_led_pcie,

  (* IO_BUFFER_TYPE = "NONE" *) output wire                          c0_ddr4_act_n,
  (* IO_BUFFER_TYPE = "NONE" *) output wire  [16:0]                  c0_ddr4_adr,
  (* IO_BUFFER_TYPE = "NONE" *) output wire  [1:0]                   c0_ddr4_ba,
  (* IO_BUFFER_TYPE = "NONE" *) output wire  [0:0]                   c0_ddr4_bg,
  (* IO_BUFFER_TYPE = "NONE" *) output wire  [0:0]                   c0_ddr4_cke,
  (* IO_BUFFER_TYPE = "NONE" *) output wire  [0:0]                   c0_ddr4_odt,
  (* IO_BUFFER_TYPE = "NONE" *) output wire  [0:0]                   c0_ddr4_cs_n,
  (* IO_BUFFER_TYPE = "NONE" *) output wire  [0:0]                   c0_ddr4_ck_t,
  (* IO_BUFFER_TYPE = "NONE" *) output wire  [0:0]                   c0_ddr4_ck_c,
  (* IO_BUFFER_TYPE = "NONE" *) output wire                          c0_ddr4_reset_n,
  (* IO_BUFFER_TYPE = "NONE" *) inout  wire  [7:0]                   c0_ddr4_dm_dbi_n,
  (* IO_BUFFER_TYPE = "NONE" *) inout  wire  [63:0]                  c0_ddr4_dq,
  (* IO_BUFFER_TYPE = "NONE" *) inout  wire  [7:0]                   c0_ddr4_dqs_c,
  (* IO_BUFFER_TYPE = "NONE" *) inout  wire  [7:0]                   c0_ddr4_dqs_t
);
  localparam int GRAD_OVERFLOW_COUNT = CORE_COUNT / CORE_COUNT_PER_FIFO;
  localparam int GRAD_WLVIA_COUNT = CORE_COUNT / SEGMENT_PER_FIFO;
  localparam int BANK_COUNT = GRAD_OVERFLOW_COUNT + GRAD_WLVIA_COUNT;
  localparam int CMD_BRAM_STRB_W = CMD_BRAM_DATA_W / 8;
  localparam int DDR_AXI_STRB_W = DDR_AXI_DATA_W / 8;

  initial begin
    assert (CORE_COUNT_PER_FIFO == 1)
      else $error(1, "router_wrapper: CORE_COUNT_PER_FIFO must be 1");
    assert (SEGMENT_PER_FIFO == 1)
      else $error(1, "router_wrapper: SEGMENT_PER_FIFO must be 1");
  end

  logic cmd_bram_en;
  logic [CMD_BRAM_STRB_W-1:0] cmd_bram_we;
  logic [CMD_BRAM_ADDR_W-1:0] cmd_bram_addr;
  logic [CMD_BRAM_DATA_W-1:0] cmd_bram_wdata;
  logic [CMD_BRAM_DATA_W-1:0] cmd_bram_rdata;
  logic [31:0] pcie_bram_byte_addr;
  logic [0:0] usr_irq_req;
  logic [0:0] usr_irq_ack;
  logic [0:0] usr_irq_ack_pcie;
  (* ASYNC_REG = "TRUE" *) logic usr_irq_req_pcie_meta_q;
  (* ASYNC_REG = "TRUE" *) logic usr_irq_req_pcie_sync_q;
  logic usr_irq_ack_pcie_hold_q;
  (* ASYNC_REG = "TRUE" *) logic [1:0] usr_irq_ack_router_sync_q;
  logic ui_clk;
  logic ui_rst;
  logic router_clk;
  logic router_rst;
  logic [3:0] router_reset_pipe_q = 4'hf;
  logic pcie_clk_axi;
  logic pcie_peripheral_aresetn;
  logic pcie_user_lnk_up;
  logic pcie_user_rstn;
  logic sys_refclk_ibuf_200m;
  logic ddr_ref_clk;
  logic ddr_sys_rst;
  logic ui_aresetn;
  logic calib_done;
  logic dbg_clk_unused;
  logic [511:0] dbg_bus_unused;
  logic router_ui_clk_unused;
  logic router_ui_rst_unused;
  logic [15:0] ddr_startup_cnt_q = '0;
  logic [7:0] router_error_unused;
  logic grad_overflow_fifo_warning [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic grad_overflow_host_ready [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic grad_wlvia_fifo_warning [0:GRAD_WLVIA_COUNT-1];
  logic grad_p_finish;
  logic router_usr_irq;
  logic softmax_enable;
  logic softmax_finish [0:BANK_COUNT-1];
  logic ddr_rd_req_valid;
  logic ddr_rd_req_ready;
  logic [19:0] ddr_rd_req_row;
  logic [$clog2((8 * ((SOFTMAX_AXI_BURST_LINES < 1) ? 1 : ((SOFTMAX_AXI_BURST_LINES > 256) ? 256 : SOFTMAX_AXI_BURST_LINES))) + 1)-1:0] ddr_rd_req_count;
  logic ddr_rd_resp_valid;
  logic ddr_rd_resp_ready;
  logic ddr_wr_valid;
  logic ddr_wr_ready;
  logic [19:0] ddr_wr_row;
  logic [$clog2((16 * (8 * ((SOFTMAX_AXI_BURST_LINES < 1) ? 1 : ((SOFTMAX_AXI_BURST_LINES > 256) ? 256 : SOFTMAX_AXI_BURST_LINES)))) + 1)-1:0] ddr_wr_count;
  logic [3:0] bridge_state;
  logic [2:0] bridge_mode;
  logic [$clog2(BANK_COUNT + 1)-1:0] bridge_bank;
  logic [19:0] bridge_row;
  logic bridge_target_is_rms;
  logic bridge_ddr_active;
  logic [DDR_AXI_ID_W-1:0] s_axi_awid;
  logic [DDR_AXI_ADDR_W-1:0] s_axi_awaddr;
  logic [7:0] s_axi_awlen;
  logic [2:0] s_axi_awsize;
  logic [1:0] s_axi_awburst;
  logic [0:0] s_axi_awlock;
  logic [3:0] s_axi_awcache;
  logic [2:0] s_axi_awprot;
  logic [3:0] s_axi_awqos;
  logic s_axi_awvalid;
  logic s_axi_awready;
  logic [DDR_AXI_DATA_W-1:0] s_axi_wdata;
  logic [DDR_AXI_STRB_W-1:0] s_axi_wstrb;
  logic s_axi_wlast;
  logic s_axi_wvalid;
  logic s_axi_wready;
  logic s_axi_bready;
  logic [DDR_AXI_ID_W-1:0] s_axi_bid;
  logic [1:0] s_axi_bresp;
  logic s_axi_bvalid;
  logic [DDR_AXI_ID_W-1:0] s_axi_arid;
  logic [DDR_AXI_ADDR_W-1:0] s_axi_araddr;
  logic [7:0] s_axi_arlen;
  logic [2:0] s_axi_arsize;
  logic [1:0] s_axi_arburst;
  logic [0:0] s_axi_arlock;
  logic [3:0] s_axi_arcache;
  logic [2:0] s_axi_arprot;
  logic [3:0] s_axi_arqos;
  logic s_axi_arvalid;
  logic s_axi_arready;
  logic s_axi_rready;
  logic [DDR_AXI_ID_W-1:0] s_axi_rid;
  logic [DDR_AXI_DATA_W-1:0] s_axi_rdata;
  logic [1:0] s_axi_rresp;
  logic s_axi_rlast;
  logic s_axi_rvalid;
  logic [DDR_AXI_ID_W-1:0] router_axi_awid;
  logic [DDR_AXI_ADDR_W-1:0] router_axi_awaddr;
  logic [7:0] router_axi_awlen;
  logic [2:0] router_axi_awsize;
  logic [1:0] router_axi_awburst;
  logic [0:0] router_axi_awlock;
  logic [3:0] router_axi_awcache;
  logic [2:0] router_axi_awprot;
  logic [3:0] router_axi_awqos;
  logic router_axi_awvalid;
  logic router_axi_awready;
  logic [DDR_AXI_DATA_W-1:0] router_axi_wdata;
  logic [DDR_AXI_STRB_W-1:0] router_axi_wstrb;
  logic router_axi_wlast;
  logic router_axi_wvalid;
  logic router_axi_wready;
  logic router_axi_bready;
  logic [DDR_AXI_ID_W-1:0] router_axi_bid;
  logic [1:0] router_axi_bresp;
  logic router_axi_bvalid;
  logic [DDR_AXI_ID_W-1:0] router_axi_arid;
  logic [DDR_AXI_ADDR_W-1:0] router_axi_araddr;
  logic [7:0] router_axi_arlen;
  logic [2:0] router_axi_arsize;
  logic [1:0] router_axi_arburst;
  logic [0:0] router_axi_arlock;
  logic [3:0] router_axi_arcache;
  logic [2:0] router_axi_arprot;
  logic [3:0] router_axi_arqos;
  logic router_axi_arvalid;
  logic router_axi_arready;
  logic router_axi_rready;
  logic [DDR_AXI_ID_W-1:0] router_axi_rid;
  logic [DDR_AXI_DATA_W-1:0] router_axi_rdata;
  logic [1:0] router_axi_rresp;
  logic router_axi_rlast;
  logic router_axi_rvalid;
  logic [3:0] axi_awregion_unused;
  logic [3:0] axi_arregion_unused;
  logic [26:0] sys_led_div;
  logic [25:0] pcie_led_div;

  assign pcie_bram_byte_addr = {14'd0, cmd_bram_addr, 6'b0};
  assign ddr_sys_rst = (!sys_rst_n) || (!(&ddr_startup_cnt_q));
  assign ui_aresetn = !ui_rst;
  assign router_rst = router_reset_pipe_q[3];
  assign test_led_sys = sys_led_div[26];
  assign test_led_pcie = pcie_led_div[25];

  IBUFDS #(
    .IBUF_LOW_PWR("FALSE")
  ) u_ddr_refclk_ibuf (
    .I(sys_clk_diff_i_clk_p[0]),
    .IB(sys_clk_diff_i_clk_n[0]),
    .O(sys_refclk_ibuf_200m)
  );

  BUFG u_ddr_refclk_bufg (
    .I(sys_refclk_ibuf_200m),
    .O(ddr_ref_clk)
  );

  BUFGCE_DIV #(
    .BUFGCE_DIVIDE(3),
    .IS_CE_INVERTED(1'b0),
    .IS_CLR_INVERTED(1'b0),
    .IS_I_INVERTED(1'b0)
  ) u_router_clk_div (
    .I(sys_refclk_ibuf_200m),
    .CE(1'b1),
    .CLR(1'b0),
    .O(router_clk) // 66.7MHz
  );

  always_ff @(posedge router_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
      router_reset_pipe_q <= 4'hf;
    end else begin
      router_reset_pipe_q <= {router_reset_pipe_q[2:0], 1'b0};
    end
  end

  always_ff @(posedge ddr_ref_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
      ddr_startup_cnt_q <= '0;
    end else if (!(&ddr_startup_cnt_q)) begin
      ddr_startup_cnt_q <= ddr_startup_cnt_q + 16'd1;
    end
  end

  ddr4_axi_0 u_ddr4_axi (
    .sys_rst(ddr_sys_rst),
    .c0_sys_clk_i(ddr_ref_clk),
    .c0_ddr4_act_n(c0_ddr4_act_n),
    .c0_ddr4_adr(c0_ddr4_adr),
    .c0_ddr4_ba(c0_ddr4_ba),
    .c0_ddr4_bg(c0_ddr4_bg),
    .c0_ddr4_cke(c0_ddr4_cke),
    .c0_ddr4_odt(c0_ddr4_odt),
    .c0_ddr4_cs_n(c0_ddr4_cs_n),
    .c0_ddr4_ck_t(c0_ddr4_ck_t),
    .c0_ddr4_ck_c(c0_ddr4_ck_c),
    .c0_ddr4_reset_n(c0_ddr4_reset_n),
    .c0_ddr4_dm_dbi_n(c0_ddr4_dm_dbi_n),
    .c0_ddr4_dq(c0_ddr4_dq),
    .c0_ddr4_dqs_c(c0_ddr4_dqs_c),
    .c0_ddr4_dqs_t(c0_ddr4_dqs_t),
    .c0_init_calib_complete(calib_done),
    .c0_ddr4_ui_clk(ui_clk),
    .c0_ddr4_ui_clk_sync_rst(ui_rst),
    .dbg_clk(dbg_clk_unused),
    .c0_ddr4_aresetn(ui_aresetn),
    .c0_ddr4_s_axi_awid(s_axi_awid),
    .c0_ddr4_s_axi_awaddr(s_axi_awaddr),
    .c0_ddr4_s_axi_awlen(s_axi_awlen),
    .c0_ddr4_s_axi_awsize(s_axi_awsize),
    .c0_ddr4_s_axi_awburst(s_axi_awburst),
    .c0_ddr4_s_axi_awlock(s_axi_awlock),
    .c0_ddr4_s_axi_awcache(s_axi_awcache),
    .c0_ddr4_s_axi_awprot(s_axi_awprot),
    .c0_ddr4_s_axi_awqos(s_axi_awqos),
    .c0_ddr4_s_axi_awvalid(s_axi_awvalid),
    .c0_ddr4_s_axi_awready(s_axi_awready),
    .c0_ddr4_s_axi_wdata(s_axi_wdata),
    .c0_ddr4_s_axi_wstrb(s_axi_wstrb),
    .c0_ddr4_s_axi_wlast(s_axi_wlast),
    .c0_ddr4_s_axi_wvalid(s_axi_wvalid),
    .c0_ddr4_s_axi_wready(s_axi_wready),
    .c0_ddr4_s_axi_bready(s_axi_bready),
    .c0_ddr4_s_axi_bid(s_axi_bid),
    .c0_ddr4_s_axi_bresp(s_axi_bresp),
    .c0_ddr4_s_axi_bvalid(s_axi_bvalid),
    .c0_ddr4_s_axi_arid(s_axi_arid),
    .c0_ddr4_s_axi_araddr(s_axi_araddr),
    .c0_ddr4_s_axi_arlen(s_axi_arlen),
    .c0_ddr4_s_axi_arsize(s_axi_arsize),
    .c0_ddr4_s_axi_arburst(s_axi_arburst),
    .c0_ddr4_s_axi_arlock(s_axi_arlock),
    .c0_ddr4_s_axi_arcache(s_axi_arcache),
    .c0_ddr4_s_axi_arprot(s_axi_arprot),
    .c0_ddr4_s_axi_arqos(s_axi_arqos),
    .c0_ddr4_s_axi_arvalid(s_axi_arvalid),
    .c0_ddr4_s_axi_arready(s_axi_arready),
    .c0_ddr4_s_axi_rready(s_axi_rready),
    .c0_ddr4_s_axi_rid(s_axi_rid),
    .c0_ddr4_s_axi_rdata(s_axi_rdata),
    .c0_ddr4_s_axi_rresp(s_axi_rresp),
    .c0_ddr4_s_axi_rlast(s_axi_rlast),
    .c0_ddr4_s_axi_rvalid(s_axi_rvalid),
    .dbg_bus(dbg_bus_unused)
  );

  router #(
    .CORE_COUNT(CORE_COUNT),
    .CHUNK_SIZE(CHUNK_SIZE),
    .CORE_COUNT_PER_FIFO(CORE_COUNT_PER_FIFO),
    .SEGMENT_PER_FIFO(SEGMENT_PER_FIFO),
    .DATA_W(DATA_W),
    .FRAC_W(FRAC_W),
    .COEFF_W(COEFF_W),
    .COEFF_FRAC_W(COEFF_FRAC_W),
    .P_DATA_W(P_DATA_W),
    .P_FRAC_W(P_FRAC_W),
    .ACC_W(ACC_W),
    .MEM_GRAD_P_W(MEM_GRAD_P_W),
    .RMS_V_W(RMS_V_W),
    .RMS_V_FRAC_W(RMS_V_FRAC_W),
    .RMSPROP_ALPHA_FXP(RMSPROP_ALPHA_FXP),
    .RMSPROP_EPS_FXP(RMSPROP_EPS_FXP),
    .GROUP_FLAG_WORD_W(GROUP_FLAG_WORD_W),
    .BANK_DEPTH(BANK_DEPTH),
    .BLOCKED_GRADP(BLOCKED_GRADP),
    .BLOCKED_SCHED(BLOCKED_SCHED),
    .WIN_ROWS(WIN_ROWS),
    .P_REGIONS(P_REGIONS),
    .DISPATCHER_FIFO_DEPTH(DISPATCHER_FIFO_DEPTH),
    .GRAD_OVERFLOW_FIFO_DEPTH(GRAD_OVERFLOW_FIFO_DEPTH),
    .GRAD_WLVIA_FIFO_DEPTH(GRAD_WLVIA_FIFO_DEPTH),
    .NATIVE_QUERY_ENABLE(NATIVE_QUERY_ENABLE),
    .QUERY_PREFETCH_FIFO_DEPTH(QUERY_PREFETCH_FIFO_DEPTH),
    .QUERY_ACT_SCALE_FXP(QUERY_ACT_SCALE_FXP),
    .SOFTMAX_GROUP_DESC_FIFO_DEPTH(SOFTMAX_GROUP_DESC_FIFO_DEPTH),
    .ACT_GROUP(ACT_GROUP),
    .ACT_PIPELINE(ACT_PIPELINE),
    .PRE_ACT_IDLE_CYCLES(PRE_ACT_IDLE_CYCLES),
    .POST_ACT_IDLE_CYCLES(POST_ACT_IDLE_CYCLES),
    .SOFTMAX_AXI_BURST_LINES(SOFTMAX_AXI_BURST_LINES),
    .PCIE_BRAM_ENABLE(1'b1),
    .CMD_BRAM_DATA_W(CMD_BRAM_DATA_W),
    .CMD_BRAM_DEPTH(CMD_BRAM_DEPTH),
    .CMD_BRAM_ADDR_W(CMD_BRAM_ADDR_W),
    .DDR_AXI_ID_W(DDR_AXI_ID_W),
    .DDR_AXI_ADDR_W(DDR_AXI_ADDR_W),
    .DDR_AXI_DATA_W(DDR_AXI_DATA_W),
    .LOGITS_DRAM_BASE_ADDR(LOGITS_DRAM_BASE_ADDR),
    .RMS_DRAM_BASE_ADDR(RMS_DRAM_BASE_ADDR)
  )
`ifdef COCOTB_SIM
  u_router_core (
`else
  u_router (
`endif
    .sys_rst_n(sys_rst_n),
    .router_clk_i(router_clk),
    .router_rst_i(router_rst),
    .ddr_ui_rst_i(ui_rst),
    .ddr_calib_done_i(calib_done),
    .cmd_bram_en_o(cmd_bram_en),
    .cmd_bram_we_o(cmd_bram_we),
    .cmd_bram_addr_o(cmd_bram_addr),
    .cmd_bram_wdata_o(cmd_bram_wdata),
    .cmd_bram_rdata_i(cmd_bram_rdata),
    .usr_irq_req_o(usr_irq_req),
    .usr_irq_ack_i(usr_irq_ack),
    .router_busy_o(router_busy_o),
    .router_error_o(router_error_unused),
    .grad_overflow_fifo_warning_o(grad_overflow_fifo_warning),
    .grad_overflow_host_ready_o(grad_overflow_host_ready),
    .grad_wlvia_fifo_warning_o(grad_wlvia_fifo_warning),
    .grad_p_finish_o(grad_p_finish),
    .usr_irq_o(router_usr_irq),
    .softmax_enable_o(softmax_enable),
    .softmax_finish_o(softmax_finish),
    .ui_clk_o(router_ui_clk_unused),
    .ui_rst_o(router_ui_rst_unused),
    .ddr_calib_done_o(ddr_calib_done_o),
    .s_axi_awid(router_axi_awid),
    .s_axi_awaddr(router_axi_awaddr),
    .s_axi_awlen(router_axi_awlen),
    .s_axi_awsize(router_axi_awsize),
    .s_axi_awburst(router_axi_awburst),
    .s_axi_awlock(router_axi_awlock),
    .s_axi_awcache(router_axi_awcache),
    .s_axi_awprot(router_axi_awprot),
    .s_axi_awqos(router_axi_awqos),
    .s_axi_awvalid(router_axi_awvalid),
    .s_axi_awready(router_axi_awready),
    .s_axi_wdata(router_axi_wdata),
    .s_axi_wstrb(router_axi_wstrb),
    .s_axi_wlast(router_axi_wlast),
    .s_axi_wvalid(router_axi_wvalid),
    .s_axi_wready(router_axi_wready),
    .s_axi_bready(router_axi_bready),
    .s_axi_bid(router_axi_bid),
    .s_axi_bresp(router_axi_bresp),
    .s_axi_bvalid(router_axi_bvalid),
    .s_axi_arid(router_axi_arid),
    .s_axi_araddr(router_axi_araddr),
    .s_axi_arlen(router_axi_arlen),
    .s_axi_arsize(router_axi_arsize),
    .s_axi_arburst(router_axi_arburst),
    .s_axi_arlock(router_axi_arlock),
    .s_axi_arcache(router_axi_arcache),
    .s_axi_arprot(router_axi_arprot),
    .s_axi_arqos(router_axi_arqos),
    .s_axi_arvalid(router_axi_arvalid),
    .s_axi_arready(router_axi_arready),
    .s_axi_rready(router_axi_rready),
    .s_axi_rid(router_axi_rid),
    .s_axi_rdata(router_axi_rdata),
    .s_axi_rresp(router_axi_rresp),
    .s_axi_rlast(router_axi_rlast),
    .s_axi_rvalid(router_axi_rvalid),
    .ddr_rd_req_valid_o(ddr_rd_req_valid),
    .ddr_rd_req_ready_o(ddr_rd_req_ready),
    .ddr_rd_req_row_o(ddr_rd_req_row),
    .ddr_rd_req_count_o(ddr_rd_req_count),
    .ddr_rd_resp_valid_o(ddr_rd_resp_valid),
    .ddr_rd_resp_ready_o(ddr_rd_resp_ready),
    .ddr_wr_valid_o(ddr_wr_valid),
    .ddr_wr_ready_o(ddr_wr_ready),
    .ddr_wr_row_o(ddr_wr_row),
    .ddr_wr_count_o(ddr_wr_count),
    .bridge_state_o(bridge_state),
    .bridge_mode_o(bridge_mode),
    .bridge_bank_o(bridge_bank),
    .bridge_row_o(bridge_row),
    .bridge_target_is_rms_o(bridge_target_is_rms),
    .bridge_ddr_active_o(bridge_ddr_active)
  );

  router_ddr_axi_clock_converter u_router_ddr_axi_clock_converter (
    .s_axi_aclk(router_clk),
    .s_axi_aresetn(!router_rst),
    .s_axi_awid(router_axi_awid),
    .s_axi_awaddr(router_axi_awaddr),
    .s_axi_awlen(router_axi_awlen),
    .s_axi_awsize(router_axi_awsize),
    .s_axi_awburst(router_axi_awburst),
    .s_axi_awlock(router_axi_awlock),
    .s_axi_awcache(router_axi_awcache),
    .s_axi_awprot(router_axi_awprot),
    .s_axi_awregion(4'b0000),
    .s_axi_awqos(router_axi_awqos),
    .s_axi_awvalid(router_axi_awvalid),
    .s_axi_awready(router_axi_awready),
    .s_axi_wdata(router_axi_wdata),
    .s_axi_wstrb(router_axi_wstrb),
    .s_axi_wlast(router_axi_wlast),
    .s_axi_wvalid(router_axi_wvalid),
    .s_axi_wready(router_axi_wready),
    .s_axi_bid(router_axi_bid),
    .s_axi_bresp(router_axi_bresp),
    .s_axi_bvalid(router_axi_bvalid),
    .s_axi_bready(router_axi_bready),
    .s_axi_arid(router_axi_arid),
    .s_axi_araddr(router_axi_araddr),
    .s_axi_arlen(router_axi_arlen),
    .s_axi_arsize(router_axi_arsize),
    .s_axi_arburst(router_axi_arburst),
    .s_axi_arlock(router_axi_arlock),
    .s_axi_arcache(router_axi_arcache),
    .s_axi_arprot(router_axi_arprot),
    .s_axi_arregion(4'b0000),
    .s_axi_arqos(router_axi_arqos),
    .s_axi_arvalid(router_axi_arvalid),
    .s_axi_arready(router_axi_arready),
    .s_axi_rid(router_axi_rid),
    .s_axi_rdata(router_axi_rdata),
    .s_axi_rresp(router_axi_rresp),
    .s_axi_rlast(router_axi_rlast),
    .s_axi_rvalid(router_axi_rvalid),
    .s_axi_rready(router_axi_rready),
    .m_axi_aclk(ui_clk),
    .m_axi_aresetn(ui_aresetn),
    .m_axi_awid(s_axi_awid),
    .m_axi_awaddr(s_axi_awaddr),
    .m_axi_awlen(s_axi_awlen),
    .m_axi_awsize(s_axi_awsize),
    .m_axi_awburst(s_axi_awburst),
    .m_axi_awlock(s_axi_awlock),
    .m_axi_awcache(s_axi_awcache),
    .m_axi_awprot(s_axi_awprot),
    .m_axi_awregion(axi_awregion_unused),
    .m_axi_awqos(s_axi_awqos),
    .m_axi_awvalid(s_axi_awvalid),
    .m_axi_awready(s_axi_awready),
    .m_axi_wdata(s_axi_wdata),
    .m_axi_wstrb(s_axi_wstrb),
    .m_axi_wlast(s_axi_wlast),
    .m_axi_wvalid(s_axi_wvalid),
    .m_axi_wready(s_axi_wready),
    .m_axi_bid(s_axi_bid),
    .m_axi_bresp(s_axi_bresp),
    .m_axi_bvalid(s_axi_bvalid),
    .m_axi_bready(s_axi_bready),
    .m_axi_arid(s_axi_arid),
    .m_axi_araddr(s_axi_araddr),
    .m_axi_arlen(s_axi_arlen),
    .m_axi_arsize(s_axi_arsize),
    .m_axi_arburst(s_axi_arburst),
    .m_axi_arlock(s_axi_arlock),
    .m_axi_arcache(s_axi_arcache),
    .m_axi_arprot(s_axi_arprot),
    .m_axi_arregion(axi_arregion_unused),
    .m_axi_arqos(s_axi_arqos),
    .m_axi_arvalid(s_axi_arvalid),
    .m_axi_arready(s_axi_arready),
    .m_axi_rid(s_axi_rid),
    .m_axi_rdata(s_axi_rdata),
    .m_axi_rresp(s_axi_rresp),
    .m_axi_rlast(s_axi_rlast),
    .m_axi_rvalid(s_axi_rvalid),
    .m_axi_rready(s_axi_rready)
  );

`ifndef COCOTB_SIM
  always_ff @(posedge pcie_clk_axi or negedge pcie_user_rstn) begin
    if (!pcie_user_rstn) begin
      usr_irq_req_pcie_meta_q <= 1'b0;
      usr_irq_req_pcie_sync_q <= 1'b0;
      usr_irq_ack_pcie_hold_q <= 1'b0;
    end else begin
      usr_irq_req_pcie_meta_q <= usr_irq_req[0];
      usr_irq_req_pcie_sync_q <= usr_irq_req_pcie_meta_q;
      if (usr_irq_ack_pcie[0]) begin
        usr_irq_ack_pcie_hold_q <= 1'b1;
      end else if (!usr_irq_req_pcie_sync_q) begin
        usr_irq_ack_pcie_hold_q <= 1'b0;
      end
    end
  end

  always_ff @(posedge router_clk) begin
    if (router_rst) begin
      usr_irq_ack_router_sync_q <= '0;
    end else begin
      usr_irq_ack_router_sync_q <= {usr_irq_ack_router_sync_q[0], usr_irq_ack_pcie_hold_q};
    end
  end

  assign usr_irq_ack[0] = usr_irq_ack_router_sync_q[1];

  pcie_wrapper u_pcie_wrapper (
    .addr_bram_i(pcie_bram_byte_addr),
    .clk_axi_o(pcie_clk_axi),
    .clk_bram_i(router_clk),
    .clk_pcie_i_clk_n({clk_pcie_n}),
    .clk_pcie_i_clk_p({clk_pcie_p}),
    .clk_sys_buf_o(),
    .clk_sys_i_clk_n(1'b0),
    .clk_sys_i_clk_p(1'b0),
    .din_bram_i(cmd_bram_wdata),
    .dout_bram_o(cmd_bram_rdata),
    .en_bram_i(cmd_bram_en),
    .pcie_7x_mgt_rtl_rxn(pcie_7x_mgt_rtl_0_rxn),
    .pcie_7x_mgt_rtl_rxp(pcie_7x_mgt_rtl_0_rxp),
    .pcie_7x_mgt_rtl_txn(pcie_7x_mgt_rtl_0_txn),
    .pcie_7x_mgt_rtl_txp(pcie_7x_mgt_rtl_0_txp),
    .peripheral_aresetn_o(pcie_peripheral_aresetn),
    .rst_bram_i(1'b0),
    .rstn_pcie_i(rstn_pcie_i),
    .user_lnk_up_o(pcie_user_lnk_up),
    .user_rstn_o(pcie_user_rstn),
    .usr_irq_ack(usr_irq_ack_pcie),
    .usr_irq_req(usr_irq_req_pcie_sync_q),
    .we_bram_i(cmd_bram_we)
  );

`else
  assign sim_cmd_bram_en_o = cmd_bram_en;
  assign sim_cmd_bram_we_o = cmd_bram_we;
  assign sim_cmd_bram_addr_o = cmd_bram_addr;
  assign sim_cmd_bram_wdata_o = cmd_bram_wdata;
  assign cmd_bram_rdata = sim_cmd_bram_rdata_i;
  assign sim_usr_irq_req_o = usr_irq_req;
  assign usr_irq_ack = sim_usr_irq_ack_i;
  assign pcie_clk_axi = router_clk;
  assign pcie_user_rstn = sys_rst_n;
  assign pcie_peripheral_aresetn = sys_rst_n;
  assign pcie_user_lnk_up = 1'b1;
  assign usr_irq_ack_pcie = '0;
  assign pcie_7x_mgt_rtl_0_txn = '0;
  assign pcie_7x_mgt_rtl_0_txp = '0;
`endif

  always_ff @(posedge router_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
      sys_led_div <= '0;
    end else if (router_rst) begin
      sys_led_div <= '0;
    end else begin
      sys_led_div <= sys_led_div + 1'b1;
    end
  end

  always_ff @(posedge pcie_clk_axi or negedge rstn_pcie_i) begin
    if (!rstn_pcie_i) begin
      pcie_led_div <= '0;
    end else begin
      pcie_led_div <= pcie_led_div + 1'b1;
    end
  end
endmodule

`endif
