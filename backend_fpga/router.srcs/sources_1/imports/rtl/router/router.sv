`ifndef FPGA_ROUTER_SV
`define FPGA_ROUTER_SV

`include "../common/segcore_pkg.sv"

module router #(
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
  parameter logic [DATA_W-1:0] RMSPROP_EPS_FXP = 32'd0,
  parameter int GROUP_FLAG_WORD_W = 32,
  parameter int BANK_DEPTH = 65536,
  // Blocked-GRADP v2: BLOCKED_GRADP turns on the NET window-slot decode in
  // the adapter; BLOCKED_SCHED instantiates blocked_ctrl and switches
  // grad_engine/softmax to the windowed block-scheduled datapath.
  parameter bit BLOCKED_GRADP = 1'b0,
  parameter bit BLOCKED_SCHED = 1'b0,
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
  parameter bit PCIE_BRAM_ENABLE = 1'b1,
  parameter int CMD_BRAM_DATA_W = 512,
  parameter int CMD_BRAM_DEPTH = 4096,
  parameter int CMD_BRAM_ADDR_W = 12,
  localparam int SOFTMAX_AXI_BURST_LINES_RAW = (SOFTMAX_AXI_BURST_LINES < 1) ? 1 : SOFTMAX_AXI_BURST_LINES,
  localparam int SOFTMAX_AXI_BURST_LINES_CLAMPED = (SOFTMAX_AXI_BURST_LINES_RAW > 256) ? 256 : SOFTMAX_AXI_BURST_LINES_RAW,
  localparam int SOFTMAX_ROW_READ_BUFFER_DEPTH = 8 * SOFTMAX_AXI_BURST_LINES_CLAMPED,
  localparam int SOFTMAX_AXI_BURST_ROW_SLOTS = 8 * SOFTMAX_AXI_BURST_LINES_CLAMPED,
  localparam int SOFTMAX_WRITE_FLUSH_THRESHOLD = 8 * SOFTMAX_AXI_BURST_ROW_SLOTS,
  localparam int SOFTMAX_ROW_WRITE_BUFFER_DEPTH = 2 * SOFTMAX_WRITE_FLUSH_THRESHOLD,
  // Global DDR row index width (logits/p/gs row space, row = 512b line).
  // Wider than SOFTMAX_ADDR_W (on-chip bank depth): ispd18t8 needs 185,637
  // rows, which overflows 17b.
  localparam int GLOBAL_ROW_W = 20,
  parameter int DDR_AXI_ID_W = 4,
  parameter int DDR_AXI_ADDR_W = 32,
  parameter int DDR_AXI_DATA_W = 512,
  parameter logic [DDR_AXI_ADDR_W-1:0] LOGITS_DRAM_BASE_ADDR = 32'h0000_0000,
  parameter logic [DDR_AXI_ADDR_W-1:0] RMS_DRAM_BASE_ADDR = 32'h0400_0000,
  // NET-on-DDR self-run image region (raw-fetch channel + INIT_SEG
  // region_sel==3 target)
  parameter logic [DDR_AXI_ADDR_W-1:0] NET_IMAGE_DRAM_BASE_ADDR = 32'h0800_0000
) (
  input  logic                                                sys_rst_n,
  input  logic                                                router_clk_i,
  input  logic                                                router_rst_i,
  input  logic                                                ddr_ui_rst_i,
  input  logic                                                ddr_calib_done_i,

  output logic                                                cmd_bram_en_o,
  output logic [(CMD_BRAM_DATA_W/8)-1:0]                      cmd_bram_we_o,
  output logic [CMD_BRAM_ADDR_W-1:0]                          cmd_bram_addr_o,
  output logic [CMD_BRAM_DATA_W-1:0]                          cmd_bram_wdata_o,
  input  logic [CMD_BRAM_DATA_W-1:0]                          cmd_bram_rdata_i,
  output logic [0:0]                                          usr_irq_req_o,
  input  logic [0:0]                                          usr_irq_ack_i,
  output logic                                                router_busy_o,
  output logic [7:0]                                          router_error_o,

  output logic                                                grad_overflow_fifo_warning_o [0:(CORE_COUNT/CORE_COUNT_PER_FIFO)-1][0:CORE_COUNT_PER_FIFO-1],
  output logic                                                grad_overflow_host_ready_o [0:(CORE_COUNT/CORE_COUNT_PER_FIFO)-1][0:CORE_COUNT_PER_FIFO-1],
  output logic                                                grad_wlvia_fifo_warning_o [0:(CORE_COUNT/SEGMENT_PER_FIFO)-1],
  output logic                                                grad_p_finish_o,
  output logic                                                usr_irq_o,
  output logic                                                softmax_enable_o,
  output logic                                                softmax_finish_o [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],

  output logic                                                ui_clk_o,
  output logic                                                ui_rst_o,
  output logic                                                ddr_calib_done_o,

  output logic [DDR_AXI_ID_W-1:0]                             s_axi_awid,
  output logic [DDR_AXI_ADDR_W-1:0]                           s_axi_awaddr,
  output logic [7:0]                                          s_axi_awlen,
  output logic [2:0]                                          s_axi_awsize,
  output logic [1:0]                                          s_axi_awburst,
  output logic [0:0]                                          s_axi_awlock,
  output logic [3:0]                                          s_axi_awcache,
  output logic [2:0]                                          s_axi_awprot,
  output logic [3:0]                                          s_axi_awqos,
  output logic                                                s_axi_awvalid,
  input  logic                                                s_axi_awready,
  output logic [DDR_AXI_DATA_W-1:0]                           s_axi_wdata,
  output logic [(DDR_AXI_DATA_W/8)-1:0]                       s_axi_wstrb,
  output logic                                                s_axi_wlast,
  output logic                                                s_axi_wvalid,
  input  logic                                                s_axi_wready,
  output logic                                                s_axi_bready,
  input  logic [DDR_AXI_ID_W-1:0]                             s_axi_bid,
  input  logic [1:0]                                          s_axi_bresp,
  input  logic                                                s_axi_bvalid,
  output logic [DDR_AXI_ID_W-1:0]                             s_axi_arid,
  output logic [DDR_AXI_ADDR_W-1:0]                           s_axi_araddr,
  output logic [7:0]                                          s_axi_arlen,
  output logic [2:0]                                          s_axi_arsize,
  output logic [1:0]                                          s_axi_arburst,
  output logic [0:0]                                          s_axi_arlock,
  output logic [3:0]                                          s_axi_arcache,
  output logic [2:0]                                          s_axi_arprot,
  output logic [3:0]                                          s_axi_arqos,
  output logic                                                s_axi_arvalid,
  input  logic                                                s_axi_arready,
  output logic                                                s_axi_rready,
  input  logic [DDR_AXI_ID_W-1:0]                             s_axi_rid,
  input  logic [DDR_AXI_DATA_W-1:0]                           s_axi_rdata,
  input  logic [1:0]                                          s_axi_rresp,
  input  logic                                                s_axi_rlast,
  input  logic                                                s_axi_rvalid,

  output logic                                                ddr_rd_req_valid_o,
  output logic                                                ddr_rd_req_ready_o,
  output logic [GLOBAL_ROW_W-1:0]                                 ddr_rd_req_row_o,
  output logic [$clog2(SOFTMAX_ROW_READ_BUFFER_DEPTH + 1)-1:0] ddr_rd_req_count_o,
  output logic                                                ddr_rd_resp_valid_o,
  output logic                                                ddr_rd_resp_ready_o,
  output logic                                                ddr_wr_valid_o,
  output logic                                                ddr_wr_ready_o,
  output logic [GLOBAL_ROW_W-1:0]                                 ddr_wr_row_o,
  output logic [$clog2(SOFTMAX_ROW_WRITE_BUFFER_DEPTH + 1)-1:0] ddr_wr_count_o,
  output logic [3:0]                                          bridge_state_o,
  output logic [2:0]                                          bridge_mode_o,
  output logic [$clog2(((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO)) + 1)-1:0] bridge_bank_o,
  output logic [GLOBAL_ROW_W-1:0]                                 bridge_row_o,
  output logic                                                bridge_target_is_rms_o,
  output logic                                                bridge_ddr_active_o
);
  import segcore_pkg::*;

  localparam int GRAD_OVERFLOW_COUNT = CORE_COUNT / CORE_COUNT_PER_FIFO;
  localparam int GRAD_WLVIA_COUNT = CORE_COUNT / SEGMENT_PER_FIFO;
  localparam int BANK_COUNT = GRAD_OVERFLOW_COUNT + GRAD_WLVIA_COUNT;
  localparam int SOFTMAX_ADDR_W = $clog2(BANK_DEPTH + 1);
  localparam int DDR_AXI_STRB_W = DDR_AXI_DATA_W / 8;
  localparam int DDR_SLOT_W = 64;
  localparam int DDR_SLOT_STRB_W = DDR_SLOT_W / 8;
  localparam int DDR_SLOT_BYTES = DDR_SLOT_W / 8;
  localparam int DDR_LINE_SLOTS = DDR_AXI_DATA_W / DDR_SLOT_W;
  localparam int DDR_LINE_BYTES = DDR_AXI_DATA_W / 8;
  localparam int LOGITS_PER_DDR_LINE = DDR_AXI_DATA_W / DATA_W;
  localparam int RMS_PER_DDR_LINE = DDR_AXI_DATA_W / RMS_V_W;
  localparam int RMS_HALF_COUNT = BANK_COUNT / RMS_PER_DDR_LINE;
  localparam int RMS_LOGICAL_ROWS_PER_BURST = 64 / RMS_HALF_COUNT;
  localparam int AXI_BURST_LINES = SOFTMAX_AXI_BURST_LINES_CLAMPED;
  localparam int AXI_BURST_ROW_SLOTS = DDR_LINE_SLOTS * AXI_BURST_LINES;
  localparam int AXI_BURST_LINE_W = (AXI_BURST_LINES <= 1) ? 1 : $clog2(AXI_BURST_LINES + 1);
  localparam int DDR_ROW_BATCH_W = $clog2(AXI_BURST_ROW_SLOTS + 1);
  localparam int BRIDGE_LINE_DEPTH = AXI_BURST_LINES;
  localparam int BRIDGE_LINE_ADDR_W = (BRIDGE_LINE_DEPTH <= 1) ? 1 : $clog2(BRIDGE_LINE_DEPTH);
  localparam int BRIDGE_BATCH_DEPTH = AXI_BURST_ROW_SLOTS;
  localparam int BRIDGE_BATCH_ADDR_W = (BRIDGE_BATCH_DEPTH <= 1) ? 1 : $clog2(BRIDGE_BATCH_DEPTH);
  localparam int BRIDGE_SLOT_W = (DDR_LINE_SLOTS <= 1) ? 1 : $clog2(DDR_LINE_SLOTS);
  localparam int BRIDGE_PACK_SLOT_W = (BANK_COUNT <= 1) ? 1 : $clog2(BANK_COUNT);
  localparam int DDR_WR_PIPE_DEPTH = 2;
  localparam int DDR_WR_PIPE_PTR_W = (DDR_WR_PIPE_DEPTH <= 1) ? 1 : $clog2(DDR_WR_PIPE_DEPTH);
  localparam int DDR_WR_PIPE_CNT_W = $clog2(DDR_WR_PIPE_DEPTH + 1);
  localparam logic [2:0] DDR_AXI_SIZE = $clog2(DDR_AXI_STRB_W);

  logic router_clk;
  logic router_rst;
  logic ddr_ready_meta_q;
  logic ddr_ready_sync_q;
  logic ddr_ready_router_s;
  logic ddr_ready_raw_s;

  logic [DDR_AXI_ID_W-1:0] m_axi_awid;
  logic [DDR_AXI_ADDR_W-1:0] m_axi_awaddr;
  logic [7:0] m_axi_awlen;
  logic [2:0] m_axi_awsize;
  logic [1:0] m_axi_awburst;
  logic [0:0] m_axi_awlock;
  logic [3:0] m_axi_awcache;
  logic [2:0] m_axi_awprot;
  logic [3:0] m_axi_awqos;
  logic m_axi_awvalid;
  logic m_axi_awready;
  logic [DDR_AXI_DATA_W-1:0] m_axi_wdata;
  logic [DDR_AXI_STRB_W-1:0] m_axi_wstrb;
  logic m_axi_wlast;
  logic m_axi_wvalid;
  logic m_axi_wready;
  logic m_axi_bready;
  logic [DDR_AXI_ID_W-1:0] m_axi_bid;
  logic [1:0] m_axi_bresp;
  logic m_axi_bvalid;
  logic [DDR_AXI_ID_W-1:0] m_axi_arid;
  logic [DDR_AXI_ADDR_W-1:0] m_axi_araddr;
  logic [7:0] m_axi_arlen;
  logic [2:0] m_axi_arsize;
  logic [1:0] m_axi_arburst;
  logic [0:0] m_axi_arlock;
  logic [3:0] m_axi_arcache;
  logic [2:0] m_axi_arprot;
  logic [3:0] m_axi_arqos;
  logic m_axi_arvalid;
  logic m_axi_arready;
  logic m_axi_rready;
  logic [DDR_AXI_ID_W-1:0] m_axi_rid;
  logic [DDR_AXI_DATA_W-1:0] m_axi_rdata;
  logic [1:0] m_axi_rresp;
  logic m_axi_rlast;
  logic m_axi_rvalid;

  logic ddr_init_done_q;

  logic ddr_rd_req_valid_s;
  logic ddr_rd_req_ready_s;
  logic [GLOBAL_ROW_W-1:0] ddr_rd_req_row_s;
  logic [$clog2(SOFTMAX_ROW_READ_BUFFER_DEPTH + 1)-1:0] ddr_rd_req_count_s;
  logic ddr_rd_resp_valid_s;
  logic ddr_rd_resp_ready_s;
  logic [BANK_COUNT-1:0] ddr_rd_resp_bank_mask_s;
  logic ddr_rd_resp_last_s;
  logic signed [DATA_W-1:0] ddr_rd_logits_s [0:BANK_COUNT-1];
  logic signed [RMS_V_W-1:0] ddr_rd_rms_s [0:BANK_COUNT-1];
  logic ddr_wr_valid_s;
  logic ddr_wr_ready_s;
  logic [GLOBAL_ROW_W-1:0] ddr_wr_row_s;
  logic [$clog2(SOFTMAX_ROW_WRITE_BUFFER_DEPTH + 1)-1:0] ddr_wr_count_s;
  logic signed [DATA_W-1:0] ddr_wr_logits_s [0:BANK_COUNT-1];
  logic signed [RMS_V_W-1:0] ddr_wr_rms_s [0:BANK_COUNT-1];
  logic [BANK_COUNT-1:0] ddr_wr_lane_mask_s;
  logic ddr_wr_pipe_push_s;
  logic ddr_wr_pipe_pop_s;
  logic ddr_wr_pipe_valid_s;
  logic [DDR_WR_PIPE_PTR_W-1:0] ddr_wr_pipe_rd_ptr_q;
  logic [DDR_WR_PIPE_PTR_W-1:0] ddr_wr_pipe_wr_ptr_q;
  logic [DDR_WR_PIPE_CNT_W-1:0] ddr_wr_pipe_entry_count_q;
  logic [GLOBAL_ROW_W-1:0] ddr_wr_pipe_row_q [0:DDR_WR_PIPE_DEPTH-1];
  logic [DDR_ROW_BATCH_W-1:0] ddr_wr_pipe_count_q [0:DDR_WR_PIPE_DEPTH-1];
  logic signed [DATA_W-1:0] ddr_wr_pipe_logits_q [0:DDR_WR_PIPE_DEPTH-1][0:BANK_COUNT-1];
  logic signed [RMS_V_W-1:0] ddr_wr_pipe_rms_q [0:DDR_WR_PIPE_DEPTH-1][0:BANK_COUNT-1];
  logic [BANK_COUNT-1:0] ddr_wr_pipe_lane_mask_q [0:DDR_WR_PIPE_DEPTH-1];
  logic ddr_wr_pipe_p_line_q [0:DDR_WR_PIPE_DEPTH-1];
  logic [GLOBAL_ROW_W-1:0] ddr_wr_pipe_row_s;
  logic [DDR_ROW_BATCH_W-1:0] ddr_wr_pipe_count_s;
  logic signed [DATA_W-1:0] ddr_wr_pipe_logits_s [0:BANK_COUNT-1];
  logic signed [RMS_V_W-1:0] ddr_wr_pipe_rms_s [0:BANK_COUNT-1];
  logic [BANK_COUNT-1:0] ddr_wr_pipe_lane_mask_s;
  logic ddr_wr_p_line_s;
  logic ddr_wr_pipe_p_line_s;

  logic [GLOBAL_ROW_W-1:0] bridge_row_idx_q;
  logic [GLOBAL_ROW_W-1:0] bridge_batch_row_base_q;
  logic [DDR_ROW_BATCH_W-1:0] bridge_batch_count_q;
  logic [DDR_ROW_BATCH_W-1:0] bridge_resp_index_q;
  logic bridge_resp_half_q;
  logic [DDR_ROW_BATCH_W-1:0] bridge_rms_chunk_base_q;
  logic [DDR_ROW_BATCH_W-1:0] bridge_rms_chunk_count_q;
  logic [DDR_ROW_BATCH_W-1:0] bridge_collect_count_q;
  logic [AXI_BURST_LINE_W-1:0] bridge_burst_line_count_q;
  logic [AXI_BURST_LINE_W-1:0] bridge_bank_line_count_s;
  logic [AXI_BURST_LINE_W-1:0] bridge_active_bank_line_count_q;
  logic [AXI_BURST_LINE_W-1:0] bridge_axi_beat_idx_q;
  logic [$clog2(BANK_COUNT + 1)-1:0] bridge_bank_q;
  logic [BANK_COUNT-1:0] bridge_wr_lane_mask_batch_q [0:AXI_BURST_ROW_SLOTS-1];
  logic [DDR_AXI_DATA_W-1:0] bridge_wdata_s;
  logic [DDR_AXI_STRB_W-1:0] bridge_wstrb_s;
  logic bridge_bank_has_write_s;

  // ---- read-side multi-outstanding (OT) ----
  // Up to BRIDGE_RD_OT_SLOTS engine-read descriptors may be in flight. Every
  // AR goes out with the same AXI ID ('0), so the MIG returns R beats in
  // issue order and response pairing is purely positional (completion queue).
  localparam int BRIDGE_RD_OT_SLOTS = 4;
  localparam int BRIDGE_RD_SLOT_W = (BRIDGE_RD_OT_SLOTS <= 1) ? 1 : $clog2(BRIDGE_RD_OT_SLOTS);
  // init staging keeps one extra logits slot of the same BRAM family
  localparam int BRIDGE_INIT_SLOT = BRIDGE_RD_OT_SLOTS;
  localparam int BRIDGE_LOGITS_SLOTS = BRIDGE_RD_OT_SLOTS + 1;
  localparam int BRIDGE_RMS_SLOTS = BRIDGE_RD_OT_SLOTS * RMS_HALF_COUNT;
  localparam int BRIDGE_RD_FIFO_DEPTH = 4;
  localparam int BRIDGE_RD_FIFO_PTR_W = (BRIDGE_RD_FIFO_DEPTH <= 1) ? 1 : $clog2(BRIDGE_RD_FIFO_DEPTH);
  localparam int BRIDGE_RD_FIFO_CNT_W = $clog2(BRIDGE_RD_FIFO_DEPTH + 1);
  // in-flight burst completion queue (one entry per issued AR burst)
  localparam int BRIDGE_CQ_DEPTH = 8;
  localparam int BRIDGE_CQ_PTR_W = (BRIDGE_CQ_DEPTH <= 1) ? 1 : $clog2(BRIDGE_CQ_DEPTH);
  localparam int BRIDGE_CQ_CNT_W = $clog2(BRIDGE_CQ_DEPTH + 1);
  localparam logic [1:0] BCQ_DEST_LOGITS = 2'd0;
  localparam logic [1:0] BCQ_DEST_RMS    = 2'd1;
  localparam logic [1:0] BCQ_DEST_FETCH  = 2'd2;

  // read request descriptor FIFO (engine row reads + net fetch)
  logic brq_kind_q [0:BRIDGE_RD_FIFO_DEPTH-1];        // 0=engine, 1=fetch
  logic [31:0] brq_base_q [0:BRIDGE_RD_FIFO_DEPTH-1]; // engine row base / fetch line base
  logic [8:0] brq_count_q [0:BRIDGE_RD_FIFO_DEPTH-1]; // engine rows / fetch lines
  logic brq_need_rms_q [0:BRIDGE_RD_FIFO_DEPTH-1];
  logic [BRIDGE_RD_FIFO_PTR_W-1:0] brq_rd_ptr_q;
  logic [BRIDGE_RD_FIFO_PTR_W-1:0] brq_wr_ptr_q;
  logic [BRIDGE_RD_FIFO_CNT_W-1:0] brq_level_q;

  // burst issuer (descriptor -> back-to-back ARs)
  logic iss_active_q;
  logic iss_kind_q;
  logic [31:0] iss_base_q;
  logic [8:0] iss_count_q;
  logic iss_need_rms_q;
  logic iss_logits_done_q;
  logic [7:0] iss_rms_left_q;
  logic [7:0] iss_rms_chunk_q;
  logic [BRIDGE_RD_SLOT_W-1:0] iss_slot_q;
  logic [BRIDGE_RD_SLOT_W-1:0] alloc_ptr_q;
  logic iss_cur_is_fetch_s;
  logic iss_cur_is_logits_s;
  logic [DDR_AXI_ADDR_W-1:0] iss_cur_addr_s;
  logic [7:0] iss_cur_len_s;
  logic [7:0] iss_cur_beats_s;
  logic [7:0] iss_cur_off_s;
  logic [7:0] iss_cur_lines_s;
  logic iss_cur_slot_last_s;
  logic iss_pop_s;

  // completion queue (in-order R pairing; one entry per issued AR burst)
  logic [1:0] bcq_dest_q [0:BRIDGE_CQ_DEPTH-1];
  logic [BRIDGE_RD_SLOT_W-1:0] bcq_slot_q [0:BRIDGE_CQ_DEPTH-1];
  logic [7:0] bcq_off_q [0:BRIDGE_CQ_DEPTH-1];
  logic [7:0] bcq_beats_q [0:BRIDGE_CQ_DEPTH-1];
  logic bcq_slot_last_q [0:BRIDGE_CQ_DEPTH-1];
  logic [BRIDGE_CQ_PTR_W-1:0] bcq_rd_ptr_q;
  logic [BRIDGE_CQ_PTR_W-1:0] bcq_wr_ptr_q;
  logic [BRIDGE_CQ_CNT_W-1:0] bcq_level_q;
  logic [7:0] recv_beat_q;
  logic recv_r_fire_s;
  logic recv_r_last_s;

  // response slot ring state
  typedef enum logic [1:0] {
    SLOT_FREE    = 2'd0,
    SLOT_FILL    = 2'd1,
    SLOT_READY   = 2'd2,
    SLOT_DELIVER = 2'd3
  } bridge_slot_state_e;
  bridge_slot_state_e slot_state_q [0:BRIDGE_RD_OT_SLOTS-1];

  // delivery queue (engine descriptors in issue order)
  logic [BRIDGE_RD_SLOT_W-1:0] bdq_slot_q [0:BRIDGE_RD_OT_SLOTS-1];
  logic [GLOBAL_ROW_W-1:0] bdq_row_q [0:BRIDGE_RD_OT_SLOTS-1];
  logic [8:0] bdq_count_q [0:BRIDGE_RD_OT_SLOTS-1];
  logic bdq_need_rms_q [0:BRIDGE_RD_OT_SLOTS-1];
  logic [BRIDGE_RD_SLOT_W-1:0] bdq_rd_ptr_q;
  logic [BRIDGE_RD_SLOT_W-1:0] bdq_wr_ptr_q;
  logic [BRIDGE_RD_SLOT_W:0] bdq_level_q;

  // delivery FSM (per-row RESP beats, ported from the old BRIDGE_RESP states)
  typedef enum logic [1:0] {
    DEL_IDLE = 2'd0,
    DEL_PREP = 2'd1,
    DEL_RESP = 2'd2
  } bridge_del_state_e;
  bridge_del_state_e del_state_q;
  logic [BRIDGE_RD_SLOT_W-1:0] del_slot_q;
  logic [GLOBAL_ROW_W-1:0] del_row_base_q;
  logic [8:0] del_count_q;
  logic del_need_rms_q;
  logic del_rms_inst_q;
  logic signed [DDR_AXI_DATA_W-1:0] bridge_del_logits_data_s;
  logic signed [DDR_AXI_DATA_W-1:0] bridge_del_rms_data_s;
  logic bridge_read_quiescent_s;
  logic brq_push_engine_s;
  logic brq_push_fetch_s;
  logic bdq_push_s;
  logic bdq_pop_s;
  logic bcq_push_s;
  logic recv_slot_ready_s;
  logic del_start_s;

  logic bridge_rd_logits_line_rd_ce_s [0:BRIDGE_LOGITS_SLOTS-1];
  logic [BRIDGE_LINE_ADDR_W-1:0] bridge_rd_logits_line_rd_addr_s [0:BRIDGE_LOGITS_SLOTS-1];
  logic signed [DDR_AXI_DATA_W-1:0] bridge_rd_logits_line_rd_data_s [0:BRIDGE_LOGITS_SLOTS-1];
  logic bridge_rd_logits_line_wr_ce_s [0:BRIDGE_LOGITS_SLOTS-1];
  logic [BRIDGE_LINE_ADDR_W-1:0] bridge_rd_logits_line_wr_addr_s [0:BRIDGE_LOGITS_SLOTS-1];
  logic signed [DDR_AXI_DATA_W-1:0] bridge_rd_logits_line_wr_data_s [0:BRIDGE_LOGITS_SLOTS-1];
  logic bridge_rd_rms_line_rd_ce_s [0:BRIDGE_RMS_SLOTS-1];
  logic [BRIDGE_LINE_ADDR_W-1:0] bridge_rd_rms_line_rd_addr_s [0:BRIDGE_RMS_SLOTS-1];
  logic signed [DDR_AXI_DATA_W-1:0] bridge_rd_rms_line_rd_data_s [0:BRIDGE_RMS_SLOTS-1];
  logic bridge_rd_rms_line_wr_ce_s [0:BRIDGE_RMS_SLOTS-1];
  logic [BRIDGE_LINE_ADDR_W-1:0] bridge_rd_rms_line_wr_addr_s [0:BRIDGE_RMS_SLOTS-1];
  logic signed [DDR_AXI_DATA_W-1:0] bridge_rd_rms_line_wr_data_s [0:BRIDGE_RMS_SLOTS-1];
  logic bridge_wr_logits_batch_rd_ce_s [0:BANK_COUNT-1];
  logic [BRIDGE_BATCH_ADDR_W-1:0] bridge_wr_logits_batch_rd_addr_s [0:BANK_COUNT-1];
  logic signed [DATA_W-1:0] bridge_wr_logits_batch_rd_data_s [0:BANK_COUNT-1];
  logic bridge_wr_logits_batch_wr_ce_s [0:BANK_COUNT-1];
  logic [BRIDGE_BATCH_ADDR_W-1:0] bridge_wr_logits_batch_wr_addr_s [0:BANK_COUNT-1];
  logic signed [DATA_W-1:0] bridge_wr_logits_batch_wr_data_s [0:BANK_COUNT-1];
  logic bridge_wr_rms_batch_rd_ce_s [0:BANK_COUNT-1];
  logic [BRIDGE_BATCH_ADDR_W-1:0] bridge_wr_rms_batch_rd_addr_s [0:BANK_COUNT-1];
  logic signed [RMS_V_W-1:0] bridge_wr_rms_batch_rd_data_s [0:BANK_COUNT-1];
  logic bridge_wr_rms_batch_wr_ce_s [0:BANK_COUNT-1];
  logic [BRIDGE_BATCH_ADDR_W-1:0] bridge_wr_rms_batch_wr_addr_s [0:BANK_COUNT-1];
  logic signed [RMS_V_W-1:0] bridge_wr_rms_batch_wr_data_s [0:BANK_COUNT-1];
  logic bridge_choose_write_s;
  logic bridge_choose_init_s;
  logic bridge_write_accept_s;
  logic bridge_wr_stage_valid_q;
  logic [BRIDGE_BATCH_ADDR_W-1:0] bridge_wr_stage_addr_q;
  logic signed [DATA_W-1:0] bridge_wr_stage_logits_q [0:BANK_COUNT-1];
  logic signed [RMS_V_W-1:0] bridge_wr_stage_rms_q [0:BANK_COUNT-1];
  logic init_line_valid_s;
  logic init_line_ready_s;
  logic [31:0] init_line_index_s;
  logic init_line_is_rms_s;
  logic init_line_is_image_s;
  logic init_line_last_s;
  logic [DDR_AXI_DATA_W-1:0] init_line_data_s;
  logic group_flag_init_valid_s;
  logic group_flag_init_ready_s;
  logic [31:0] group_flag_init_addr_s;
  logic [GROUP_FLAG_WORD_W-1:0] group_flag_init_data_s [0:BANK_COUNT-1];
  logic readback_start_s;
  logic readback_is_grad_s;
  logic readback_req_valid_s;
  logic readback_req_ready_s;
  logic [$clog2(BANK_DEPTH)-1:0] readback_req_addr_s;
  logic readback_resp_valid_s;
  logic signed [P_DATA_W-1:0] readback_p_data_s [0:BANK_COUNT-1];
  logic signed [MEM_GRAD_P_W-1:0] readback_grad_p_data_s [0:BANK_COUNT-1];
  logic readback_advance_s;
  logic bridge_init_can_accept_s;
  logic [AXI_BURST_LINE_W-1:0] bridge_init_collect_count_q;
  logic [31:0] bridge_init_line_base_q;
  logic bridge_init_command_last_q;
  logic pcie_system_reset_hold_s;
  logic pcie_system_reset_hold_q;
  logic pcie_full_reset_active_s;
  logic bridge_reset_s;
  logic pcie_grad_iteration_done_s;
  logic grad_p_task_ready_s [0:CORE_COUNT-1];
  logic dispatcher_fifo_overflow_s [0:(CORE_COUNT/CORE_COUNT_PER_FIFO)-1][0:CORE_COUNT_PER_FIFO-1];
  logic [15:0] gradp_debug_core_s [0:CORE_COUNT-1];
  logic [31:0] gradp_debug_bank_s;
  logic [(CORE_COUNT*13+2)*32-1:0] gradp_debug_trace_s;
  logic [31:0] stall_probe_s;
  logic grad_engine_rst_n_s;

  // ---- Blocked-GRADP v2 interconnection wires ----
  localparam int BLK_WIN_ADDR_W =
    BLOCKED_SCHED ? $clog2(2 * WIN_ROWS) : $clog2(BANK_DEPTH);
  localparam int BLK_GFLAG_ADDR_W = $clog2(2 * (WIN_ROWS / GROUP_FLAG_WORD_W));
  localparam int BLK_IDX_W = (P_REGIONS <= 1) ? 1 : $clog2(P_REGIONS);
  // adapter -> blocked_ctrl
  logic blk_desc_wr_valid_s;
  logic [10:0] blk_desc_wr_addr_s;
  logic [31:0] blk_desc_wr_data_s;
  logic [10:0] blk_desc_rd_addr_s;
  logic [31:0] blk_desc_rd_data_s;
  logic blk_block_done_s;
  logic blk_iter_last_s;
  // blocked_ctrl -> adapter
  logic blk_spill_done_s;
  logic blk_iter_done_s;
  logic blk_upd_phantom_seen_s;
  // NET-on-DDR self-run: adapter <-> master (blocked_ctrl) / net_fetcher
  logic sr_go_s;
  logic [31:0] sr_go_iters_s;
  logic blk_go_s;
  logic dbg_load_kick_s;
  logic [5:0] dbg_load_blk_s;
  logic dbg_load_half_s;
  logic sr_play_s;
  logic sr_play_last_s;
  logic sr_play_ack_s;
  logic sr_done_s;
  logic sr_running_s;
  logic [31:0] sr_iter_cnt_s;
  logic [7:0] sr_error_s;
  logic [5:0] sr_cur_blk_s;
  logic [2:0] sr_mn_state_s;
  logic [9:0] sr_upd_probe_s;
  logic [223:0] nf_perf_probe_s;
  logic [3:0] sr_ld_state_s;
  logic [5:0] sr_ld_blk_s;
  logic sr_ld_done_valid_s;
  logic [16:0] sr_ld_row_cnt_s;
  // net_fetcher debug probes
  logic [2:0] nf_dbg_f_state_s;
  logic [6:0] nf_dbg_cf_sec_s;
  logic [6:0] nf_dbg_nf_sec_s;
  logic [31:0] nf_dbg_cf_line_s;
  logic [31:0] nf_dbg_nf_line_s;
  logic nf_dbg_flight_s;
  // engine / loader probe buses
  logic [33:0] eng_probe_s;
  logic [19:0] sr_ld_probe2_s;
  logic [15:0] bridge_probe_s;
  // net_fetcher -> adapter NET frame stream (FWFT)
  logic nf_frame_valid_s;
  logic [255:0] nf_frame_data_s;
  logic nf_frame_last_s;
  logic nf_frame_pop_s;
  // net_fetcher -> blocked_ctrl ctrl lane stream (FWFT)
  logic cf_lane_valid_s;
  logic [63:0] cf_lane_data_s;
  logic cf_lane_pop_s;
  // net_fetcher <-> bridge raw line fetch channel
  logic fr_req_valid_s;
  logic fr_req_ready_s;
  logic [31:0] fr_req_line_s;
  logic [8:0] fr_req_count_s;
  logic fr_resp_valid_s;
  logic fr_resp_ready_s;
  logic [511:0] fr_resp_data_s;
  logic fr_resp_last_s;
  // blocked_ctrl -> grad_engine context
  logic blk_cur_half_s;
  logic blk_play_half_s;
  logic [16:0] blk_real_cnt_s [0:BANK_COUNT-1];
  logic [31:0] blk_xwin_base_s [0:BANK_COUNT-1];
  logic [31:0] blk_p_row_base_s;
  logic [SOFTMAX_ADDR_W-1:0] blk_bank_valid_len_s [0:BANK_COUNT-1];
  logic blk_update_active_s;
  logic blk_load_active_s;
  logic blk_pre_load_done_s;
  logic blk_eng_iter_done_s;
  // grad_engine -> blocked_ctrl status
  logic [BLK_IDX_W-1:0] blk_pre_seg_idx_s;
  logic blk_pre_seg_active_s;
  logic blk_softmax_all_finish_s;
  logic blk_p_asm_drained_s;
  logic blk_p_wr_strobe_s [0:BANK_COUNT-1];
  logic blk_wr_phantom_pop_s [0:BANK_COUNT-1];
  // blocked_ctrl -> grad_engine pre-softmax segment count (descriptor 0x1F0)
  logic [BLK_IDX_W:0] blk_pre_seg_count_s;
  // blocked_ctrl <-> grad_engine loader ports
  logic ld_grad_win_wr_ce_s [0:BANK_COUNT-1];
  logic [BLK_WIN_ADDR_W-1:0] ld_grad_win_wr_addr_s [0:BANK_COUNT-1];
  logic signed [MEM_GRAD_P_W-1:0] ld_grad_win_wr_data_s [0:BANK_COUNT-1];
  logic ld_grad_win_rd_req_s [0:BANK_COUNT-1];
  logic [BLK_WIN_ADDR_W-1:0] ld_grad_win_rd_addr_s [0:BANK_COUNT-1];
  logic ld_grad_win_rd_grant_s [0:BANK_COUNT-1];
  logic signed [MEM_GRAD_P_W-1:0] ld_grad_win_rd_data_s [0:BANK_COUNT-1];
  logic ld_pwin_wr_ce_s [0:BANK_COUNT-1];
  logic [BLK_WIN_ADDR_W-1:0] ld_pwin_wr_addr_s [0:BANK_COUNT-1];
  logic signed [P_DATA_W-1:0] ld_pwin_wr_data_s [0:BANK_COUNT-1];
  logic ld_gflag_wr_ce_s [0:BANK_COUNT-1];
  logic [BLK_GFLAG_ADDR_W-1:0] ld_gflag_wr_addr_s [0:BANK_COUNT-1];
  logic [GROUP_FLAG_WORD_W-1:0] ld_gflag_wr_data_s [0:BANK_COUNT-1];
  logic ld_gflag_wr_special_s;
  logic ld_xwin_rd_req_s [0:BANK_COUNT-1];
  localparam int XWIN_ADDR_W = 15;  // clog2(XWIN_DEPTH=32768), matches grad_engine
  logic [XWIN_ADDR_W-1:0] ld_xwin_rd_addr_s [0:BANK_COUNT-1];
  logic signed [P_DATA_W-1:0] ld_xwin_rd_data_s [0:BANK_COUNT-1];
  logic p_wr_entry_valid_s [0:BANK_COUNT-1];
  logic [SOFTMAX_ADDR_W-1:0] p_wr_entry_addr_s [0:BANK_COUNT-1];
  logic signed [P_DATA_W-1:0] p_wr_entry_data_s [0:BANK_COUNT-1];
  // segment descriptor / meta row stream (blocked_ctrl <-> grad_engine)
  logic [31:0] seg_logits_base_s;
  logic meta_rd_req_valid_s;
  logic meta_rd_req_ready_s;
  logic [GLOBAL_ROW_W-1:0] meta_rd_req_row_s;
  logic [15:0] meta_rd_req_count_s;
  logic meta_rd_resp_valid_s;
  logic meta_rd_resp_ready_s;
  logic meta_rd_resp_last_s;
  logic [31:0] meta_rd_resp_lane_s [0:BANK_COUNT-1];

  typedef enum logic [2:0] {
    BRIDGE_MODE_NONE       = 3'd0,
    BRIDGE_MODE_INIT       = 3'd1,
    BRIDGE_MODE_SOFT_RD    = 3'd2,
    BRIDGE_MODE_SOFT_WR    = 3'd3
  } bridge_mode_e;

  typedef enum logic [4:0] {
    BRIDGE_IDLE            = 5'd0,
    BRIDGE_AW              = 5'd1,
    BRIDGE_W               = 5'd2,
    BRIDGE_B               = 5'd3,
    BRIDGE_WR_COLLECT      = 5'd7,
    BRIDGE_W_BUILD_REQ     = 5'd9,
    BRIDGE_W_BUILD_READ    = 5'd10,
    BRIDGE_W_BUILD_CAPTURE = 5'd11,
    BRIDGE_AW_PREP         = 5'd13,
    BRIDGE_INIT_COLLECT    = 5'd14,
    BRIDGE_INIT_W_READ     = 5'd15,
    BRIDGE_INIT_W_CAPTURE  = 5'd16
  } bridge_state_e;

  bridge_state_e bridge_state_q;
  bridge_mode_e bridge_mode_q;
  logic bridge_target_is_rms_q;
  logic bridge_target_is_image_q;
  logic bridge_need_rms_q;
  logic update_softmax_phase_s;
  logic [3:0] system_phase_s;
  banked_idx_t segment_idx_system_s [0:CORE_COUNT-1];
  logic [$clog2(CHUNK_SIZE)-1:0] segment_l_system_s [0:CORE_COUNT-1];
  logic [$clog2(CHUNK_SIZE)-1:0] segment_r_system_s [0:CORE_COUNT-1];
  logic signed [DATA_W-1:0] segment_w_system_s [0:CORE_COUNT-1];
  logic signed [COEFF_W-1:0] segment_coeff_system_s [0:CORE_COUNT-1];
  logic [2:0] segment_type_system_s [0:CORE_COUNT-1];
  logic [1:0] mode_overflow_system_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic mode_overflow_valid_system_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic [2:0] mode_wlvia_system_s [0:GRAD_WLVIA_COUNT-1];
  logic [GLOBAL_ROW_W-1:0] bank_valid_len_system_s [0:BANK_COUNT-1];
  // block-local low 17 bits for the engine's bank_valid_len_i (the
  // system-side register is GLOBAL_ROW_W wide for the bridge's
  // grand-total resp mask; the engine only uses block-local lengths)
  logic [16:0] bank_valid_len_engine_s [0:BANK_COUNT-1];
  for (genvar vlen_gb = 0; vlen_gb < BANK_COUNT; vlen_gb++) begin : GEN_BANK_VLEN_ENGINE
    assign bank_valid_len_engine_s[vlen_gb] = bank_valid_len_system_s[vlen_gb][16:0];
  end
  banked_idx_t pcie_segment_idx_s [0:CORE_COUNT-1];
  logic [$clog2(CHUNK_SIZE)-1:0] pcie_segment_l_s [0:CORE_COUNT-1];
  logic [$clog2(CHUNK_SIZE)-1:0] pcie_segment_r_s [0:CORE_COUNT-1];
  logic signed [DATA_W-1:0] pcie_segment_w_s [0:CORE_COUNT-1];
  logic signed [COEFF_W-1:0] pcie_segment_coeff_s [0:CORE_COUNT-1];
  logic [2:0] pcie_segment_type_s [0:CORE_COUNT-1];
  logic [1:0] pcie_mode_overflow_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic pcie_mode_overflow_valid_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic [2:0] pcie_mode_wlvia_s [0:GRAD_WLVIA_COUNT-1];
  logic [GLOBAL_ROW_W-1:0] pcie_bank_valid_len_s [0:BANK_COUNT-1];
  logic [GLOBAL_ROW_W-1:0] bank_valid_len_system_q [0:BANK_COUNT-1];

  assign router_clk = router_clk_i;
  assign router_rst = router_rst_i;
  assign ddr_ready_raw_s = ddr_calib_done_i && !ddr_ui_rst_i;
  assign ddr_ready_router_s = ddr_ready_sync_q;
  assign ui_clk_o = router_clk;
  assign ui_rst_o = router_rst;
  assign ddr_calib_done_o = ddr_ready_router_s;
  assign bridge_reset_s = router_rst || pcie_full_reset_active_s;
  assign grad_engine_rst_n_s = sys_rst_n &&
    !pcie_system_reset_hold_s && !pcie_system_reset_hold_q;

  assign s_axi_awid = m_axi_awid;
  assign s_axi_awlen = m_axi_awlen;
  assign s_axi_awsize = m_axi_awsize;
  assign s_axi_awburst = m_axi_awburst;
  assign s_axi_awlock = m_axi_awlock;
  assign s_axi_awcache = m_axi_awcache;
  assign s_axi_awprot = m_axi_awprot;
  assign s_axi_awqos = m_axi_awqos;
  assign s_axi_awvalid = m_axi_awvalid;
  assign s_axi_awaddr = m_axi_awaddr;
  assign s_axi_wdata = m_axi_wdata;
  assign s_axi_wstrb = m_axi_wstrb;
  assign s_axi_wlast = m_axi_wlast;
  assign s_axi_wvalid = m_axi_wvalid;
  assign s_axi_bready = m_axi_bready;
  assign s_axi_arid = m_axi_arid;
  assign s_axi_arlen = m_axi_arlen;
  assign s_axi_arsize = m_axi_arsize;
  assign s_axi_arburst = m_axi_arburst;
  assign s_axi_arlock = m_axi_arlock;
  assign s_axi_arcache = m_axi_arcache;
  assign s_axi_arprot = m_axi_arprot;
  assign s_axi_arqos = m_axi_arqos;
  assign s_axi_arvalid = m_axi_arvalid;
  assign s_axi_araddr = m_axi_araddr;
  assign s_axi_rready = m_axi_rready;

  assign m_axi_awready = s_axi_awready;
  assign m_axi_wready = s_axi_wready;
  assign m_axi_bid = s_axi_bid;
  assign m_axi_bresp = s_axi_bresp;
  assign m_axi_bvalid = s_axi_bvalid;
  assign m_axi_arready = s_axi_arready;
  assign m_axi_rid = s_axi_rid;
  assign m_axi_rdata = s_axi_rdata;
  assign m_axi_rresp = s_axi_rresp;
  assign m_axi_rlast = s_axi_rlast;
  assign m_axi_rvalid = s_axi_rvalid;

  always_ff @(posedge router_clk) begin
    if (router_rst) begin
      ddr_ready_meta_q <= 1'b0;
      ddr_ready_sync_q <= 1'b0;
    end else begin
      ddr_ready_meta_q <= ddr_ready_raw_s;
      ddr_ready_sync_q <= ddr_ready_meta_q;
    end
  end

  always_ff @(posedge router_clk) begin
    if (router_rst) begin
      ddr_init_done_q <= 1'b0;
    end else begin
      if (PCIE_BRAM_ENABLE && ddr_ready_router_s) begin
        ddr_init_done_q <= 1'b1;
      end
    end
  end

  always_comb begin
    for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
      segment_idx_system_s[core_idx] = pcie_segment_idx_s[core_idx];
      segment_l_system_s[core_idx] = pcie_segment_l_s[core_idx];
      segment_r_system_s[core_idx] = pcie_segment_r_s[core_idx];
      segment_w_system_s[core_idx] = pcie_segment_w_s[core_idx];
      segment_coeff_system_s[core_idx] = pcie_segment_coeff_s[core_idx];
      segment_type_system_s[core_idx] = pcie_segment_type_s[core_idx];
    end

    for (int group_idx = 0; group_idx < GRAD_OVERFLOW_COUNT; group_idx++) begin
      for (int lane_idx = 0; lane_idx < CORE_COUNT_PER_FIFO; lane_idx++) begin
        mode_overflow_system_s[group_idx][lane_idx] = pcie_mode_overflow_s[group_idx][lane_idx];
        mode_overflow_valid_system_s[group_idx][lane_idx] = pcie_mode_overflow_valid_s[group_idx][lane_idx];
      end
    end

    for (int group_idx = 0; group_idx < GRAD_WLVIA_COUNT; group_idx++) begin
      mode_wlvia_system_s[group_idx] = pcie_mode_wlvia_s[group_idx];
    end

    for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
      bank_valid_len_system_s[bank_idx] = bank_valid_len_system_q[bank_idx];
    end
  end

  always_ff @(posedge router_clk) begin
    if (router_rst) begin
      pcie_system_reset_hold_q <= 1'b1;
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        bank_valid_len_system_q[bank_idx] <= '0;
      end
    end else begin
      pcie_system_reset_hold_q <= pcie_system_reset_hold_s;
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        bank_valid_len_system_q[bank_idx] <= pcie_bank_valid_len_s[bank_idx];
      end
    end
  end

  router_pcie_bram_adapter #(
    .CORE_COUNT(CORE_COUNT),
    .CHUNK_SIZE(CHUNK_SIZE),
    .CORE_COUNT_PER_FIFO(CORE_COUNT_PER_FIFO),
    .SEGMENT_PER_FIFO(SEGMENT_PER_FIFO),
    .DATA_W(DATA_W),
    .FRAC_W(FRAC_W),
    .COEFF_W(COEFF_W),
    .P_DATA_W(P_DATA_W),
    .MEM_GRAD_P_W(MEM_GRAD_P_W),
    .RMS_V_W(RMS_V_W),
    .GROUP_FLAG_WORD_W(GROUP_FLAG_WORD_W),
    .BANK_DEPTH(BANK_DEPTH),
    .BLOCKED_GRADP(BLOCKED_GRADP),
    .BLOCKED_SCHED(BLOCKED_SCHED),
    .NET_IMAGE_DRAM_BASE_ADDR(NET_IMAGE_DRAM_BASE_ADDR),
    .BRAM_DATA_W(CMD_BRAM_DATA_W),
    .BRAM_DEPTH(CMD_BRAM_DEPTH),
    .BRAM_ADDR_W(CMD_BRAM_ADDR_W)
  ) u_pcie_bram_adapter (
    .clk(router_clk),
    .rst_n(sys_rst_n && !router_rst),
    .enable_i(PCIE_BRAM_ENABLE),
    .cmd_bram_en_o(cmd_bram_en_o),
    .cmd_bram_we_o(cmd_bram_we_o),
    .cmd_bram_addr_o(cmd_bram_addr_o),
    .cmd_bram_wdata_o(cmd_bram_wdata_o),
    .cmd_bram_rdata_i(cmd_bram_rdata_i),
    .usr_irq_req_o(usr_irq_req_o),
    .usr_irq_ack_i(usr_irq_ack_i),
    .system_event_i(usr_irq_o),
    .system_phase_i(system_phase_s),
    .grad_p_finish_i(grad_p_finish_o),
    .softmax_enable_i(softmax_enable_o),
    .bridge_ddr_active_i(bridge_ddr_active_o),
    .grad_overflow_host_ready_i(grad_overflow_host_ready_o),
    .dispatcher_fifo_overflow_i(dispatcher_fifo_overflow_s),
    .grad_wlvia_fifo_warning_i(grad_wlvia_fifo_warning_o),
    .grad_task_ready_i(grad_p_task_ready_s),
    .gradp_debug_core_i(gradp_debug_core_s),
    .gradp_debug_bank_i(gradp_debug_bank_s),
    .gradp_debug_trace_i(gradp_debug_trace_s),
    .stall_probe_i(stall_probe_s),
    .grad_iteration_done_o(pcie_grad_iteration_done_s),
    .readback_start_o(readback_start_s),
    .readback_is_grad_o(readback_is_grad_s),
    .readback_req_valid_o(readback_req_valid_s),
    .readback_req_ready_i(readback_req_ready_s),
    .readback_req_addr_o(readback_req_addr_s),
    .readback_resp_valid_i(readback_resp_valid_s),
    .readback_p_data_i(readback_p_data_s),
    .readback_grad_p_data_i(readback_grad_p_data_s),
    .readback_advance_o(readback_advance_s),
    .init_line_valid_o(init_line_valid_s),
    .init_line_ready_i(init_line_ready_s),
    .init_line_index_o(init_line_index_s),
    .init_line_is_rms_o(init_line_is_rms_s),
    .init_line_is_image_o(init_line_is_image_s),
    .init_line_last_o(init_line_last_s),
    .init_line_data_o(init_line_data_s),
    .group_flag_init_valid_o(group_flag_init_valid_s),
    .group_flag_init_ready_i(group_flag_init_ready_s),
    .group_flag_init_addr_o(group_flag_init_addr_s),
    .group_flag_init_data_o(group_flag_init_data_s),
    .system_reset_hold_o(pcie_system_reset_hold_s),
    .full_reset_active_o(pcie_full_reset_active_s),
    .segment_idx_o(pcie_segment_idx_s),
    .segment_l_o(pcie_segment_l_s),
    .segment_r_o(pcie_segment_r_s),
    .segment_w_o(pcie_segment_w_s),
    .segment_coeff_o(pcie_segment_coeff_s),
    .segment_type_o(pcie_segment_type_s),
    .mode_overflow_o(pcie_mode_overflow_s),
    .mode_overflow_valid_o(pcie_mode_overflow_valid_s),
    .mode_wlvia_o(pcie_mode_wlvia_s),
    .bank_valid_len_o(pcie_bank_valid_len_s),
    .blk_desc_wr_valid_o(blk_desc_wr_valid_s),
    .blk_desc_wr_addr_o(blk_desc_wr_addr_s),
    .blk_desc_wr_data_o(blk_desc_wr_data_s),
    .blk_desc_rd_addr_o(blk_desc_rd_addr_s),
    .blk_desc_rd_data_i(blk_desc_rd_data_s),
    .blk_block_done_o(blk_block_done_s),
    .blk_iter_last_o(blk_iter_last_s),
    .blk_spill_done_i(blk_spill_done_s),
    .blk_iter_done_i(blk_iter_done_s),
    .blk_upd_phantom_seen_i(blk_upd_phantom_seen_s),
    .sr_go_o(sr_go_s),
    .sr_go_iters_o(sr_go_iters_s),
    .blk_go_o(blk_go_s),
    .dbg_load_kick_o(dbg_load_kick_s),
    .dbg_load_blk_o(dbg_load_blk_s),
    .dbg_load_half_o(dbg_load_half_s),
    .sr_play_i(sr_play_s),
    .sr_play_last_i(sr_play_last_s),
    .sr_play_ack_o(sr_play_ack_s),
    .sr_done_i(sr_done_s),
    .sr_running_i(sr_running_s),
    .sr_iter_cnt_i(sr_iter_cnt_s),
    .sr_error_i(sr_error_s),
    .sr_cur_blk_i(sr_cur_blk_s),
    .sr_mn_state_i(sr_mn_state_s),
    .sr_ld_state_i(sr_ld_state_s),
    .sr_ld_blk_i(sr_ld_blk_s),
    .sr_ld_done_valid_i(sr_ld_done_valid_s),
    .sr_ld_row_cnt_i(sr_ld_row_cnt_s),
    .nf_dbg_f_state_i(nf_dbg_f_state_s),
    .nf_dbg_cf_sec_i(nf_dbg_cf_sec_s),
    .nf_dbg_nf_sec_i(nf_dbg_nf_sec_s),
    .nf_dbg_cf_line_i(nf_dbg_cf_line_s),
    .nf_dbg_nf_line_i(nf_dbg_nf_line_s),
    .nf_dbg_flight_i(nf_dbg_flight_s),
    .eng_probe_i(eng_probe_s),
    .ld_probe2_i(sr_ld_probe2_s),
    .bridge_probe_i(bridge_probe_s),
    .sr_upd_probe_i(sr_upd_probe_s),
    .nf_perf_probe_i(nf_perf_probe_s),
    .sr_frame_valid_i(nf_frame_valid_s),
    .sr_frame_data_i(nf_frame_data_s),
    .sr_frame_last_i(nf_frame_last_s),
    .sr_frame_pop_o(nf_frame_pop_s),
    .busy_o(router_busy_o),
    .error_o(router_error_o)
  );

  grad_engine #(
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
    .BLOCKED_SCHED(BLOCKED_SCHED),
    .WIN_ROWS(WIN_ROWS),
    .P_REGIONS(P_REGIONS),
    .DDR_ROW_W(GLOBAL_ROW_W)
  ) u_system (
    .clk(router_clk),
    .grad_engine_rst_n(grad_engine_rst_n_s),
    .segment_idx_i(segment_idx_system_s),
    .segment_l_i(segment_l_system_s),
    .segment_r_i(segment_r_system_s),
    .segment_w_i(segment_w_system_s),
    .segment_coeff_i(segment_coeff_system_s),
    .segment_type_i(segment_type_system_s),
    .mode_overflow_i(mode_overflow_system_s),
    .mode_overflow_valid_i(mode_overflow_valid_system_s),
    .mode_wlvia_i(mode_wlvia_system_s),
    .bank_valid_len_i(bank_valid_len_engine_s),
    .readback_start_i(readback_start_s),
    .readback_is_grad_i(readback_is_grad_s),
    .readback_req_valid_i(readback_req_valid_s),
    .readback_req_ready_o(readback_req_ready_s),
    .readback_req_addr_i(readback_req_addr_s),
    .readback_resp_valid_o(readback_resp_valid_s),
    .readback_p_data_o(readback_p_data_s),
    .readback_grad_p_data_o(readback_grad_p_data_s),
    .readback_advance_i(readback_advance_s),
    .grad_overflow_host_ready_o(grad_overflow_host_ready_o),
    .grad_overflow_fifo_warning_o(grad_overflow_fifo_warning_o),
    .dispatcher_fifo_overflow_o(dispatcher_fifo_overflow_s),
    .grad_wlvia_fifo_warning_o(grad_wlvia_fifo_warning_o),
    .grad_p_finish_o(grad_p_finish_o),
    .grad_p_task_ready_o(grad_p_task_ready_s),
    .gradp_debug_core_o(gradp_debug_core_s),
    .gradp_debug_bank_o(gradp_debug_bank_s),
    .gradp_debug_trace_o(gradp_debug_trace_s),
    .stall_probe_o(stall_probe_s),
    .usr_irq_o(usr_irq_o),
    .phase_o(system_phase_s),
    .softmax_enable_o(softmax_enable_o),
    .softmax_finish_o(softmax_finish_o),
    .ddr_rd_req_valid_o(ddr_rd_req_valid_s),
    .ddr_rd_req_ready_i(ddr_rd_req_ready_s),
    .ddr_rd_req_row_o(ddr_rd_req_row_s),
    .ddr_rd_req_count_o(ddr_rd_req_count_s),
    .ddr_rd_resp_valid_i(ddr_rd_resp_valid_s),
    .ddr_rd_resp_ready_o(ddr_rd_resp_ready_s),
    .ddr_rd_resp_bank_mask_i(ddr_rd_resp_bank_mask_s),
    .ddr_rd_resp_last_i(ddr_rd_resp_last_s),
    .ddr_rd_logits_i(ddr_rd_logits_s),
    .ddr_rd_rms_i(ddr_rd_rms_s),
    .ddr_wr_valid_o(ddr_wr_valid_s),
    .ddr_wr_p_line_o(ddr_wr_p_line_s),
    .ddr_wr_ready_i(ddr_wr_ready_s),
    .ddr_wr_row_o(ddr_wr_row_s),
    .ddr_wr_count_o(ddr_wr_count_s),
    .ddr_wr_logits_o(ddr_wr_logits_s),
    .ddr_wr_rms_o(ddr_wr_rms_s),
    .ddr_wr_lane_mask_o(ddr_wr_lane_mask_s),
    // ---- Blocked-GRADP v2 ----
    .blk_cur_half_i(blk_cur_half_s),
    .blk_play_half_i(blk_play_half_s),
    .blk_xwin_base_i(blk_xwin_base_s),
    .blk_p_row_base_i(blk_p_row_base_s),
    .blk_real_cnt_i(blk_real_cnt_s),
    .blk_bank_valid_len_i(blk_bank_valid_len_s),
    .blk_update_active_i(blk_update_active_s),
    .blk_load_active_i(blk_load_active_s),
    .blk_pre_load_done_i(blk_pre_load_done_s),
    .blk_iter_done_i(blk_eng_iter_done_s),
    .blk_go_i(blk_go_s),
    .ld_grad_win_wr_ce_i(ld_grad_win_wr_ce_s),
    .ld_grad_win_wr_addr_i(ld_grad_win_wr_addr_s),
    .ld_grad_win_wr_data_i(ld_grad_win_wr_data_s),
    .ld_grad_win_rd_req_i(ld_grad_win_rd_req_s),
    .ld_grad_win_rd_addr_i(ld_grad_win_rd_addr_s),
    .ld_grad_win_rd_grant_o(ld_grad_win_rd_grant_s),
    .ld_grad_win_rd_data_o(ld_grad_win_rd_data_s),
    .ld_pwin_wr_ce_i(ld_pwin_wr_ce_s),
    .ld_pwin_wr_addr_i(ld_pwin_wr_addr_s),
    .ld_pwin_wr_data_i(ld_pwin_wr_data_s),
    .ld_gflag_wr_ce_i(ld_gflag_wr_ce_s),
    .ld_gflag_wr_addr_i(ld_gflag_wr_addr_s),
    .ld_gflag_wr_special_i(ld_gflag_wr_special_s),
    .ld_gflag_wr_data_i(ld_gflag_wr_data_s),
    .ld_xwin_rd_req_i(ld_xwin_rd_req_s),
    .ld_xwin_rd_addr_i(ld_xwin_rd_addr_s),
    .ld_xwin_rd_data_o(ld_xwin_rd_data_s),
    .blk_pre_seg_idx_o(blk_pre_seg_idx_s),
    .blk_pre_seg_active_o(blk_pre_seg_active_s),
    .blk_pre_seg_count_i(blk_pre_seg_count_s),
    .blk_iter_start_o(),
    .blk_softmax_all_finish_o(blk_softmax_all_finish_s),
    .blk_p_asm_drained_o(blk_p_asm_drained_s),
    .eng_probe_o(eng_probe_s),
    .blk_gradp_finish_o(),
    .blk_p_wr_strobe_o(blk_p_wr_strobe_s),
    .blk_rd_row_base_i(seg_logits_base_s[GLOBAL_ROW_W-1:0]),
    .blk_wr_row_base_i(seg_logits_base_s[GLOBAL_ROW_W-1:0]),
    .meta_rd_req_valid_i(meta_rd_req_valid_s),
    .meta_rd_req_ready_o(meta_rd_req_ready_s),
    .meta_rd_req_row_i(meta_rd_req_row_s),
    .meta_rd_req_count_i(meta_rd_req_count_s),
    .meta_rd_resp_valid_o(meta_rd_resp_valid_s),
    .meta_rd_resp_ready_i(meta_rd_resp_ready_s),
    .meta_rd_resp_last_o(meta_rd_resp_last_s),
    .meta_rd_resp_lane_o(meta_rd_resp_lane_s),
    .blk_wr_phantom_pop_o(blk_wr_phantom_pop_s)
  );

  // Blocked-GRADP v2 block controller.
  generate
    if (BLOCKED_SCHED) begin : GEN_BLOCKED_CTRL
      blocked_ctrl #(
        .BANK_COUNT(BANK_COUNT),
        .P_REGIONS(P_REGIONS),
        .WIN_ROWS(WIN_ROWS),
        .ROW_W(GLOBAL_ROW_W),
        .P_DATA_W(P_DATA_W),
        .MEM_GRAD_P_W(MEM_GRAD_P_W),
        .GROUP_FLAG_WORD_W(GROUP_FLAG_WORD_W)
      ) u_blocked_ctrl (
        .clk(router_clk),
        .rst_n(grad_engine_rst_n_s),
        .go_i(sr_go_s),
        .go_iters_i(sr_go_iters_s),
        .system_phase_i(system_phase_s),
        .cf_lane_valid_i(cf_lane_valid_s),
        .cf_lane_data_i(cf_lane_data_s),
        .cf_lane_pop_o(cf_lane_pop_s),
        .desc_wr_valid_i(blk_desc_wr_valid_s),
        .desc_wr_addr_i(blk_desc_wr_addr_s),
        .desc_wr_data_i(blk_desc_wr_data_s),
        .desc_rd_addr_i(blk_desc_rd_addr_s),
        .desc_rd_data_o(blk_desc_rd_data_s),
        .block_done_i(blk_block_done_s),
        .sr_play_o(sr_play_s),
        .sr_play_last_o(sr_play_last_s),
        .sr_play_ack_i(sr_play_ack_s),
        .sr_done_o(sr_done_s),
        .sr_running_o(sr_running_s),
        .sr_iter_cnt_o(sr_iter_cnt_s),
        .sr_error_o(sr_error_s),
        .sr_cur_blk_o(sr_cur_blk_s),
        .sr_mn_state_o(sr_mn_state_s),
        .sr_upd_probe_o(sr_upd_probe_s),
        .sr_ld_state_o(sr_ld_state_s),
        .sr_ld_blk_o(sr_ld_blk_s),
        .sr_ld_done_valid_o(sr_ld_done_valid_s),
        .sr_ld_row_cnt_o(sr_ld_row_cnt_s),
        .sr_ld_probe2_o(sr_ld_probe2_s),
        .spill_done_o(blk_spill_done_s),
        .iter_done_o(blk_iter_done_s),
        .cur_half_o(blk_cur_half_s),
        .play_half_o(blk_play_half_s),
        .pre_seg_idx_i(blk_pre_seg_idx_s),
        .pre_seg_active_i(blk_pre_seg_active_s),
        .pre_load_done_o(blk_pre_load_done_s),
        .dbg_load_kick_i(dbg_load_kick_s),
        .dbg_load_blk_i(dbg_load_blk_s),
        .dbg_load_half_i(dbg_load_half_s),
        .pre_seg_count_o(blk_pre_seg_count_s),
        .real_cnt_o(blk_real_cnt_s),
        .xwin_base_o(blk_xwin_base_s),
        .p_base_o(blk_p_row_base_s),
        .bank_valid_len_o(blk_bank_valid_len_s),
        .engine_update_active_o(blk_update_active_s),
        .engine_load_active_o(blk_load_active_s),
        .engine_iter_done_o(blk_eng_iter_done_s),
        .softmax_all_finish_i(blk_softmax_all_finish_s),
        .p_asm_drained_i(blk_p_asm_drained_s),
        .p_wr_strobe_i(blk_p_wr_strobe_s),
        .wr_phantom_pop_i(blk_wr_phantom_pop_s),
        .upd_phantom_seen_o(blk_upd_phantom_seen_s),
        .ld_grad_win_wr_ce_o(ld_grad_win_wr_ce_s),
        .ld_grad_win_wr_addr_o(ld_grad_win_wr_addr_s),
        .ld_grad_win_wr_data_o(ld_grad_win_wr_data_s),
        .ld_grad_win_rd_req_o(ld_grad_win_rd_req_s),
        .ld_grad_win_rd_addr_o(ld_grad_win_rd_addr_s),
        .ld_grad_win_rd_grant_i(ld_grad_win_rd_grant_s),
        .ld_grad_win_rd_data_i(ld_grad_win_rd_data_s),
        .ld_pwin_wr_ce_o(ld_pwin_wr_ce_s),
        .ld_pwin_wr_addr_o(ld_pwin_wr_addr_s),
        .ld_pwin_wr_data_o(ld_pwin_wr_data_s),
        .ld_gflag_wr_ce_o(ld_gflag_wr_ce_s),
        .ld_gflag_wr_addr_o(ld_gflag_wr_addr_s),
        .ld_gflag_wr_data_o(ld_gflag_wr_data_s),
        .ld_gflag_wr_special_o(ld_gflag_wr_special_s),
        .ld_xwin_rd_req_o(ld_xwin_rd_req_s),
        .ld_xwin_rd_addr_o(ld_xwin_rd_addr_s),
        .ld_xwin_rd_data_i(ld_xwin_rd_data_s),
        .meta_rd_req_valid_o(meta_rd_req_valid_s),
        .meta_rd_req_ready_i(meta_rd_req_ready_s),
        .meta_rd_req_row_o(meta_rd_req_row_s),
        .meta_rd_req_count_o(meta_rd_req_count_s),
        .meta_rd_resp_valid_i(meta_rd_resp_valid_s),
        .meta_rd_resp_ready_o(meta_rd_resp_ready_s),
        .meta_rd_resp_lane_i(meta_rd_resp_lane_s),
        .seg_logits_base_o(seg_logits_base_s)
      );

      // NET-on-DDR self-run stream fetcher: reads the image directory on
      // GO and streams NET frames (to the adapter) and ctrl lanes (to
      // blocked_ctrl) from DDR via the bridge raw-fetch channel
      net_fetcher #(
        .BLOCK_SLOTS(64),
        .DIRECTORY_WORDS(16)
      ) u_net_fetcher (
        .clk(router_clk),
        .rst_n(grad_engine_rst_n_s),
        .go_i(sr_go_s),
        .dbg_kick_i(dbg_load_kick_s),
        .dbg_sec_i(dbg_load_blk_s),
        .fr_req_valid_o(fr_req_valid_s),
        .fr_req_ready_i(fr_req_ready_s),
        .fr_req_line_o(fr_req_line_s),
        .fr_req_count_o(fr_req_count_s),
        .fr_resp_valid_i(fr_resp_valid_s),
        .fr_resp_ready_o(fr_resp_ready_s),
        .fr_resp_data_i(fr_resp_data_s),
        .fr_resp_last_i(fr_resp_last_s),
        .nf_frame_valid_o(nf_frame_valid_s),
        .nf_frame_data_o(nf_frame_data_s),
        .nf_frame_last_o(nf_frame_last_s),
        .nf_frame_pop_i(nf_frame_pop_s),
        .cf_lane_valid_o(cf_lane_valid_s),
        .cf_lane_data_o(cf_lane_data_s),
        .cf_lane_pop_i(cf_lane_pop_s),
        .dbg_f_state_o(nf_dbg_f_state_s),
        .dbg_cf_sec_o(nf_dbg_cf_sec_s),
        .dbg_nf_sec_o(nf_dbg_nf_sec_s),
        .dbg_cf_line_o(nf_dbg_cf_line_s),
        .dbg_nf_line_o(nf_dbg_nf_line_s),
        .dbg_flight_o(nf_dbg_flight_s),
        .nf_perf_probe_o(nf_perf_probe_s)
      );
    end
  endgenerate

  // Read-side slot ring: one w512_d64 BRAM per OT slot (logits slots 0..N-1
  // plus one init-staging slot; rms slots only for read descriptors). Same
  // IP as before, no new BRAM IP required.
  for (genvar slot_idx = 0; slot_idx < BRIDGE_LOGITS_SLOTS; slot_idx++) begin : GEN_BRIDGE_LOGITS_LINE_BRAM
    logic rd_ce [0:0];
    logic [BRIDGE_LINE_ADDR_W-1:0] rd_addr [0:0];
    logic signed [DDR_AXI_DATA_W-1:0] rd_data [0:0];
    logic wr_ce [0:0];
    logic [BRIDGE_LINE_ADDR_W-1:0] wr_addr [0:0];
    logic signed [DDR_AXI_DATA_W-1:0] wr_data [0:0];
    assign rd_ce[0] = bridge_rd_logits_line_rd_ce_s[slot_idx];
    assign rd_addr[0] = bridge_rd_logits_line_rd_addr_s[slot_idx];
    assign bridge_rd_logits_line_rd_data_s[slot_idx] = rd_data[0];
    assign wr_ce[0] = bridge_rd_logits_line_wr_ce_s[slot_idx];
    assign wr_addr[0] = bridge_rd_logits_line_wr_addr_s[slot_idx];
    assign wr_data[0] = bridge_rd_logits_line_wr_data_s[slot_idx];
    sram_1r1w #(
      .DATA_W(DDR_AXI_DATA_W),
      .BANK_COUNT(1),
      .BANK_DEPTH(BRIDGE_LINE_DEPTH)
    ) u_bridge_rd_logits_line_bram (
      .clk(router_clk),
      .rd_ce_i(rd_ce),
      .rd_addr_i(rd_addr),
      .rd_data_o(rd_data),
      .wr_ce_i(wr_ce),
      .wr_addr_i(wr_addr),
      .wr_data_i(wr_data)
    );
  end

  for (genvar slot_idx = 0; slot_idx < BRIDGE_RMS_SLOTS; slot_idx++) begin : GEN_BRIDGE_RMS_LINE_BRAM
    logic rd_ce [0:0];
    logic [BRIDGE_LINE_ADDR_W-1:0] rd_addr [0:0];
    logic signed [DDR_AXI_DATA_W-1:0] rd_data [0:0];
    logic wr_ce [0:0];
    logic [BRIDGE_LINE_ADDR_W-1:0] wr_addr [0:0];
    logic signed [DDR_AXI_DATA_W-1:0] wr_data [0:0];
    assign rd_ce[0] = bridge_rd_rms_line_rd_ce_s[slot_idx];
    assign rd_addr[0] = bridge_rd_rms_line_rd_addr_s[slot_idx];
    assign bridge_rd_rms_line_rd_data_s[slot_idx] = rd_data[0];
    assign wr_ce[0] = bridge_rd_rms_line_wr_ce_s[slot_idx];
    assign wr_addr[0] = bridge_rd_rms_line_wr_addr_s[slot_idx];
    assign wr_data[0] = bridge_rd_rms_line_wr_data_s[slot_idx];
    sram_1r1w #(
      .DATA_W(DDR_AXI_DATA_W),
      .BANK_COUNT(1),
      .BANK_DEPTH(BRIDGE_LINE_DEPTH)
    ) u_bridge_rd_rms_line_bram (
      .clk(router_clk),
      .rd_ce_i(rd_ce),
      .rd_addr_i(rd_addr),
      .rd_data_o(rd_data),
      .wr_ce_i(wr_ce),
      .wr_addr_i(wr_addr),
      .wr_data_i(wr_data)
    );
  end

  sram_1r1w #(
    .DATA_W(DATA_W),
    .BANK_COUNT(BANK_COUNT),
    .BANK_DEPTH(BRIDGE_BATCH_DEPTH)
  ) u_bridge_wr_logits_batch_bram (
    .clk(router_clk),
    .rd_ce_i(bridge_wr_logits_batch_rd_ce_s),
    .rd_addr_i(bridge_wr_logits_batch_rd_addr_s),
    .rd_data_o(bridge_wr_logits_batch_rd_data_s),
    .wr_ce_i(bridge_wr_logits_batch_wr_ce_s),
    .wr_addr_i(bridge_wr_logits_batch_wr_addr_s),
    .wr_data_i(bridge_wr_logits_batch_wr_data_s)
  );

  sram_1r1w #(
    .DATA_W(RMS_V_W),
    .BANK_COUNT(BANK_COUNT),
    .BANK_DEPTH(BRIDGE_BATCH_DEPTH)
  ) u_bridge_wr_rms_batch_bram (
    .clk(router_clk),
    .rd_ce_i(bridge_wr_rms_batch_rd_ce_s),
    .rd_addr_i(bridge_wr_rms_batch_rd_addr_s),
    .rd_data_o(bridge_wr_rms_batch_rd_data_s),
    .wr_ce_i(bridge_wr_rms_batch_wr_ce_s),
    .wr_addr_i(bridge_wr_rms_batch_wr_addr_s),
    .wr_data_i(bridge_wr_rms_batch_wr_data_s)
  );

  initial begin
    assert (CORE_COUNT_PER_FIFO == 1)
      else $error(1, "router: CORE_COUNT_PER_FIFO must be 1");
    assert (SEGMENT_PER_FIFO == 1)
      else $error(1, "router: SEGMENT_PER_FIFO must be 1");
    assert (DDR_AXI_DATA_W == 512)
      else $error(1, "router bridge BRAM IP expects 512-bit AXI data");
    assert (BRIDGE_LINE_DEPTH == 64)
      else $error(1, "router bridge line BRAM IP expects depth 64");
    assert (BRIDGE_BATCH_DEPTH == 512)
      else $error(1, "router bridge batch BRAM IP expects depth 512");
    assert (DATA_W == 32)
      else $error(1, "router bridge logits batch BRAM IP expects 32-bit data");
    assert (RMS_V_W == 64)
      else $error(1, "router bridge RMS batch BRAM IP expects 64-bit data");
    assert ((BANK_COUNT == 8) || (BANK_COUNT == 16))
      else $error(1, "router DDR row layout supports 8 or 16 softmax banks");
    assert (LOGITS_PER_DDR_LINE >= BANK_COUNT)
      else $error(1, "router logits DDR row layout requires one 512-bit line per logical row");
    assert (RMS_PER_DDR_LINE == 8)
      else $error(1, "router RMS DDR row layout expects 8 64-bit RMS values per 512-bit line");
    assert (RMS_HALF_COUNT == ((BANK_COUNT == 8) ? 1 : 2))
      else $error(1, "router RMS DDR row layout has an invalid physical-row count");
  end

  assign update_softmax_phase_s = BLOCKED_SCHED && blk_update_active_s;
  assign ddr_rd_req_valid_o = ddr_rd_req_valid_s;
  assign ddr_rd_req_ready_o = ddr_rd_req_ready_s;
  assign ddr_rd_req_row_o = ddr_rd_req_row_s;
  assign ddr_rd_req_count_o = ddr_rd_req_count_s;
  assign ddr_rd_resp_valid_o = ddr_rd_resp_valid_s;
  assign ddr_rd_resp_ready_o = ddr_rd_resp_ready_s;
  assign ddr_wr_valid_o = ddr_wr_valid_s;
  assign ddr_wr_ready_o = ddr_wr_ready_s;
  assign ddr_wr_row_o = ddr_wr_row_s;
  assign ddr_wr_count_o = ddr_wr_count_s;
  assign bridge_state_o = bridge_state_q;
  // packed bridge/fetch probes for the adapter debug word (self-run bring-up)
  assign bridge_probe_s = {
    3'b0,
    nf_dbg_flight_s,        // [12]
    fr_req_ready_s,         // [11]
    fr_req_valid_s,         // [10]
    m_axi_rready,           // [9]
    m_axi_rvalid,           // [8]
    m_axi_arready,          // [7]
    m_axi_arvalid,          // [6]
    ddr_rd_resp_valid_s,
    ddr_rd_req_ready_s,     // [4]
    ddr_rd_req_valid_s,     // [3]
    bridge_state_q          // [2:0] (5-state enum fits in 3..0 is 5b -> use [7:3]?)
  };
  assign bridge_mode_o = bridge_mode_q;
  assign bridge_bank_o = bridge_bank_q;
  assign bridge_row_o = bridge_row_idx_q;
  assign bridge_target_is_rms_o = bridge_target_is_rms_q;
  assign ddr_wr_pipe_valid_s = (ddr_wr_pipe_entry_count_q != '0);
  assign ddr_wr_pipe_push_s = ddr_wr_valid_s && ddr_wr_ready_s;

  // ---- read-side OT: grant / enqueue / response-routing wires ----
  // Read quiescence = no descriptor queued, no burst in flight, nothing
  // delivering. Used by init (host phase, strictest ordering).
  assign bridge_read_quiescent_s =
    (brq_level_q == '0) && (bcq_level_q == '0) && !m_axi_arvalid &&
    !iss_active_q && (bdq_level_q == '0) && (del_state_q == DEL_IDLE);

  assign bridge_ddr_active_o = ddr_init_done_q &&
    (
      (bridge_state_q != BRIDGE_IDLE) ||
      init_line_valid_s ||
      softmax_enable_o ||
      ddr_rd_req_valid_s ||
      ddr_wr_pipe_valid_s ||
      ddr_wr_valid_s ||
      (brq_level_q != '0) ||
      (bcq_level_q != '0) ||
      iss_active_q ||
      (bdq_level_q != '0) ||
      (del_state_q != DEL_IDLE)
    );

  // init keeps the highest priority and its exact old accept conditions;
  // the read side must additionally be fully quiescent when the FIRST line
  // is accepted (during INIT_COLLECT read ARs cannot issue anyway).
  assign bridge_init_can_accept_s =
    PCIE_BRAM_ENABLE &&
    ddr_init_done_q &&
    init_line_valid_s &&
    ((bridge_state_q == BRIDGE_IDLE) || (bridge_state_q == BRIDGE_INIT_COLLECT)) &&
    (bridge_init_collect_count_q < AXI_BURST_LINE_W'(AXI_BURST_LINES)) &&
    !ddr_rd_req_valid_s &&
    !ddr_wr_pipe_valid_s &&
    ((bridge_state_q == BRIDGE_INIT_COLLECT) || bridge_read_quiescent_s);
  assign bridge_choose_init_s = bridge_init_can_accept_s;

  // Write batch grant: pending pipe entries + no in-flight read burst
  // (completion queue drained, no AR mid-handshake). A descriptor sitting
  // half-expanded in the issuer (iss_active_q) must NOT block the grant:
  // its remaining ARs are already gated on the pipe draining, so requiring
  // !iss_active_q here would deadlock (read waits for pipe, pipe waits for
  // issuer). Read delivery may still be running - its data is already
  // on-chip, so write bursts safely overlap read response delivery.
  assign bridge_choose_write_s =
    ddr_init_done_q &&
    (
      ((bridge_state_q == BRIDGE_IDLE) && ddr_wr_pipe_valid_s &&
        (bcq_level_q == '0) && !m_axi_arvalid) ||
      (ddr_wr_pipe_valid_s && (bridge_state_q == BRIDGE_WR_COLLECT) && (bridge_collect_count_q < bridge_batch_count_q))
    );

  // Engine read enqueue: whenever the descriptor FIFO has space. AR issue
  // is gated separately (write pipe drained + write FSM idle), which keeps
  // today's read/write ordering barrier at burst granularity.
  assign ddr_rd_req_ready_s =
    ddr_init_done_q && (brq_level_q < BRIDGE_RD_FIFO_CNT_W'(BRIDGE_RD_FIFO_DEPTH));

  // NET-on-DDR raw fetch: lowest-priority read - enqueue only when the read
  // FIFO is empty, no engine read is pending and no write batch waits
  // (same conditions as the old bridge_choose_fetch_s).
  assign fr_req_ready_s =
    ddr_init_done_q &&
    (brq_level_q == '0) &&
    !ddr_rd_req_valid_s &&
    !ddr_wr_pipe_valid_s;

  // descriptor FIFO enqueue fires
  assign brq_push_engine_s = ddr_rd_req_valid_s && ddr_rd_req_ready_s;
  assign brq_push_fetch_s = fr_req_valid_s && fr_req_ready_s;

  // R channel: in-order completion queue; rready back-pressures only when
  // the head burst is a fetch whose FIFO is full (same as the old FETCH_R).
  assign m_axi_rready =
    (bcq_level_q != '0) &&
    ((bcq_dest_q[bcq_rd_ptr_q] != BCQ_DEST_FETCH) || fr_resp_ready_s);
  assign recv_r_fire_s = m_axi_rvalid && m_axi_rready;
  assign recv_r_last_s = ((recv_beat_q + 1'b1) >= bcq_beats_q[bcq_rd_ptr_q]);

  // raw fetch response stream (net_fetcher FIFO push side)
  assign fr_resp_valid_s =
    recv_r_fire_s && (bcq_dest_q[bcq_rd_ptr_q] == BCQ_DEST_FETCH);
  assign fr_resp_data_s = m_axi_rdata;
  assign fr_resp_last_s = fr_resp_valid_s && recv_r_last_s;

  assign ddr_wr_ready_s = ddr_init_done_q && (ddr_wr_pipe_entry_count_q < DDR_WR_PIPE_DEPTH);
  assign init_line_ready_s = bridge_init_can_accept_s;
  assign bridge_write_accept_s = ddr_wr_pipe_valid_s && bridge_choose_write_s;
  assign ddr_wr_pipe_pop_s = bridge_write_accept_s;

  assign m_axi_awid = '0;
  assign m_axi_awlen = axi_len_for_line_count(bridge_active_bank_line_count_q);
  assign m_axi_awsize = DDR_AXI_SIZE;
  assign m_axi_awburst = 2'b01;
  assign m_axi_awlock = 1'b0;
  assign m_axi_awcache = 4'b0011;
  assign m_axi_awprot = 3'b000;
  assign m_axi_awqos = 4'b0000;
  assign m_axi_arid = '0;
  assign m_axi_arsize = DDR_AXI_SIZE;
  assign m_axi_arburst = 2'b01;
  assign m_axi_arlock = 1'b0;
  assign m_axi_arcache = 4'b0011;
  assign m_axi_arprot = 3'b000;
  assign m_axi_arqos = 4'b0000;

  function automatic logic [7:0] axi_len_for_line_count(
    input logic [AXI_BURST_LINE_W-1:0] line_count
  );
    int unsigned beat_count;
    begin
      beat_count = int'(line_count);
      if (beat_count == 0) begin
        axi_len_for_line_count = 8'd0;
      end else if (beat_count >= 256) begin
        axi_len_for_line_count = 8'hff;
      end else begin
        axi_len_for_line_count = beat_count - 1;
      end
    end
  endfunction

  function automatic logic [AXI_BURST_LINE_W-1:0] line_count_for_rows(
    input logic [DDR_ROW_BATCH_W-1:0] row_count
  );
    begin
      if (row_count == '0) begin
        line_count_for_rows = '0;
      end else if (row_count > DDR_ROW_BATCH_W'(AXI_BURST_LINES)) begin
        line_count_for_rows = AXI_BURST_LINE_W'(AXI_BURST_LINES);
      end else begin
        line_count_for_rows = AXI_BURST_LINE_W'(row_count);
      end
    end
  endfunction

  function automatic logic [AXI_BURST_LINE_W-1:0] rms_line_count_for_rows(
    input logic [DDR_ROW_BATCH_W-1:0] row_count
  );
    int unsigned line_count;
    begin
      line_count = int'(row_count) * RMS_HALF_COUNT;
      if (line_count > AXI_BURST_LINES) begin
        line_count = AXI_BURST_LINES;
      end
      rms_line_count_for_rows = AXI_BURST_LINE_W'(line_count);
    end
  endfunction

  function automatic logic [DDR_ROW_BATCH_W-1:0] min_rms_chunk_count(
    input logic [DDR_ROW_BATCH_W-1:0] row_base,
    input logic [DDR_ROW_BATCH_W-1:0] total_rows
  );
    logic [DDR_ROW_BATCH_W-1:0] rows_left;
    begin
      if (row_base >= total_rows) begin
        min_rms_chunk_count = '0;
      end else begin
        rows_left = total_rows - row_base;
        if (rows_left > DDR_ROW_BATCH_W'(RMS_LOGICAL_ROWS_PER_BURST)) begin
          min_rms_chunk_count = DDR_ROW_BATCH_W'(RMS_LOGICAL_ROWS_PER_BURST);
        end else begin
          min_rms_chunk_count = rows_left;
        end
      end
    end
  endfunction

  function automatic logic [DDR_WR_PIPE_PTR_W-1:0] ddr_wr_pipe_ptr_inc(
    input logic [DDR_WR_PIPE_PTR_W-1:0] ptr
  );
    if (ptr == (DDR_WR_PIPE_DEPTH - 1)) begin
      return '0;
    end
    return ptr + 1'b1;
  endfunction

  function automatic logic [DDR_AXI_ADDR_W-1:0] logits_addr_for(
    input logic [GLOBAL_ROW_W-1:0] row_idx
  );
    begin
      logits_addr_for = LOGITS_DRAM_BASE_ADDR + (int'(row_idx) * DDR_LINE_BYTES);
    end
  endfunction

  function automatic logic [DDR_AXI_ADDR_W-1:0] rms_addr_for(
    input logic [GLOBAL_ROW_W-1:0] row_idx,
    input int unsigned half_idx
  );
    int unsigned addr_idx;
    begin
      addr_idx = (int'(row_idx) * RMS_HALF_COUNT) + half_idx;
      rms_addr_for = RMS_DRAM_BASE_ADDR + (addr_idx * DDR_LINE_BYTES);
    end
  endfunction

  function automatic logic signed [DATA_W-1:0] extract_logits_lane(
    input logic [DDR_AXI_DATA_W-1:0] line_data,
    input int unsigned bank_idx
  );
    begin
      extract_logits_lane = line_data[(bank_idx * DATA_W) +: DATA_W];
    end
  endfunction

  function automatic logic signed [RMS_V_W-1:0] extract_rms_lane(
    input logic [DDR_AXI_DATA_W-1:0] line_data,
    input int unsigned bank_idx
  );
    int unsigned lane_idx;
    begin
      lane_idx = bank_idx % RMS_PER_DDR_LINE;
      extract_rms_lane = line_data[(lane_idx * RMS_V_W) +: RMS_V_W];
    end
  endfunction

  always_comb begin
    ddr_wr_pipe_row_s = '0;
    ddr_wr_pipe_count_s = '0;
    ddr_wr_pipe_lane_mask_s = '0;
    for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
      ddr_wr_pipe_logits_s[bank_idx] = '0;
      ddr_wr_pipe_rms_s[bank_idx] = '0;
    end

    if (ddr_wr_pipe_valid_s) begin
      ddr_wr_pipe_row_s = ddr_wr_pipe_row_q[ddr_wr_pipe_rd_ptr_q];
      ddr_wr_pipe_p_line_s = ddr_wr_pipe_p_line_q[ddr_wr_pipe_rd_ptr_q];
      ddr_wr_pipe_count_s = ddr_wr_pipe_count_q[ddr_wr_pipe_rd_ptr_q];
      ddr_wr_pipe_lane_mask_s = ddr_wr_pipe_lane_mask_q[ddr_wr_pipe_rd_ptr_q];
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        ddr_wr_pipe_logits_s[bank_idx] = ddr_wr_pipe_logits_q[ddr_wr_pipe_rd_ptr_q][bank_idx];
        ddr_wr_pipe_rms_s[bank_idx] = ddr_wr_pipe_rms_q[ddr_wr_pipe_rd_ptr_q][bank_idx];
      end
    end
  end

  always_comb begin
    int unsigned build_logical_offset;
    int unsigned build_half;

    bridge_wdata_s = '0;
    bridge_wstrb_s = '1;
    bridge_bank_has_write_s = 1'b0;
    bridge_bank_line_count_s = bridge_burst_line_count_q;

    for (int slot_idx = 0; slot_idx < BRIDGE_LOGITS_SLOTS; slot_idx++) begin
      bridge_rd_logits_line_rd_ce_s[slot_idx] = 1'b0;
      bridge_rd_logits_line_rd_addr_s[slot_idx] = '0;
      bridge_rd_logits_line_wr_ce_s[slot_idx] = 1'b0;
      bridge_rd_logits_line_wr_addr_s[slot_idx] = '0;
      bridge_rd_logits_line_wr_data_s[slot_idx] = '0;
    end
    for (int slot_idx = 0; slot_idx < BRIDGE_RMS_SLOTS; slot_idx++) begin
      bridge_rd_rms_line_rd_ce_s[slot_idx] = 1'b0;
      bridge_rd_rms_line_rd_addr_s[slot_idx] = '0;
      bridge_rd_rms_line_wr_ce_s[slot_idx] = 1'b0;
      bridge_rd_rms_line_wr_addr_s[slot_idx] = '0;
      bridge_rd_rms_line_wr_data_s[slot_idx] = '0;
    end

    for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
      bridge_wr_logits_batch_rd_ce_s[bank_idx] = 1'b0;
      bridge_wr_logits_batch_rd_addr_s[bank_idx] = '0;
      bridge_wr_logits_batch_wr_ce_s[bank_idx] = 1'b0;
      bridge_wr_logits_batch_wr_addr_s[bank_idx] = '0;
      bridge_wr_logits_batch_wr_data_s[bank_idx] = '0;
      bridge_wr_rms_batch_rd_ce_s[bank_idx] = 1'b0;
      bridge_wr_rms_batch_rd_addr_s[bank_idx] = '0;
      bridge_wr_rms_batch_wr_ce_s[bank_idx] = 1'b0;
      bridge_wr_rms_batch_wr_addr_s[bank_idx] = '0;
      bridge_wr_rms_batch_wr_data_s[bank_idx] = '0;
    end

    // R-beat landing: head-of-completion-queue burst writes its slot BRAM
    // (fetch beats bypass the BRAMs entirely, see fr_resp_*)
    if (recv_r_fire_s && (bcq_dest_q[bcq_rd_ptr_q] == BCQ_DEST_LOGITS)) begin
      bridge_rd_logits_line_wr_ce_s[bcq_slot_q[bcq_rd_ptr_q]] = 1'b1;
      bridge_rd_logits_line_wr_addr_s[bcq_slot_q[bcq_rd_ptr_q]] =
        BRIDGE_LINE_ADDR_W'(int'(bcq_off_q[bcq_rd_ptr_q]) + int'(recv_beat_q));
      bridge_rd_logits_line_wr_data_s[bcq_slot_q[bcq_rd_ptr_q]] = m_axi_rdata;
    end
    if (recv_r_fire_s && (bcq_dest_q[bcq_rd_ptr_q] == BCQ_DEST_RMS)) begin
      int unsigned rms_wr_line;
      int unsigned rms_wr_inst;
      rms_wr_line = int'(bcq_off_q[bcq_rd_ptr_q]) + int'(recv_beat_q);
      rms_wr_inst =
        (int'(bcq_slot_q[bcq_rd_ptr_q]) * RMS_HALF_COUNT) + (rms_wr_line / AXI_BURST_LINES);
      bridge_rd_rms_line_wr_ce_s[rms_wr_inst] = 1'b1;
      bridge_rd_rms_line_wr_addr_s[rms_wr_inst] =
        BRIDGE_LINE_ADDR_W'(rms_wr_line % AXI_BURST_LINES);
      bridge_rd_rms_line_wr_data_s[rms_wr_inst] = m_axi_rdata;
    end

    if (bridge_init_can_accept_s) begin
      bridge_rd_logits_line_wr_ce_s[BRIDGE_INIT_SLOT] = 1'b1;
      bridge_rd_logits_line_wr_addr_s[BRIDGE_INIT_SLOT] = BRIDGE_LINE_ADDR_W'(bridge_init_collect_count_q);
      bridge_rd_logits_line_wr_data_s[BRIDGE_INIT_SLOT] = init_line_data_s;
    end

    if (bridge_state_q == BRIDGE_INIT_W_READ) begin
      bridge_rd_logits_line_rd_ce_s[BRIDGE_INIT_SLOT] = 1'b1;
      bridge_rd_logits_line_rd_addr_s[BRIDGE_INIT_SLOT] = BRIDGE_LINE_ADDR_W'(bridge_axi_beat_idx_q);
    end

    // delivery BRAM read (1-cycle latency, consumed in DEL_RESP)
    if (del_state_q == DEL_PREP) begin
      int unsigned del_rms_line;
      int unsigned del_rms_inst;
      bridge_rd_logits_line_rd_ce_s[del_slot_q] = 1'b1;
      bridge_rd_logits_line_rd_addr_s[del_slot_q] = BRIDGE_LINE_ADDR_W'(bridge_resp_index_q);
      if (del_need_rms_q) begin
        del_rms_line =
          (int'(bridge_resp_index_q) * RMS_HALF_COUNT) + int'(bridge_resp_half_q);
        del_rms_inst =
          (int'(del_slot_q) * RMS_HALF_COUNT) + (del_rms_line / AXI_BURST_LINES);
        bridge_rd_rms_line_rd_ce_s[del_rms_inst] = 1'b1;
        bridge_rd_rms_line_rd_addr_s[del_rms_inst] =
          BRIDGE_LINE_ADDR_W'(del_rms_line % AXI_BURST_LINES);
      end
    end

    if (bridge_wr_stage_valid_q) begin
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        bridge_wr_logits_batch_wr_ce_s[bank_idx] = 1'b1;
        bridge_wr_logits_batch_wr_addr_s[bank_idx] = bridge_wr_stage_addr_q;
        bridge_wr_logits_batch_wr_data_s[bank_idx] = bridge_wr_stage_logits_q[bank_idx];
        bridge_wr_rms_batch_wr_ce_s[bank_idx] = 1'b1;
        bridge_wr_rms_batch_wr_addr_s[bank_idx] = bridge_wr_stage_addr_q;
        bridge_wr_rms_batch_wr_data_s[bank_idx] = bridge_wr_stage_rms_q[bank_idx];
      end
    end

    if (
      (bridge_state_q == BRIDGE_W_BUILD_READ) &&
      (bridge_mode_q == BRIDGE_MODE_SOFT_WR)
    ) begin
      build_half = int'(bridge_axi_beat_idx_q) % RMS_HALF_COUNT;
      build_logical_offset = bridge_target_is_rms_q
        ? (int'(bridge_rms_chunk_base_q) + (int'(bridge_axi_beat_idx_q) / RMS_HALF_COUNT))
        : int'(bridge_axi_beat_idx_q);
      if (bridge_target_is_rms_q) begin
        for (int lane_idx = 0; lane_idx < RMS_PER_DDR_LINE; lane_idx++) begin
          int bank_idx;
          bank_idx = (build_half * RMS_PER_DDR_LINE) + lane_idx;
          bridge_wr_rms_batch_rd_ce_s[bank_idx] = 1'b1;
          bridge_wr_rms_batch_rd_addr_s[bank_idx] = BRIDGE_BATCH_ADDR_W'(build_logical_offset);
        end
      end else begin
        for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
          bridge_wr_logits_batch_rd_ce_s[bank_idx] = 1'b1;
          bridge_wr_logits_batch_rd_addr_s[bank_idx] = BRIDGE_BATCH_ADDR_W'(build_logical_offset);
        end
      end
    end

    if (bridge_mode_q == BRIDGE_MODE_INIT) begin
      bridge_bank_line_count_s = bridge_burst_line_count_q;
      bridge_bank_has_write_s = 1'b1;
    end else if (bridge_mode_q == BRIDGE_MODE_SOFT_WR) begin
      bridge_bank_has_write_s = 1'b1;
      if (bridge_target_is_rms_q) begin
        bridge_bank_line_count_s = rms_line_count_for_rows(bridge_rms_chunk_count_q);
      end else begin
        bridge_bank_line_count_s = line_count_for_rows(bridge_batch_count_q);
      end
    end
  end

  // delivery BRAM read-data mux (valid in DEL_RESP, one cycle after DEL_PREP)
  assign bridge_del_logits_data_s = bridge_rd_logits_line_rd_data_s[del_slot_q];
  assign bridge_del_rms_data_s =
    bridge_rd_rms_line_rd_data_s[(int'(del_slot_q) * RMS_HALF_COUNT) + int'(del_rms_inst_q)];

  always_ff @(posedge router_clk) begin
    if (!bridge_reset_s && (bridge_mode_q == BRIDGE_MODE_SOFT_WR) && m_axi_wvalid && m_axi_wready) begin
      assert (m_axi_wstrb == {DDR_AXI_STRB_W{1'b1}})
        else $fatal(1, "router ddr_bridge: softmax write-back attempted a narrow AXI burst");
    end
  end

  // ---------------------------------------------------------------------
  // Read-side multi-outstanding pipeline
  //
  //   descriptor FIFO (brq) -> burst issuer (AR) -> MIG -> completion queue
  //   (bcq, positional pairing: all ARs share AXI ID 0 so R beats return in
  //   issue order) -> response slot ring -> in-order delivery FSM
  //
  // Ordering rules (preserve the old single-outstanding external semantics):
  //   - ARs issue only while the write pipe is empty AND the write FSM is
  //     IDLE: a read never overtakes pending/in-flight writes (RAW barrier,
  //     same as the old !ddr_wr_pipe_valid_s term in bridge_choose_read_s).
  //   - A write batch grants only with the completion queue empty and no AR
  //     mid-handshake: a write never overtakes in-flight reads (WAR).
  //   - Delivery walks the slot ring in issue order, so the engine sees the
  //     same per-row beat sequence as before; fetch beats keep streaming
  //     straight into the fetcher FIFO.
  // ---------------------------------------------------------------------

  // descriptor FIFO (engine reads + fetches; engine wins via the fetch gate)
  always_ff @(posedge router_clk) begin
    if (bridge_reset_s) begin
      brq_rd_ptr_q <= '0;
      brq_wr_ptr_q <= '0;
      brq_level_q <= '0;
      for (int fifo_idx = 0; fifo_idx < BRIDGE_RD_FIFO_DEPTH; fifo_idx++) begin
        brq_kind_q[fifo_idx] <= 1'b0;
        brq_base_q[fifo_idx] <= '0;
        brq_count_q[fifo_idx] <= '0;
        brq_need_rms_q[fifo_idx] <= 1'b0;
      end
    end else begin
      if (brq_push_engine_s) begin
        brq_kind_q[brq_wr_ptr_q] <= 1'b0;
        brq_base_q[brq_wr_ptr_q] <= 32'(ddr_rd_req_row_s);
        brq_count_q[brq_wr_ptr_q] <= 9'(ddr_rd_req_count_s);
        brq_need_rms_q[brq_wr_ptr_q] <= update_softmax_phase_s;
      end else if (brq_push_fetch_s) begin
        brq_kind_q[brq_wr_ptr_q] <= 1'b1;
        brq_base_q[brq_wr_ptr_q] <= fr_req_line_s;
        brq_count_q[brq_wr_ptr_q] <= fr_req_count_s;
        brq_need_rms_q[brq_wr_ptr_q] <= 1'b0;
      end
      if (brq_push_engine_s || brq_push_fetch_s) begin
        brq_wr_ptr_q <= brq_wr_ptr_q + 1'b1;
      end
      if (iss_pop_s) begin
        brq_rd_ptr_q <= brq_rd_ptr_q + 1'b1;
      end
      unique case ({(brq_push_engine_s || brq_push_fetch_s), iss_pop_s})
        2'b10: brq_level_q <= brq_level_q + 1'b1;
        2'b01: brq_level_q <= brq_level_q - 1'b1;
        default: brq_level_q <= brq_level_q;
      endcase
    end
  end

  // delivery queue (engine descriptors in issue order; fetch skips it)
  assign bdq_push_s = iss_pop_s && !brq_kind_q[brq_rd_ptr_q];
  always_ff @(posedge router_clk) begin
    if (bridge_reset_s) begin
      bdq_rd_ptr_q <= '0;
      bdq_wr_ptr_q <= '0;
      bdq_level_q <= '0;
      for (int fifo_idx = 0; fifo_idx < BRIDGE_RD_OT_SLOTS; fifo_idx++) begin
        bdq_slot_q[fifo_idx] <= '0;
        bdq_row_q[fifo_idx] <= '0;
        bdq_count_q[fifo_idx] <= '0;
        bdq_need_rms_q[fifo_idx] <= 1'b0;
      end
    end else begin
      if (bdq_push_s) begin
        bdq_slot_q[bdq_wr_ptr_q] <= alloc_ptr_q;
        bdq_row_q[bdq_wr_ptr_q] <= GLOBAL_ROW_W'(brq_base_q[brq_rd_ptr_q]);
        bdq_count_q[bdq_wr_ptr_q] <= brq_count_q[brq_rd_ptr_q];
        bdq_need_rms_q[bdq_wr_ptr_q] <= brq_need_rms_q[brq_rd_ptr_q];
        bdq_wr_ptr_q <= bdq_wr_ptr_q + 1'b1;
      end
      if (bdq_pop_s) begin
        bdq_rd_ptr_q <= bdq_rd_ptr_q + 1'b1;
      end
      unique case ({bdq_push_s, bdq_pop_s})
        2'b10: bdq_level_q <= bdq_level_q + 1'b1;
        2'b01: bdq_level_q <= bdq_level_q - 1'b1;
        default: bdq_level_q <= bdq_level_q;
      endcase
    end
  end

  // slot ring state (FILL at pop, READY at last landed burst, DELIVER at
  // delivery start, FREE at delivery done; events never collide on a slot)
  always_ff @(posedge router_clk) begin
    if (bridge_reset_s) begin
      for (int slot_idx = 0; slot_idx < BRIDGE_RD_OT_SLOTS; slot_idx++) begin
        slot_state_q[slot_idx] <= SLOT_FREE;
      end
    end else begin
      if (bdq_pop_s) begin
        slot_state_q[del_slot_q] <= SLOT_FREE;
      end
      if (del_start_s) begin
        slot_state_q[bdq_slot_q[bdq_rd_ptr_q]] <= SLOT_DELIVER;
      end
      if (recv_slot_ready_s) begin
        slot_state_q[bcq_slot_q[bcq_rd_ptr_q]] <= SLOT_READY;
      end
      if (bdq_push_s) begin
        slot_state_q[alloc_ptr_q] <= SLOT_FILL;
      end
    end
  end

  // current-burst combinational view of the descriptor under expansion
  always_comb begin
    logic [AXI_BURST_LINE_W-1:0] cur_lines;
    iss_cur_is_fetch_s = iss_kind_q;
    iss_cur_is_logits_s = !iss_kind_q && !iss_logits_done_q;
    cur_lines = '0;
    iss_cur_addr_s = '0;
    iss_cur_off_s = '0;
    iss_cur_slot_last_s = 1'b0;
    if (iss_kind_q) begin
      // fetch: one burst, streamed straight to the fetcher FIFO
      cur_lines = AXI_BURST_LINE_W'(iss_count_q);
      iss_cur_addr_s = NET_IMAGE_DRAM_BASE_ADDR + (int'(iss_base_q) * DDR_LINE_BYTES);
    end else if (!iss_logits_done_q) begin
      cur_lines = line_count_for_rows(DDR_ROW_BATCH_W'(iss_count_q));
      iss_cur_addr_s = logits_addr_for(GLOBAL_ROW_W'(iss_base_q));
      iss_cur_slot_last_s = !iss_need_rms_q;
    end else begin
      cur_lines = (iss_rms_left_q > 8'(AXI_BURST_LINES))
        ? AXI_BURST_LINE_W'(AXI_BURST_LINES)
        : AXI_BURST_LINE_W'(iss_rms_left_q);
      iss_cur_addr_s = rms_addr_for(
        GLOBAL_ROW_W'(iss_base_q) +
          GLOBAL_ROW_W'(int'(iss_rms_chunk_q) * RMS_LOGICAL_ROWS_PER_BURST),
        0
      );
      iss_cur_off_s = iss_rms_chunk_q * 8'(AXI_BURST_LINES);
      iss_cur_slot_last_s = (iss_rms_left_q <= 8'(AXI_BURST_LINES));
    end
    iss_cur_lines_s = 8'(cur_lines);
    iss_cur_len_s = axi_len_for_line_count(cur_lines);
    iss_cur_beats_s = iss_cur_len_s + 8'd1;

    // descriptor pop: write side quiescent (pipe empty + write FSM IDLE)
    // and, for engine reads, a free response slot
    iss_pop_s =
      !iss_active_q && (brq_level_q != '0) &&
      (bridge_state_q == BRIDGE_IDLE) && !ddr_wr_pipe_valid_s &&
      (brq_kind_q[brq_rd_ptr_q] || (slot_state_q[alloc_ptr_q] == SLOT_FREE));
  end

  // burst issuer: expand the popped descriptor into back-to-back AR bursts
  always_ff @(posedge router_clk) begin
    if (bridge_reset_s) begin
      iss_active_q <= 1'b0;
      iss_kind_q <= 1'b0;
      iss_base_q <= '0;
      iss_count_q <= '0;
      iss_need_rms_q <= 1'b0;
      iss_logits_done_q <= 1'b0;
      iss_rms_left_q <= '0;
      iss_rms_chunk_q <= '0;
      iss_slot_q <= '0;
      alloc_ptr_q <= '0;
      m_axi_arvalid <= 1'b0;
      m_axi_araddr <= '0;
      m_axi_arlen <= '0;
    end else begin
      if (iss_pop_s) begin
        assert (brq_count_q[brq_rd_ptr_q] != '0)
          else $fatal(1, "router ddr_bridge: zero-count read descriptor");
        iss_active_q <= 1'b1;
        iss_kind_q <= brq_kind_q[brq_rd_ptr_q];
        iss_base_q <= brq_base_q[brq_rd_ptr_q];
        iss_count_q <= brq_count_q[brq_rd_ptr_q];
        iss_need_rms_q <= brq_need_rms_q[brq_rd_ptr_q];
        iss_logits_done_q <= 1'b0;
        iss_rms_left_q <= brq_need_rms_q[brq_rd_ptr_q]
          ? 8'(int'(brq_count_q[brq_rd_ptr_q]) * RMS_HALF_COUNT)
          : '0;
        iss_rms_chunk_q <= '0;
        iss_slot_q <= alloc_ptr_q;
        if (!brq_kind_q[brq_rd_ptr_q]) begin
          alloc_ptr_q <= alloc_ptr_q + 1'b1;
        end
      end

      if (!m_axi_arvalid) begin
        if (
          iss_active_q && (bridge_state_q == BRIDGE_IDLE) && !ddr_wr_pipe_valid_s &&
          (bcq_level_q < BRIDGE_CQ_CNT_W'(BRIDGE_CQ_DEPTH))
        ) begin
          m_axi_arvalid <= 1'b1;
          m_axi_araddr <= iss_cur_addr_s;
          m_axi_arlen <= iss_cur_len_s;
        end
      end else if (m_axi_arready) begin
        m_axi_arvalid <= 1'b0;
        if (iss_kind_q) begin
          iss_active_q <= 1'b0;
        end else if (!iss_logits_done_q) begin
          iss_logits_done_q <= 1'b1;
          if (!iss_need_rms_q) begin
            iss_active_q <= 1'b0;
          end
        end else begin
          iss_rms_left_q <= iss_rms_left_q - iss_cur_lines_s;
          iss_rms_chunk_q <= iss_rms_chunk_q + 1'b1;
          if (iss_rms_left_q <= 8'(AXI_BURST_LINES)) begin
            iss_active_q <= 1'b0;
          end
        end
      end
    end
  end

  // completion queue + R receiver (positional pairing, in-order by AXI ID)
  assign bcq_push_s = m_axi_arvalid && m_axi_arready;
  assign recv_slot_ready_s =
    recv_r_fire_s && recv_r_last_s && bcq_slot_last_q[bcq_rd_ptr_q];
  always_ff @(posedge router_clk) begin
    if (bridge_reset_s) begin
      bcq_rd_ptr_q <= '0;
      bcq_wr_ptr_q <= '0;
      bcq_level_q <= '0;
      recv_beat_q <= '0;
      for (int fifo_idx = 0; fifo_idx < BRIDGE_CQ_DEPTH; fifo_idx++) begin
        bcq_dest_q[fifo_idx] <= BCQ_DEST_LOGITS;
        bcq_slot_q[fifo_idx] <= '0;
        bcq_off_q[fifo_idx] <= '0;
        bcq_beats_q[fifo_idx] <= '0;
        bcq_slot_last_q[fifo_idx] <= 1'b0;
      end
    end else begin
      if (bcq_push_s) begin
        bcq_dest_q[bcq_wr_ptr_q] <= iss_cur_is_fetch_s
          ? BCQ_DEST_FETCH
          : (iss_cur_is_logits_s ? BCQ_DEST_LOGITS : BCQ_DEST_RMS);
        bcq_slot_q[bcq_wr_ptr_q] <= iss_slot_q;
        bcq_off_q[bcq_wr_ptr_q] <= iss_cur_off_s;
        bcq_beats_q[bcq_wr_ptr_q] <= iss_cur_beats_s;
        bcq_slot_last_q[bcq_wr_ptr_q] <= iss_cur_slot_last_s && !iss_cur_is_fetch_s;
        bcq_wr_ptr_q <= bcq_wr_ptr_q + 1'b1;
      end
      if (recv_r_fire_s) begin
        if (recv_r_last_s) begin
          recv_beat_q <= '0;
          bcq_rd_ptr_q <= bcq_rd_ptr_q + 1'b1;
        end else begin
          recv_beat_q <= recv_beat_q + 1'b1;
        end
      end
      unique case ({bcq_push_s, (recv_r_fire_s && recv_r_last_s)})
        2'b10: bcq_level_q <= bcq_level_q + 1'b1;
        2'b01: bcq_level_q <= bcq_level_q - 1'b1;
        default: bcq_level_q <= bcq_level_q;
      endcase
    end
  end

  // delivery FSM: per-row RESP beats from the slot ring, in request order.
  // Ported from the retired BRIDGE_RESP_PREP/BRIDGE_RESP states; the rms
  // chunk re-fetch chain is gone (a slot now holds every rms line of the
  // request before delivery starts).
  assign del_start_s =
    (del_state_q == DEL_IDLE) && (bdq_level_q != '0) &&
    (slot_state_q[bdq_slot_q[bdq_rd_ptr_q]] == SLOT_READY);
  assign bdq_pop_s =
    (del_state_q == DEL_RESP) && ddr_rd_resp_valid_s && ddr_rd_resp_ready_s &&
    !(del_need_rms_q && (RMS_HALF_COUNT > 1) && !bridge_resp_half_q) &&
    ((bridge_resp_index_q + 1'b1) >= DDR_ROW_BATCH_W'(del_count_q));
  always_ff @(posedge router_clk) begin
    if (bridge_reset_s) begin
      del_state_q <= DEL_IDLE;
      del_slot_q <= '0;
      del_row_base_q <= '0;
      del_count_q <= '0;
      del_need_rms_q <= 1'b0;
      del_rms_inst_q <= 1'b0;
      bridge_resp_index_q <= '0;
      bridge_resp_half_q <= 1'b0;
      ddr_rd_resp_valid_s <= 1'b0;
      ddr_rd_resp_bank_mask_s <= '0;
      ddr_rd_resp_last_s <= 1'b0;
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        ddr_rd_logits_s[bank_idx] <= '0;
        ddr_rd_rms_s[bank_idx] <= '0;
      end
    end else begin
      if (ddr_rd_resp_valid_s && ddr_rd_resp_ready_s) begin
        ddr_rd_resp_valid_s <= 1'b0;
      end
      case (del_state_q)
        DEL_IDLE: begin
          if (del_start_s) begin
            del_slot_q <= bdq_slot_q[bdq_rd_ptr_q];
            del_row_base_q <= bdq_row_q[bdq_rd_ptr_q];
            del_count_q <= bdq_count_q[bdq_rd_ptr_q];
            del_need_rms_q <= bdq_need_rms_q[bdq_rd_ptr_q];
            bridge_resp_index_q <= '0;
            bridge_resp_half_q <= 1'b0;
            del_state_q <= DEL_PREP;
          end
        end

        DEL_PREP: begin
          ddr_rd_resp_valid_s <= 1'b0;
          del_rms_inst_q <=
            ((((int'(bridge_resp_index_q) * RMS_HALF_COUNT) +
               int'(bridge_resp_half_q)) / AXI_BURST_LINES) != 0);
          del_state_q <= DEL_RESP;
        end

        DEL_RESP: begin
          if (!ddr_rd_resp_valid_s) begin
            logic [GLOBAL_ROW_W-1:0] resp_row;
            resp_row = del_row_base_q + GLOBAL_ROW_W'(bridge_resp_index_q);
            ddr_rd_resp_bank_mask_s <= '0;
            ddr_rd_resp_last_s <=
              !del_need_rms_q || (RMS_HALF_COUNT == 1) || bridge_resp_half_q;
            for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
              logic bank_in_resp_half;
              bank_in_resp_half = !del_need_rms_q ||
                ((bank_idx / RMS_PER_DDR_LINE) == int'(bridge_resp_half_q));
              if ((resp_row < bank_valid_len_system_s[bank_idx]) && bank_in_resp_half) begin
                ddr_rd_resp_bank_mask_s[bank_idx] <= 1'b1;
                ddr_rd_logits_s[bank_idx] <= extract_logits_lane(
                  bridge_del_logits_data_s,
                  bank_idx
                );
                ddr_rd_rms_s[bank_idx] <= del_need_rms_q
                  ? extract_rms_lane(bridge_del_rms_data_s, bank_idx)
                  : '0;
              end else begin
                ddr_rd_logits_s[bank_idx] <= '0;
                ddr_rd_rms_s[bank_idx] <= '0;
              end
            end
            ddr_rd_resp_valid_s <= 1'b1;
          end else if (ddr_rd_resp_ready_s) begin
            if (del_need_rms_q && (RMS_HALF_COUNT > 1) && !bridge_resp_half_q) begin
              ddr_rd_resp_valid_s <= 1'b0;
              bridge_resp_half_q <= 1'b1;
              del_state_q <= DEL_PREP;
            end else if ((bridge_resp_index_q + 1'b1) >= DDR_ROW_BATCH_W'(del_count_q)) begin
              ddr_rd_resp_valid_s <= 1'b0;
              bridge_resp_index_q <= '0;
              bridge_resp_half_q <= 1'b0;
              del_state_q <= DEL_IDLE;
            end else begin
              ddr_rd_resp_valid_s <= 1'b0;
              bridge_resp_index_q <= bridge_resp_index_q + 1'b1;
              bridge_resp_half_q <= 1'b0;
              del_state_q <= DEL_PREP;
            end
          end
        end

        default: begin
          del_state_q <= DEL_IDLE;
        end
      endcase
    end
  end

  always_ff @(posedge router_clk) begin
    if (bridge_reset_s) begin
      bridge_state_q <= BRIDGE_IDLE;
      bridge_mode_q <= BRIDGE_MODE_NONE;
      bridge_target_is_rms_q <= 1'b0;
      bridge_target_is_image_q <= 1'b0;
      bridge_need_rms_q <= 1'b0;
      bridge_row_idx_q <= '0;
      bridge_batch_row_base_q <= '0;
      bridge_batch_count_q <= '0;
      bridge_rms_chunk_base_q <= '0;
      bridge_rms_chunk_count_q <= '0;
      bridge_collect_count_q <= '0;
      bridge_burst_line_count_q <= '0;
      bridge_active_bank_line_count_q <= '0;
      bridge_axi_beat_idx_q <= '0;
      bridge_bank_q <= '0;
      ddr_wr_pipe_rd_ptr_q <= '0;
      ddr_wr_pipe_wr_ptr_q <= '0;
      ddr_wr_pipe_entry_count_q <= '0;
      bridge_wr_stage_valid_q <= 1'b0;
      bridge_wr_stage_addr_q <= '0;
      bridge_init_collect_count_q <= '0;
      bridge_init_line_base_q <= '0;
      bridge_init_command_last_q <= 1'b0;
      m_axi_awvalid <= 1'b0;
      m_axi_wvalid <= 1'b0;
      m_axi_wlast <= 1'b0;
      m_axi_bready <= 1'b0;
      m_axi_awaddr <= '0;
      m_axi_wdata <= '0;
      m_axi_wstrb <= '0;
      for (int pipe_idx = 0; pipe_idx < DDR_WR_PIPE_DEPTH; pipe_idx++) begin
        ddr_wr_pipe_row_q[pipe_idx] <= '0;
        ddr_wr_pipe_p_line_q[pipe_idx] <= '0;
        ddr_wr_pipe_p_line_q[pipe_idx] <= '0;
        ddr_wr_pipe_count_q[pipe_idx] <= '0;
        ddr_wr_pipe_lane_mask_q[pipe_idx] <= '0;
        for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
          ddr_wr_pipe_logits_q[pipe_idx][bank_idx] <= '0;
          ddr_wr_pipe_rms_q[pipe_idx][bank_idx] <= '0;
        end
      end
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        bridge_wr_stage_logits_q[bank_idx] <= '0;
        bridge_wr_stage_rms_q[bank_idx] <= '0;
      end
      for (int row_offset = 0; row_offset < AXI_BURST_ROW_SLOTS; row_offset++) begin
        bridge_wr_lane_mask_batch_q[row_offset] <= '0;
      end
    end else begin
      bridge_wr_stage_valid_q <= 1'b0;
      if (ddr_wr_pipe_push_s) begin
        ddr_wr_pipe_row_q[ddr_wr_pipe_wr_ptr_q] <= ddr_wr_row_s;
        ddr_wr_pipe_count_q[ddr_wr_pipe_wr_ptr_q] <= DDR_ROW_BATCH_W'(ddr_wr_count_s);
        ddr_wr_pipe_lane_mask_q[ddr_wr_pipe_wr_ptr_q] <= ddr_wr_lane_mask_s;
        ddr_wr_pipe_p_line_q[ddr_wr_pipe_wr_ptr_q] <= ddr_wr_p_line_s;
        for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
          ddr_wr_pipe_logits_q[ddr_wr_pipe_wr_ptr_q][bank_idx] <= ddr_wr_logits_s[bank_idx];
          ddr_wr_pipe_rms_q[ddr_wr_pipe_wr_ptr_q][bank_idx] <= ddr_wr_rms_s[bank_idx];
        end
      end
      if (ddr_wr_pipe_push_s) begin
        ddr_wr_pipe_wr_ptr_q <= ddr_wr_pipe_ptr_inc(ddr_wr_pipe_wr_ptr_q);
      end
      if (ddr_wr_pipe_pop_s) begin
        ddr_wr_pipe_rd_ptr_q <= ddr_wr_pipe_ptr_inc(ddr_wr_pipe_rd_ptr_q);
      end
      unique case ({ddr_wr_pipe_push_s, ddr_wr_pipe_pop_s})
        2'b10: ddr_wr_pipe_entry_count_q <= ddr_wr_pipe_entry_count_q + DDR_WR_PIPE_CNT_W'(1);
        2'b01: ddr_wr_pipe_entry_count_q <= ddr_wr_pipe_entry_count_q - DDR_WR_PIPE_CNT_W'(1);
        default: ddr_wr_pipe_entry_count_q <= ddr_wr_pipe_entry_count_q;
      endcase

      case (bridge_state_q)
        BRIDGE_IDLE: begin
          m_axi_awvalid <= 1'b0;
          m_axi_wvalid <= 1'b0;
          m_axi_wlast <= 1'b0;
          m_axi_bready <= 1'b0;
          m_axi_wstrb <= '0;
          if (ddr_init_done_q) begin
            if (bridge_choose_init_s) begin
              bridge_mode_q <= BRIDGE_MODE_INIT;
              bridge_row_idx_q <= GLOBAL_ROW_W'(init_line_index_s);
              bridge_batch_row_base_q <= GLOBAL_ROW_W'(init_line_index_s);
              bridge_batch_count_q <= DDR_ROW_BATCH_W'(1);
              bridge_burst_line_count_q <= AXI_BURST_LINE_W'(1);
              bridge_axi_beat_idx_q <= '0;
              bridge_init_collect_count_q <= AXI_BURST_LINE_W'(1);
              bridge_init_line_base_q <= init_line_index_s;
              bridge_init_command_last_q <= init_line_last_s;
              bridge_bank_q <= '0;
              bridge_target_is_rms_q <= init_line_is_rms_s;
              bridge_target_is_image_q <= init_line_is_image_s;
              bridge_need_rms_q <= 1'b0;
              if (init_line_last_s || (AXI_BURST_LINES == 1)) begin
                bridge_state_q <= BRIDGE_AW_PREP;
              end else begin
                bridge_state_q <= BRIDGE_INIT_COLLECT;
              end
            end else if (bridge_choose_write_s) begin
              bridge_mode_q <= BRIDGE_MODE_SOFT_WR;
              bridge_row_idx_q <= ddr_wr_pipe_row_s;
              bridge_batch_row_base_q <= ddr_wr_pipe_row_s;
              bridge_batch_count_q <= ddr_wr_pipe_count_s;
              bridge_burst_line_count_q <= line_count_for_rows(ddr_wr_pipe_count_s);
              bridge_axi_beat_idx_q <= '0;
              bridge_collect_count_q <= DDR_ROW_BATCH_W'(1);
              bridge_bank_q <= '0;
              bridge_target_is_rms_q <= 1'b0;
              bridge_need_rms_q <= update_softmax_phase_s && !ddr_wr_pipe_p_line_s;
              bridge_rms_chunk_base_q <= '0;
              bridge_rms_chunk_count_q <= min_rms_chunk_count('0, ddr_wr_pipe_count_s);
              bridge_wr_lane_mask_batch_q[0] <= ddr_wr_pipe_lane_mask_s;
              bridge_wr_stage_valid_q <= 1'b1;
              bridge_wr_stage_addr_q <= '0;
              for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
                bridge_wr_stage_logits_q[bank_idx] <= ddr_wr_pipe_logits_s[bank_idx];
                bridge_wr_stage_rms_q[bank_idx] <= ddr_wr_pipe_rms_s[bank_idx];
              end
              for (int row_offset = 1; row_offset < AXI_BURST_ROW_SLOTS; row_offset++) begin
                bridge_wr_lane_mask_batch_q[row_offset] <= '0;
              end
              if (ddr_wr_pipe_count_s <= 1) begin
                bridge_state_q <= BRIDGE_AW_PREP;
              end else begin
                bridge_state_q <= BRIDGE_WR_COLLECT;
              end
            end
          end
        end

        BRIDGE_INIT_COLLECT: begin
          if (bridge_init_can_accept_s) begin
            if (bridge_init_collect_count_q == '0) begin
              bridge_init_line_base_q <= init_line_index_s;
              bridge_target_is_rms_q <= init_line_is_rms_s;
              bridge_target_is_image_q <= init_line_is_image_s;
            end else begin
              assert (init_line_is_rms_s == bridge_target_is_rms_q)
                else $fatal(1, "router ddr_bridge: mixed targets in one PCIe initialization burst");
              assert (init_line_is_image_s == bridge_target_is_image_q)
                else $fatal(1, "router ddr_bridge: mixed image targets in one PCIe initialization burst");
              assert (init_line_index_s == (bridge_init_line_base_q + bridge_init_collect_count_q))
                else $fatal(1, "router ddr_bridge: non-contiguous PCIe initialization lines");
            end
            if (init_line_last_s || ((bridge_init_collect_count_q + 1'b1) >= AXI_BURST_LINE_W'(AXI_BURST_LINES))) begin
              bridge_burst_line_count_q <= bridge_init_collect_count_q + 1'b1;
              bridge_state_q <= BRIDGE_AW_PREP;
            end
            if (init_line_last_s) begin
              bridge_init_command_last_q <= 1'b1;
            end
            bridge_init_collect_count_q <= bridge_init_collect_count_q + 1'b1;
          end
        end

        BRIDGE_WR_COLLECT: begin
          if (bridge_write_accept_s) begin
            bridge_wr_lane_mask_batch_q[bridge_collect_count_q] <= ddr_wr_pipe_lane_mask_s;
            bridge_wr_stage_valid_q <= 1'b1;
            bridge_wr_stage_addr_q <= BRIDGE_BATCH_ADDR_W'(bridge_collect_count_q);
            for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
              bridge_wr_stage_logits_q[bank_idx] <= ddr_wr_pipe_logits_s[bank_idx];
              bridge_wr_stage_rms_q[bank_idx] <= ddr_wr_pipe_rms_s[bank_idx];
            end
            if ((bridge_collect_count_q + 1'b1) >= bridge_batch_count_q) begin
              bridge_collect_count_q <= '0;
              bridge_bank_q <= '0;
              bridge_state_q <= BRIDGE_AW_PREP;
            end else begin
              bridge_collect_count_q <= bridge_collect_count_q + 1'b1;
            end
          end
        end

        BRIDGE_AW_PREP: begin
          if ((bridge_mode_q == BRIDGE_MODE_SOFT_WR) && (bridge_bank_line_count_s == '0)) begin
            bridge_state_q <= BRIDGE_IDLE;
            bridge_mode_q <= BRIDGE_MODE_NONE;
          end else begin
            bridge_active_bank_line_count_q <= bridge_bank_line_count_s;
            bridge_state_q <= BRIDGE_AW;
          end
        end

        BRIDGE_AW: begin
          m_axi_awaddr <= (bridge_target_is_image_q && (bridge_mode_q == BRIDGE_MODE_INIT))
            ? (NET_IMAGE_DRAM_BASE_ADDR + (bridge_init_line_base_q * DDR_LINE_BYTES))
            : (
              bridge_target_is_rms_q
                ? ((bridge_mode_q == BRIDGE_MODE_INIT)
                  ? (RMS_DRAM_BASE_ADDR + (bridge_init_line_base_q * DDR_LINE_BYTES))
                  : rms_addr_for(bridge_batch_row_base_q + GLOBAL_ROW_W'(bridge_rms_chunk_base_q), 0))
                : ((bridge_mode_q == BRIDGE_MODE_INIT)
                  ? (LOGITS_DRAM_BASE_ADDR + (bridge_init_line_base_q * DDR_LINE_BYTES))
                  : logits_addr_for(bridge_batch_row_base_q))
            );
          if (!m_axi_awvalid) begin
            m_axi_awvalid <= 1'b1;
          end else if (m_axi_awready) begin
            m_axi_awvalid <= 1'b0;
            bridge_axi_beat_idx_q <= '0;
            if (bridge_mode_q == BRIDGE_MODE_INIT) begin
              bridge_state_q <= BRIDGE_INIT_W_READ;
            end else begin
              m_axi_wvalid <= 1'b0;
              m_axi_wlast <= 1'b0;
              bridge_state_q <= BRIDGE_W_BUILD_REQ;
            end
          end
        end

        BRIDGE_W_BUILD_REQ: begin
          bridge_state_q <= BRIDGE_W_BUILD_READ;
        end

        BRIDGE_INIT_W_READ: begin
          bridge_state_q <= BRIDGE_INIT_W_CAPTURE;
        end

        BRIDGE_INIT_W_CAPTURE: begin
          m_axi_wvalid <= 1'b1;
          m_axi_wdata <= bridge_rd_logits_line_rd_data_s[BRIDGE_INIT_SLOT];
          m_axi_wstrb <= '1;
          m_axi_wlast <= ((bridge_axi_beat_idx_q + 1'b1) >= bridge_active_bank_line_count_q);
          bridge_state_q <= BRIDGE_W;
        end

        BRIDGE_W_BUILD_READ: begin
          bridge_state_q <= BRIDGE_W_BUILD_CAPTURE;
        end

        BRIDGE_W_BUILD_CAPTURE: begin
          logic [DDR_AXI_DATA_W-1:0] next_line;
          logic [DDR_AXI_STRB_W-1:0] next_strb;
          int unsigned build_logical_offset;
          int unsigned build_half;
          build_half = int'(bridge_axi_beat_idx_q) % RMS_HALF_COUNT;
          build_logical_offset = bridge_target_is_rms_q
            ? (int'(bridge_rms_chunk_base_q) + (int'(bridge_axi_beat_idx_q) / RMS_HALF_COUNT))
            : int'(bridge_axi_beat_idx_q);
          next_line = '0;
          next_strb = '1;
          if (bridge_target_is_rms_q) begin
            for (int lane_idx = 0; lane_idx < RMS_PER_DDR_LINE; lane_idx++) begin
              int bank_idx;
              bank_idx = (build_half * RMS_PER_DDR_LINE) + lane_idx;
              if (bridge_wr_lane_mask_batch_q[build_logical_offset][bank_idx]) begin
                next_line[(lane_idx * RMS_V_W) +: RMS_V_W] =
                  bridge_wr_rms_batch_rd_data_s[bank_idx];
              end
            end
          end else begin
            for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
              if (bridge_wr_lane_mask_batch_q[build_logical_offset][bank_idx]) begin
                next_line[(bank_idx * DATA_W) +: DATA_W] =
                  bridge_wr_logits_batch_rd_data_s[bank_idx];
              end
            end
          end
          m_axi_wvalid <= 1'b1;
          m_axi_wdata <= next_line;
          m_axi_wstrb <= next_strb;
          m_axi_wlast <= ((bridge_axi_beat_idx_q + 1'b1) >= bridge_active_bank_line_count_q);
          bridge_state_q <= BRIDGE_W;
        end

        BRIDGE_W: begin
          if (m_axi_wvalid && m_axi_wready) begin
            if ((bridge_axi_beat_idx_q + 1'b1) >= bridge_active_bank_line_count_q) begin
              m_axi_wvalid <= 1'b0;
              m_axi_wlast <= 1'b0;
              m_axi_bready <= 1'b1;
              bridge_axi_beat_idx_q <= '0;
              bridge_state_q <= BRIDGE_B;
            end else begin
              logic [AXI_BURST_LINE_W-1:0] next_beat_idx;
              next_beat_idx = bridge_axi_beat_idx_q + 1'b1;
              m_axi_wvalid <= 1'b0;
              m_axi_wlast <= 1'b0;
              bridge_axi_beat_idx_q <= next_beat_idx;
              if (bridge_mode_q == BRIDGE_MODE_INIT) begin
                bridge_state_q <= BRIDGE_INIT_W_READ;
              end else begin
                bridge_state_q <= BRIDGE_W_BUILD_REQ;
              end
            end
          end
        end

        BRIDGE_B: begin
          if (m_axi_bvalid && m_axi_bready) begin
            m_axi_bready <= 1'b0;
            bridge_axi_beat_idx_q <= '0;
            if (bridge_mode_q == BRIDGE_MODE_INIT) begin
              bridge_init_collect_count_q <= '0;
              if (bridge_init_command_last_q) begin
                bridge_state_q <= BRIDGE_IDLE;
                bridge_mode_q <= BRIDGE_MODE_NONE;
              end else begin
                bridge_state_q <= BRIDGE_INIT_COLLECT;
              end
            end else begin
              if (!bridge_target_is_rms_q && bridge_need_rms_q) begin
                bridge_target_is_rms_q <= 1'b1;
                bridge_rms_chunk_base_q <= '0;
                bridge_rms_chunk_count_q <= min_rms_chunk_count('0, bridge_batch_count_q);
                bridge_state_q <= BRIDGE_AW_PREP;
              end else if (
                bridge_target_is_rms_q &&
                ((bridge_rms_chunk_base_q + bridge_rms_chunk_count_q) < bridge_batch_count_q)
              ) begin
                logic [DDR_ROW_BATCH_W-1:0] next_chunk_base;
                next_chunk_base = bridge_rms_chunk_base_q + bridge_rms_chunk_count_q;
                bridge_rms_chunk_base_q <= next_chunk_base;
                bridge_rms_chunk_count_q <= min_rms_chunk_count(next_chunk_base, bridge_batch_count_q);
                bridge_state_q <= BRIDGE_AW_PREP;
              end else begin
                bridge_state_q <= BRIDGE_IDLE;
                bridge_mode_q <= BRIDGE_MODE_NONE;
              end
            end
          end
        end

        default: begin
          bridge_state_q <= BRIDGE_IDLE;
        end
      endcase
    end
  end

endmodule

`endif
