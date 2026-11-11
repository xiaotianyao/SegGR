`ifndef FPGA_ROUTER_PCIE_BRAM_ADAPTER_SV
`define FPGA_ROUTER_PCIE_BRAM_ADAPTER_SV

`include "../common/segcore_pkg.sv"

module router_pcie_bram_adapter #(
  parameter int CORE_COUNT = 8,
  parameter int CHUNK_SIZE = 64,
  parameter int CORE_COUNT_PER_FIFO = 1,
  parameter int SEGMENT_PER_FIFO = 1,
  parameter int DATA_W = `SEGCORE_DATA_W,
  parameter int FRAC_W = `SEGCORE_FRAC_W,
  parameter int COEFF_W = `SEGCORE_COEFF_W,
  parameter int P_DATA_W = 16,
  parameter int MEM_GRAD_P_W = 64,
  parameter int RMS_V_W = 64,
  parameter int GROUP_FLAG_WORD_W = 32,
  parameter int BANK_DEPTH = 65536,
  // Blocked-GRADP: NET lane idx23 carries a window slot instead of
  // (bank, mem); see decode_banked_idx23. Inert when 0 (v1.0.5 behavior).
  parameter bit BLOCKED_GRADP = 1'b0,
  // Blocked-GRADP v2: per-block NET gating (block_ready), block-boundary
  // sync (block_done/iter_last), block ctrl lane stream and the block
  // descriptor/segment-init commands. Inert when 0.
  parameter bit BLOCKED_SCHED = 1'b0,
  // NET-on-DDR self-run image region base (INIT_SEG region_sel==3 writes
  // here; the bridge raw-fetch channel reads it back)
  parameter logic [31:0] NET_IMAGE_DRAM_BASE_ADDR = 32'h0200_0000,
  parameter int BRAM_DATA_W = 512,
  parameter int BRAM_DEPTH = 4096,
  parameter int BRAM_ADDR_W = 12,
  parameter int STATIC_WEIGHT_DEPTH = 4096,
  parameter int BRAM_ADA_CMD_FIFO_DEPTH = 512,
  parameter int BRAM_ADA_STATIC_W_FIFO_DEPTH = 128,
  // Correctness (phantom-hunt) probes: V1-V4 sticky captures, mask-set log,
  // pairing counters. 0 = pruned out of the netlist entirely and the
  // M10-M13 mirror words carry the performance layout instead.
  parameter bit PROBE_CORRECTNESS_EN = 1'b0
) (
  input  logic                                                clk,
  input  logic                                                rst_n,
  input  logic                                                enable_i,

  output logic                                                cmd_bram_en_o,
  output logic [(BRAM_DATA_W/8)-1:0]                          cmd_bram_we_o,
  output logic [BRAM_ADDR_W-1:0]                              cmd_bram_addr_o,
  output logic [BRAM_DATA_W-1:0]                              cmd_bram_wdata_o,
  input  logic [BRAM_DATA_W-1:0]                              cmd_bram_rdata_i,

  output logic [0:0]                                          usr_irq_req_o,
  input  logic [0:0]                                          usr_irq_ack_i,

  input  logic                                                system_event_i,
  input  logic [3:0]                                          system_phase_i,
  input  logic                                                grad_p_finish_i,
  input  logic                                                softmax_enable_i,
  input  logic                                                bridge_ddr_active_i,
  input  logic                                                grad_overflow_host_ready_i [0:(CORE_COUNT/CORE_COUNT_PER_FIFO)-1][0:CORE_COUNT_PER_FIFO-1],
  input  logic                                                grad_wlvia_fifo_warning_i [0:(CORE_COUNT/SEGMENT_PER_FIFO)-1],
  input  logic                                                dispatcher_fifo_overflow_i [0:(CORE_COUNT/CORE_COUNT_PER_FIFO)-1][0:CORE_COUNT_PER_FIFO-1],
  input  logic                                                grad_task_ready_i [0:CORE_COUNT-1],
  input  wire logic [15:0]                                    gradp_debug_core_i [0:CORE_COUNT-1],
  input  wire logic [31:0]                                    gradp_debug_bank_i,
  input  wire logic [(CORE_COUNT*13+2)*32-1:0]                gradp_debug_trace_i,
  output logic                                                grad_iteration_done_o,

  output logic                                                readback_start_o,
  output logic                                                readback_is_grad_o,
  output logic                                                readback_req_valid_o,
  input  logic                                                readback_req_ready_i,
  output logic [$clog2(BANK_DEPTH)-1:0]                       readback_req_addr_o,
  input  logic                                                readback_resp_valid_i,
  input  logic signed [P_DATA_W-1:0]                          readback_p_data_i [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  input  logic signed [MEM_GRAD_P_W-1:0]                      readback_grad_p_data_i [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  output logic                                                readback_advance_o,

  output logic                                                init_line_valid_o,
  input  logic                                                init_line_ready_i,
  output logic [31:0]                                         init_line_index_o,
  output logic                                                init_line_is_rms_o,
  // INIT_SEG region_sel==3: the NET/ctrl self-run image region (third DDR
  // base, NET_IMAGE_DRAM_BASE_ADDR)
  output logic                                                init_line_is_image_o,
  output logic                                                init_line_last_o,
  output logic [BRAM_DATA_W-1:0]                              init_line_data_o,
  output logic                                                group_flag_init_valid_o,
  input  logic                                                group_flag_init_ready_i,
  output logic [31:0]                                         group_flag_init_addr_o,
  output logic [GROUP_FLAG_WORD_W-1:0]                        group_flag_init_data_o [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  output logic                                                system_reset_hold_o,
  output logic                                                full_reset_active_o,

  output segcore_pkg::banked_idx_t                            segment_idx_o [0:CORE_COUNT-1],
  output logic [$clog2(CHUNK_SIZE)-1:0]                       segment_l_o [0:CORE_COUNT-1],
  output logic [$clog2(CHUNK_SIZE)-1:0]                       segment_r_o [0:CORE_COUNT-1],
  output logic signed [DATA_W-1:0]                            segment_w_o [0:CORE_COUNT-1],
  output logic signed [COEFF_W-1:0]                           segment_coeff_o [0:CORE_COUNT-1],
  output logic [2:0]                                          segment_type_o [0:CORE_COUNT-1],
  output logic [1:0]                                          mode_overflow_o [0:(CORE_COUNT/CORE_COUNT_PER_FIFO)-1][0:CORE_COUNT_PER_FIFO-1],
  output logic                                                mode_overflow_valid_o [0:(CORE_COUNT/CORE_COUNT_PER_FIFO)-1][0:CORE_COUNT_PER_FIFO-1],
  output logic [2:0]                                          mode_wlvia_o [0:(CORE_COUNT/SEGMENT_PER_FIFO)-1],
  output logic [19:0]                                         bank_valid_len_o [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],

  // ---- Blocked-GRADP v2 (BLOCKED_SCHED=1): blocked_ctrl interface ----
  // descriptor register file write (CMD_LOAD_BLOCK_DESC) / comb read
  // (CMD_INIT_SEG segment base lookup)
  output logic                                                blk_desc_wr_valid_o,
  output logic [10:0]                                         blk_desc_wr_addr_o,
  output logic [31:0]                                         blk_desc_wr_data_o,
  output logic [10:0]                                         blk_desc_rd_addr_o,
  input  logic [31:0]                                         blk_desc_rd_data_i,
  // block-boundary sync: blk_block_done_o (+blk_iter_last_o) pulses when a
  // self-run block play has observed the GRADP drain
  output logic                                                blk_block_done_o,
  output logic                                                blk_iter_last_o,
  input  logic                                                blk_spill_done_i,
  input  logic                                                blk_iter_done_i,
  input  logic                                                blk_upd_phantom_seen_i,
  // ---- NET-on-DDR self-run (CMD_GO + iteration-master play) ----
  // GO launch: N iterations + a one-cycle go to the master and the
  // net_fetcher; blk_go_o kicks the engine phase WAIT_GRADP -> GRADP
  output logic                                                sr_go_o,
  output logic [31:0]                                         sr_go_iters_o,
  output logic                                                blk_go_o,
  // play handshake with the iteration master (blocked_ctrl)
  input  logic                                                sr_play_i,
  input  logic                                                sr_play_last_i,
  output logic                                                sr_play_ack_o,
  // self-run status from the master (debug/WAIT selector 8)
  input  logic                                                sr_done_i,
  input  logic                                                sr_running_i,
  input  logic [31:0]                                         sr_iter_cnt_i,
  // host debug window load kick (CMD_DBG_LOAD_BLOCK; READ_P v4 support)
  output logic                                                dbg_load_kick_o,
  output logic [5:0]                                          dbg_load_blk_o,
  output logic                                                dbg_load_half_o,
  input  logic [7:0]                                          sr_error_i,
  input  logic [5:0]                                          sr_cur_blk_i,
  input  logic [2:0]                                          sr_mn_state_i,
  // self-run bring-up probes (blocked_ctrl loader + net_fetcher cursors)
  input  logic [3:0]                                          sr_ld_state_i,
  input  logic [5:0]                                          sr_ld_blk_i,
  input  logic                                                sr_ld_done_valid_i,
  input  logic [16:0]                                         sr_ld_row_cnt_i,
  input  logic [2:0]                                          nf_dbg_f_state_i,
  input  logic [6:0]                                          nf_dbg_cf_sec_i,
  input  logic [6:0]                                          nf_dbg_nf_sec_i,
  input  logic [31:0]                                         nf_dbg_cf_line_i,
  input  logic [31:0]                                         nf_dbg_nf_line_i,
  input  logic                                                nf_dbg_flight_i,
  input  logic [33:0]                                         eng_probe_i,
  input  logic [19:0]                                         ld_probe2_i,
  input  logic [15:0]                                         bridge_probe_i,
  // update-rail status from blocked_ctrl (performance mirror M13):
  // [0]=upd_active [6:1]=upd_blk [7]=upd_half [8]=upd_pend [9]=0
  input  logic [9:0]                                          sr_upd_probe_i,
  // net_fetcher performance bundle (watermarks / burst stats / full cycles)
  input  logic [223:0]                                        nf_perf_probe_i,
  // grad_p stall probe (P0/P1 stall counters):
  // [3:0]=p req stall [11:4]=p bank grant pulse [19:12]=grad commit pulse
  // [27:20]=update source hold stall
  input  logic [31:0]                                         stall_probe_i,
  // NET frame stream from the net_fetcher (FWFT)
  input  logic                                                sr_frame_valid_i,
  input  logic [255:0]                                        sr_frame_data_i,
  input  logic                                                sr_frame_last_i,
  output logic                                                sr_frame_pop_o,

  output logic                                                busy_o,
  output logic [7:0]                                          error_o
);
  import segcore_pkg::*;

  localparam int ADDR_W = (CHUNK_SIZE <= 1) ? 1 : $clog2(CHUNK_SIZE);
  localparam int GRAD_OVERFLOW_COUNT = CORE_COUNT / CORE_COUNT_PER_FIFO;
  localparam int GRAD_WLVIA_COUNT = CORE_COUNT / SEGMENT_PER_FIFO;
  localparam int BANK_COUNT = GRAD_OVERFLOW_COUNT + GRAD_WLVIA_COUNT;
  localparam int BANK_VALID_LEN_W = 20;
  localparam int MEM_ADDR_W = $clog2(BANK_DEPTH);
  localparam int BRAM_STRB_W = BRAM_DATA_W / 8;
  localparam int SUBMIT_INDEX = BRAM_DEPTH - 1;
  localparam int STATUS_INDEX = 3;
  localparam int EVENT_INDEX = 4;
  localparam int PAYLOAD_BASE_INDEX = 64;
  localparam int LANES_PER_WORD = BRAM_DATA_W / 64;
  localparam int LOGITS_PER_WORD = BRAM_DATA_W / 32;
  localparam int RMS_PER_WORD = BRAM_DATA_W / RMS_V_W;
  localparam int RMS_LINES_PER_ROW = (BANK_COUNT + RMS_PER_WORD - 1) / RMS_PER_WORD;
  localparam int GROUP_FLAG_MEM_DEPTH_RAW =
    (BANK_DEPTH + GROUP_FLAG_WORD_W - 1) / GROUP_FLAG_WORD_W;
  localparam int GROUP_FLAG_MEM_DEPTH =
    (GROUP_FLAG_MEM_DEPTH_RAW < 2) ? 2 : GROUP_FLAG_MEM_DEPTH_RAW;
  localparam int STATIC_WEIGHTS_PER_WORD = BRAM_DATA_W / 32;
  localparam int STATIC_WEIGHT_WORD_DEPTH =
    (STATIC_WEIGHT_DEPTH + STATIC_WEIGHTS_PER_WORD - 1) / STATIC_WEIGHTS_PER_WORD;
  localparam int STATIC_WEIGHT_WORD_ADDR_W =
    (STATIC_WEIGHT_WORD_DEPTH <= 1) ? 1 : $clog2(STATIC_WEIGHT_WORD_DEPTH);
  localparam int STATIC_WEIGHT_LANE_W =
    (STATIC_WEIGHTS_PER_WORD <= 1) ? 1 : $clog2(STATIC_WEIGHTS_PER_WORD);
  localparam int CONFIG_LEN_WORD_COUNT = (BANK_COUNT + 15) / 16;
  localparam int P_ROWS_PER_WORD = BRAM_DATA_W / (BANK_COUNT * P_DATA_W);
  localparam int GRAD_ROWS_PER_WORD = BRAM_DATA_W / (BANK_COUNT * MEM_GRAD_P_W);
  localparam int MAX_READBACK_ROWS_PER_WORD =
    (P_ROWS_PER_WORD > GRAD_ROWS_PER_WORD) ? P_ROWS_PER_WORD : GRAD_ROWS_PER_WORD;
  localparam int READBACK_SLOT_W =
    (MAX_READBACK_ROWS_PER_WORD <= 1) ? 1 : $clog2(MAX_READBACK_ROWS_PER_WORD);
  localparam int BRAM_ADA_CMD_FIFO_WARNING_THR =
    (BRAM_ADA_CMD_FIFO_DEPTH > 16) ? (BRAM_ADA_CMD_FIFO_DEPTH - 16) : BRAM_ADA_CMD_FIFO_DEPTH;
  localparam int BRAM_ADA_STATIC_W_FIFO_WARNING_THR =
    (BRAM_ADA_STATIC_W_FIFO_DEPTH > 8) ? (BRAM_ADA_STATIC_W_FIFO_DEPTH - 8) : BRAM_ADA_STATIC_W_FIFO_DEPTH;

  logic [GRAD_OVERFLOW_COUNT-1:0] dispatcher_fifo_overflow_flat_s;
  always_comb begin
    dispatcher_fifo_overflow_flat_s = '0;
    for (int group_idx = 0; group_idx < GRAD_OVERFLOW_COUNT; group_idx++) begin
      for (int lane_idx = 0; lane_idx < CORE_COUNT_PER_FIFO; lane_idx++) begin
        dispatcher_fifo_overflow_flat_s[group_idx] |= dispatcher_fifo_overflow_i[group_idx][lane_idx];
      end
    end
  end

  localparam logic [31:0] SUBMIT_MAGIC = 32'h0000_55AA;
  localparam logic [15:0] CMD_MAGIC = 16'hC0DE;
  localparam logic [7:0] PROTOCOL_VERSION = 8'd1;

  localparam logic [7:0] CMD_NOP             = 8'h00;
  localparam logic [7:0] CMD_GET_CAPS        = 8'h01;
  localparam logic [7:0] CMD_CONFIG          = 8'h02;
  localparam logic [7:0] CMD_NET             = 8'h03;
  localparam logic [7:0] CMD_WAIT            = 8'h04;
  localparam logic [7:0] CMD_INIT_LOGITS_DDR = 8'h05;
  localparam logic [7:0] CMD_INIT_STATIC_WEIGHTS = 8'h06;
  localparam logic [7:0] CMD_INIT_RMS_DDR    = 8'h07;
  localparam logic [7:0] CMD_INIT_GROUP_FLAGS = 8'h08;
  localparam logic [7:0] CMD_READ_P          = 8'h09;
  localparam logic [7:0] CMD_READ_GRAD_P     = 8'h0A;
  localparam logic [7:0] CMD_DEBUG           = 8'h10;
  localparam logic [7:0] CMD_RESET           = 8'h20;
  // Blocked-GRADP v2 command set
  localparam logic [7:0] CMD_LOAD_BLOCK_CTRL = 8'h30;  // retired (NET-on-DDR)
  localparam logic [7:0] CMD_LOAD_BLOCK_DESC = 8'h31;
  localparam logic [7:0] CMD_INIT_SEG        = 8'h32;
  localparam logic [7:0] CMD_GO              = 8'h33;
  localparam logic [7:0] CMD_DBG_LOAD_BLOCK  = 8'h34;
  localparam logic [7:0] CMD_SET_MIRROR_PAGE = 8'h35;

  localparam logic [7:0] ST_IDLE  = 8'd0;
  localparam logic [7:0] ST_BUSY  = 8'd1;
  localparam logic [7:0] ST_DONE  = 8'd2;
  localparam logic [7:0] ST_ERROR = 8'd3;
  localparam logic [7:0] ST_EVENT = 8'd4;

  localparam logic [7:0] ERR_OK                = 8'd0;
  localparam logic [7:0] ERR_BAD_MAGIC         = 8'd1;
  localparam logic [7:0] ERR_BAD_VERSION       = 8'd2;
  localparam logic [7:0] ERR_UNKNOWN_CMD       = 8'd3;
  localparam logic [7:0] ERR_RANGE_ERROR       = 8'd4;
  localparam logic [7:0] ERR_BUSY              = 8'd5;
  localparam logic [7:0] ERR_BAD_LENGTH        = 8'd6;
  localparam logic [7:0] ERR_ROUTER_NOT_READY  = 8'd7;
  localparam logic [7:0] ERR_TIMEOUT           = 8'd8;
  localparam logic [7:0] ERR_CHECKSUM_FAIL     = 8'd9;
  localparam logic [7:0] ERR_DEBUG_UNSUPPORTED = 8'd10;
  localparam logic [7:0] ERR_FIFO_RESIDUE      = 8'd11;
  localparam logic [7:0] ERR_DISPATCHER_FIFO_OVERFLOW = 8'd12;
  localparam logic [7:0] ERR_UNSUPPORTED_CMD   = 8'd13;

  typedef enum logic [6:0] {
    A_FLAG_ADDR,
    A_FLAG_WAIT0,
    A_FLAG_WAIT1,
    A_FLAG_CHECK,
    A_CLEAR_FLAG,
    A_DESC0_ADDR,
    A_DESC0_WAIT0,
    A_DESC0_WAIT1,
    A_DESC0_CAPTURE,
    A_DESC1_ADDR,
    A_DESC1_WAIT0,
    A_DESC1_WAIT1,
    A_DESC1_CAPTURE,
    A_DESC2_ADDR,
    A_DESC2_WAIT0,
    A_DESC2_WAIT1,
    A_DESC2_CAPTURE,
    A_VALIDATE,
    A_WRITE_BUSY,
    A_DISPATCH,
    A_CAPS_WRITE,
    A_CONFIG_META_ADDR,
    A_CONFIG_META_WAIT0,
    A_CONFIG_META_WAIT1,
    A_CONFIG_META_CAPTURE,
    A_CONFIG_LEN_ADDR,
    A_CONFIG_LEN_WAIT0,
    A_CONFIG_LEN_WAIT1,
    A_CONFIG_LEN_CAPTURE,
    A_CONFIG_WLVIA_ADDR,
    A_CONFIG_WLVIA_WAIT0,
    A_CONFIG_WLVIA_WAIT1,
    A_CONFIG_WLVIA_CAPTURE,
    A_CONFIG_WLVIA_ARM,
    A_CONFIG_WLVIA_ISSUE,
    A_INIT_META_ADDR,
    A_INIT_META_WAIT0,
    A_INIT_META_WAIT1,
    A_INIT_META_CAPTURE,
    A_INIT_DATA_ADDR,
    A_INIT_DATA_WAIT0,
    A_INIT_DATA_WAIT1,
    A_INIT_DATA_CAPTURE,
    A_INIT_EMIT,
    A_GROUP_FLAG_WAIT_READY,
    A_GROUP_FLAG_WAIT0,
    A_GROUP_FLAG_WAIT1,
    A_GROUP_FLAG_STREAM,
    A_GROUP_FLAG_DRAIN,
    A_STATIC_META_ADDR,
    A_STATIC_META_WAIT0,
    A_STATIC_META_WAIT1,
    A_STATIC_META_CAPTURE,
    A_STATIC_DATA_ADDR,
    A_STATIC_DATA_WAIT0,
    A_STATIC_DATA_WAIT1,
    A_STATIC_DATA_CAPTURE,
    A_NET_META_ADDR,
    A_NET_META_WAIT0,
    A_NET_META_WAIT1,
    A_NET_META_CAPTURE,
    A_NET_LANE_ADDR,
    A_NET_LANE_WAIT0,
    A_NET_LANE_WAIT1,
    A_NET_LANE_CAPTURE,
    A_NET_WAIT_READY,
    A_NET_STATIC_WAIT,
    A_NET_STREAM,
    A_NET_ARM,
    A_NET_ISSUE,
    A_WAIT_CMD,
    A_READBACK_REQ,
    A_READBACK_RESP,
    A_READBACK_FINISH,
    A_DEBUG_STATIC_READ,
    A_DEBUG_WRITE,
    A_RESET_WAIT,
    A_WRITE_FINAL,
    A_WRITE_EVENT,
    A_IRQ_WAIT_ACK,
    // Blocked-GRADP v2 states
    A_BDESC_META_ADDR,
    A_BDESC_META_WAIT0,
    A_BDESC_META_WAIT1,
    A_BDESC_META_CAPTURE,
    A_BDESC_ADDR,
    A_BDESC_WAIT0,
    A_BDESC_WAIT1,
    A_BDESC_CAPTURE,
    A_BDESC_WRITE,
    A_BDESC_DONE
  } adapter_state_e;

  adapter_state_e state_q;
  logic [BRAM_DATA_W-1:0] bram_data_q;
  logic usr_irq_ack_meta_q;
  logic usr_irq_ack_sync_q;
  logic [7:0] status_state_q;
  logic [7:0] last_cmd_q;
  logic [7:0] seq_id_q;
  logic [7:0] error_code_q;
  logic [31:0] processed_count_q;
  logic [31:0] elapsed_cycles_q;
  logic [31:0] detail_flags_q;
  logic [31:0] event_counter_q;
  logic event_pending_q;

  logic [15:0] desc_magic_q;
  logic [7:0] desc_version_q;
  logic [7:0] cmd_type_q;
  logic [31:0] payload_start_q;
  logic [31:0] payload_end_q;
  logic [31:0] result_start_q;
  logic [31:0] result_capacity_q;
  logic [31:0] arg0_q;
  logic [31:0] arg1_q;
  logic [31:0] arg2_q;

  logic [31:0] command_cycle_q;
  logic [31:0] result_index_q;
  logic [31:0] config_word_idx_q;
  logic config_apply_bank_len_q;
  logic config_apply_wlvia_q;
  logic [31:0] config_wlvia_word_addr_q;
  logic [2:0] config_wlvia_setup_idx_q;
  logic signed [DATA_W-1:0] config_wlvia_value_q [0:5];
  logic [31:0] net_frame_idx_q;
  logic [31:0] net_flags_q;
  // Blocked-GRADP v2 command/sync state
  logic [10:0] bdesc_addr_q;
  logic [15:0] bdesc_count_q;
  logic [15:0] bdesc_written_q;
  logic [31:0] bdesc_word_idx_q;
  logic [3:0] bdesc_lane_idx_q;
  logic [BRAM_DATA_W-1:0] bdesc_word_q;
  logic blk_wait_spill_q;
  // NET-on-DDR self-run play state
  logic self_run_q;          // a self-run block play walk is in progress
  logic sr_play_last_q;      // played block is the iteration's last
  logic sr_last_frame_q;     // latched frame_last of the frame in capture
  logic sr_done_seen_q;      // sr_done_i seen since the last CMD_GO
  logic init_is_rms_q;
  logic init_is_image_q;
  localparam int FULL_RESET_CYCLES = 1000;
  localparam int FULL_RESET_COUNT_W = $clog2(FULL_RESET_CYCLES + 1);
  logic full_reset_active_q;
  logic [FULL_RESET_COUNT_W-1:0] full_reset_count_q;
  logic [31:0] wait_selector_q;
  logic [31:0] wait_timeout_q;
  logic [31:0] wait_elapsed_q;
  logic [31:0] init_start_row_q;
  logic [31:0] init_row_count_q;
  logic [31:0] init_total_value_count_q;
  logic [31:0] init_payload_word_idx_q;
  logic [31:0] init_cur_row_q;
  logic [BRAM_DATA_W-1:0] init_word_q;
  logic [31:0] group_flag_issue_count_q;
  logic [31:0] group_flag_return_count_q;
  logic [31:0] readback_row_offset_q;
  logic [31:0] readback_expected_row_q;
  logic [READBACK_SLOT_W-1:0] readback_slot_q;
  logic [BRAM_DATA_W-1:0] readback_word_q;
  logic [31:0] max_bank_valid_len_s;
  logic [31:0] static_weight_count_q;
  logic [31:0] static_loaded_count_q;
  logic [31:0] static_payload_word_idx_q;
  logic static_mem_wr_en_s;
  logic [STATIC_WEIGHT_WORD_ADDR_W-1:0] static_mem_wr_addr_s;
  logic [BRAM_DATA_W-1:0] static_mem_wr_data_s;
  logic static_mem_rd_en_s [0:CORE_COUNT-1];
  logic [STATIC_WEIGHT_WORD_ADDR_W-1:0] static_mem_rd_addr_s [0:CORE_COUNT-1];
  logic [BRAM_DATA_W-1:0] static_mem_rd_data_s [0:CORE_COUNT-1];
  logic static_weight_needed_s [0:CORE_COUNT-1];

  logic net_static_weight_bad_q;
  logic all_grad_task_ready_s;
  logic wait_condition_s;
  logic [63:0] cmd_fifo_head_s [0:CORE_COUNT-1];
  logic signed [DATA_W-1:0] static_w_fifo_head_s [0:CORE_COUNT-1];
  logic cmd_fifo_push_s [0:CORE_COUNT-1];
  logic cmd_fifo_pop_s [0:CORE_COUNT-1];
  logic [63:0] cmd_fifo_push_data_s [0:CORE_COUNT-1];
  logic static_w_fifo_push_s [0:CORE_COUNT-1];
  logic static_w_fifo_pop_s [0:CORE_COUNT-1];
  logic signed [DATA_W-1:0] static_w_fifo_push_data_s [0:CORE_COUNT-1];
  logic adapter_fifo_rst_n_s;
  logic cmd_fifo_empty_s [0:CORE_COUNT-1];
  logic cmd_fifo_full_s [0:CORE_COUNT-1];
  logic cmd_fifo_full_warning_s [0:CORE_COUNT-1];
  logic static_w_fifo_empty_s [0:CORE_COUNT-1];
  logic static_w_fifo_full_s [0:CORE_COUNT-1];
  logic static_w_fifo_full_warning_s [0:CORE_COUNT-1];
  logic cmd_fifo_ready_s [0:CORE_COUNT-1];
  logic static_w_fifo_ready_s [0:CORE_COUNT-1];
  logic cmd_fifo_output_valid_s [0:CORE_COUNT-1];
  logic static_w_fifo_output_valid_s [0:CORE_COUNT-1];
  logic cmd_fifo_head_need_static_s [0:CORE_COUNT-1];
  // Exact occupancy shadows for the IP cmd/static-w fifos. The Vivado
  // standard-mode fifo's empty/valid flags lag at the stream tail (the
  // M-series phantom lesson): an underflow read produces a bogus head that
  // can decode as static-needing and deadlock the issue stage. All pop and
  // occupancy decisions therefore use these exact push/pop counters instead
  // of the IP flags. Pushes are counted one cycle late so a set bit always
  // implies the word is physically readable by the IP fifo.
  logic [31:0] cmd_fifo_shadow_q [0:CORE_COUNT-1];
  logic [31:0] static_w_fifo_shadow_q [0:CORE_COUNT-1];
  logic cmd_fifo_push_d1_q [0:CORE_COUNT-1];
  logic static_w_fifo_push_d1_q [0:CORE_COUNT-1];
  // Issue pipeline (fixed-latency, 1 cmd/cycle): every command walks the
  // same three stages regardless of type, so a static-weight read is just
  // a normal pipeline stage instead of a variable-latency detour. Strict
  // order is structural (single lane); a need-static command with an empty
  // sw FIFO simply stalls S2 (the retired cmd_hold mechanism's lap hazard
  // cannot occur).
  //   S1: cmd_fifo pop issued, waiting for the head to become readable
  //   S2: head in hand; if it needs a static weight, pop the sw FIFO this
  //       cycle (stall while the sw FIFO is empty); plain commands idle here
  //   S3: assembled command (raw record + resolved weight) awaiting emit
  // S1: 2-entry queue between the cmd fifo pop and the S2 transfer. Each
  // entry carries {valid,data} as an inseparable pair: a pop only claims a
  // free slot (pend), and exactly one cycle later the popped word is
  // captured into the SAME slot (HAND_WRITTEN_STD guarantees pop(T) ->
  // dout(T+1)). Backpressure can therefore never separate validity from
  // data or miss the one-cycle output_valid pulse.
  logic [1:0]  iss_s1_valid_q [0:CORE_COUNT-1];
  logic [1:0]  iss_s1_pend_q [0:CORE_COUNT-1];
  logic [63:0] iss_s1_data_q [0:CORE_COUNT-1][0:1];
  logic        iss_s1_wr_ptr_q [0:CORE_COUNT-1];
  logic        iss_s1_rd_ptr_q [0:CORE_COUNT-1];
  logic [1:0]  iss_s1_count_q [0:CORE_COUNT-1];
  logic        iss_s1_pop_d1_q [0:CORE_COUNT-1];
  logic        iss_s1_pop_d1_ptr_q [0:CORE_COUNT-1];
  logic iss_s2_valid_q [0:CORE_COUNT-1];
  logic [63:0] iss_s2_cmd_q [0:CORE_COUNT-1];
  logic iss_s2_need_static_q [0:CORE_COUNT-1];
  logic iss_s3_valid_q [0:CORE_COUNT-1];
  logic [63:0] iss_s3_cmd_q [0:CORE_COUNT-1];
  logic signed [DATA_W-1:0] iss_s3_static_w_q [0:CORE_COUNT-1];
  logic iss_s3_need_static_q [0:CORE_COUNT-1];
  logic iss_s1_adv_s [0:CORE_COUNT-1];
  logic iss_s2_adv_s [0:CORE_COUNT-1];
  logic iss_s3_emit_s [0:CORE_COUNT-1];
  logic [BRAM_DATA_W-1:0] debug_word_s [0:6];
  logic [BRAM_DATA_W-1:0] debug_word_q [0:6];
  logic [2:0] debug_word_idx_q;
  logic debug_static_mode_q;
  logic [7:0] debug_static_addr_q;
  logic debug_trace_mode_q;
  logic [7:0] debug_trace_sel_q;
  logic [3:0] phase_prev_q;
  logic [15:0] emit_count_q [0:CORE_COUNT-1];
  logic [15:0] task_ready_rise_count_q [0:CORE_COUNT-1];
  logic grad_task_ready_prev_q [0:CORE_COUNT-1];
  logic net_fifo_space_ready_s;
  logic net_all_fifo_empty_s;
  logic net_issue_active_s;
  logic net_fifo_residue_s;
  logic [31:0] net_fifo_residue_detail_s;
  logic net_lane_static_bad_s;
  // NOTE: net_pending_static_mask_q is read by the (untouchable)
  // GEN_CORRECTNESS_PROBES sim-only block; declaration kept, all live-logic
  // writers removed with the legacy CMD_NET play states.
  logic net_pending_static_mask_q [0:CORE_COUNT-1];
  // self-run streaming play (A_NET_STREAM): one-deep frame pipe between the
  // capture cycle (cmd push + static read issue) and the static-weight push
  // cycle, so consecutive frames overlap (1 frame/cycle sustained)
  logic sr_pipe_valid_q;
  logic sr_pipe_last_q;
  logic sr_pipe_static_mask_q [0:CORE_COUNT-1];
  logic [STATIC_WEIGHT_LANE_W-1:0] sr_pipe_static_lane_q [0:CORE_COUNT-1];
  logic sr_frame_pop_q;

  localparam logic [2:0] WLVIA_MODE_IDLE                    = 3'd0;
  localparam logic [2:0] WLVIA_MODE_SET_LAMBDA_WL           = 3'd1;
  localparam logic [2:0] WLVIA_MODE_SET_LAMBDA_VIA          = 3'd2;
  localparam logic [2:0] WLVIA_MODE_SET_HOR_EDGE_LEN_NORMAL = 3'd3;
  localparam logic [2:0] WLVIA_MODE_SET_VER_EDGE_LEN_NORMAL = 3'd4;
  localparam logic [2:0] WLVIA_MODE_SET_HOR_EDGE_LEN_LAST   = 3'd5;
  localparam logic [2:0] WLVIA_MODE_SET_VER_EDGE_LEN_LAST   = 3'd6;
  localparam logic [1:0] OVERFLOW_MODE_IDLE                 = 2'd0;
  localparam logic [1:0] OVERFLOW_MODE_NEW_TASK             = 2'd1;
  localparam logic [1:0] OVERFLOW_MODE_SEGMENT              = 2'd2;
  localparam logic [1:0] OVERFLOW_MODE_SETW                 = 2'd3;

  initial begin
    assert (BRAM_DATA_W == 512)
      else $error(1, "router_pcie_bram_adapter: scheme requires 512-bit BRAM words");
    assert (CORE_COUNT <= LANES_PER_WORD)
      else $error(1, "router_pcie_bram_adapter: single-word NET frames require CORE_COUNT <= LANES_PER_WORD");
    assert (LANES_PER_WORD >= 1)
      else $error(1, "router_pcie_bram_adapter: invalid BRAM/lane geometry");
    assert (CORE_COUNT_PER_FIFO == 1)
      else $error(1, "router_pcie_bram_adapter: CORE_COUNT_PER_FIFO must be 1");
    assert (SEGMENT_PER_FIFO == 1)
      else $error(1, "router_pcie_bram_adapter: SEGMENT_PER_FIFO must be 1");
    assert ((CORE_COUNT % CORE_COUNT_PER_FIFO) == 0)
      else $error(1, "router_pcie_bram_adapter: CORE_COUNT must be divisible by CORE_COUNT_PER_FIFO");
    assert ((CORE_COUNT % SEGMENT_PER_FIFO) == 0)
      else $error(1, "router_pcie_bram_adapter: CORE_COUNT must be divisible by SEGMENT_PER_FIFO");
    assert (BRAM_DEPTH >= 128)
      else $error(1, "router_pcie_bram_adapter: BRAM_DEPTH is too small for reserved control area");
    assert (BANK_COUNT <= 128)
      else $error(1, "router_pcie_bram_adapter: NET packed_idx23 supports at most 128 banks");
    assert (BANK_DEPTH <= (1 << 16))
      else $error(1, "router_pcie_bram_adapter: NET packed_idx23 supports at most 65536 rows");
    assert (LOGITS_PER_WORD == 16)
      else $error(1, "router_pcie_bram_adapter: INIT_LOGITS_DDR requires 512-bit physical lines");
    assert (STATIC_WEIGHTS_PER_WORD == 16)
      else $error(1, "router_pcie_bram_adapter: INIT_STATIC_WEIGHTS expects sixteen 32-bit weights per BRAM word");
    assert ((BANK_COUNT * GROUP_FLAG_WORD_W) <= BRAM_DATA_W)
      else $error(1, "router_pcie_bram_adapter: group flags must fit in one BRAM word");
    assert ((BANK_COUNT * P_DATA_W) <= BRAM_DATA_W)
      else $error(1, "router_pcie_bram_adapter: one P row must fit in one BRAM word");
    assert ((BRAM_DATA_W % (BANK_COUNT * P_DATA_W)) == 0)
      else $error(1, "router_pcie_bram_adapter: P readback rows must pack evenly");
    assert ((BANK_COUNT * MEM_GRAD_P_W) <= BRAM_DATA_W)
      else $error(1, "router_pcie_bram_adapter: one grad_p row must fit in one BRAM word");
    assert ((BRAM_DATA_W % (BANK_COUNT * MEM_GRAD_P_W)) == 0)
      else $error(1, "router_pcie_bram_adapter: grad_p readback rows must pack evenly");
    assert (STATIC_WEIGHT_DEPTH <= (1 << MEM_IDX_W))
      else $error(1, "router_pcie_bram_adapter: STATIC_WEIGHT_DEPTH exceeds mem_idx address capacity");
    assert (STATIC_WEIGHT_DEPTH <= (1 << 16))
      else $error(1, "router_pcie_bram_adapter: NET packed_idx23 supports at most 65536 static weights");
    assert (BRAM_ADA_CMD_FIFO_DEPTH >= 16)
      else $error(1, "router_pcie_bram_adapter: BRAM_ADA_CMD_FIFO_DEPTH must be at least 16");
    assert (BRAM_ADA_STATIC_W_FIFO_DEPTH >= 8)
      else $error(1, "router_pcie_bram_adapter: BRAM_ADA_STATIC_W_FIFO_DEPTH must be at least 8");
  end

  generate
    for (genvar read_port_idx = 0; read_port_idx < CORE_COUNT; read_port_idx++) begin : GEN_STATIC_WEIGHT_BRAM
      (* ram_style = "block" *) logic [BRAM_DATA_W-1:0] mem_q [0:STATIC_WEIGHT_WORD_DEPTH-1];

      always_ff @(posedge clk) begin
        if (static_mem_wr_en_s) begin
          mem_q[static_mem_wr_addr_s] <= static_mem_wr_data_s;
        end
        if (static_mem_rd_en_s[read_port_idx]) begin
          static_mem_rd_data_s[read_port_idx] <= mem_q[static_mem_rd_addr_s[read_port_idx]];
        end
      end
    end
  endgenerate

  assign adapter_fifo_rst_n_s = rst_n && enable_i && !full_reset_active_q;
  assign full_reset_active_o = full_reset_active_q;
  assign readback_is_grad_o = 1'b0;
  assign readback_req_valid_o = (state_q == A_READBACK_REQ);
  assign readback_req_addr_o = MEM_ADDR_W'(arg0_q + readback_row_offset_q);

  always_comb begin
    max_bank_valid_len_s = '0;
    for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
      if (bank_valid_len_o[bank_idx] > max_bank_valid_len_s) begin
        max_bank_valid_len_s = 32'(bank_valid_len_o[bank_idx]);
      end
    end
  end

  generate
    for (genvar fifo_core_idx = 0; fifo_core_idx < CORE_COUNT; fifo_core_idx++) begin : GEN_ADAPTER_FIFOS
      fifo #(
        .DATA_W(64),
        .FIFO_DEPTH(BRAM_ADA_CMD_FIFO_DEPTH),
        .WARNING_THR(BRAM_ADA_CMD_FIFO_WARNING_THR),
        .FWFT_MODE(1'b0),
        .COMMON_CLOCK_MODE(1'b1),
        // Handwritten standard-mode ring: exact flags + occupancy-gated pop;
        // replaces the Vivado IP fifo whose lagging tail valid/empty caused
        // phantom cmds (M-series lesson, reproduced on the self-run walk).
        .HAND_WRITTEN_STD(1'b1)
      ) u_cmd_fifo (
        .clk(clk),
        .rst_n(adapter_fifo_rst_n_s),
        .input_data_i(cmd_fifo_push_data_s[fifo_core_idx]),
        .input_valid_i(cmd_fifo_push_s[fifo_core_idx]),
        .pop_i(cmd_fifo_pop_s[fifo_core_idx]),
        .output_data_o(cmd_fifo_head_s[fifo_core_idx]),
        .output_valid_o(cmd_fifo_output_valid_s[fifo_core_idx]),
        .empty_o(cmd_fifo_empty_s[fifo_core_idx]),
        .full_warning_o(cmd_fifo_full_warning_s[fifo_core_idx]),
        .full_o(cmd_fifo_full_s[fifo_core_idx]),
        .single_o(),
        .ready_o(cmd_fifo_ready_s[fifo_core_idx])
      );

      fifo #(
        .DATA_W(DATA_W),
        .FIFO_DEPTH(BRAM_ADA_STATIC_W_FIFO_DEPTH),
        .WARNING_THR(BRAM_ADA_STATIC_W_FIFO_WARNING_THR),
        .FWFT_MODE(1'b0),
        .COMMON_CLOCK_MODE(1'b1),
        .HAND_WRITTEN_STD(1'b1)
      ) u_static_w_fifo (
        .clk(clk),
        .rst_n(adapter_fifo_rst_n_s),
        .input_data_i(static_w_fifo_push_data_s[fifo_core_idx]),
        .input_valid_i(static_w_fifo_push_s[fifo_core_idx]),
        .pop_i(static_w_fifo_pop_s[fifo_core_idx]),
        .output_data_o(static_w_fifo_head_s[fifo_core_idx]),
        .output_valid_o(static_w_fifo_output_valid_s[fifo_core_idx]),
        .empty_o(static_w_fifo_empty_s[fifo_core_idx]),
        .full_warning_o(static_w_fifo_full_warning_s[fifo_core_idx]),
        .full_o(static_w_fifo_full_s[fifo_core_idx]),
        .single_o(),
        .ready_o(static_w_fifo_ready_s[fifo_core_idx])
      );
    end
  endgenerate

  // Occupancy shadows (see the declaration note): pushes are registered one
  // cycle before counting so shadow>0 implies the IP fifo can actually serve
  // the word; pops are only ever requested when the shadow is non-zero, so
  // the IP fifo can never be underflow-read (the M-series phantom source).
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
        cmd_fifo_shadow_q[core_idx] <= '0;
        static_w_fifo_shadow_q[core_idx] <= '0;
        cmd_fifo_push_d1_q[core_idx] <= 1'b0;
        static_w_fifo_push_d1_q[core_idx] <= 1'b0;
      end
    end else if (!enable_i || full_reset_active_q) begin
      for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
        cmd_fifo_shadow_q[core_idx] <= '0;
        static_w_fifo_shadow_q[core_idx] <= '0;
        cmd_fifo_push_d1_q[core_idx] <= 1'b0;
        static_w_fifo_push_d1_q[core_idx] <= 1'b0;
      end
    end else begin
      for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
        cmd_fifo_push_d1_q[core_idx] <= cmd_fifo_push_s[core_idx];
        static_w_fifo_push_d1_q[core_idx] <= static_w_fifo_push_s[core_idx];
        unique case ({cmd_fifo_push_d1_q[core_idx], cmd_fifo_pop_s[core_idx]})
          2'b10: cmd_fifo_shadow_q[core_idx] <= cmd_fifo_shadow_q[core_idx] + 1'b1;
          2'b01: cmd_fifo_shadow_q[core_idx] <= cmd_fifo_shadow_q[core_idx] - 1'b1;
          default: cmd_fifo_shadow_q[core_idx] <= cmd_fifo_shadow_q[core_idx];
        endcase
        unique case ({static_w_fifo_push_d1_q[core_idx], static_w_fifo_pop_s[core_idx]})
          2'b10: static_w_fifo_shadow_q[core_idx] <= static_w_fifo_shadow_q[core_idx] + 1'b1;
          2'b01: static_w_fifo_shadow_q[core_idx] <= static_w_fifo_shadow_q[core_idx] - 1'b1;
          default: static_w_fifo_shadow_q[core_idx] <= static_w_fifo_shadow_q[core_idx];
        endcase
      end
    end
  end

  function automatic logic [BRAM_ADDR_W-1:0] bram_addr(
    input int unsigned word_index
  );
    begin
      bram_addr = BRAM_ADDR_W'(word_index);
    end
  endfunction

  function automatic logic [BRAM_DATA_W-1:0] pack_status(
    input logic [7:0] state_value,
    input logic [7:0] cmd_value,
    input logic [7:0] seq_value,
    input logic [7:0] error_value,
    input logic [31:0] processed_value,
    input logic [31:0] elapsed_value,
    input logic [31:0] detail_value
  );
    logic [BRAM_DATA_W-1:0] word;
    begin
      word = '0;
      word[7:0] = state_value;
      word[15:8] = cmd_value;
      word[23:16] = seq_value;
      word[31:24] = error_value;
      word[63:32] = processed_value;
      word[95:64] = elapsed_value;
      word[127:96] = detail_value;
      pack_status = word;
    end
  endfunction

  // Debug snapshot combinational packing (W0-W6)
  always_comb begin
    // W0: existing status word
    debug_word_s[0] = pack_status(status_state_q, last_cmd_q, seq_id_q, error_code_q, processed_count_q, command_cycle_q, {28'd0, system_phase_i});

    // W1: adapter state snapshot
    debug_word_s[1] = '0;
    debug_word_s[1][7:0] = 8'(state_q);
    debug_word_s[1][11:8] = system_phase_i;
    debug_word_s[1][63:32] = net_fifo_residue_detail_s;
    for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
      debug_word_s[1][64 + core_idx] = grad_task_ready_i[core_idx];
      debug_word_s[1][68 + core_idx] = grad_overflow_host_ready_i[core_idx / CORE_COUNT_PER_FIFO][core_idx % CORE_COUNT_PER_FIFO];
    end
    debug_word_s[1][72] = grad_p_finish_i;
    debug_word_s[1][73] = all_grad_task_ready_s;
    debug_word_s[1][74] = softmax_enable_i;
    debug_word_s[1][75] = bridge_ddr_active_i;
    debug_word_s[1][76] = enable_i;
    // NET-on-DDR self-run master status: [79]=done [80]=running
    // [88:81]={mn_state, cur_blk(6b)} [89]=phantom_err [95:90]=0
    // [127:96]=completed iteration count
    debug_word_s[1][79] = sr_done_i;
    debug_word_s[1][80] = sr_running_i;
    debug_word_s[1][88:81] = {sr_mn_state_i, sr_cur_blk_i};
    debug_word_s[1][89] = sr_error_i[0];
    debug_word_s[1][95:90] = '0;
    debug_word_s[1][127:96] = sr_iter_cnt_i;

    // W2-W3: issue_stage stuck frame
    for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
      debug_word_s[2][core_idx * 32 +: 32] = iss_s3_cmd_q[core_idx][31:0];
      debug_word_s[3][core_idx * 32 +: 32] = iss_s3_cmd_q[core_idx][63:32];
    end
    for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
      debug_word_s[3][128 + core_idx * 16 +: 16] = iss_s3_static_w_q[core_idx];
      debug_word_s[3][192 + core_idx] = iss_s3_valid_q[core_idx];
      debug_word_s[3][200 + core_idx] = iss_s3_need_static_q[core_idx];
    end

    // Static weight table readback mode (CMD_DEBUG with arg0[31] set): W2
    // carries the raw 512-bit mem_q word at debug_static_addr_q sampled from
    // read port 0 (lane L at [L*32 +: 32], matching the INIT_STATIC_WEIGHTS
    // pack order); W3 echoes the word address in its low 32 bits. All other
    // words keep their default meaning.
    if (debug_static_mode_q) begin
      debug_word_s[2] = static_mem_rd_data_s[0];
      debug_word_s[3] = '0;
      debug_word_s[3][31:0] = {24'd0, debug_static_addr_q};
    end

    // Overflow trace readback mode (CMD_DEBUG with arg0[30] set and arg0[31]
    // clear): W2[31:0] carries the 32-bit trace field selected by
    // debug_trace_sel_q (selector = core*8 + field for the per-core
    // dispatcher taps, CORE_COUNT*8 = merge capture count,
    // CORE_COUNT*8+1 = merge nonzero count, CORE_COUNT*8+2 + core*5 + k =
    // per-core segcore taps; out-of-range selectors read as 0); W3[31:0]
    // echoes the selector. Pure combinational mux, so it flows through the
    // normal A_DEBUG_WRITE capture with no extra state. All other words
    // keep their default meaning.
    if (debug_trace_mode_q) begin
      debug_word_s[2] = '0;
      if (debug_trace_sel_q < 8'(CORE_COUNT*13+2)) begin
        debug_word_s[2][31:0] = gradp_debug_trace_i[debug_trace_sel_q*32 +: 32];
      end
      debug_word_s[3] = '0;
      debug_word_s[3][31:0] = {24'd0, debug_trace_sel_q};
    end

    // W4: dispatcher state per core
    debug_word_s[4] = '0;
    for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
      debug_word_s[4][core_idx * 16 +: 16] = gradp_debug_core_i[core_idx];
    end

    // W5: bank state
    debug_word_s[5] = '0;
    debug_word_s[5][31:0] = gradp_debug_bank_i;
    // self-run bring-up probes (W5 high bits; harmless in all modes):
    //   [35:32]=ld_state [39:36]=fetcher f_state [46:40]=cf_sec [53:47]=nf_sec
    //   [85:54]=cf_line  [117:86]=nf_line        [134:118]=ld_row_cnt
    //   [140:135]=ld_blk [141]=ld_done_valid     [142]=fetcher in-flight
    debug_word_s[5][35:32] = sr_ld_state_i;
    debug_word_s[5][39:36] = {1'b0, nf_dbg_f_state_i};
    debug_word_s[5][46:40] = nf_dbg_cf_sec_i;
    debug_word_s[5][53:47] = nf_dbg_nf_sec_i;
    debug_word_s[5][85:54] = nf_dbg_cf_line_i;
    debug_word_s[5][117:86] = nf_dbg_nf_line_i;
    debug_word_s[5][134:118] = sr_ld_row_cnt_i;
    debug_word_s[5][140:135] = sr_ld_blk_i;
    debug_word_s[5][141] = sr_ld_done_valid_i;
    debug_word_s[5][142] = nf_dbg_flight_i;

    // W6: counters
    debug_word_s[6] = '0;
    for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
      debug_word_s[6][core_idx * 16 +: 16] = emit_count_q[core_idx];
      debug_word_s[6][64 + core_idx * 16 +: 16] = task_ready_rise_count_q[core_idx];
    end
    // W6 high bits: self-run hang localization probes
    //   [132:128]=bridge_state [133]=rd_req_valid [134]=rd_req_ready
    //   [135]=rd_resp_valid  [139:136]={arvalid,arready,rvalid,rready}
    //   [140]=fr_req_valid   [141]=fr_req_ready  [142]=fetch_in_flight
    //   [155:143]=eng_probe  [175:156]=ld_probe2 (bit maps at the sources)
    debug_word_s[6][132:128] = bridge_probe_i[4:0];
    debug_word_s[6][135:133] = bridge_probe_i[7:5];
    debug_word_s[6][139:136] = bridge_probe_i[11:8];
    debug_word_s[6][142:140] = bridge_probe_i[14:12];
    debug_word_s[6][155:143] = eng_probe_i[12:0];
    debug_word_s[6][175:156] = ld_probe2_i;
    // W6 [209:176]: phase_q / pbank req / pbank grant / update_active
    //   [179:176]=phase_q [187:180]=pbank_req [195:188]=pbank_grant [196]=upd_active
    debug_word_s[6][179:176] = eng_probe_i[16:13];
    debug_word_s[6][187:180] = eng_probe_i[24:17];
    debug_word_s[6][195:188] = eng_probe_i[32:25];
    debug_word_s[6][196] = eng_probe_i[33];
  end

  // ---------------------------------------------------------------------
  // Self-run status mirror. The seven CMD_DEBUG snapshot words plus the
  // self-run diagnostics below are captured atomically (one cycle) into a
  // snapshot register row and streamed, one word per BRAM write-port idle
  // cycle, into the fixed mirror region
  // [MIRROR_BASE_INDEX, MIRROR_BASE_INDEX+MIRROR_WORDS) at the top of the
  // command BRAM (payload capacity is reduced accordingly on the host/TB
  // side). Each frame dwells ~MIRROR_DWELL_CYCLES so a host-side
  // "read seq -> read frame -> read seq" coherence check always lands
  // inside a single frame regardless of AXI read latency. Self-run has no
  // command traffic by software discipline, so the mirror gets nearly all
  // of the port; command FSM writes still take strict priority (the mirror
  // yields and resumes mid-frame; the snapshot row is held frozen while a
  // frame streams, so every BRAM frame is one coherent snapshot).
  // ---------------------------------------------------------------------
  localparam int MIRROR_BASE_INDEX = 8;
  localparam int MIRROR_WORDS = 16;
  localparam int MIRROR_DWELL_CYCLES = 65536;
  localparam logic [15:0] MIRROR_MAGIC = 16'hDB60;
  // runtime mirror page select (header bits [65:64]); only the perf/stall
  // layouts (PROBE_CORRECTNESS_EN=0) are pageable
  localparam logic [1:0] MIRROR_PAGE_PERF = 2'd0;
  localparam logic [1:0] MIRROR_PAGE_STALL_ENG = 2'd1;
  localparam logic [1:0] MIRROR_PAGE_STALL_BANK = 2'd2;

  // FSM-side BRAM write registers; the output ports below mux these with
  // the mirror streamer (FSM wins whenever its write port is active).
  logic [BRAM_STRB_W-1:0] fsm_bram_we_q;
  logic [BRAM_ADDR_W-1:0] fsm_bram_addr_q;
  logic [BRAM_DATA_W-1:0] fsm_bram_wdata_q;

  typedef enum logic [1:0] {MIR_DWELL, MIR_SNAP, MIR_STREAM} mirror_state_e;
  mirror_state_e mirror_state_q;
  logic [31:0] mirror_frame_seq_q;
  logic [16:0] mirror_dwell_q;
  logic [4:0] mirror_wr_idx_q;
  logic [BRAM_DATA_W-1:0] mirror_snap_q [0:MIRROR_WORDS-2];
  logic [BRAM_DATA_W-1:0] mirror_content_s [0:MIRROR_WORDS-2];
  logic [BRAM_STRB_W-1:0] mirror_bram_we_q;
  logic [BRAM_ADDR_W-1:0] mirror_bram_addr_q;
  logic [BRAM_DATA_W-1:0] mirror_bram_wdata_q;
  logic [1:0] mirror_tail_q;
  logic sr_running_prev_q;
  logic mirror_active_s;
  logic mirror_frame_done_s;
  logic selfrun_gate_s;

  // Self-run pairing probes (V-series) declarations; see the probe block
  // below for the rationale (M10..M12 mirror readout).
  logic [15:0] prb_sw_push_cnt_q   [0:CORE_COUNT-1];
  logic [15:0] prb_sw_pop_cnt_q    [0:CORE_COUNT-1];
  logic [15:0] prb_cmd_push_cnt_q  [0:CORE_COUNT-1];
  logic [15:0] prb_issue_nst_cnt_q [0:CORE_COUNT-1];
  logic [15:0] prb_issue_dir_cnt_q [0:CORE_COUNT-1];
  logic [15:0] prb_hold_set_cnt_q  [0:CORE_COUNT-1];
  logic [3:0]  prb_cmd_pushed_flag_q;

  logic        prb_v1_sticky_q, prb_v2_sticky_q, prb_v3_sticky_q, prb_v4_sticky_q;
  logic [1:0]  prb_v1_core_q, prb_v2_core_q, prb_v3_core_q, prb_v4_core_q;
  logic [31:0] prb_v1_frame_q, prb_v2_frame_q, prb_v3_frame_q, prb_v4_frame_q;
  logic [15:0] prb_v1_shadow_q, prb_v1_fifocnt_q;
  logic [63:0] prb_v2_rec_q, prb_v3_rec_q, prb_v4_rec_q;
  logic        prb_v4_rd_done_q;
  logic [3:0]  prb_v1_mismatch_cnt_q [0:CORE_COUNT-1];

  // mask-set content log: last 4 mask-set events (frame_idx + core +
  // full lane_record) for content-level comparison against the image
  logic [31:0] prb_mask_set_cnt_q;
  logic [2:0]  prb_log_count_q;
  logic [31:0] prb_log_frame_q [0:3];
  logic [1:0]  prb_log_core_q  [0:3];
  logic [63:0] prb_log_rec_q   [0:3];

  // the fifo instance's own occupancy (hierarchical read of the
  // hand-written ring's count_q) for the V1 shadow-vs-storage check
  logic [9:0]  prb_fifo_cnt_s [0:CORE_COUNT-1];

  // -------------------------------------------------------------------
  // Performance counters (mirror M10-M13 when PROBE_CORRECTNESS_EN=0):
  // per-phase cycles, bridge occupancy, backpressure cycles and FIFO
  // watermarks. Cleared on the sr_running rising edge (GO), free-running
  // and host-readable any time after that.
  // -------------------------------------------------------------------
  logic [31:0] perf_total_q;
  logic [31:0] perf_play_q;
  logic [31:0] perf_waitload_q;
  logic [31:0] perf_spill_q;
  logic [31:0] perf_upd_q;
  logic [31:0] perf_overlap_q;
  logic [31:0] perf_bridge_busy_q;
  logic [31:0] perf_net_starve_q;
  logic [31:0] perf_space_stall_q;
  logic [31:0] perf_emit_bp_q;
  logic [9:0]  perf_cmd_wm_q [0:CORE_COUNT-1];
  logic [9:0]  perf_sw_wm_q [0:CORE_COUNT-1];
  // stall-localization counters (mirror pages STALL_ENG / STALL_BANK):
  // per-core dispatcher state residency + frontend/upd-fifo/emit stalls,
  // per-bank p grant / grad commit pulses, per-source hold stall
  logic [31:0] perf_dsp_state_q [0:CORE_COUNT-1][0:7];
  logic [31:0] perf_fe_wait_q [0:CORE_COUNT-1];
  logic [31:0] perf_preq_stall_q [0:CORE_COUNT-1];
  logic [31:0] perf_upd_warn_q [0:CORE_COUNT-1];
  logic [31:0] perf_emit_bp_core_q [0:CORE_COUNT-1];
  logic [31:0] perf_pgrant_bank_q [0:BANK_COUNT-1];
  logic [31:0] perf_commit_bank_q [0:BANK_COUNT-1];
  logic [31:0] perf_hold_stall_q [0:GRAD_OVERFLOW_COUNT+GRAD_WLVIA_COUNT-1];
  logic [1:0]  mirror_page_q;
  logic        mirror_kick_q;
  logic        sr_running_d1_q;
  logic        perf_go_pulse_s;
  logic        perf_issue_valid_any_s;
  logic        perf_emit_ready_all_s;
  logic        perf_emit_blocked_s;
  // per-lane cmd fifo occupancy (genvar-indexed hierarchical taps; a
  // runtime-variable index into a generate block is illegal)
  logic [9:0]  perf_cmd_count_s [0:CORE_COUNT-1];

  assign perf_go_pulse_s = sr_running_i && !sr_running_d1_q;
  assign perf_emit_blocked_s = perf_issue_valid_any_s && !perf_emit_ready_all_s;

  always_comb begin
    perf_issue_valid_any_s = 1'b0;
    perf_emit_ready_all_s = 1'b1;
    for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
      perf_issue_valid_any_s |= iss_s3_valid_q[core_idx];
      perf_emit_ready_all_s &= grad_task_ready_i[core_idx];
      perf_emit_ready_all_s &=
        grad_overflow_host_ready_i[core_idx / CORE_COUNT_PER_FIFO][core_idx % CORE_COUNT_PER_FIFO];
      perf_emit_ready_all_s &= !grad_wlvia_fifo_warning_i[core_idx / SEGMENT_PER_FIFO];
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sr_running_d1_q <= 1'b0;
      perf_total_q <= '0;
      perf_play_q <= '0;
      perf_waitload_q <= '0;
      perf_spill_q <= '0;
      perf_upd_q <= '0;
      perf_overlap_q <= '0;
      perf_bridge_busy_q <= '0;
      perf_net_starve_q <= '0;
      perf_space_stall_q <= '0;
      perf_emit_bp_q <= '0;
      for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
        perf_cmd_wm_q[core_idx] <= '0;
        perf_sw_wm_q[core_idx] <= '0;
        perf_fe_wait_q[core_idx] <= '0;
        perf_preq_stall_q[core_idx] <= '0;
        perf_upd_warn_q[core_idx] <= '0;
        perf_emit_bp_core_q[core_idx] <= '0;
        for (int state_idx = 0; state_idx < 8; state_idx++) begin
          perf_dsp_state_q[core_idx][state_idx] <= '0;
        end
      end
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        perf_pgrant_bank_q[bank_idx] <= '0;
        perf_commit_bank_q[bank_idx] <= '0;
      end
      for (int src_idx = 0; src_idx < (GRAD_OVERFLOW_COUNT + GRAD_WLVIA_COUNT); src_idx++) begin
        perf_hold_stall_q[src_idx] <= '0;
      end
    end else begin
      sr_running_d1_q <= sr_running_i;
      if (perf_go_pulse_s) begin
        perf_total_q <= '0;
        perf_play_q <= '0;
        perf_waitload_q <= '0;
        perf_spill_q <= '0;
        perf_upd_q <= '0;
        perf_overlap_q <= '0;
        perf_bridge_busy_q <= '0;
        perf_net_starve_q <= '0;
        perf_space_stall_q <= '0;
        perf_emit_bp_q <= '0;
        for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
          perf_cmd_wm_q[core_idx] <= '0;
          perf_sw_wm_q[core_idx] <= '0;
          perf_fe_wait_q[core_idx] <= '0;
          perf_preq_stall_q[core_idx] <= '0;
          perf_upd_warn_q[core_idx] <= '0;
          perf_emit_bp_core_q[core_idx] <= '0;
          for (int state_idx = 0; state_idx < 8; state_idx++) begin
            perf_dsp_state_q[core_idx][state_idx] <= '0;
          end
        end
        for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
          perf_pgrant_bank_q[bank_idx] <= '0;
          perf_commit_bank_q[bank_idx] <= '0;
        end
        for (int src_idx = 0; src_idx < (GRAD_OVERFLOW_COUNT + GRAD_WLVIA_COUNT); src_idx++) begin
          perf_hold_stall_q[src_idx] <= '0;
        end
      end else begin
        if (sr_running_i) begin
          perf_total_q <= perf_total_q + 1'b1;
        end
        if (sr_mn_state_i == 3'd2) begin
          perf_play_q <= perf_play_q + 1'b1;
        end
        if (sr_mn_state_i == 3'd1) begin
          perf_waitload_q <= perf_waitload_q + 1'b1;
        end
        if (sr_mn_state_i == 3'd3) begin
          perf_spill_q <= perf_spill_q + 1'b1;
        end
        if (sr_upd_probe_i[0]) begin
          perf_upd_q <= perf_upd_q + 1'b1;
        end
        if ((sr_mn_state_i == 3'd2) && sr_upd_probe_i[0]) begin
          perf_overlap_q <= perf_overlap_q + 1'b1;
        end
        if (bridge_probe_i[4:0] != 5'd0) begin
          perf_bridge_busy_q <= perf_bridge_busy_q + 1'b1;
        end
        if (
          self_run_q &&
          ((state_q == A_NET_LANE_ADDR) || (state_q == A_NET_STREAM)) &&
          !sr_frame_valid_i
        ) begin
          perf_net_starve_q <= perf_net_starve_q + 1'b1;
        end
        if (
          (((state_q == A_NET_LANE_CAPTURE) || (state_q == A_NET_STATIC_WAIT)) &&
           !net_fifo_space_ready_s) ||
          ((state_q == A_NET_STREAM) && sr_frame_valid_i && !net_fifo_space_ready_s)
        ) begin
          perf_space_stall_q <= perf_space_stall_q + 1'b1;
        end
        if (perf_emit_blocked_s) begin
          perf_emit_bp_q <= perf_emit_bp_q + 1'b1;
        end
        if (sr_running_i) begin
          for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
            perf_dsp_state_q[core_idx][gradp_debug_core_i[core_idx][2:0]] <=
              perf_dsp_state_q[core_idx][gradp_debug_core_i[core_idx][2:0]] + 1'b1;
            if (gradp_debug_core_i[core_idx][8]) begin
              perf_fe_wait_q[core_idx] <= perf_fe_wait_q[core_idx] + 1'b1;
            end
            if (stall_probe_i[core_idx]) begin
              perf_preq_stall_q[core_idx] <= perf_preq_stall_q[core_idx] + 1'b1;
            end
            if (gradp_debug_core_i[core_idx][10]) begin
              perf_upd_warn_q[core_idx] <= perf_upd_warn_q[core_idx] + 1'b1;
            end
            if (iss_s3_valid_q[core_idx] && perf_emit_blocked_s) begin
              perf_emit_bp_core_q[core_idx] <= perf_emit_bp_core_q[core_idx] + 1'b1;
            end
          end
          for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
            if (stall_probe_i[4 + bank_idx]) begin
              perf_pgrant_bank_q[bank_idx] <= perf_pgrant_bank_q[bank_idx] + 1'b1;
            end
            if (stall_probe_i[12 + bank_idx]) begin
              perf_commit_bank_q[bank_idx] <= perf_commit_bank_q[bank_idx] + 1'b1;
            end
          end
          for (int src_idx = 0; src_idx < (GRAD_OVERFLOW_COUNT + GRAD_WLVIA_COUNT); src_idx++) begin
            if (stall_probe_i[20 + src_idx]) begin
              perf_hold_stall_q[src_idx] <= perf_hold_stall_q[src_idx] + 1'b1;
            end
          end
        end
        for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
          if (perf_cmd_count_s[core_idx] > perf_cmd_wm_q[core_idx]) begin
            perf_cmd_wm_q[core_idx] <= perf_cmd_count_s[core_idx];
          end
          if (prb_fifo_cnt_s[core_idx] > perf_sw_wm_q[core_idx]) begin
            perf_sw_wm_q[core_idx] <= prb_fifo_cnt_s[core_idx];
          end
        end
      end
    end
  end

  // NET frame word source: host-command plays read the cmd BRAM payload;
  // a self-run play reads the net_fetcher's head frame (low 256b)
  logic [BRAM_DATA_W-1:0] net_frame_word_s;
  assign net_frame_word_s =
    self_run_q ? {{(BRAM_DATA_W-256){1'b0}}, sr_frame_data_i} : bram_data_q;

  // Self-run submit-poll gate: while a self-run (plus the mirror tail
  // frames) owns the adapter, the A_FLAG poll of the submit flag is
  // disabled, so the command FSM never touches the BRAM port except its
  // sporadic boundary status/event writes. A host command submitted
  // during the gate is not lost or corrupted: its submit magic simply
  // waits at SUBMIT_INDEX and is detected after the gate lifts.
  assign selfrun_gate_s = sr_running_i || (mirror_tail_q != 2'd0);

  // The streaming play pops the fetcher head combinationally in the same
  // cycle the lanes are captured: the head data is combinational (FWFT),
  // so a registered pop would capture every head twice and fire the
  // fetcher's `last` (pop-count based) one frame early — the closing
  // frame was then never delivered and the dispatcher wedged in ST_SETW.
  assign sr_frame_pop_o =
    sr_frame_pop_q ||
    (self_run_q && (state_q == A_NET_STREAM) && sr_frame_valid_i &&
     net_fifo_space_ready_s && !(sr_pipe_valid_q && sr_pipe_last_q));

  // BRAM write-port arbitration: the mirror drives the port only while it
  // holds a pending word (valid) AND the FSM is not writing this cycle
  // (ready); at all other times the FSM's address/data pass through
  // untouched — including its reads (we=0), which must never see the
  // mirror's address (broke full_reset/command reads when the mux keyed
  // on fsm_we alone).
  assign cmd_bram_we_o =
    ((mirror_bram_we_q != '0) && (fsm_bram_we_q == '0)) ? mirror_bram_we_q    : fsm_bram_we_q;
  assign cmd_bram_addr_o =
    ((mirror_bram_we_q != '0) && (fsm_bram_we_q == '0)) ? mirror_bram_addr_q  : fsm_bram_addr_q;
  assign cmd_bram_wdata_o =
    ((mirror_bram_we_q != '0) && (fsm_bram_we_q == '0)) ? mirror_bram_wdata_q : fsm_bram_wdata_q;

  always_comb begin
    // M1..M7: the CMD_DEBUG snapshot words (W0..W6) verbatim.
    for (int w = 0; w < 7; w++) begin
      mirror_content_s[w] = debug_word_s[w];
    end
    // M8: issue emit gates + adapter play state
    //   [3:0]=grad_task_ready [7:4]=overflow_host_ready [11:8]=wlvia_warn
    //   [19:12]=adapter state_q [51:20]=wait_elapsed [52]=self_run
    //   [84:53]=net_frame_idx [85]=sr_last_frame [86]=sr_frame_valid
    //   [119:88]=processed_count
    mirror_content_s[7] = '0;
    for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
      mirror_content_s[7][core_idx] = grad_task_ready_i[core_idx];
      mirror_content_s[7][4 + core_idx] =
        grad_overflow_host_ready_i[core_idx / CORE_COUNT_PER_FIFO][core_idx % CORE_COUNT_PER_FIFO];
      mirror_content_s[7][8 + core_idx] = grad_wlvia_fifo_warning_i[core_idx / SEGMENT_PER_FIFO];
    end
    mirror_content_s[7][19:12] = 8'(state_q);
    mirror_content_s[7][51:20] = wait_elapsed_q;
    mirror_content_s[7][52] = self_run_q;
    mirror_content_s[7][84:53] = net_frame_idx_q;
    mirror_content_s[7][85] = sr_last_frame_q;
    mirror_content_s[7][86] = sr_frame_valid_i;
    mirror_content_s[7][119:88] = processed_count_q;
    // M9: per-lane FIFO bookkeeping
    //   [63:0]=cmd_fifo shadow (4x16) [127:64]=static_w shadow (4x16)
    //   [131:128]/[135:132]=cmd/static full_warning
    //   [139:136]/[143:140]=cmd/static output_valid
    //   [175:144]=static_weight_count [207:176]=static_loaded_count
    //   [239:208]=gflag_issue_count [271:240]=gflag_return_count
    mirror_content_s[8] = '0;
    for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
      mirror_content_s[8][core_idx * 16 +: 16] = 16'(cmd_fifo_shadow_q[core_idx]);
      mirror_content_s[8][64 + core_idx * 16 +: 16] = 16'(static_w_fifo_shadow_q[core_idx]);
      mirror_content_s[8][128 + core_idx] = cmd_fifo_full_warning_s[core_idx];
      mirror_content_s[8][132 + core_idx] = static_w_fifo_full_warning_s[core_idx];
      mirror_content_s[8][136 + core_idx] = cmd_fifo_output_valid_s[core_idx];
      mirror_content_s[8][140 + core_idx] = static_w_fifo_output_valid_s[core_idx];
    end
    mirror_content_s[8][175:144] = static_weight_count_q;
    mirror_content_s[8][207:176] = static_loaded_count_q;
    mirror_content_s[8][239:208] = group_flag_issue_count_q;
    mirror_content_s[8][271:240] = group_flag_return_count_q;
    if (PROBE_CORRECTNESS_EN) begin
    // M10: pairing-probe counters (per lane, 16b) + fifo real count (10b)
    //   [63:0]=sw_push [127:64]=sw_pop [191:128]=cmd_push [231:192]=fifo_cnt
    mirror_content_s[9] = '0;
    for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
      mirror_content_s[9][core_idx * 16 +: 16] = prb_sw_push_cnt_q[core_idx];
      mirror_content_s[9][64 + core_idx * 16 +: 16] = prb_sw_pop_cnt_q[core_idx];
      mirror_content_s[9][128 + core_idx * 16 +: 16] = prb_cmd_push_cnt_q[core_idx];
      mirror_content_s[9][192 + core_idx * 10 +: 10] = prb_fifo_cnt_s[core_idx];
    end
    end else begin
    // M10 (perf): per-phase cycles since GO
    //   [31:0]total [63:32]play [95:64]waitload [127:96]spill
    //   [159:128]upd [191:160]overlap [223:192]bridge_busy
    mirror_content_s[9] = '0;
    mirror_content_s[9][31:0] = perf_total_q;
    mirror_content_s[9][63:32] = perf_play_q;
    mirror_content_s[9][95:64] = perf_waitload_q;
    mirror_content_s[9][127:96] = perf_spill_q;
    mirror_content_s[9][159:128] = perf_upd_q;
    mirror_content_s[9][191:160] = perf_overlap_q;
    mirror_content_s[9][223:192] = perf_bridge_busy_q;
    end
    if (PROBE_CORRECTNESS_EN) begin
    // M11: issue/hold counters + sticky violation flags/frames
    //   [63:0]=issue_needstatic [127:64]=issue_direct [191:128]=hold_set
    //   [195:192]=flags {v4,v3,v2,v1} [203:196]=cores {v4,v3,v2,v1}
    //   [227:196] frame_v1 [259:228] frame_v2 [291:260] frame_v3
    //   [323:292] frame_v4 [339:324] v1_shadow [355:340] v1_fifocnt
    mirror_content_s[10] = '0;
    for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
      mirror_content_s[10][core_idx * 16 +: 16] = prb_issue_nst_cnt_q[core_idx];
      mirror_content_s[10][64 + core_idx * 16 +: 16] = prb_issue_dir_cnt_q[core_idx];
      mirror_content_s[10][128 + core_idx * 16 +: 16] = prb_hold_set_cnt_q[core_idx];
    end
    mirror_content_s[10][195:192] =
      {prb_v4_sticky_q, prb_v3_sticky_q, prb_v2_sticky_q, prb_v1_sticky_q};
    mirror_content_s[10][197:196] = prb_v1_core_q;
    mirror_content_s[10][199:198] = prb_v2_core_q;
    mirror_content_s[10][201:200] = prb_v3_core_q;
    mirror_content_s[10][203:202] = prb_v4_core_q;
    mirror_content_s[10][227:196] = prb_v1_frame_q;
    mirror_content_s[10][259:228] = prb_v2_frame_q;
    mirror_content_s[10][291:260] = prb_v3_frame_q;
    mirror_content_s[10][323:292] = prb_v4_frame_q;
    mirror_content_s[10][339:324] = prb_v1_shadow_q;
    mirror_content_s[10][355:340] = prb_v1_fifocnt_q;
    mirror_content_s[10][356] = prb_v4_rd_done_q;
    end else begin
    // M11 (perf): backpressure cycles since GO
    //   [31:0]net_starve (fetcher/bridge starving PLAY)
    //   [63:32]space_stall (engine backpressure at CAPTURE/STATIC_WAIT)
    //   [95:64]emit_backpressure (issue stage valid with a closed emit gate)
    mirror_content_s[10] = '0;
    mirror_content_s[10][31:0] = perf_net_starve_q;
    mirror_content_s[10][63:32] = perf_space_stall_q;
    mirror_content_s[10][95:64] = perf_emit_bp_q;
    end
    if (PROBE_CORRECTNESS_EN) begin
    // M12: first-violation records
    //   [63:0]=V2 cmd record [127:64]=V3 lane record [191:128]=V4 old wait_cmd
    mirror_content_s[11] = '0;
    mirror_content_s[11][63:0] = prb_v2_rec_q;
    mirror_content_s[11][127:64] = prb_v3_rec_q;
    mirror_content_s[11][191:128] = prb_v4_rec_q;
    end else begin
    // M12 (perf): FIFO watermarks (max occupancy since GO)
    //   [9:0]cmd lane0 [19:10]cmd lane1 [29:20]cmd lane2 [39:30]cmd lane3
    //   [49:40]sw lane0 [59:50]sw lane1 [69:60]sw lane2 [79:70]sw lane3
    //   [89:80]net_fetcher NET fifo words [95:90]ctrl fifo words
    mirror_content_s[11] = '0;
    for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
      mirror_content_s[11][core_idx * 10 +: 10] = perf_cmd_wm_q[core_idx];
      mirror_content_s[11][40 + core_idx * 10 +: 10] = perf_sw_wm_q[core_idx];
    end
    mirror_content_s[11][89:80] = nf_perf_probe_i[8:0];
    mirror_content_s[11][95:90] = nf_perf_probe_i[14:9];
    end
    if (PROBE_CORRECTNESS_EN) begin
    // M13: mask-set content log
    //   [31:0]=mask_set total count [34:32]=log entries valid
    //   entry i at [96+98*i +: 98]: [31:0] frame_idx [33:32] core [97:34] lane_record
    mirror_content_s[12] = '0;
    mirror_content_s[12][31:0] = prb_mask_set_cnt_q;
    mirror_content_s[12][34:32] = prb_log_count_q;
    for (int log_idx = 0; log_idx < 4; log_idx++) begin
      mirror_content_s[12][96 + log_idx * 98 +: 32] = prb_log_frame_q[log_idx];
      mirror_content_s[12][96 + log_idx * 98 + 32 +: 2] = prb_log_core_q[log_idx];
      mirror_content_s[12][96 + log_idx * 98 + 34 +: 64] = prb_log_rec_q[log_idx];
    end
    end else begin
    // M13 (perf): update-rail status + raw-fetch burst statistics
    //   [7:0] upd rail probe [39:24] burst<8 [55:40] burst 8-31
    //   [71:56] burst 32-63 [87:72] burst=64 [119:88] total bursts
    //   [151:120] total lines [183:152] net_fifo_full cycles
    //   [215:184] ctrl_fifo_full cycles
    mirror_content_s[12] = '0;
    mirror_content_s[12][9:0] = sr_upd_probe_i;
    mirror_content_s[12][39:24] = nf_perf_probe_i[30:15];
    mirror_content_s[12][55:40] = nf_perf_probe_i[46:31];
    mirror_content_s[12][71:56] = nf_perf_probe_i[62:47];
    mirror_content_s[12][87:72] = nf_perf_probe_i[78:63];
    mirror_content_s[12][119:88] = nf_perf_probe_i[110:79];
    mirror_content_s[12][151:120] = nf_perf_probe_i[142:111];
    mirror_content_s[12][183:152] = nf_perf_probe_i[174:143];
    mirror_content_s[12][215:184] = nf_perf_probe_i[206:175];
    end
    // M14..M15: reserved
    mirror_content_s[13] = '0;
    mirror_content_s[14] = '0;
    if (!PROBE_CORRECTNESS_EN && (mirror_page_q == MIRROR_PAGE_STALL_ENG)) begin
      // STALL_ENG page: frame word 9+c = core c dispatcher state residency
      // (state s at [s*32 +: 32]); word 13 [127:0]=p req stall per core,
      // [255:128]=FE_WAIT_RESP residency; word 14 [127:0]=upd fifo warning
      // residency, [255:128]=emit backpressure per core
      for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
        for (int state_idx = 0; state_idx < 8; state_idx++) begin
          mirror_content_s[9 + core_idx][state_idx * 32 +: 32] =
            perf_dsp_state_q[core_idx][state_idx];
        end
        mirror_content_s[13][core_idx * 32 +: 32] = perf_preq_stall_q[core_idx];
        mirror_content_s[13][128 + core_idx * 32 +: 32] = perf_fe_wait_q[core_idx];
        mirror_content_s[14][core_idx * 32 +: 32] = perf_upd_warn_q[core_idx];
        mirror_content_s[14][128 + core_idx * 32 +: 32] = perf_emit_bp_core_q[core_idx];
      end
    end else if (!PROBE_CORRECTNESS_EN && (mirror_page_q == MIRROR_PAGE_STALL_BANK)) begin
      // STALL_BANK page: word 9 = per-bank p grant count, word 10 =
      // per-bank grad commit count, word 11 = per-source hold stall
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        mirror_content_s[9][bank_idx * 32 +: 32] = perf_pgrant_bank_q[bank_idx];
        mirror_content_s[10][bank_idx * 32 +: 32] = perf_commit_bank_q[bank_idx];
      end
      for (int src_idx = 0; src_idx < (GRAD_OVERFLOW_COUNT + GRAD_WLVIA_COUNT); src_idx++) begin
        mirror_content_s[11][src_idx * 32 +: 32] = perf_hold_stall_q[src_idx];
      end
    end
  end

  // The mirror is gated on the master run flag (GO -> RUN_DONE), NOT on
  // the per-block play flag: self_run_q rises/falls per block play, and a
  // block shorter than the dwell would reset the dwell before any frame
  // streamed (cutoff=1000 evidence: mirror never fired). sr_running_i
  // covers the whole run including inter-block spill/update.
  assign mirror_active_s = sr_running_i || (mirror_tail_q != 2'd0);
  // a frame completes when its last pending word actually lands on the
  // port (valid && no FSM write that cycle)
  assign mirror_frame_done_s =
    (mirror_state_q == MIR_STREAM) && (mirror_bram_we_q != '0) &&
    (fsm_bram_we_q == '0) && (mirror_wr_idx_q == 5'(MIRROR_WORDS - 1));

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mirror_state_q <= MIR_DWELL;
      mirror_frame_seq_q <= '0;
      mirror_dwell_q <= '0;
      mirror_wr_idx_q <= '0;
      mirror_bram_we_q <= '0;
      mirror_bram_addr_q <= '0;
      mirror_bram_wdata_q <= '0;
      mirror_tail_q <= '0;
      sr_running_prev_q <= 1'b0;
      for (int w = 0; w < (MIRROR_WORDS - 1); w++) begin
        mirror_snap_q[w] <= '0;
      end
    end else begin
      // tail credit: when the self-run retires, keep streaming two more
      // full frames so the frame carrying the done latch always lands
      sr_running_prev_q <= sr_running_i;
      if (sr_running_prev_q && !sr_running_i) begin
        mirror_tail_q <= 2'd2;
      end else if (mirror_kick_q && !sr_running_i) begin
        // host page-switch kick: stream one extra frame carrying the new
        // mirror page after the self-run has retired
        mirror_tail_q <= 2'd1;
      end else if ((mirror_tail_q != 2'd0) && mirror_frame_done_s) begin
        mirror_tail_q <= mirror_tail_q - 1'b1;
      end

      unique case (mirror_state_q)
        MIR_DWELL: begin
          mirror_bram_we_q <= '0;
          // stream only while a self-run owns the adapter (plus the tail
          // frames above); outside self-run the payload region (incl. the
          // mirror words) belongs to host command traffic (v1 NET payloads
          // may use the full BRAM)
          if (!mirror_active_s) begin
            mirror_dwell_q <= 17'(MIRROR_DWELL_CYCLES);
          end else if (mirror_dwell_q != '0) begin
            mirror_dwell_q <= mirror_dwell_q - 1'b1;
          end else begin
            mirror_state_q <= MIR_SNAP;
          end
        end
        MIR_SNAP: begin
          // one-cycle atomic snapshot of the whole frame
          for (int w = 0; w < (MIRROR_WORDS - 1); w++) begin
            mirror_snap_q[w] <= mirror_content_s[w];
          end
          mirror_frame_seq_q <= mirror_frame_seq_q + 1'b1;
          mirror_wr_idx_q <= '0;
          mirror_state_q <= MIR_STREAM;
        end
        MIR_STREAM: begin
          // valid/ready write handshake: the pending word (valid) is held
          // until a cycle with no FSM write (ready) carries it onto the
          // port; FSM writes always win the mux, so a pending word simply
          // waits — it is never silently dropped.
          if (mirror_bram_we_q == '0) begin
            mirror_bram_we_q <= {BRAM_STRB_W{1'b1}};
            mirror_bram_addr_q <= bram_addr(MIRROR_BASE_INDEX + mirror_wr_idx_q);
            if (mirror_wr_idx_q == 5'd0) begin
              mirror_bram_wdata_q <= {
                {(BRAM_DATA_W - 82) {1'b0}},
                2'(mirror_page_q),
                16'(MIRROR_WORDS), mirror_frame_seq_q, MIRROR_MAGIC
              };
            end else begin
              mirror_bram_wdata_q <= mirror_snap_q[
                (mirror_wr_idx_q <= 5'd1) ? 5'd0 : (mirror_wr_idx_q - 1'b1)
              ];
            end
          end else if (fsm_bram_we_q == '0) begin
            // the pending word reached the port this cycle
            mirror_bram_we_q <= '0;
            if (mirror_wr_idx_q == 5'(MIRROR_WORDS - 1)) begin
              mirror_state_q <= MIR_DWELL;
              mirror_dwell_q <= 17'(MIRROR_DWELL_CYCLES);
              mirror_wr_idx_q <= '0;
            end else begin
              mirror_wr_idx_q <= mirror_wr_idx_q + 1'b1;
            end
          end
        end
        default: begin
          mirror_state_q <= MIR_DWELL;
        end
      endcase
    end
  end

  // ---------------------------------------------------------------------
  // Self-run pairing probes (V-series): per-lane push/pop/issue counters
  // plus sticky first-violation captures for the four disjoint orphan
  // mechanisms (V1 accounting desync, V2 need_static cmd issued via the
  // direct path, V3 static push without its cmd, V4 wait_cmd overwrite
  // while a static read is pending). Passive observation only; FF only;
  // read out through the status mirror (M10..M12). Instantiated only when
  // PROBE_CORRECTNESS_EN=1 (pruned otherwise).
  // ---------------------------------------------------------------------
  generate
    for (genvar prb_g = 0; prb_g < CORE_COUNT; prb_g++) begin : GEN_PRB_FIFO_CNT
      assign prb_fifo_cnt_s[prb_g] =
        GEN_ADAPTER_FIFOS[prb_g].u_static_w_fifo.count_q;
      assign perf_cmd_count_s[prb_g] =
        GEN_ADAPTER_FIFOS[prb_g].u_cmd_fifo.count_q;
    end
  endgenerate

  generate
    if (PROBE_CORRECTNESS_EN) begin : GEN_CORRECTNESS_PROBES
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      prb_cmd_pushed_flag_q <= '0;
      prb_v1_sticky_q <= 1'b0;
      prb_v2_sticky_q <= 1'b0;
      prb_v3_sticky_q <= 1'b0;
      prb_v4_sticky_q <= 1'b0;
      prb_v1_core_q <= '0;
      prb_v2_core_q <= '0;
      prb_v3_core_q <= '0;
      prb_v4_core_q <= '0;
      prb_v1_frame_q <= '0;
      prb_v2_frame_q <= '0;
      prb_v3_frame_q <= '0;
      prb_v4_frame_q <= '0;
      prb_v1_shadow_q <= '0;
      prb_v1_fifocnt_q <= '0;
      prb_v2_rec_q <= '0;
      prb_v3_rec_q <= '0;
      prb_v4_rec_q <= '0;
      prb_v4_rd_done_q <= 1'b0;
      prb_mask_set_cnt_q <= '0;
      prb_log_count_q <= '0;
      for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
        prb_sw_push_cnt_q[core_idx] <= '0;
        prb_sw_pop_cnt_q[core_idx] <= '0;
        prb_cmd_push_cnt_q[core_idx] <= '0;
        prb_issue_nst_cnt_q[core_idx] <= '0;
        prb_issue_dir_cnt_q[core_idx] <= '0;
        prb_hold_set_cnt_q[core_idx] <= '0;
        prb_v1_mismatch_cnt_q[core_idx] <= '0;
      end
      for (int log_idx = 0; log_idx < 4; log_idx++) begin
        prb_log_frame_q[log_idx] <= '0;
        prb_log_core_q[log_idx] <= '0;
        prb_log_rec_q[log_idx] <= '0;
      end
    end else if (!enable_i || full_reset_active_q) begin
      prb_cmd_pushed_flag_q <= '0;
      prb_v1_sticky_q <= 1'b0;
      prb_v2_sticky_q <= 1'b0;
      prb_v3_sticky_q <= 1'b0;
      prb_v4_sticky_q <= 1'b0;
      prb_v4_rd_done_q <= 1'b0;
      prb_mask_set_cnt_q <= '0;
      prb_log_count_q <= '0;
      for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
        prb_sw_push_cnt_q[core_idx] <= '0;
        prb_sw_pop_cnt_q[core_idx] <= '0;
        prb_cmd_push_cnt_q[core_idx] <= '0;
        prb_issue_nst_cnt_q[core_idx] <= '0;
        prb_issue_dir_cnt_q[core_idx] <= '0;
        prb_hold_set_cnt_q[core_idx] <= '0;
        prb_v1_mismatch_cnt_q[core_idx] <= '0;
      end
    end else begin
      // mask-set content log: replicate the FSM's mask-set condition
      // (CAPTURE && space && lane_valid && need_static) per lane; count ALL
      // sets per cycle (popcount) and keep the last 4 events, prioritizing
      // the lowest core so lane-0 phantoms are always captured
      begin
        logic [63:0] prb_lane_rec_v [0:CORE_COUNT-1];
        logic [CORE_COUNT-1:0] prb_set_v;
        int unsigned prb_set_n;
        int prb_log_core_sel;
        prb_set_n = 0;
        prb_log_core_sel = -1;
        for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
          prb_lane_rec_v[core_idx] = net_frame_word_s[(core_idx * 64) +: 64];
          prb_set_v[core_idx] =
            ((state_q == A_NET_LANE_CAPTURE) ||
             ((state_q == A_NET_STREAM) && sr_frame_valid_i)) &&
            net_fifo_space_ready_s &&
            prb_lane_rec_v[core_idx][0] &&
            lane_needs_static_weight(prb_lane_rec_v[core_idx]);
          if (prb_set_v[core_idx]) begin
            prb_set_n = prb_set_n + 1;
            if (prb_log_core_sel < 0) begin
              prb_log_core_sel = core_idx;
            end
          end
        end
        if (prb_set_n != 0) begin
          prb_mask_set_cnt_q <= prb_mask_set_cnt_q + 32'(prb_set_n);
          prb_log_count_q <= (prb_log_count_q == 3'd4) ? 3'd4 : (prb_log_count_q + 1'b1);
          for (int log_idx = 3; log_idx > 0; log_idx--) begin
            prb_log_frame_q[log_idx] <= prb_log_frame_q[log_idx - 1];
            prb_log_core_q[log_idx] <= prb_log_core_q[log_idx - 1];
            prb_log_rec_q[log_idx] <= prb_log_rec_q[log_idx - 1];
          end
          prb_log_frame_q[0] <= net_frame_idx_q;
          prb_log_core_q[0] <= 2'(prb_log_core_sel);
          prb_log_rec_q[0] <= prb_lane_rec_v[prb_log_core_sel];
        end
      end
      for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
        if (static_w_fifo_push_s[core_idx]) begin
          prb_sw_push_cnt_q[core_idx] <= prb_sw_push_cnt_q[core_idx] + 1'b1;
        end
        if (static_w_fifo_pop_s[core_idx]) begin
          prb_sw_pop_cnt_q[core_idx] <= prb_sw_pop_cnt_q[core_idx] + 1'b1;
        end
        if (cmd_fifo_push_s[core_idx]) begin
          prb_cmd_push_cnt_q[core_idx] <= prb_cmd_push_cnt_q[core_idx] + 1'b1;
        end
        if (iss_s2_adv_s[core_idx] && iss_s2_need_static_q[core_idx]) begin
          prb_issue_nst_cnt_q[core_idx] <= prb_issue_nst_cnt_q[core_idx] + 1'b1;
        end
        if (iss_s2_adv_s[core_idx] && !iss_s2_need_static_q[core_idx]) begin
          prb_issue_dir_cnt_q[core_idx] <= prb_issue_dir_cnt_q[core_idx] + 1'b1;
        end
        // sw-empty stall cycles at S1->S2 (the retired cmd_hold's role)
        if (
          net_issue_active_s &&
          iss_s1_valid_q[core_idx][iss_s1_rd_ptr_q[core_idx]] &&
          lane_needs_static_weight(iss_s1_data_q[core_idx][iss_s1_rd_ptr_q[core_idx]]) &&
          !(
            (static_w_fifo_shadow_q[core_idx] != '0) &&
            static_w_fifo_ready_s[core_idx]
          )
        ) begin
          prb_hold_set_cnt_q[core_idx] <= prb_hold_set_cnt_q[core_idx] + 1'b1;
        end

        // per-frame cmd-pushed flag (fresh at every CAPTURE / stream fire)
        if ((state_q == A_NET_LANE_CAPTURE) || (state_q == A_NET_STREAM)) begin
          prb_cmd_pushed_flag_q[core_idx] <= cmd_fifo_push_s[core_idx];
        end

        // V2: a need_static lane record advanced as a plain (unsubstituted)
        // command — the head decode and the transfer must always agree
        if (
          !prb_v2_sticky_q && iss_s2_adv_s[core_idx] &&
          !iss_s2_need_static_q[core_idx] &&
          lane_needs_static_weight(iss_s2_cmd_q[core_idx])
        ) begin
          prb_v2_sticky_q <= 1'b1;
          prb_v2_core_q <= 2'(core_idx);
          prb_v2_rec_q <= iss_s2_cmd_q[core_idx];
          prb_v2_frame_q <= net_frame_idx_q;
        end

        // V3: a static weight pushed for a lane whose cmd never entered
        // the cmd FIFO
        if (
          !prb_v3_sticky_q && static_w_fifo_push_s[core_idx] &&
          net_pending_static_mask_q[core_idx] && !prb_cmd_pushed_flag_q[core_idx]
        ) begin
          prb_v3_sticky_q <= 1'b1;
          prb_v3_core_q <= 2'(core_idx);
          prb_v3_rec_q <= net_frame_word_s[(core_idx * 64) +: 64];
          prb_v3_frame_q <= net_frame_idx_q;
        end

        // V4: a sw FIFO pop without a need-static command at the S1->S2
        // transfer (pairing/count guard for the pipeline static path)
        if (
          !prb_v4_sticky_q && static_w_fifo_pop_s[core_idx] &&
          !lane_needs_static_weight(iss_s1_data_q[core_idx][iss_s1_rd_ptr_q[core_idx]])
        ) begin
          prb_v4_sticky_q <= 1'b1;
          prb_v4_core_q <= 2'(core_idx);
          prb_v4_rec_q <= iss_s1_data_q[core_idx][iss_s1_rd_ptr_q[core_idx]];
          prb_v4_frame_q <= net_frame_idx_q;
          prb_v4_rd_done_q <= 1'b0;
        end

        // V1: adapter shadow vs the ring's own count. The shadow legally
        // trails a push by one cycle (push_d1), so fifo>shadow by exactly 1
        // is normal; flag only a real desync: fifo ahead by >=2 or shadow
        // ahead at all, persisting 3+ cycles.
        if (
          (32'(prb_fifo_cnt_s[core_idx]) > (static_w_fifo_shadow_q[core_idx] + 1'b1)) ||
          (static_w_fifo_shadow_q[core_idx] > 32'(prb_fifo_cnt_s[core_idx]))
        ) begin
          if (prb_v1_mismatch_cnt_q[core_idx] != 4'd15) begin
            prb_v1_mismatch_cnt_q[core_idx] <= prb_v1_mismatch_cnt_q[core_idx] + 1'b1;
          end
        end else begin
          prb_v1_mismatch_cnt_q[core_idx] <= '0;
        end
        if (!prb_v1_sticky_q && (prb_v1_mismatch_cnt_q[core_idx] >= 4'd3)) begin
          prb_v1_sticky_q <= 1'b1;
          prb_v1_core_q <= 2'(core_idx);
          prb_v1_shadow_q <= 16'(static_w_fifo_shadow_q[core_idx]);
          prb_v1_fifocnt_q <= 16'(prb_fifo_cnt_s[core_idx]);
          prb_v1_frame_q <= net_frame_idx_q;
        end
      end
    end
  end
    end
  endgenerate

  function automatic logic [BRAM_DATA_W-1:0] pack_event_word();
    logic [BRAM_DATA_W-1:0] word;
    begin
      word = '0;
      word[31:0] = event_counter_q;
      word[63:32] = elapsed_cycles_q;
      word[95:64] = wait_elapsed_q;
      word[127:96] = {24'd0, system_phase_i};
      pack_event_word = word;
    end
  endfunction

  function automatic logic [BRAM_DATA_W-1:0] pack_caps_word(
    input int unsigned cap_index
  );
    logic [BRAM_DATA_W-1:0] word;
    begin
      word = '0;
      unique case (cap_index)
        0: begin
          word[15:0] = CMD_MAGIC;
          word[23:16] = PROTOCOL_VERSION;
          word[39:24] = 16'(BRAM_DATA_W);
          word[63:40] = 24'(BRAM_DEPTH);
        end
        1: begin
          word[31:0] = 32'(CORE_COUNT);
          word[63:32] = 32'(CHUNK_SIZE);
          word[95:64] = 32'(BANK_COUNT);
          word[127:96] = 32'(BANK_DEPTH);
        end
        2: begin
          word[31:0] = 32'(DATA_W);
          word[63:32] = 32'(FRAC_W);
          word[95:64] = 32'(P_DATA_W);
          word[127:96] = 32'(COEFF_W);
        end
        3: begin
          word[0] = 1'b1;   // NOP
          word[1] = 1'b1;   // GET_CAPS
          word[2] = 1'b1;   // CONFIG
          word[3] = 1'b1;   // NET
          word[4] = 1'b1;   // WAIT
          word[5] = 1'b1;   // INIT_LOGITS_DDR
          word[6] = 1'b1;   // INIT_STATIC_WEIGHTS
           word[7] = 1'b1;   // INIT_RMS_DDR
           word[8] = 1'b1;   // INIT_GROUP_FLAGS
          word[9] = 1'b1;   // READ_P
          word[10] = 1'b1;  // READ_GRAD_P
          word[16] = 1'b1;  // DEBUG
          word[32] = 1'b1;  // RESET
        end
        4: begin
          word[0] = 1'b1;   // 512-bit BRAM command words.
          word[1] = 1'b1;   // Low-32-bit submit detection.
          word[2] = 1'b1;   // Level-held usr_irq_req.
          word[3] = 1'b1;   // NET lane records are 64-bit slots.
          word[4] = 1'b1;   // NET prefix/SETW weights use static table indexes.
          word[5] = BLOCKED_SCHED;   // Blocked-GRADP v2 command set
                                     // (LOAD_BLOCK_CTRL/LOAD_BLOCK_DESC/INIT_SEG,
                                     //  block-gated NET with net_flags[4]/[3])
        end
        default: begin
          word = '0;
        end
      endcase
      pack_caps_word = word;
    end
  endfunction

  function automatic banked_idx_t decode_banked_idx23(
    input logic [22:0] packed_idx,
    input logic is_window_slot
  );
    banked_idx_t result;
    begin
      result = '0;
      if (BLOCKED_GRADP && is_window_slot) begin
        // Blocked-GRADP window slot re-decode: the host packs slot directly
        // into idx23 (slot < 2^17 for the 128K window halves). Recover the
        // downstream (bank, mem) semantics:
        //   bank = slot[2:0]              (sub-bank)
        //   mem  = slot[16:3] (14 bits)   (row within sub-bank)
        // NOTE: keep the part-selects within packed_idx[22:0]; wider slices
        // of the bank field blow up under synthesis (BANK_IDX_W=8 there).
        result.bank_idx = packed_idx[2:0];
        result.mem_idx[13:0] = packed_idx[16:3];
      end else begin
        if (MEM_IDX_W >= 16) begin
          result.mem_idx[15:0] = packed_idx[15:0];
        end else begin
          result.mem_idx = packed_idx[MEM_IDX_W-1:0];
        end
        if (BANK_IDX_W >= 7) begin
          result.bank_idx[6:0] = packed_idx[22:16];
        end else begin
          result.bank_idx = packed_idx[16 +: BANK_IDX_W];
        end
      end
      decode_banked_idx23 = result;
    end
  endfunction

  function automatic logic lane_needs_static_weight(
    input logic [63:0] lane_record
  );
    begin
      lane_needs_static_weight =
        (lane_record[60:59] == OVERFLOW_MODE_SETW) ||
        ((lane_record[60:59] == OVERFLOW_MODE_SEGMENT) && (lane_record[55:40] == 16'd0));
    end
  endfunction

  function automatic logic [ADDR_W-1:0] decode_addr8(
    input logic [7:0] value
  );
    logic [ADDR_W-1:0] result;
    begin
      result = ADDR_W'(value);
      decode_addr8 = result;
    end
  endfunction

  function automatic logic signed [COEFF_W-1:0] decode_coeff16(
    input logic signed [15:0] value
  );
    begin
      if (COEFF_W > 16) begin
        decode_coeff16 = {{(COEFF_W-16){value[15]}}, value};
      end else if (COEFF_W == 16) begin
        decode_coeff16 = value;
      end else begin
        decode_coeff16 = value[COEFF_W-1:0];
      end
    end
  endfunction

  function automatic logic [BANK_VALID_LEN_W-1:0] decode_bank_len32(
    input logic [31:0] value
  );
    logic [BANK_VALID_LEN_W-1:0] result;
    begin
      result = BANK_VALID_LEN_W'(value);
      decode_bank_len32 = result;
    end
  endfunction

  function automatic logic signed [DATA_W-1:0] decode_logits32(
    input logic signed [31:0] value
  );
    begin
      if (DATA_W > 32) begin
        decode_logits32 = {{(DATA_W-32){value[31]}}, value};
      end else if (DATA_W == 32) begin
        decode_logits32 = value;
      end else begin
        decode_logits32 = value[DATA_W-1:0];
      end
    end
  endfunction

  function automatic logic signed [RMS_V_W-1:0] decode_rms64(
    input logic signed [63:0] value
  );
    begin
      if (RMS_V_W > 64) begin
        decode_rms64 = {{(RMS_V_W-64){value[63]}}, value};
      end else if (RMS_V_W == 64) begin
        decode_rms64 = value;
      end else begin
        decode_rms64 = value[RMS_V_W-1:0];
      end
    end
  endfunction

  function automatic logic [STATIC_WEIGHT_WORD_ADDR_W-1:0] static_weight_word_addr(
    input logic [MEM_IDX_W-1:0] weight_idx
  );
    begin
      static_weight_word_addr = STATIC_WEIGHT_WORD_ADDR_W'(weight_idx >> STATIC_WEIGHT_LANE_W);
    end
  endfunction

  function automatic logic [STATIC_WEIGHT_LANE_W-1:0] static_weight_lane(
    input logic [MEM_IDX_W-1:0] weight_idx
  );
    begin
      static_weight_lane = STATIC_WEIGHT_LANE_W'(weight_idx);
    end
  endfunction

  function automatic logic signed [DATA_W-1:0] select_static_weight(
    input logic [BRAM_DATA_W-1:0] word,
    input logic [STATIC_WEIGHT_LANE_W-1:0] lane
  );
    logic signed [31:0] raw_weight;
    begin
      raw_weight = word[(int'(lane) * 32) +: 32];
      select_static_weight = decode_logits32(raw_weight);
    end
  endfunction

  function automatic logic [7:0] descriptor_error();
    int unsigned payload_start_int;
    int unsigned payload_end_int;
    int unsigned result_start_int;
    int unsigned result_end_int;
    int unsigned payload_len_int;
    int unsigned read_end_int;
    int unsigned read_rows_per_word_int;
    int unsigned read_required_words_int;
    begin
      payload_start_int = int'(payload_start_q);
      payload_end_int = int'(payload_end_q);
      result_start_int = int'(result_start_q);
      result_end_int = int'(result_start_q + result_capacity_q);
      payload_len_int = (payload_end_int >= payload_start_int) ? (payload_end_int - payload_start_int) : 0;
      read_end_int = int'(arg0_q) + int'(arg1_q);
      read_rows_per_word_int = (cmd_type_q == CMD_READ_GRAD_P) ?
        GRAD_ROWS_PER_WORD : P_ROWS_PER_WORD;
      read_required_words_int =
        (int'(arg1_q) + read_rows_per_word_int - 1) / read_rows_per_word_int;

      if (desc_magic_q != CMD_MAGIC) begin
        descriptor_error = ERR_BAD_MAGIC;
      end else if (desc_version_q != PROTOCOL_VERSION) begin
        descriptor_error = ERR_BAD_VERSION;
      end else if (
        (cmd_type_q != CMD_NOP) &&
        (cmd_type_q != CMD_GET_CAPS) &&
        (cmd_type_q != CMD_CONFIG) &&
        (cmd_type_q != CMD_NET) &&
        (cmd_type_q != CMD_WAIT) &&
        (cmd_type_q != CMD_INIT_LOGITS_DDR) &&
        (cmd_type_q != CMD_INIT_STATIC_WEIGHTS) &&
        (cmd_type_q != CMD_INIT_RMS_DDR) &&
        (cmd_type_q != CMD_INIT_GROUP_FLAGS) &&
        (cmd_type_q != CMD_READ_P) &&
        (cmd_type_q != CMD_READ_GRAD_P) &&
        (cmd_type_q != CMD_DEBUG) &&
        (cmd_type_q != CMD_RESET) &&
        (cmd_type_q != CMD_LOAD_BLOCK_CTRL) &&
        (cmd_type_q != CMD_LOAD_BLOCK_DESC) &&
        (cmd_type_q != CMD_INIT_SEG) &&
        (cmd_type_q != CMD_GO) &&
        (cmd_type_q != CMD_DBG_LOAD_BLOCK) &&
        (cmd_type_q != CMD_SET_MIRROR_PAGE)
      ) begin
        descriptor_error = ERR_UNKNOWN_CMD;
      end else if (
        // NET-on-DDR self-run: the per-command ctrl lane stream is retired,
        // and a blocked build replays NET frames from the DDR image (the
        // monolithic CMD_NET flow keeps its original meaning at
        // BLOCKED_SCHED=0)
        (cmd_type_q == CMD_LOAD_BLOCK_CTRL) ||
        (BLOCKED_SCHED && (cmd_type_q == CMD_NET))
      ) begin
        descriptor_error = ERR_UNSUPPORTED_CMD;
      end else if (
        ((cmd_type_q == CMD_GO) && (arg0_q == 0))
      ) begin
        descriptor_error = ERR_RANGE_ERROR;
      end else if (
        (payload_end_int < payload_start_int) ||
        ((payload_len_int != 0) && (payload_start_int < PAYLOAD_BASE_INDEX)) ||
        (payload_end_int > SUBMIT_INDEX) ||
        ((result_capacity_q != 0) && (result_start_int < PAYLOAD_BASE_INDEX)) ||
        (result_end_int > SUBMIT_INDEX)
      ) begin
        descriptor_error = ERR_RANGE_ERROR;
      end else if (
        ((cmd_type_q == CMD_READ_P) || (cmd_type_q == CMD_READ_GRAD_P)) &&
        (payload_len_int != 0)
      ) begin
        descriptor_error = ERR_BAD_LENGTH;
      end else if (
        (cmd_type_q == CMD_READ_P) &&
        (system_phase_i != 4'd1) &&
        (system_phase_i != 4'd5)
      ) begin
        descriptor_error = ERR_ROUTER_NOT_READY;
      end else if (
        ((cmd_type_q == CMD_READ_P) || (cmd_type_q == CMD_READ_GRAD_P)) &&
        (
          (arg1_q == 0) ||
          (arg0_q != readback_expected_row_q) ||
          (arg0_q >= BANK_DEPTH) ||
          (read_end_int > BANK_DEPTH)
        )
      ) begin
        descriptor_error = ERR_RANGE_ERROR;
      end else if (
        ((cmd_type_q == CMD_READ_P) || (cmd_type_q == CMD_READ_GRAD_P)) &&
        ((result_capacity_q == 0) || (read_required_words_int > int'(result_capacity_q)))
      ) begin
        descriptor_error = ERR_BAD_LENGTH;
      end else if (
        ((cmd_type_q == CMD_READ_P) || (cmd_type_q == CMD_READ_GRAD_P)) &&
        arg2_q[0] &&
        // Blocked-GRADP v2: bank_valid_len is configured to the full p
        // window depth (bridge resp mask pass-through) while READ_P walks
        // only the used rows, so the monolithic "advance must cover the
        // whole bank" rule does not apply.
        !BLOCKED_SCHED &&
        (read_end_int < int'(max_bank_valid_len_s))
      ) begin
        descriptor_error = ERR_RANGE_ERROR;
      end else if ((cmd_type_q == CMD_GET_CAPS) && (result_capacity_q < 5)) begin
        descriptor_error = ERR_BAD_LENGTH;
      end else if ((cmd_type_q == CMD_CONFIG) && (payload_len_int < 1)) begin
        descriptor_error = ERR_BAD_LENGTH;
      end else if ((cmd_type_q == CMD_NET) && (payload_len_int < 1)) begin
        descriptor_error = ERR_BAD_LENGTH;
      end else if ((cmd_type_q == CMD_INIT_LOGITS_DDR) && (payload_len_int < 1)) begin
        descriptor_error = ERR_BAD_LENGTH;
      end else if ((cmd_type_q == CMD_INIT_STATIC_WEIGHTS) && (payload_len_int < 1)) begin
        descriptor_error = ERR_BAD_LENGTH;
      end else if ((cmd_type_q == CMD_INIT_RMS_DDR) && (payload_len_int < 1)) begin
        descriptor_error = ERR_BAD_LENGTH;
      end else if ((cmd_type_q == CMD_INIT_SEG) && (payload_len_int < 1)) begin
        descriptor_error = ERR_BAD_LENGTH;
      end else if ((cmd_type_q == CMD_LOAD_BLOCK_DESC) && (payload_len_int < 1)) begin
        descriptor_error = ERR_BAD_LENGTH;
      end else if (
        (cmd_type_q == CMD_INIT_GROUP_FLAGS) &&
        (
          (arg1_q == 0) ||
          (arg0_q >= GROUP_FLAG_MEM_DEPTH) ||
          (arg1_q > (GROUP_FLAG_MEM_DEPTH - arg0_q))
        )
      ) begin
        descriptor_error = ERR_RANGE_ERROR;
      end else if (
        (cmd_type_q == CMD_INIT_GROUP_FLAGS) &&
        (payload_len_int < int'(arg1_q))
      ) begin
        descriptor_error = ERR_BAD_LENGTH;
      end else if ((cmd_type_q == CMD_DEBUG) && (result_capacity_q < 7)) begin
        descriptor_error = ERR_BAD_LENGTH;
      end else begin
        descriptor_error = ERR_OK;
      end
    end
  endfunction

  always_comb begin
    net_fifo_space_ready_s = 1'b1;
    net_all_fifo_empty_s = 1'b1;
    net_fifo_residue_detail_s = '0;
    net_lane_static_bad_s = 1'b0;
    net_issue_active_s = 1'b0;
    static_mem_wr_en_s = enable_i && (state_q == A_STATIC_DATA_CAPTURE);
    static_mem_wr_addr_s = STATIC_WEIGHT_WORD_ADDR_W'(static_payload_word_idx_q);
    static_mem_wr_data_s = bram_data_q;
    // net_fifo_space_ready_s must be fully accumulated across ALL lanes
    // before any consumer is evaluated: computing it inside the per-lane
    // loop below let lane i's pushes see a partially-accumulated value
    // (only lanes <= i checked), so while the FSM parked on another lane's
    // fifo warning, lane 0 kept pushing every cycle (the cutoff=0 phantom
    // static weights, 2026-08-26 root cause).
    for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
      if (
        !cmd_fifo_ready_s[core_idx] ||
        !static_w_fifo_ready_s[core_idx] ||
        cmd_fifo_full_warning_s[core_idx] ||
        static_w_fifo_full_warning_s[core_idx]
      ) begin
        net_fifo_space_ready_s = 1'b0;
      end
    end
    for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
      int overflow_group_idx;
      int overflow_lane_idx;
      logic [63:0] lane_record;
      logic lane_valid;
      logic lane_need_static;
      logic [6:0] packed_bank;
      logic [15:0] packed_mem;
      overflow_group_idx = core_idx / CORE_COUNT_PER_FIFO;
      overflow_lane_idx = core_idx % CORE_COUNT_PER_FIFO;
      lane_record = net_frame_word_s[(core_idx * 64) +: 64];
      lane_valid = lane_record[0];
      lane_need_static = lane_needs_static_weight(lane_record);
      packed_bank = lane_record[23:17];
      packed_mem = lane_record[16:1];

      cmd_fifo_head_need_static_s[core_idx] = lane_needs_static_weight(cmd_fifo_head_s[core_idx]);

      if (
        (cmd_fifo_shadow_q[core_idx] != '0) ||
        (static_w_fifo_shadow_q[core_idx] != '0) ||
        cmd_fifo_output_valid_s[core_idx] ||
        static_w_fifo_output_valid_s[core_idx] ||
        (iss_s1_count_q[core_idx] != '0) ||
        iss_s2_valid_q[core_idx] ||
        iss_s3_valid_q[core_idx]
      ) begin
        net_all_fifo_empty_s = 1'b0;
      end

      // Residue bitmap for the pre-command check, one nibble per condition
      // group. Mirrors the net_all_fifo_empty_s composition above.
      net_fifo_residue_detail_s[core_idx]      = (cmd_fifo_shadow_q[core_idx] != '0);
      net_fifo_residue_detail_s[4 + core_idx]  = (static_w_fifo_shadow_q[core_idx] != '0);
      net_fifo_residue_detail_s[8 + core_idx]  = cmd_fifo_output_valid_s[core_idx];
      net_fifo_residue_detail_s[12 + core_idx] = static_w_fifo_output_valid_s[core_idx];
      net_fifo_residue_detail_s[16 + core_idx] = (iss_s1_count_q[core_idx] != '0);
      net_fifo_residue_detail_s[20 + core_idx] = iss_s2_valid_q[core_idx];
      net_fifo_residue_detail_s[24 + core_idx] = iss_s3_valid_q[core_idx];
      net_fifo_residue_detail_s[28 + core_idx] = 1'b0;

      if (
        enable_i &&
        (state_q == A_NET_STREAM) && sr_frame_valid_i && net_fifo_space_ready_s &&
        lane_valid &&
        lane_need_static &&
        ((packed_bank != 7'd0) || (32'(packed_mem) >= static_weight_count_q))
      ) begin
        net_lane_static_bad_s = 1'b1;
      end

      static_weight_needed_s[core_idx] = lane_need_static;
      static_mem_rd_en_s[core_idx] =
        enable_i &&
        (state_q == A_NET_STREAM) && sr_frame_valid_i && net_fifo_space_ready_s &&
        lane_valid &&
        lane_need_static &&
        !((packed_bank != 7'd0) || (32'(packed_mem) >= static_weight_count_q));
      static_mem_rd_addr_s[core_idx] = static_weight_word_addr(MEM_IDX_W'(packed_mem));

      cmd_fifo_push_s[core_idx] =
        enable_i &&
        (state_q == A_NET_STREAM) && sr_frame_valid_i &&
        net_fifo_space_ready_s &&
        lane_valid &&
        !net_lane_static_bad_s;
      cmd_fifo_push_data_s[core_idx] = lane_record;
      static_w_fifo_push_s[core_idx] =
        enable_i &&
        (state_q == A_NET_STREAM) &&
        sr_pipe_valid_q &&
        sr_pipe_static_mask_q[core_idx];
      static_w_fifo_push_data_s[core_idx] =
        select_static_weight(
          static_mem_rd_data_s[core_idx],
          sr_pipe_static_lane_q[core_idx]
        );

      cmd_fifo_pop_s[core_idx] = 1'b0;
      static_w_fifo_pop_s[core_idx] = 1'b0;
      iss_s1_adv_s[core_idx] = 1'b0;
      iss_s2_adv_s[core_idx] = 1'b0;
      iss_s3_emit_s[core_idx] = 1'b0;

      unique case (state_q)
        A_NET_LANE_ADDR,
        A_NET_STREAM,
        A_NET_ISSUE: begin
          net_issue_active_s = 1'b1;
        end
        default: begin
        end
      endcase

      // S3 -> emit: the only downstream gate (host_ready / wlvia / NEW_TASK
      // needs grad_task_ready). A fire advances S3 this cycle.
      if (
        enable_i &&
        net_issue_active_s &&
        iss_s3_valid_q[core_idx] &&
        grad_overflow_host_ready_i[overflow_group_idx][overflow_lane_idx] &&
        // WLVIA backpressure: pause emitting further segment words for this
        // lane while its wlvia result FIFO is near full. The downstream
        // dispatcher tolerates idle cycles between commands, so pausing the
        // broadcast here is order- and value-preserving.
        !grad_wlvia_fifo_warning_i[core_idx / SEGMENT_PER_FIFO] &&
        (
          (iss_s3_cmd_q[core_idx][60:59] != OVERFLOW_MODE_NEW_TASK) ||
          grad_task_ready_i[core_idx]
        )
      ) begin
        iss_s3_emit_s[core_idx] = 1'b1;
      end

      // S2 -> S3: whenever S3 has room. The command's static weight (if any)
      // was already popped when it entered S2, so the readable sw head now
      // holds the right word.
      if (
        enable_i &&
        net_issue_active_s &&
        iss_s2_valid_q[core_idx] &&
        (!iss_s3_valid_q[core_idx] || iss_s3_emit_s[core_idx])
      ) begin
        iss_s2_adv_s[core_idx] = 1'b1;
      end

      // S1 -> S2: the head slot carries a complete {valid,data} pair; the
      // transfer is pulse-independent, so S2 backpressure can never lose
      // it. A need-static command pops the sw FIFO in the same cycle (the
      // weight becomes the readable head next cycle, in time for the S3
      // load); it stalls in place while the sw FIFO is empty (strict
      // order: nothing behind it may pass).
      if (
        enable_i &&
        net_issue_active_s &&
        iss_s1_valid_q[core_idx][iss_s1_rd_ptr_q[core_idx]] &&
        (!iss_s2_valid_q[core_idx] || iss_s2_adv_s[core_idx]) &&
        (
          !lane_needs_static_weight(iss_s1_data_q[core_idx][iss_s1_rd_ptr_q[core_idx]]) ||
          (
            (static_w_fifo_shadow_q[core_idx] != '0) &&
            static_w_fifo_ready_s[core_idx]
          )
        )
      ) begin
        iss_s1_adv_s[core_idx] = 1'b1;
        if (lane_needs_static_weight(iss_s1_data_q[core_idx][iss_s1_rd_ptr_q[core_idx]])) begin
          static_w_fifo_pop_s[core_idx] = 1'b1;
        end
      end

      // S1 entry: pop the next command whenever a slot is free (or freed by
      // this cycle's transfer).
      if (
        enable_i &&
        net_issue_active_s &&
        (cmd_fifo_shadow_q[core_idx] != '0) &&
        cmd_fifo_ready_s[core_idx] &&
        ((iss_s1_count_q[core_idx] != 2'd2) || iss_s1_adv_s[core_idx])
      ) begin
        cmd_fifo_pop_s[core_idx] = 1'b1;
      end
    end

    // CMD_DEBUG static-table readback: while A_DEBUG_STATIC_READ is active the
    // system is idle (no NET issue in flight), so read port 0 is repurposed to
    // fetch the requested word. The default drivers above never assert the
    // port outside A_NET_STREAM, so there is no conflict.
    if (debug_static_mode_q && (state_q == A_DEBUG_STATIC_READ)) begin
      static_mem_rd_en_s[0] = 1'b1;
      static_mem_rd_addr_s[0] = STATIC_WEIGHT_WORD_ADDR_W'(debug_static_addr_q);
    end

    net_fifo_residue_s = (net_fifo_residue_detail_s != '0);
  end

  always_comb begin
    all_grad_task_ready_s = 1'b1;
    for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
      all_grad_task_ready_s &= grad_task_ready_i[core_idx];
    end
  end

  always_comb begin
    unique case (wait_selector_q)
      32'd0: wait_condition_s = (system_phase_i == 4'd1);
      32'd1: wait_condition_s = grad_p_finish_i && all_grad_task_ready_s;
      32'd2: wait_condition_s = !softmax_enable_i;
      32'd3: wait_condition_s = !bridge_ddr_active_i;
      default: wait_condition_s = 1'b0;
    endcase
  end

  // Blocked-GRADP v2: INIT_SEG segment base lookup. The meta word of
  // CMD_INIT_SEG carries {region_sel[2:0] @ [72:70], block_id[5:0] @
  // [69:64]}; the segment base row/line is read combinationally from the
  // blocked_ctrl descriptor file (same map as the descriptor writes).
  // region_sel==3 selects the NET/ctrl self-run image region: its lines
  // are image-relative (start_line = image line index, base 0 within the
  // NET_IMAGE region; the bridge adds NET_IMAGE_DRAM_BASE_ADDR).
  // v4: region 4 = gs bitstream region, region 5 = special bitstream
  // region (3-bit region_sel; decoding only [71:70] aliases 4->logits and
  // 5->rms and corrupts both).
  logic [10:0] seg_desc_addr_s;
  always_comb begin
    unique case (bram_data_q[72:70])
      3'd0: seg_desc_addr_s = 11'h440 + {5'b0, bram_data_q[69:64]};
      3'd1: seg_desc_addr_s = 11'h480 + {5'b0, bram_data_q[69:64]};
      3'd4: seg_desc_addr_s = 11'h580 + {5'b0, bram_data_q[69:64]};
      3'd5: seg_desc_addr_s = 11'h5C0 + {5'b0, bram_data_q[69:64]};
      default: seg_desc_addr_s = 11'h4C0 + {5'b0, bram_data_q[69:64]};
    endcase
  end
  assign blk_desc_rd_addr_o = seg_desc_addr_s;

  // Blocked-GRADP v2 pulses:
  //  - blk_block_done_o (+blk_iter_last_o): pulses when a self-run block
  //    play has observed the GRADP drain (self_run_q); blk_iter_last_o
  //    qualifies it as the iteration's last block (sr_play_last_q).
  //  - sr_done_seen_q: re-armed by CMD_GO, set by the master's sr_done_i;
  //    backs the self-run completion wait (selector 8).
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      blk_block_done_o <= 1'b0;
      blk_iter_last_o <= 1'b0;
      sr_done_seen_q <= 1'b0;
    end else begin
      if (sr_done_i) begin
        sr_done_seen_q <= 1'b1;
      end
      blk_block_done_o <=
        BLOCKED_SCHED && (state_q == A_WAIT_CMD) && wait_condition_s &&
        self_run_q;
      blk_iter_last_o <=
        BLOCKED_SCHED && (state_q == A_WAIT_CMD) && wait_condition_s &&
        self_run_q && sr_play_last_q;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state_q <= A_FLAG_ADDR;
      bram_data_q <= '0;
      usr_irq_ack_meta_q <= 1'b0;
      usr_irq_ack_sync_q <= 1'b0;
      cmd_bram_en_o <= 1'b0;
      fsm_bram_we_q <= '0;
      fsm_bram_addr_q <= '0;
      fsm_bram_wdata_q <= '0;
      usr_irq_req_o <= '0;
      grad_iteration_done_o <= 1'b0;
      readback_start_o <= 1'b0;
      readback_advance_o <= 1'b0;
      init_line_valid_o <= 1'b0;
      init_line_index_o <= '0;
      init_line_is_rms_o <= 1'b0;
      init_line_last_o <= 1'b0;
      init_line_data_o <= '0;
      group_flag_init_valid_o <= 1'b0;
      group_flag_init_addr_o <= '0;
      system_reset_hold_o <= 1'b1;
      busy_o <= 1'b0;
      error_o <= ERR_OK;
      debug_word_idx_q <= '0;
      debug_static_mode_q <= 1'b0;
      debug_static_addr_q <= '0;
      debug_trace_mode_q <= 1'b0;
      debug_trace_sel_q <= '0;
      phase_prev_q <= '0;
      for (int debug_word_idx = 0; debug_word_idx < 7; debug_word_idx++) begin
        debug_word_q[debug_word_idx] <= '0;
      end
      status_state_q <= ST_IDLE;
      last_cmd_q <= CMD_NOP;
      seq_id_q <= '0;
      error_code_q <= ERR_OK;
      processed_count_q <= '0;
      elapsed_cycles_q <= '0;
      detail_flags_q <= '0;
      event_counter_q <= '0;
      event_pending_q <= 1'b0;
      desc_magic_q <= '0;
      desc_version_q <= '0;
      cmd_type_q <= CMD_NOP;
      payload_start_q <= '0;
      payload_end_q <= '0;
      result_start_q <= '0;
      result_capacity_q <= '0;
      arg0_q <= '0;
      arg1_q <= '0;
      arg2_q <= '0;
      command_cycle_q <= '0;
      result_index_q <= '0;
      config_word_idx_q <= '0;
      config_apply_bank_len_q <= 1'b0;
      config_apply_wlvia_q <= 1'b0;
      config_wlvia_word_addr_q <= '0;
      config_wlvia_setup_idx_q <= '0;
      net_frame_idx_q <= '0;
      net_flags_q <= '0;
      full_reset_active_q <= 1'b0;
      self_run_q <= 1'b0;
      sr_play_last_q <= 1'b0;
      sr_last_frame_q <= 1'b0;
      sr_pipe_valid_q <= 1'b0;
      sr_pipe_last_q <= 1'b0;
      sr_done_seen_q <= 1'b0;
      sr_play_ack_o <= 1'b0;
      sr_frame_pop_q <= 1'b0;
      sr_go_o <= 1'b0;
      sr_go_iters_o <= '0;
      blk_go_o <= 1'b0;
      dbg_load_kick_o <= 1'b0;
      mirror_page_q <= '0;
      mirror_kick_q <= 1'b0;
      dbg_load_blk_o <= '0;
      dbg_load_half_o <= 1'b0;
      bdesc_addr_q <= '0;
      bdesc_count_q <= '0;
      bdesc_written_q <= '0;
      bdesc_word_idx_q <= '0;
      bdesc_lane_idx_q <= '0;
      bdesc_word_q <= '0;
      blk_wait_spill_q <= 1'b0;
      init_is_rms_q <= 1'b0;
      init_is_image_q <= 1'b0;
      init_line_is_image_o <= 1'b0;
      blk_desc_wr_valid_o <= 1'b0;
      blk_desc_wr_addr_o <= '0;
      blk_desc_wr_data_o <= '0;
      full_reset_count_q <= '0;
      wait_selector_q <= '0;
      wait_timeout_q <= '0;
      wait_elapsed_q <= '0;
      init_start_row_q <= '0;
      init_row_count_q <= '0;
      init_total_value_count_q <= '0;
      init_payload_word_idx_q <= '0;
      init_cur_row_q <= '0;
      init_word_q <= '0;
      group_flag_issue_count_q <= '0;
      group_flag_return_count_q <= '0;
      readback_row_offset_q <= '0;
      readback_expected_row_q <= '0;
      readback_slot_q <= '0;
      readback_word_q <= '0;
      static_weight_count_q <= '0;
      static_loaded_count_q <= '0;
      static_payload_word_idx_q <= '0;
      net_static_weight_bad_q <= 1'b0;
      for (int setup_idx = 0; setup_idx < 6; setup_idx++) begin
        config_wlvia_value_q[setup_idx] <= '0;
      end
      for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
        segment_idx_o[core_idx] <= banked_idx_zero();
        segment_l_o[core_idx] <= '0;
        segment_r_o[core_idx] <= '0;
        segment_w_o[core_idx] <= '0;
        segment_coeff_o[core_idx] <= '0;
        segment_type_o[core_idx] <= '0;
        sr_pipe_static_mask_q[core_idx] <= 1'b0;
        sr_pipe_static_lane_q[core_idx] <= '0;
        iss_s1_valid_q[core_idx] <= '0;
        iss_s1_pend_q[core_idx] <= '0;
        iss_s1_wr_ptr_q[core_idx] <= 1'b0;
        iss_s1_rd_ptr_q[core_idx] <= 1'b0;
        iss_s1_count_q[core_idx] <= '0;
        iss_s1_pop_d1_q[core_idx] <= 1'b0;
        iss_s1_pop_d1_ptr_q[core_idx] <= 1'b0;
        iss_s2_valid_q[core_idx] <= 1'b0;
        iss_s2_cmd_q[core_idx] <= '0;
        iss_s2_need_static_q[core_idx] <= 1'b0;
        iss_s3_valid_q[core_idx] <= 1'b0;
        iss_s3_cmd_q[core_idx] <= '0;
        iss_s3_static_w_q[core_idx] <= '0;
        iss_s3_need_static_q[core_idx] <= 1'b0;
        emit_count_q[core_idx] <= '0;
        task_ready_rise_count_q[core_idx] <= '0;
        grad_task_ready_prev_q[core_idx] <= 1'b0;
      end
      for (int group_idx = 0; group_idx < GRAD_OVERFLOW_COUNT; group_idx++) begin
        for (int lane_idx = 0; lane_idx < CORE_COUNT_PER_FIFO; lane_idx++) begin
          mode_overflow_o[group_idx][lane_idx] <= '0;
          mode_overflow_valid_o[group_idx][lane_idx] <= 1'b0;
        end
      end
      for (int group_idx = 0; group_idx < GRAD_WLVIA_COUNT; group_idx++) begin
        mode_wlvia_o[group_idx] <= '0;
      end
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        bank_valid_len_o[bank_idx] <= '0;
        group_flag_init_data_o[bank_idx] <= '0;
      end
    end else begin
      // Saturated counters for DEBUG snapshots, reset on GRADP phase entry
      if (system_phase_i == 4'd2 && phase_prev_q != 4'd2) begin
        for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
          emit_count_q[core_idx] <= '0;
          task_ready_rise_count_q[core_idx] <= '0;
          grad_task_ready_prev_q[core_idx] <= 1'b0;
        end
      end else begin
        for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
          if (iss_s3_emit_s[core_idx]) begin
            if (emit_count_q[core_idx] != 16'hFFFF) begin
              emit_count_q[core_idx] <= emit_count_q[core_idx] + 1'b1;
            end
          end
          if (grad_task_ready_i[core_idx] && !grad_task_ready_prev_q[core_idx]) begin
            if (task_ready_rise_count_q[core_idx] != 16'hFFFF) begin
              task_ready_rise_count_q[core_idx] <= task_ready_rise_count_q[core_idx] + 1'b1;
            end
          end
          grad_task_ready_prev_q[core_idx] <= grad_task_ready_i[core_idx];
        end
      end
      phase_prev_q <= system_phase_i;

      usr_irq_ack_meta_q <= usr_irq_ack_i[0];
      usr_irq_ack_sync_q <= usr_irq_ack_meta_q;
      bram_data_q <= cmd_bram_rdata_i;
      cmd_bram_en_o <= enable_i;
      fsm_bram_we_q <= '0;
      fsm_bram_wdata_q <= '0;
      group_flag_init_valid_o <= 1'b0;
      readback_start_o <= 1'b0;
      readback_advance_o <= 1'b0;
      // one-cycle pulses: GO launch / engine phase kick / fetcher frame pop
      sr_go_o <= 1'b0;
      blk_go_o <= 1'b0;
      dbg_load_kick_o <= 1'b0;
      mirror_kick_q <= 1'b0;
      sr_frame_pop_q <= 1'b0;
      for (int group_idx = 0; group_idx < GRAD_OVERFLOW_COUNT; group_idx++) begin
        for (int lane_idx = 0; lane_idx < CORE_COUNT_PER_FIFO; lane_idx++) begin
          mode_overflow_o[group_idx][lane_idx] <= '0;
          mode_overflow_valid_o[group_idx][lane_idx] <= 1'b0;
        end
      end
      for (int group_idx = 0; group_idx < GRAD_WLVIA_COUNT; group_idx++) begin
        mode_wlvia_o[group_idx] <= '0;
      end
      for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
        segment_w_o[core_idx] <= '0;
        if (iss_s3_emit_s[core_idx]) begin
          int overflow_group_idx;
          int overflow_lane_idx;
          overflow_group_idx = core_idx / CORE_COUNT_PER_FIFO;
          overflow_lane_idx = core_idx % CORE_COUNT_PER_FIFO;
          // Only native query segments (SEGMENT with nonzero coeff) carry a
          // blocked window slot in idx23; SETW / static-weight-index commands
          // must keep the plain (bank, mem) decode for the static table.
          segment_idx_o[core_idx] <= decode_banked_idx23(
            iss_s3_cmd_q[core_idx][23:1],
            (iss_s3_cmd_q[core_idx][60:59] == OVERFLOW_MODE_SEGMENT) &&
              (iss_s3_cmd_q[core_idx][55:40] != 16'd0)
          );
          segment_l_o[core_idx] <= decode_addr8(iss_s3_cmd_q[core_idx][31:24]);
          segment_r_o[core_idx] <= decode_addr8(iss_s3_cmd_q[core_idx][39:32]);
          segment_w_o[core_idx] <=
            iss_s3_need_static_q[core_idx] ? iss_s3_static_w_q[core_idx] : '0;
          segment_coeff_o[core_idx] <= decode_coeff16(iss_s3_cmd_q[core_idx][55:40]);
          segment_type_o[core_idx] <= iss_s3_cmd_q[core_idx][58:56];
          mode_overflow_o[overflow_group_idx][overflow_lane_idx] <= iss_s3_cmd_q[core_idx][60:59];
          mode_overflow_valid_o[overflow_group_idx][overflow_lane_idx] <= 1'b1;
          if (core_idx < GRAD_WLVIA_COUNT) begin
            mode_wlvia_o[core_idx] <= iss_s3_cmd_q[core_idx][63:61];
          end
        end
      end

      // Issue pipeline registers: one command per stage, all types walk the
      // same three stages (S1 pop+capture -> S2 head/static-pop -> S3 emit).
      for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
        if (!enable_i) begin
          iss_s1_valid_q[core_idx] <= '0;
          iss_s1_pend_q[core_idx] <= '0;
          iss_s1_wr_ptr_q[core_idx] <= 1'b0;
          iss_s1_rd_ptr_q[core_idx] <= 1'b0;
          iss_s1_count_q[core_idx] <= '0;
          iss_s1_pop_d1_q[core_idx] <= 1'b0;
          iss_s1_pop_d1_ptr_q[core_idx] <= 1'b0;
          iss_s2_valid_q[core_idx] <= 1'b0;
          iss_s2_cmd_q[core_idx] <= '0;
          iss_s2_need_static_q[core_idx] <= 1'b0;
          iss_s3_valid_q[core_idx] <= 1'b0;
          iss_s3_cmd_q[core_idx] <= '0;
          iss_s3_static_w_q[core_idx] <= '0;
          iss_s3_need_static_q[core_idx] <= 1'b0;
        end else begin
          // S1 queue: a pop claims a slot (pend); exactly one cycle later
          // the popped word is captured into that same slot (HAND_WRITTEN_STD
          // guarantees pop(T) -> dout+valid(T+1)). valid and data advance as
          // one pair from then on.
          if (cmd_fifo_pop_s[core_idx]) begin
            iss_s1_pend_q[core_idx][iss_s1_wr_ptr_q[core_idx]] <= 1'b1;
            iss_s1_wr_ptr_q[core_idx] <= !iss_s1_wr_ptr_q[core_idx];
          end
          iss_s1_pop_d1_q[core_idx] <= cmd_fifo_pop_s[core_idx];
          iss_s1_pop_d1_ptr_q[core_idx] <= iss_s1_wr_ptr_q[core_idx];
          if (
            iss_s1_pop_d1_q[core_idx] && cmd_fifo_output_valid_s[core_idx]
          ) begin
            iss_s1_data_q[core_idx][iss_s1_pop_d1_ptr_q[core_idx]] <=
              cmd_fifo_head_s[core_idx];
            iss_s1_pend_q[core_idx][iss_s1_pop_d1_ptr_q[core_idx]] <= 1'b0;
            iss_s1_valid_q[core_idx][iss_s1_pop_d1_ptr_q[core_idx]] <= 1'b1;
          end

          if (iss_s1_adv_s[core_idx]) begin
            iss_s1_valid_q[core_idx][iss_s1_rd_ptr_q[core_idx]] <= 1'b0;
            iss_s1_rd_ptr_q[core_idx] <= !iss_s1_rd_ptr_q[core_idx];
          end

          unique case ({cmd_fifo_pop_s[core_idx], iss_s1_adv_s[core_idx]})
            2'b10: iss_s1_count_q[core_idx] <= iss_s1_count_q[core_idx] + 1'b1;
            2'b01: iss_s1_count_q[core_idx] <= iss_s1_count_q[core_idx] - 1'b1;
            default: iss_s1_count_q[core_idx] <= iss_s1_count_q[core_idx];
          endcase

          if (iss_s2_adv_s[core_idx] && !iss_s1_adv_s[core_idx]) begin
            iss_s2_valid_q[core_idx] <= 1'b0;
          end else if (iss_s1_adv_s[core_idx]) begin
            iss_s2_valid_q[core_idx] <= 1'b1;
            iss_s2_cmd_q[core_idx] <=
              iss_s1_data_q[core_idx][iss_s1_rd_ptr_q[core_idx]];
            iss_s2_need_static_q[core_idx] <=
              lane_needs_static_weight(iss_s1_data_q[core_idx][iss_s1_rd_ptr_q[core_idx]]);
          end

          if (iss_s3_emit_s[core_idx] && !iss_s2_adv_s[core_idx]) begin
            iss_s3_valid_q[core_idx] <= 1'b0;
          end else if (iss_s2_adv_s[core_idx]) begin
            iss_s3_valid_q[core_idx] <= 1'b1;
            iss_s3_cmd_q[core_idx] <= iss_s2_cmd_q[core_idx];
            iss_s3_need_static_q[core_idx] <= iss_s2_need_static_q[core_idx];
            // the sw FIFO was popped when this command entered S2 (previous
            // cycle), so the readable sw head now holds its weight
            iss_s3_static_w_q[core_idx] <=
              iss_s2_need_static_q[core_idx] ? static_w_fifo_head_s[core_idx] : '0;
          end
        end
      end

      if (system_event_i) begin
        event_counter_q <= event_counter_q + 1'b1;
        event_pending_q <= 1'b1;
      end
      if (system_phase_i != 4'd2) begin
        grad_iteration_done_o <= 1'b0;
      end
      if (
        (system_phase_i != 4'd1) &&
        (system_phase_i != 4'd5) &&
        (system_phase_i != 4'd6)
      ) begin
        readback_expected_row_q <= '0;
      end

      if (busy_o) begin
        command_cycle_q <= command_cycle_q + 1'b1;
      end

      if (!enable_i) begin
        state_q <= A_FLAG_ADDR;
        busy_o <= 1'b0;
        full_reset_active_q <= 1'b0;
        full_reset_count_q <= '0;
        usr_irq_req_o <= '0;
        grad_iteration_done_o <= 1'b0;
        init_line_valid_o <= 1'b0;
        group_flag_init_valid_o <= 1'b0;
        readback_start_o <= 1'b0;
        readback_advance_o <= 1'b0;
        system_reset_hold_o <= 1'b0;
        fsm_bram_addr_q <= '0;
        fsm_bram_we_q <= '0;
        status_state_q <= ST_IDLE;
        for (int group_idx = 0; group_idx < GRAD_OVERFLOW_COUNT; group_idx++) begin
          for (int lane_idx = 0; lane_idx < CORE_COUNT_PER_FIFO; lane_idx++) begin
            mode_overflow_o[group_idx][lane_idx] <= '0;
            mode_overflow_valid_o[group_idx][lane_idx] <= 1'b0;
          end
        end
        for (int group_idx = 0; group_idx < GRAD_WLVIA_COUNT; group_idx++) begin
          mode_wlvia_o[group_idx] <= '0;
        end
      end else begin
        unique case (state_q)
          A_FLAG_ADDR: begin
            busy_o <= 1'b0;
            status_state_q <= ST_IDLE;
            // self-run gate: the submit poll is disabled while a self-run
            // (plus the mirror tail) owns the adapter; a command submitted
            // during the gate waits at SUBMIT_INDEX until the gate lifts
            if (!selfrun_gate_s) begin
              fsm_bram_addr_q <= bram_addr(SUBMIT_INDEX);
            end
            for (int group_idx = 0; group_idx < GRAD_OVERFLOW_COUNT; group_idx++) begin
              for (int lane_idx = 0; lane_idx < CORE_COUNT_PER_FIFO; lane_idx++) begin
                mode_overflow_o[group_idx][lane_idx] <= '0;
                mode_overflow_valid_o[group_idx][lane_idx] <= 1'b0;
              end
            end
            for (int group_idx = 0; group_idx < GRAD_WLVIA_COUNT; group_idx++) begin
              mode_wlvia_o[group_idx] <= '0;
            end
            state_q <= A_FLAG_WAIT0;
          end

          A_FLAG_WAIT0: begin
            state_q <= A_FLAG_WAIT1;
          end

          A_FLAG_WAIT1: begin
            state_q <= A_FLAG_CHECK;
          end

          A_FLAG_CHECK: begin
            if (!selfrun_gate_s) begin
              fsm_bram_addr_q <= bram_addr(SUBMIT_INDEX);
            end
            if (!selfrun_gate_s && (bram_data_q[31:0] == SUBMIT_MAGIC)) begin
              busy_o <= 1'b1;
              command_cycle_q <= '0;
              elapsed_cycles_q <= '0;
              processed_count_q <= '0;
              detail_flags_q <= '0;
              error_code_q <= ERR_OK;
              fsm_bram_wdata_q <= '0;
              fsm_bram_we_q <= {BRAM_STRB_W{1'b1}};
              state_q <= A_CLEAR_FLAG;
            end else if (!selfrun_gate_s && event_pending_q && !usr_irq_req_o[0]) begin
              status_state_q <= ST_EVENT;
              last_cmd_q <= CMD_WAIT;
              error_code_q <= ERR_OK;
              processed_count_q <= event_counter_q;
              elapsed_cycles_q <= '0;
              detail_flags_q <= {28'd0, system_phase_i};
              state_q <= A_WRITE_EVENT;
            end else if (BLOCKED_SCHED && sr_play_i) begin
              // NET-on-DDR self-run block play: stream this block's NET
              // frames from the net_fetcher through the normal lane
              // decode/issue path, then observe the GRADP drain and pulse
              // blk_block_done_o for the iteration master
              self_run_q <= 1'b1;
              sr_play_last_q <= sr_play_last_i;
              sr_play_ack_o <= 1'b1;
              net_frame_idx_q <= '0;
              state_q <= A_NET_LANE_ADDR;
            end else begin
              state_q <= A_FLAG_WAIT0;
            end
          end

          A_CLEAR_FLAG: begin
            fsm_bram_addr_q <= bram_addr(0);
            state_q <= A_DESC0_ADDR;
          end

          A_DESC0_ADDR: begin
            fsm_bram_addr_q <= bram_addr(0);
            state_q <= A_DESC0_WAIT0;
          end

          A_DESC0_WAIT0: begin
            state_q <= A_DESC0_WAIT1;
          end

          A_DESC0_WAIT1: begin
            state_q <= A_DESC0_CAPTURE;
          end

          A_DESC0_CAPTURE: begin
            desc_magic_q <= bram_data_q[15:0];
            desc_version_q <= bram_data_q[23:16];
            cmd_type_q <= bram_data_q[31:24];
            seq_id_q <= bram_data_q[47:40];
            payload_start_q <= bram_data_q[95:64];
            payload_end_q <= bram_data_q[127:96];
            last_cmd_q <= bram_data_q[31:24];
            fsm_bram_addr_q <= bram_addr(1);
            state_q <= A_DESC1_ADDR;
          end

          A_DESC1_ADDR: begin
            fsm_bram_addr_q <= bram_addr(1);
            state_q <= A_DESC1_WAIT0;
          end

          A_DESC1_WAIT0: begin
            state_q <= A_DESC1_WAIT1;
          end

          A_DESC1_WAIT1: begin
            state_q <= A_DESC1_CAPTURE;
          end

          A_DESC1_CAPTURE: begin
            result_start_q <= bram_data_q[31:0];
            result_capacity_q <= bram_data_q[63:32];
            arg0_q <= bram_data_q[95:64];
            arg1_q <= bram_data_q[127:96];
            fsm_bram_addr_q <= bram_addr(2);
            state_q <= A_DESC2_ADDR;
          end

          A_DESC2_ADDR: begin
            fsm_bram_addr_q <= bram_addr(2);
            state_q <= A_DESC2_WAIT0;
          end

          A_DESC2_WAIT0: begin
            state_q <= A_DESC2_WAIT1;
          end

          A_DESC2_WAIT1: begin
            state_q <= A_DESC2_CAPTURE;
          end

          A_DESC2_CAPTURE: begin
            arg2_q <= bram_data_q[31:0];
            state_q <= A_VALIDATE;
          end

          A_VALIDATE: begin
            error_code_q <= descriptor_error();
            error_o <= descriptor_error();
            if (descriptor_error() == ERR_OK) begin
              status_state_q <= ST_BUSY;
              state_q <= A_WRITE_BUSY;
            end else begin
              status_state_q <= ST_ERROR;
              state_q <= A_WRITE_FINAL;
            end
          end

          A_WRITE_BUSY: begin
            fsm_bram_addr_q <= bram_addr(STATUS_INDEX);
            fsm_bram_wdata_q <= pack_status(ST_BUSY, cmd_type_q, seq_id_q, ERR_OK, '0, command_cycle_q, '0);
            fsm_bram_we_q <= {BRAM_STRB_W{1'b1}};
            state_q <= A_DISPATCH;
          end

          A_DISPATCH: begin
            unique case (cmd_type_q)
              CMD_NOP: begin
                processed_count_q <= '0;
                error_code_q <= ERR_OK;
                status_state_q <= ST_DONE;
                state_q <= A_WRITE_FINAL;
              end
              CMD_GET_CAPS: begin
                result_index_q <= '0;
                state_q <= A_CAPS_WRITE;
              end
              CMD_CONFIG: begin
                fsm_bram_addr_q <= bram_addr(payload_start_q);
                state_q <= A_CONFIG_META_ADDR;
              end
              CMD_WAIT: begin
                wait_selector_q <= arg0_q;
                wait_timeout_q <= arg1_q;
                wait_elapsed_q <= '0;
                state_q <= A_WAIT_CMD;
              end
              CMD_DEBUG: begin
                result_index_q <= '0;
                debug_word_idx_q <= '0;
                // arg0[31] selects the static weight table readback mode; in
                // that mode arg0[7:0] is the 512-bit word address read back
                // through debug W2 (default debug snapshot is bit-identical
                // when arg0[31] is clear).
                // arg0[30] (with arg0[31] clear) selects the overflow trace
                // readback mode: arg0[7:0] is the trace field selector,
                // returned in W2[31:0] with the selector echoed in W3[31:0]
                // (pure comb mux through the normal A_DEBUG_WRITE path).
                debug_static_mode_q <= arg0_q[31];
                debug_static_addr_q <= arg0_q[7:0];
                debug_trace_mode_q <= arg0_q[30] && !arg0_q[31];
                debug_trace_sel_q <= arg0_q[7:0];
                if (arg0_q[31]) begin
                  state_q <= A_DEBUG_STATIC_READ;
                end else begin
                  state_q <= A_DEBUG_WRITE;
                end
              end
              CMD_RESET: begin
                for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
                  bank_valid_len_o[bank_idx] <= '0;
                end
                for (int setup_idx = 0; setup_idx < 6; setup_idx++) begin
                  config_wlvia_value_q[setup_idx] <= '0;
                end
                for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
                  segment_idx_o[core_idx] <= banked_idx_zero();
                  segment_l_o[core_idx] <= '0;
                  segment_r_o[core_idx] <= '0;
                  segment_w_o[core_idx] <= '0;
                  segment_coeff_o[core_idx] <= '0;
                  segment_type_o[core_idx] <= '0;
                  iss_s1_valid_q[core_idx] <= '0;
                  iss_s1_pend_q[core_idx] <= '0;
                  iss_s1_wr_ptr_q[core_idx] <= 1'b0;
                  iss_s1_rd_ptr_q[core_idx] <= 1'b0;
                  iss_s1_count_q[core_idx] <= '0;
                  iss_s1_pop_d1_q[core_idx] <= 1'b0;
                  iss_s1_pop_d1_ptr_q[core_idx] <= 1'b0;
                  iss_s2_valid_q[core_idx] <= 1'b0;
                  iss_s2_cmd_q[core_idx] <= '0;
                  iss_s2_need_static_q[core_idx] <= 1'b0;
                  iss_s3_valid_q[core_idx] <= 1'b0;
                  iss_s3_cmd_q[core_idx] <= '0;
                  iss_s3_static_w_q[core_idx] <= '0;
                  iss_s3_need_static_q[core_idx] <= 1'b0;
                end
                for (int group_idx = 0; group_idx < GRAD_OVERFLOW_COUNT; group_idx++) begin
                  for (int lane_idx = 0; lane_idx < CORE_COUNT_PER_FIFO; lane_idx++) begin
                    mode_overflow_o[group_idx][lane_idx] <= '0;
                    mode_overflow_valid_o[group_idx][lane_idx] <= 1'b0;
                  end
                end
                for (int group_idx = 0; group_idx < GRAD_WLVIA_COUNT; group_idx++) begin
                  mode_wlvia_o[group_idx] <= '0;
                end
                system_reset_hold_o <= 1'b1;
                full_reset_active_q <= 1'b1;
                full_reset_count_q <= '0;
                grad_iteration_done_o <= 1'b0;
                event_counter_q <= '0;
                event_pending_q <= 1'b0;
                readback_start_o <= 1'b0;
                readback_advance_o <= 1'b0;
                init_line_valid_o <= 1'b0;
                group_flag_init_valid_o <= 1'b0;
                static_weight_count_q <= '0;
                static_loaded_count_q <= '0;
                static_payload_word_idx_q <= '0;
                readback_row_offset_q <= '0;
                readback_expected_row_q <= '0;
                readback_slot_q <= '0;
                readback_word_q <= '0;
                config_word_idx_q <= '0;
                config_apply_bank_len_q <= 1'b0;
                config_apply_wlvia_q <= 1'b0;
                net_frame_idx_q <= '0;
                net_flags_q <= '0;
                self_run_q <= 1'b0;
                sr_play_last_q <= 1'b0;
                sr_last_frame_q <= 1'b0;
                sr_play_ack_o <= 1'b0;
                sr_frame_pop_q <= 1'b0;
                bdesc_written_q <= '0;
                bdesc_word_idx_q <= '0;
                bdesc_lane_idx_q <= '0;
                blk_wait_spill_q <= 1'b0;
                init_is_rms_q <= 1'b0;
                init_is_image_q <= 1'b0;
                blk_desc_wr_valid_o <= 1'b0;
                wait_selector_q <= '0;
                wait_timeout_q <= '0;
                wait_elapsed_q <= '0;
                init_start_row_q <= '0;
                init_row_count_q <= '0;
                init_total_value_count_q <= '0;
                init_payload_word_idx_q <= '0;
                init_cur_row_q <= '0;
                init_word_q <= '0;
                group_flag_issue_count_q <= '0;
                group_flag_return_count_q <= '0;
                result_index_q <= '0;
                debug_word_idx_q <= '0;
                debug_static_mode_q <= 1'b0;
                debug_static_addr_q <= '0;
                debug_trace_mode_q <= 1'b0;
                debug_trace_sel_q <= '0;
                state_q <= A_RESET_WAIT;
              end
              CMD_INIT_SEG: begin
                fsm_bram_addr_q <= bram_addr(payload_start_q);
                state_q <= A_INIT_META_ADDR;
              end
              CMD_GO: begin
                // NET-on-DDR self-run launch: N iterations (arg0) to the
                // master, one go pulse to master + net_fetcher, and a phase
                // kick so the engine leaves WAIT_GRADP for GRADP. The run
                // itself is asynchronous; this command completes at once.
                sr_go_o <= 1'b1;
                sr_go_iters_o <= arg0_q;
                blk_go_o <= 1'b1;
                sr_done_seen_q <= 1'b0;
                processed_count_q <= '0;
                error_code_q <= ERR_OK;
                status_state_q <= ST_DONE;
                state_q <= A_WRITE_FINAL;
              end
              CMD_DBG_LOAD_BLOCK: begin
                // Host debug window load (READ_P v4): one kick pulse to
                // blocked_ctrl with {arg0 = block ordinal, arg1[0] = half};
                // the load itself is asynchronous -- the host polls the
                // debug word-5 ld_done_valid/ld_blk fields for completion.
                dbg_load_kick_o <= 1'b1;
                dbg_load_blk_o <= arg0_q[5:0];
                dbg_load_half_o <= arg1_q[0];
                // A dbg load starts a new readback session: re-arm the
                // sequential-row cursor at the half base (host arg0_base =
                // half<<14), otherwise the next block's first READ_P chunk
                // fails the arg0 == readback_expected_row_q check.
                readback_expected_row_q <= 32'(arg1_q[0]) << 14;
                processed_count_q <= '0;
                error_code_q <= ERR_OK;
                status_state_q <= ST_DONE;
                state_q <= A_WRITE_FINAL;
              end
              CMD_SET_MIRROR_PAGE: begin
                // Mirror page select (arg0[1:0]) + one-frame kick so the new
                // page is streamed even after the self-run has retired (the
                // mirror only streams while sr_running/tail is active).
                mirror_page_q <= arg0_q[1:0];
                mirror_kick_q <= 1'b1;
                processed_count_q <= '0;
                error_code_q <= ERR_OK;
                status_state_q <= ST_DONE;
                state_q <= A_WRITE_FINAL;
              end
              CMD_LOAD_BLOCK_DESC: begin
                fsm_bram_addr_q <= bram_addr(payload_start_q);
                state_q <= A_BDESC_META_ADDR;
              end
              CMD_INIT_STATIC_WEIGHTS: begin
                fsm_bram_addr_q <= bram_addr(payload_start_q);
                state_q <= A_STATIC_META_ADDR;
              end
              CMD_READ_P: begin
                readback_start_o <= 1'b1;
                readback_row_offset_q <= '0;
                readback_slot_q <= '0;
                readback_word_q <= '0;
                result_index_q <= '0;
                state_q <= A_READBACK_REQ;
              end
              default: begin
                error_code_q <= ERR_UNKNOWN_CMD;
                status_state_q <= ST_ERROR;
                state_q <= A_WRITE_FINAL;
              end
            endcase
          end

          A_CAPS_WRITE: begin
            fsm_bram_addr_q <= bram_addr(result_start_q + result_index_q);
            fsm_bram_wdata_q <= pack_caps_word(result_index_q);
            fsm_bram_we_q <= {BRAM_STRB_W{1'b1}};
            if (result_index_q == 32'd4) begin
              processed_count_q <= 32'd5;
              error_code_q <= ERR_OK;
              status_state_q <= ST_DONE;
              state_q <= A_WRITE_FINAL;
            end else begin
              result_index_q <= result_index_q + 1'b1;
            end
          end

          A_CONFIG_META_ADDR: begin
            fsm_bram_addr_q <= bram_addr(payload_start_q);
            state_q <= A_CONFIG_META_WAIT0;
          end

          A_CONFIG_META_WAIT0: begin
            state_q <= A_CONFIG_META_WAIT1;
          end

          A_CONFIG_META_WAIT1: begin
            state_q <= A_CONFIG_META_CAPTURE;
          end

          A_CONFIG_META_CAPTURE: begin
            logic [31:0] required_words;
            detail_flags_q <= bram_data_q[95:64];
            config_apply_bank_len_q <= bram_data_q[64];
            config_apply_wlvia_q <= bram_data_q[69];
            config_wlvia_word_addr_q <= payload_start_q + 32'd1 + (bram_data_q[64] ? 32'(CONFIG_LEN_WORD_COUNT) : 32'd0);
            required_words = 32'd1 +
              (bram_data_q[64] ? 32'(CONFIG_LEN_WORD_COUNT) : 32'd0) +
              (bram_data_q[69] ? 32'd1 : 32'd0);
            if ((payload_end_q - payload_start_q) < required_words) begin
              error_code_q <= ERR_BAD_LENGTH;
              status_state_q <= ST_ERROR;
              state_q <= A_WRITE_FINAL;
            end else if (bram_data_q[64]) begin
              config_word_idx_q <= '0;
              fsm_bram_addr_q <= bram_addr(payload_start_q + 1);
              state_q <= A_CONFIG_LEN_ADDR;
            end else if (bram_data_q[69]) begin
              if (bram_data_q[65]) begin
                system_reset_hold_o <= 1'b1;
              end
              if (bram_data_q[66]) begin
                system_reset_hold_o <= 1'b0;
              end
              fsm_bram_addr_q <= bram_addr(payload_start_q + 1);
              state_q <= A_CONFIG_WLVIA_ADDR;
            end else begin
              if (bram_data_q[65]) begin
                system_reset_hold_o <= 1'b1;
              end
              if (bram_data_q[66]) begin
                system_reset_hold_o <= 1'b0;
              end
              processed_count_q <= '0;
              error_code_q <= ERR_OK;
              status_state_q <= ST_DONE;
              state_q <= A_WRITE_FINAL;
            end
          end

          A_CONFIG_LEN_ADDR: begin
            fsm_bram_addr_q <= bram_addr(payload_start_q + 1 + config_word_idx_q);
            state_q <= A_CONFIG_LEN_WAIT0;
          end

          A_CONFIG_LEN_WAIT0: begin
            state_q <= A_CONFIG_LEN_WAIT1;
          end

          A_CONFIG_LEN_WAIT1: begin
            state_q <= A_CONFIG_LEN_CAPTURE;
          end

          A_CONFIG_LEN_CAPTURE: begin
            logic len_bad;
            logic [31:0] len_limit;
            len_bad = 1'b0;
            // Blocked-GRADP v2: the v2 host configures bank_valid_len to the
            // grand-total unified row space (logits + p + gs/special, up to
            // 2^20-1 rows) so the bridge resp mask passes every region row
            // beyond BANK_DEPTH. The register field is 20 bits wide
            // (BANK_VALID_LEN_W) and holds those values; non-blocked keeps
            // the BANK_DEPTH limit.
            len_limit = BLOCKED_SCHED ? 32'd1048575 : 32'(BANK_DEPTH);
            for (int lane_idx = 0; lane_idx < 16; lane_idx++) begin
              int bank_idx;
              logic [31:0] len_value;
              bank_idx = int'(config_word_idx_q) * 16 + lane_idx;
              len_value = bram_data_q[(lane_idx * 32) +: 32];
              if (bank_idx < BANK_COUNT) begin
                if (len_value > len_limit) begin
                  len_bad = 1'b1;
                end else begin
                  bank_valid_len_o[bank_idx] <= decode_bank_len32(len_value);
                end
              end
            end
            if (len_bad) begin
              error_code_q <= ERR_RANGE_ERROR;
              status_state_q <= ST_ERROR;
              state_q <= A_WRITE_FINAL;
            end else if (config_word_idx_q + 1'b1 >= CONFIG_LEN_WORD_COUNT) begin
              if (detail_flags_q[1]) begin
                system_reset_hold_o <= 1'b1;
              end
              if (detail_flags_q[2]) begin
                system_reset_hold_o <= 1'b0;
              end
              if (config_apply_wlvia_q) begin
                fsm_bram_addr_q <= bram_addr(config_wlvia_word_addr_q);
                state_q <= A_CONFIG_WLVIA_ADDR;
              end else begin
                processed_count_q <= 32'(BANK_COUNT);
                error_code_q <= ERR_OK;
                status_state_q <= ST_DONE;
                state_q <= A_WRITE_FINAL;
              end
            end else begin
              config_word_idx_q <= config_word_idx_q + 1'b1;
              fsm_bram_addr_q <= bram_addr(payload_start_q + 1 + config_word_idx_q + 1'b1);
              state_q <= A_CONFIG_LEN_ADDR;
            end
          end

          A_CONFIG_WLVIA_ADDR: begin
            fsm_bram_addr_q <= bram_addr(config_wlvia_word_addr_q);
            state_q <= A_CONFIG_WLVIA_WAIT0;
          end

          A_CONFIG_WLVIA_WAIT0: begin
            state_q <= A_CONFIG_WLVIA_WAIT1;
          end

          A_CONFIG_WLVIA_WAIT1: begin
            state_q <= A_CONFIG_WLVIA_CAPTURE;
          end

          A_CONFIG_WLVIA_CAPTURE: begin
            for (int setup_idx = 0; setup_idx < 6; setup_idx++) begin
              logic signed [31:0] setup_value;
              setup_value = bram_data_q[(setup_idx * 32) +: 32];
              config_wlvia_value_q[setup_idx] <= DATA_W'($signed(setup_value));
            end
            config_wlvia_setup_idx_q <= '0;
            state_q <= A_CONFIG_WLVIA_ARM;
          end

          A_CONFIG_WLVIA_ARM: begin
            for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
              segment_idx_o[core_idx] <= banked_idx_zero();
              segment_l_o[core_idx] <= '0;
              segment_r_o[core_idx] <= '0;
              segment_w_o[core_idx] <= '0;
              segment_coeff_o[core_idx] <= '0;
              segment_type_o[core_idx] <= '0;
            end
            for (int group_idx = 0; group_idx < GRAD_OVERFLOW_COUNT; group_idx++) begin
              for (int lane_idx = 0; lane_idx < CORE_COUNT_PER_FIFO; lane_idx++) begin
                mode_overflow_o[group_idx][lane_idx] <= '0;
                mode_overflow_valid_o[group_idx][lane_idx] <= 1'b0;
              end
            end
            for (int group_idx = 0; group_idx < GRAD_WLVIA_COUNT; group_idx++) begin
              mode_wlvia_o[group_idx] <= WLVIA_MODE_IDLE;
            end
            unique case (config_wlvia_setup_idx_q)
              3'd0: begin
                for (int group_idx = 0; group_idx < GRAD_WLVIA_COUNT; group_idx++) begin
                  mode_wlvia_o[group_idx] <= WLVIA_MODE_SET_LAMBDA_WL;
                  segment_w_o[group_idx * SEGMENT_PER_FIFO] <= config_wlvia_value_q[0];
                end
              end
              3'd1: begin
                for (int group_idx = 0; group_idx < GRAD_WLVIA_COUNT; group_idx++) begin
                  mode_wlvia_o[group_idx] <= WLVIA_MODE_SET_LAMBDA_VIA;
                  segment_w_o[group_idx * SEGMENT_PER_FIFO] <= config_wlvia_value_q[1];
                end
              end
              3'd2: begin
                for (int group_idx = 0; group_idx < GRAD_WLVIA_COUNT; group_idx++) begin
                  mode_wlvia_o[group_idx] <= WLVIA_MODE_SET_HOR_EDGE_LEN_NORMAL;
                  segment_w_o[group_idx * SEGMENT_PER_FIFO] <= config_wlvia_value_q[2];
                end
              end
              3'd3: begin
                for (int group_idx = 0; group_idx < GRAD_WLVIA_COUNT; group_idx++) begin
                  mode_wlvia_o[group_idx] <= WLVIA_MODE_SET_VER_EDGE_LEN_NORMAL;
                  segment_w_o[group_idx * SEGMENT_PER_FIFO] <= config_wlvia_value_q[3];
                end
              end
              3'd4: begin
                for (int group_idx = 0; group_idx < GRAD_WLVIA_COUNT; group_idx++) begin
                  mode_wlvia_o[group_idx] <= WLVIA_MODE_SET_HOR_EDGE_LEN_LAST;
                  segment_w_o[group_idx * SEGMENT_PER_FIFO] <= config_wlvia_value_q[4];
                end
              end
              default: begin
                for (int group_idx = 0; group_idx < GRAD_WLVIA_COUNT; group_idx++) begin
                  mode_wlvia_o[group_idx] <= WLVIA_MODE_SET_VER_EDGE_LEN_LAST;
                  segment_w_o[group_idx * SEGMENT_PER_FIFO] <= config_wlvia_value_q[5];
                end
              end
            endcase
            state_q <= A_CONFIG_WLVIA_ISSUE;
          end

          A_CONFIG_WLVIA_ISSUE: begin
            for (int group_idx = 0; group_idx < GRAD_WLVIA_COUNT; group_idx++) begin
              mode_wlvia_o[group_idx] <= WLVIA_MODE_IDLE;
            end
            for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
              segment_w_o[core_idx] <= '0;
            end
            if (config_wlvia_setup_idx_q >= 3'd5) begin
              processed_count_q <= (config_apply_bank_len_q ? 32'(BANK_COUNT) : 32'd0) + 32'd6;
              error_code_q <= ERR_OK;
              status_state_q <= ST_DONE;
              state_q <= A_WRITE_FINAL;
            end else begin
              config_wlvia_setup_idx_q <= config_wlvia_setup_idx_q + 1'b1;
              state_q <= A_CONFIG_WLVIA_ARM;
            end
          end

          A_INIT_META_ADDR: begin
            fsm_bram_addr_q <= bram_addr(payload_start_q);
            state_q <= A_INIT_META_WAIT0;
          end

          A_INIT_META_WAIT0: begin
            state_q <= A_INIT_META_WAIT1;
          end

          A_INIT_META_WAIT1: begin
            state_q <= A_INIT_META_CAPTURE;
          end

          A_INIT_META_CAPTURE: begin
            logic [31:0] start_line_value;
            logic [31:0] line_count_value;
            logic [31:0] max_line_count;
            logic [31:0] seg_base_value;
            logic        is_seg_cmd;
            is_seg_cmd = BLOCKED_SCHED && (cmd_type_q == CMD_INIT_SEG);
            start_line_value = bram_data_q[31:0];
            line_count_value = bram_data_q[63:32];
            // CMD_INIT_SEG: {start_line, line_count} are segment-relative;
            // the base comes from the blocked_ctrl descriptor table
            // (blk_desc_rd_addr_o is driven combinationally from this word).
            // region_sel==3 (image region): lines are image-relative, base 0
            // (the bridge adds NET_IMAGE_DRAM_BASE_ADDR); block_id ignored.
            seg_base_value = !is_seg_cmd
              ? '0
              : (
                (bram_data_q[72:70] == 3'd3)
                  ? 32'd0
                  : blk_desc_rd_data_i
              );
            max_line_count = BANK_DEPTH;
            init_start_row_q <= seg_base_value + start_line_value;
            init_row_count_q <= line_count_value;
            init_total_value_count_q <= line_count_value;
            init_cur_row_q <= seg_base_value + start_line_value;
            init_payload_word_idx_q <= '0;
            init_is_rms_q <= is_seg_cmd && (bram_data_q[72:70] == 3'd1);
            init_is_image_q <= is_seg_cmd && (bram_data_q[72:70] == 3'd3);
            if (
              (line_count_value == 0) ||
              (
                !is_seg_cmd &&
                ((start_line_value + line_count_value) > max_line_count)
              )
            ) begin
              error_code_q <= ERR_RANGE_ERROR;
              status_state_q <= ST_ERROR;
              state_q <= A_WRITE_FINAL;
            end else if ((payload_end_q - payload_start_q) < (1 + line_count_value)) begin
              error_code_q <= ERR_BAD_LENGTH;
              status_state_q <= ST_ERROR;
              state_q <= A_WRITE_FINAL;
            end else begin
              fsm_bram_addr_q <= bram_addr(payload_start_q + 1);
              state_q <= A_INIT_DATA_ADDR;
            end
          end

          A_INIT_DATA_ADDR: begin
            fsm_bram_addr_q <= bram_addr(payload_start_q + 1 + init_payload_word_idx_q);
            state_q <= A_INIT_DATA_WAIT0;
          end

          A_INIT_DATA_WAIT0: begin
            state_q <= A_INIT_DATA_WAIT1;
          end

          A_INIT_DATA_WAIT1: begin
            state_q <= A_INIT_DATA_CAPTURE;
          end

          A_INIT_DATA_CAPTURE: begin
            init_word_q <= bram_data_q;
            state_q <= A_INIT_EMIT;
          end

          A_INIT_EMIT: begin
            if (!init_line_valid_o) begin
              init_line_valid_o <= 1'b1;
              init_line_index_o <= init_cur_row_q;
              init_line_is_rms_o <= init_is_rms_q;
              init_line_is_image_o <= init_is_image_q;
              init_line_last_o <= ((processed_count_q + 1'b1) >= init_total_value_count_q);
              init_line_data_o <= init_word_q;
            end else if (init_line_ready_i) begin
              init_line_valid_o <= 1'b0;
              init_line_last_o <= 1'b0;
              processed_count_q <= processed_count_q + 1'b1;
              if ((processed_count_q + 1'b1) >= init_total_value_count_q) begin
                // Blocked-GRADP v2: keep the system reset held through the
                // whole init sequence. Releasing it here would start a rogue
                // pre-softmax on half-initialized segments whose DDR reads
                // deadlock the bridge init path (a pending read request
                // blocks init line acceptance while the bridge cannot
                // service reads mid-command). Monolithic keeps the v1.0.5
                // behavior (the rogue run there is read-free: bank lengths
                // are still zero until CONFIG).
                if (!BLOCKED_SCHED) begin
                  system_reset_hold_o <= 1'b0;
                end
                error_code_q <= ERR_OK;
                status_state_q <= ST_DONE;
                state_q <= A_WRITE_FINAL;
              end else begin
                init_cur_row_q <= init_cur_row_q + 1'b1;
                init_payload_word_idx_q <= init_payload_word_idx_q + 1'b1;
                fsm_bram_addr_q <= bram_addr(payload_start_q + 1 + init_payload_word_idx_q + 1'b1);
                state_q <= A_INIT_DATA_ADDR;
              end
            end
          end

          A_STATIC_META_ADDR: begin
            fsm_bram_addr_q <= bram_addr(payload_start_q);
            state_q <= A_STATIC_META_WAIT0;
          end

          A_STATIC_META_WAIT0: begin
            state_q <= A_STATIC_META_WAIT1;
          end

          A_STATIC_META_WAIT1: begin
            state_q <= A_STATIC_META_CAPTURE;
          end

          A_STATIC_META_CAPTURE: begin
            logic [31:0] weight_count_value;
            logic [31:0] data_word_count;
            weight_count_value = bram_data_q[31:0];
            data_word_count = (weight_count_value + (STATIC_WEIGHTS_PER_WORD - 1)) / STATIC_WEIGHTS_PER_WORD;
            static_weight_count_q <= weight_count_value;
            static_loaded_count_q <= '0;
            static_payload_word_idx_q <= '0;
            if (weight_count_value > 32'(STATIC_WEIGHT_DEPTH)) begin
              error_code_q <= ERR_RANGE_ERROR;
              status_state_q <= ST_ERROR;
              state_q <= A_WRITE_FINAL;
            end else if ((payload_end_q - payload_start_q) < (1 + data_word_count)) begin
              error_code_q <= ERR_BAD_LENGTH;
              status_state_q <= ST_ERROR;
              state_q <= A_WRITE_FINAL;
            end else if (weight_count_value == 0) begin
              processed_count_q <= '0;
              error_code_q <= ERR_OK;
              status_state_q <= ST_DONE;
              state_q <= A_WRITE_FINAL;
            end else begin
              fsm_bram_addr_q <= bram_addr(payload_start_q + 1);
              state_q <= A_STATIC_DATA_ADDR;
            end
          end

          A_STATIC_DATA_ADDR: begin
            fsm_bram_addr_q <= bram_addr(payload_start_q + 1 + static_payload_word_idx_q);
            state_q <= A_STATIC_DATA_WAIT0;
          end

          A_STATIC_DATA_WAIT0: begin
            state_q <= A_STATIC_DATA_WAIT1;
          end

          A_STATIC_DATA_WAIT1: begin
            state_q <= A_STATIC_DATA_CAPTURE;
          end

          A_STATIC_DATA_CAPTURE: begin
            if ((static_loaded_count_q + 32'(STATIC_WEIGHTS_PER_WORD)) >= static_weight_count_q) begin
              processed_count_q <= static_weight_count_q;
              error_code_q <= ERR_OK;
              status_state_q <= ST_DONE;
              state_q <= A_WRITE_FINAL;
            end else begin
              static_loaded_count_q <= static_loaded_count_q + 32'(STATIC_WEIGHTS_PER_WORD);
              static_payload_word_idx_q <= static_payload_word_idx_q + 1'b1;
              fsm_bram_addr_q <= bram_addr(payload_start_q + 1 + static_payload_word_idx_q + 1'b1);
              state_q <= A_STATIC_DATA_ADDR;
            end
          end

          A_NET_LANE_ADDR: begin
            // self-run play: enter the pipelined streaming loop (one frame
            // per cycle; starve/space are handled inside the stream).
            state_q <= A_NET_STREAM;
          end

          A_NET_WAIT_READY: begin
            error_code_q <= ERR_RANGE_ERROR;
            status_state_q <= ST_ERROR;
            state_q <= A_WRITE_FINAL;
          end

          A_NET_STREAM: begin
            // Pipelined self-run play: one frame per cycle. Stage 0 (fire):
            // pop the fetcher head, push lane commands (comb), issue static
            // weight reads, and latch the pipe registers. Stage 1 (comb,
            // next cycle): the static-weight FIFO push reads the pipe. The
            // last frame's pipe drains into A_NET_ISSUE.
            if (sr_pipe_valid_q) begin
              processed_count_q <= processed_count_q + 1'b1;
            end
            if (sr_pipe_valid_q && sr_pipe_last_q) begin
              sr_pipe_valid_q <= 1'b0;
              state_q <= A_NET_ISSUE;
            end else if (sr_frame_valid_i && net_fifo_space_ready_s) begin
              sr_pipe_valid_q <= 1'b1;
              sr_pipe_last_q <= sr_frame_last_i;
              net_frame_idx_q <= net_frame_idx_q + 1'b1;
              wait_elapsed_q <= '0;
              for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
                logic [63:0] lane_record;
                logic [22:0] packed_idx;
                lane_record = net_frame_word_s[(core_idx * 64) +: 64];
                packed_idx = lane_record[23:1];
                sr_pipe_static_mask_q[core_idx] <=
                  lane_record[0] && lane_needs_static_weight(lane_record);
                sr_pipe_static_lane_q[core_idx] <=
                  static_weight_lane(MEM_IDX_W'(packed_idx[15:0]));
              end
              if (net_lane_static_bad_s) begin
                state_q <= A_NET_WAIT_READY;
              end
            end else begin
              sr_pipe_valid_q <= 1'b0;
            end
          end

          A_NET_ISSUE: begin
            if (net_all_fifo_empty_s) begin
              // self-run play: all frames issued; now observe the GRADP
              // drain before the master's block_done pulse
              wait_selector_q <= 32'd1;
              wait_timeout_q <= '0;
              wait_elapsed_q <= '0;
              state_q <= A_WAIT_CMD;
            end else begin
              wait_elapsed_q <= wait_elapsed_q + 1'b1;
            end
          end

          // Blocked-GRADP v2: block descriptor file writes. Payload word 0
          // is the header {start_addr[8:0], count[24:16]} (32-bit descriptor
          // words); data words follow, sixteen 32-bit words per BRAM word.
          A_BDESC_META_ADDR: begin
            fsm_bram_addr_q <= bram_addr(payload_start_q);
            state_q <= A_BDESC_META_WAIT0;
          end

          A_BDESC_META_WAIT0: begin
            state_q <= A_BDESC_META_WAIT1;
          end

          A_BDESC_META_WAIT1: begin
            state_q <= A_BDESC_META_CAPTURE;
          end

          A_BDESC_META_CAPTURE: begin
            logic [31:0] desc_word_need;
            desc_word_need = 32'd1 + ((32'(bram_data_q[24:16]) + 32'd15) >> 4);
            bdesc_addr_q <= bram_data_q[10:0];
            bdesc_count_q <= bram_data_q[24:16];
            bdesc_written_q <= '0;
            bdesc_word_idx_q <= '0;
            bdesc_lane_idx_q <= '0;
            if (
              (bram_data_q[24:16] == 16'd0) ||
              ({21'b0, bram_data_q[10:0]} + 32'(bram_data_q[24:16])) > 32'd2048
            ) begin
              error_code_q <= ERR_RANGE_ERROR;
              status_state_q <= ST_ERROR;
              state_q <= A_WRITE_FINAL;
            end else if ((payload_end_q - payload_start_q) < desc_word_need) begin
              error_code_q <= ERR_BAD_LENGTH;
              status_state_q <= ST_ERROR;
              state_q <= A_WRITE_FINAL;
            end else begin
              fsm_bram_addr_q <= bram_addr(payload_start_q + 1);
              state_q <= A_BDESC_ADDR;
            end
          end

          A_BDESC_ADDR: begin
            blk_desc_wr_valid_o <= 1'b0;
            fsm_bram_addr_q <= bram_addr(payload_start_q + 32'd1 + bdesc_word_idx_q);
            state_q <= A_BDESC_WAIT0;
          end

          A_BDESC_WAIT0: begin
            state_q <= A_BDESC_WAIT1;
          end

          A_BDESC_WAIT1: begin
            state_q <= A_BDESC_CAPTURE;
          end

          A_BDESC_CAPTURE: begin
            blk_desc_wr_valid_o <= 1'b0;
            bdesc_word_q <= bram_data_q;
            bdesc_lane_idx_q <= '0;
            state_q <= A_BDESC_WRITE;
          end

          A_BDESC_WRITE: begin
            blk_desc_wr_valid_o <= 1'b1;
            blk_desc_wr_addr_o <= bdesc_addr_q + bdesc_written_q[10:0];
            blk_desc_wr_data_o <= bdesc_word_q[(int'(bdesc_lane_idx_q) * 32) +: 32];
            bdesc_written_q <= bdesc_written_q + 1'b1;
            if ((bdesc_written_q + 1'b1) >= bdesc_count_q) begin
              state_q <= A_BDESC_DONE;
            end else if (bdesc_lane_idx_q == 4'd15) begin
              bdesc_word_idx_q <= bdesc_word_idx_q + 1'b1;
              state_q <= A_BDESC_ADDR;
            end else begin
              bdesc_lane_idx_q <= bdesc_lane_idx_q + 1'b1;
            end
          end

          A_BDESC_DONE: begin
            blk_desc_wr_valid_o <= 1'b0;
            processed_count_q <= 32'(bdesc_count_q);
            error_code_q <= ERR_OK;
            status_state_q <= ST_DONE;
            state_q <= A_WRITE_FINAL;
          end

          A_WAIT_CMD: begin
            if (wait_condition_s) begin
              if (self_run_q) begin
                // self-run play complete: blk_block_done_o (+blk_iter_last_o
                // on sr_play_last_q) pulses this cycle; return to the idle
                // poll for the master's next play
                self_run_q <= 1'b0;
                sr_play_ack_o <= 1'b0;
                state_q <= A_FLAG_ADDR;
              end else begin
                blk_wait_spill_q <= 1'b0;
                error_code_q <= ERR_OK;
                status_state_q <= ST_DONE;
                state_q <= A_WRITE_FINAL;
              end
            end else if ((wait_timeout_q != 0) && (wait_elapsed_q >= wait_timeout_q)) begin
              error_code_q <= ERR_TIMEOUT;
              status_state_q <= ST_ERROR;
              state_q <= A_WRITE_FINAL;
            end else begin
              wait_elapsed_q <= wait_elapsed_q + 1'b1;
            end
          end

          A_READBACK_REQ: begin
            if (readback_req_ready_i) begin
              state_q <= A_READBACK_RESP;
            end
          end

          A_READBACK_RESP: begin
            if (readback_resp_valid_i) begin
              logic [BRAM_DATA_W-1:0] packed_word;
              int unsigned rows_per_word;
              logic last_row;
              logic word_full;

              packed_word = readback_word_q;
              rows_per_word = P_ROWS_PER_WORD;
              last_row = ((readback_row_offset_q + 1'b1) >= arg1_q);
              word_full = ((int'(readback_slot_q) + 1) >= rows_per_word);
              for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
                packed_word[
                  ((int'(readback_slot_q) * BANK_COUNT + bank_idx) * P_DATA_W) +:
                  P_DATA_W
                ] = readback_p_data_i[bank_idx];
              end

              processed_count_q <= processed_count_q + 1'b1;
              if (last_row || word_full) begin
                fsm_bram_addr_q <= bram_addr(result_start_q + result_index_q);
                fsm_bram_wdata_q <= packed_word;
                fsm_bram_we_q <= {BRAM_STRB_W{1'b1}};
                readback_word_q <= '0;
                readback_slot_q <= '0;
                if (last_row) begin
                  detail_flags_q <= result_index_q + 1'b1;
                  state_q <= A_READBACK_FINISH;
                end else begin
                  result_index_q <= result_index_q + 1'b1;
                  readback_row_offset_q <= readback_row_offset_q + 1'b1;
                  state_q <= A_READBACK_REQ;
                end
              end else begin
                readback_word_q <= packed_word;
                readback_slot_q <= readback_slot_q + 1'b1;
                readback_row_offset_q <= readback_row_offset_q + 1'b1;
                state_q <= A_READBACK_REQ;
              end
            end
          end

          A_READBACK_FINISH: begin
            if (arg2_q[0]) begin
              readback_advance_o <= 1'b1;
              readback_expected_row_q <= '0;
            end else begin
              readback_expected_row_q <= arg0_q + arg1_q;
            end
            error_code_q <= ERR_OK;
            status_state_q <= ST_DONE;
            state_q <= A_WRITE_FINAL;
          end

          A_DEBUG_STATIC_READ: begin
            // One cycle with read port 0 asserted lets the synchronous
            // static-weight BRAM register mem_q[debug_static_addr_q] into
            // static_mem_rd_data_s[0]; the A_DEBUG_WRITE idx0 capture below
            // then samples a stable word.
            state_q <= A_DEBUG_WRITE;
          end

          A_DEBUG_WRITE: begin
            if (debug_word_idx_q == 3'd0) begin
              debug_word_q <= debug_word_s;
            end
            fsm_bram_addr_q <= bram_addr(result_start_q + debug_word_idx_q);
            fsm_bram_wdata_q <= (debug_word_idx_q == 3'd0) ? debug_word_s[0] : debug_word_q[debug_word_idx_q];
            fsm_bram_we_q <= {BRAM_STRB_W{1'b1}};
            if (debug_word_idx_q == 3'd6) begin
              processed_count_q <= 32'd7;
              error_code_q <= ERR_OK;
              status_state_q <= ST_DONE;
              state_q <= A_WRITE_FINAL;
            end else begin
              debug_word_idx_q <= debug_word_idx_q + 1'b1;
            end
          end

          A_RESET_WAIT: begin
            if ((full_reset_count_q + 1'b1) >= FULL_RESET_CYCLES) begin
              full_reset_active_q <= 1'b0;
              full_reset_count_q <= '0;
              processed_count_q <= '0;
              detail_flags_q <= FULL_RESET_CYCLES;
              error_code_q <= ERR_OK;
              status_state_q <= ST_DONE;
              state_q <= A_WRITE_FINAL;
            end else begin
              full_reset_count_q <= full_reset_count_q + 1'b1;
            end
          end

          A_WRITE_FINAL: begin
            elapsed_cycles_q <= command_cycle_q;
            fsm_bram_addr_q <= bram_addr(STATUS_INDEX);
            fsm_bram_wdata_q <= pack_status(status_state_q, cmd_type_q, seq_id_q, error_code_q, processed_count_q, command_cycle_q, detail_flags_q);
            fsm_bram_we_q <= {BRAM_STRB_W{1'b1}};
            busy_o <= 1'b0;
            error_o <= error_code_q;
            usr_irq_req_o[0] <= 1'b1;
            state_q <= A_IRQ_WAIT_ACK;
          end

          A_WRITE_EVENT: begin
            fsm_bram_addr_q <= bram_addr(EVENT_INDEX);
            fsm_bram_wdata_q <= pack_event_word();
            fsm_bram_we_q <= {BRAM_STRB_W{1'b1}};
            usr_irq_req_o[0] <= 1'b1;
            event_pending_q <= 1'b0;
            state_q <= A_IRQ_WAIT_ACK;
          end

          A_IRQ_WAIT_ACK: begin
            if (usr_irq_ack_sync_q) begin
              usr_irq_req_o[0] <= 1'b0;
              state_q <= A_FLAG_ADDR;
            end
          end

          default: begin
            state_q <= A_FLAG_ADDR;
          end
        endcase
      end
    end
  end

`ifndef SYNTHESIS
  // ---- NET-on-DDR self-run debug probes (sim only) ----
  logic self_run_dbg_q;
  logic [15:0] sr_stuck_cnt_q;
  logic [31:0] dbg_nsw_push_q [0:CORE_COUNT-1];
  logic [31:0] dbg_nsw_need_q [0:CORE_COUNT-1];
  always_ff @(posedge clk) begin
    self_run_dbg_q <= self_run_q;
    if (rst_n && sr_go_o) begin
      $display("%t ADA CMD_GO n=%0d", $time, arg0_q);
    end
    if (rst_n && (self_run_q != self_run_dbg_q)) begin
      $display("%t ADA self_run %0d -> %0d last_blk=%0d", $time, self_run_dbg_q, self_run_q, sr_play_last_q);
    end
    if (rst_n && blk_block_done_o) begin
      $display("%t ADA block_done iter_last=%0d", $time, blk_iter_last_o);
    end
    if (rst_n && self_run_q && sr_frame_pop_o && (net_frame_idx_q[10:0] == 11'd0)) begin
      $display("%t ADA play frame=%0d", $time, net_frame_idx_q);
    end
    // stuck-play dump: while a self-run play waits on fifo drain (A_NET_ISSUE)
    // or the GRADP drain (A_WAIT_CMD), periodically dump the blocking bitmap;
    // A_NET_STREAM included: the issue pipeline can wedge there too
    if (rst_n && self_run_q &&
        ((state_q == A_NET_ISSUE) || (state_q == A_WAIT_CMD) ||
         (state_q == A_NET_STREAM))) begin
      sr_stuck_cnt_q <= sr_stuck_cnt_q + 1'b1;
      if (sr_stuck_cnt_q == 16'd50000) begin
        sr_stuck_cnt_q <= '0;
        $display(
          "%t ADA STUCK st=%0d residue=%h finish=%0d task_rdy=%h elapsed=%0d",
          $time, state_q, net_fifo_residue_detail_s, grad_p_finish_i,
          all_grad_task_ready_s, wait_elapsed_q
        );
        for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
          $display(
            "%t ADA STUCK core=%0d cmd_empty=%0d cmd_ovalid=%0d s1n=%0d s2v=%0d s3v=%0d sw_empty=%0d sw_ovalid=%0d s2nst=%0d",
            $time, core_idx,
            cmd_fifo_empty_s[core_idx], cmd_fifo_output_valid_s[core_idx],
            iss_s1_count_q[core_idx], iss_s2_valid_q[core_idx],
            iss_s3_valid_q[core_idx],
            static_w_fifo_empty_s[core_idx], static_w_fifo_output_valid_s[core_idx],
            iss_s2_need_static_q[core_idx]
          );
          $display(
            "%t ADA STUCK core=%0d s3cmd=%h head=%h host_rdy=%0d wlvia_warn=%0d nsw_push=%0d nsw_need=%0d dspdbg=%04h",
            $time, core_idx, iss_s3_cmd_q[core_idx], cmd_fifo_head_s[core_idx],
            grad_overflow_host_ready_i[core_idx / CORE_COUNT_PER_FIFO][core_idx % CORE_COUNT_PER_FIFO],
            grad_wlvia_fifo_warning_i[core_idx / SEGMENT_PER_FIFO],
            dbg_nsw_push_q[core_idx], dbg_nsw_need_q[core_idx],
            gradp_debug_core_i[core_idx]
          );
        end
      end
    end else begin
      sr_stuck_cnt_q <= '0;
    end
  end

  // self-run static-weight pairing audit (sim only): count static pushes vs
  // sw-empty stall events at the S1->S2 transfer per core, and log each one
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
        dbg_nsw_push_q[core_idx] <= '0;
        dbg_nsw_need_q[core_idx] <= '0;
      end
    end else begin
      for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
        if (static_w_fifo_push_s[core_idx]) begin
          dbg_nsw_push_q[core_idx] <= dbg_nsw_push_q[core_idx] + 1'b1;
        end
        if (
          net_issue_active_s &&
          iss_s1_valid_q[core_idx][iss_s1_rd_ptr_q[core_idx]] &&
          lane_needs_static_weight(iss_s1_data_q[core_idx][iss_s1_rd_ptr_q[core_idx]]) &&
          !(
            (static_w_fifo_shadow_q[core_idx] != '0) &&
            static_w_fifo_ready_s[core_idx]
          ) &&
          (dbg_nsw_need_q[core_idx] == '0)
        ) begin
          dbg_nsw_need_q[core_idx] <= dbg_nsw_need_q[core_idx] + 1'b1;
          $display(
            "%t ADA SWSTALL core=%0d frame=%0d head=%h sw_empty=%0d nsw_push=%0d nsw_need=%0d",
            $time, core_idx, net_frame_idx_q, cmd_fifo_head_s[core_idx],
            static_w_fifo_empty_s[core_idx], dbg_nsw_push_q[core_idx],
            dbg_nsw_need_q[core_idx] + 1
          );
        end else if (static_w_fifo_pop_s[core_idx]) begin
          dbg_nsw_need_q[core_idx] <= '0;
        end
      end
    end
  end

  // tail forensics (sim only): every cmd/static fifo event at the block tail
  // to catch the exact shadow/IP divergence moment
  always_ff @(posedge clk) begin
    if (rst_n && self_run_q && (net_frame_idx_q >= 32'd5940)) begin
      for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
        if (cmd_fifo_push_s[core_idx]) begin
          $display("%t ADA TPUSH core=%0d frame=%0d rec=%h ipfull=%0d ipwarn=%0d",
            $time, core_idx, net_frame_idx_q, cmd_fifo_push_data_s[core_idx],
            cmd_fifo_full_s[core_idx], cmd_fifo_full_warning_s[core_idx]);
        end
        if (cmd_fifo_pop_s[core_idx]) begin
          $display("%t ADA TPOP core=%0d frame=%0d shadow=%0d ipempty=%0d ipvalid=%0d",
            $time, core_idx, net_frame_idx_q, cmd_fifo_shadow_q[core_idx],
            cmd_fifo_empty_s[core_idx], cmd_fifo_output_valid_s[core_idx]);
        end
        if (iss_s1_adv_s[core_idx]) begin
          $display("%t ADA TREAD core=%0d frame=%0d head=%h need_st=%0d swshadow=%0d",
            $time, core_idx, net_frame_idx_q,
            iss_s1_data_q[core_idx][iss_s1_rd_ptr_q[core_idx]],
            lane_needs_static_weight(iss_s1_data_q[core_idx][iss_s1_rd_ptr_q[core_idx]]),
            static_w_fifo_shadow_q[core_idx]);
        end
        if (static_w_fifo_push_s[core_idx]) begin
          $display("%t ADA TSWPUSH core=%0d frame=%0d data=%h",
            $time, core_idx, net_frame_idx_q, static_w_fifo_push_data_s[core_idx]);
        end
        if (static_w_fifo_pop_s[core_idx]) begin
          $display("%t ADA TSWPOP core=%0d frame=%0d swshadow=%0d swempty=%0d",
            $time, core_idx, net_frame_idx_q, static_w_fifo_shadow_q[core_idx],
            static_w_fifo_empty_s[core_idx]);
        end
      end
    end
  end

  // ---- full-stream FIFO forensics dump (sim only) ----
  // +ADA_FIFO_DUMP=<path> writes one line per pairing/fifo event for the
  // whole run (no frame_idx cutoff). Parsed offline by
  // verif/fifo_stream_diff.py against the expected stream derived from the
  // partition artifact.
  string ada_dump_path;
  integer ada_dump_fd = 0;
  integer ada_dump_lines = 0;
  initial begin
    if ($value$plusargs("ADA_FIFO_DUMP=%s", ada_dump_path)) begin
      ada_dump_fd = $fopen(ada_dump_path, "w");
      if (ada_dump_fd == 0) begin
        $display("%t ADA DUMP: failed to open %s", $time, ada_dump_path);
      end
    end
  end
  final begin
    if (ada_dump_fd != 0) begin
      $fclose(ada_dump_fd);
    end
  end

  task automatic ada_dump_line(input string line);
    if (ada_dump_fd != 0) begin
      ada_dump_lines = ada_dump_lines + 1;
      $fwrite(ada_dump_fd, "%0t %s\n", $time, line);
      if ((ada_dump_lines % 4096) == 0) begin
        $fflush(ada_dump_fd);
      end
    end
  endtask

  always_ff @(posedge clk) begin
    if (rst_n && (ada_dump_fd != 0)) begin
      if (sr_go_o) begin
        ada_dump_line($sformatf("BLK go n=%0d", arg0_q));
      end
      if (blk_block_done_o) begin
        ada_dump_line($sformatf("BLK done iter_last=%0d", blk_iter_last_o));
      end
      if (sr_frame_pop_o) begin
        ada_dump_line($sformatf("FPOP frame=%0d", net_frame_idx_q));
      end
      for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
        logic [63:0] dump_lane_record;
        dump_lane_record = net_frame_word_s[(core_idx * 64) +: 64];
        if (
          ((state_q == A_NET_LANE_CAPTURE) ||
           ((state_q == A_NET_STREAM) && sr_frame_valid_i)) &&
          net_fifo_space_ready_s &&
          dump_lane_record[0] &&
          lane_needs_static_weight(dump_lane_record)
        ) begin
          ada_dump_line($sformatf("MSET core=%0d frame=%0d rec=%016h",
            core_idx, net_frame_idx_q, dump_lane_record));
        end
        if (cmd_fifo_push_s[core_idx]) begin
          ada_dump_line($sformatf("CPUSH core=%0d frame=%0d rec=%016h",
            core_idx, net_frame_idx_q, cmd_fifo_push_data_s[core_idx]));
        end
        if (cmd_fifo_pop_s[core_idx]) begin
          ada_dump_line($sformatf("CPOP core=%0d frame=%0d head=%016h",
            core_idx, net_frame_idx_q, cmd_fifo_head_s[core_idx]));
        end
        if (iss_s1_adv_s[core_idx]) begin
          ada_dump_line($sformatf("CRD core=%0d frame=%0d head=%016h need_st=%0d",
            core_idx, net_frame_idx_q,
            iss_s1_data_q[core_idx][iss_s1_rd_ptr_q[core_idx]],
            lane_needs_static_weight(iss_s1_data_q[core_idx][iss_s1_rd_ptr_q[core_idx]])));
        end
        if (static_w_fifo_push_s[core_idx]) begin
          ada_dump_line($sformatf("SWPUSH core=%0d frame=%0d data=%016h lane=%0d",
            core_idx, net_frame_idx_q, static_w_fifo_push_data_s[core_idx],
            sr_pipe_static_lane_q[core_idx]));
        end
        if (static_w_fifo_pop_s[core_idx]) begin
          ada_dump_line($sformatf("SWPOP core=%0d frame=%0d",
            core_idx, net_frame_idx_q));
        end
        if (iss_s3_emit_s[core_idx]) begin
          ada_dump_line($sformatf("ISS core=%0d frame=%0d need_st=%0d cmd=%016h w=%016h",
            core_idx, net_frame_idx_q, iss_s3_need_static_q[core_idx],
            iss_s3_cmd_q[core_idx], iss_s3_static_w_q[core_idx]));
        end
      end
    end
  end
`endif

endmodule

`endif
