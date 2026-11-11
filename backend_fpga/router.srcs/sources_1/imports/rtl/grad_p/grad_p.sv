`ifndef FPGA_GRAD_P_SV
`define FPGA_GRAD_P_SV

`include "../common/segcore_pkg.sv"

module grad_p #(
  parameter int CORE_COUNT = 8,
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
  parameter int BANK_DEPTH = 65536,
  parameter int DISPATCHER_FIFO_DEPTH = 512,
  parameter int GRAD_OVERFLOW_FIFO_DEPTH = 512,
  parameter int GRAD_WLVIA_FIFO_DEPTH = 512,
  parameter bit NATIVE_QUERY_ENABLE = 1'b1,
  parameter int QUERY_PREFETCH_FIFO_DEPTH = 16,
  parameter int QUERY_ACT_SCALE_FXP = (1 << (FRAC_W - 1)),
  parameter int ACT_GROUP = 8,
  parameter int ACT_PIPELINE = 2,
  parameter int PRE_ACT_IDLE_CYCLES = 0,
  parameter int POST_ACT_IDLE_CYCLES = ACT_PIPELINE + 2
) (
  input  logic                                                clk,
  input  logic                                                rst_n,
  input  segcore_pkg::banked_idx_t                            segment_idx_i [0:CORE_COUNT-1],
  input  logic [$clog2(CHUNK_SIZE)-1:0]                       segment_l_i [0:CORE_COUNT-1],
  input  logic [$clog2(CHUNK_SIZE)-1:0]                       segment_r_i [0:CORE_COUNT-1],
  input  logic signed [DATA_W-1:0]                            segment_w_i [0:CORE_COUNT-1],
  input  logic signed [COEFF_W-1:0]                           segment_coeff_i [0:CORE_COUNT-1],
  input  logic [2:0]                                          segment_type_i [0:CORE_COUNT-1],
  input  logic [1:0]                                          mode_overflow_i [0:(CORE_COUNT/CORE_COUNT_PER_FIFO)-1][0:CORE_COUNT_PER_FIFO-1],
  input  logic                                                mode_overflow_valid_i [0:(CORE_COUNT/CORE_COUNT_PER_FIFO)-1][0:CORE_COUNT_PER_FIFO-1],
  input  logic [2:0]                                          mode_wlvia_i [0:(CORE_COUNT/SEGMENT_PER_FIFO)-1],
  output logic                                                grad_overflow_host_ready_o [0:(CORE_COUNT/CORE_COUNT_PER_FIFO)-1][0:CORE_COUNT_PER_FIFO-1],
  output logic                                                grad_overflow_fifo_warning_o [0:(CORE_COUNT/CORE_COUNT_PER_FIFO)-1][0:CORE_COUNT_PER_FIFO-1],
  output logic                                                dispatcher_fifo_overflow_o [0:(CORE_COUNT/CORE_COUNT_PER_FIFO)-1][0:CORE_COUNT_PER_FIFO-1],
  output logic                                                grad_wlvia_fifo_warning_o [0:(CORE_COUNT/SEGMENT_PER_FIFO)-1],
  output logic                                                finish_o,
  output logic                                                task_ready_o [0:CORE_COUNT-1],
  output logic                                                grad_mem_rd_ce_o [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  output logic [segcore_pkg::MEM_IDX_W-1:0]                   grad_mem_rd_addr_o [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  input  logic signed [ACC_W-1:0]                             grad_mem_rd_data_i [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  output logic                                                grad_mem_wr_ce_o [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  output logic [segcore_pkg::MEM_IDX_W-1:0]                   grad_mem_wr_addr_o [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  output logic signed [ACC_W-1:0]                             grad_mem_wr_data_o [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  output logic                                                p_mem_rd_ce_o [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  output logic [segcore_pkg::MEM_IDX_W-1:0]                   p_mem_rd_addr_o [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  input  logic signed [P_DATA_W-1:0]                          p_mem_rd_data_i [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  output logic [15:0]                                         debug_core_o [0:CORE_COUNT-1],
  output logic [31:0]                                         debug_bank_o,
  // Flattened overflow-datapath trace taps ((CORE_COUNT*13+2) x 32-bit):
  // core c dispatcher field f at [(c*8+f)*32 +: 32] (f=0..7, see
  // dispatcher_core.debug_trace_o),
  // field CORE_COUNT*8   = merge_overflow_capture_count,
  // field CORE_COUNT*8+1 = merge_overflow_nonzero_count,
  // core c segcore field k at [(CORE_COUNT*8+2+c*5+k)*32 +: 32] (k=0..4,
  // see grad_overflow_core_xN.debug_seg_trace_o). Diagnostic only.
  output logic [(CORE_COUNT*13+2)*32-1:0]                     gradp_debug_trace_o,
  // Stall probe bundle for the adapter performance mirror (P0/P1 counters):
  //   [3:0]   p native-query request stalled (valid && !grant), per overflow
  //           group (lanes OR-combined)
  //   [11:4]  p bank grant pulse (one cycle per accepted request), per bank
  //   [19:12] grad update commit pulse, per bank
  //   [20 +: TOTAL_SOURCES] update source hold stalled
  //           (hold_valid && !commit_clear, level)
  output logic [31:0]                                         stall_probe_o
);
  import segcore_pkg::*;

  localparam int ADDR_W = (CHUNK_SIZE <= 1) ? 1 : $clog2(CHUNK_SIZE);
  localparam int GRAD_OVERFLOW_COUNT = CORE_COUNT / CORE_COUNT_PER_FIFO;
  localparam int GRAD_WLVIA_COUNT = CORE_COUNT / SEGMENT_PER_FIFO;
  localparam int BANK_COUNT = GRAD_OVERFLOW_COUNT + GRAD_WLVIA_COUNT;
  localparam int TOTAL_SOURCES = GRAD_OVERFLOW_COUNT + GRAD_WLVIA_COUNT;
  localparam int SOURCE_IDX_W = (TOTAL_SOURCES <= 1) ? 1 : $clog2(TOTAL_SOURCES);
  localparam int OVERFLOW_GROUP_IDX_W = (GRAD_OVERFLOW_COUNT <= 1) ? 1 : $clog2(GRAD_OVERFLOW_COUNT);
  localparam int P_REQ_COUNT = GRAD_OVERFLOW_COUNT * CORE_COUNT_PER_FIFO;
  localparam int P_REQ_IDX_W = (P_REQ_COUNT <= 1) ? 1 : $clog2(P_REQ_COUNT);

  localparam int OVERFLOW_MODE_SEGMENT = 2'd2;
  localparam int WLVIA_MODE_IDLE = 3'b000;
  localparam int WLVIA_MODE_COMPUTE = 3'b111;
  localparam int TYPE_VER_VIA = 3'd5;

  banked_idx_t overflow_segment_idx_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic [ADDR_W-1:0] overflow_segment_l_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic [ADDR_W-1:0] overflow_segment_r_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic signed [DATA_W-1:0] overflow_segment_w_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic signed [COEFF_W-1:0] overflow_segment_coeff_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic [2:0] overflow_segment_type_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];

  banked_idx_t wlvia_segment_idx_s [0:GRAD_WLVIA_COUNT-1][0:SEGMENT_PER_FIFO-1];
  logic signed [DATA_W-1:0] wlvia_segment_len_s [0:GRAD_WLVIA_COUNT-1][0:SEGMENT_PER_FIFO-1];
  logic [2:0] wlvia_segment_type_s [0:GRAD_WLVIA_COUNT-1][0:SEGMENT_PER_FIFO-1];
  logic wlvia_segment_valid_s [0:GRAD_WLVIA_COUNT-1][0:SEGMENT_PER_FIFO-1];
  logic [ADDR_W:0] segment_span_s [0:CORE_COUNT-1];

  banked_idx_t overflow_upd_idx_s [0:GRAD_OVERFLOW_COUNT-1];
  logic signed [DATA_W-1:0] overflow_upd_w_s [0:GRAD_OVERFLOW_COUNT-1];
  logic overflow_upd_valid_s [0:GRAD_OVERFLOW_COUNT-1];
  logic overflow_pop_enable_s [0:GRAD_OVERFLOW_COUNT-1];
  logic overflow_fifo_empty_s [0:GRAD_OVERFLOW_COUNT-1];
  logic overflow_dispatcher_pending_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic overflow_pending_push_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic overflow_task_finish_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic overflow_task_fifo_warning_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic overflow_task_fifo_overflow_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic overflow_host_ready_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic overflow_p_req_valid_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  banked_idx_t overflow_p_req_idx_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic overflow_p_req_grant_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic overflow_p_resp_valid_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic signed [P_DATA_W-1:0] overflow_p_resp_data_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic p_bank_req_valid_s [0:BANK_COUNT-1];
  logic [MEM_IDX_W-1:0] p_bank_req_addr_s [0:BANK_COUNT-1];
  logic [OVERFLOW_GROUP_IDX_W-1:0] p_bank_req_group_s [0:BANK_COUNT-1];
  logic [CORE_COUNT_PER_FIFO-1:0] p_bank_req_lane_sel_s [0:BANK_COUNT-1];
  logic [P_REQ_IDX_W-1:0] p_bank_rr_start_q [0:BANK_COUNT-1];
  logic [P_REQ_IDX_W-1:0] p_bank_rr_start_d [0:BANK_COUNT-1];
  logic p_bank_resp_pending_q [0:BANK_COUNT-1];
  logic [OVERFLOW_GROUP_IDX_W-1:0] p_bank_resp_group_q [0:BANK_COUNT-1];
  logic [CORE_COUNT_PER_FIFO-1:0] p_bank_resp_lane_sel_q [0:BANK_COUNT-1];

  banked_idx_t wlvia_upd_idx_s [0:GRAD_WLVIA_COUNT-1];
  logic signed [DATA_W-1:0] wlvia_upd_w_s [0:GRAD_WLVIA_COUNT-1];
  logic wlvia_upd_valid_s [0:GRAD_WLVIA_COUNT-1];
  logic wlvia_pop_enable_s [0:GRAD_WLVIA_COUNT-1];
  logic wlvia_fifo_empty_s [0:GRAD_WLVIA_COUNT-1];
  logic wlvia_pending_push_s [0:GRAD_WLVIA_COUNT-1][0:SEGMENT_PER_FIFO-1];
  logic wlvia_fifo_warning_s [0:GRAD_WLVIA_COUNT-1];

  banked_idx_t source_upd_idx_s [0:TOTAL_SOURCES-1];
  logic signed [DATA_W-1:0] source_upd_w_s [0:TOTAL_SOURCES-1];
  logic source_upd_valid_s [0:TOTAL_SOURCES-1];
  logic source_pop_enable_s [0:TOTAL_SOURCES-1];
  banked_idx_t source_hold_idx_q [0:TOTAL_SOURCES-1];
  logic signed [DATA_W-1:0] source_hold_w_q [0:TOTAL_SOURCES-1];
  logic source_hold_valid_q [0:TOTAL_SOURCES-1];
  logic source_capture_fire_s [0:TOTAL_SOURCES-1];
  logic source_commit_clear_s [0:TOTAL_SOURCES-1];
  logic [15:0] overflow_debug_core_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic [8*32-1:0] overflow_debug_trace_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic [5*32-1:0] overflow_debug_seg_trace_s [0:GRAD_OVERFLOW_COUNT-1][0:CORE_COUNT_PER_FIFO-1];
  logic [31:0] merge_overflow_capture_count_q;
  logic [31:0] merge_overflow_nonzero_count_q;
  logic [31:0] merge_overflow_capture_inc_s;
  logic [31:0] merge_overflow_nonzero_inc_s;

  logic bank_select_valid_s [0:BANK_COUNT-1];
  logic [SOURCE_IDX_W-1:0] bank_select_src_s [0:BANK_COUNT-1];
  logic [MEM_IDX_W-1:0] bank_select_addr_s [0:BANK_COUNT-1];
  logic signed [DATA_W-1:0] bank_select_delta_s [0:BANK_COUNT-1];
  logic [SOURCE_IDX_W-1:0] bank_select_rr_next_s [0:BANK_COUNT-1];
  logic bank_select_valid_q [0:BANK_COUNT-1];
  logic [SOURCE_IDX_W-1:0] bank_select_src_q [0:BANK_COUNT-1];
  logic [MEM_IDX_W-1:0] bank_select_addr_q [0:BANK_COUNT-1];
  logic signed [DATA_W-1:0] bank_select_delta_q [0:BANK_COUNT-1];
  logic [SOURCE_IDX_W-1:0] bank_select_rr_next_q [0:BANK_COUNT-1];

  logic bank_commit_valid_s [0:BANK_COUNT-1];
  logic [SOURCE_IDX_W-1:0] bank_commit_src_s [0:BANK_COUNT-1];
  logic [MEM_IDX_W-1:0] bank_commit_addr_s [0:BANK_COUNT-1];
  logic signed [DATA_W-1:0] bank_commit_delta_s [0:BANK_COUNT-1];
  logic bank_rmw_pending_q [0:BANK_COUNT-1];
  logic [MEM_IDX_W-1:0] bank_rmw_addr_q [0:BANK_COUNT-1];
  logic signed [DATA_W-1:0] bank_rmw_delta_q [0:BANK_COUNT-1];
  logic [BANK_COUNT-1:0] grad_mem_pending_s;
  logic [SOURCE_IDX_W-1:0] bank_rr_start_q [0:BANK_COUNT-1];

  initial begin
    assert (CORE_COUNT >= 1)
      else $error(1, "grad_p: CORE_COUNT must be >= 1");
    assert (CORE_COUNT_PER_FIFO == 1)
      else $error(1, "grad_p: CORE_COUNT_PER_FIFO must be 1");
    assert (SEGMENT_PER_FIFO == 1)
      else $error(1, "grad_p: SEGMENT_PER_FIFO must be 1");
    assert ((CORE_COUNT % CORE_COUNT_PER_FIFO) == 0)
      else $error(1, "grad_p: CORE_COUNT must be divisible by CORE_COUNT_PER_FIFO");
    assert ((CORE_COUNT % SEGMENT_PER_FIFO) == 0)
      else $error(1, "grad_p: CORE_COUNT must be divisible by SEGMENT_PER_FIFO");
    assert (BANK_COUNT <= (1 << BANK_IDX_W))
      else $error(1, "grad_p: BANK_COUNT exceeds banked_idx_t.bank_idx width");
    assert (BANK_DEPTH <= (1 << MEM_IDX_W))
      else $error(1, "grad_p: BANK_DEPTH exceeds banked_idx_t.mem_idx width");
    assert (TOTAL_SOURCES <= 12)
      else $error(1, "grad_p: TOTAL_SOURCES exceeds stall_probe_o source field");
  end

  generate
    for (genvar core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin : GEN_SEGMENT_SPAN
      assign segment_span_s[core_idx] = {1'b0, segment_r_i[core_idx]} - {1'b0, segment_l_i[core_idx]} + 1'b1;
    end
  endgenerate

  generate
    for (genvar overflow_group_idx = 0; overflow_group_idx < GRAD_OVERFLOW_COUNT; overflow_group_idx++) begin : GEN_OVERFLOW_GROUP
      for (genvar overflow_lane_idx = 0; overflow_lane_idx < CORE_COUNT_PER_FIFO; overflow_lane_idx++) begin : GEN_OVERFLOW_LANE
        localparam int GLOBAL_CORE_IDX = (overflow_group_idx * CORE_COUNT_PER_FIFO) + overflow_lane_idx;

        assign overflow_segment_idx_s[overflow_group_idx][overflow_lane_idx] = segment_idx_i[GLOBAL_CORE_IDX];
        assign overflow_segment_l_s[overflow_group_idx][overflow_lane_idx] = segment_l_i[GLOBAL_CORE_IDX];
        assign overflow_segment_r_s[overflow_group_idx][overflow_lane_idx] = segment_r_i[GLOBAL_CORE_IDX];
        assign overflow_segment_w_s[overflow_group_idx][overflow_lane_idx] = segment_w_i[GLOBAL_CORE_IDX];
        assign overflow_segment_coeff_s[overflow_group_idx][overflow_lane_idx] = segment_coeff_i[GLOBAL_CORE_IDX];
        assign overflow_segment_type_s[overflow_group_idx][overflow_lane_idx] = segment_type_i[GLOBAL_CORE_IDX];
      end

      grad_overflow_core_xN #(
        .CORE_COUNT_PER_FIFO(CORE_COUNT_PER_FIFO),
        .ADDR_W(ADDR_W),
        .DATA_W(DATA_W),
        .FRAC_W(FRAC_W),
        .COEFF_W(COEFF_W),
        .COEFF_FRAC_W(COEFF_FRAC_W),
        .P_DATA_W(P_DATA_W),
        .P_FRAC_W(P_FRAC_W),
        .BANK_DEPTH(BANK_DEPTH),
        .DISPATCHER_FIFO_DEPTH(DISPATCHER_FIFO_DEPTH),
        .UPD_FIFO_DEPTH(GRAD_OVERFLOW_FIFO_DEPTH),
        .NATIVE_QUERY_ENABLE(NATIVE_QUERY_ENABLE),
        .QUERY_PREFETCH_FIFO_DEPTH(QUERY_PREFETCH_FIFO_DEPTH),
        .QUERY_ACT_SCALE_FXP(QUERY_ACT_SCALE_FXP),
        .ACT_GROUP(ACT_GROUP),
        .ACT_PIPELINE(ACT_PIPELINE),
        .PRE_ACT_IDLE_CYCLES(PRE_ACT_IDLE_CYCLES),
        .POST_ACT_IDLE_CYCLES(POST_ACT_IDLE_CYCLES)
      ) u_grad_overflow_core_xN (
        .clk(clk),
        .rst_n(rst_n),
        .mode_i(mode_overflow_i[overflow_group_idx]),
        .mode_valid_i(mode_overflow_valid_i[overflow_group_idx]),
        .segment_idx_i(overflow_segment_idx_s[overflow_group_idx]),
        .segment_l_i(overflow_segment_l_s[overflow_group_idx]),
        .segment_r_i(overflow_segment_r_s[overflow_group_idx]),
        .segment_w_i(overflow_segment_w_s[overflow_group_idx]),
        .segment_coeff_i(overflow_segment_coeff_s[overflow_group_idx]),
        .segment_type_i(overflow_segment_type_s[overflow_group_idx]),
        .pop_enable_i(overflow_pop_enable_s[overflow_group_idx]),
        .host_ready_o(overflow_host_ready_s[overflow_group_idx]),
        .p_req_valid_o(overflow_p_req_valid_s[overflow_group_idx]),
        .p_req_idx_o(overflow_p_req_idx_s[overflow_group_idx]),
        .p_req_grant_i(overflow_p_req_grant_s[overflow_group_idx]),
        .p_resp_valid_i(overflow_p_resp_valid_s[overflow_group_idx]),
        .p_resp_data_i(overflow_p_resp_data_s[overflow_group_idx]),
        .upd_idx_o(overflow_upd_idx_s[overflow_group_idx]),
        .upd_w_o(overflow_upd_w_s[overflow_group_idx]),
        .upd_valid_o(overflow_upd_valid_s[overflow_group_idx]),
        .fifo_empty_o(overflow_fifo_empty_s[overflow_group_idx]),
        .dispatcher_pending_o(overflow_dispatcher_pending_s[overflow_group_idx]),
        .pending_push_o(overflow_pending_push_s[overflow_group_idx]),
        .task_finish_o(overflow_task_finish_s[overflow_group_idx]),
        .task_fifo_warning_o(overflow_task_fifo_warning_s[overflow_group_idx]),
        .task_fifo_overflow_o(overflow_task_fifo_overflow_s[overflow_group_idx]),
        .debug_core_o(overflow_debug_core_s[overflow_group_idx]),
        .debug_trace_o(overflow_debug_trace_s[overflow_group_idx]),
        .debug_seg_trace_o(overflow_debug_seg_trace_s[overflow_group_idx])
     );

      for (genvar overflow_lane_idx = 0; overflow_lane_idx < CORE_COUNT_PER_FIFO; overflow_lane_idx++) begin : GEN_OVERFLOW_WARN
        localparam int READY_CORE_IDX = (overflow_group_idx * CORE_COUNT_PER_FIFO) + overflow_lane_idx;

        assign grad_overflow_fifo_warning_o[overflow_group_idx][overflow_lane_idx] =
          overflow_task_fifo_warning_s[overflow_group_idx][overflow_lane_idx];
        assign dispatcher_fifo_overflow_o[overflow_group_idx][overflow_lane_idx] =
          overflow_task_fifo_overflow_s[overflow_group_idx][overflow_lane_idx];
        assign grad_overflow_host_ready_o[overflow_group_idx][overflow_lane_idx] =
          overflow_host_ready_s[overflow_group_idx][overflow_lane_idx];
        assign task_ready_o[READY_CORE_IDX] =
          overflow_task_finish_s[overflow_group_idx][overflow_lane_idx] &&
          !overflow_dispatcher_pending_s[overflow_group_idx][overflow_lane_idx];
      end
    end
  endgenerate

  always_comb begin
    for (int overflow_group_idx = 0; overflow_group_idx < GRAD_OVERFLOW_COUNT; overflow_group_idx++) begin
      for (int overflow_lane_idx = 0; overflow_lane_idx < CORE_COUNT_PER_FIFO; overflow_lane_idx++) begin
        overflow_p_req_grant_s[overflow_group_idx][overflow_lane_idx] = 1'b0;
        overflow_p_resp_valid_s[overflow_group_idx][overflow_lane_idx] = 1'b0;
        overflow_p_resp_data_s[overflow_group_idx][overflow_lane_idx] = '0;
      end
    end

    for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
      p_bank_req_valid_s[bank_idx] = 1'b0;
      p_bank_req_addr_s[bank_idx] = '0;
      p_bank_req_group_s[bank_idx] = '0;
      p_bank_req_lane_sel_s[bank_idx] = '0;
      p_bank_rr_start_d[bank_idx] = p_bank_rr_start_q[bank_idx];

      // Pipelined grant (v4.4): the bank accepts one request EVERY cycle
      // (the BRAM supports a read per cycle); the response returns with a
      // fixed one-cycle latency. The old single-outstanding gate
      // (!p_bank_resp_pending_q) halved the bank throughput to serialize
      // the per-core blocking frontend, which the prefetch FIFOs retired.
      for (int req_offset = 0; req_offset < P_REQ_COUNT; req_offset++) begin
        int req_linear_idx;
        int overflow_group_idx;
        int overflow_lane_idx;

        req_linear_idx = p_bank_rr_start_q[bank_idx] + req_offset;
        if (req_linear_idx >= P_REQ_COUNT) begin
          req_linear_idx -= P_REQ_COUNT;
        end
        overflow_group_idx = req_linear_idx / CORE_COUNT_PER_FIFO;
        overflow_lane_idx = req_linear_idx % CORE_COUNT_PER_FIFO;

        if (
          !p_bank_req_valid_s[bank_idx] &&
          overflow_p_req_valid_s[overflow_group_idx][overflow_lane_idx] &&
          (overflow_p_req_idx_s[overflow_group_idx][overflow_lane_idx].bank_idx == bank_idx[BANK_IDX_W-1:0])
        ) begin
          p_bank_req_valid_s[bank_idx] = 1'b1;
          p_bank_req_addr_s[bank_idx] =
            overflow_p_req_idx_s[overflow_group_idx][overflow_lane_idx].mem_idx;
          p_bank_req_group_s[bank_idx] = overflow_group_idx[OVERFLOW_GROUP_IDX_W-1:0];
          p_bank_req_lane_sel_s[bank_idx][overflow_lane_idx] = 1'b1;
          overflow_p_req_grant_s[overflow_group_idx][overflow_lane_idx] = 1'b1;
          if (P_REQ_COUNT > 1) begin
            if (req_linear_idx == (P_REQ_COUNT - 1)) begin
              p_bank_rr_start_d[bank_idx] = '0;
            end else begin
              p_bank_rr_start_d[bank_idx] = req_linear_idx + 1'b1;
            end
          end
        end
      end

      if (p_bank_resp_pending_q[bank_idx]) begin
        for (int overflow_lane_idx = 0; overflow_lane_idx < CORE_COUNT_PER_FIFO; overflow_lane_idx++) begin
          if (p_bank_resp_lane_sel_q[bank_idx][overflow_lane_idx]) begin
            overflow_p_resp_valid_s[p_bank_resp_group_q[bank_idx]][overflow_lane_idx] = 1'b1;
            overflow_p_resp_data_s[p_bank_resp_group_q[bank_idx]][overflow_lane_idx] =
              p_mem_rd_data_i[bank_idx];
          end
        end
      end
    end
  end

  always_comb begin
    for (int wlvia_group_idx = 0; wlvia_group_idx < GRAD_WLVIA_COUNT; wlvia_group_idx++) begin
      for (int wlvia_lane_idx = 0; wlvia_lane_idx < SEGMENT_PER_FIFO; wlvia_lane_idx++) begin
        int global_core_idx;
        int overflow_group_idx_int;
        int overflow_lane_idx_int;

        global_core_idx = (wlvia_group_idx * SEGMENT_PER_FIFO) + wlvia_lane_idx;
        overflow_group_idx_int = global_core_idx / CORE_COUNT_PER_FIFO;
        overflow_lane_idx_int = global_core_idx % CORE_COUNT_PER_FIFO;

        wlvia_segment_idx_s[wlvia_group_idx][wlvia_lane_idx] = segment_idx_i[global_core_idx];
        wlvia_segment_type_s[wlvia_group_idx][wlvia_lane_idx] = segment_type_i[global_core_idx];
        wlvia_segment_len_s[wlvia_group_idx][wlvia_lane_idx] = segment_w_i[global_core_idx];
        wlvia_segment_valid_s[wlvia_group_idx][wlvia_lane_idx] = 1'b0;

        if (mode_wlvia_i[wlvia_group_idx] == WLVIA_MODE_COMPUTE) begin
          wlvia_segment_len_s[wlvia_group_idx][wlvia_lane_idx] =
            $signed({{(DATA_W-(ADDR_W+1)){1'b0}}, segment_span_s[global_core_idx]}) <<< FRAC_W;
          wlvia_segment_valid_s[wlvia_group_idx][wlvia_lane_idx] =
            (mode_overflow_i[overflow_group_idx_int][overflow_lane_idx_int] == OVERFLOW_MODE_SEGMENT) &&
            (segment_coeff_i[global_core_idx] != '0) &&
            (segment_type_i[global_core_idx] <= TYPE_VER_VIA);
        end
      end
    end
  end

  generate
    for (genvar wlvia_group_idx = 0; wlvia_group_idx < GRAD_WLVIA_COUNT; wlvia_group_idx++) begin : GEN_WLVIA_GROUP
      grad_wlvia #(
        .DATA_W(DATA_W),
        .FRAC_W(FRAC_W),
        .BANK_DEPTH(BANK_DEPTH),
        .FIFO_DEPTH(GRAD_WLVIA_FIFO_DEPTH),
        .SEGMENT_PER_FIFO(SEGMENT_PER_FIFO)
      ) u_grad_wlvia (
        .clk(clk),
        .rst_n(rst_n),
        .mode_i(mode_wlvia_i[wlvia_group_idx]),
        .pop_enable_i(wlvia_pop_enable_s[wlvia_group_idx]),
        .segment_idx_i(wlvia_segment_idx_s[wlvia_group_idx]),
        .segment_len_i(wlvia_segment_len_s[wlvia_group_idx]),
        .segment_type_i(wlvia_segment_type_s[wlvia_group_idx]),
        .segment_valid_i(wlvia_segment_valid_s[wlvia_group_idx]),
        .upd_idx_o(wlvia_upd_idx_s[wlvia_group_idx]),
        .upd_w_o(wlvia_upd_w_s[wlvia_group_idx]),
        .upd_valid_o(wlvia_upd_valid_s[wlvia_group_idx]),
        .fifo_empty_o(wlvia_fifo_empty_s[wlvia_group_idx]),
        .pending_push_o(wlvia_pending_push_s[wlvia_group_idx]),
        .fifo_warning_o(wlvia_fifo_warning_s[wlvia_group_idx])
      );

      assign grad_wlvia_fifo_warning_o[wlvia_group_idx] = wlvia_fifo_warning_s[wlvia_group_idx];
    end
  endgenerate

  always_comb begin
    logic all_overflow_task_finish;
    logic all_overflow_fifo_empty;
    logic all_overflow_dispatcher_idle;
    logic all_overflow_pending_push_idle;
    logic all_wlvia_fifo_empty;
    logic all_wlvia_pending_push_idle;
    logic no_source_hold_pending;
    logic no_bank_select_pending;
    logic no_grad_mem_pending;
    logic no_p_bank_resp_pending;

    all_overflow_task_finish = 1'b1;
    all_overflow_fifo_empty = 1'b1;
    all_overflow_dispatcher_idle = 1'b1;
    all_overflow_pending_push_idle = 1'b1;
    all_wlvia_fifo_empty = 1'b1;
    all_wlvia_pending_push_idle = 1'b1;
    no_source_hold_pending = 1'b1;
    no_bank_select_pending = 1'b1;
    no_grad_mem_pending = 1'b1;
    no_p_bank_resp_pending = 1'b1;

    for (int overflow_group_idx = 0; overflow_group_idx < GRAD_OVERFLOW_COUNT; overflow_group_idx++) begin
      all_overflow_fifo_empty &= overflow_fifo_empty_s[overflow_group_idx];
      for (int overflow_lane_idx = 0; overflow_lane_idx < CORE_COUNT_PER_FIFO; overflow_lane_idx++) begin
        all_overflow_task_finish &= overflow_task_finish_s[overflow_group_idx][overflow_lane_idx];
        all_overflow_dispatcher_idle &= !overflow_dispatcher_pending_s[overflow_group_idx][overflow_lane_idx];
        all_overflow_pending_push_idle &= !overflow_pending_push_s[overflow_group_idx][overflow_lane_idx];
      end
    end

    for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
      no_p_bank_resp_pending &= !p_bank_resp_pending_q[bank_idx];
    end

    for (int source_idx = 0; source_idx < TOTAL_SOURCES; source_idx++) begin
      no_source_hold_pending &= !source_hold_valid_q[source_idx];
    end

    for (int wlvia_group_idx = 0; wlvia_group_idx < GRAD_WLVIA_COUNT; wlvia_group_idx++) begin
      all_wlvia_fifo_empty &= wlvia_fifo_empty_s[wlvia_group_idx];
      for (int wlvia_lane_idx = 0; wlvia_lane_idx < SEGMENT_PER_FIFO; wlvia_lane_idx++) begin
        all_wlvia_pending_push_idle &= !wlvia_pending_push_s[wlvia_group_idx][wlvia_lane_idx];
      end
    end

    for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
      no_bank_select_pending &= !bank_select_valid_q[bank_idx];
      no_grad_mem_pending &= !grad_mem_pending_s[bank_idx];
    end

    finish_o =
      all_overflow_task_finish &&
      all_overflow_fifo_empty &&
      all_overflow_dispatcher_idle &&
      all_overflow_pending_push_idle &&
      all_wlvia_fifo_empty &&
      all_wlvia_pending_push_idle &&
      no_source_hold_pending &&
      no_bank_select_pending &&
      no_grad_mem_pending &&
      no_p_bank_resp_pending;
  end

  always_comb begin
    for (int source_idx = 0; source_idx < GRAD_OVERFLOW_COUNT; source_idx++) begin
      source_upd_idx_s[source_idx] = overflow_upd_idx_s[source_idx];
      source_upd_w_s[source_idx] = overflow_upd_w_s[source_idx];
      source_upd_valid_s[source_idx] = overflow_upd_valid_s[source_idx];
    end

    for (int source_idx = 0; source_idx < GRAD_WLVIA_COUNT; source_idx++) begin
      source_upd_idx_s[GRAD_OVERFLOW_COUNT + source_idx] = wlvia_upd_idx_s[source_idx];
      source_upd_w_s[GRAD_OVERFLOW_COUNT + source_idx] = wlvia_upd_w_s[source_idx];
      source_upd_valid_s[GRAD_OVERFLOW_COUNT + source_idx] = wlvia_upd_valid_s[source_idx];
    end

    for (int source_idx = 0; source_idx < TOTAL_SOURCES; source_idx++) begin
      source_capture_fire_s[source_idx] = 1'b0;
      source_commit_clear_s[source_idx] = 1'b0;
      source_pop_enable_s[source_idx] = 1'b0;

      if (source_upd_valid_s[source_idx]) begin
        assert (
          !$isunknown(source_upd_idx_s[source_idx].bank_idx) &&
          !$isunknown(source_upd_idx_s[source_idx].mem_idx) &&
          (source_upd_idx_s[source_idx].bank_idx < BANK_COUNT) &&
          (source_upd_idx_s[source_idx].mem_idx < BANK_DEPTH)
        ) else $fatal(
          1,
          "grad_p: source=%0d emitted out-of-range banked_idx bank=%0d mem=%0d",
          source_idx,
          source_upd_idx_s[source_idx].bank_idx,
          source_upd_idx_s[source_idx].mem_idx
        );
      end

      if (source_hold_valid_q[source_idx]) begin
        assert (
          !$isunknown(source_hold_idx_q[source_idx].bank_idx) &&
          !$isunknown(source_hold_idx_q[source_idx].mem_idx) &&
          (source_hold_idx_q[source_idx].bank_idx < BANK_COUNT) &&
          (source_hold_idx_q[source_idx].mem_idx < BANK_DEPTH)
        ) else $fatal(
          1,
          "grad_p: source_hold=%0d contains out-of-range banked_idx bank=%0d mem=%0d",
          source_idx,
          source_hold_idx_q[source_idx].bank_idx,
          source_hold_idx_q[source_idx].mem_idx
        );
      end
    end

    for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
      bank_select_valid_s[bank_idx] = 1'b0;
      bank_select_src_s[bank_idx] = '0;
      bank_select_addr_s[bank_idx] = '0;
      bank_select_delta_s[bank_idx] = fxp_zero();
      bank_select_rr_next_s[bank_idx] = bank_rr_start_q[bank_idx];

      if (!bank_select_valid_q[bank_idx]) begin
        for (int source_offset = 0; source_offset < TOTAL_SOURCES; source_offset++) begin
          int source_idx_int;

          source_idx_int = bank_rr_start_q[bank_idx] + source_offset;
          if (source_idx_int >= TOTAL_SOURCES) begin
            source_idx_int -= TOTAL_SOURCES;
          end

          if (
            !bank_select_valid_s[bank_idx] &&
            source_hold_valid_q[source_idx_int] &&
            (source_hold_idx_q[source_idx_int].bank_idx == bank_idx[BANK_IDX_W-1:0])
          ) begin
            bank_select_valid_s[bank_idx] = 1'b1;
            bank_select_src_s[bank_idx] = source_idx_int[SOURCE_IDX_W-1:0];
            bank_select_addr_s[bank_idx] = source_hold_idx_q[source_idx_int].mem_idx;
            bank_select_delta_s[bank_idx] = source_hold_w_q[source_idx_int];
            if (TOTAL_SOURCES > 1) begin
              if (source_idx_int == (TOTAL_SOURCES - 1)) begin
                bank_select_rr_next_s[bank_idx] = '0;
              end else begin
                bank_select_rr_next_s[bank_idx] = source_idx_int + 1;
              end
            end
          end
        end
      end

      bank_commit_valid_s[bank_idx] = bank_select_valid_q[bank_idx];
      bank_commit_src_s[bank_idx] = bank_select_src_q[bank_idx];
      bank_commit_addr_s[bank_idx] = bank_select_addr_q[bank_idx];
      bank_commit_delta_s[bank_idx] = bank_select_delta_q[bank_idx];
      if (bank_commit_valid_s[bank_idx]) begin
        int commit_source_idx;

        commit_source_idx = bank_commit_src_s[bank_idx];
        source_commit_clear_s[commit_source_idx] = 1'b1;
      end
    end

    for (int source_idx = 0; source_idx < TOTAL_SOURCES; source_idx++) begin
      source_capture_fire_s[source_idx] =
        source_upd_valid_s[source_idx] &&
        (!source_hold_valid_q[source_idx] || source_commit_clear_s[source_idx]);
      source_pop_enable_s[source_idx] = source_capture_fire_s[source_idx];
    end
  end

  generate
    for (genvar pop_idx = 0; pop_idx < GRAD_OVERFLOW_COUNT; pop_idx++) begin : GEN_OVERFLOW_POP
      assign overflow_pop_enable_s[pop_idx] = source_pop_enable_s[pop_idx];
    end

    for (genvar pop_idx = 0; pop_idx < GRAD_WLVIA_COUNT; pop_idx++) begin : GEN_WLVIA_POP
      assign wlvia_pop_enable_s[pop_idx] = source_pop_enable_s[GRAD_OVERFLOW_COUNT + pop_idx];
    end

    for (genvar bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin : GEN_P_BANK_REQ
      assign p_mem_rd_ce_o[bank_idx] = p_bank_req_valid_s[bank_idx];
      assign p_mem_rd_addr_o[bank_idx] = p_bank_req_addr_s[bank_idx];
    end

    for (genvar grad_mem_bank_idx = 0; grad_mem_bank_idx < BANK_COUNT; grad_mem_bank_idx++) begin : GEN_GRAD_MEM_WRITE
      assign grad_mem_pending_s[grad_mem_bank_idx] = bank_rmw_pending_q[grad_mem_bank_idx];
      assign grad_mem_rd_ce_o[grad_mem_bank_idx] = bank_commit_valid_s[grad_mem_bank_idx];
      assign grad_mem_rd_addr_o[grad_mem_bank_idx] = bank_commit_addr_s[grad_mem_bank_idx];
      assign grad_mem_wr_ce_o[grad_mem_bank_idx] = bank_rmw_pending_q[grad_mem_bank_idx];
      assign grad_mem_wr_addr_o[grad_mem_bank_idx] = bank_rmw_addr_q[grad_mem_bank_idx];
      assign grad_mem_wr_data_o[grad_mem_bank_idx] =
        bank_rmw_pending_q[grad_mem_bank_idx]
          ? (grad_mem_rd_data_i[grad_mem_bank_idx] +
             {{(ACC_W-DATA_W){bank_rmw_delta_q[grad_mem_bank_idx][DATA_W-1]}}, bank_rmw_delta_q[grad_mem_bank_idx]})
          : '0;
    end
  endgenerate

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int source_idx = 0; source_idx < TOTAL_SOURCES; source_idx++) begin
        source_hold_idx_q[source_idx] <= banked_idx_zero();
        source_hold_w_q[source_idx] <= fxp_zero();
        source_hold_valid_q[source_idx] <= 1'b0;
      end

      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        bank_rr_start_q[bank_idx] <= '0;
        bank_select_valid_q[bank_idx] <= 1'b0;
        bank_select_src_q[bank_idx] <= '0;
        bank_select_addr_q[bank_idx] <= '0;
        bank_select_delta_q[bank_idx] <= fxp_zero();
        bank_select_rr_next_q[bank_idx] <= '0;
        bank_rmw_pending_q[bank_idx] <= 1'b0;
        bank_rmw_addr_q[bank_idx] <= '0;
        bank_rmw_delta_q[bank_idx] <= fxp_zero();
        p_bank_rr_start_q[bank_idx] <= '0;
        p_bank_resp_pending_q[bank_idx] <= 1'b0;
        p_bank_resp_group_q[bank_idx] <= '0;
        p_bank_resp_lane_sel_q[bank_idx] <= '0;
      end
    end else begin
      for (int source_idx = 0; source_idx < TOTAL_SOURCES; source_idx++) begin
        if (source_capture_fire_s[source_idx]) begin
          source_hold_idx_q[source_idx] <= source_upd_idx_s[source_idx];
          source_hold_w_q[source_idx] <= source_upd_w_s[source_idx];
          source_hold_valid_q[source_idx] <= 1'b1;
        end else if (source_commit_clear_s[source_idx]) begin
          source_hold_idx_q[source_idx] <= banked_idx_zero();
          source_hold_w_q[source_idx] <= fxp_zero();
          source_hold_valid_q[source_idx] <= 1'b0;
        end
      end

      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        if (bank_select_valid_q[bank_idx]) begin
          bank_rr_start_q[bank_idx] <= bank_select_rr_next_q[bank_idx];
          bank_select_valid_q[bank_idx] <= 1'b0;
        end else begin
          bank_select_valid_q[bank_idx] <= bank_select_valid_s[bank_idx];
          if (bank_select_valid_s[bank_idx]) begin
            bank_select_src_q[bank_idx] <= bank_select_src_s[bank_idx];
            bank_select_addr_q[bank_idx] <= bank_select_addr_s[bank_idx];
            bank_select_delta_q[bank_idx] <= bank_select_delta_s[bank_idx];
            bank_select_rr_next_q[bank_idx] <= bank_select_rr_next_s[bank_idx];
          end
        end
        p_bank_rr_start_q[bank_idx] <= p_bank_rr_start_d[bank_idx];
        bank_rmw_pending_q[bank_idx] <= bank_commit_valid_s[bank_idx];
        if (bank_commit_valid_s[bank_idx]) begin
          bank_rmw_addr_q[bank_idx] <= bank_commit_addr_s[bank_idx];
          bank_rmw_delta_q[bank_idx] <= bank_commit_delta_s[bank_idx];
        end
        // one-cycle response pipeline: resp valid follows the grant by
        // exactly one (BRAM read) cycle; routing info shifts with it
        p_bank_resp_pending_q[bank_idx] <= p_bank_req_valid_s[bank_idx];
        if (p_bank_req_valid_s[bank_idx]) begin
          p_bank_resp_group_q[bank_idx] <= p_bank_req_group_s[bank_idx];
          p_bank_resp_lane_sel_q[bank_idx] <= p_bank_req_lane_sel_s[bank_idx];
        end
      end
    end
  end

  generate
    for (genvar debug_core_idx = 0; debug_core_idx < CORE_COUNT; debug_core_idx++) begin : GEN_DEBUG_CORE
      localparam int DEBUG_OVERFLOW_GROUP_IDX = debug_core_idx / CORE_COUNT_PER_FIFO;
      localparam int DEBUG_OVERFLOW_LANE_IDX = debug_core_idx % CORE_COUNT_PER_FIFO;
      assign debug_core_o[debug_core_idx] = overflow_debug_core_s[DEBUG_OVERFLOW_GROUP_IDX][DEBUG_OVERFLOW_LANE_IDX];
    end
  endgenerate

  // Merge-side trace counters (field CORE_COUNT*8 / CORE_COUNT*8+1 of
  // gradp_debug_trace_o): how many overflow-source updates the merge capture
  // accepted, and how many of them carried a nonzero upd value. The
  // increment is combinational over the per-source capture fires so
  // simultaneous captures on the same cycle all count.
  always_comb begin
    merge_overflow_capture_inc_s = '0;
    merge_overflow_nonzero_inc_s = '0;
    for (int source_idx = 0; source_idx < GRAD_OVERFLOW_COUNT; source_idx++) begin
      if (source_capture_fire_s[source_idx]) begin
        merge_overflow_capture_inc_s = merge_overflow_capture_inc_s + 1'b1;
        if (source_upd_w_s[source_idx] != '0) begin
          merge_overflow_nonzero_inc_s = merge_overflow_nonzero_inc_s + 1'b1;
        end
      end
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      merge_overflow_capture_count_q <= '0;
      merge_overflow_nonzero_count_q <= '0;
    end else begin
      merge_overflow_capture_count_q <=
        merge_overflow_capture_count_q + merge_overflow_capture_inc_s;
      merge_overflow_nonzero_count_q <=
        merge_overflow_nonzero_count_q + merge_overflow_nonzero_inc_s;
    end
  end

  always_comb begin
    gradp_debug_trace_o = '0;
    for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
      gradp_debug_trace_o[core_idx*8*32 +: 8*32] =
        overflow_debug_trace_s[core_idx / CORE_COUNT_PER_FIFO][core_idx % CORE_COUNT_PER_FIFO];
      gradp_debug_trace_o[(CORE_COUNT*8+2+core_idx*5)*32 +: 5*32] =
        overflow_debug_seg_trace_s[core_idx / CORE_COUNT_PER_FIFO][core_idx % CORE_COUNT_PER_FIFO];
    end
    gradp_debug_trace_o[CORE_COUNT*8*32 +: 32] = merge_overflow_capture_count_q;
    gradp_debug_trace_o[(CORE_COUNT*8+1)*32 +: 32] = merge_overflow_nonzero_count_q;
  end

  assign debug_bank_o = {
    source_hold_valid_q[7], bank_select_valid_q[7], bank_rmw_pending_q[7], p_bank_resp_pending_q[7],
    source_hold_valid_q[6], bank_select_valid_q[6], bank_rmw_pending_q[6], p_bank_resp_pending_q[6],
    source_hold_valid_q[5], bank_select_valid_q[5], bank_rmw_pending_q[5], p_bank_resp_pending_q[5],
    source_hold_valid_q[4], bank_select_valid_q[4], bank_rmw_pending_q[4], p_bank_resp_pending_q[4],
    source_hold_valid_q[3], bank_select_valid_q[3], bank_rmw_pending_q[3], p_bank_resp_pending_q[3],
    source_hold_valid_q[2], bank_select_valid_q[2], bank_rmw_pending_q[2], p_bank_resp_pending_q[2],
    source_hold_valid_q[1], bank_select_valid_q[1], bank_rmw_pending_q[1], p_bank_resp_pending_q[1],
    source_hold_valid_q[0], bank_select_valid_q[0], bank_rmw_pending_q[0], p_bank_resp_pending_q[0]
  };

  always_comb begin
    stall_probe_o = '0;
    for (int group_idx = 0; group_idx < GRAD_OVERFLOW_COUNT; group_idx++) begin
      for (int lane_idx = 0; lane_idx < CORE_COUNT_PER_FIFO; lane_idx++) begin
        stall_probe_o[group_idx] |=
          overflow_p_req_valid_s[group_idx][lane_idx] &&
          !overflow_p_req_grant_s[group_idx][lane_idx];
      end
    end
    for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
      stall_probe_o[4 + bank_idx] = p_bank_req_valid_s[bank_idx];
      stall_probe_o[12 + bank_idx] = bank_commit_valid_s[bank_idx];
    end
    for (int source_idx = 0; source_idx < TOTAL_SOURCES; source_idx++) begin
      stall_probe_o[20 + source_idx] =
        source_hold_valid_q[source_idx] && !source_commit_clear_s[source_idx];
    end
  end

endmodule

`endif
