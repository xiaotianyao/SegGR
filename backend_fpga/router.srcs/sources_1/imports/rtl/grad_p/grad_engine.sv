`ifndef FPGA_GRAD_ENGINE_SV
`define FPGA_GRAD_ENGINE_SV

`include "../common/segcore_pkg.sv"

module grad_engine #(
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
  parameter int MEM_GRAD_P_W = 64,
  parameter int RMS_V_W = 64,
  parameter int RMS_V_FRAC_W = (2 * FRAC_W),
  parameter logic [DATA_W-1:0] RMSPROP_ALPHA_FXP = 32'd64881,
  parameter logic [DATA_W-1:0] RMSPROP_EPS_FXP = 32'd0,
  parameter int GROUP_FLAG_WORD_W = 32,
  parameter int BANK_DEPTH = 65536,
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
  localparam int SOFTMAX_AXI_BURST_LINES_RAW = (SOFTMAX_AXI_BURST_LINES < 1) ? 1 : SOFTMAX_AXI_BURST_LINES,
  localparam int SOFTMAX_AXI_BURST_LINES_CLAMPED = (SOFTMAX_AXI_BURST_LINES_RAW > 256) ? 256 : SOFTMAX_AXI_BURST_LINES_RAW,
  localparam int SOFTMAX_ROW_READ_BUFFER_DEPTH = 8 * SOFTMAX_AXI_BURST_LINES_CLAMPED,
  localparam int SOFTMAX_AXI_BURST_ROW_SLOTS = 8 * SOFTMAX_AXI_BURST_LINES_CLAMPED,
  localparam int SOFTMAX_WRITE_FLUSH_THRESHOLD = 8 * SOFTMAX_AXI_BURST_ROW_SLOTS,
  localparam int SOFTMAX_ROW_WRITE_BUFFER_DEPTH = 2 * SOFTMAX_WRITE_FLUSH_THRESHOLD,
  // Blocked-GRADP v2 (see fpga/router/blocked_v2_rtl_design.md Chap. 1.1):
  // block-scheduled run with ping-pong grad/pwin window halves, a global
  // p window (region-base addressed) and a windowed group_flag. Only
  // meaningful together with the adapter-side BLOCKED_GRADP slot decode.
  parameter bit BLOCKED_SCHED = 1'b0,
  parameter int WIN_ROWS = 16384,       // rows of one window half per bank
  parameter int P_REGIONS = 64,         // max block count (descriptor regions)
  parameter int XWIN_DEPTH = 32768,     // xwin_p rows per bank (cross-window p cache)
  // Global DDR row index width (logits row space); forwarded to u_softmax.
  parameter int DDR_ROW_W = 20,
  localparam int GRAD_OVERFLOW_COUNT = CORE_COUNT / CORE_COUNT_PER_FIFO,
  localparam int GRAD_WLVIA_COUNT = CORE_COUNT / SEGMENT_PER_FIFO,
  localparam int BANK_COUNT = GRAD_OVERFLOW_COUNT + GRAD_WLVIA_COUNT,
  localparam int WIN_ADDR_W = BLOCKED_SCHED ? $clog2(2 * WIN_ROWS) : $clog2(BANK_DEPTH),
  localparam int WIN_ROW_W = $clog2(WIN_ROWS),
  localparam int GFLAG_WIN_WORDS = (WIN_ROWS + GROUP_FLAG_WORD_W - 1) / GROUP_FLAG_WORD_W,
  localparam int GFLAG_WIN_ADDR_W = $clog2(2 * GFLAG_WIN_WORDS),
  localparam int XWIN_ADDR_W = $clog2(XWIN_DEPTH)
) (
  input  logic                                                clk,
  input  logic                                                grad_engine_rst_n,
  input  segcore_pkg::banked_idx_t                            segment_idx_i [0:CORE_COUNT-1],
  input  logic [$clog2(CHUNK_SIZE)-1:0]                       segment_l_i [0:CORE_COUNT-1],
  input  logic [$clog2(CHUNK_SIZE)-1:0]                       segment_r_i [0:CORE_COUNT-1],
  input  logic signed [DATA_W-1:0]                            segment_w_i [0:CORE_COUNT-1],
  input  logic signed [COEFF_W-1:0]                           segment_coeff_i [0:CORE_COUNT-1],
  input  logic [2:0]                                          segment_type_i [0:CORE_COUNT-1],
  input  logic [1:0]                                          mode_overflow_i [0:(CORE_COUNT/CORE_COUNT_PER_FIFO)-1][0:CORE_COUNT_PER_FIFO-1],
  input  logic                                                mode_overflow_valid_i [0:(CORE_COUNT/CORE_COUNT_PER_FIFO)-1][0:CORE_COUNT_PER_FIFO-1],
  input  logic [2:0]                                          mode_wlvia_i [0:(CORE_COUNT/SEGMENT_PER_FIFO)-1],
  input  logic [$clog2(BANK_DEPTH + 1)-1:0]                   bank_valid_len_i [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  input  logic                                                readback_start_i,
  input  logic                                                readback_is_grad_i,
  input  logic                                                readback_req_valid_i,
  output logic                                                readback_req_ready_o,
  input  logic [$clog2(BANK_DEPTH)-1:0]                       readback_req_addr_i,
  output logic                                                readback_resp_valid_o,
  output logic signed [P_DATA_W-1:0]                          readback_p_data_o [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  output logic signed [MEM_GRAD_P_W-1:0]                      readback_grad_p_data_o [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  input  logic                                                readback_advance_i,
  output logic                                                grad_overflow_fifo_warning_o [0:(CORE_COUNT/CORE_COUNT_PER_FIFO)-1][0:CORE_COUNT_PER_FIFO-1],
  output logic                                                dispatcher_fifo_overflow_o [0:(CORE_COUNT/CORE_COUNT_PER_FIFO)-1][0:CORE_COUNT_PER_FIFO-1],
  output logic                                                grad_overflow_host_ready_o [0:(CORE_COUNT/CORE_COUNT_PER_FIFO)-1][0:CORE_COUNT_PER_FIFO-1],
  output logic                                                grad_wlvia_fifo_warning_o [0:(CORE_COUNT/SEGMENT_PER_FIFO)-1],
  output logic                                                grad_p_finish_o,
  output logic                                                grad_p_task_ready_o [0:CORE_COUNT-1],
  output logic                                                usr_irq_o,
  output logic [3:0]                                          phase_o,
  output logic                                                softmax_enable_o,
  output logic                                                softmax_finish_o [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  output logic                                                ddr_rd_req_valid_o,
  input  logic                                                ddr_rd_req_ready_i,
  output logic [DDR_ROW_W-1:0]                                ddr_rd_req_row_o,
  output logic [$clog2(SOFTMAX_ROW_READ_BUFFER_DEPTH + 1)-1:0] ddr_rd_req_count_o,
  input  logic                                                ddr_rd_resp_valid_i,
  output logic                                                ddr_rd_resp_ready_o,
  input  logic [((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1:0] ddr_rd_resp_bank_mask_i,
  input  logic                                                ddr_rd_resp_last_i,
  input  logic signed [DATA_W-1:0]                            ddr_rd_logits_i [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  input  logic signed [RMS_V_W-1:0]                           ddr_rd_rms_i [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  output logic                                                ddr_wr_valid_o,
  output logic                                                ddr_wr_p_line_o,
  input  logic                                                ddr_wr_ready_i,
  output logic [DDR_ROW_W-1:0]                                ddr_wr_row_o,
  output logic [$clog2(SOFTMAX_ROW_WRITE_BUFFER_DEPTH + 1)-1:0] ddr_wr_count_o,
  output logic signed [DATA_W-1:0]                            ddr_wr_logits_o [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  output logic signed [RMS_V_W-1:0]                           ddr_wr_rms_o [0:((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1],
  output logic [((CORE_COUNT/CORE_COUNT_PER_FIFO)+(CORE_COUNT/SEGMENT_PER_FIFO))-1:0] ddr_wr_lane_mask_o,
  // ---- Blocked-GRADP v2 interface to blocked_ctrl (BLOCKED_SCHED=1) ----
  // Per-block/per-segment context, driven by blocked_ctrl and static within
  // one block update or one pre-softmax segment.
  input  logic                                                blk_cur_half_i,
  // play-rail window half (UPD(b) || PLAY(b+1): the GRADP/play-side window
  // accesses use this half while blk_cur_half_i carries the update rail's)
  input  logic                                                blk_play_half_i,
  input  logic [$clog2(BANK_DEPTH + 1)-1:0]                   blk_bank_valid_len_i [0:BANK_COUNT-1],
  input  logic                                                blk_update_active_i,
  input  logic                                                blk_load_active_i,
  input  logic                                                blk_pre_load_done_i,
  input  logic                                                blk_iter_done_i,
  // NET-on-DDR self-run: the adapter's CMD_GO pulses this to kick the
  // phase WAIT_GRADP -> GRADP without a host readback round-trip
  input  logic                                                blk_go_i,
  // Loader: grad window write (memset/refill) and read (spill). Loader
  // addresses carry the ping-pong half bit explicitly ({half, row}).
  input  logic                                                ld_grad_win_wr_ce_i [0:BANK_COUNT-1],
  input  logic [WIN_ADDR_W-1:0]                               ld_grad_win_wr_addr_i [0:BANK_COUNT-1],
  input  logic signed [MEM_GRAD_P_W-1:0]                      ld_grad_win_wr_data_i [0:BANK_COUNT-1],
  input  logic                                                ld_grad_win_rd_req_i [0:BANK_COUNT-1],
  input  logic [WIN_ADDR_W-1:0]                               ld_grad_win_rd_addr_i [0:BANK_COUNT-1],
  output logic                                                ld_grad_win_rd_grant_o [0:BANK_COUNT-1],
  output logic signed [MEM_GRAD_P_W-1:0]                      ld_grad_win_rd_data_o [0:BANK_COUNT-1],
  // Loader: pwin write (block-load gather)
  input  logic                                                ld_pwin_wr_ce_i [0:BANK_COUNT-1],
  input  logic [WIN_ADDR_W-1:0]                               ld_pwin_wr_addr_i [0:BANK_COUNT-1],
  input  logic signed [P_DATA_W-1:0]                          ld_pwin_wr_data_i [0:BANK_COUNT-1],
  // Loader: windowed group_flag write
  input  logic                                                ld_gflag_wr_ce_i [0:BANK_COUNT-1],
  input  logic [GFLAG_WIN_ADDR_W-1:0]                         ld_gflag_wr_addr_i [0:BANK_COUNT-1],
  input  logic [GROUP_FLAG_WORD_W-1:0]                        ld_gflag_wr_data_i [0:BANK_COUNT-1],
  input  logic                                                ld_gflag_wr_special_i,
  // Loader: xwin_p read (cross-window p cache refill; data registered one
  // cycle after the request)
  input  logic                                                ld_xwin_rd_req_i [0:BANK_COUNT-1],
  input  logic [XWIN_ADDR_W-1:0]                              ld_xwin_rd_addr_i [0:BANK_COUNT-1],
  output logic signed [P_DATA_W-1:0]                          ld_xwin_rd_data_o [0:BANK_COUNT-1],
  // xwin_p region base + real-row count of the current update context
  // (from blocked_ctrl's descriptor file)
  input  logic [31:0]                                         blk_xwin_base_i [0:BANK_COUNT-1],
  input  logic [31:0]                                         blk_p_row_base_i,
  input  logic [16:0]                                         blk_real_cnt_i [0:BANK_COUNT-1],
  // v4 p write sideband (one entry per row per bank, in row order):
  // softmax_ddr aligns rows across banks, packs 2 rows per lane into
  // logits-format p lines, and issues full-line writes to the DDR p region.
  output logic                                                p_pack_valid_o [0:BANK_COUNT-1],
  input  logic                                                p_pack_ready_i [0:BANK_COUNT-1],
  output logic [$clog2(BANK_DEPTH + 1)-1:0]                   p_pack_row_o [0:BANK_COUNT-1],
  output logic signed [P_DATA_W-1:0]                          p_pack_lane_o [0:BANK_COUNT-1],
  // p sideband stage-1 vacancy (bank-engine writeback backpressure: the
  // pulse-style p write must not fire unless the demux can hold it)
  output logic                                                p_sb_free_o [0:BANK_COUNT-1],
  // Status back to blocked_ctrl
  output logic [$clog2(P_REGIONS)-1:0]                        blk_pre_seg_idx_o,
  output logic                                                blk_pre_seg_active_o,
  // actual block count N (descriptor 0x1F0 via blocked_ctrl); the blocked
  // pre-softmax scans exactly N segments instead of a fixed P_REGIONS
  input  logic [$clog2(P_REGIONS):0]                          blk_pre_seg_count_i,
  output logic                                                blk_iter_start_o,
  output logic                                                blk_softmax_all_finish_o,
  // p write session fully retired into DDR (u_softmax's p_line_asm
  // drained_o, forwarded for blocked_ctrl's update-kick gate)
  output logic                                                blk_p_asm_drained_o,
  // debug probes (self-run bring-up): [7:0]=engine local_rd_ce per bank,
  // [8]=softmax_phase_active, [9]=softmax_enable, [12:10]=softmax_ddr
  // {meta_active, reader_req_valid, reader_resp_pending},
  // [16:13]=phase_q, [24:17]=ld_xwin_rd_req_i, [32:25]=p_pack_valid_o,
  // [33]=blk_update_active_i
  output logic [33:0]                                           eng_probe_o,
  output logic                                                blk_gradp_finish_o,
  // Phantom-emit census strobe: one pulse per update-cycle p write request
  // (pre-mask); blocked_ctrl counts these per bank against n_b.
  output logic                                                blk_p_wr_strobe_o [0:BANK_COUNT-1],
  // Segment row bases (row units) for the current context, driven by
  // blocked_ctrl (seg_logits_base_o); 0 in monolithic mode.
  input  logic [DDR_ROW_W-1:0]                                blk_rd_row_base_i,
  input  logic [DDR_ROW_W-1:0]                                blk_wr_row_base_i,
  // Meta row stream passthrough (blocked_ctrl <-> bridge; arbitrated with
  // the logits/rms row stream inside u_softmax)
  input  logic                                                meta_rd_req_valid_i,
  output logic                                                meta_rd_req_ready_o,
  input  logic [DDR_ROW_W-1:0]                                meta_rd_req_row_i,
  input  logic [15:0]                                         meta_rd_req_count_i,
  output logic                                                meta_rd_resp_valid_o,
  input  logic                                                meta_rd_resp_ready_i,
  output logic                                                meta_rd_resp_last_o,
  output logic [31:0]                                         meta_rd_resp_lane_o [0:BANK_COUNT-1],
  // Row-writer phantom-discard strobes (update outputs popped beyond n_b)
  output logic                                                blk_wr_phantom_pop_o [0:BANK_COUNT-1],
  output logic [15:0]                                       gradp_debug_core_o [0:CORE_COUNT-1],
  output logic [31:0]                                       gradp_debug_bank_o,
  output logic [(CORE_COUNT*13+2)*32-1:0]                   gradp_debug_trace_o,
  output logic [31:0]                                       stall_probe_o
);
  import segcore_pkg::*;

  localparam int CORE_MEM_ADDR_W = segcore_pkg::MEM_IDX_W;
  localparam int MEM_ADDR_W = $clog2(BANK_DEPTH);
  localparam int SOFTMAX_ADDR_W = $clog2(BANK_DEPTH + 1);
  localparam int GROUP_FLAG_WORD_SHIFT = (GROUP_FLAG_WORD_W <= 1) ? 0 : $clog2(GROUP_FLAG_WORD_W);
  localparam int GROUP_FLAG_MEM_DEPTH_RAW = (BANK_DEPTH + GROUP_FLAG_WORD_W - 1) / GROUP_FLAG_WORD_W;
  localparam int GROUP_FLAG_MEM_DEPTH = (GROUP_FLAG_MEM_DEPTH_RAW < 2) ? 2 : GROUP_FLAG_MEM_DEPTH_RAW;
  localparam int GROUP_FLAG_MEM_ADDR_W = $clog2(GROUP_FLAG_MEM_DEPTH);
  // Blocked-GRADP v2: the grad/p windows are ping-pong doubled (two halves
  // of WIN_ROWS rows each); the half bit rides at the top window address
  // bit. The grad window (and pwin) are sized by WIN_ROWS, NOT BANK_DEPTH:
  // only the partition's per-block footprint lives on-chip windowed, while
  // u_mem_p keeps the full BANK_DEPTH candidate space as the global p
  // window (region-base addressed).
  localparam int WINDOW_DEPTH = BLOCKED_SCHED ? (2 * WIN_ROWS) : BANK_DEPTH;
  localparam int PRE_SEG_IDX_W = $clog2(P_REGIONS);
  // Effective pre-softmax segment count (0 = P_REGIONS fallback; matches the
  // blocked_ctrl descriptor default).
  logic [PRE_SEG_IDX_W:0] pre_seg_count_s;
  assign pre_seg_count_s = (blk_pre_seg_count_i == '0)
    ? (PRE_SEG_IDX_W+1)'(P_REGIONS)
    : blk_pre_seg_count_i;
  // Ignore softmax all_finish for this many cycles after (re-)enabling a
  // pre-softmax segment: row_reader/writer done flags from the previous
  // segment may still be set when enable rises again.
  localparam int PRE_GUARD_CYCLES = 16;
  localparam logic signed [MEM_GRAD_P_W-1:0] MEM_GRAD_P_MIN = {1'b1, {(MEM_GRAD_P_W-1){1'b0}}};
  localparam logic signed [MEM_GRAD_P_W-1:0] MEM_GRAD_P_MAX = {1'b0, {(MEM_GRAD_P_W-1){1'b1}}};

  logic grad_mem_rd_ce_grad_p_s [0:BANK_COUNT-1];
  logic [CORE_MEM_ADDR_W-1:0] grad_mem_rd_addr_grad_p_s [0:BANK_COUNT-1];
  logic signed [MEM_GRAD_P_W-1:0] grad_mem_rd_data_s [0:BANK_COUNT-1];
  logic signed [ACC_W-1:0] grad_mem_rd_data_acc_s [0:BANK_COUNT-1];
  logic grad_mem_wr_ce_s [0:BANK_COUNT-1];
  logic [CORE_MEM_ADDR_W-1:0] grad_mem_wr_addr_s [0:BANK_COUNT-1];
  logic signed [ACC_W-1:0] grad_mem_wr_data_s [0:BANK_COUNT-1];
  logic grad_mem_wr_ce_mux_s [0:BANK_COUNT-1];
  logic grad_mem_rd_ce_mux_s [0:BANK_COUNT-1];
  logic [WIN_ADDR_W-1:0] grad_mem_rd_addr_mux_s [0:BANK_COUNT-1];
  logic [WIN_ADDR_W-1:0] grad_mem_wr_addr_mux_s [0:BANK_COUNT-1];
  logic signed [MEM_GRAD_P_W-1:0] grad_mem_wr_data_mux_s [0:BANK_COUNT-1];
  // Blocked: the grad window is split into two independent 1r1w memories by
  // window half (h0/h1, WIN_ROWS each) so the UPDATE stream read of half b
  // and the GRADP RMW of half b+1 run physically in parallel (UPD(b) ||
  // PLAY(b+1) overlap). The {half, row} addressing of all consumers is
  // unchanged; only the ce demux and the read-data half select live here.
  logic grad_mem_rd_ce_h0_s [0:BANK_COUNT-1];
  logic grad_mem_rd_ce_h1_s [0:BANK_COUNT-1];
  logic grad_mem_wr_ce_h0_s [0:BANK_COUNT-1];
  logic grad_mem_wr_ce_h1_s [0:BANK_COUNT-1];
  logic [WIN_ROW_W-1:0] grad_mem_rd_row_addr_h0_s [0:BANK_COUNT-1];
  logic [WIN_ROW_W-1:0] grad_mem_rd_row_addr_h1_s [0:BANK_COUNT-1];
  logic [WIN_ROW_W-1:0] grad_mem_wr_row_addr_s [0:BANK_COUNT-1];
  logic signed [MEM_GRAD_P_W-1:0] grad_mem_rd_data_h0_s [0:BANK_COUNT-1];
  logic signed [MEM_GRAD_P_W-1:0] grad_mem_rd_data_h1_s [0:BANK_COUNT-1];
  logic grad_mem_rd_half_q [0:BANK_COUNT-1];
  // UPDATE-rail dedicated grad read channel (blocked only): drives the read
  // port of the half memory selected by blk_cur_half_i, concurrently with
  // the play channel (RMW/spill/readback/loader) on the other half. The two
  // channels can never target the same half: the update rail lags the play
  // rail by exactly one block, and adjacent blocks use opposite halves.
  logic grad_upd_rd_ce_s [0:BANK_COUNT-1];
  logic [WIN_ROW_W-1:0] grad_upd_rd_row_s [0:BANK_COUNT-1];
  logic signed [MEM_GRAD_P_W-1:0] grad_upd_rd_data_s [0:BANK_COUNT-1];

  logic signed [P_DATA_W-1:0] p_rdata_grad_p_s [0:BANK_COUNT-1];
  logic signed [P_DATA_W-1:0] p_rdata_upd_s [0:BANK_COUNT-1];
  // v4 pwin split (per-half 1r1w) ports
  logic pwin_query_ce_s [0:BANK_COUNT-1];
  logic [WIN_ROW_W-1:0] pwin_query_addr_s [0:BANK_COUNT-1];
  logic pwin_upd_ce_s [0:BANK_COUNT-1];
  logic [WIN_ROW_W-1:0] pwin_upd_addr_s [0:BANK_COUNT-1];
  logic rb_pwin_ce_s [0:BANK_COUNT-1];
  logic pwin_rd_ce_h0_s [0:BANK_COUNT-1];
  logic [WIN_ROW_W-1:0] pwin_rd_addr_h0_s [0:BANK_COUNT-1];
  logic signed [P_DATA_W-1:0] pwin_rd_data_h0_s [0:BANK_COUNT-1];
  logic pwin_rd_ce_h1_s [0:BANK_COUNT-1];
  logic [WIN_ROW_W-1:0] pwin_rd_addr_h1_s [0:BANK_COUNT-1];
  logic signed [P_DATA_W-1:0] pwin_rd_data_h1_s [0:BANK_COUNT-1];
  logic ld_pwin_wr_ce_h0_s [0:BANK_COUNT-1];
  logic ld_pwin_wr_ce_h1_s [0:BANK_COUNT-1];
  logic [WIN_ROW_W-1:0] ld_pwin_wr_addr_h_s [0:BANK_COUNT-1];
  // v4 xwin_p ports (read = loader refill; write = special demux)
  logic xwin_rd_ce_s [0:BANK_COUNT-1];
  logic [XWIN_ADDR_W-1:0] xwin_rd_addr_s [0:BANK_COUNT-1];
  logic signed [P_DATA_W-1:0] xwin_rd_data_s [0:BANK_COUNT-1];
  logic xwin_wr_ce_s [0:BANK_COUNT-1];
  logic [XWIN_ADDR_W-1:0] xwin_wr_addr_s [0:BANK_COUNT-1];
  logic signed [P_DATA_W-1:0] xwin_wr_data_s [0:BANK_COUNT-1];
  // v4 special-window read (p write demux) + demux pipeline
  logic sp_win_rd_ce_s [0:BANK_COUNT-1];
  logic [GFLAG_WIN_ADDR_W-1:0] sp_win_rd_addr_s [0:BANK_COUNT-1];
  logic [GROUP_FLAG_WORD_W-1:0] sp_rdata_s [0:BANK_COUNT-1];
  logic pwr_fire_s [0:BANK_COUNT-1];
  logic upd_or_pre_active_s;
  logic pwr_s1_valid_q [0:BANK_COUNT-1];
  logic pwr_s1_special_q [0:BANK_COUNT-1];
  logic pwr_s1_real_q [0:BANK_COUNT-1];
  logic [SOFTMAX_ADDR_W-1:0] pwr_s1_addr_q [0:BANK_COUNT-1];
  logic signed [P_DATA_W-1:0] pwr_s1_data_q [0:BANK_COUNT-1];
  logic [16:0] xwin_wr_cnt_q [0:BANK_COUNT-1];
  logic signed [GROUP_FLAG_WORD_W-1:0] group_flag_rdata_signed_s [0:BANK_COUNT-1];
  logic [GROUP_FLAG_WORD_W-1:0] group_flag_rdata_s [0:BANK_COUNT-1];
  logic p_mem_rd_ce_grad_p_s [0:BANK_COUNT-1];
  logic [CORE_MEM_ADDR_W-1:0] p_mem_rd_addr_grad_p_s [0:BANK_COUNT-1];
  // Blocked-GRADP v2: windowed group_flag read side (write side is the
  // loader port). Read latency matches the retired single-port BRAM (1 cycle).
  logic gflag_win_rd_ce_s [0:BANK_COUNT-1];
  logic [GFLAG_WIN_ADDR_W-1:0] gflag_win_rd_addr_s [0:BANK_COUNT-1];
  // Blocked-GRADP v2: pwin (ping-pong p read cache for GRADP native query)
  // Effective per-block geometry: in blocked mode blocked_ctrl swaps the
  // descriptor set per segment/block; monolithic mode keeps the CONFIG one.
  logic [SOFTMAX_ADDR_W-1:0] bank_valid_len_eff_s [0:BANK_COUNT-1];

  logic softmax_local_rd_ce_s [0:BANK_COUNT-1];
  logic [2:0] soft_dbg_reader_meta_s;
  logic [SOFTMAX_ADDR_W-1:0] softmax_local_rd_addr_s [0:BANK_COUNT-1];
  logic softmax_p_wr_valid_s [0:BANK_COUNT-1];
  logic [SOFTMAX_ADDR_W-1:0] softmax_p_wr_addr_s [0:BANK_COUNT-1];
  logic signed [P_DATA_W-1:0] softmax_p_wr_data_s [0:BANK_COUNT-1];
  logic signed [ACC_W-1:0] softmax_grad_p_i_s [0:BANK_COUNT-1];
  logic mode_overflow_active_s;
  logic any_bank_valid_len_s;
  logic softmax_phase_active_s;
  logic all_softmax_finish_s;
  logic softmax_all_finish_s;
  logic readback_resp_pending_q;
  logic readback_resp_is_grad_q;
  logic [MEM_ADDR_W-1:0] readback_resp_addr_q;
  logic readback_req_fire_s;
  logic [1:0] usr_irq_count_q;

  // ---- Blocked-GRADP v2 state ----
  // Pre-softmax segment micro-sequencer (blocked mode): one softmax pass
  // per block segment, gated by the blocked_ctrl gflag load handshake.
  typedef enum logic [1:0] {
    PS_WAIT_LOAD,
    PS_RUN,
    PS_GAP
  } pre_seq_state_e;
  pre_seq_state_e pre_state_q;
  logic [PRE_SEG_IDX_W-1:0] pre_seg_idx_q;
  logic [4:0] pre_guard_q;
  logic [1:0] pre_gap_q;
  logic blk_iter_start_q;
  // Phantom-emit guard census: count p-write requests per bank while a
  // blocked update runs; beyond n_b the write is a fifo-lag phantom and is
  // masked off (v2 keeps the M-series lesson: per-bank output count must
  // equal the block's row count).
  logic [SOFTMAX_ADDR_W-1:0] blk_p_wr_cnt_q [0:BANK_COUNT-1];
  logic blk_p_wr_allow_s [0:BANK_COUNT-1];

  typedef enum logic [3:0] {
    PHASE_PRE_SOFTMAX      = 4'd0,
    PHASE_WAIT_GRADP       = 4'd1,
    PHASE_GRADP            = 4'd2,
    PHASE_UPDATE_SOFTMAX    = 4'd3,
    PHASE_CLEAR_GRADP      = 4'd4,
    PHASE_READ_P           = 4'd5,
    PHASE_READ_GRAD_P      = 4'd6
  } system_phase_e;

  // phase_q is broadcast chip-wide as system_phase (fanout 219 measured on
  // the 100MHz timing run, 83% net delay on its paths). Force replication so
  // each consumer region gets a local copy. 64 gave one replica per bit-net
  // (69->35); 16 targets ~14 copies so each copy serves a local cluster.
  (* max_fanout = 16 *) system_phase_e phase_q;
  system_phase_e phase_n;

  assign usr_irq_o = (usr_irq_count_q != '0);
  assign phase_o = phase_q;
  assign readback_req_ready_o = !readback_resp_pending_q &&
    (((phase_q == PHASE_READ_P) && !readback_is_grad_i) ||
     ((phase_q == PHASE_READ_GRAD_P) && readback_is_grad_i));
  assign readback_req_fire_s = readback_req_valid_i && readback_req_ready_o;
  assign readback_resp_valid_o = readback_resp_pending_q;

  // Blocked-GRADP v2 status outputs
  assign blk_pre_seg_idx_o = pre_seg_idx_q;
  assign blk_pre_seg_active_o = BLOCKED_SCHED && (phase_q == PHASE_PRE_SOFTMAX);
  assign blk_iter_start_o = blk_iter_start_q;
  assign blk_softmax_all_finish_o = all_softmax_finish_s;
  // packed engine probes (see port comment for the bit map)
  always_comb begin
    eng_probe_o[33] = blk_update_active_i;
    for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
      eng_probe_o[25 + bank_idx] = p_pack_valid_o[bank_idx];
      eng_probe_o[17 + bank_idx] = ld_xwin_rd_req_i[bank_idx];
    end
    eng_probe_o[16:13] = 4'(phase_q);
    eng_probe_o[12:10] = soft_dbg_reader_meta_s;
    eng_probe_o[9] = softmax_enable_o;
    eng_probe_o[8] = softmax_phase_active_s;
    for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
      eng_probe_o[bank_idx] = softmax_local_rd_ce_s[bank_idx];
    end
  end
  assign blk_gradp_finish_o = grad_p_finish_o;
  assign ld_grad_win_rd_data_o = grad_mem_rd_data_s;

  function automatic logic signed [ACC_W-1:0] mem_grad_p_to_acc(
    input logic signed [MEM_GRAD_P_W-1:0] value
  );
    begin
      // Size cast keeps the signed type and sign-extends; unlike a replication
      // concat it is also legal when ACC_W == MEM_GRAD_P_W.
      mem_grad_p_to_acc = ACC_W'(value);
    end
  endfunction

  function automatic logic signed [MEM_GRAD_P_W-1:0] acc_to_mem_grad_p(
    input logic signed [ACC_W-1:0] value
  );
    logic signed [ACC_W-1:0] min_value;
    logic signed [ACC_W-1:0] max_value;
    begin
      min_value = ACC_W'(MEM_GRAD_P_MIN);
      max_value = ACC_W'(MEM_GRAD_P_MAX);
      if (value < min_value) begin
        acc_to_mem_grad_p = MEM_GRAD_P_MIN;
      end else if (value > max_value) begin
        acc_to_mem_grad_p = MEM_GRAD_P_MAX;
      end else begin
        acc_to_mem_grad_p = value[MEM_GRAD_P_W-1:0];
      end
    end
  endfunction

  initial begin
    assert (CORE_COUNT_PER_FIFO == 1)
      else $error(1, "grad_engine: CORE_COUNT_PER_FIFO must be 1");
    assert (SEGMENT_PER_FIFO == 1)
      else $error(1, "grad_engine: SEGMENT_PER_FIFO must be 1");
    assert (GROUP_FLAG_WORD_W >= 1)
      else $error(1, "grad_engine: GROUP_FLAG_WORD_W must be >= 1");
    assert ((GROUP_FLAG_WORD_W & (GROUP_FLAG_WORD_W - 1)) == 0)
      else $error(1, "grad_engine: GROUP_FLAG_WORD_W must be a power-of-two");
    assert (ACC_W >= MEM_GRAD_P_W)
      else $error(1, "grad_engine: ACC_W must be >= MEM_GRAD_P_W");
    assert (BANK_DEPTH <= (1 << CORE_MEM_ADDR_W))
      else $error(1, "grad_engine: BANK_DEPTH exceeds segcore_pkg::MEM_IDX_W address capacity");
    assert (!BLOCKED_SCHED || (2 * WIN_ROWS <= BANK_DEPTH))
      else $error(1, "grad_engine: grad window exceeds BANK_DEPTH");
  end

  // Phantom-emit guard census (blocked update only): the mask uses the
  // exact per-bank row count of the current block, so p writes beyond n_b
  // (fifo-lag phantoms) never reach the global p window. The pre-mask
  // strobe is exported so blocked_ctrl can census them against n_b.
  always_ff @(posedge clk or negedge grad_engine_rst_n) begin
    if (!grad_engine_rst_n) begin
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        blk_p_wr_cnt_q[bank_idx] <= '0;
        pwr_s1_valid_q[bank_idx] <= 1'b0;
        pwr_s1_real_q[bank_idx] <= 1'b0;
        pwr_s1_addr_q[bank_idx] <= '0;
        pwr_s1_data_q[bank_idx] <= '0;
        xwin_wr_cnt_q[bank_idx] <= '0;
      end
    end else begin
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        // demux stage-1: a 1-deep skid buffer, NOT a shift stage. Latch on
        // the strobe (aligned with the registered special-window read) and
        // HOLD until the sideband FIFO accepts the entry (p_pack_ready_i);
        // only then may the next write latch. A plain "valid <= fire"
        // shift stage silently drops entries whenever the FIFO is full,
        // which wedges the assembler's exact-address row alignment.
        if (pwr_fire_s[bank_idx]) begin
          pwr_s1_valid_q[bank_idx] <= 1'b1;
          pwr_s1_real_q[bank_idx] <=
            (softmax_p_wr_addr_s[bank_idx] < blk_real_cnt_i[bank_idx]);
          pwr_s1_addr_q[bank_idx] <= softmax_p_wr_addr_s[bank_idx];
          pwr_s1_data_q[bank_idx] <= softmax_p_wr_data_s[bank_idx];
        end else if (p_pack_ready_i[bank_idx]) begin
          pwr_s1_valid_q[bank_idx] <= 1'b0;
        end
        // counters: reset between update runs (shared discipline with the
        // phantom census); advance on the actual accepted p write (the
        // update-only strobe would not count pre-softmax writes, leaving
        // the phantom gate open in pre)
        if (!upd_or_pre_active_s) begin
          blk_p_wr_cnt_q[bank_idx] <= '0;
          xwin_wr_cnt_q[bank_idx] <= '0;
        end else begin
          if (pwr_fire_s[bank_idx]) begin
            blk_p_wr_cnt_q[bank_idx] <= blk_p_wr_cnt_q[bank_idx] + 1'b1;
          end
          if (xwin_wr_ce_s[bank_idx]) begin
            xwin_wr_cnt_q[bank_idx] <= xwin_wr_cnt_q[bank_idx] + 1'b1;
          end
        end
      end
    end
  end

  always_comb begin
    mode_overflow_active_s = 1'b0;
    any_bank_valid_len_s = 1'b0;
    all_softmax_finish_s = softmax_all_finish_s;

    for (int overflow_group_idx = 0; overflow_group_idx < GRAD_OVERFLOW_COUNT; overflow_group_idx++) begin
      for (int overflow_lane_idx = 0; overflow_lane_idx < CORE_COUNT_PER_FIFO; overflow_lane_idx++) begin
        mode_overflow_active_s |= (mode_overflow_i[overflow_group_idx][overflow_lane_idx] != '0);
      end
    end

    softmax_phase_active_s = grad_engine_rst_n &&
      BLOCKED_SCHED &&
      (
        ((phase_q == PHASE_PRE_SOFTMAX) && (pre_state_q == PS_RUN)) ||
        blk_update_active_i
      );
    upd_or_pre_active_s = blk_update_active_i ||
      (BLOCKED_SCHED && (phase_q == PHASE_PRE_SOFTMAX) && (pre_state_q == PS_RUN));
    softmax_enable_o = softmax_phase_active_s;

    for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
      bank_valid_len_eff_s[bank_idx] = BLOCKED_SCHED
        ? blk_bank_valid_len_i[bank_idx]
        : bank_valid_len_i[bank_idx];
      blk_p_wr_allow_s[bank_idx] =
        (blk_p_wr_cnt_q[bank_idx] < bank_valid_len_eff_s[bank_idx]);
      blk_p_wr_strobe_o[bank_idx] = BLOCKED_SCHED && blk_update_active_i &&
        softmax_p_wr_valid_s[bank_idx] &&
        (softmax_p_wr_addr_s[bank_idx] < bank_valid_len_eff_s[bank_idx]);

      any_bank_valid_len_s |= (bank_valid_len_eff_s[bank_idx] != '0);
      grad_mem_rd_data_acc_s[bank_idx] = mem_grad_p_to_acc(grad_mem_rd_data_s[bank_idx]);
      softmax_grad_p_i_s[bank_idx] = '0;

      ld_grad_win_rd_grant_o[bank_idx] = 1'b0;
      grad_upd_rd_ce_s[bank_idx] = 1'b0;
      grad_upd_rd_row_s[bank_idx] = '0;

      readback_p_data_o[bank_idx] = '0;
      readback_grad_p_data_o[bank_idx] = '0;
      if (
        readback_resp_pending_q &&
        (readback_resp_addr_q < bank_valid_len_i[bank_idx])
      ) begin
        // v4 READ_P: pwin content at {half = resp addr MSB, row} (the
        // readback pwin channel issued the read at fire; resp_pending
        // aligns the registered BRAM data)
        readback_p_data_o[bank_idx] = readback_resp_addr_q[WIN_ROW_W]
          ? pwin_rd_data_h1_s[bank_idx]
          : pwin_rd_data_h0_s[bank_idx];
      end

      // ---- grad window read ports ----
      if (BLOCKED_SCHED) begin
        // play channel: GRADP RMW has priority; loader spill reads fill
        // idle cycles. During the blocked iteration phase_q stays GRADP.
        // Drives the half selected by the address MSB (blk_play_half_i).
        if (phase_q == PHASE_GRADP) begin
          ld_grad_win_rd_grant_o[bank_idx] =
            ld_grad_win_rd_req_i[bank_idx] && !grad_mem_rd_ce_grad_p_s[bank_idx];
          grad_mem_rd_ce_mux_s[bank_idx] =
            grad_mem_rd_ce_grad_p_s[bank_idx] || ld_grad_win_rd_grant_o[bank_idx];
          grad_mem_rd_addr_mux_s[bank_idx] = grad_mem_rd_ce_grad_p_s[bank_idx]
            ? {blk_play_half_i, grad_mem_rd_addr_grad_p_s[bank_idx][WIN_ROW_W-1:0]}
            : ld_grad_win_rd_addr_i[bank_idx];
        end else begin
          // Outside GRADP the play channel is idle; the loader may read
          // freely (no RMW contention in these phases).
          ld_grad_win_rd_grant_o[bank_idx] = ld_grad_win_rd_req_i[bank_idx];
          grad_mem_rd_ce_mux_s[bank_idx] = ld_grad_win_rd_req_i[bank_idx];
          grad_mem_rd_addr_mux_s[bank_idx] = ld_grad_win_rd_addr_i[bank_idx];
        end
        // update channel: dedicated stream read into the blk_cur_half_i
        // half memory (UPD(b) || PLAY(b+1): never the same half).
        grad_upd_rd_ce_s[bank_idx] = blk_update_active_i &&
          softmax_local_rd_ce_s[bank_idx] &&
          (softmax_local_rd_addr_s[bank_idx] < bank_valid_len_eff_s[bank_idx]);
        grad_upd_rd_row_s[bank_idx] =
          softmax_local_rd_addr_s[bank_idx][WIN_ROW_W-1:0];
        softmax_grad_p_i_s[bank_idx] =
          mem_grad_p_to_acc(grad_upd_rd_data_s[bank_idx]);
      end else begin
        grad_mem_rd_ce_mux_s[bank_idx] = 1'b0;
        grad_mem_rd_addr_mux_s[bank_idx] = '0;
      end

      // ---- grad window write port ----
      if (BLOCKED_SCHED && blk_load_active_i) begin
        // Block load owns the grad window write port (memset/refill of the
        // next half); the GRADP RMW of this half never overlaps it (the
        // next block's NET is gated on load completion).
        grad_mem_wr_ce_mux_s[bank_idx] = ld_grad_win_wr_ce_i[bank_idx];
        grad_mem_wr_addr_mux_s[bank_idx] = ld_grad_win_wr_addr_i[bank_idx];
        grad_mem_wr_data_mux_s[bank_idx] = ld_grad_win_wr_data_i[bank_idx];
      end else begin
        grad_mem_wr_ce_mux_s[bank_idx] = grad_mem_wr_ce_s[bank_idx];
        if (BLOCKED_SCHED) begin
          grad_mem_wr_addr_mux_s[bank_idx] =
            {blk_play_half_i, grad_mem_wr_addr_s[bank_idx][WIN_ROW_W-1:0]};
        end else begin
          grad_mem_wr_addr_mux_s[bank_idx] = grad_mem_wr_addr_s[bank_idx][MEM_ADDR_W-1:0];
        end
        grad_mem_wr_data_mux_s[bank_idx] = acc_to_mem_grad_p(grad_mem_wr_data_s[bank_idx]);
      end

      // ---- v4 pwin read ports (per-half 1r1w; GRADP query uses the play
      // half, the update/pre-softmax local p read uses the update half;
      // they never collide: during PLAY(b+1) the two halves are distinct) --
      pwin_query_ce_s[bank_idx] = BLOCKED_SCHED && (phase_q == PHASE_GRADP) &&
        p_mem_rd_ce_grad_p_s[bank_idx];
      pwin_query_addr_s[bank_idx] = p_mem_rd_addr_grad_p_s[bank_idx][WIN_ROW_W-1:0];
      pwin_upd_ce_s[bank_idx] = BLOCKED_SCHED && softmax_phase_active_s &&
        softmax_local_rd_ce_s[bank_idx] &&
        (softmax_local_rd_addr_s[bank_idx] < bank_valid_len_eff_s[bank_idx]);
      pwin_upd_addr_s[bank_idx] = softmax_local_rd_addr_s[bank_idx][WIN_ROW_W-1:0];
      // per-half ce/addr: each half's conditions are written independently
      // (handoff pitfall: never derive one half's condition from the other).
      // The address mux must follow the UPDATE channel by cur_half+upd_ce
      // (symmetric with the ce): selecting by play_half misroutes the
      // update's p_old reads to the stale query address whenever mn_half
      // has not yet advanced to the next block's half (upd_half==play_half
      // window) -- every sample then reads one frozen row's p, breaking the
      // group gradient centering (uniform norm, p_new == p_init).
      // Third channel: READ_P readback (system idle; address carries the
      // window half in its MSB, same convention as the grad window
      // readback).
      rb_pwin_ce_s[bank_idx] = BLOCKED_SCHED && readback_req_fire_s &&
        !readback_is_grad_i;
      pwin_rd_ce_h0_s[bank_idx] =
        (!blk_play_half_i && pwin_query_ce_s[bank_idx]) ||
        (!blk_cur_half_i && pwin_upd_ce_s[bank_idx]) ||
        (rb_pwin_ce_s[bank_idx] && !readback_req_addr_i[WIN_ROW_W]);
      pwin_rd_addr_h0_s[bank_idx] =
        (rb_pwin_ce_s[bank_idx] && !readback_req_addr_i[WIN_ROW_W])
          ? readback_req_addr_i[WIN_ROW_W-1:0]
          : (
            (!blk_cur_half_i && pwin_upd_ce_s[bank_idx])
              ? pwin_upd_addr_s[bank_idx]
              : pwin_query_addr_s[bank_idx]
          );
      pwin_rd_ce_h1_s[bank_idx] =
        (blk_play_half_i && pwin_query_ce_s[bank_idx]) ||
        (blk_cur_half_i && pwin_upd_ce_s[bank_idx]) ||
        (rb_pwin_ce_s[bank_idx] && readback_req_addr_i[WIN_ROW_W]);
      pwin_rd_addr_h1_s[bank_idx] =
        (rb_pwin_ce_s[bank_idx] && readback_req_addr_i[WIN_ROW_W])
          ? readback_req_addr_i[WIN_ROW_W-1:0]
          : (
            (blk_cur_half_i && pwin_upd_ce_s[bank_idx])
              ? pwin_upd_addr_s[bank_idx]
              : pwin_query_addr_s[bank_idx]
          );
      // data out: GRADP query follows play half, the softmax p_i follows the
      // update half
      p_rdata_grad_p_s[bank_idx] = blk_play_half_i
        ? pwin_rd_data_h1_s[bank_idx]
        : pwin_rd_data_h0_s[bank_idx];
      p_rdata_upd_s[bank_idx] = blk_cur_half_i
        ? pwin_rd_data_h1_s[bank_idx]
        : pwin_rd_data_h0_s[bank_idx];
      // loader pwin write demux (ld address carries {half, row})
      ld_pwin_wr_ce_h0_s[bank_idx] =
        ld_pwin_wr_ce_i[bank_idx] && !ld_pwin_wr_addr_i[bank_idx][WIN_ROW_W];
      ld_pwin_wr_ce_h1_s[bank_idx] =
        ld_pwin_wr_ce_i[bank_idx] && ld_pwin_wr_addr_i[bank_idx][WIN_ROW_W];
      ld_pwin_wr_addr_h_s[bank_idx] = ld_pwin_wr_addr_i[bank_idx][WIN_ROW_W-1:0];

      // ---- v4 p write demux (update/pre-softmax) ----------------------------
      // stage 0: strobe -> special-window read (registered, data next cycle)
      // stage 1 (next cycle): special-real -> xwin_p write (region base +
      // running counter); everything else -> row-writer p sideband (trivial
      // real rows authoritative to the DDR p region; skip rows are dead
      // traffic into it, as in the old design)
      pwr_fire_s[bank_idx] = BLOCKED_SCHED && softmax_phase_active_s &&
        softmax_p_wr_valid_s[bank_idx] &&
        (softmax_p_wr_addr_s[bank_idx] < bank_valid_len_eff_s[bank_idx]) &&
        blk_p_wr_allow_s[bank_idx];
      sp_win_rd_ce_s[bank_idx] = pwr_fire_s[bank_idx];
      sp_win_rd_addr_s[bank_idx] = {
        blk_cur_half_i,
        softmax_p_wr_addr_s[bank_idx][WIN_ROW_W-1:GROUP_FLAG_WORD_SHIFT]
      };
      // xwin_p read port: loader xwin refill (registered SRAM read)
      xwin_rd_ce_s[bank_idx] = ld_xwin_rd_req_i[bank_idx];
      xwin_rd_addr_s[bank_idx] = ld_xwin_rd_addr_i[bank_idx];
      ld_xwin_rd_data_o[bank_idx] = xwin_rd_data_s[bank_idx];
      // xwin_p write: fires exactly once per special row, on the cycle its
      // stage-1 entry is consumed by the sideband FIFO (p_pack_ready_i).
      // Without the ready term a stalled entry would re-fire every cycle
      // and run the xwin address counter away.
      xwin_wr_ce_s[bank_idx] =
        pwr_s1_valid_q[bank_idx] && pwr_s1_real_q[bank_idx] &&
        p_pack_ready_i[bank_idx] &&
        sp_rdata_s[bank_idx][pwr_s1_addr_q[bank_idx][GROUP_FLAG_WORD_SHIFT-1:0]];
      xwin_wr_addr_s[bank_idx] = XWIN_ADDR_W'(
        blk_xwin_base_i[bank_idx][XWIN_ADDR_W-1:0] + xwin_wr_cnt_q[bank_idx]
      );
      xwin_wr_data_s[bank_idx] = pwr_s1_data_q[bank_idx];
      // p sideband to softmax_ddr's p line assembler: ONE ENTRY PER ROW in
      // window row order, special rows included (the assembler aligns banks
      // by exact row address, so a skipped row would deadlock it). Special
      // rows therefore also land in the DDR p region as a non-authoritative
      // duplicate; the authoritative copy is the xwin_p write above, and the
      // loader's XREFILL overwrites the pwin from xwin_p on every window
      // load, so the DDR duplicate is never consumed.
      p_pack_valid_o[bank_idx] = pwr_s1_valid_q[bank_idx];
      p_pack_row_o[bank_idx] = pwr_s1_addr_q[bank_idx];
      p_pack_lane_o[bank_idx] = pwr_s1_data_q[bank_idx];
      // stage-1 vacancy for the bank-engine writeback backpressure: the s1
      // register can accept a new entry (empty, or its current entry is
      // being consumed into the sideband FIFO this cycle)
      p_sb_free_o[bank_idx] =
        !pwr_s1_valid_q[bank_idx] || p_pack_ready_i[bank_idx];

      // ---- group_flag read port ----
      // Windowed group_flag (ping-pong, loader-written).
      gflag_win_rd_ce_s[bank_idx] = softmax_phase_active_s &&
        softmax_local_rd_ce_s[bank_idx] &&
        (softmax_local_rd_addr_s[bank_idx] < bank_valid_len_eff_s[bank_idx]);
      gflag_win_rd_addr_s[bank_idx] = {
        blk_cur_half_i,
        softmax_local_rd_addr_s[bank_idx][WIN_ROW_W-1:GROUP_FLAG_WORD_SHIFT]
      };
      group_flag_rdata_s[bank_idx] = group_flag_rdata_signed_s[bank_idx];
    end
    all_softmax_finish_s &= any_bank_valid_len_s;
  end

  always_ff @(posedge clk or negedge grad_engine_rst_n) begin
    if (!grad_engine_rst_n) begin
      // Blocked mode has no CLEAR_GRADP phase (memset moved into the block
      // load); the initial monolithic clear is skipped as well.
      phase_q <= PHASE_PRE_SOFTMAX;
      readback_resp_pending_q <= 1'b0;
      readback_resp_is_grad_q <= 1'b0;
      readback_resp_addr_q <= '0;
      usr_irq_count_q <= '0;
      pre_state_q <= PS_WAIT_LOAD;
      pre_seg_idx_q <= '0;
      pre_guard_q <= '0;
      pre_gap_q <= '0;
      blk_iter_start_q <= 1'b0;
    end else begin
      phase_n = phase_q;
      unique case (phase_q)
        PHASE_PRE_SOFTMAX: begin
          if (BLOCKED_SCHED) begin
            // 16-segment sub-scan: one softmax pass per block segment.
            // blocked_ctrl loads the segment's group_flag window half
            // (blk_pre_load_done_i) before each pass; p read/write use the
            // segment's global p window region; no DDR writeback.
            unique case (pre_state_q)
              PS_WAIT_LOAD: begin
                if (blk_pre_load_done_i) begin
                  pre_state_q <= PS_RUN;
                  pre_guard_q <= 5'(PRE_GUARD_CYCLES);
                end
              end
              PS_RUN: begin
                if (pre_guard_q != '0) begin
                  pre_guard_q <= pre_guard_q - 1'b1;
                end else if (all_softmax_finish_s) begin
                  // scan exactly the configured number of block segments
                  if ({1'b0, pre_seg_idx_q} == (pre_seg_count_s - 5'd1)) begin
                    phase_n = PHASE_WAIT_GRADP;
                    pre_seg_idx_q <= '0;
                    pre_state_q <= PS_WAIT_LOAD;
                  end else begin
                    pre_seg_idx_q <= pre_seg_idx_q + 1'b1;
                    pre_state_q <= PS_GAP;
                    pre_gap_q <= 2'd3;
                  end
                end
              end
              PS_GAP: begin
                // A few enable-low cycles so the row_reader/writer done
                // flags of the finished segment clear before the next run.
                if (pre_gap_q != '0) begin
                  pre_gap_q <= pre_gap_q - 1'b1;
                end else begin
                  pre_state_q <= PS_WAIT_LOAD;
                end
              end
              default: begin
                pre_state_q <= PS_WAIT_LOAD;
              end
            endcase
          end
        end
        PHASE_WAIT_GRADP: begin
          if (BLOCKED_SCHED && blk_go_i) begin
            // NET-on-DDR self-run launch (adapter CMD_GO)
            phase_n = PHASE_GRADP;
          end else if (readback_start_i && !readback_is_grad_i) begin
            phase_n = PHASE_READ_P;
          end else if (mode_overflow_active_s) begin
            phase_n = PHASE_GRADP;
          end
        end
        PHASE_READ_P: begin
          if (readback_advance_i && !readback_is_grad_i) begin
            phase_n = PHASE_GRADP;
          end
        end
        PHASE_GRADP: begin
          if (BLOCKED_SCHED) begin
            // Blocked: the whole per-iteration block loop (load/gradp/
            // spill/update per block) is sequenced by blocked_ctrl while
            // phase_q stays GRADP; no UPDATE_SOFTMAX/CLEAR_GRADP and no
            // READ_GRAD_P (blocked readback is READ_P of the global p
            // window only).
            if (blk_iter_done_i) begin
              phase_n = PHASE_WAIT_GRADP;
            end
          end
        end
        default: begin
          phase_n = phase_q;
        end
      endcase
      phase_q <= phase_n;

      // New-iteration pulse for blocked_ctrl (phase entered GRADP)
      blk_iter_start_q <=
        BLOCKED_SCHED && (phase_n == PHASE_GRADP) && (phase_q != PHASE_GRADP);

      readback_resp_pending_q <= readback_req_fire_s;
      if (readback_req_fire_s) begin
        readback_resp_is_grad_q <= readback_is_grad_i;
        readback_resp_addr_q <= readback_req_addr_i;
      end

      if ((phase_n == PHASE_WAIT_GRADP) && (phase_q != PHASE_WAIT_GRADP)) begin
        usr_irq_count_q <= 2'd2;
      end else if (usr_irq_count_q != '0) begin
        usr_irq_count_q <= usr_irq_count_q - 1'b1;
      end

    end
  end

  grad_p #(
    .CORE_COUNT(CORE_COUNT),
    .CHUNK_SIZE(CHUNK_SIZE),
    .CORE_COUNT_PER_FIFO(CORE_COUNT_PER_FIFO),
    .SEGMENT_PER_FIFO(SEGMENT_PER_FIFO),
    .DATA_W(DATA_W),
    .FRAC_W(FRAC_W),
    .COEFF_W(COEFF_W),
    .COEFF_FRAC_W(COEFF_FRAC_W),
    .ACC_W(ACC_W),
    .BANK_DEPTH(BANK_DEPTH),
    .DISPATCHER_FIFO_DEPTH(DISPATCHER_FIFO_DEPTH),
    .GRAD_OVERFLOW_FIFO_DEPTH(GRAD_OVERFLOW_FIFO_DEPTH),
    .GRAD_WLVIA_FIFO_DEPTH(GRAD_WLVIA_FIFO_DEPTH),
    .NATIVE_QUERY_ENABLE(NATIVE_QUERY_ENABLE),
    .QUERY_PREFETCH_FIFO_DEPTH(QUERY_PREFETCH_FIFO_DEPTH),
    .QUERY_ACT_SCALE_FXP(QUERY_ACT_SCALE_FXP),
    .ACT_GROUP(ACT_GROUP),
    .ACT_PIPELINE(ACT_PIPELINE),
    .PRE_ACT_IDLE_CYCLES(PRE_ACT_IDLE_CYCLES),
    .POST_ACT_IDLE_CYCLES(POST_ACT_IDLE_CYCLES)
  ) u_grad_p (
    .clk(clk),
    .rst_n(grad_engine_rst_n),
    .segment_idx_i(segment_idx_i),
    .segment_l_i(segment_l_i),
    .segment_r_i(segment_r_i),
    .segment_w_i(segment_w_i),
    .segment_coeff_i(segment_coeff_i),
    .segment_type_i(segment_type_i),
    .grad_overflow_host_ready_o(grad_overflow_host_ready_o),
    .mode_overflow_i(mode_overflow_i),
    .mode_overflow_valid_i(mode_overflow_valid_i),
    .mode_wlvia_i(mode_wlvia_i),
    .grad_overflow_fifo_warning_o(grad_overflow_fifo_warning_o),
    .dispatcher_fifo_overflow_o(dispatcher_fifo_overflow_o),
    .grad_wlvia_fifo_warning_o(grad_wlvia_fifo_warning_o),
    .finish_o(grad_p_finish_o),
    .task_ready_o(grad_p_task_ready_o),
    .grad_mem_rd_ce_o(grad_mem_rd_ce_grad_p_s),
    .grad_mem_rd_addr_o(grad_mem_rd_addr_grad_p_s),
    .grad_mem_rd_data_i(grad_mem_rd_data_acc_s),
    .grad_mem_wr_ce_o(grad_mem_wr_ce_s),
    .grad_mem_wr_addr_o(grad_mem_wr_addr_s),
    .grad_mem_wr_data_o(grad_mem_wr_data_s),
    .p_mem_rd_ce_o(p_mem_rd_ce_grad_p_s),
    .p_mem_rd_addr_o(p_mem_rd_addr_grad_p_s),
    .p_mem_rd_data_i(p_rdata_grad_p_s),
    .debug_core_o(gradp_debug_core_o),
    .debug_bank_o(gradp_debug_bank_o),
    .gradp_debug_trace_o(gradp_debug_trace_o),
    .stall_probe_o(stall_probe_o)
  );

  generate
    if (BLOCKED_SCHED) begin : GEN_GRAD_WIN_SPLIT
      // grad window split by window half: two independent 1r1w memories.
      // Parallelism (see module plan): the UPDATE stream reads half b (R of
      // one mem) while the GRADP RMW of half b+1 uses R+W of the other; the
      // block loader writes half b+2 (= half b) W while UPDATE reads half b
      // R (1r1w simultaneous, and its kick is gated on the UPDATE retire);
      // spill of b+1 reads half b+1 R after the RMW of b+1 has drained.
      for (genvar bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin : GEN_GRAD_HALF
        // two channels into two half memories: the update channel owns the
        // blk_cur_half_i half, the play channel owns the address-MSB half.
        // Adjacent blocks use opposite halves, so they never collide.
        assign grad_mem_rd_ce_h0_s[bank_idx] =
          (grad_mem_rd_ce_mux_s[bank_idx] &&
           !grad_mem_rd_addr_mux_s[bank_idx][WIN_ADDR_W-1]) ||
          (blk_update_active_i && !blk_cur_half_i && grad_upd_rd_ce_s[bank_idx]);
        assign grad_mem_rd_ce_h1_s[bank_idx] =
          (grad_mem_rd_ce_mux_s[bank_idx] &&
           grad_mem_rd_addr_mux_s[bank_idx][WIN_ADDR_W-1]) ||
          (blk_update_active_i && blk_cur_half_i && grad_upd_rd_ce_s[bank_idx]);
        assign grad_mem_rd_row_addr_h0_s[bank_idx] =
          (blk_update_active_i && !blk_cur_half_i && grad_upd_rd_ce_s[bank_idx])
            ? grad_upd_rd_row_s[bank_idx]
            : grad_mem_rd_addr_mux_s[bank_idx][WIN_ROW_W-1:0];
        assign grad_mem_rd_row_addr_h1_s[bank_idx] =
          (blk_update_active_i && blk_cur_half_i && grad_upd_rd_ce_s[bank_idx])
            ? grad_upd_rd_row_s[bank_idx]
            : grad_mem_rd_addr_mux_s[bank_idx][WIN_ROW_W-1:0];
        assign grad_mem_wr_ce_h0_s[bank_idx] =
          grad_mem_wr_ce_mux_s[bank_idx] &&
          !grad_mem_wr_addr_mux_s[bank_idx][WIN_ADDR_W-1];
        assign grad_mem_wr_ce_h1_s[bank_idx] =
          grad_mem_wr_ce_mux_s[bank_idx] &&
          grad_mem_wr_addr_mux_s[bank_idx][WIN_ADDR_W-1];
        assign grad_mem_wr_row_addr_s[bank_idx] =
          grad_mem_wr_addr_mux_s[bank_idx][WIN_ROW_W-1:0];
        assign grad_mem_rd_data_s[bank_idx] = grad_mem_rd_half_q[bank_idx]
          ? grad_mem_rd_data_h1_s[bank_idx]
          : grad_mem_rd_data_h0_s[bank_idx];
        assign grad_upd_rd_data_s[bank_idx] = blk_cur_half_i
          ? grad_mem_rd_data_h1_s[bank_idx]
          : grad_mem_rd_data_h0_s[bank_idx];
        always_ff @(posedge clk) begin
          if (grad_mem_rd_ce_mux_s[bank_idx]) begin
            grad_mem_rd_half_q[bank_idx] <=
              grad_mem_rd_addr_mux_s[bank_idx][WIN_ADDR_W-1];
          end
        end
`ifndef SYNTHESIS
        always_ff @(posedge clk) begin
          if (grad_engine_rst_n && blk_update_active_i && grad_upd_rd_ce_s[bank_idx] &&
              grad_mem_rd_ce_mux_s[bank_idx] &&
              (grad_mem_rd_addr_mux_s[bank_idx][WIN_ADDR_W-1] == blk_cur_half_i)) begin
            $fatal(1, "grad window read-port collision: play and update channels hit the same half (bank %0d)", bank_idx);
          end
        end
`endif
      end

      sram_1r1w #(
        .DATA_W(MEM_GRAD_P_W),
        .BANK_COUNT(BANK_COUNT),
        .BANK_DEPTH(WIN_ROWS)
      ) u_mem_grad_p_h0 (
        .clk(clk),
        .rd_ce_i(grad_mem_rd_ce_h0_s),
        .rd_addr_i(grad_mem_rd_row_addr_h0_s),
        .rd_data_o(grad_mem_rd_data_h0_s),
        .wr_ce_i(grad_mem_wr_ce_h0_s),
        .wr_addr_i(grad_mem_wr_row_addr_s),
        .wr_data_i(grad_mem_wr_data_mux_s)
      );

      sram_1r1w #(
        .DATA_W(MEM_GRAD_P_W),
        .BANK_COUNT(BANK_COUNT),
        .BANK_DEPTH(WIN_ROWS)
      ) u_mem_grad_p_h1 (
        .clk(clk),
        .rd_ce_i(grad_mem_rd_ce_h1_s),
        .rd_addr_i(grad_mem_rd_row_addr_h1_s),
        .rd_data_o(grad_mem_rd_data_h1_s),
        .wr_ce_i(grad_mem_wr_ce_h1_s),
        .wr_addr_i(grad_mem_wr_row_addr_s),
        .wr_data_i(grad_mem_wr_data_mux_s)
      );
    end
  endgenerate

  generate
    if (BLOCKED_SCHED) begin : GEN_BLOCKED_MEM
      // v4: u_mem_p (global p window) retired - trivial p is authoritative in
      // the DDR p region, cross-window p in xwin_p (below).
      // pwin: ping-pong p read cache, split into two independent 1r1w halves
      // (same pattern as the grad window): the GRADP query port and the
      // update-rail p read port each own one half's read port, so PLAY(b+1)
      // and UPD(b) run physically in parallel. The loader writes the loading
      // half (defer gate orders it after the conflicting update retires).
      sram_1r1w #(
        .DATA_W(P_DATA_W),
        .BANK_COUNT(BANK_COUNT),
        .BANK_DEPTH(WIN_ROWS)
      ) u_mem_pwin_h0 (
        .clk(clk),
        .rd_ce_i(pwin_rd_ce_h0_s),
        .rd_addr_i(pwin_rd_addr_h0_s),
        .rd_data_o(pwin_rd_data_h0_s),
        .wr_ce_i(ld_pwin_wr_ce_h0_s),
        .wr_addr_i(ld_pwin_wr_addr_h_s),
        .wr_data_i(ld_pwin_wr_data_i)
      );
      sram_1r1w #(
        .DATA_W(P_DATA_W),
        .BANK_COUNT(BANK_COUNT),
        .BANK_DEPTH(WIN_ROWS)
      ) u_mem_pwin_h1 (
        .clk(clk),
        .rd_ce_i(pwin_rd_ce_h1_s),
        .rd_addr_i(pwin_rd_addr_h1_s),
        .rd_data_o(pwin_rd_data_h1_s),
        .wr_ce_i(ld_pwin_wr_ce_h1_s),
        .wr_addr_i(ld_pwin_wr_addr_h_s),
        .wr_data_i(ld_pwin_wr_data_i)
      );

      // xwin_p: cross-window p cache (authoritative for candidates whose
      // group finalizes in a later block). Read by the block loader's xwin
      // refill; written by the update/pre-softmax special-p demux.
      sram_1r1w #(
        .DATA_W(P_DATA_W),
        .BANK_COUNT(BANK_COUNT),
        .BANK_DEPTH(XWIN_DEPTH)
      ) u_mem_xwin_p (
        .clk(clk),
        .rd_ce_i(xwin_rd_ce_s),
        .rd_addr_i(xwin_rd_addr_s),
        .rd_data_o(xwin_rd_data_s),
        .wr_ce_i(xwin_wr_ce_s),
        .wr_addr_i(xwin_wr_addr_s),
        .wr_data_i(xwin_wr_data_s)
      );

      // Windowed group_flag: two independent 1r1w windows per bank
      // (gs window: group-start bits; special window: cross-window marks),
      // each 2 ping-pong halves x WIN_ROWS/32 words. Independent read ports
      // let the engine's gs chain and the p-write special demux proceed in
      // parallel. Registered reads preserve the 1-cycle BRAM latency.
      for (genvar bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin : GEN_GFLAG_WIN
        (* ram_style = "block" *) logic [GROUP_FLAG_WORD_W-1:0] gs_win_q [0:(2*GFLAG_WIN_WORDS)-1];
        (* ram_style = "block" *) logic [GROUP_FLAG_WORD_W-1:0] sp_win_q [0:(2*GFLAG_WIN_WORDS)-1];
        logic [GROUP_FLAG_WORD_W-1:0] gflag_rdata_q;
        logic [GROUP_FLAG_WORD_W-1:0] sp_rdata_q;

        always_ff @(posedge clk) begin
          if (ld_gflag_wr_ce_i[bank_idx] && !ld_gflag_wr_special_i) begin
            gs_win_q[ld_gflag_wr_addr_i[bank_idx]] <= ld_gflag_wr_data_i[bank_idx];
          end
          if (ld_gflag_wr_ce_i[bank_idx] && ld_gflag_wr_special_i) begin
            sp_win_q[ld_gflag_wr_addr_i[bank_idx]] <= ld_gflag_wr_data_i[bank_idx];
          end
          if (gflag_win_rd_ce_s[bank_idx]) begin
            gflag_rdata_q <= gs_win_q[gflag_win_rd_addr_s[bank_idx]];
          end
          if (sp_win_rd_ce_s[bank_idx]) begin
            sp_rdata_q <= sp_win_q[sp_win_rd_addr_s[bank_idx]];
          end
        end

        assign group_flag_rdata_signed_s[bank_idx] = gflag_rdata_q;
        assign sp_rdata_s[bank_idx] = sp_rdata_q;
      end
    end
  endgenerate

  softmax_ddr #(
    .BANK_COUNT(BANK_COUNT),
    .BANK_DEPTH(BANK_DEPTH),
    .DATA_W(DATA_W),
    .FRAC_W(FRAC_W),
    .P_DATA_W(P_DATA_W),
    .P_FRAC_W(P_FRAC_W),
    .ACC_W(ACC_W),
    .GROUP_FLAG_WORD_W(GROUP_FLAG_WORD_W),
    .RMS_V_W(RMS_V_W),
    .RMSPROP_ALPHA_FXP(RMSPROP_ALPHA_FXP),
    .RMSPROP_EPS_FXP(RMSPROP_EPS_FXP),
    .BURST_LINES(SOFTMAX_AXI_BURST_LINES),
    .BLOCKED_SCHED(BLOCKED_SCHED),
    .DDR_ROW_W(DDR_ROW_W)
  ) u_softmax (
    .clk(clk),
    .rst_n(grad_engine_rst_n),
    .enable_i(softmax_enable_o),
    .update_mode_i(blk_update_active_i),
    .p_asm_active_i(
      blk_update_active_i ||
      ((phase_q == PHASE_PRE_SOFTMAX) && (pre_state_q == PS_RUN))
    ),
    .p_i(p_rdata_upd_s),
    .grad_p_i(softmax_grad_p_i_s),
    .group_start_flag_word_i(group_flag_rdata_s),
    .bank_valid_len_i(bank_valid_len_eff_s),
    .rd_row_base_i(BLOCKED_SCHED ? blk_rd_row_base_i : DDR_ROW_W'(0)),
    .wr_row_base_i(BLOCKED_SCHED ? blk_wr_row_base_i : DDR_ROW_W'(0)),
    .meta_rd_req_valid_i(BLOCKED_SCHED ? meta_rd_req_valid_i : 1'b0),
    .meta_rd_req_ready_o(meta_rd_req_ready_o),
    .meta_rd_req_row_i(meta_rd_req_row_i),
    .meta_rd_req_count_i(meta_rd_req_count_i[$clog2(SOFTMAX_ROW_READ_BUFFER_DEPTH + 1)-1:0]),
    .meta_rd_resp_valid_o(meta_rd_resp_valid_o),
    .meta_rd_resp_ready_i(BLOCKED_SCHED ? meta_rd_resp_ready_i : 1'b0),
    .meta_rd_resp_last_o(meta_rd_resp_last_o),
    .meta_rd_resp_lane_o(meta_rd_resp_lane_o),
    .p_row_base_i(blk_p_row_base_i),
    .p_pack_valid_i(p_pack_valid_o),
    .p_pack_ready_o(p_pack_ready_i),
    .p_pack_row_i(p_pack_row_o),
    .p_pack_lane_i(p_pack_lane_o),
    .p_sb_ready_i(p_sb_free_o),
    .phantom_pop_o(blk_wr_phantom_pop_o),
    .local_rd_ce_o(softmax_local_rd_ce_s),
    .local_rd_addr_o(softmax_local_rd_addr_s),
    .p_wr_valid_o(softmax_p_wr_valid_s),
    .p_wr_addr_o(softmax_p_wr_addr_s),
    .p_wr_data_o(softmax_p_wr_data_s),
    .ddr_rd_req_valid_o(ddr_rd_req_valid_o),
    .ddr_rd_req_ready_i(ddr_rd_req_ready_i),
    .ddr_rd_req_row_o(ddr_rd_req_row_o),
    .ddr_rd_req_count_o(ddr_rd_req_count_o),
    .ddr_rd_resp_valid_i(ddr_rd_resp_valid_i),
    .ddr_rd_resp_ready_o(ddr_rd_resp_ready_o),
    .ddr_rd_resp_bank_mask_i(ddr_rd_resp_bank_mask_i),
    .ddr_rd_resp_last_i(ddr_rd_resp_last_i),
    .ddr_rd_logits_i(ddr_rd_logits_i),
    .ddr_rd_rms_i(ddr_rd_rms_i),
    .ddr_wr_valid_o(ddr_wr_valid_o),
    .ddr_wr_p_line_o(ddr_wr_p_line_o),
    .ddr_wr_ready_i(ddr_wr_ready_i),
    .ddr_wr_row_o(ddr_wr_row_o),
    .ddr_wr_count_o(ddr_wr_count_o),
    .ddr_wr_logits_o(ddr_wr_logits_o),
    .ddr_wr_rms_o(ddr_wr_rms_o),
    .ddr_wr_lane_mask_o(ddr_wr_lane_mask_o),
    .finish_o(softmax_finish_o),
    .all_finish_o(softmax_all_finish_s),
    .dbg_reader_meta_o(soft_dbg_reader_meta_s),
    .p_asm_drained_o(blk_p_asm_drained_o)
  );

`ifndef SYNTHESIS
  // ---- NET-on-DDR self-run debug probes (sim only) ----
  // Periodic GRADP-phase drain status: which grad_p lanes still have
  // unfinished tasks / pending work when the drain refuses to complete.
  logic [19:0] eng_gradp_dbg_cnt_q;
  always_ff @(posedge clk) begin
    if (grad_engine_rst_n && BLOCKED_SCHED && (phase_q == PHASE_GRADP)) begin
      eng_gradp_dbg_cnt_q <= eng_gradp_dbg_cnt_q + 1'b1;
      if (eng_gradp_dbg_cnt_q == 20'd30000) begin
        eng_gradp_dbg_cnt_q <= '0;
        $display("%t ENG GRADP status finish=%0d bank=%h", $time, grad_p_finish_o, gradp_debug_bank_o);
        for (int core_idx = 0; core_idx < CORE_COUNT; core_idx++) begin
          $display("%t ENG GRADP core=%0d task_rdy=%0d host_rdy=%0d dbg=%h",
            $time, core_idx, grad_p_task_ready_o[core_idx],
            grad_overflow_host_ready_o[core_idx / CORE_COUNT_PER_FIFO][core_idx % CORE_COUNT_PER_FIFO],
            gradp_debug_core_o[core_idx]);
        end
      end
    end else begin
      eng_gradp_dbg_cnt_q <= '0;
    end
  end

  // per-bank FIFO drain snapshot, genvar-tapped (a runtime index into
  // GEN_BANK is illegal at vopt); printed by the periodic probe below.
  // bit legend [27:0]: in_done, fetch_pend, held, empty{in,addr,log,rms,
  // p,grad,rdesc,flen,raw,rnorm,rmsnext,rmswb,sqrtctx,sqrtres,gdivctx,
  // norm,lognew,logwb,pnew,recipctx}, vld{gdiv,cordic,recip},
  // sent{divid,divis}
  logic [27:0] predbg_bank_flags_q [0:7];
  logic [3:0]  predbg_bank_collect_q [0:7];
  logic [15:0] predbg_in_pop_q [0:7];
  logic [15:0] predbg_sample_push_q [0:7];
  logic [15:0] predbg_desc_push_q [0:7];
  logic [15:0] predbg_desc_len_sum_q [0:7];
  logic [15:0] predbg_raw_pop_q [0:7];
  logic [15:0] predbg_flush_push_q [0:7];
  logic [31:0] predbg_gs_h1w0_q [0:7];
  logic [31:0] predbg_gs_h1w1_q [0:7];
  logic [16:0] predbg_pwc_q [0:7];
  logic        predbg_s1v_q [0:7];
  logic        predbg_prdy_q [0:7];
  logic        predbg_fire_q [0:7];
  logic        prepwr_valid_q [0:7];
  logic        predbg_allow_q [0:7];
  for (genvar gb = 0; gb < 8; gb++) begin : GEN_PREDBG_TAP
    always_ff @(posedge clk) begin
      predbg_pwc_q[gb] <= blk_p_wr_cnt_q[gb];
      predbg_s1v_q[gb] <= pwr_s1_valid_q[gb];
      predbg_prdy_q[gb] <= p_pack_ready_i[gb];
      predbg_fire_q[gb] <= pwr_fire_s[gb];
      prepwr_valid_q[gb] <= softmax_p_wr_valid_s[gb];
      predbg_allow_q[gb] <= blk_p_wr_allow_s[gb];
      predbg_gs_h1w0_q[gb] <= GEN_BLOCKED_MEM.GEN_GFLAG_WIN[gb].gs_win_q[GFLAG_WIN_WORDS + 0];
      predbg_gs_h1w1_q[gb] <= GEN_BLOCKED_MEM.GEN_GFLAG_WIN[gb].gs_win_q[GFLAG_WIN_WORDS + 1];
      predbg_in_pop_q[gb] <=
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.dbg_in_pop_cnt_q;
      predbg_sample_push_q[gb] <=
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.dbg_sample_push_cnt_q;
      predbg_desc_push_q[gb] <=
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.dbg_desc_push_cnt_q;
      predbg_desc_len_sum_q[gb] <=
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.dbg_desc_len_sum_q;
      predbg_raw_pop_q[gb] <=
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.dbg_raw_pop_cnt_q;
      predbg_flush_push_q[gb] <=
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.dbg_flush_push_cnt_q;
      predbg_bank_flags_q[gb] <= {
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.input_done_i,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.fetch_pending_q,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.held_valid_q,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.in_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.addr_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.logits_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.rms_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.p_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.grad_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.raw_desc_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.final_len_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.raw_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.raw_norm_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.rms_next_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.rms_wb_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.sqrt_context_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.sqrt_result_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.grad_div_context_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.norm_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.logits_new_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.logits_wb_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.p_new_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.recip_context_fifo_empty_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.grad_div_m_tvalid_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.cordic_m_tvalid_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.recip_m_tvalid_s,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.grad_dividend_sent_q,
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.grad_divisor_sent_q
      };
      predbg_bank_collect_q[gb] <=
        u_softmax.GEN_BANK[gb].u_softmax_bank_engine.collect_count_q;
    end
  end

  // ---- pre-softmax stall debug (sim only): periodic finish breakdown
  // while a segment plays, plus state-change pulses ----
  logic [15:0] eng_pre_dbg_cnt_q;
  pre_seq_state_e eng_pre_dbg_state_q;
  always_ff @(posedge clk) begin
    if (!grad_engine_rst_n) begin
      eng_pre_dbg_cnt_q <= '0;
      eng_pre_dbg_state_q <= PS_WAIT_LOAD;
    end else begin
      if (pre_state_q != eng_pre_dbg_state_q) begin
        $display("%t PREDBG state %0d -> %0d seg=%0d phase=%0d asm[rdyln=%0d head=%0d maxln=%0d burst=%b left=%0d fin=%b] b7[occ=%0d trow=%0d] b7pwc=%0d",
          $time, eng_pre_dbg_state_q, pre_state_q, pre_seg_idx_q, phase_q,
          u_softmax.u_p_line_asm.ready_lines_s,
          u_softmax.u_p_line_asm.head_line_q,
          u_softmax.u_p_line_asm.session_max_lines_s,
          u_softmax.u_p_line_asm.burst_active_q,
          u_softmax.u_p_line_asm.burst_left_q,
          u_softmax.u_p_line_asm.final_s,
          u_softmax.u_p_line_asm.occupancy_s[7],
          u_softmax.u_p_line_asm.tail_row_q[7],
          blk_p_wr_cnt_q[7]);
        $display("%t PREDBG asmdbg limit=%0d vl7=%0d vl0=%0d upd_act=%b",
          $time,
          u_softmax.u_p_line_asm.ready_limit_s,
          u_softmax.u_p_line_asm.valid_len_sess_q[7],
          u_softmax.u_p_line_asm.valid_len_sess_q[0],
          u_softmax.u_p_line_asm.update_active_i);
        eng_pre_dbg_state_q <= pre_state_q;
      end
      if (BLOCKED_SCHED && (phase_q == PHASE_PRE_SOFTMAX)) begin
        eng_pre_dbg_cnt_q <= eng_pre_dbg_cnt_q + 1'b1;
        if (eng_pre_dbg_cnt_q == 16'd1024) begin
          eng_pre_dbg_cnt_q <= '0;
          $display("%t PREDBG seg=%0d allfin=%b rd_done=%b wr_done=%b meta_act=%b rd_req=%b rd_pend=%b asm[rdyln=%0d head=%0d maxln=%0d burst=%b left=%0d fin=%b] fin[7:0]=%b%b%b%b%b%b%b%b",
            $time, pre_seg_idx_q, all_softmax_finish_s,
            u_softmax.reader_done_s, u_softmax.writer_done_s,
            u_softmax.owner_head_meta_s, u_softmax.reader_req_valid_s,
            u_softmax.reader_resp_pending_s,
            u_softmax.u_p_line_asm.ready_lines_s,
            u_softmax.u_p_line_asm.head_line_q,
            u_softmax.u_p_line_asm.session_max_lines_s,
            u_softmax.u_p_line_asm.burst_active_q,
            u_softmax.u_p_line_asm.burst_left_q,
            u_softmax.u_p_line_asm.final_s,
            u_softmax.finish_o[7], u_softmax.finish_o[6],
            u_softmax.finish_o[5], u_softmax.finish_o[4],
            u_softmax.finish_o[3], u_softmax.finish_o[2],
            u_softmax.finish_o[1], u_softmax.finish_o[0]);
          // per-FIFO drain breakdown for any unfinished bank (flags legend
          // at GEN_PREDBG_TAP)
          for (int b = 0; b < 8; b++) begin
            if (!u_softmax.finish_o[b]) begin
              $display("%t PREDBG bank%0d: collect=%0d flags[27:0]=%b",
                $time, b, predbg_bank_collect_q[b], predbg_bank_flags_q[b]);
              $display("%t PREDBG bank%0d: in_pop=%0d sample_push=%0d desc_push=%0d desc_len_sum=%0d raw_pop=%0d flush_push=%0d",
                $time, b, predbg_in_pop_q[b], predbg_sample_push_q[b],
                predbg_desc_push_q[b], predbg_desc_len_sum_q[b],
                predbg_raw_pop_q[b], predbg_flush_push_q[b]);
              $display("%t PREDBG bank%0d: gs_h1w0=%h gs_h1w1=%h",
                $time, b, predbg_gs_h1w0_q[b], predbg_gs_h1w1_q[b]);
              $display("%t PREDBG bank%0d: p_wr_cnt=%0d s1v=%b prdy=%b pasm[occ=%0d trow=%0d]",
                $time, b, predbg_pwc_q[b], predbg_s1v_q[b], predbg_prdy_q[b],
                u_softmax.u_p_line_asm.occupancy_s[b],
                u_softmax.u_p_line_asm.tail_row_q[b]);
              $display("%t PREDBG bank%0d: fire=%b p_wr_valid=%b allow=%b vleneff=%0d paddr=%0d",
                $time, b, predbg_fire_q[b], prepwr_valid_q[b], predbg_allow_q[b],
                bank_valid_len_eff_s[b], softmax_p_wr_addr_s[b]);
            end
          end
        end
      end else begin
        eng_pre_dbg_cnt_q <= '0;
      end
    end
  end
`endif

endmodule

`endif
