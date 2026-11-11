`ifndef FPGA_BLOCKED_CTRL_SV
`define FPGA_BLOCKED_CTRL_SV

// Blocked-GRADP v2 block controller. See fpga/router/blocked_v2_rtl_design.md
// (Chap. 1.2) and routing/plan_blocked_v2.md for the architecture contract.
//
// Owns per-iteration block orchestration:
//   ctrl lane BRAM (per-block refill/spill lanes, streamed by the adapter
//   via CMD_LOAD_BLOCK_CTRL) / carry BRAM (cross-block partial gradients)
//   / block descriptor register file (host-written at init), and drives:
//     - pre-softmax service: per-segment meta fetch -> group_flag window
//     - block load:  window memset -> p gather (pwin) -> gflag -> refill
//     - block end:   spill (window -> carry)
//     - block update: context swap (region base / n_b / half) + run/phantom
//                     census while grad_engine streams the block's segment
//
// Window/slot addressing: slot = {row[16:3], bank[2:0]}; window addresses
// are {half, row} with the half bit prepended here. p_global_idx is the
// 17b authoritative position of a candidate inside its bank's global p
// window (upper bits of the 20b meta field reserved).
//
// The loader (block b+1) overlaps the update of block b (ping-pong halves);
// its refill stage is gated on block b's spill completion because a carry
// entry spilled at block b may be refilled into block b+1 (slot-ordered
// carry regions are disjoint per block, but the write must retire first).

module blocked_ctrl #(
  parameter int BANK_COUNT = 8,
  parameter int P_REGIONS = 64,
  parameter int WIN_ROWS = 16384,
  parameter int CARRY_DEPTH = 65536,
  parameter int CTRL_HALF_LANES = 16384,
  parameter int P_IDX_W = 17,
  parameter int XWIN_DEPTH = 32768,     // xwin_p rows per bank (cross-window p cache)
  parameter int META_LANE_W = 32,     // stream lane width (p halves / gs / special)
  parameter int ROW_W = 20,           // DDR row address width
  parameter int SLOT_W = 17,
  parameter int DDR_COUNT_W = 16,
  parameter int P_DATA_W = 16,
  parameter int MEM_GRAD_P_W = 64,
  parameter int GROUP_FLAG_WORD_W = 32,
  localparam int WIN_ROW_W = $clog2(WIN_ROWS),
  localparam int WIN_ADDR_W = WIN_ROW_W + 1,
  localparam int CARRY_ADDR_W = $clog2(CARRY_DEPTH),
  localparam int XWIN_ADDR_W = $clog2(XWIN_DEPTH),
  localparam int CTRL_LANE_W = $clog2(CTRL_HALF_LANES),
  localparam int CTRL_ADDR_W = CTRL_LANE_W + 1,
  localparam int GFLAG_WORDS = WIN_ROWS / GROUP_FLAG_WORD_W,
  localparam int GFLAG_ADDR_W = $clog2(2 * GFLAG_WORDS),
  localparam int BANK_IDX_W = $clog2(BANK_COUNT),
  localparam int BLK_IDX_W = $clog2(P_REGIONS),
  localparam int GFLAG_WORD_SHIFT = $clog2(GROUP_FLAG_WORD_W)
) (
  input  logic                          clk,
  input  logic                          rst_n,

  // ---- self-run control (NET-on-DDR iteration master) ----
  // GO pulse + iteration count from the adapter's CMD_GO; the master then
  // drives the whole block loop for go_iters_i iterations with zero host
  // interaction and replays the same DDR-resident image every iteration.
  input  logic                          go_i,
  input  logic [31:0]                   go_iters_i,
  // engine phase (4'd2 = GRADP); the first NET play is gated on it
  input  logic [3:0]                    system_phase_i,

  // ---- ctrl lane stream (DDR prefetch via net_fetcher), FWFT ----
  // Consumers pop strictly in image order (block hdr -> refill -> spill);
  // a pop registers the head lane, matching the retired ctrl BRAM's
  // one-cycle read latency.
  input  logic                          cf_lane_valid_i,
  input  logic [63:0]                   cf_lane_data_i,
  output logic                          cf_lane_pop_o,

  // ---- descriptor register file (host init, word writes) ----
  input  logic                          desc_wr_valid_i,
  input  logic [10:0]                   desc_wr_addr_i,
  input  logic [31:0]                   desc_wr_data_i,
  // descriptor comb read (adapter CMD_INIT_SEG base lookup)
  input  logic [10:0]                   desc_rd_addr_i,
  output logic [31:0]                   desc_rd_data_o,

  // ---- block/iteration events (adapter self-run walk) ----
  input  logic                          block_done_i,   // pulse: current block NET drained

  // ---- self-run NET play request (to the adapter self-run walk) ----
  output logic                          sr_play_o,      // level: play this block's NET
  output logic                          sr_play_last_o, // this block is the iteration's last
  input  logic                          sr_play_ack_i,  // adapter accepted the play request

  // ---- self-run status (to the adapter debug/WAIT paths) ----
  output logic                          sr_done_o,      // latched: N iterations completed
  output logic                          sr_running_o,   // a GO run is in progress
  output logic [31:0]                   sr_iter_cnt_o,  // completed iteration count
  output logic [7:0]                    sr_error_o,     // [0] = phantom census anomaly
  output logic [5:0]                    sr_cur_blk_o,   // current block index (progress)
  output logic [2:0]                    sr_mn_state_o,  // main FSM state (progress)
  // update-rail status (performance mirror): [0]=upd_active [6:1]=upd_blk
  // [7]=upd_half [8]=upd_pend_valid [9]=0
  output logic [9:0]                    sr_upd_probe_o,
  output logic [3:0]                    sr_ld_state_o,  // load FSM state (probe)
  output logic [5:0]                    sr_ld_blk_o,    // block being loaded (probe)
  output logic                          sr_ld_done_valid_o, // load-done flag pending (probe)
  output logic [16:0]                   sr_ld_row_cnt_o,    // loader row counter (probe)
  // gather/meta handshake probes: [0]=meta_req_valid [1]=meta_req_ready
  // [2]=meta_resp_valid [3]=row_hold_valid [11:4]=gath_pend [19:12]=gath_done
  output logic [19:0]                   sr_ld_probe2_o,

  // ---- status to adapter / router ----
  output logic                          spill_done_o,   // pulse: block spill retired
  output logic                          iter_done_o,    // pulse: last block update drained
  output logic                          cur_half_o,
  // play-rail window half for the grad engine's play-side accesses
  // (UPD(b) || PLAY(b+1): cur_half_o carries the update rail's half)
  output logic                          play_half_o,

  // ---- grad_engine context / handshake ----
  input  logic [BLK_IDX_W-1:0]          pre_seg_idx_i,
  input  logic                          pre_seg_active_i,
  output logic                          pre_load_done_o,
  // host-driven debug window load (READ_P v4): full-service load of an
  // arbitrary block into an explicit half, outside the iteration (system
  // idle, so no defer/half-alternation concerns)
  input  logic                          dbg_load_kick_i,
  input  logic [BLK_IDX_W-1:0]        dbg_load_blk_i,
  input  logic                        dbg_load_half_i,
  // actual block count N for the pre-softmax segment scan (descriptor 0x400)
  output logic [BLK_IDX_W:0]            pre_seg_count_o,
  output logic [16:0]                   real_cnt_o [0:BANK_COUNT-1],
  output logic [31:0]                   xwin_base_o [0:BANK_COUNT-1],
  output logic [16:0]                   bank_valid_len_o [0:BANK_COUNT-1],
  output logic                          engine_update_active_o,
  output logic                          engine_load_active_o,
  output logic                          engine_iter_done_o,
  input  logic                          softmax_all_finish_i,
  // p write session fully retired into DDR (p_line_asm drained): the next
  // update kick must wait for it, because the assembler restarts its
  // session on the update_active edge and a restart with lines still
  // buffered silently discards them
  input  logic                          p_asm_drained_i,
  input  logic                          p_wr_strobe_i [0:BANK_COUNT-1],
  // row-writer phantom-discard strobes (entries popped beyond n_b)
  input  logic                          wr_phantom_pop_i [0:BANK_COUNT-1],
  // phantom census of the last update (pre-mask p-write requests per bank)
  output logic                          upd_phantom_seen_o,

  // ---- loader ports into grad_engine memories ----
  output logic                          ld_grad_win_wr_ce_o [0:BANK_COUNT-1],
  output logic [WIN_ADDR_W-1:0]         ld_grad_win_wr_addr_o [0:BANK_COUNT-1],
  output logic signed [MEM_GRAD_P_W-1:0] ld_grad_win_wr_data_o [0:BANK_COUNT-1],
  output logic                          ld_grad_win_rd_req_o [0:BANK_COUNT-1],
  output logic [WIN_ADDR_W-1:0]         ld_grad_win_rd_addr_o [0:BANK_COUNT-1],
  input  logic                          ld_grad_win_rd_grant_i [0:BANK_COUNT-1],
  input  logic signed [MEM_GRAD_P_W-1:0] ld_grad_win_rd_data_i [0:BANK_COUNT-1],
  output logic                          ld_pwin_wr_ce_o [0:BANK_COUNT-1],
  output logic [WIN_ADDR_W-1:0]         ld_pwin_wr_addr_o [0:BANK_COUNT-1],
  output logic signed [P_DATA_W-1:0]    ld_pwin_wr_data_o [0:BANK_COUNT-1],
  output logic                          ld_gflag_wr_ce_o [0:BANK_COUNT-1],
  output logic [GFLAG_ADDR_W-1:0]       ld_gflag_wr_addr_o [0:BANK_COUNT-1],
  output logic [GROUP_FLAG_WORD_W-1:0]  ld_gflag_wr_data_o [0:BANK_COUNT-1],
  // gflag write target select: 0 = gs window, 1 = special window
  output logic                          ld_gflag_wr_special_o,
  // loader read port into the xwin_p store (cross-window p cache; read data
  // registered one cycle after the request, like the carry BRAM)
  output logic                          ld_xwin_rd_req_o [0:BANK_COUNT-1],
  output logic [XWIN_ADDR_W-1:0]        ld_xwin_rd_addr_o [0:BANK_COUNT-1],
  input  logic signed [P_DATA_W-1:0]    ld_xwin_rd_data_i [0:BANK_COUNT-1],

  // ---- meta segment DDR read stream (to row_reader/bridge) ----
  output logic                          meta_rd_req_valid_o,
  input  logic                          meta_rd_req_ready_i,
  output logic [ROW_W-1:0]              meta_rd_req_row_o,
  output logic [DDR_COUNT_W-1:0]        meta_rd_req_count_o,
  input  logic                          meta_rd_resp_valid_i,
  output logic                          meta_rd_resp_ready_o,
  input  logic [META_LANE_W-1:0]        meta_rd_resp_lane_i [0:BANK_COUNT-1],

  // ---- current context DDR segment descriptors (row_reader/writer) ----
  output logic [31:0]                   p_base_o,
  output logic [31:0]                   seg_logits_base_o
);

  // ---------------------------------------------------------------------
  // Control lane format (64b lanes, one block per ping-pong half)
  //   lane 0 (hdr):  {n_refill[15:0], n_spill[31:16], n_xwin[47:32],
  //                   real_rows_max[63:48]}
  //   xwin_p refill: {xwin_idx[31:0], slot[63:32]}  (slot ascending)
  //   grad refill:   {carry_addr[15:0], slot[33:17]}   (carry_addr ascending)
  //   grad spill:    {carry_addr[33:17], slot[16:0]}   (slot ascending;
  //                  carry write address rides the lane, host interval-packed)
  //   window_rows_max lives in the descriptor table (seg_rows_o of the
  //   block, see the desc map below)
  // ---------------------------------------------------------------------

  typedef enum logic [2:0] {
    MN_IDLE,
    MN_WAIT_LOAD,
    MN_PLAY,
    MN_SPILL,
    MN_DONE
  } main_state_e;

  typedef enum logic [3:0] {
    LD_IDLE,
    LD_HDR_RD,
    LD_HDR_CAP,
    LD_PBULK_REQ,
    LD_PBULK,
    LD_GS_REQ,
    LD_GS,
    LD_SP_REQ,
    LD_SP,
    LD_XREFILL,
    LD_REFILL,
    LD_DONE
  } load_state_e;

  main_state_e mn_state_q;
  load_state_e ld_state_q;

  logic [BLK_IDX_W-1:0] blk_idx_q;
  logic [BLK_IDX_W-1:0] ld_blk_q;
  logic ld_pre_mode_q;
  logic ld_dbg_mode_q;  // current load is a host debug load (READ_P v4)
  // Ping-pong half tracking follows the load order: loads are strictly
  // sequential (one per block in execution order), so the half of the next
  // load simply toggles. This keeps the halves aligned for any block count
  // N (odd or even) and across the iteration wrap.
  logic ld_next_half_q;
  logic ld_cur_half_q;
  logic ld_done_half_q;
  logic mn_half_q;
  logic ld_half_s;
  logic [BLK_IDX_W-1:0] spill_done_blk_q;
  logic spill_done_valid_q;

  // self-run master state
  logic        ld_adopted_q;     // block's load adopted in MN_WAIT_LOAD
  logic        sr_done_q;
  logic        sr_running_q;
  logic        sr_phantom_err_q;
  logic [31:0] iter_cnt_q;
  logic [31:0] go_iters_q;

  // load kick handshake (main FSM -> load FSM)
  logic ld_kick_valid_q;
  logic [BLK_IDX_W-1:0] ld_kick_blk_q;

  // block header (captured by the load FSM; the main FSM copies the spill
  // geometry before the next block's header read may overwrite it)
  logic [15:0] hdr_n_refill_q;
  logic [15:0] hdr_n_spill_q;
  logic [15:0] hdr_n_xwin_q;
  logic [15:0] mn_n_spill_q;

  // load-done / pre-load-done bookkeeping
  logic ld_done_valid_q;
  logic [BLK_IDX_W-1:0] ld_done_blk_q;
  logic pre_loaded_valid_q;
  logic [BLK_IDX_W-1:0] pre_loaded_idx_q;

  // p bulk / bitstream fetch: the bridge read channel serves one request at
  // a time and the request count field is bounded, so every region stream is
  // fetched in 64-line chunks (interleaves with the overlapped update's
  // logits/rms reads).
  localparam int STREAM_CHUNK_LINES = 64;

  // stream request bookkeeping (shared by PBULK / GS / SP stages)
  logic [16:0] st_line_q;       // current line index within the region
  logic [16:0] st_lines_q;      // total lines of this stream
  logic [16:0] st_chunk_end_q;  // next chunk boundary

  // PBULK beat hold (2 unpack writes per resp beat)
  logic        pb_hold_valid_q;
  logic        pb_phase_q;
  logic [31:0] pb_hold_lanes_q [0:BANK_COUNT-1];

  // memset
  logic [16:0] memset_row_q;
  logic memset_done_q;

  // refill pipeline: s0 ctrl read -> s1 carry read -> s2 window write
  logic [15:0] refill_launch_q;
  logic rf_s1_valid_q;
  logic rf_s2_valid_q;
  logic [SLOT_W-1:0] rf_s2_slot_q;

  // xwin refill pipeline: s0 ctrl read -> s1 xwin_p read -> s2 pwin write
  logic [15:0] xw_launch_q;
  logic xw_s1_valid_q;
  logic xw_s2_valid_q;
  logic [SLOT_W-1:0] xw_s2_slot_q;

  // spill pipeline: s0 ctrl read -> s1 window read -> s2 carry write
  logic [15:0] sp_launch_q;
  logic sp_s1_valid_q;
  logic sp_s2_valid_q;
  logic [SLOT_W-1:0] sp_s2_slot_q;
  logic [CARRY_ADDR_W-1:0] sp_s2_addr_q;

  // update guard/census
  logic [4:0] upd_guard_q;
  logic [16:0] upd_p_cnt_q [0:BANK_COUNT-1];
  logic upd_phantom_seen_q;

  // ---- update rail (UPD(b) overlaps the play rail's PLAY(b+1)) ----
  // The play rail (MN_IDLE/WAIT_LOAD/PLAY/SPILL) kicks an update when the
  // block's spill retires and never waits for it; the update rail lags by
  // exactly one block, so the two rails always use opposite window halves.
  typedef enum logic [0:0] {
    UPD_IDLE,
    UPD_ACTIVE
  } upd_state_e;
  upd_state_e upd_state_q;
  logic [BLK_IDX_W-1:0] upd_blk_q;
  logic upd_half_q;
  // one-deep pending slot for a spill that retires while the rail is busy
  logic upd_pend_valid_q;
  logic [BLK_IDX_W-1:0] upd_pend_blk_q;
  logic upd_pend_half_q;
  // play-rail iteration counter (drives final_blk_s); sr_iter_cnt_o counts
  // update-rail retirements (= completed iterations)
  logic [31:0] play_iter_cnt_q;
  // load kick deferred while the target half is still read by the update rail
  logic ld_kick_defer_q;
  logic [BLK_IDX_W-1:0] ld_kick_defer_blk_q;
  // set once the update's engine has actually gone busy (all_finish fell)
  // since the update start; retire waits for real activity, not just the
  // stale all_finish of the previous segment (bridge contention under the
  // UPD(b) || PLAY(b+1) overlap can delay the engine start past the old
  // fixed 16-cycle guard, producing zero-write updates)
  logic upd_seen_busy_q;

  // ---------------------------------------------------------------------
  // Descriptor register file (host init). 11-bit word address map
  // (P_REGIONS = 64; host constants in verif/router_host.py
  // BLOCKED_V2_DESC_* must match this map exactly):
  //   11'h000-11'h1FF  real_cnt[{block,bank}]        (17b)
  //   11'h200-11'h3FF  n_b[{block,bank}]             (17b)
  //   11'h400          block_count (actual blocks N, 1..P_REGIONS; 0 = P_REGIONS)
  //   11'h440-11'h47F  seg_logits_base[block]        (32b)
  //   11'h480-11'h4BF  seg_rms_base[block]           (32b)
  //   11'h4C0-11'h4FF  seg_meta_base[block]          (32b)
  //   11'h500-11'h53F  seg_rows_max[block]           (17b)
  //   11'h540-11'h57F  p_base[block]                 (32b, v4: p region line base)
  //   11'h580-11'h5BF  gs_base[block]                (32b, v4: gs bitstream base)
  //   11'h5C0-11'h5FF  sp_base[block]                (32b, v4: special bitstream base)
  //   11'h600-11'h7FF  xwin_base[{block,bank}]       (32b, v4: xwin_p region base)
  // ---------------------------------------------------------------------
  logic [16:0] desc_real_cnt_q [0:P_REGIONS*BANK_COUNT-1];
  logic [16:0] desc_n_b_q [0:P_REGIONS*BANK_COUNT-1];
  logic [31:0] desc_seg_logits_base_q [0:P_REGIONS-1];
  logic [31:0] desc_seg_rms_base_q [0:P_REGIONS-1];
  logic [31:0] desc_seg_meta_base_q [0:P_REGIONS-1];
  logic [16:0] desc_seg_rows_q [0:P_REGIONS-1];
  logic [31:0] desc_p_base_q [0:P_REGIONS-1];
  logic [31:0] desc_gs_base_q [0:P_REGIONS-1];
  logic [31:0] desc_sp_base_q [0:P_REGIONS-1];
  logic [31:0] desc_xwin_base_q [0:P_REGIONS*BANK_COUNT-1];
  logic [BLK_IDX_W:0] desc_block_count_q;

  // NOTE: no reset on this block by design - the host writes the descriptor
  // image (including the block count word) via CMD_LOAD_BLOCK_DESC while the
  // system reset hold keeps rst_n low; gating the writes with rst_n would
  // drop them. The engine is held in the same reset until after the image
  // lands, so the unwritten state is never sampled.
  always_ff @(posedge clk) begin
    if (desc_wr_valid_i) begin
      if (desc_wr_addr_i == 11'h400) begin
        // block count N lives at 0x400 (the retired carry_base slot); this
        // special case must precede the [10:9]==2'b10 class decode, which
        // would otherwise swallow it into the casez default
        desc_block_count_q <= desc_wr_data_i[BLK_IDX_W:0];
      end else if (desc_wr_addr_i[10:9] == 2'b00) begin
        desc_real_cnt_q[desc_wr_addr_i[8:0]] <= desc_wr_data_i[16:0];
      end else if (desc_wr_addr_i[10:9] == 2'b01) begin
        desc_n_b_q[desc_wr_addr_i[8:0]] <= desc_wr_data_i[16:0];
      end else if (desc_wr_addr_i[10:9] == 2'b10) begin
        casez (desc_wr_addr_i[8:6])
          3'b001: desc_seg_logits_base_q[desc_wr_addr_i[5:0]] <= desc_wr_data_i;
          3'b010: desc_seg_rms_base_q[desc_wr_addr_i[5:0]] <= desc_wr_data_i;
          3'b011: desc_seg_meta_base_q[desc_wr_addr_i[5:0]] <= desc_wr_data_i;
          3'b100: desc_seg_rows_q[desc_wr_addr_i[5:0]] <= desc_wr_data_i[16:0];
          3'b101: desc_p_base_q[desc_wr_addr_i[5:0]] <= desc_wr_data_i;
          3'b110: desc_gs_base_q[desc_wr_addr_i[5:0]] <= desc_wr_data_i;
          3'b111: desc_sp_base_q[desc_wr_addr_i[5:0]] <= desc_wr_data_i;
          default: ;
        endcase
      end else if (desc_wr_addr_i[10:9] == 2'b11) begin
        desc_xwin_base_q[desc_wr_addr_i[8:0]] <= desc_wr_data_i;
      end
    end
  end

  // effective block count N (descriptor 0x400; 0 = P_REGIONS fallback,
  // matches pre_seg_count_o)
  logic [BLK_IDX_W:0] block_count_s;
  assign block_count_s =
    (desc_block_count_q == '0) ? (BLK_IDX_W+1)'(P_REGIONS) : desc_block_count_q;

  // last block of the run's last iteration: no further load is kicked and
  // its update drain completes the run (play-rail view)
  logic final_blk_s;
  assign final_blk_s =
    ({1'b0, blk_idx_q} + 1'b1 >= block_count_s) &&
    (play_iter_cnt_q + 1'b1 >= go_iters_q);

  // ---------------------------------------------------------------------
  // carry BRAM (small on-chip array, registered read). The ctrl lane
  // stream now comes from the DDR image via the net_fetcher lane FIFO;
  // a pop registers the head lane, matching the retired ctrl BRAM's
  // one-cycle read latency, so all downstream pipelines are unchanged.
  // ---------------------------------------------------------------------
  (* ram_style = "block" *) logic [63:0] carry_mem_q [0:CARRY_DEPTH-1];

  logic ctrl_pop_s;
  logic [63:0] ctrl_rd_data_q;
  logic carry_rd_ce_s;
  logic [CARRY_ADDR_W-1:0] carry_rd_addr_s;
  logic [63:0] carry_rd_data_q;
  logic carry_wr_ce_s;
  logic [CARRY_ADDR_W-1:0] carry_wr_addr_s;
  logic [63:0] carry_wr_data_s;

  assign cf_lane_pop_o = ctrl_pop_s;

  always_ff @(posedge clk) begin
    if (ctrl_pop_s) begin
      ctrl_rd_data_q <= cf_lane_data_i;
    end
  end

  always_ff @(posedge clk) begin
    if (carry_wr_ce_s) begin
      carry_mem_q[carry_wr_addr_s] <= carry_wr_data_s;
    end
    if (carry_rd_ce_s) begin
      carry_rd_data_q <= carry_mem_q[carry_rd_addr_s];
    end
  end

  // ---------------------------------------------------------------------
  // Current context (pre-softmax segment or iteration block) descriptor mux.
  // With the dual-rail overlap the update-side context follows the update
  // rail's block (upd_blk_q); the play rail only needs its half bit
  // (play_half_o), all other context outputs are update-rail exclusive.
  // ---------------------------------------------------------------------
  logic [BLK_IDX_W-1:0] ctx_idx_s;
  assign ctx_idx_s = pre_seg_active_i ? pre_seg_idx_i : upd_blk_q;
  // engine window half: pre-softmax scans ping-pong by segment order; the
  // update rail's half serves the update-side window accesses
  assign cur_half_o = pre_seg_active_i ? pre_seg_idx_i[0] : upd_half_q;
  assign play_half_o = pre_seg_active_i ? pre_seg_idx_i[0] : mn_half_q;
  // actual block count for the pre-softmax segment scan (0 = P_REGIONS)
  assign pre_seg_count_o =
    (desc_block_count_q == '0) ? (BLK_IDX_W+1)'(P_REGIONS) : desc_block_count_q;

  always_comb begin
    for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
      real_cnt_o[bank_idx] = desc_real_cnt_q[{ctx_idx_s, BANK_IDX_W'(bank_idx)}];
      xwin_base_o[bank_idx] = desc_xwin_base_q[{ctx_idx_s, BANK_IDX_W'(bank_idx)}];
      bank_valid_len_o[bank_idx] = desc_n_b_q[{ctx_idx_s, BANK_IDX_W'(bank_idx)}];
    end
  end
  assign p_base_o = desc_p_base_q[ctx_idx_s];
  assign seg_logits_base_o = desc_seg_logits_base_q[ctx_idx_s];

  assign pre_load_done_o =
    pre_loaded_valid_q && (pre_loaded_idx_q == pre_seg_idx_i);
  assign upd_phantom_seen_o = upd_phantom_seen_q;

  // ---------------------------------------------------------------------
  // ctrl lane pop arbitration: spill (main) > header (load) > refill
  // (load), gated on the fetcher lane FIFO being non-empty. The image
  // order is hdr -> refill -> spill per block, and the consumption order
  // matches (spill(b) retires before hdr(b+1) is needed: the next load's
  // refill stage is gated on the previous spill anyway, and the header pop
  // itself only gates the meta fetch).
  // ---------------------------------------------------------------------
  // descriptor comb read (adapter INIT_SEG base lookup; same map as writes)
  always_comb begin
    if (desc_rd_addr_i == 11'h400) begin
      // same precedence caveat as the write decode: 0x400 must be matched
      // before the [10:9]==2'b10 class
      desc_rd_data_o = {{(27-BLK_IDX_W){1'b0}}, desc_block_count_q};
    end else if (desc_rd_addr_i[10:9] == 2'b00) begin
      desc_rd_data_o = {15'b0, desc_real_cnt_q[desc_rd_addr_i[8:0]]};
    end else if (desc_rd_addr_i[10:9] == 2'b01) begin
      desc_rd_data_o = {15'b0, desc_n_b_q[desc_rd_addr_i[8:0]]};
    end else if (desc_rd_addr_i[10:9] == 2'b10) begin
      casez (desc_rd_addr_i[8:6])
        3'b001: desc_rd_data_o = desc_seg_logits_base_q[desc_rd_addr_i[5:0]];
        3'b010: desc_rd_data_o = desc_seg_rms_base_q[desc_rd_addr_i[5:0]];
        3'b011: desc_rd_data_o = desc_seg_meta_base_q[desc_rd_addr_i[5:0]];
        3'b100: desc_rd_data_o = {15'b0, desc_seg_rows_q[desc_rd_addr_i[5:0]]};
        3'b101: desc_rd_data_o = desc_p_base_q[desc_rd_addr_i[5:0]];
        3'b110: desc_rd_data_o = desc_gs_base_q[desc_rd_addr_i[5:0]];
        3'b111: desc_rd_data_o = desc_sp_base_q[desc_rd_addr_i[5:0]];
        default: desc_rd_data_o = '0;
      endcase
    end else if (desc_rd_addr_i[10:9] == 2'b11) begin
      desc_rd_data_o = desc_xwin_base_q[desc_rd_addr_i[8:0]];
    end else begin
      desc_rd_data_o = '0;
    end
  end

  logic refill_allowed_s;
  logic refill_ctrl_req_s;
  logic spill_win_stall_s;

  // A refill of block k may read carry entries spilled at block k-1, so the
  // refill stage waits until the previous block's spill has retired. A host
  // debug load (READ_P v4) runs without a self-run -- no block ever spills,
  // so the wait would wedge; its refill data (grad-window carry slots) is
  // irrelevant to the p readback and is rewritten by the self-run's own
  // loads before any play.
  assign refill_allowed_s =
    memset_done_q &&
    (ld_dbg_mode_q ||
     (ld_blk_q == '0) ||
     (spill_done_valid_q && (spill_done_blk_q == BLK_IDX_W'(ld_blk_q - 1'b1))));

  assign refill_ctrl_req_s =
    (ld_state_q == LD_REFILL) && (refill_launch_q < hdr_n_refill_q) &&
    refill_allowed_s;

  // While a spill window read is stalled the lane word register must not be
  // overwritten by a new ctrl read.
  assign spill_win_stall_s =
    sp_s1_valid_q && !ld_grad_win_rd_grant_i[ctrl_rd_data_q[BANK_IDX_W-1:0]];

  logic hdr_ctrl_req_s;
  logic spill_ctrl_req_s;
  logic xwin_ctrl_req_s;
  logic hdr_ctrl_grant_s;
  logic xwin_ctrl_grant_s;
  logic refill_ctrl_grant_s;
  logic spill_ctrl_grant_s;

  assign hdr_ctrl_req_s = (ld_state_q == LD_HDR_RD);
  assign xwin_ctrl_req_s =
    (ld_state_q == LD_XREFILL) && (xw_launch_q < hdr_n_xwin_q) &&
    !xw_s1_valid_q && !xw_s2_valid_q;
  assign spill_ctrl_req_s =
    (mn_state_q == MN_SPILL) && (sp_launch_q < mn_n_spill_q) &&
    !spill_win_stall_s;

  // spill(b) retire pulses: the no-spill fast path (PLAY exit with zero
  // spill lanes) and the drained pipeline path. Each retire kicks the
  // update rail; the play rail never waits for the update to finish.
  logic spill_retire_play_s;
  logic spill_retire_spill_s;
  logic upd_kick_req_s;
  assign spill_retire_play_s =
    block_done_i && (mn_state_q == MN_PLAY) && (mn_n_spill_q == '0);
  assign spill_retire_spill_s =
    (mn_state_q == MN_SPILL) && (sp_launch_q >= mn_n_spill_q) &&
    !sp_s1_valid_q && !sp_s2_valid_q && !spill_ctrl_grant_s;
  assign upd_kick_req_s = spill_retire_play_s || spill_retire_spill_s;

  // A load kick of block k writes the grad-window half ld_next_half_q; the
  // update rail may still be streaming the same half (update of block k-2).
  // Defer such kicks until the conflicting update retires.
  logic ld_kick_upd_block_s;
  assign ld_kick_upd_block_s =
    ((upd_state_q == UPD_ACTIVE) && (upd_half_q == ld_next_half_q)) ||
    (upd_pend_valid_q && (upd_pend_half_q == ld_next_half_q));

  always_comb begin
    ctrl_pop_s = 1'b0;
    hdr_ctrl_grant_s = 1'b0;
    xwin_ctrl_grant_s = 1'b0;
    refill_ctrl_grant_s = 1'b0;
    spill_ctrl_grant_s = 1'b0;
    if (spill_ctrl_req_s && cf_lane_valid_i) begin
      ctrl_pop_s = 1'b1;
      spill_ctrl_grant_s = 1'b1;
    end else if (hdr_ctrl_req_s && cf_lane_valid_i) begin
      ctrl_pop_s = 1'b1;
      hdr_ctrl_grant_s = 1'b1;
    end else if (xwin_ctrl_req_s && cf_lane_valid_i) begin
      ctrl_pop_s = 1'b1;
      xwin_ctrl_grant_s = 1'b1;
    end else if (refill_ctrl_req_s && cf_lane_valid_i) begin
      ctrl_pop_s = 1'b1;
      refill_ctrl_grant_s = 1'b1;
    end
  end

  // ---------------------------------------------------------------------
  // Load FSM: pre-softmax gflag service + per-block load
  // (memset / p bulk stream / gflag stream / xwin refill / carry refill
  // into the ping-pong half)
  // ---------------------------------------------------------------------
  logic [16:0] ld_rows_max_s;
  logic [16:0] st_chunk_len_s;
  logic [16:0] gs_lines_s;
  logic [16:0] pb_row_s;
  logic [GFLAG_ADDR_W-1:0] gflag_word_addr_s;
  logic [31:0] st_p_base_s;
  logic [31:0] st_gs_base_s;
  logic [31:0] st_sp_base_s;
  // pre-softmax gflag loads ping-pong by segment order (ld_blk_q = segment
  // index); iteration loads follow the load-order ping-pong half
  assign ld_half_s = ld_pre_mode_q ? ld_blk_q[0] : ld_cur_half_q;
  assign ld_rows_max_s = desc_seg_rows_q[ld_blk_q];
  assign st_p_base_s = desc_p_base_q[ld_blk_q];
  assign st_gs_base_s = desc_gs_base_q[ld_blk_q];
  assign st_sp_base_s = desc_sp_base_q[ld_blk_q];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ld_state_q <= LD_IDLE;
      ld_blk_q <= '0;
      ld_pre_mode_q <= 1'b0;
      ld_dbg_mode_q <= 1'b0;
      ld_next_half_q <= 1'b0;
      ld_cur_half_q <= 1'b0;
      ld_done_half_q <= 1'b0;
      hdr_n_refill_q <= '0;
      hdr_n_spill_q <= '0;
      hdr_n_xwin_q <= '0;
      ld_done_valid_q <= 1'b0;
      ld_done_blk_q <= '0;
      pre_loaded_valid_q <= 1'b0;
      pre_loaded_idx_q <= '0;
      ld_kick_valid_q <= 1'b0;
      ld_kick_blk_q <= '0;
      ld_kick_defer_q <= 1'b0;
      ld_kick_defer_blk_q <= '0;
      st_line_q <= '0;
      st_lines_q <= '0;
      st_chunk_end_q <= '0;
      pb_hold_valid_q <= 1'b0;
      pb_phase_q <= 1'b0;
      memset_row_q <= '0;
      memset_done_q <= 1'b0;
      refill_launch_q <= '0;
      rf_s1_valid_q <= 1'b0;
      rf_s2_valid_q <= 1'b0;
      rf_s2_slot_q <= '0;
      xw_launch_q <= '0;
      xw_s1_valid_q <= 1'b0;
      xw_s2_valid_q <= 1'b0;
      xw_s2_slot_q <= '0;
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        pb_hold_lanes_q[bank_idx] <= '0;
      end
    end else begin
      // main FSM -> load FSM kick. Block 0 of a run is kicked by GO; the
      // next block (or the wrap to block 0 of the next iteration) is
      // kicked when the current block's NET has drained (MN_PLAY exit), so
      // the next load overlaps the current block's spill/update. The kick
      // is held until the (idle) loader takes it; kicks whose target half
      // is still read by the update rail are deferred (ld_kick_upd_block_s).
      if (go_i && (mn_state_q == MN_IDLE)) begin
        ld_kick_valid_q <= 1'b1;
        ld_kick_blk_q <= '0;
      end else if (
        block_done_i && (mn_state_q == MN_PLAY) && !final_blk_s
      ) begin
        if (ld_kick_upd_block_s) begin
          // the play rail will advance blk_idx_q at this block's spill
          // retire, before the update retires and the deferred kick fires;
          // the kick target must be latched NOW (it is the block after the
          // one whose PLAY just drained)
          ld_kick_defer_q <= 1'b1;
          ld_kick_defer_blk_q <=
            ({1'b0, blk_idx_q} + 1'b1 < block_count_s)
              ? (blk_idx_q + 1'b1)
              : '0;
        end else begin
          ld_kick_valid_q <= 1'b1;
          ld_kick_blk_q <=
            // NB: compare against the full-width block_count_s. Truncating
            // block_count to BLK_IDX_W bits wraps 16 -> 0 and makes every
            // post-block-0 kick reload block 0 (full-scale 16-block hang).
            ({1'b0, blk_idx_q} + 1'b1 < block_count_s)
              ? (blk_idx_q + 1'b1)
              : '0;
        end
      end else if (ld_kick_defer_q && !ld_kick_upd_block_s) begin
        ld_kick_defer_q <= 1'b0;
        ld_kick_valid_q <= 1'b1;
        ld_kick_blk_q <= ld_kick_defer_blk_q;
      end else if ((ld_state_q == LD_IDLE) && ld_kick_valid_q) begin
        ld_kick_valid_q <= 1'b0;
      end

      // pre-softmax load bookkeeping (engine left PRE_SOFTMAX)
      if (!pre_seg_active_i) begin
        pre_loaded_valid_q <= 1'b0;
      end

      // main FSM consumes the load-done flag
      if (
        ld_done_valid_q &&
        (mn_state_q == MN_WAIT_LOAD) && (ld_done_blk_q == blk_idx_q)
      ) begin
        ld_done_valid_q <= 1'b0;
      end

      unique case (ld_state_q)
        LD_IDLE: begin
          if (dbg_load_kick_i) begin
            // host debug load (READ_P v4): full service into an explicit
            // half; the ping-pong alternation is NOT advanced and the load
            // reports through the normal ld_done_valid/ld_done_blk channel
            ld_state_q <= LD_HDR_RD;
            ld_blk_q <= dbg_load_blk_i;
            ld_pre_mode_q <= 1'b0;
            ld_dbg_mode_q <= 1'b1;
            ld_cur_half_q <= dbg_load_half_i;
            memset_row_q <= '0;
            memset_done_q <= 1'b0;
          end else if (ld_kick_valid_q) begin
            // iteration block load (full service); the half follows the
            // load-order ping-pong (one load per block, strictly in order)
            ld_state_q <= LD_HDR_RD;
            ld_blk_q <= ld_kick_blk_q;
            ld_pre_mode_q <= 1'b0;
            ld_dbg_mode_q <= 1'b0;
            ld_cur_half_q <= ld_next_half_q;
            ld_next_half_q <= ~ld_next_half_q;
            memset_row_q <= '0;
            memset_done_q <= 1'b0;
          end else if (
            pre_seg_active_i &&
            (!pre_loaded_valid_q || (pre_loaded_idx_q != pre_seg_idx_i))
          ) begin
            // pre-softmax segment service (gs/special bitstreams -> gflag)
            ld_state_q <= LD_GS_REQ;
            ld_blk_q <= pre_seg_idx_i;
            ld_pre_mode_q <= 1'b1;
            // the full-service path (re)initializes the stream counters at
            // LD_HDR_CAP / PBULK exit, but pre mode enters at LD_GS_REQ and
            // would otherwise inherit a stale st_lines_q from the previous
            // context (a shorter previous block truncates this block's gs
            // stream, leaving the tail gflag words unwritten)
            st_line_q <= '0;
            st_lines_q <= 17'((desc_seg_rows_q[pre_seg_idx_i] + 17'd31) >> 5);
            hdr_n_refill_q <= '0;
            hdr_n_spill_q <= '0;
            hdr_n_xwin_q <= '0;
            memset_done_q <= 1'b1;   // no memset in pre-softmax service
          end
        end
        LD_HDR_RD: begin
          if (hdr_ctrl_grant_s) begin
            ld_state_q <= LD_HDR_CAP;
          end
        end
        LD_HDR_CAP: begin
          hdr_n_refill_q <= ctrl_rd_data_q[15:0];
          hdr_n_spill_q <= ctrl_rd_data_q[31:16];
          hdr_n_xwin_q <= ctrl_rd_data_q[47:32];
          st_line_q <= '0;
          st_lines_q <= 17'((desc_seg_rows_q[ld_blk_q] + 17'd1) >> 1);
          ld_state_q <= LD_PBULK_REQ;
        end
        // ---- PBULK: stream the block's DDR p region into pwin (indiscrimi-
        // nate, including hole garbage; 2 p rows per line, lane[15:0]=row 2j,
        // lane[31:16]=row 2j+1) ----
        LD_PBULK_REQ: begin
          if (meta_rd_req_ready_i) begin
            st_chunk_end_q <= st_line_q + st_chunk_len_s;
            pb_hold_valid_q <= 1'b0;
            ld_state_q <= LD_PBULK;
          end
        end
        LD_PBULK: begin
          if (meta_rd_resp_valid_i && meta_rd_resp_ready_o) begin
            pb_hold_valid_q <= 1'b1;
            pb_phase_q <= 1'b0;
            for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
              pb_hold_lanes_q[bank_idx] <= meta_rd_resp_lane_i[bank_idx];
            end
          end else if (pb_hold_valid_q) begin
            if (pb_phase_q) begin
              pb_hold_valid_q <= 1'b0;
              if (st_line_q + 1'b1 >= st_lines_q) begin
                st_line_q <= '0;
                st_lines_q <= gs_lines_s;
                ld_state_q <= LD_GS_REQ;
              end else begin
                st_line_q <= st_line_q + 1'b1;
                if (st_line_q + 1'b1 == st_chunk_end_q) begin
                  ld_state_q <= LD_PBULK_REQ;
                end
              end
            end
            pb_phase_q <= ~pb_phase_q;
          end
        end
        // ---- GS: stream the gs bitstream into the gflag window (one 32b
        // word per beat: gflag word 2j) ----
        LD_GS_REQ: begin
          if (meta_rd_req_ready_i) begin
            st_chunk_end_q <= st_line_q + st_chunk_len_s;
            ld_state_q <= LD_GS;
          end
        end
        LD_GS: begin
          if (meta_rd_resp_valid_i && meta_rd_resp_ready_o) begin
            if (st_line_q + 1'b1 >= st_lines_q) begin
              st_line_q <= '0;
              st_lines_q <= gs_lines_s;
              // pre-softmax service also needs the special window: the
              // pre-softmax p writes demux on it (special rows -> xwin_p),
              // otherwise their initial authoritative p is silently lost
              ld_state_q <= LD_SP_REQ;
            end else begin
              st_line_q <= st_line_q + 1'b1;
              if (st_line_q + 1'b1 == st_chunk_end_q) begin
                ld_state_q <= LD_GS_REQ;
              end
            end
          end
        end
        // ---- SP: stream the special bitstream into the gflag window (gflag
        // word 2j+1) ----
        LD_SP_REQ: begin
          if (meta_rd_req_ready_i) begin
            st_chunk_end_q <= st_line_q + st_chunk_len_s;
            ld_state_q <= LD_SP;
          end
        end
        LD_SP: begin
          if (meta_rd_resp_valid_i && meta_rd_resp_ready_o) begin
            if (st_line_q + 1'b1 >= st_lines_q) begin
              // pre-softmax service stops after gs+sp (no xrefill/refill)
              ld_state_q <= ld_pre_mode_q ? LD_DONE : LD_XREFILL;
              xw_launch_q <= '0;
              xw_s1_valid_q <= 1'b0;
              xw_s2_valid_q <= 1'b0;
            end else begin
              st_line_q <= st_line_q + 1'b1;
              if (st_line_q + 1'b1 == st_chunk_end_q) begin
                ld_state_q <= LD_SP_REQ;
              end
            end
          end
        end
        // ---- XREFILL: cross-window p refill (ctrl xwin_p lanes: s0 lane pop
        // -> s1 xwin_p read -> s2 pwin write; overwrites PBULK's hole
        // garbage and all skip rows) ----
        LD_XREFILL: begin
          if (xwin_ctrl_grant_s) begin
            xw_launch_q <= xw_launch_q + 1'b1;
          end
          xw_s1_valid_q <= xwin_ctrl_grant_s;
          xw_s2_valid_q <= xw_s1_valid_q;
          if (xw_s1_valid_q) begin
            xw_s2_slot_q <= ctrl_rd_data_q[32+SLOT_W-1:32];
          end
          if (
            (xw_launch_q >= hdr_n_xwin_q) &&
            !xw_s1_valid_q && !xw_s2_valid_q && !xwin_ctrl_grant_s
          ) begin
            ld_state_q <= LD_REFILL;
            refill_launch_q <= '0;
            rf_s1_valid_q <= 1'b0;
            rf_s2_valid_q <= 1'b0;
          end
        end
        LD_REFILL: begin
          // stage 0: ctrl lane read (arbitrated, gated on memset + the
          // previous block's spill)
          if (refill_ctrl_grant_s) begin
            refill_launch_q <= refill_launch_q + 1'b1;
          end
          rf_s1_valid_q <= refill_ctrl_grant_s;
          // stage 1: carry read (address taken combinationally from the
          // lane word); stage 2 (window write) fires next cycle
          rf_s2_valid_q <= rf_s1_valid_q;
          if (rf_s1_valid_q) begin
            rf_s2_slot_q <= ctrl_rd_data_q[16+SLOT_W:17];
          end
          // done when all lanes launched, the pipeline drained and the
          // memset retired (it shares the window write port)
          if (
            (refill_launch_q >= hdr_n_refill_q) &&
            !rf_s1_valid_q && !rf_s2_valid_q && !refill_ctrl_grant_s &&
            memset_done_q
          ) begin
            ld_state_q <= LD_DONE;
          end
        end
        LD_DONE: begin
          if (ld_pre_mode_q) begin
            pre_loaded_valid_q <= 1'b1;
            pre_loaded_idx_q <= ld_blk_q;
          end else begin
            ld_done_valid_q <= 1'b1;
            ld_done_blk_q <= ld_blk_q;
            ld_done_half_q <= ld_cur_half_q;
          end
          ld_state_q <= LD_IDLE;
        end
        default: begin
          ld_state_q <= LD_IDLE;
        end
      endcase

      // memset runs concurrently with the region streams (the grad window
      // write port is exclusively the loader's while load is active)
      if (
        !ld_pre_mode_q &&
        (ld_state_q != LD_IDLE) && (ld_state_q != LD_HDR_RD) &&
        (ld_state_q != LD_HDR_CAP) && (ld_state_q != LD_REFILL) &&
        (ld_state_q != LD_DONE) &&
        !memset_done_q
      ) begin
        if (memset_row_q >= ld_rows_max_s - 1'b1) begin
          memset_done_q <= 1'b1;
        end else begin
          memset_row_q <= memset_row_q + 1'b1;
        end
      end
    end
  end

  // refill stage-1 carry read (combinational issue, registered data)
  assign carry_rd_ce_s = rf_s1_valid_q;
  assign carry_rd_addr_s = ctrl_rd_data_q[CARRY_ADDR_W-1:0];

  // stream chunk length: min(STREAM_CHUNK_LINES, remaining)
  assign st_chunk_len_s =
    ((st_lines_q - st_line_q) > 17'(STREAM_CHUNK_LINES))
      ? 17'(STREAM_CHUNK_LINES)
      : (st_lines_q - st_line_q);
  // gs/special bitstream line count for the current context (gflag covers
  // all window rows, 32 rows per line/word)
  assign gs_lines_s = 17'((ld_rows_max_s + 17'd31) >> 5);

  // ---------------------------------------------------------------------
  // Loader output ports
  // ---------------------------------------------------------------------
  always_comb begin
    for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
      ld_grad_win_wr_ce_o[bank_idx] = 1'b0;
      ld_grad_win_wr_addr_o[bank_idx] = '0;
      ld_grad_win_wr_data_o[bank_idx] = '0;
      ld_grad_win_rd_req_o[bank_idx] = 1'b0;
      ld_grad_win_rd_addr_o[bank_idx] = '0;
      ld_pwin_wr_ce_o[bank_idx] = 1'b0;
      ld_pwin_wr_addr_o[bank_idx] = '0;
      ld_pwin_wr_data_o[bank_idx] = '0;
      ld_gflag_wr_ce_o[bank_idx] = 1'b0;
      ld_gflag_wr_addr_o[bank_idx] = '0;
      ld_gflag_wr_data_o[bank_idx] = '0;
      ld_xwin_rd_req_o[bank_idx] = 1'b0;
      ld_xwin_rd_addr_o[bank_idx] = '0;
    end

    // memset: clear rows 0..n_b of the loading half (all banks in parallel)
    if (
      !ld_pre_mode_q &&
      (ld_state_q != LD_IDLE) && (ld_state_q != LD_HDR_RD) &&
      (ld_state_q != LD_HDR_CAP) && (ld_state_q != LD_REFILL) &&
      (ld_state_q != LD_DONE) &&
      !memset_done_q
    ) begin
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        if (memset_row_q < desc_n_b_q[{ld_blk_q, BANK_IDX_W'(bank_idx)}]) begin
          ld_grad_win_wr_ce_o[bank_idx] = 1'b1;
          ld_grad_win_wr_addr_o[bank_idx] = {ld_half_s, memset_row_q[WIN_ROW_W-1:0]};
          ld_grad_win_wr_data_o[bank_idx] = '0;
        end
      end
    end

    // refill stage 2: window[slot] = carry word (single bank strobe); the
    // refill pipeline only launches after memset_done, so no port conflict
    if (rf_s2_valid_q) begin
      ld_grad_win_wr_ce_o[rf_s2_slot_q[BANK_IDX_W-1:0]] = 1'b1;
      ld_grad_win_wr_addr_o[rf_s2_slot_q[BANK_IDX_W-1:0]] =
        {ld_half_s, rf_s2_slot_q[SLOT_W-1:BANK_IDX_W]};
      ld_grad_win_wr_data_o[rf_s2_slot_q[BANK_IDX_W-1:0]] = carry_rd_data_q;
    end

    // PBULK unpack: two pwin writes per held beat (all banks in parallel);
    // pb_phase_q==0 writes row 2j (lane[15:0]), ==1 writes row 2j+1
    // (lane[31:16])
    if ((ld_state_q == LD_PBULK) && pb_hold_valid_q) begin
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        ld_pwin_wr_ce_o[bank_idx] = 1'b1;
        ld_pwin_wr_addr_o[bank_idx] =
          {ld_half_s, pb_row_s[WIN_ROW_W-1:0]};
        ld_pwin_wr_data_o[bank_idx] = pb_phase_q
          ? pb_hold_lanes_q[bank_idx][31:16]
          : pb_hold_lanes_q[bank_idx][15:0];
      end
    end

    // GS/SP beat: direct gflag window word write (all banks in parallel);
    // gs -> word 2j, special -> word 2j+1
    if (
      ((ld_state_q == LD_GS) || (ld_state_q == LD_SP)) &&
      meta_rd_resp_valid_i && meta_rd_resp_ready_o
    ) begin
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        ld_gflag_wr_ce_o[bank_idx] = 1'b1;
        ld_gflag_wr_addr_o[bank_idx] =
          {ld_half_s, gflag_word_addr_s[GFLAG_ADDR_W-2:0]};
        ld_gflag_wr_data_o[bank_idx] = meta_rd_resp_lane_i[bank_idx];
      end
    end

    // xwin refill stage 1: xwin_p read request (bank/idx from the lane)
    if (xw_s1_valid_q) begin
      ld_xwin_rd_req_o[ctrl_rd_data_q[32+BANK_IDX_W-1:32]] = 1'b1;
      ld_xwin_rd_addr_o[ctrl_rd_data_q[32+BANK_IDX_W-1:32]] =
        ctrl_rd_data_q[XWIN_ADDR_W-1:0];
    end
    // xwin refill stage 2: pwin[slot] = xwin word (single bank strobe)
    if (xw_s2_valid_q) begin
      ld_pwin_wr_ce_o[xw_s2_slot_q[BANK_IDX_W-1:0]] = 1'b1;
      ld_pwin_wr_addr_o[xw_s2_slot_q[BANK_IDX_W-1:0]] =
        {ld_half_s, xw_s2_slot_q[SLOT_W-1:BANK_IDX_W]};
      ld_pwin_wr_data_o[xw_s2_slot_q[BANK_IDX_W-1:0]] =
        ld_xwin_rd_data_i[xw_s2_slot_q[BANK_IDX_W-1:0]];
    end

    // spill stage 1: window read request, slot taken combinationally from
    // the lane word (held while sp_s1_valid_q until granted)
    if (sp_s1_valid_q) begin
      ld_grad_win_rd_req_o[ctrl_rd_data_q[BANK_IDX_W-1:0]] = 1'b1;
      ld_grad_win_rd_addr_o[ctrl_rd_data_q[BANK_IDX_W-1:0]] =
        {mn_half_q, ctrl_rd_data_q[SLOT_W-1:BANK_IDX_W]};
    end
  end

  // PBULK row of the current unpack phase
  assign pb_row_s = 17'((st_line_q << 1) + 17'(pb_phase_q));

  // gflag window word address for the GS/SP beat (word j; the gs/special
  // target memory is selected by ld_gflag_wr_special_o = (state == LD_SP))
  assign gflag_word_addr_s = GFLAG_ADDR_W'(st_line_q);
  assign ld_gflag_wr_special_o = (ld_state_q == LD_SP);

  // stream request: one request per chunk (row = region base + lines already
  // fetched, count = chunk length); base follows the stage's region.
  // NB: st_line_q is only 17b -- zero-EXTEND it with a cast; a
  // [ROW_W-1:0] part-select is out of bounds and X-pads the upper bits
  // in simulation (the ROW_W=20 gs-address corruption, 2026-09-08).
  assign meta_rd_req_valid_o =
    (ld_state_q == LD_PBULK_REQ) || (ld_state_q == LD_GS_REQ) ||
    (ld_state_q == LD_SP_REQ);
  assign meta_rd_req_row_o =
    (ld_state_q == LD_PBULK_REQ)
      ? st_p_base_s[ROW_W-1:0] + ROW_W'(st_line_q)
      : ((ld_state_q == LD_SP_REQ)
          ? st_sp_base_s[ROW_W-1:0] + ROW_W'(st_line_q)
          : st_gs_base_s[ROW_W-1:0] + ROW_W'(st_line_q));
  assign meta_rd_req_count_o = DDR_COUNT_W'(st_chunk_len_s);
  assign meta_rd_resp_ready_o =
    ((ld_state_q == LD_PBULK) && !pb_hold_valid_q) ||
    (ld_state_q == LD_GS) || (ld_state_q == LD_SP);

  // engine grad-window write-port ownership during an iteration-mode load
  assign engine_load_active_o =
    !ld_pre_mode_q &&
    ((ld_state_q == LD_PBULK_REQ) || (ld_state_q == LD_PBULK) ||
     (ld_state_q == LD_GS_REQ) || (ld_state_q == LD_GS) ||
     (ld_state_q == LD_SP_REQ) || (ld_state_q == LD_SP) ||
     (ld_state_q == LD_XREFILL) || (ld_state_q == LD_REFILL));

  // ---------------------------------------------------------------------
  // Main FSM (NET-on-DDR self-run iteration master):
  //   GO -> MN_WAIT_LOAD -> MN_PLAY -> MN_SPILL -> next block;
  //   the last block of the last iteration latches sr_done_o and retires
  //   to MN_DONE. No host interaction exists anywhere in the loop.
  // ---------------------------------------------------------------------
  assign carry_wr_ce_s = sp_s2_valid_q;
  // packed carry slot address from the spill lane (host static interval
  // packing); the bump-allocation desc_carry_base descriptor is retired
  assign carry_wr_addr_s = sp_s2_addr_q;
  assign carry_wr_data_s = ld_grad_win_rd_data_i[sp_s2_slot_q[BANK_IDX_W-1:0]];

  assign sr_done_o = sr_done_q;
  assign sr_running_o = sr_running_q;
  assign sr_upd_probe_o[0] = (upd_state_q == UPD_ACTIVE);
  assign sr_upd_probe_o[6:1] = 6'(upd_blk_q);
  assign sr_upd_probe_o[7] = upd_half_q;
  assign sr_upd_probe_o[8] = upd_pend_valid_q;
  assign sr_upd_probe_o[9] = 1'b0;
  assign sr_iter_cnt_o = iter_cnt_q;
  assign sr_error_o = {7'b0, sr_phantom_err_q};
  assign sr_cur_blk_o = 6'(blk_idx_q);
  assign sr_mn_state_o = 3'(mn_state_q);
  assign sr_ld_state_o = 4'(ld_state_q);
  assign sr_ld_blk_o = 6'(ld_blk_q);
  assign sr_ld_done_valid_o = ld_done_valid_q;
  assign sr_ld_row_cnt_o = st_line_q;
  assign sr_ld_probe2_o = {
    8'(xw_launch_q[7:0]), pb_hold_valid_q, xw_s1_valid_q, xw_s2_valid_q,
    meta_rd_resp_valid_i, meta_rd_req_ready_i, meta_rd_req_valid_o,
    4'(ld_state_q)
  };

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mn_state_q <= MN_IDLE;
      blk_idx_q <= '0;
      mn_half_q <= 1'b0;
      spill_done_blk_q <= '0;
      spill_done_valid_q <= 1'b0;
      mn_n_spill_q <= '0;
      sp_launch_q <= '0;
      sp_s1_valid_q <= 1'b0;
      sp_s2_valid_q <= 1'b0;
      sp_s2_slot_q <= '0;
      engine_update_active_o <= 1'b0;
      engine_iter_done_o <= 1'b0;
      iter_done_o <= 1'b0;
      spill_done_o <= 1'b0;
      upd_guard_q <= '0;
      upd_phantom_seen_q <= 1'b0;
      upd_seen_busy_q <= 1'b0;
      upd_state_q <= UPD_IDLE;
      upd_blk_q <= '0;
      upd_half_q <= 1'b0;
      upd_pend_valid_q <= 1'b0;
      upd_pend_blk_q <= '0;
      upd_pend_half_q <= 1'b0;
      play_iter_cnt_q <= '0;
      sr_play_o <= 1'b0;
      sr_play_last_o <= 1'b0;
      sr_done_q <= 1'b0;
      sr_running_q <= 1'b0;
      sr_phantom_err_q <= 1'b0;
      iter_cnt_q <= '0;
      go_iters_q <= '0;
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        upd_p_cnt_q[bank_idx] <= '0;
      end
    end else begin
      engine_iter_done_o <= 1'b0;
      iter_done_o <= 1'b0;
      spill_done_o <= 1'b0;

      // row-writer phantom discards feed the same sticky flag as the
      // p-write census (any discard means the engine emitted beyond n_b)
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        if (wr_phantom_pop_i[bank_idx]) begin
          upd_phantom_seen_q <= 1'b1;
`ifndef SYNTHESIS
          $display("%t SRCTRL WR PHANTOM POP bank=%0d (upd ublk=%0d uhalf=%0d play blk=%0d)",
            $time, bank_idx, upd_blk_q, upd_half_q, blk_idx_q);
`endif
        end
      end

      unique case (mn_state_q)
        MN_IDLE: begin
          if (go_i) begin
            // start a self-run: block 0's load is kicked by the GO branch in
            // the loader handshake above
            blk_idx_q <= '0;
            iter_cnt_q <= '0;
            play_iter_cnt_q <= '0;
            go_iters_q <= go_iters_i;
            spill_done_valid_q <= 1'b0;
            upd_phantom_seen_q <= 1'b0;
            upd_state_q <= UPD_IDLE;
            upd_pend_valid_q <= 1'b0;
            sr_done_q <= 1'b0;
            sr_phantom_err_q <= 1'b0;
            sr_running_q <= 1'b1;
            mn_state_q <= MN_WAIT_LOAD;
          end
        end
        MN_WAIT_LOAD: begin
          if (ld_done_valid_q && (ld_done_blk_q == blk_idx_q)) begin
            // copy the block's spill geometry before the next load's
            // header read may overwrite the shared header registers
            mn_n_spill_q <= hdr_n_spill_q;
            mn_half_q <= ld_done_half_q;
            ld_adopted_q <= 1'b1;
          end
          // the first play of a run is gated on the engine phase having
          // reached GRADP (the adapter's GO also pulses blk_go_i)
          if (ld_adopted_q && (system_phase_i == 4'd2)) begin
            ld_adopted_q <= 1'b0;
            sr_play_o <= 1'b1;
            sr_play_last_o <= ({1'b0, blk_idx_q} + 1'b1 >= block_count_s);
            mn_state_q <= MN_PLAY;
          end
        end
        MN_PLAY: begin
          // the adapter self-run walk streams this block's NET frames from
          // the fetcher and pulses block_done_i when GRADP has drained
          if (sr_play_ack_i) begin
            sr_play_o <= 1'b0;
          end
          if (block_done_i) begin
            sp_launch_q <= '0;
            sp_s1_valid_q <= 1'b0;
            sp_s2_valid_q <= 1'b0;
            if (mn_n_spill_q == '0) begin
              // no spill lanes: the spill of this block is trivially
              // retired; the update rail is kicked combinationally
              // (upd_kick_req_s) and the play rail advances without
              // waiting for the update to finish
              spill_done_o <= 1'b1;
              spill_done_valid_q <= 1'b1;
              spill_done_blk_q <= blk_idx_q;
              if ({1'b0, blk_idx_q} + 1'b1 >= block_count_s) begin
                if (play_iter_cnt_q + 1'b1 >= go_iters_q) begin
                  mn_state_q <= MN_DONE;
                end else begin
                  play_iter_cnt_q <= play_iter_cnt_q + 1'b1;
                  blk_idx_q <= '0;
                  mn_state_q <= MN_WAIT_LOAD;
                end
              end else begin
                blk_idx_q <= blk_idx_q + 1'b1;
                mn_state_q <= MN_WAIT_LOAD;
              end
            end else begin
              mn_state_q <= MN_SPILL;
            end
          end
        end
        MN_SPILL: begin
          // stage 0: ctrl lane pop (arbitrated)
          if (spill_ctrl_grant_s) begin
            sp_launch_q <= sp_launch_q + 1'b1;
          end
          // stage 1: lane word -> window read request (sp_s1_valid_q holds
          // the request until granted; the slot rides ctrl_rd_data_q)
          if (spill_ctrl_grant_s) begin
            sp_s1_valid_q <= 1'b1;
          end else if (
            sp_s1_valid_q &&
            ld_grad_win_rd_grant_i[ctrl_rd_data_q[BANK_IDX_W-1:0]]
          ) begin
            sp_s1_valid_q <= 1'b0;
          end
          // stage 2: window read data -> carry write; the carry slot address
          // rides the spill lane at [16+CARRY_ADDR_W:17] (host-packed static
          // interval assignment; bump layout is a valid special case)
          if (
            sp_s1_valid_q &&
            ld_grad_win_rd_grant_i[ctrl_rd_data_q[BANK_IDX_W-1:0]]
          ) begin
            sp_s2_valid_q <= 1'b1;
            sp_s2_slot_q <= ctrl_rd_data_q[SLOT_W-1:0];
            sp_s2_addr_q <= ctrl_rd_data_q[16+CARRY_ADDR_W:17];
          end else begin
            sp_s2_valid_q <= 1'b0;
          end
          // done when all lanes launched and the pipeline drained
          if (
            (sp_launch_q >= mn_n_spill_q) &&
            !sp_s1_valid_q && !sp_s2_valid_q && !spill_ctrl_grant_s
          ) begin
            spill_done_o <= 1'b1;
            spill_done_valid_q <= 1'b1;
            spill_done_blk_q <= blk_idx_q;
            // kick the update rail (upd_kick_req_s) and advance; the play
            // rail never waits for the update to finish
            if ({1'b0, blk_idx_q} + 1'b1 >= block_count_s) begin
              if (play_iter_cnt_q + 1'b1 >= go_iters_q) begin
                mn_state_q <= MN_DONE;
              end else begin
                play_iter_cnt_q <= play_iter_cnt_q + 1'b1;
                blk_idx_q <= '0;
                mn_state_q <= MN_WAIT_LOAD;
              end
            end else begin
              blk_idx_q <= blk_idx_q + 1'b1;
              mn_state_q <= MN_WAIT_LOAD;
            end
          end
        end
        MN_DONE: begin
          // latched terminal state; a fresh GO re-arms the master
          if (go_i) begin
            blk_idx_q <= '0;
            iter_cnt_q <= '0;
            play_iter_cnt_q <= '0;
            go_iters_q <= go_iters_i;
            spill_done_valid_q <= 1'b0;
            upd_phantom_seen_q <= 1'b0;
            upd_state_q <= UPD_IDLE;
            upd_pend_valid_q <= 1'b0;
            sr_done_q <= 1'b0;
            sr_phantom_err_q <= 1'b0;
            sr_running_q <= 1'b1;
            mn_state_q <= MN_WAIT_LOAD;
          end
        end
        default: begin
          mn_state_q <= MN_IDLE;
        end
      endcase

      // ---- update rail: UPD(b) overlaps the play rail's PLAY(b+1) ----
      // Kicked by the play rail's spill retire (upd_kick_req_s); lags the
      // play rail by exactly one block (opposite window halves). Single
      // in-flight update; a spill retiring while busy fills the one-deep
      // pending slot. A start (direct or from the slot) additionally
      // waits for the previous p write session to have fully drained
      // into DDR (p_asm_drained_i): p_line_asm restarts its session on
      // the update_active edge, and a restart with lines still buffered
      // would silently discard them. The kick itself is never blocked --
      // a start deferred by the drain gate is parked in the pending slot.
      unique case (upd_state_q)
        UPD_IDLE: begin
          if (upd_pend_valid_q && p_asm_drained_i) begin
            upd_pend_valid_q <= 1'b0;
            upd_state_q <= UPD_ACTIVE;
            upd_blk_q <= upd_pend_blk_q;
            upd_half_q <= upd_pend_half_q;
            engine_update_active_o <= 1'b1;
            upd_guard_q <= 5'd16;
            upd_seen_busy_q <= 1'b0;
            for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
              upd_p_cnt_q[bank_idx] <= '0;
            end
          end else if (upd_kick_req_s && p_asm_drained_i) begin
            upd_state_q <= UPD_ACTIVE;
            upd_blk_q <= blk_idx_q;
            upd_half_q <= mn_half_q;
            engine_update_active_o <= 1'b1;
            upd_guard_q <= 5'd16;
            upd_seen_busy_q <= 1'b0;
            for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
              upd_p_cnt_q[bank_idx] <= '0;
            end
          end
        end
        UPD_ACTIVE: begin
          // census of pre-mask p-write requests (phantom-emit guard)
          for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
            if (p_wr_strobe_i[bank_idx]) begin
              upd_p_cnt_q[bank_idx] <= upd_p_cnt_q[bank_idx] + 1'b1;
            end
          end
          // track real engine activity: all_finish is enable-gated and goes
          // stale-high at update start; it only falls once the row
          // reader/writer actually starts working (which the overlap's
          // bridge contention can delay well past the fixed guard)
          if (!softmax_all_finish_i) begin
            upd_seen_busy_q <= 1'b1;
          end
          if (upd_guard_q != '0) begin
            upd_guard_q <= upd_guard_q - 1'b1;
          end else if (upd_seen_busy_q && softmax_all_finish_i) begin
            // update drained: census against the descriptor, then retire
            engine_update_active_o <= 1'b0;
`ifndef SYNTHESIS
            for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
              if (upd_p_cnt_q[bank_idx] != desc_n_b_q[{upd_blk_q, BANK_IDX_W'(bank_idx)}]) begin
                $display("%t SRCTRL UPD CENSUS MISMATCH ublk=%0d uhalf=%0d bank=%0d cnt=%0d n_b=%0d",
                  $time, upd_blk_q, upd_half_q, bank_idx, upd_p_cnt_q[bank_idx],
                  desc_n_b_q[{upd_blk_q, BANK_IDX_W'(bank_idx)}]);
              end
            end
`endif
            for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
              if (upd_p_cnt_q[bank_idx] != desc_n_b_q[{upd_blk_q, BANK_IDX_W'(bank_idx)}]) begin
                upd_phantom_seen_q <= 1'b1;
              end
            end
            if ({1'b0, upd_blk_q} + 1'b1 >= block_count_s) begin
              // iteration boundary (counted at update retire = completed)
              iter_cnt_q <= iter_cnt_q + 1'b1;
              if (iter_cnt_q + 1'b1 >= go_iters_q) begin
                // final iteration drained: tell the engine (phase ->
                // WAIT_GRADP, usr_irq) and latch the done status
                engine_iter_done_o <= 1'b1;
                iter_done_o <= 1'b1;
                sr_done_q <= 1'b1;
                sr_running_q <= 1'b0;
                sr_phantom_err_q <= upd_phantom_seen_q;
              end
            end
            // Always retire through UPD_IDLE (one bubble cycle): the
            // grad_engine p-write mask counter (blk_p_wr_cnt_q) only
            // resets while blk_update_active_i is LOW; holding it high
            // through a back-to-back pend start leaves the previous
            // update's count in place and masks every p write of the
            // next update (census cnt=0, 2026-09-01).
            upd_state_q <= UPD_IDLE;
          end
        end
        default: begin
          upd_state_q <= UPD_IDLE;
        end
      endcase
      // pend capture: a spill retires while the update rail is busy, or
      // while the drain gate (p_asm_drained_i) holds the rail in
      // UPD_IDLE. The one-deep slot can only overflow if an update
      // outlasts two full play+spill cycles (or the drain stalls that
      // long), which the block geometry rules out; flag it.
      if (upd_kick_req_s &&
          ((upd_state_q == UPD_ACTIVE) || upd_pend_valid_q ||
           !p_asm_drained_i)) begin
        upd_pend_valid_q <= 1'b1;
        upd_pend_blk_q <= blk_idx_q;
        upd_pend_half_q <= mn_half_q;
      end
`ifndef SYNTHESIS
      if (rst_n && upd_kick_req_s && upd_pend_valid_q &&
          ((upd_state_q == UPD_ACTIVE) || !p_asm_drained_i)) begin
        $fatal(1, "blocked_ctrl: update-rail pending slot overflow (two spills retired during one update)");
      end
`endif
    end
  end

`ifndef SYNTHESIS
  // ---- NET-on-DDR self-run debug probes (sim only) ----
  main_state_e mn_dbg_q;
  load_state_e ld_dbg_q;
  upd_state_e upd_dbg_q;
  logic sr_done_dbg_q;
  always_ff @(posedge clk) begin
    if (rst_n && (mn_state_q != mn_dbg_q)) begin
      $display("%t SRCTRL mn %0d -> %0d blk=%0d iter=%0d adopted=%0d play=%0d",
        $time, mn_dbg_q, mn_state_q, blk_idx_q, iter_cnt_q, ld_adopted_q, sr_play_o);
    end
    if (rst_n && (ld_state_q != ld_dbg_q)) begin
      $display("%t SRCTRL ld %0d -> %0d blk=%0d pre=%0d rows=%0d",
        $time, ld_dbg_q, ld_state_q, ld_blk_q, ld_pre_mode_q, st_line_q);
    end
    if (rst_n && go_i) begin
      $display("%t SRCTRL GO iters=%0d", $time, go_iters_i);
    end
    if (rst_n && block_done_i) begin
      $display("%t SRCTRL block_done blk=%0d", $time, blk_idx_q);
    end
    if (rst_n && (upd_state_q != upd_dbg_q)) begin
      $display("%t SRCTRL upd %0d -> %0d ublk=%0d uhalf=%0d pend=%0d iters=%0d",
        $time, upd_dbg_q, upd_state_q, upd_blk_q, upd_half_q, upd_pend_valid_q, iter_cnt_q);
    end
    if (rst_n && sr_done_q && !sr_done_dbg_q) begin
      $display("%t SRCTRL RUN_DONE iter=%0d", $time, iter_cnt_q);
    end
    mn_dbg_q <= mn_state_q;
    ld_dbg_q <= ld_state_q;
    upd_dbg_q <= upd_state_q;
    sr_done_dbg_q <= sr_done_q;
  end
`endif

endmodule

`endif
