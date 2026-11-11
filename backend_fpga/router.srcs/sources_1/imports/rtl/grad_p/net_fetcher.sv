`ifndef FPGA_NET_FETCHER_SV
`define FPGA_NET_FETCHER_SV

// Blocked-GRADP v2 NET-on-DDR self-run stream fetcher. See
// fpga/router/verif/blocked_net_image.py for the image format contract.
//
// The whole per-iteration control plane is static, so the host writes one
// DDR image at init (all 512-bit lines, unified image-line addressing):
//   lines [0, DIRECTORY_WORDS)  directory: 64 block slots x 4 x 32b
//                               {ctrl_base, ctrl_words, net_base, net_words}
//                               (ctrl_words in 64b lanes, net_words in 256b
//                               frames; all-zero entry = inactive slot)
//   then per ACTIVE block, in ascending block-id (= execution) order:
//                               ctrl section (8 x 64b lanes per line, last
//                               line zero-padded) then net section (256b
//                               frames, 2 per line, odd tail zero-padded).
//
// After a GO pulse this module reads the directory, compacts the active
// slots (net_words != 0) into ordinal order, and then streams both section
// series on demand: a 512b raw-line fetch port (served by the router
// bridge raw-fetch channel at lowest priority) refills two small word
// FIFOs; consumers pop strictly in order:
//   - NET frame stream (to the adapter self-run walk): 256b frames, two per
//     line, FWFT; frame_last marks the block's final frame. The odd-tail
//     padding half of a block's last line is never served.
//   - ctrl lane stream (to blocked_ctrl): 64b lanes, eight per line, FWFT;
//     the tail-padding lanes of a block's last ctrl line are never served.
// Consumption order is the image order for each stream independently, so no
// block-boundary signalling is needed beyond per-block counts. After the
// last active block's section the stream rewinds to ordinal 0 (the master
// replays the same image for N iterations).

module net_fetcher #(
  parameter int NET_FIFO_WORDS = 256,    // 512b lines of NET frame storage
  parameter int CTRL_FIFO_WORDS = 32,    // 512b lines of ctrl lane storage
  parameter int FETCH_BURST_LINES = 64,  // raw-fetch burst cap (lines)
  parameter int DIRECTORY_WORDS = 16,    // image header lines
  parameter int BLOCK_SLOTS = 64,
  localparam int NET_CNT_W = $clog2(NET_FIFO_WORDS + 1),
  localparam int CTRL_CNT_W = $clog2(CTRL_FIFO_WORDS + 1),
  localparam int SLOT_IDX_W = (BLOCK_SLOTS <= 1) ? 1 : $clog2(BLOCK_SLOTS)
) (
  input  logic         clk,
  input  logic         rst_n,

  // run control (from the adapter's CMD_GO)
  input  logic         go_i,          // pulse: read the directory, start streaming

  // on-demand ctrl section re-serve (READ_P v4 debug window load). Pulsed
  // together with the blocked_ctrl dbg_load_kick, only while the system is
  // idle. dbg_sec_i is the compacted active-table ordinal (== block ordinal
  // when every block slot is active, which the image writer enforces). A
  // kick in F_IDLE first loads the directory (the pre-GO dbg readback case).
  input  logic         dbg_kick_i,
  input  logic [SLOT_IDX_W-1:0] dbg_sec_i,

  // raw line fetch port (router bridge raw-fetch channel, lowest priority)
  output logic         fr_req_valid_o,
  input  logic         fr_req_ready_i,   // 1-cycle accept (burst launched)
  output logic [31:0]  fr_req_line_o,    // absolute image line index
  output logic [8:0]   fr_req_count_o,   // lines in this burst (>= 1)
  input  logic         fr_resp_valid_i,
  output logic         fr_resp_ready_o,
  input  logic [511:0] fr_resp_data_i,
  input  logic         fr_resp_last_i,

  // NET frame stream (to the adapter self-run walk), FWFT
  output logic         nf_frame_valid_o,
  output logic [255:0] nf_frame_data_o,
  output logic         nf_frame_last_o,
  input  logic         nf_frame_pop_i,

  // ctrl lane stream (to blocked_ctrl), FWFT
  output logic         cf_lane_valid_o,
  output logic [63:0]  cf_lane_data_o,
  input  logic         cf_lane_pop_i,

  // debug observability (self-run bring-up probes; read-only)
  output logic [2:0]   dbg_f_state_o,
  output logic [6:0]   dbg_cf_sec_o,
  output logic [6:0]   dbg_nf_sec_o,
  output logic [31:0]  dbg_cf_line_o,
  output logic [31:0]  dbg_nf_line_o,
  output logic         dbg_flight_o,
  // performance bundle for the status mirror (M12 watermarks / M13 burst
  // stats), packed as:
  //   [8:0] net fifo watermark [14:9] ctrl fifo watermark
  //   [30:15] bursts <8 [46:31] bursts 8-31 [62:47] bursts 32-63
  //   [78:63] bursts =64 [110:79] total bursts [142:111] total lines
  //   [174:143] net fifo full cycles [206:175] ctrl fifo full cycles
  output logic [223:0] nf_perf_probe_o
);

  localparam int SEC_PTR_W = SLOT_IDX_W + 1;

  // ---------------------------------------------------------------------
  // Directory registers (raw BLOCK_SLOTS x 4 x 32b) and the compacted active
  // table (ordinal order; slot s is active iff net_words[s] != 0).
  // The compaction runs SERIALLY (one slot per cycle in F_DIR_COMPACT):
  // a single-cycle combinational compaction costs O(BLOCK_SLOTS^2) mux
  // cells (~4.5M at 64 slots — killed the build), while the serial walk is
  // ~free and its latency is paid once per GO.
  // ---------------------------------------------------------------------
  logic [31:0] dir_q [0:(BLOCK_SLOTS * 4) - 1];

  logic [31:0] act_ctrl_base_q  [0:BLOCK_SLOTS-1];
  logic [31:0] act_ctrl_words_q [0:BLOCK_SLOTS-1];
  logic [31:0] act_net_base_q   [0:BLOCK_SLOTS-1];
  logic [31:0] act_net_words_q  [0:BLOCK_SLOTS-1];
  logic [SEC_PTR_W-1:0] act_count_q;

  // serial compaction cursor (slot being inspected in F_DIR_COMPACT)
  logic [SLOT_IDX_W-1:0] cmp_slot_q;

  typedef enum logic [2:0] {
    F_IDLE,
    F_DIR_REQ,
    F_DIR_RECV,
    F_DIR_COMPACT,
    F_RUN
  } fetch_state_e;
  fetch_state_e f_state_q;

  logic        flight_q;        // a burst has been accepted, responses pending
  logic        flight_net_q;    // response target: 1 = net FIFO, 0 = ctrl FIFO
  logic [4:0]  dir_recv_cnt_q;

  // ---------------------------------------------------------------------
  // Per-stream section cursors (into the compacted active table)
  // ---------------------------------------------------------------------
  // ctrl stream fetch side
  logic [SEC_PTR_W-1:0] cf_sec_q;
  logic [31:0]          cf_line_q;
  logic [31:0]          cf_lines_left_q;
  logic [31:0]          cf_lanes_left_q;
  // ctrl stream re-sync requests (latched; flushed when no burst in flight)
  logic                   dbg_pend_q;   // dbg re-serve section dbg_sec_q
  logic [SLOT_IDX_W-1:0]  dbg_sec_q;
  logic                   go_pend_q;    // GO re-sync: rewind ctrl stream to 0
  // net stream fetch side
  logic [SEC_PTR_W-1:0] nf_sec_q;
  logic [31:0]          nf_line_q;
  logic [31:0]          nf_lines_left_q;
  logic [31:0]          nf_frames_left_q;

  // ---------------------------------------------------------------------
  // Word FIFOs (handwritten ring buffers; ram_style for BRAM inference)
  // ---------------------------------------------------------------------
  (* ram_style = "block" *) logic [511:0] net_fifo_mem_q [0:NET_FIFO_WORDS-1];
  (* ram_style = "block" *) logic [511:0] ctrl_fifo_mem_q [0:CTRL_FIFO_WORDS-1];
  localparam int NET_PTR_W = (NET_FIFO_WORDS <= 1) ? 1 : $clog2(NET_FIFO_WORDS);
  localparam int CTRL_PTR_W = (CTRL_FIFO_WORDS <= 1) ? 1 : $clog2(CTRL_FIFO_WORDS);
  logic [NET_PTR_W-1:0]  net_wr_ptr_q, net_rd_ptr_q;
  logic [NET_CNT_W-1:0]  net_count_q;
  logic [CTRL_PTR_W-1:0] ctrl_wr_ptr_q, ctrl_rd_ptr_q;
  logic [CTRL_CNT_W-1:0] ctrl_count_q;

  function automatic logic [NET_PTR_W-1:0] net_ptr_inc(input logic [NET_PTR_W-1:0] ptr);
    return (ptr == NET_PTR_W'(NET_FIFO_WORDS - 1)) ? '0 : (ptr + 1'b1);
  endfunction
  function automatic logic [CTRL_PTR_W-1:0] ctrl_ptr_inc(input logic [CTRL_PTR_W-1:0] ptr);
    return (ptr == CTRL_PTR_W'(CTRL_FIFO_WORDS - 1)) ? '0 : (ptr + 1'b1);
  endfunction

  logic net_push_s, net_pop_word_s;
  logic ctrl_push_s, ctrl_pop_word_s;
  assign net_push_s =
    (f_state_q == F_RUN) && fr_resp_valid_i && fr_resp_ready_o &&
    flight_q && flight_net_q;
  assign ctrl_push_s =
    (f_state_q == F_RUN) && fr_resp_valid_i && fr_resp_ready_o &&
    flight_q && !flight_net_q;

  always_ff @(posedge clk) begin
    if (net_push_s) begin
      net_fifo_mem_q[net_wr_ptr_q] <= fr_resp_data_i;
    end
    if (ctrl_push_s) begin
      ctrl_fifo_mem_q[ctrl_wr_ptr_q] <= fr_resp_data_i;
    end
  end

  // ---------------------------------------------------------------------
  // NET frame unpack: two 256b frames per 512b line
  // ---------------------------------------------------------------------
  logic        nf_half_q;          // next frame half in the head line
  logic [31:0] nf_served_q;        // frames served in the current net section
  logic        nf_pop_s;

  assign nf_pop_s = nf_frame_pop_i && nf_frame_valid_o;
  assign net_pop_word_s =
    nf_pop_s && (nf_half_q || (nf_served_q + 1'b1 >= act_net_words_q[nf_sec_q]));

  assign nf_frame_valid_o =
    (net_count_q != '0) && (f_state_q == F_RUN) &&
    (nf_served_q < act_net_words_q[nf_sec_q]);
  assign nf_frame_data_o = nf_half_q
    ? net_fifo_mem_q[net_rd_ptr_q][511:256]
    : net_fifo_mem_q[net_rd_ptr_q][255:0];
  assign nf_frame_last_o = (nf_served_q + 1'b1 >= act_net_words_q[nf_sec_q]);

  // ---------------------------------------------------------------------
  // ctrl lane unpack: eight 64b lanes per 512b line
  // ---------------------------------------------------------------------
  logic [2:0]  cf_lane_idx_q;
  logic [31:0] cf_served_q;        // lanes served in the current ctrl section
  logic        cf_pop_s;

  assign cf_pop_s = cf_lane_pop_i && cf_lane_valid_o;
  assign ctrl_pop_word_s =
    cf_pop_s &&
    (
      (cf_lane_idx_q == 3'd7) ||
      (cf_served_q + 1'b1 >= act_ctrl_words_q[cf_sec_q])
    );

  assign cf_lane_valid_o =
    (ctrl_count_q != '0) && (f_state_q == F_RUN) &&
    (cf_served_q < act_ctrl_words_q[cf_sec_q]) &&
    // suppressed while a re-sync is pending so the loader/master cannot pop
    // stale lanes between the kick and the flush
    !dbg_pend_q && !go_pend_q;
  assign cf_lane_data_o = ctrl_fifo_mem_q[ctrl_rd_ptr_q][(cf_lane_idx_q * 64) +: 64];

  // ---------------------------------------------------------------------
  // Fetch issue. Directory first, then keep both FIFOs fed; ctrl bursts
  // have priority over net bursts (small, and the block load stalls on
  // lane 0). Bursts are "batch-then-fire": the desired length is the full
  // remaining need capped by FETCH_BURST_LINES, the target FIFO capacity,
  // and the next 4KB (64-line) boundary (the bridge issues one AXI burst
  // per request and AXI bursts must not cross 4KB; image section bases are
  // contiguous and not aligned). A burst is launched only once the target
  // FIFO has room for the whole desired length, so the burst size is never
  // chopped by transient free space (which used to fragment bursts to
  // ~3.5 lines avg and drown the bridge in handshake overhead). The
  // response stream therefore never stalls the bridge mid-burst (pops
  // during the burst only create more room). desired <= FIFO capacity by
  // construction, so the free-space threshold is always reachable: no
  // deadlock (while lines_left > 0 the consumer still has frames/lanes to
  // pop, and an empty FIFO satisfies any desired immediately).
  // ---------------------------------------------------------------------
  logic [NET_CNT_W-1:0]  net_free_s;
  logic [CTRL_CNT_W-1:0] ctrl_free_s;
  logic [31:0]           cf_fetch_lines_s;
  logic [31:0]           nf_fetch_lines_s;
  logic                  cf_fetch_want_s;
  logic                  nf_fetch_want_s;

  assign net_free_s = NET_CNT_W'(NET_FIFO_WORDS) - net_count_q;
  assign ctrl_free_s = CTRL_CNT_W'(CTRL_FIFO_WORDS) - ctrl_count_q;

  // lines remaining to the next 4KB (64-line) boundary; 64 when aligned
  function automatic logic [31:0] lines_to_4kb(input logic [31:0] line);
    return 32'd64 - {26'd0, line[5:0]};
  endfunction

  always_comb begin
    cf_fetch_lines_s = cf_lines_left_q;
    if (cf_fetch_lines_s > 32'(FETCH_BURST_LINES)) begin
      cf_fetch_lines_s = 32'(FETCH_BURST_LINES);
    end
    if (cf_fetch_lines_s > 32'(CTRL_FIFO_WORDS)) begin
      cf_fetch_lines_s = 32'(CTRL_FIFO_WORDS);
    end
    if (cf_fetch_lines_s > lines_to_4kb(cf_line_q)) begin
      cf_fetch_lines_s = lines_to_4kb(cf_line_q);
    end
    nf_fetch_lines_s = nf_lines_left_q;
    if (nf_fetch_lines_s > 32'(FETCH_BURST_LINES)) begin
      nf_fetch_lines_s = 32'(FETCH_BURST_LINES);
    end
    if (nf_fetch_lines_s > 32'(NET_FIFO_WORDS)) begin
      nf_fetch_lines_s = 32'(NET_FIFO_WORDS);
    end
    if (nf_fetch_lines_s > lines_to_4kb(nf_line_q)) begin
      nf_fetch_lines_s = lines_to_4kb(nf_line_q);
    end
  end

  assign cf_fetch_want_s =
    (f_state_q == F_RUN) && !flight_q && (cf_fetch_lines_s != '0) &&
    (32'(ctrl_free_s) >= cf_fetch_lines_s) && !dbg_pend_q && !go_pend_q;
  assign nf_fetch_want_s =
    (f_state_q == F_RUN) && !flight_q && !cf_fetch_want_s &&
    (nf_fetch_lines_s != '0) && (32'(net_free_s) >= nf_fetch_lines_s);

  assign fr_req_valid_o =
    (f_state_q == F_DIR_REQ) || cf_fetch_want_s || nf_fetch_want_s;
  assign fr_req_line_o = (f_state_q == F_DIR_REQ)
    ? 32'd0
    : (cf_fetch_want_s ? cf_line_q : nf_line_q);
  assign fr_req_count_o = (f_state_q == F_DIR_REQ)
    ? 9'(DIRECTORY_WORDS)
    : (cf_fetch_want_s ? 9'(cf_fetch_lines_s[8:0]) : 9'(nf_fetch_lines_s[8:0]));

  // Bursts are launched with space accounted for, so the response is
  // always acceptable (pops only free more).
  assign fr_resp_ready_o =
    (f_state_q == F_DIR_RECV) ||
    (flight_q && (flight_net_q ? (net_count_q != NET_CNT_W'(NET_FIFO_WORDS))
                               : (ctrl_count_q != CTRL_CNT_W'(CTRL_FIFO_WORDS))));

  // ---------------------------------------------------------------------
  // Main sequential logic
  // ---------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      f_state_q <= F_IDLE;
      flight_q <= 1'b0;
      flight_net_q <= 1'b0;
      dir_recv_cnt_q <= '0;
      cmp_slot_q <= '0;
      act_count_q <= '0;
      cf_sec_q <= '0;
      cf_line_q <= '0;
      cf_lines_left_q <= '0;
      cf_lanes_left_q <= '0;
      cf_lane_idx_q <= '0;
      cf_served_q <= '0;
      dbg_pend_q <= 1'b0;
      dbg_sec_q <= '0;
      go_pend_q <= 1'b0;
      nf_sec_q <= '0;
      nf_line_q <= '0;
      nf_lines_left_q <= '0;
      nf_frames_left_q <= '0;
      nf_half_q <= 1'b0;
      nf_served_q <= '0;
      net_wr_ptr_q <= '0;
      net_rd_ptr_q <= '0;
      net_count_q <= '0;
      ctrl_wr_ptr_q <= '0;
      ctrl_rd_ptr_q <= '0;
      ctrl_count_q <= '0;
    end else begin
      unique case ({net_push_s, net_pop_word_s})
        2'b10: net_count_q <= net_count_q + 1'b1;
        2'b01: net_count_q <= net_count_q - 1'b1;
        default: net_count_q <= net_count_q;
      endcase
      unique case ({ctrl_push_s, ctrl_pop_word_s})
        2'b10: ctrl_count_q <= ctrl_count_q + 1'b1;
        2'b01: ctrl_count_q <= ctrl_count_q - 1'b1;
        default: ctrl_count_q <= ctrl_count_q;
      endcase
      if (net_push_s) begin
        net_wr_ptr_q <= net_ptr_inc(net_wr_ptr_q);
      end
      if (net_pop_word_s) begin
        net_rd_ptr_q <= net_ptr_inc(net_rd_ptr_q);
      end
      if (ctrl_push_s) begin
        ctrl_wr_ptr_q <= ctrl_ptr_inc(ctrl_wr_ptr_q);
      end
      if (ctrl_pop_word_s) begin
        ctrl_rd_ptr_q <= ctrl_ptr_inc(ctrl_rd_ptr_q);
      end

      if (dbg_kick_i) begin
        dbg_pend_q <= 1'b1;
        dbg_sec_q <= dbg_sec_i;
      end
      // A GO while already running re-syncs the ctrl stream to section 0
      // (a preceding dbg readback session leaves the stream mid-section);
      // the net stream is untouched (its pop cursors stay section-aligned
      // in every flow). A GO in F_IDLE takes the directory-load path below.
      if (go_i && (f_state_q == F_RUN)) begin
        go_pend_q <= 1'b1;
      end

      unique case (f_state_q)
        F_IDLE: begin
          // a dbg kick before any GO (the pre-GO READ_P v4 readback) needs
          // the directory too; F_RUN's pending flush redirects the stream
          // to dbg_sec_q once the directory is in
          if (go_i || dbg_kick_i) begin
            f_state_q <= F_DIR_REQ;
          end
        end
        F_DIR_REQ: begin
          if (fr_req_ready_i) begin
            flight_q <= 1'b1;
            flight_net_q <= 1'b0;
            dir_recv_cnt_q <= '0;
            cmp_slot_q <= '0;
            act_count_q <= '0;
            f_state_q <= F_DIR_RECV;
          end
        end
        F_DIR_RECV: begin
          if (fr_resp_valid_i && fr_resp_ready_o) begin
            // 16 x 32b directory fields per 512b line, in order; field f of
            // slot s sits at 32b word 4s+f of the directory.
            for (int lane_idx = 0; lane_idx < 16; lane_idx++) begin
              dir_q[(int'(dir_recv_cnt_q) * 16) + lane_idx] <=
                fr_resp_data_i[(lane_idx * 32) +: 32];
            end
            dir_recv_cnt_q <= dir_recv_cnt_q + 1'b1;
            if (fr_resp_last_i) begin
              flight_q <= 1'b0;
              // The last line's dir_q write lands on this same edge, so the
              // first compaction cycle (which reads dir_q) must start on the
              // next state. cmp_slot_q/act_count_q were cleared in F_DIR_REQ.
              f_state_q <= F_DIR_COMPACT;
            end
          end
        end
        F_DIR_COMPACT: begin
          // serial compaction: inspect one directory slot per cycle; append
          // active slots (net_words != 0) to the active table in order. One
          // extra cycle is spent after the last slot so its table write
          // retires before F_RUN starts consuming act_*_q.
          if (dir_q[(int'(cmp_slot_q) * 4) + 3] != '0) begin
            act_ctrl_base_q[act_count_q]  <= dir_q[(int'(cmp_slot_q) * 4) + 0];
            act_ctrl_words_q[act_count_q] <= dir_q[(int'(cmp_slot_q) * 4) + 1];
            act_net_base_q[act_count_q]   <= dir_q[(int'(cmp_slot_q) * 4) + 2];
            act_net_words_q[act_count_q]  <= dir_q[(int'(cmp_slot_q) * 4) + 3];
            act_count_q <= act_count_q + 1'b1;
          end
          if (cmp_slot_q >= SLOT_IDX_W'(BLOCK_SLOTS - 1)) begin
            cf_sec_q <= '0;
            nf_sec_q <= '0;
            f_state_q <= F_RUN;
          end else begin
            cmp_slot_q <= cmp_slot_q + 1'b1;
          end
        end
        F_RUN: begin
          // ---- ctrl stream section walk (reload when fully consumed AND
          // fully fetched; one section per cycle, wrap at the end) ----
          if ((cf_lines_left_q == '0) && (cf_lanes_left_q == '0) &&
              !flight_q && (cf_sec_q < act_count_q)) begin
            cf_line_q <= act_ctrl_base_q[cf_sec_q];
            cf_lines_left_q <= (act_ctrl_words_q[cf_sec_q] + 32'd7) >> 3;
            cf_lanes_left_q <= act_ctrl_words_q[cf_sec_q];
            cf_served_q <= '0;
            cf_lane_idx_q <= '0;
          end
          // ---- net stream section walk ----
          if ((nf_lines_left_q == '0) && (nf_frames_left_q == '0) &&
              !flight_q && (nf_sec_q < act_count_q)) begin
            nf_line_q <= act_net_base_q[nf_sec_q];
            nf_lines_left_q <= (act_net_words_q[nf_sec_q] + 32'd1) >> 1;
            nf_frames_left_q <= act_net_words_q[nf_sec_q];
            nf_served_q <= '0;
            nf_half_q <= 1'b0;
          end
          // ---- burst launch (ctrl has priority) ----
          if (!flight_q) begin
            if (cf_fetch_want_s && fr_req_ready_i) begin
              flight_q <= 1'b1;
              flight_net_q <= 1'b0;
              cf_line_q <= cf_line_q + cf_fetch_lines_s;
              cf_lines_left_q <= cf_lines_left_q - cf_fetch_lines_s;
            end else if (nf_fetch_want_s && fr_req_ready_i) begin
              flight_q <= 1'b1;
              flight_net_q <= 1'b1;
              nf_line_q <= nf_line_q + nf_fetch_lines_s;
              nf_lines_left_q <= nf_lines_left_q - nf_fetch_lines_s;
            end
          end else if (fr_resp_valid_i && fr_resp_ready_o && fr_resp_last_i) begin
            flight_q <= 1'b0;
          end

          // ---- ctrl lane pop accounting ----
          if (cf_pop_s) begin
            cf_served_q <= cf_served_q + 1'b1;
            cf_lanes_left_q <= cf_lanes_left_q - 1'b1;
            if ((cf_lane_idx_q == 3'd7) ||
                (cf_served_q + 1'b1 >= act_ctrl_words_q[cf_sec_q])) begin
              cf_lane_idx_q <= '0;
            end else begin
              cf_lane_idx_q <= cf_lane_idx_q + 1'b1;
            end
            // section end: hop to the next section (wrap for the next
            // iteration); the walk above reloads it on a later cycle
            if (cf_served_q + 1'b1 >= act_ctrl_words_q[cf_sec_q]) begin
              cf_sec_q <=
                (cf_sec_q + 1'b1 >= act_count_q) ? '0 : (cf_sec_q + 1'b1);
            end
          end

          // ---- net frame pop accounting ----
          if (nf_pop_s) begin
            nf_served_q <= nf_served_q + 1'b1;
            nf_frames_left_q <= nf_frames_left_q - 1'b1;
            if (net_pop_word_s) begin
              nf_half_q <= 1'b0;
            end else begin
              nf_half_q <= ~nf_half_q;
            end
            if (nf_served_q + 1'b1 >= act_net_words_q[nf_sec_q]) begin
              nf_sec_q <=
                (nf_sec_q + 1'b1 >= act_count_q) ? '0 : (nf_sec_q + 1'b1);
            end
          end

          // ---- ctrl stream re-sync flush (LAST in F_RUN so it overrides
          // the section walk / pop accounting above on the same cycle) ----
          // On a dbg re-serve or a GO re-sync: drop the stale prefetch
          // (after a self-run the ctrl FIFO holds section-0 lines; after a
          // dbg load it holds the section's unconsumed spill tail), rewind
          // the cursors, and let the section walk above restart the stream
          // from act_ctrl_base_q[target] on the next cycle. The directory
          // tables survive a self-run (only F_DIR_RECV/F_DIR_COMPACT write
          // them), so no directory reload is needed. Waits for the
          // in-flight burst to retire so late responses cannot land in the
          // flushed FIFO; cf_fetch_want_s/cf_lane_valid_o stay suppressed
          // while pending.
          if ((dbg_pend_q || go_pend_q) && !flight_q) begin
            cf_sec_q <= dbg_pend_q ? SEC_PTR_W'({1'b0, dbg_sec_q}) : '0;
            cf_line_q <= '0;
            cf_lines_left_q <= '0;
            cf_lanes_left_q <= '0;
            cf_served_q <= '0;
            cf_lane_idx_q <= '0;
            ctrl_count_q <= '0;
            ctrl_rd_ptr_q <= '0;
            ctrl_wr_ptr_q <= '0;
            dbg_pend_q <= 1'b0;
            go_pend_q <= 1'b0;
          end
        end
        default: begin
          f_state_q <= F_IDLE;
        end
      endcase
    end
  end

`ifndef SYNTHESIS
  // ---- NET-on-DDR self-run debug probes (sim only) ----
  fetch_state_e f_dbg_q;
  always_ff @(posedge clk) begin
    f_dbg_q <= f_state_q;
    if (rst_n && (f_state_q != f_dbg_q)) begin
      $display("%t FETCH state %0d -> %0d act=%0d", $time, f_dbg_q, f_state_q, act_count_q);
    end
    if (rst_n && go_i) begin
      $display("%t FETCH GO", $time);
    end
    if (rst_n && dbg_kick_i) begin
      $display("%t FETCH DBG kick sec=%0d", $time, dbg_sec_i);
    end
    if (rst_n && dbg_pend_q && !flight_q) begin
      $display("%t FETCH DBG flush -> sec=%0d", $time, dbg_sec_q);
    end
    if (rst_n && go_pend_q && !flight_q) begin
      $display("%t FETCH GO resync flush (ctrl -> sec 0)", $time);
    end
    if (rst_n && fr_req_valid_o && fr_req_ready_i) begin
      $display("%t FETCH burst line=%0d count=%0d tgt=%0s",
        $time, fr_req_line_o, fr_req_count_o, cf_fetch_want_s ? "ctrl" : "net");
    end
    if (rst_n && nf_frame_pop_i && nf_frame_last_o) begin
      $display("%t FETCH net block frames done sec=%0d served=%0d",
        $time, nf_sec_q, nf_served_q + 1);
    end
    if (rst_n && cf_lane_pop_i && (cf_served_q + 1'b1 >= act_ctrl_words_q[cf_sec_q])) begin
      $display("%t FETCH ctrl block lanes done sec=%0d", $time, cf_sec_q);
    end
  end
`endif

  // ---------------------------------------------------------------------
  // Performance counters (status mirror M12/M13): raw-fetch burst-length
  // histogram, total bursts/lines, FIFO full cycles and watermarks.
  // Cleared on go_i, free-running afterwards.
  // ---------------------------------------------------------------------
  logic [15:0] perf_burst_lt8_q;
  logic [15:0] perf_burst_8_31_q;
  logic [15:0] perf_burst_32_63_q;
  logic [15:0] perf_burst_eq64_q;
  logic [31:0] perf_bursts_q;
  logic [31:0] perf_lines_q;
  logic [31:0] perf_net_full_cyc_q;
  logic [31:0] perf_ctrl_full_cyc_q;
  logic [NET_CNT_W-1:0] perf_net_wm_q;
  logic [CTRL_CNT_W-1:0] perf_ctrl_wm_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      perf_burst_lt8_q <= '0;
      perf_burst_8_31_q <= '0;
      perf_burst_32_63_q <= '0;
      perf_burst_eq64_q <= '0;
      perf_bursts_q <= '0;
      perf_lines_q <= '0;
      perf_net_full_cyc_q <= '0;
      perf_ctrl_full_cyc_q <= '0;
      perf_net_wm_q <= '0;
      perf_ctrl_wm_q <= '0;
    end else if (go_i) begin
      perf_burst_lt8_q <= '0;
      perf_burst_8_31_q <= '0;
      perf_burst_32_63_q <= '0;
      perf_burst_eq64_q <= '0;
      perf_bursts_q <= '0;
      perf_lines_q <= '0;
      perf_net_full_cyc_q <= '0;
      perf_ctrl_full_cyc_q <= '0;
      perf_net_wm_q <= '0;
      perf_ctrl_wm_q <= '0;
    end else begin
      if (fr_req_valid_o && fr_req_ready_i) begin
        perf_bursts_q <= perf_bursts_q + 1'b1;
        perf_lines_q <= perf_lines_q + 32'(fr_req_count_o);
        if (fr_req_count_o < 9'd8) begin
          perf_burst_lt8_q <= perf_burst_lt8_q + 1'b1;
        end else if (fr_req_count_o < 9'd32) begin
          perf_burst_8_31_q <= perf_burst_8_31_q + 1'b1;
        end else if (fr_req_count_o < 9'd64) begin
          perf_burst_32_63_q <= perf_burst_32_63_q + 1'b1;
        end else begin
          perf_burst_eq64_q <= perf_burst_eq64_q + 1'b1;
        end
      end
      if (net_count_q == NET_CNT_W'(NET_FIFO_WORDS)) begin
        perf_net_full_cyc_q <= perf_net_full_cyc_q + 1'b1;
      end
      if (ctrl_count_q == CTRL_CNT_W'(CTRL_FIFO_WORDS)) begin
        perf_ctrl_full_cyc_q <= perf_ctrl_full_cyc_q + 1'b1;
      end
      if (net_count_q > perf_net_wm_q) begin
        perf_net_wm_q <= net_count_q;
      end
      if (ctrl_count_q > perf_ctrl_wm_q) begin
        perf_ctrl_wm_q <= ctrl_count_q;
      end
    end
  end

  assign nf_perf_probe_o[8:0] = NET_CNT_W'(perf_net_wm_q);
  assign nf_perf_probe_o[14:9] = CTRL_CNT_W'(perf_ctrl_wm_q);
  assign nf_perf_probe_o[30:15] = perf_burst_lt8_q;
  assign nf_perf_probe_o[46:31] = perf_burst_8_31_q;
  assign nf_perf_probe_o[62:47] = perf_burst_32_63_q;
  assign nf_perf_probe_o[78:63] = perf_burst_eq64_q;
  assign nf_perf_probe_o[110:79] = perf_bursts_q;
  assign nf_perf_probe_o[142:111] = perf_lines_q;
  assign nf_perf_probe_o[174:143] = perf_net_full_cyc_q;
  assign nf_perf_probe_o[206:175] = perf_ctrl_full_cyc_q;
  assign nf_perf_probe_o[223:207] = '0;

  // debug observability probes (combinational taps; no functional impact)
  assign dbg_f_state_o = f_state_q;
  assign dbg_cf_sec_o = cf_sec_q;
  assign dbg_nf_sec_o = nf_sec_q;
  assign dbg_cf_line_o = cf_line_q;
  assign dbg_nf_line_o = nf_line_q;
  assign dbg_flight_o = flight_q;

endmodule

`endif
