`ifndef FPGA_SOFTMAX_ROW_READER_SV
`define FPGA_SOFTMAX_ROW_READER_SV

module softmax_row_reader #(
  parameter int BANK_COUNT = 16,
  parameter int BANK_DEPTH = 65536,
  parameter int DATA_W = 32,
  parameter int RMS_V_W = 64,
  parameter int BURST_LINES = 64,
  // Global DDR row index width (segment base + window-relative row); the
  // window-relative counters stay at ADDR_W.
  parameter int DDR_ROW_W = 20,
  localparam int BURST_LINES_RAW = (BURST_LINES < 1) ? 1 : BURST_LINES,
  localparam int BURST_LINES_CLAMPED = (BURST_LINES_RAW > 256) ? 256 : BURST_LINES_RAW,
  localparam int ROW_BUFFER_DEPTH = 8 * BURST_LINES_CLAMPED
) (
  input  logic                                       clk,
  input  logic                                       rst_n,
  input  logic                                       enable_i,
  input  logic                                       update_mode_i,
  input  logic [$clog2(BANK_DEPTH + 1)-1:0]          bank_valid_len_i [0:BANK_COUNT-1],
  // Blocked-GRADP v2: segment row base (row units, added only to the DDR
  // request row; entry addresses stay window-relative). 0 in monolithic
  // mode, which reproduces v1.0.5 exactly.
  input  logic [DDR_ROW_W-1:0]                         rd_row_base_i,
  input  logic                                       bank_accept_i [0:BANK_COUNT-1],
  output logic                                       ddr_rd_req_valid_o,
  input  logic                                       ddr_rd_req_ready_i,
  output logic [DDR_ROW_W-1:0]                         ddr_rd_req_row_o,
  output logic [$clog2(ROW_BUFFER_DEPTH + 1)-1:0]    ddr_rd_req_count_o,
  input  logic                                       ddr_rd_resp_valid_i,
  output logic                                       ddr_rd_resp_ready_o,
  input  logic [BANK_COUNT-1:0]                      ddr_rd_resp_bank_mask_i,
  input  logic                                       ddr_rd_resp_last_i,
  input  logic signed [DATA_W-1:0]                   ddr_rd_logits_i [0:BANK_COUNT-1],
  input  logic signed [RMS_V_W-1:0]                  ddr_rd_rms_i [0:BANK_COUNT-1],
  output logic [$clog2(BANK_DEPTH + 1)+DATA_W+RMS_V_W-1:0] bank_entry_o [0:BANK_COUNT-1],
  output logic                                       bank_entry_valid_o [0:BANK_COUNT-1],
  // Blocked-GRADP v2: a read request is outstanding on the shared bridge
  // channel (softmax_ddr uses it to arbitrate the meta row stream).
  output logic                                       resp_pending_o,
  output logic                                       done_o,
  // Per-bank input-stream-done: every row emitted to this bank has been
  // accepted by it. Each bank engine's trailing-group flush keys off "no
  // more input for THIS bank"; a single shared done would deadlock against
  // a sibling bank stalled on p backpressure, because that sibling's stall
  // traces back to the very row the flush must produce.
  output logic                                       done_per_bank_o [0:BANK_COUNT-1]
);
  localparam int ADDR_W = $clog2(BANK_DEPTH + 1);
  localparam int ENTRY_W = ADDR_W + DATA_W + RMS_V_W;
  localparam int DDR_LINE_SLOTS = 8;
  localparam int AXI_BURST_LINES = BURST_LINES_CLAMPED;
  localparam int AXI_BURST_ROW_SLOTS = DDR_LINE_SLOTS * AXI_BURST_LINES;
  localparam int READ_BATCH_TARGET = AXI_BURST_LINES;
  localparam int BUF_PTR_W = (ROW_BUFFER_DEPTH <= 1) ? 1 : $clog2(ROW_BUFFER_DEPTH);
  localparam int BUF_CNT_W = $clog2(ROW_BUFFER_DEPTH + 1);
  localparam int PACK_ROW_LSB = 0;
  localparam int PACK_ACTIVE_LSB = PACK_ROW_LSB + ADDR_W;
  localparam int PACK_LOGITS_LSB = PACK_ACTIVE_LSB + BANK_COUNT;
  localparam int PACK_RMS_LSB = PACK_LOGITS_LSB + (BANK_COUNT * DATA_W);
  localparam int ENTRY_PACK_W = PACK_RMS_LSB + (BANK_COUNT * RMS_V_W);

  // Next row to request from DDR. Kept as row_idx_q for existing cocotb debug probes.
  logic [ADDR_W-1:0] row_idx_q;
  logic [ADDR_W-1:0] pending_row_q;
  logic pending_resp_q;
  logic [BUF_CNT_W-1:0] pending_count_q;
  logic [BUF_PTR_W-1:0] buf_wr_ptr_q;
  logic [BUF_PTR_W-1:0] buf_rd_ptr_q;
  logic [BUF_CNT_W-1:0] buf_count_q;
  logic [BUF_CNT_W-1:0] buf_mem_count_q;
  logic [BUF_CNT_W-1:0] free_count_s;
  logic [BUF_CNT_W-1:0] line_remaining_s;
  logic [BUF_CNT_W-1:0] remaining_count_s;
  logic [BUF_CNT_W-1:0] req_count_s;
  logic req_pipe_busy_s;
  logic req_pipe_start_s;
  logic read_done_q;
  // Multi-outstanding request tracking (up to PEND_DEPTH requests in
  // flight on the shared bridge channel; the bridge OT pipeline delivers
  // responses strictly in request order, so a FIFO of per-request beat
  // counts suffices to label arriving rows).
  localparam int PEND_DEPTH = 4;
  localparam int PEND_PTR_W = (PEND_DEPTH <= 1) ? 1 : $clog2(PEND_DEPTH);
  localparam int PEND_CNT_W = $clog2(PEND_DEPTH + 1);
  localparam int RMS_PER_DDR_LINE = 512 / RMS_V_W;
  localparam int RMS_HALF_COUNT = (BANK_COUNT + RMS_PER_DDR_LINE - 1) / RMS_PER_DDR_LINE;
  logic [BUF_CNT_W-1:0] pend_count_q [0:PEND_DEPTH-1];  // beats left per pending request
  logic [PEND_PTR_W-1:0] pend_rd_ptr_q;
  logic [PEND_PTR_W-1:0] pend_wr_ptr_q;
  logic [PEND_CNT_W-1:0] pend_level_q;
  logic [BUF_CNT_W-1:0] pend_beats_total_q;             // buffer-entry reservation
  logic pend_full_s;
  logic pend_pop_fire_s;
  logic [BUF_CNT_W-1:0] fire_beats_s;
  logic [BUF_CNT_W-1:0] fire_free_s;
  logic [BUF_CNT_W-1:0] fire_free_rows_s;
  // Per-bank emitted/accepted row counters for done_per_bank_o
  logic [ADDR_W-1:0] bank_emit_cnt_q [0:BANK_COUNT-1];
  logic [ADDR_W-1:0] bank_accept_cnt_q [0:BANK_COUNT-1];
  logic [BANK_COUNT-1:0] emit_active_mask_s;
  logic [BANK_COUNT-1:0] pending_active_mask_s;
  logic [ENTRY_PACK_W-1:0] push_word_s;
  logic [ENTRY_PACK_W-1:0] buf_rd_word_q;
  logic buf_out_valid_q;
  logic mem_read_fire_s;
  logic [ADDR_W-1:0] emit_row_s;
  logic signed [DATA_W-1:0] emit_logits_s [0:BANK_COUNT-1];
  logic signed [RMS_V_W-1:0] emit_rms_s [0:BANK_COUNT-1];
  logic req_fire_s;
  logic push_fire_s;
  logic pop_fire_s;
  logic buf_has_space_s;

  (* ram_style = "block" *) logic [ENTRY_PACK_W-1:0] entry_buf_q [0:ROW_BUFFER_DEPTH-1];

  localparam int MAX_STAGE0_COUNT = (BANK_COUNT + 1) / 2;
  localparam int MAX_STAGE1_COUNT = (MAX_STAGE0_COUNT + 1) / 2;

  logic req_s0_valid_q;
  logic req_s1_valid_q;
  logic req_s2_valid_q;
  logic req_s3_valid_q;
  logic [ADDR_W-1:0] req_s0_max_q [0:MAX_STAGE0_COUNT-1];
  logic [ADDR_W-1:0] req_s1_max_q [0:MAX_STAGE1_COUNT-1];
  logic [ADDR_W-1:0] req_s2_max_q;
  logic [ADDR_W-1:0] req_s0_row_q, req_s1_row_q, req_s2_row_q, req_s3_row_q;
  logic [BUF_CNT_W-1:0] req_s0_free_q, req_s1_free_q, req_s2_free_q, req_s3_free_q;
  logic [BUF_CNT_W-1:0] req_s0_line_q, req_s1_line_q, req_s2_line_q, req_s3_line_q;
  logic [BUF_CNT_W-1:0] req_s3_remaining_q;
  logic req_valid_q;
  logic [ADDR_W-1:0] req_row_q;
  logic [BUF_CNT_W-1:0] req_count_q;

  function automatic logic [ADDR_W-1:0] max_addr(
    input logic [ADDR_W-1:0] lhs,
    input logic [ADDR_W-1:0] rhs
  );
    begin
      return (lhs > rhs) ? lhs : rhs;
    end
  endfunction

  function automatic logic [BUF_CNT_W-1:0] min_count(
    input logic [BUF_CNT_W-1:0] lhs,
    input logic [BUF_CNT_W-1:0] rhs
  );
    begin
      return (lhs < rhs) ? lhs : rhs;
    end
  endfunction

  logic all_active_accept_s;
  logic row_active_s [0:BANK_COUNT-1];

  assign emit_row_s = buf_rd_word_q[PACK_ROW_LSB +: ADDR_W];
  assign emit_active_mask_s =
    buf_out_valid_q ? buf_rd_word_q[PACK_ACTIVE_LSB +: BANK_COUNT] : '0;

  generate
    for (genvar bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin : GEN_UNPACK_ENTRY
      assign emit_logits_s[bank_idx] =
        buf_rd_word_q[PACK_LOGITS_LSB + (bank_idx * DATA_W) +: DATA_W];
      assign emit_rms_s[bank_idx] =
        buf_rd_word_q[PACK_RMS_LSB + (bank_idx * RMS_V_W) +: RMS_V_W];
    end
  endgenerate

  generate
    for (genvar done_bank_idx = 0; done_bank_idx < BANK_COUNT; done_bank_idx++) begin : GEN_BANK_DONE
      assign done_per_bank_o[done_bank_idx] =
        enable_i && read_done_q && !pending_resp_q && !req_pipe_busy_s &&
        (bank_emit_cnt_q[done_bank_idx] == bank_accept_cnt_q[done_bank_idx]);
    end
  endgenerate

  function automatic logic [BUF_PTR_W-1:0] ptr_inc(
    input logic [BUF_PTR_W-1:0] ptr
  );
    if (ptr == (ROW_BUFFER_DEPTH - 1)) begin
      return '0;
    end
    return ptr + 1'b1;
  endfunction

  function automatic logic [BUF_CNT_W-1:0] burst_slots_left(
    input logic [ADDR_W-1:0] row_idx
  );
    int unsigned slot_idx;
    begin
      slot_idx = int'(row_idx) % READ_BATCH_TARGET;
      burst_slots_left = BUF_CNT_W'(READ_BATCH_TARGET - slot_idx);
    end
  endfunction

  initial begin
    assert (ROW_BUFFER_DEPTH >= 1)
      else $error(1, "softmax_row_reader: ROW_BUFFER_DEPTH must be >= 1");
  end

  always_ff @(posedge clk) begin
    if (push_fire_s) begin
      entry_buf_q[buf_wr_ptr_q] <= push_word_s;
    end
    if (mem_read_fire_s) begin
      buf_rd_word_q <= entry_buf_q[buf_rd_ptr_q];
    end
  end

  always_comb begin
    all_active_accept_s = 1'b1;
    pending_active_mask_s = '0;
    push_word_s = '0;
    for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
      row_active_s[bank_idx] = emit_active_mask_s[bank_idx];
      if (row_active_s[bank_idx]) begin
        all_active_accept_s &= bank_accept_i[bank_idx];
      end
      pending_active_mask_s[bank_idx] =
        (pending_row_q < bank_valid_len_i[bank_idx]) && ddr_rd_resp_bank_mask_i[bank_idx];
      bank_entry_valid_o[bank_idx] = 1'b0;
      bank_entry_o[bank_idx] = '0;
    end
    push_word_s[PACK_ROW_LSB +: ADDR_W] = pending_row_q;
    push_word_s[PACK_ACTIVE_LSB +: BANK_COUNT] = pending_active_mask_s;
    for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
      push_word_s[PACK_LOGITS_LSB + (bank_idx * DATA_W) +: DATA_W] = ddr_rd_logits_i[bank_idx];
      push_word_s[PACK_RMS_LSB + (bank_idx * RMS_V_W) +: RMS_V_W] = ddr_rd_rms_i[bank_idx];
    end

    pop_fire_s = enable_i && buf_out_valid_q && all_active_accept_s;
    free_count_s = BUF_CNT_W'(ROW_BUFFER_DEPTH) - buf_count_q - pend_beats_total_q +
      (pop_fire_s ? BUF_CNT_W'(1) : '0);
    line_remaining_s = burst_slots_left(row_idx_q);
    remaining_count_s = req_s3_remaining_q;
    req_count_s = req_count_q;
    buf_has_space_s = (free_count_s != '0);
    pend_full_s = (pend_level_q == PEND_CNT_W'(PEND_DEPTH));
    req_pipe_busy_s =
      req_s0_valid_q || req_s1_valid_q || req_s2_valid_q || req_s3_valid_q || req_valid_q;
    req_pipe_start_s =
      enable_i && !read_done_q && !req_pipe_busy_s && !pend_full_s && (free_count_s != '0);
    ddr_rd_req_valid_o = req_valid_q;
    // Segment-descriptor base is added only here: every internal row
    // (entry addr, burst alignment, ragged gating) stays window-relative.
    ddr_rd_req_row_o = DDR_ROW_W'(req_row_q + rd_row_base_i);
    ddr_rd_req_count_o = req_count_q;
    ddr_rd_resp_ready_o = (pend_level_q != '0) && buf_has_space_s;
    resp_pending_o = (pend_level_q != '0);
    req_fire_s = req_valid_q && ddr_rd_req_ready_i;
    push_fire_s = ddr_rd_resp_valid_i && ddr_rd_resp_ready_o;
    // multi-outstanding fire-time bookkeeping: reserve buffer entries in
    // beats (1 entry per response beat), convert back to rows for the
    // request-count computation, and never trust the pipe-stage snapshot
    // more than the current free count (beats may arrive mid-computation)
    fire_beats_s = BUF_CNT_W'(int'(req_count_q) *
      ((update_mode_i && (RMS_HALF_COUNT > 1)) ? RMS_HALF_COUNT : 1));
    fire_free_s = min_count(req_s3_free_q, free_count_s);
    fire_free_rows_s = (update_mode_i && (RMS_HALF_COUNT > 1))
      ? BUF_CNT_W'(int'(fire_free_s) / RMS_HALF_COUNT)
      : fire_free_s;
    pend_pop_fire_s = push_fire_s && (pend_count_q[pend_rd_ptr_q] <= BUF_CNT_W'(1));
    // probe-compatible aliases (single-outstanding era names)
    pending_resp_q = (pend_level_q != '0);
    pending_count_q = pend_count_q[pend_rd_ptr_q];
    mem_read_fire_s = enable_i && ((buf_out_valid_q == 1'b0) || pop_fire_s) &&
      (buf_mem_count_q != '0);
    done_o = enable_i && read_done_q && !pending_resp_q && !req_pipe_busy_s && (buf_count_q == '0);

    if (pop_fire_s) begin
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        if (row_active_s[bank_idx]) begin
          bank_entry_valid_o[bank_idx] = 1'b1;
          bank_entry_o[bank_idx] = {
            emit_row_s,
            emit_logits_s[bank_idx],
            update_mode_i ? emit_rms_s[bank_idx] : '0
          };
        end
      end
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      row_idx_q <= '0;
      pending_row_q <= '0;
      pend_rd_ptr_q <= '0;
      pend_wr_ptr_q <= '0;
      pend_level_q <= '0;
      pend_beats_total_q <= '0;
      for (int pend_idx = 0; pend_idx < PEND_DEPTH; pend_idx++) begin
        pend_count_q[pend_idx] <= '0;
      end
      buf_wr_ptr_q <= '0;
      buf_rd_ptr_q <= '0;
      buf_count_q <= '0;
      buf_mem_count_q <= '0;
      buf_out_valid_q <= 1'b0;
      read_done_q <= 1'b0;
      req_s0_valid_q <= 1'b0;
      req_s1_valid_q <= 1'b0;
      req_s2_valid_q <= 1'b0;
      req_s3_valid_q <= 1'b0;
      req_valid_q <= 1'b0;
      req_s0_row_q <= '0;
      req_s1_row_q <= '0;
      req_s2_row_q <= '0;
      req_s3_row_q <= '0;
      req_s0_free_q <= '0;
      req_s1_free_q <= '0;
      req_s2_free_q <= '0;
      req_s3_free_q <= '0;
      req_s0_line_q <= '0;
      req_s1_line_q <= '0;
      req_s2_line_q <= '0;
      req_s3_line_q <= '0;
      req_s2_max_q <= '0;
      req_s3_remaining_q <= '0;
      req_row_q <= '0;
      req_count_q <= '0;
      for (int stage_idx = 0; stage_idx < MAX_STAGE0_COUNT; stage_idx++) begin
        req_s0_max_q[stage_idx] <= '0;
      end
      for (int stage_idx = 0; stage_idx < MAX_STAGE1_COUNT; stage_idx++) begin
        req_s1_max_q[stage_idx] <= '0;
      end
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        bank_emit_cnt_q[bank_idx] <= '0;
        bank_accept_cnt_q[bank_idx] <= '0;
      end
    end else if (!enable_i) begin
      row_idx_q <= '0;
      pending_row_q <= '0;
      pend_rd_ptr_q <= '0;
      pend_wr_ptr_q <= '0;
      pend_level_q <= '0;
      pend_beats_total_q <= '0;
      for (int pend_idx = 0; pend_idx < PEND_DEPTH; pend_idx++) begin
        pend_count_q[pend_idx] <= '0;
      end
      buf_wr_ptr_q <= '0;
      buf_rd_ptr_q <= '0;
      buf_count_q <= '0;
      buf_mem_count_q <= '0;
      buf_out_valid_q <= 1'b0;
      read_done_q <= 1'b0;
      req_s0_valid_q <= 1'b0;
      req_s1_valid_q <= 1'b0;
      req_s2_valid_q <= 1'b0;
      req_s3_valid_q <= 1'b0;
      req_valid_q <= 1'b0;
      req_s0_row_q <= '0;
      req_s1_row_q <= '0;
      req_s2_row_q <= '0;
      req_s3_row_q <= '0;
      req_s0_free_q <= '0;
      req_s1_free_q <= '0;
      req_s2_free_q <= '0;
      req_s3_free_q <= '0;
      req_s0_line_q <= '0;
      req_s1_line_q <= '0;
      req_s2_line_q <= '0;
      req_s3_line_q <= '0;
      req_s2_max_q <= '0;
      req_s3_remaining_q <= '0;
      req_row_q <= '0;
      req_count_q <= '0;
      for (int stage_idx = 0; stage_idx < MAX_STAGE0_COUNT; stage_idx++) begin
        req_s0_max_q[stage_idx] <= '0;
      end
      for (int stage_idx = 0; stage_idx < MAX_STAGE1_COUNT; stage_idx++) begin
        req_s1_max_q[stage_idx] <= '0;
      end
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        bank_emit_cnt_q[bank_idx] <= '0;
        bank_accept_cnt_q[bank_idx] <= '0;
      end
    end else begin
      logic [ADDR_W-1:0] s2_max_next;
      logic [ADDR_W-1:0] remaining_raw;
      logic [BUF_CNT_W-1:0] remaining_next;
      logic [BUF_CNT_W-1:0] req_count_next;

      if (req_fire_s) begin
        if (pend_level_q == '0) begin
          // (re)sync the response row label; while requests are pending it
          // simply runs contiguously across request boundaries
          pending_row_q <= req_row_q;
        end
        pend_count_q[pend_wr_ptr_q] <= fire_beats_s;
        pend_wr_ptr_q <= pend_wr_ptr_q + 1'b1;
        row_idx_q <= req_row_q + ADDR_W'(req_count_q);
        req_valid_q <= 1'b0;
      end else if (req_valid_q) begin
        req_valid_q <= 1'b1;
      end else begin
        req_valid_q <= 1'b0;
        if (req_s3_valid_q) begin
          req_count_next = min_count(fire_free_rows_s, req_s3_remaining_q);
          req_count_next = min_count(req_count_next, req_s3_line_q);
          req_count_next = min_count(req_count_next, BUF_CNT_W'(READ_BATCH_TARGET));
          if (req_s3_remaining_q == '0) begin
            read_done_q <= 1'b1;
          end else if (
            (req_count_next != '0) &&
            ((fire_free_rows_s >= BUF_CNT_W'(READ_BATCH_TARGET)) ||
             (req_s3_line_q <= fire_free_rows_s) ||
             (req_s3_remaining_q <= fire_free_rows_s))
          ) begin
            req_valid_q <= 1'b1;
            req_row_q <= req_s3_row_q;
            req_count_q <= req_count_next;
            read_done_q <= 1'b0;
          end
        end
      end

      unique case ({req_fire_s, pend_pop_fire_s})
        2'b10: pend_level_q <= pend_level_q + 1'b1;
        2'b01: pend_level_q <= pend_level_q - 1'b1;
        default: pend_level_q <= pend_level_q;
      endcase
      pend_beats_total_q <= pend_beats_total_q
        + (req_fire_s ? fire_beats_s : '0)
        - (push_fire_s ? BUF_CNT_W'(1) : '0);

      if (req_pipe_start_s) begin
        req_s0_valid_q <= 1'b1;
        req_s0_row_q <= row_idx_q;
        req_s0_free_q <= free_count_s;
        req_s0_line_q <= line_remaining_s;
        for (int stage_idx = 0; stage_idx < MAX_STAGE0_COUNT; stage_idx++) begin
          if (((2 * stage_idx) + 1) < BANK_COUNT) begin
            req_s0_max_q[stage_idx] <=
              max_addr(bank_valid_len_i[2 * stage_idx], bank_valid_len_i[(2 * stage_idx) + 1]);
          end else begin
            req_s0_max_q[stage_idx] <= bank_valid_len_i[2 * stage_idx];
          end
        end
      end else begin
        req_s0_valid_q <= 1'b0;
      end

      req_s1_valid_q <= req_s0_valid_q;
      req_s1_row_q <= req_s0_row_q;
      req_s1_free_q <= req_s0_free_q;
      req_s1_line_q <= req_s0_line_q;
      for (int stage_idx = 0; stage_idx < MAX_STAGE1_COUNT; stage_idx++) begin
        if (((2 * stage_idx) + 1) < MAX_STAGE0_COUNT) begin
          req_s1_max_q[stage_idx] <=
            max_addr(req_s0_max_q[2 * stage_idx], req_s0_max_q[(2 * stage_idx) + 1]);
        end else begin
          req_s1_max_q[stage_idx] <= req_s0_max_q[2 * stage_idx];
        end
      end

      s2_max_next = '0;
      for (int stage_idx = 0; stage_idx < MAX_STAGE1_COUNT; stage_idx++) begin
        s2_max_next = max_addr(s2_max_next, req_s1_max_q[stage_idx]);
      end
      req_s2_valid_q <= req_s1_valid_q;
      req_s2_row_q <= req_s1_row_q;
      req_s2_free_q <= req_s1_free_q;
      req_s2_line_q <= req_s1_line_q;
      req_s2_max_q <= s2_max_next;

      remaining_next = '0;
      remaining_raw = '0;
      if (req_s2_row_q < req_s2_max_q) begin
        remaining_raw = req_s2_max_q - req_s2_row_q;
        if (remaining_raw > ADDR_W'(ROW_BUFFER_DEPTH)) begin
          remaining_next = BUF_CNT_W'(ROW_BUFFER_DEPTH);
        end else begin
          remaining_next = BUF_CNT_W'(remaining_raw);
        end
      end
      req_s3_valid_q <= req_s2_valid_q;
      req_s3_row_q <= req_s2_row_q;
      req_s3_free_q <= req_s2_free_q;
      req_s3_line_q <= req_s2_line_q;
      req_s3_remaining_q <= remaining_next;

      unique case ({push_fire_s, pop_fire_s})
        2'b01: buf_count_q <= buf_count_q - 1'b1;
        2'b10: buf_count_q <= buf_count_q + 1'b1;
        default: buf_count_q <= buf_count_q;
      endcase

      if (push_fire_s) begin
        for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
          if (pending_active_mask_s[bank_idx]) begin
            bank_emit_cnt_q[bank_idx] <= bank_emit_cnt_q[bank_idx] + 1'b1;
          end
        end
      end
      if (pop_fire_s) begin
        for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
          if (row_active_s[bank_idx]) begin
            bank_accept_cnt_q[bank_idx] <= bank_accept_cnt_q[bank_idx] + 1'b1;
          end
        end
      end

      unique case ({push_fire_s, mem_read_fire_s})
        2'b01: buf_mem_count_q <= buf_mem_count_q - 1'b1;
        2'b10: buf_mem_count_q <= buf_mem_count_q + 1'b1;
        default: buf_mem_count_q <= buf_mem_count_q;
      endcase

      if (mem_read_fire_s) begin
        buf_rd_ptr_q <= ptr_inc(buf_rd_ptr_q);
      end

      if (mem_read_fire_s) begin
        buf_out_valid_q <= 1'b1;
      end else if (pop_fire_s) begin
        buf_out_valid_q <= 1'b0;
      end

      if (push_fire_s) begin
        pend_count_q[pend_rd_ptr_q] <= pend_count_q[pend_rd_ptr_q] - 1'b1;
        if (pend_count_q[pend_rd_ptr_q] <= BUF_CNT_W'(1)) begin
          pend_rd_ptr_q <= pend_rd_ptr_q + 1'b1;
        end
        if (ddr_rd_resp_last_i) begin
          pending_row_q <= pending_row_q + 1'b1;
        end
        buf_wr_ptr_q <= ptr_inc(buf_wr_ptr_q);
      end
    end
  end

endmodule

`endif
