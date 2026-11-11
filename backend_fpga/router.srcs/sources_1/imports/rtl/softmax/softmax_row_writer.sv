`ifndef FPGA_SOFTMAX_ROW_WRITER_SV
`define FPGA_SOFTMAX_ROW_WRITER_SV

module softmax_row_writer #(
  parameter int BANK_COUNT = 16,
  parameter int BANK_DEPTH = 65536,
  parameter int DATA_W = 32,
  parameter int RMS_V_W = 64,
  parameter int BURST_LINES = 64,
  // Blocked-GRADP v2: discard engine output entries beyond the block's
  // per-bank row count (fifo-lag phantoms): popped so the engine out fifo
  // drains (finish_o needs out_fifo_empty), never buffered nor written.
  parameter bit BLOCKED_SCHED = 1'b0,
  // Global DDR row index width (segment base + window-relative row); the
  // window-relative counters stay at ADDR_W.
  parameter int DDR_ROW_W = 20,
  localparam int BURST_LINES_RAW = (BURST_LINES < 1) ? 1 : BURST_LINES,
  localparam int BURST_LINES_CLAMPED = (BURST_LINES_RAW > 256) ? 256 : BURST_LINES_RAW,
  localparam int DDR_LINE_SLOTS = 8,
  localparam int AXI_BURST_ROW_SLOTS = DDR_LINE_SLOTS * BURST_LINES_CLAMPED,
  localparam int WRITE_FLUSH_THRESHOLD = 8 * AXI_BURST_ROW_SLOTS,
  localparam int ROW_BUFFER_DEPTH = 2 * WRITE_FLUSH_THRESHOLD
) (
  input  logic                                                      clk,
  input  logic                                                      rst_n,
  input  logic                                                      enable_i,
  input  logic                                                      update_mode_i,
  input  logic [$clog2(BANK_DEPTH + 1)-1:0]                         bank_valid_len_i [0:BANK_COUNT-1],
  // Blocked-GRADP v2: segment row base (row units, added only to the DDR
  // write row; entry addresses stay window-relative). 0 in monolithic mode.
  input  logic [DDR_ROW_W-1:0]                                      wr_row_base_i,
  input  logic [$clog2(BANK_DEPTH + 1)+DATA_W+RMS_V_W-1:0]          bank_entry_i [0:BANK_COUNT-1],
  input  logic                                                      bank_entry_valid_i [0:BANK_COUNT-1],
  output logic                                                      bank_entry_pop_o [0:BANK_COUNT-1],
  // pulses once per discarded phantom entry (blocked mode census)
  output logic                                                      phantom_pop_o [0:BANK_COUNT-1],
  output logic                                                      ddr_wr_valid_o,
  input  logic                                                      ddr_wr_ready_i,
  output logic [DDR_ROW_W-1:0]                                      ddr_wr_row_o,
  output logic [$clog2(ROW_BUFFER_DEPTH + 1)-1:0]                   ddr_wr_count_o,
  output logic signed [DATA_W-1:0]                                  ddr_wr_logits_o [0:BANK_COUNT-1],
  output logic signed [RMS_V_W-1:0]                                 ddr_wr_rms_o [0:BANK_COUNT-1],
  output logic [BANK_COUNT-1:0]                                     ddr_wr_lane_mask_o,
  output logic                                                      done_o,
  // A write burst has been launched (head entry presented) and not all of
  // its entries have been accepted into the bridge pipe yet. The v4 p-write
  // mux uses this to make producer switches batch-atomic: the p line
  // assembler may only take the channel while this is low, so a bridge
  // write batch never mixes entries from two producers (a mixed batch
  // desyncs the bridge's entry count and wedges it in WR_COLLECT).
  output logic                                                      wr_burst_busy_o
);
  localparam int ADDR_W = $clog2(BANK_DEPTH + 1);
  localparam int ENTRY_W = ADDR_W + DATA_W + RMS_V_W;
  localparam int WRITE_BATCH_TARGET = BURST_LINES_CLAMPED;
  localparam int FLUSH_THRESHOLD = WRITE_BATCH_TARGET;
  localparam int BUF_PTR_W = (ROW_BUFFER_DEPTH <= 1) ? 1 : $clog2(ROW_BUFFER_DEPTH);
  localparam int BUF_CNT_W = $clog2(ROW_BUFFER_DEPTH + 1);
  localparam int PACK_LOGITS_LSB = 0;
  localparam int PACK_RMS_LSB = PACK_LOGITS_LSB + DATA_W;
  localparam int BANK_ENTRY_PACK_W = PACK_RMS_LSB + RMS_V_W;
  localparam int MAX_LEN_GROUP_SIZE = 4;
  localparam int MAX_LEN_GROUP_COUNT =
    (BANK_COUNT + MAX_LEN_GROUP_SIZE - 1) / MAX_LEN_GROUP_SIZE;

  typedef struct packed {
    logic [ADDR_W-1:0]         addr;
    logic signed [DATA_W-1:0]  logits;
    logic signed [RMS_V_W-1:0] rms;
  } out_entry_t;

  logic [ADDR_W-1:0] head_row_q;
  logic [BUF_PTR_W-1:0] head_ptr_q;
  logic [ADDR_W-1:0] tail_row_q [0:BANK_COUNT-1];
  logic [BUF_PTR_W-1:0] tail_ptr_q [0:BANK_COUNT-1];
  logic [BUF_CNT_W-1:0] bank_occupancy_s [0:BANK_COUNT-1];
  logic bank_has_space_s [0:BANK_COUNT-1];

  logic flush_active_q;
  logic [BUF_CNT_W-1:0] flush_batch_count_q;
  logic [BUF_CNT_W-1:0] flush_rows_left_q;
  logic flush_cmd_valid_q;
  logic [ADDR_W-1:0] flush_cmd_row_q;
  logic [BUF_CNT_W-1:0] flush_cmd_count_q;
  logic [ADDR_W-1:0] bank_valid_len_q [0:BANK_COUNT-1];
  logic [ADDR_W-1:0] max_len_pipe0_q [0:BANK_COUNT-1];
  logic [ADDR_W-1:0] max_len_grp_q [0:MAX_LEN_GROUP_COUNT-1];
  logic [ADDR_W-1:0] max_valid_len_q;
  logic max_len_pipe0_valid_q;
  logic max_len_pipe1_valid_q;
  logic max_len_pipe2_valid_q;
  logic [ADDR_W-1:0] ready_bank_limit_s [0:BANK_COUNT-1];
  logic [ADDR_W-1:0] ready_min4_s [0:MAX_LEN_GROUP_COUNT-1];
  logic [ADDR_W-1:0] ready_stage0_min_q [0:MAX_LEN_GROUP_COUNT-1];
  logic [ADDR_W-1:0] ready_stage0_head_row_q;
  logic [ADDR_W-1:0] ready_stage0_max_valid_len_q;
  logic ready_stage0_valid_q;
  logic [ADDR_W-1:0] ready_limit_s;
  logic [BUF_CNT_W-1:0] ready_rows_s;
  logic [ADDR_W-1:0] ready_limit_q;
  logic [BUF_CNT_W-1:0] ready_rows_q;
  logic [ADDR_W-1:0] ready_head_row_q;
  logic [ADDR_W-1:0] ready_max_valid_len_q;
  logic ready_pipe_valid_q;
  logic ready_for_head_s;
  logic can_schedule_s;
  logic scheduler_busy_s;
  logic cand0_valid_q;
  logic [ADDR_W-1:0] cand0_head_row_q;
  logic [BUF_CNT_W-1:0] cand0_ready_rows_q;
  logic [BUF_CNT_W-1:0] cand0_line_slots_q;
  logic cand0_final_q;
  logic cand0_threshold_q;
  logic cand_valid_q;
  logic [ADDR_W-1:0] cand_head_row_q;
  logic [BUF_CNT_W-1:0] cand_count_q;
  logic cand_start_q;
  logic sched0_valid_q;
  logic [ADDR_W-1:0] sched0_row_q;
  logic [BUF_CNT_W-1:0] sched0_count_q;
  logic sched0_start_q;
  logic sched1_valid_q;
  logic [ADDR_W-1:0] sched1_row_q;
  logic [BUF_CNT_W-1:0] sched1_count_q;
  logic sched1_start_q;
  logic sched2_valid_q;
  logic [ADDR_W-1:0] sched2_row_q;
  logic [BUF_CNT_W-1:0] sched2_count_q;
  logic sched2_start_q;
  logic write_fire_s;
  logic has_more_after_write_s;
  logic head_data_valid_q;
  logic head_read_fire_s;
  logic [BUF_PTR_W-1:0] head_rd_ptr_s;
  out_entry_t head_s [0:BANK_COUNT-1];
  logic [BANK_ENTRY_PACK_W-1:0] bank_wr_data_s [0:BANK_COUNT-1];
  logic [BANK_ENTRY_PACK_W-1:0] bank_rd_data_s [0:BANK_COUNT-1];
  logic signed [DATA_W-1:0] head_logits_s [0:BANK_COUNT-1];
  logic signed [RMS_V_W-1:0] head_rms_s [0:BANK_COUNT-1];
  // real (stored) pop: engine entry matches the bank's write pointer and is
  // inside the block's row count; a pop without store is a phantom discard
  logic bank_entry_store_s [0:BANK_COUNT-1];

  generate
    for (genvar bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin : GEN_BANK_BRAM
      (* ram_style = "block" *) logic [BANK_ENTRY_PACK_W-1:0] mem_q [0:ROW_BUFFER_DEPTH-1];

      always_ff @(posedge clk) begin
        if (bank_entry_pop_o[bank_idx] && bank_entry_store_s[bank_idx]) begin
          mem_q[tail_ptr_q[bank_idx]] <= bank_wr_data_s[bank_idx];
        end
        if (head_read_fire_s) begin
          bank_rd_data_s[bank_idx] <= mem_q[head_rd_ptr_s];
        end
      end
    end
  endgenerate

  assign head_rd_ptr_s = write_fire_s ? ptr_inc(head_ptr_q) : head_ptr_q;
  assign scheduler_busy_s =
    ready_stage0_valid_q ||
    ready_pipe_valid_q ||
    cand0_valid_q ||
    cand_valid_q ||
    sched0_valid_q ||
    sched1_valid_q ||
    sched2_valid_q ||
    flush_cmd_valid_q ||
    flush_active_q;
  assign can_schedule_s = !scheduler_busy_s;
  assign ready_for_head_s =
    ready_pipe_valid_q &&
    (ready_head_row_q == head_row_q);
  assign has_more_after_write_s =
    (flush_cmd_valid_q && (flush_cmd_count_q > BUF_CNT_W'(1))) ||
    (flush_active_q && (flush_rows_left_q > BUF_CNT_W'(1)));

  function automatic logic [BUF_CNT_W-1:0] min3_count(
    input logic [BUF_CNT_W-1:0] a,
    input logic [BUF_CNT_W-1:0] b,
    input logic [BUF_CNT_W-1:0] c
  );
    logic [BUF_CNT_W-1:0] min_ab;
    begin
      min_ab = (a < b) ? a : b;
      min3_count = (min_ab < c) ? min_ab : c;
    end
  endfunction

  function automatic logic [ADDR_W-1:0] min_addr(
    input logic [ADDR_W-1:0] a,
    input logic [ADDR_W-1:0] b
  );
    begin
      min_addr = (a < b) ? a : b;
    end
  endfunction

  function automatic logic [ADDR_W-1:0] min4_addr(
    input logic [ADDR_W-1:0] a,
    input logic [ADDR_W-1:0] b,
    input logic [ADDR_W-1:0] c,
    input logic [ADDR_W-1:0] d
  );
    logic [ADDR_W-1:0] min_ab;
    logic [ADDR_W-1:0] min_cd;
    begin
      min_ab = min_addr(a, b);
      min_cd = min_addr(c, d);
      min4_addr = min_addr(min_ab, min_cd);
    end
  endfunction

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        bank_valid_len_q[bank_idx] <= '0;
        max_len_pipe0_q[bank_idx] <= '0;
      end
      for (int group_idx = 0; group_idx < MAX_LEN_GROUP_COUNT; group_idx++) begin
        max_len_grp_q[group_idx] <= '0;
      end
      max_valid_len_q <= '0;
      max_len_pipe0_valid_q <= 1'b0;
      max_len_pipe1_valid_q <= 1'b0;
      max_len_pipe2_valid_q <= 1'b0;
    end else begin
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        bank_valid_len_q[bank_idx] <= bank_valid_len_i[bank_idx];
      end
      if (!enable_i) begin
        for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
          max_len_pipe0_q[bank_idx] <= '0;
        end
        for (int group_idx = 0; group_idx < MAX_LEN_GROUP_COUNT; group_idx++) begin
          max_len_grp_q[group_idx] <= '0;
        end
        max_valid_len_q <= '0;
        max_len_pipe0_valid_q <= 1'b0;
        max_len_pipe1_valid_q <= 1'b0;
        max_len_pipe2_valid_q <= 1'b0;
      end else begin
        logic [ADDR_W-1:0] final_max_len;

        for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
          max_len_pipe0_q[bank_idx] <= bank_valid_len_q[bank_idx];
        end
        max_len_pipe0_valid_q <= 1'b1;

        for (int group_idx = 0; group_idx < MAX_LEN_GROUP_COUNT; group_idx++) begin
          logic [ADDR_W-1:0] group_max_len;
          group_max_len = '0;
          for (int lane_idx = 0; lane_idx < MAX_LEN_GROUP_SIZE; lane_idx++) begin
            int bank_idx;
            bank_idx = group_idx * MAX_LEN_GROUP_SIZE + lane_idx;
            if (bank_idx < BANK_COUNT) begin
              if (max_len_pipe0_q[bank_idx] > group_max_len) begin
                group_max_len = max_len_pipe0_q[bank_idx];
              end
            end
          end
          max_len_grp_q[group_idx] <= group_max_len;
        end
        max_len_pipe1_valid_q <= max_len_pipe0_valid_q;

        final_max_len = '0;
        for (int group_idx = 0; group_idx < MAX_LEN_GROUP_COUNT; group_idx++) begin
          if (max_len_grp_q[group_idx] > final_max_len) begin
            final_max_len = max_len_grp_q[group_idx];
          end
        end
        max_valid_len_q <= final_max_len;
        max_len_pipe2_valid_q <= max_len_pipe1_valid_q;
      end
    end
  end

  generate
    for (genvar bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin : GEN_UNPACK_HEAD
      assign head_logits_s[bank_idx] =
        bank_rd_data_s[bank_idx][PACK_LOGITS_LSB +: DATA_W];
      assign head_rms_s[bank_idx] =
        bank_rd_data_s[bank_idx][PACK_RMS_LSB +: RMS_V_W];
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
      slot_idx = int'(row_idx) % WRITE_BATCH_TARGET;
      burst_slots_left = BUF_CNT_W'(WRITE_BATCH_TARGET - slot_idx);
    end
  endfunction

  initial begin
    assert (ROW_BUFFER_DEPTH >= 1)
      else $error(1, "softmax_row_writer: ROW_BUFFER_DEPTH must be >= 1");
    assert (FLUSH_THRESHOLD <= ROW_BUFFER_DEPTH)
      else $error(1, "softmax_row_writer: FLUSH_THRESHOLD must fit in ROW_BUFFER_DEPTH");
    assert ((BANK_COUNT == 8) || (BANK_COUNT == 16))
      else $error(1, "softmax_row_writer: ready_limit tree supports BANK_COUNT=8 or 16");
  end

  generate
    if (BANK_COUNT == 16) begin : GEN_READY_LIMIT_16
      always_comb begin
        for (int bank_idx = 0; bank_idx < 16; bank_idx++) begin
          ready_bank_limit_s[bank_idx] =
            (tail_row_q[bank_idx] < bank_valid_len_q[bank_idx]) ?
              tail_row_q[bank_idx] : max_valid_len_q;
        end
        ready_min4_s[0] = min4_addr(ready_bank_limit_s[0], ready_bank_limit_s[1], ready_bank_limit_s[2], ready_bank_limit_s[3]);
        ready_min4_s[1] = min4_addr(ready_bank_limit_s[4], ready_bank_limit_s[5], ready_bank_limit_s[6], ready_bank_limit_s[7]);
        ready_min4_s[2] = min4_addr(ready_bank_limit_s[8], ready_bank_limit_s[9], ready_bank_limit_s[10], ready_bank_limit_s[11]);
        ready_min4_s[3] = min4_addr(ready_bank_limit_s[12], ready_bank_limit_s[13], ready_bank_limit_s[14], ready_bank_limit_s[15]);
        ready_limit_s = min4_addr(
          ready_stage0_min_q[0], ready_stage0_min_q[1], ready_stage0_min_q[2], ready_stage0_min_q[3]
        );
      end
    end else if (BANK_COUNT == 8) begin : GEN_READY_LIMIT_8
      always_comb begin
        for (int bank_idx = 0; bank_idx < 8; bank_idx++) begin
          ready_bank_limit_s[bank_idx] =
            (tail_row_q[bank_idx] < bank_valid_len_q[bank_idx]) ?
              tail_row_q[bank_idx] : max_valid_len_q;
        end
        ready_min4_s[0] = min4_addr(ready_bank_limit_s[0], ready_bank_limit_s[1], ready_bank_limit_s[2], ready_bank_limit_s[3]);
        ready_min4_s[1] = min4_addr(ready_bank_limit_s[4], ready_bank_limit_s[5], ready_bank_limit_s[6], ready_bank_limit_s[7]);
        ready_limit_s = min_addr(ready_stage0_min_q[0], ready_stage0_min_q[1]);
      end
    end else begin : GEN_READY_LIMIT_UNSUPPORTED
      always_comb begin
        for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
          ready_bank_limit_s[bank_idx] = '0;
        end
        for (int group_idx = 0; group_idx < MAX_LEN_GROUP_COUNT; group_idx++) begin
          ready_min4_s[group_idx] = '0;
        end
        ready_limit_s = '0;
      end
    end
  endgenerate

  always_comb begin

    if (ready_limit_s > ready_stage0_head_row_q) begin
      ready_rows_s = BUF_CNT_W'(ready_limit_s - ready_stage0_head_row_q);
    end else begin
      ready_rows_s = '0;
    end

    ddr_wr_lane_mask_o = '0;
    for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
      head_s[bank_idx] = bank_entry_i[bank_idx];
      bank_wr_data_s[bank_idx] = '0;
      bank_wr_data_s[bank_idx][PACK_LOGITS_LSB +: DATA_W] = head_s[bank_idx].logits;
      bank_wr_data_s[bank_idx][PACK_RMS_LSB +: RMS_V_W] = head_s[bank_idx].rms;
      bank_entry_pop_o[bank_idx] = 1'b0;
      bank_entry_store_s[bank_idx] = 1'b0;
      phantom_pop_o[bank_idx] = 1'b0;

      if (tail_row_q[bank_idx] >= head_row_q) begin
        bank_occupancy_s[bank_idx] = BUF_CNT_W'(tail_row_q[bank_idx] - head_row_q);
      end else begin
        bank_occupancy_s[bank_idx] = '0;
      end
      bank_has_space_s[bank_idx] = (bank_occupancy_s[bank_idx] < BUF_CNT_W'(ROW_BUFFER_DEPTH));

      if (head_data_valid_q && (head_row_q < bank_valid_len_q[bank_idx])) begin
        ddr_wr_lane_mask_o[bank_idx] = 1'b1;
        ddr_wr_logits_o[bank_idx] = head_logits_s[bank_idx];
        ddr_wr_rms_o[bank_idx] = head_rms_s[bank_idx];
      end else begin
        ddr_wr_logits_o[bank_idx] = '0;
        ddr_wr_rms_o[bank_idx] = '0;
      end

      if (
        enable_i &&
        update_mode_i &&
        (tail_row_q[bank_idx] < bank_valid_len_q[bank_idx]) &&
        bank_has_space_s[bank_idx] &&
        bank_entry_valid_i[bank_idx] &&
        (head_s[bank_idx].addr == tail_row_q[bank_idx])
      ) begin
        bank_entry_pop_o[bank_idx] = 1'b1;
        bank_entry_store_s[bank_idx] = 1'b1;
      end else if (
        BLOCKED_SCHED &&
        enable_i &&
        update_mode_i &&
        bank_entry_valid_i[bank_idx] &&
        (tail_row_q[bank_idx] >= bank_valid_len_q[bank_idx])
      ) begin
        // Phantom-emit discard (blocked mode): the bank's legitimate stream
        // is exactly n_b entries; anything valid beyond that is a fifo-lag
        // phantom. Pop it so the engine out fifo drains to empty (finish_o
        // requires it), but never buffer it and never advance tail_row, so
        // it can neither reach the DDR segment nor alias a real row.
        bank_entry_pop_o[bank_idx] = 1'b1;
        phantom_pop_o[bank_idx] = 1'b1;
      end
    end

    ddr_wr_row_o = DDR_ROW_W'(wr_row_base_i +
      (flush_cmd_valid_q ? flush_cmd_row_q : head_row_q));
    ddr_wr_count_o = flush_cmd_valid_q ? flush_cmd_count_q : flush_batch_count_q;
    ddr_wr_valid_o =
      enable_i &&
      update_mode_i &&
      head_data_valid_q &&
      (flush_cmd_valid_q || flush_active_q);
    write_fire_s = ddr_wr_valid_o && ddr_wr_ready_i;
    head_read_fire_s = enable_i && update_mode_i &&
      (
        (!head_data_valid_q &&
          (
            (ready_for_head_s && (ready_rows_q != '0)) ||
            cand0_valid_q ||
            cand_valid_q ||
            sched0_valid_q ||
            sched1_valid_q ||
            sched2_valid_q ||
            flush_cmd_valid_q ||
            flush_active_q
          )
        ) ||
        (write_fire_s && has_more_after_write_s)
      );

    done_o = max_len_pipe2_valid_q && enable_i && (
      !update_mode_i ||
      (
        (head_row_q >= max_valid_len_q) &&
        !scheduler_busy_s &&
        !head_data_valid_q
      )
    );

    // Busy from the burst head until the last entry is accepted into the
    // bridge pipe (the bridge may still be collecting those last entries,
    // but they are already in the pipe, so a producer switch then is safe).
    wr_burst_busy_o = flush_cmd_valid_q || flush_active_q;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      head_row_q <= '0;
      head_ptr_q <= '0;
      flush_active_q <= 1'b0;
      flush_batch_count_q <= '0;
      flush_rows_left_q <= '0;
      flush_cmd_valid_q <= 1'b0;
      flush_cmd_row_q <= '0;
      flush_cmd_count_q <= '0;
      head_data_valid_q <= 1'b0;
      ready_stage0_valid_q <= 1'b0;
      ready_stage0_head_row_q <= '0;
      ready_stage0_max_valid_len_q <= '0;
      for (int group_idx = 0; group_idx < MAX_LEN_GROUP_COUNT; group_idx++) begin
        ready_stage0_min_q[group_idx] <= '0;
      end
      ready_limit_q <= '0;
      ready_rows_q <= '0;
      ready_head_row_q <= '0;
      ready_max_valid_len_q <= '0;
      ready_pipe_valid_q <= 1'b0;
      cand0_valid_q <= 1'b0;
      cand0_head_row_q <= '0;
      cand0_ready_rows_q <= '0;
      cand0_line_slots_q <= '0;
      cand0_final_q <= 1'b0;
      cand0_threshold_q <= 1'b0;
      cand_valid_q <= 1'b0;
      cand_head_row_q <= '0;
      cand_count_q <= '0;
      cand_start_q <= 1'b0;
      sched0_valid_q <= 1'b0;
      sched0_row_q <= '0;
      sched0_count_q <= '0;
      sched0_start_q <= 1'b0;
      sched1_valid_q <= 1'b0;
      sched1_row_q <= '0;
      sched1_count_q <= '0;
      sched1_start_q <= 1'b0;
      sched2_valid_q <= 1'b0;
      sched2_row_q <= '0;
      sched2_count_q <= '0;
      sched2_start_q <= 1'b0;
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        tail_row_q[bank_idx] <= '0;
        tail_ptr_q[bank_idx] <= '0;
      end
    end else if (!enable_i || !update_mode_i) begin
      head_row_q <= '0;
      head_ptr_q <= '0;
      flush_active_q <= 1'b0;
      flush_batch_count_q <= '0;
      flush_rows_left_q <= '0;
      flush_cmd_valid_q <= 1'b0;
      flush_cmd_row_q <= '0;
      flush_cmd_count_q <= '0;
      head_data_valid_q <= 1'b0;
      ready_stage0_valid_q <= 1'b0;
      ready_stage0_head_row_q <= '0;
      ready_stage0_max_valid_len_q <= '0;
      for (int group_idx = 0; group_idx < MAX_LEN_GROUP_COUNT; group_idx++) begin
        ready_stage0_min_q[group_idx] <= '0;
      end
      ready_limit_q <= '0;
      ready_rows_q <= '0;
      ready_head_row_q <= '0;
      ready_max_valid_len_q <= '0;
      ready_pipe_valid_q <= 1'b0;
      cand0_valid_q <= 1'b0;
      cand0_head_row_q <= '0;
      cand0_ready_rows_q <= '0;
      cand0_line_slots_q <= '0;
      cand0_final_q <= 1'b0;
      cand0_threshold_q <= 1'b0;
      cand_valid_q <= 1'b0;
      cand_head_row_q <= '0;
      cand_count_q <= '0;
      cand_start_q <= 1'b0;
      sched0_valid_q <= 1'b0;
      sched0_row_q <= '0;
      sched0_count_q <= '0;
      sched0_start_q <= 1'b0;
      sched1_valid_q <= 1'b0;
      sched1_row_q <= '0;
      sched1_count_q <= '0;
      sched1_start_q <= 1'b0;
      sched2_valid_q <= 1'b0;
      sched2_row_q <= '0;
      sched2_count_q <= '0;
      sched2_start_q <= 1'b0;
      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        tail_row_q[bank_idx] <= '0;
        tail_ptr_q[bank_idx] <= '0;
      end
    end else begin
      if (can_schedule_s && max_len_pipe2_valid_q) begin
        ready_stage0_valid_q <= 1'b1;
        ready_stage0_min_q[0] <= ready_min4_s[0];
        ready_stage0_min_q[1] <= ready_min4_s[1];
        ready_stage0_min_q[2] <= ready_min4_s[2];
        ready_stage0_min_q[3] <= ready_min4_s[3];
        ready_stage0_head_row_q <= head_row_q;
        ready_stage0_max_valid_len_q <= max_valid_len_q;
      end else begin
        ready_stage0_valid_q <= 1'b0;
      end

      if (ready_stage0_valid_q) begin
        ready_pipe_valid_q <= 1'b1;
        ready_limit_q <= ready_limit_s;
        ready_rows_q <= ready_rows_s;
        ready_head_row_q <= ready_stage0_head_row_q;
        ready_max_valid_len_q <= ready_stage0_max_valid_len_q;
      end else begin
        ready_pipe_valid_q <= 1'b0;
      end

      cand0_valid_q <=
        ready_pipe_valid_q &&
        (ready_head_row_q == head_row_q) &&
        (ready_rows_q != '0);
      cand0_head_row_q <= ready_head_row_q;
      cand0_ready_rows_q <= ready_rows_q;
      cand0_line_slots_q <= burst_slots_left(ready_head_row_q);
      cand0_final_q <= (ready_limit_q >= ready_max_valid_len_q);
      cand0_threshold_q <= (ready_rows_q >= BUF_CNT_W'(FLUSH_THRESHOLD));

      cand_valid_q <= cand0_valid_q;
      cand_head_row_q <= cand0_head_row_q;
      cand_count_q <= min3_count(
        cand0_ready_rows_q,
        cand0_line_slots_q,
        BUF_CNT_W'(WRITE_BATCH_TARGET)
      );
      cand_start_q <= cand0_valid_q && (cand0_threshold_q || cand0_final_q);

      sched0_valid_q <= cand_valid_q;
      sched0_row_q <= cand_head_row_q;
      sched0_count_q <= cand_count_q;
      sched0_start_q <= cand_start_q && (cand_count_q != '0);

      sched1_valid_q <= sched0_valid_q;
      sched1_row_q <= sched0_row_q;
      sched1_count_q <= sched0_count_q;
      sched1_start_q <= sched0_start_q;

      sched2_valid_q <= sched1_valid_q;
      sched2_row_q <= sched1_row_q;
      sched2_count_q <= sched1_count_q;
      sched2_start_q <= sched1_start_q;

      if (!flush_cmd_valid_q && !flush_active_q && sched2_valid_q && sched2_start_q) begin
        flush_cmd_valid_q <= 1'b1;
        flush_cmd_row_q <= sched2_row_q;
        flush_cmd_count_q <= sched2_count_q;
      end

      if (head_read_fire_s) begin
        head_data_valid_q <= 1'b1;
      end else if (write_fire_s) begin
        head_data_valid_q <= 1'b0;
      end

      if (write_fire_s) begin
        if (flush_cmd_valid_q) begin
          flush_cmd_valid_q <= 1'b0;
          if (flush_cmd_count_q > BUF_CNT_W'(1)) begin
            flush_active_q <= 1'b1;
            flush_batch_count_q <= flush_cmd_count_q;
            flush_rows_left_q <= flush_cmd_count_q - BUF_CNT_W'(1);
          end else begin
            flush_active_q <= 1'b0;
            flush_batch_count_q <= '0;
            flush_rows_left_q <= '0;
          end
        end else if (flush_active_q) begin
          if (flush_rows_left_q > BUF_CNT_W'(1)) begin
            flush_rows_left_q <= flush_rows_left_q - BUF_CNT_W'(1);
          end else begin
            flush_active_q <= 1'b0;
            flush_batch_count_q <= '0;
            flush_rows_left_q <= '0;
          end
        end
      end

      for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
        if (bank_entry_pop_o[bank_idx] && bank_entry_store_s[bank_idx]) begin
          tail_ptr_q[bank_idx] <= ptr_inc(tail_ptr_q[bank_idx]);
          tail_row_q[bank_idx] <= tail_row_q[bank_idx] + 1'b1;
        end
      end

      if (write_fire_s) begin
        head_ptr_q <= ptr_inc(head_ptr_q);
        head_row_q <= head_row_q + 1'b1;
      end
    end
  end

endmodule

`endif
