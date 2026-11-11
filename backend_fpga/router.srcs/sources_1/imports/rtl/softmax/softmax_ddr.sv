`ifndef FPGA_SOFTMAX_DDR_SV
`define FPGA_SOFTMAX_DDR_SV

`include "../common/segcore_pkg.sv"

module softmax_ddr #(
  parameter int BANK_COUNT = 16,
  parameter int BANK_DEPTH = 65536,
  parameter int DATA_W = `SEGCORE_DATA_W,
  parameter int FRAC_W = `SEGCORE_FRAC_W,
  parameter int P_DATA_W = 16,
  parameter int P_FRAC_W = 15,
  parameter int ACC_W = 64,
  parameter int GROUP_FLAG_WORD_W = 32,
  parameter int RMS_V_W = 64,
  parameter logic [DATA_W-1:0] RMSPROP_ALPHA_FXP = 32'd64881,
  parameter logic [RMS_V_W-1:0] RMSPROP_EPS_FXP = '0,
  parameter logic signed [DATA_W-1:0] LR_FXP = 32'sd19661,
  parameter int MAX_GROUP_LEN = 8,
  parameter int IN_FIFO_DEPTH = 32,
  parameter int OUT_FIFO_DEPTH = 32,
  parameter int BURST_LINES = 64,
  // Blocked-GRADP v2: per-context segment descriptors (row bases) and the
  // meta row stream (block load / pre-softmax group_flag fetch). With
  // BLOCKED_SCHED=0 and zero row bases the data path is exactly v1.0.5.
  parameter bit BLOCKED_SCHED = 1'b0,
  parameter int META_LANE_W = 22,
  // Global DDR row index width (logits/p/gs row space); wider than the
  // on-chip bank depth width ADDR_W, which stays block-internal.
  parameter int DDR_ROW_W = 20,
  localparam int BURST_LINES_RAW = (BURST_LINES < 1) ? 1 : BURST_LINES,
  localparam int BURST_LINES_CLAMPED = (BURST_LINES_RAW > 256) ? 256 : BURST_LINES_RAW,
  localparam int ROW_READ_BUFFER_DEPTH = 8 * BURST_LINES_CLAMPED,
  localparam int AXI_BURST_ROW_SLOTS = 8 * BURST_LINES_CLAMPED,
  localparam int WRITE_FLUSH_THRESHOLD = 8 * AXI_BURST_ROW_SLOTS,
  localparam int ROW_WRITE_BUFFER_DEPTH = 2 * WRITE_FLUSH_THRESHOLD
) (
  input  logic                                                      clk,
  input  logic                                                      rst_n,
  input  logic                                                      enable_i,
  input  logic                                                      update_mode_i,
  // v4: p line assembler session gate = pre-softmax OR update active
  // (update_mode_i alone is update-only in blocked mode; the pre-softmax
  // p writes would otherwise never assemble)
  input  logic                                                      p_asm_active_i,
  input  logic signed [P_DATA_W-1:0]                                p_i [0:BANK_COUNT-1],
  input  logic signed [ACC_W-1:0]                                   grad_p_i [0:BANK_COUNT-1],
  input  logic [GROUP_FLAG_WORD_W-1:0]                              group_start_flag_word_i [0:BANK_COUNT-1],
  input  logic [$clog2(BANK_DEPTH + 1)-1:0]                         bank_valid_len_i [0:BANK_COUNT-1],
  // ---- Blocked-GRADP v2: segment descriptors (row units) ----
  // 0 in monolithic mode; blocked_ctrl muxes them per context (pre-softmax
  // segment / update block).
  input  logic [DDR_ROW_W-1:0]                                      rd_row_base_i,
  input  logic [DDR_ROW_W-1:0]                                      wr_row_base_i,
  // ---- Blocked-GRADP v2: meta row stream (blocked_ctrl <-> bridge) ----
  // Third row stream over the shared single-outstanding bridge read
  // channel; a meta lane rides in the response logits field.
  input  logic                                                      meta_rd_req_valid_i,
  output logic                                                      meta_rd_req_ready_o,
  input  logic [DDR_ROW_W-1:0]                                      meta_rd_req_row_i,
  input  logic [$clog2(ROW_READ_BUFFER_DEPTH + 1)-1:0]              meta_rd_req_count_i,
  output logic                                                      meta_rd_resp_valid_o,
  input  logic                                                      meta_rd_resp_ready_i,
  output logic                                                      meta_rd_resp_last_o,
  output logic [31:0]                                               meta_rd_resp_lane_o [0:BANK_COUNT-1],
  // v4 p write sideband (one entry per window row per bank, in row order;
  // the internal p_line_asm aligns and packs them into full p lines)
  input  logic [31:0]                                               p_row_base_i,
  input  logic                                                      p_pack_valid_i [0:BANK_COUNT-1],
  output logic                                                      p_pack_ready_o [0:BANK_COUNT-1],
  input  logic [$clog2(BANK_DEPTH + 1)-1:0]                         p_pack_row_i [0:BANK_COUNT-1],
  input  logic signed [P_DATA_W-1:0]                                p_pack_lane_i [0:BANK_COUNT-1],
  // p sideband stage-1 vacancy, forwarded to the bank engines' writeback
  // gate (pulse-style p writes must not fire into a stalled demux)
  input  logic                                                      p_sb_ready_i [0:BANK_COUNT-1],
  // per-bank phantom-discard strobes from the row writer (blocked census)
  output logic                                                      phantom_pop_o [0:BANK_COUNT-1],
  output logic                                                      local_rd_ce_o [0:BANK_COUNT-1],
  output logic [$clog2(BANK_DEPTH + 1)-1:0]                         local_rd_addr_o [0:BANK_COUNT-1],
  output logic                                                      p_wr_valid_o [0:BANK_COUNT-1],
  output logic [$clog2(BANK_DEPTH + 1)-1:0]                         p_wr_addr_o [0:BANK_COUNT-1],
  output logic signed [P_DATA_W-1:0]                                p_wr_data_o [0:BANK_COUNT-1],
  output logic                                                      ddr_rd_req_valid_o,
  input  logic                                                      ddr_rd_req_ready_i,
  output logic [DDR_ROW_W-1:0]                                      ddr_rd_req_row_o,
  output logic [$clog2(ROW_READ_BUFFER_DEPTH + 1)-1:0]              ddr_rd_req_count_o,
  input  logic                                                      ddr_rd_resp_valid_i,
  output logic                                                      ddr_rd_resp_ready_o,
  input  logic [BANK_COUNT-1:0]                                     ddr_rd_resp_bank_mask_i,
  input  logic                                                      ddr_rd_resp_last_i,
  input  logic signed [DATA_W-1:0]                                  ddr_rd_logits_i [0:BANK_COUNT-1],
  input  logic signed [RMS_V_W-1:0]                                 ddr_rd_rms_i [0:BANK_COUNT-1],
  output logic                                                      ddr_wr_valid_o,
  input  logic                                                      ddr_wr_ready_i,
  output logic [DDR_ROW_W-1:0]                                      ddr_wr_row_o,
  output logic [$clog2(ROW_WRITE_BUFFER_DEPTH + 1)-1:0]             ddr_wr_count_o,
  output logic signed [DATA_W-1:0]                                  ddr_wr_logits_o [0:BANK_COUNT-1],
  output logic signed [RMS_V_W-1:0]                                 ddr_wr_rms_o [0:BANK_COUNT-1],
  output logic [BANK_COUNT-1:0]                                     ddr_wr_lane_mask_o,
  output logic                                                      ddr_wr_p_line_o,
  output logic                                                      finish_o [0:BANK_COUNT-1],
  output logic                                                      all_finish_o,
  // debug probes (self-run bring-up): {meta_active, reader_req_valid,
  // reader_resp_pending}
  output logic [2:0]                                                dbg_reader_meta_o,
  // p write session fully retired into DDR (p_line_asm drained_o);
  // blocked_ctrl gates the next update kick on it so a session restart
  // can never discard buffered-but-unwritten lines
  output logic                                                      p_asm_drained_o
);
  localparam int ADDR_W = $clog2(BANK_DEPTH + 1);
  localparam int ENTRY_W = ADDR_W + DATA_W + RMS_V_W;
  localparam int REQ_CNT_W = $clog2(ROW_READ_BUFFER_DEPTH + 1);

  logic [ENTRY_W-1:0] bank_in_entry_s [0:BANK_COUNT-1];
  logic               bank_in_valid_s [0:BANK_COUNT-1];
  logic               bank_in_ready_s [0:BANK_COUNT-1];
  logic [ENTRY_W-1:0] bank_out_entry_s [0:BANK_COUNT-1];
  logic               bank_out_valid_s [0:BANK_COUNT-1];
  logic               bank_out_pop_s [0:BANK_COUNT-1];
  logic               reader_done_s;
  logic               bank_reader_done_s [0:BANK_COUNT-1];
  logic               writer_done_s;

  // row writer channel wires (muxed with the v4 p write channel)
  logic               writer_wr_valid_s;
  logic               writer_wr_ready_s;
  logic               writer_burst_busy_s;
  logic [DDR_ROW_W-1:0] writer_wr_row_s;
  logic [$clog2(ROW_WRITE_BUFFER_DEPTH + 1)-1:0] writer_wr_count_s;
  logic signed [DATA_W-1:0] writer_wr_logits_s [0:BANK_COUNT-1];
  logic signed [RMS_V_W-1:0] writer_wr_rms_s [0:BANK_COUNT-1];
  logic [BANK_COUNT-1:0] writer_wr_lane_mask_s;
  logic               p_take_s;
  logic               asm_valid_s;
  logic               asm_ready_s;
  logic [15:0]        asm_count_s;
  logic [31:0]        asm_row_s;
  logic [31:0]        asm_line_s [0:BANK_COUNT-1];
  logic               asm_burst_busy_s;

  // row reader channel wires (muxed onto the shared bridge read channel)
  logic               reader_req_valid_s;
  logic               reader_req_ready_s;
  logic [DDR_ROW_W-1:0]  reader_req_row_s;
  logic [REQ_CNT_W-1:0] reader_req_count_s;
  logic               reader_resp_valid_s;
  logic               reader_resp_ready_s;
  logic               reader_resp_pending_s;

  // ---------------------------------------------------------------------
  // Bridge read channel arbitration (blocked mode): the meta row stream
  // shares the bridge read channel with the logits/rms row stream. The
  // bridge now keeps multiple reads outstanding (OT pipeline) and delivers
  // responses strictly in request order, so this mux only needs an owner
  // FIFO: every granted request pushes {owner, row count}, and response
  // beats route to the owner at the FIFO head. The reader keeps request
  // priority, same as before; with BLOCKED_SCHED=0 the meta path is inert
  // and the mux degenerates to the v1.0.5 direct connection.
  // ---------------------------------------------------------------------
  localparam int OWNER_FIFO_DEPTH = 4;
  localparam int OWNER_PTR_W = (OWNER_FIFO_DEPTH <= 1) ? 1 : $clog2(OWNER_FIFO_DEPTH);
  localparam int OWNER_CNT_W = $clog2(OWNER_FIFO_DEPTH + 1);

  logic               owner_q [0:OWNER_FIFO_DEPTH-1];  // 0=reader, 1=meta
  logic [15:0]        owner_left_q [0:OWNER_FIFO_DEPTH-1];
  logic [OWNER_PTR_W-1:0] owner_rd_ptr_q;
  logic [OWNER_PTR_W-1:0] owner_wr_ptr_q;
  logic [OWNER_CNT_W-1:0] owner_level_q;
  logic               meta_req_grant_s;
  logic               owner_full_s;
  logic               owner_empty_s;
  logic               owner_head_meta_s;
  logic               reader_req_fire_s;
  logic               meta_req_fire_s;
  logic               resp_row_fire_s;
  logic               resp_pop_s;

  assign owner_full_s = (owner_level_q == OWNER_CNT_W'(OWNER_FIFO_DEPTH));
  assign owner_empty_s = (owner_level_q == '0);
  assign owner_head_meta_s = !owner_empty_s && owner_q[owner_rd_ptr_q];

  assign reader_req_fire_s = reader_req_valid_s && reader_req_ready_s;
  assign meta_req_fire_s = meta_rd_req_valid_i && meta_rd_req_ready_o;
  assign resp_row_fire_s =
    ddr_rd_resp_valid_i && ddr_rd_resp_ready_o && ddr_rd_resp_last_i;
  assign resp_pop_s = resp_row_fire_s && (owner_left_q[owner_rd_ptr_q] <= 16'd1);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      owner_rd_ptr_q <= '0;
      owner_wr_ptr_q <= '0;
      owner_level_q <= '0;
      for (int fifo_idx = 0; fifo_idx < OWNER_FIFO_DEPTH; fifo_idx++) begin
        owner_q[fifo_idx] <= 1'b0;
        owner_left_q[fifo_idx] <= '0;
      end
    end else begin
      if (reader_req_fire_s) begin
        owner_q[owner_wr_ptr_q] <= 1'b0;
        owner_left_q[owner_wr_ptr_q] <= 16'(reader_req_count_s);
      end else if (meta_req_fire_s) begin
        owner_q[owner_wr_ptr_q] <= 1'b1;
        owner_left_q[owner_wr_ptr_q] <= 16'(meta_rd_req_count_i);
      end
      if (reader_req_fire_s || meta_req_fire_s) begin
        owner_wr_ptr_q <= owner_wr_ptr_q + 1'b1;
      end
      if (resp_row_fire_s) begin
        owner_left_q[owner_rd_ptr_q] <= owner_left_q[owner_rd_ptr_q] - 16'd1;
      end
      if (resp_pop_s) begin
        owner_rd_ptr_q <= owner_rd_ptr_q + 1'b1;
      end
      unique case ({(reader_req_fire_s || meta_req_fire_s), resp_pop_s})
        2'b10: owner_level_q <= owner_level_q + 1'b1;
        2'b01: owner_level_q <= owner_level_q - 1'b1;
        default: owner_level_q <= owner_level_q;
      endcase
    end
  end

  assign meta_req_grant_s = BLOCKED_SCHED && meta_rd_req_valid_i &&
    !owner_full_s && !reader_req_valid_s;

  assign ddr_rd_req_valid_o =
    meta_req_grant_s ? 1'b1 : (!owner_full_s && reader_req_valid_s);
  assign ddr_rd_req_row_o =
    meta_req_grant_s ? meta_rd_req_row_i : reader_req_row_s;
  assign ddr_rd_req_count_o =
    meta_req_grant_s ? meta_rd_req_count_i : reader_req_count_s;
  assign meta_rd_req_ready_o = meta_req_grant_s && ddr_rd_req_ready_i;
  assign reader_req_ready_s =
    !meta_req_grant_s && !owner_full_s && ddr_rd_req_ready_i;

  assign reader_resp_valid_s = ddr_rd_resp_valid_i && !owner_head_meta_s;
  assign meta_rd_resp_valid_o = ddr_rd_resp_valid_i && owner_head_meta_s;
  assign meta_rd_resp_last_o = ddr_rd_resp_last_i;
  assign ddr_rd_resp_ready_o =
    owner_head_meta_s ? meta_rd_resp_ready_i : reader_resp_ready_s;

  generate
    for (genvar meta_bank_idx = 0; meta_bank_idx < BANK_COUNT; meta_bank_idx++) begin : GEN_META_LANE
      // a meta lane rides in the response logits field (22b of 32b)
      assign meta_rd_resp_lane_o[meta_bank_idx] =
        ddr_rd_logits_i[meta_bank_idx][31:0];
    end
  endgenerate

  softmax_row_reader #(
    .BANK_COUNT(BANK_COUNT),
    .BANK_DEPTH(BANK_DEPTH),
    .DATA_W(DATA_W),
    .RMS_V_W(RMS_V_W),
    .BURST_LINES(BURST_LINES),
    .DDR_ROW_W(DDR_ROW_W)
  ) u_softmax_row_reader (
    .clk(clk),
    .rst_n(rst_n),
    .enable_i(enable_i),
    .update_mode_i(update_mode_i),
    .bank_valid_len_i(bank_valid_len_i),
    .rd_row_base_i(rd_row_base_i),
    .bank_accept_i(bank_in_ready_s),
    .ddr_rd_req_valid_o(reader_req_valid_s),
    .ddr_rd_req_ready_i(reader_req_ready_s),
    .ddr_rd_req_row_o(reader_req_row_s),
    .ddr_rd_req_count_o(reader_req_count_s),
    .ddr_rd_resp_valid_i(reader_resp_valid_s),
    .ddr_rd_resp_ready_o(reader_resp_ready_s),
    .ddr_rd_resp_bank_mask_i(ddr_rd_resp_bank_mask_i),
    .ddr_rd_resp_last_i(ddr_rd_resp_last_i),
    .ddr_rd_logits_i(ddr_rd_logits_i),
    .ddr_rd_rms_i(ddr_rd_rms_i),
    .bank_entry_o(bank_in_entry_s),
    .bank_entry_valid_o(bank_in_valid_s),
    .resp_pending_o(reader_resp_pending_s),
    .done_o(reader_done_s),
    .done_per_bank_o(bank_reader_done_s)
  );

  softmax_row_writer #(
    .BANK_COUNT(BANK_COUNT),
    .BANK_DEPTH(BANK_DEPTH),
    .DATA_W(DATA_W),
    .RMS_V_W(RMS_V_W),
    .BURST_LINES(BURST_LINES),
    .BLOCKED_SCHED(BLOCKED_SCHED),
    .DDR_ROW_W(DDR_ROW_W)
  ) u_softmax_row_writer (
    .clk(clk),
    .rst_n(rst_n),
    .enable_i(enable_i),
    .update_mode_i(update_mode_i),
    .bank_valid_len_i(bank_valid_len_i),
    .wr_row_base_i(wr_row_base_i),
    .bank_entry_i(bank_out_entry_s),
    .bank_entry_valid_i(bank_out_valid_s),
    .bank_entry_pop_o(bank_out_pop_s),
    .phantom_pop_o(phantom_pop_o),
    .ddr_wr_valid_o(writer_wr_valid_s),
    .ddr_wr_ready_i(writer_wr_ready_s),
    .ddr_wr_row_o(writer_wr_row_s),
    .ddr_wr_count_o(writer_wr_count_s),
    .ddr_wr_logits_o(writer_wr_logits_s),
    .ddr_wr_rms_o(writer_wr_rms_s),
    .ddr_wr_lane_mask_o(writer_wr_lane_mask_s),
    .done_o(writer_done_s),
    .wr_burst_busy_o(writer_burst_busy_s)
  );

  genvar bank_idx;
  generate
    for (bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin : GEN_BANK
      softmax_bank_engine #(
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
        .LR_FXP(LR_FXP),
        .MAX_GROUP_LEN(MAX_GROUP_LEN),
        .IN_FIFO_DEPTH(IN_FIFO_DEPTH),
        .OUT_FIFO_DEPTH(OUT_FIFO_DEPTH)
      ) u_softmax_bank_engine (
        .clk(clk),
        .rst_n(rst_n),
        .enable_i(enable_i),
        .update_mode_i(update_mode_i),
        .input_done_i(bank_reader_done_s[bank_idx]),
        .bank_valid_len_i(bank_valid_len_i[bank_idx]),
        .in_entry_i(bank_in_entry_s[bank_idx]),
        .in_valid_i(bank_in_valid_s[bank_idx]),
        .in_ready_o(bank_in_ready_s[bank_idx]),
        .local_rd_ce_o(local_rd_ce_o[bank_idx]),
        .local_rd_addr_o(local_rd_addr_o[bank_idx]),
        .p_i(p_i[bank_idx]),
        .grad_p_i(grad_p_i[bank_idx]),
        .group_start_flag_word_i(group_start_flag_word_i[bank_idx]),
        .p_wr_valid_o(p_wr_valid_o[bank_idx]),
        .p_wr_addr_o(p_wr_addr_o[bank_idx]),
        .p_wr_data_o(p_wr_data_o[bank_idx]),
        .p_sb_ready_i(p_sb_ready_i[bank_idx]),
        .out_entry_o(bank_out_entry_s[bank_idx]),
        .out_valid_o(bank_out_valid_s[bank_idx]),
        .out_pop_i(bank_out_pop_s[bank_idx]),
        .finish_o(finish_o[bank_idx])
      );
    end
  endgenerate

  always_comb begin
    all_finish_o = enable_i && reader_done_s && writer_done_s;
    for (int bank_idx_int = 0; bank_idx_int < BANK_COUNT; bank_idx_int++) begin
      all_finish_o &= finish_o[bank_idx_int];
    end
  end

  // ---------------------------------------------------------------------
  // v4 p line assembler + write mux: assembled p line bursts take priority
  // (the assembler holds a burst in flight); the row writer is held off for
  // the whole burst and retries after. Producer switches are batch-atomic:
  // the assembler may only take the channel while the writer has no burst
  // in flight (wr_burst_busy_o), and once taken it keeps the channel until
  // its burst has fully streamed (asm_own_q), so a bridge write batch never
  // mixes entries from two producers -- a mixed batch desyncs the bridge's
  // entry count (batch = head-declared N entries) and wedges it in
  // WR_COLLECT with an empty pipe. The write carries p_line_mode=1 so the
  // bridge skips the rms companion (p region is a logits-target full-line
  // write).
  p_line_asm #(
    .BANK_COUNT(BANK_COUNT),
    .SOFTMAX_ADDR_W($clog2(BANK_DEPTH + 1)),
    .P_DATA_W(P_DATA_W)
  ) u_p_line_asm (
    .clk(clk),
    .rst_n(rst_n),
    .update_active_i(p_asm_active_i),
    .p_row_base_i(p_row_base_i),
    .bank_valid_len_i(bank_valid_len_i),
    .p_valid_i(p_pack_valid_i),
    .p_ready_o(p_pack_ready_o),
    .p_addr_i(p_pack_row_i),
    .p_data_i(p_pack_lane_i),
    .asm_valid_o(asm_valid_s),
    .asm_ready_i(asm_ready_s),
    .asm_count_o(asm_count_s),
    .asm_row_o(asm_row_s),
    .asm_line_o(asm_line_s),
    .burst_busy_o(asm_burst_busy_s),
    .drained_o(p_asm_drained_o)
  );

  // Producer-ownership latch: once the assembler takes the channel it keeps
  // it until its burst has fully streamed into the bridge pipe, even if the
  // writer starts a new burst meanwhile -- and the assembler may only take
  // the channel while the writer has no burst in flight
  // (writer_burst_busy_s covers until the writer's last entry is in the
  // pipe). The release keys on burst_busy_o (the assembler's
  // wr_burst_busy_o equivalent), NOT on asm_valid_s alone: valid follows
  // the output-stage refill, and only burst_busy guarantees an inter-beat
  // bubble can never drop ownership mid-batch. Together with the bridge's
  // entry-count batching this makes every write batch single-producer end
  // to end.
  logic asm_own_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      asm_own_q <= 1'b0;
    end else if (!asm_own_q && asm_valid_s && !writer_burst_busy_s) begin
      asm_own_q <= 1'b1;
    end else if (asm_own_q && !asm_valid_s && !asm_burst_busy_s) begin
      asm_own_q <= 1'b0;
    end
  end

  always_comb begin
    p_take_s = asm_own_q || (asm_valid_s && !writer_burst_busy_s);
    asm_ready_s = p_take_s && ddr_wr_ready_i;
    // valid follows the OWNER's valid, not the ownership latch: asm_own_q
    // clears one cycle after asm_valid_s falls, and without this
    // qualification the latch would push one phantom entry (stale
    // row/count) into the bridge pipe in that gap cycle, desyncing the
    // bridge's batch count into a WR_COLLECT wedge.
    ddr_wr_valid_o = p_take_s ? asm_valid_s : writer_wr_valid_s;
    ddr_wr_row_o = p_take_s
      ? asm_row_s[DDR_ROW_W-1:0]
      : writer_wr_row_s;
    ddr_wr_count_o = p_take_s ? asm_count_s : writer_wr_count_s;
    ddr_wr_lane_mask_o = p_take_s ? {BANK_COUNT{1'b1}} : writer_wr_lane_mask_s;
    ddr_wr_p_line_o = p_take_s;
    writer_wr_ready_s = p_take_s ? 1'b0 : ddr_wr_ready_i;
    for (int bank_idx_int = 0; bank_idx_int < BANK_COUNT; bank_idx_int++) begin
      ddr_wr_logits_o[bank_idx_int] = p_take_s
        ? asm_line_s[bank_idx_int]
        : writer_wr_logits_s[bank_idx_int];
      ddr_wr_rms_o[bank_idx_int] = writer_wr_rms_s[bank_idx_int];
    end
  end

  assign dbg_reader_meta_o =
    {!owner_empty_s, reader_req_valid_s, reader_resp_pending_s};

endmodule

`endif
