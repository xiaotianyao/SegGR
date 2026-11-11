`ifndef FPGA_P_LINE_ASM_SV
`define FPGA_P_LINE_ASM_SV

// v4 p line assembler (row-writer-isomorphic, 2026-09-07 rewrite).
//
// The p write stream of each bank (one 16b entry per window row, in row
// order) lands in a DEEP per-bank pair-word buffer; a min-over-banks
// scheduler issues atomic line bursts to the bridge once every bank's
// watermark covers the lines to write. The retired implementation used
// 8-deep per-bank FIFOs with a hard lockstep alignment (all owed banks'
// FIFO heads must equal asm_row_idx in the same cycle): any bank falling
// >9 rows behind backpressured every bank, which (via the reader's
// shared-row pop and the bank pipeline) formed a circular wait that
// wedged full-scale runs three times (pre-softmax seg-1, update blk-2,
// update blk-7).
//
// Structure (mirrors softmax_row_writer):
//   per-bank intake pairing (two 16b rows -> one 32b pair word, odd tail
//   padded with a hole hi half) ->
//   per-bank BRAM pair-word buffer (P_LINE_BUF_LINES words; the depth is
//   a SKEW LIMIT, not a block capacity: the min scheduler drains
//   continuously, so blocks of any size stream through; intake
//   backpressure only when the fastest bank leads the slowest by more
//   than the buffer depth) ->
//   min-over-banks line scheduler: a bank reports its completed pair
//   count, or session_max_lines once fully written; a burst fires when
//   ready_lines >= BURST_LINES, and everything remaining drains as soon
//   as every bank is fully written (final flush, independent of
//   update_active_i falling) ->
//   writer-isomorphic output stage: the BRAM read address is
//   forward-looking (the next line is fetched in the SAME cycle its
//   predecessor is accepted), so asm_valid_o stays high for the whole
//   burst with no inter-beat bubbles; burst_busy_o spans head-to-last-
//   accept exactly like softmax_row_writer's wr_burst_busy_o, so the
//   write mux keeps producer switches batch-atomic without having to
//   infer burst end from the valid waveform.
//
// Contract to the bridge write mux (unchanged): one burst = N 512b
// lines, every entry carries count=N, row = its own p-region line index
// (consecutive within the burst), line = BANK_COUNT x 32b ({hi,lo} p
// per bank), full-lane p_line=1. The bridge latches the batch count from
// the head entry and collects exactly N entries.
module p_line_asm #(
  parameter int BANK_COUNT = 8,
  parameter int SOFTMAX_ADDR_W = 17,
  parameter int P_DATA_W = 16,
  // Pair-word (2 rows) buffer depth per bank. 4096 words = 8192 rows of
  // skew absorption (full t5 blocks max ~3600 lines; raise for t8-scale
  // 16384-row windows).
  parameter int P_LINE_BUF_LINES = 4096,
  parameter int BURST_LINES = 32,
  localparam int ROW_W = SOFTMAX_ADDR_W,
  localparam int LINE_W = SOFTMAX_ADDR_W,
  localparam int LINE_CNT_W = LINE_W + 1,
  localparam int PTR_W = (P_LINE_BUF_LINES <= 1) ? 1 : $clog2(P_LINE_BUF_LINES)
) (
  input  logic                          clk,
  input  logic                          rst_n,

  input  logic                          update_active_i,  // update/pre-softmax in progress

  input  logic [31:0]                   p_row_base_i,     // p region line base (logits space)
  input  logic [SOFTMAX_ADDR_W-1:0]     bank_valid_len_i [0:BANK_COUNT-1],

  // per-bank p sideband (in row order; one entry per row)
  input  logic                          p_valid_i [0:BANK_COUNT-1],
  output logic                          p_ready_o [0:BANK_COUNT-1],
  input  logic [SOFTMAX_ADDR_W-1:0]     p_addr_i [0:BANK_COUNT-1],
  input  logic signed [P_DATA_W-1:0]    p_data_i [0:BANK_COUNT-1],

  // p line write burst (logits-target, full strobes; asm_count_o is the
  // burst length, constant over the burst -- the bridge latches it from
  // the first beat; asm_row_o is the beat's own line index, consecutive
  // within a burst)
  output logic                          asm_valid_o,
  input  logic                          asm_ready_i,
  output logic [15:0]                   asm_count_o,
  output logic [31:0]                   asm_row_o,
  output logic [31:0]                   asm_line_o [0:BANK_COUNT-1],

  // fully drained (all rows buffered AND all lines written AND no burst
  // or read in flight); the testbench's post-update drain wait keys on it
  output logic                          drained_o,
  // burst in flight (head fetch launched .. last entry accepted into the
  // bridge pipe); mirrors softmax_row_writer's wr_burst_busy_o so the
  // write mux can hold producer ownership across any output-stage gap
  output logic                          burst_busy_o
);

  // ---------------------------------------------------------------------
  // Session context (latched on update_rise; the live inputs are
  // ctx-muxed to the block being played and would jump mid-tail)
  // ---------------------------------------------------------------------
  logic [ROW_W-1:0] valid_len_sess_q [0:BANK_COUNT-1];
  logic [ROW_W-1:0] max_valid_len_sess_q;
  logic [31:0]      row_base_sess_q;
  logic             update_active_dly_q;
  logic             update_rise_s;
  logic [ROW_W-1:0] max_valid_len_s;
  logic [LINE_W-1:0] session_max_lines_s;

  assign update_rise_s = update_active_i && !update_active_dly_q;

  // Session restart is DEFERRED until the in-flight burst has fully
  // drained: update_rise only captures the new session's context into
  // holding registers and raises restart_pend; the pointer reset and the
  // ctx switch are applied once the burst emitter is idle. Restarting
  // mid-burst retargets the in-flight burst to the new row_base and
  // scrambles the head/ready accounting into a free-running write flood
  // (the update(5)->update(6) boundary, 2026-09-07). While pending, the
  // intake is held off so the new session cannot start early.
  logic              restart_pend_q;
  logic [31:0]       hold_row_base_q;
  logic [ROW_W-1:0]  hold_max_vl_q;
  logic [ROW_W-1:0]  hold_valid_len_q [0:BANK_COUNT-1];
  // burst emitter registers (declared early; emitter_idle_s uses them)
  logic                burst_active_q;
  logic [LINE_CNT_W-1:0] burst_left_q;
  logic [LINE_CNT_W-1:0] burst_count_q;
  logic                burst_start_s;
  logic                data_valid_q;
  logic [LINE_W-1:0]   present_line_q;
  logic [31:0]         data_q [0:BANK_COUNT-1];
  logic                burst_pop_s;
  logic                rd_fire_s;
  logic [LINE_W-1:0]   rd_addr_s;
  logic              emitter_idle_s;

  assign emitter_idle_s = !burst_active_q && !data_valid_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      update_active_dly_q <= 1'b0;
      restart_pend_q <= 1'b0;
      hold_row_base_q <= '0;
      hold_max_vl_q <= '0;
      for (int b = 0; b < BANK_COUNT; b++) begin
        hold_valid_len_q[b] <= '0;
      end
      row_base_sess_q <= '0;
      max_valid_len_sess_q <= '0;
      for (int b = 0; b < BANK_COUNT; b++) begin
        valid_len_sess_q[b] <= '0;
      end
    end else begin
      update_active_dly_q <= update_active_i;
      if (update_rise_s) begin
        hold_row_base_q <= p_row_base_i;
        hold_max_vl_q <= max_valid_len_s;
        for (int b = 0; b < BANK_COUNT; b++) begin
          hold_valid_len_q[b] <= bank_valid_len_i[b];
        end
        restart_pend_q <= 1'b1;
      end
      if (restart_pend_q && emitter_idle_s) begin
        row_base_sess_q <= hold_row_base_q;
        max_valid_len_sess_q <= hold_max_vl_q;
        for (int b = 0; b < BANK_COUNT; b++) begin
          valid_len_sess_q[b] <= hold_valid_len_q[b];
        end
        restart_pend_q <= 1'b0;
      end
    end
  end

  // ---------------------------------------------------------------------
  // Per-bank intake: pair two consecutive rows into one 32b word; an odd
  // final row is flushed with a zero hi half once the bank is fully
  // written (hole halves are never consumed)
  // ---------------------------------------------------------------------
  logic [P_DATA_W-1:0] pair_lo_q [0:BANK_COUNT-1];
  logic                pair_lo_valid_q [0:BANK_COUNT-1];
  logic [ROW_W-1:0]    tail_row_q [0:BANK_COUNT-1];   // rows accepted this session
  logic [LINE_W-1:0]   tail_line_q [0:BANK_COUNT-1];  // pair words written this session
  logic [LINE_W-1:0]   head_line_q;                   // pair words drained (global)

  // pair-word buffer write sideband (the BRAMs themselves live in
  // GEN_BANK_BRAM below, one 1-D array per bank -- a single 2-D unpacked
  // array of this size cannot be mapped to block RAM by synthesis)
  logic              bank_wr_s [0:BANK_COUNT-1];
  logic [31:0]       bank_wr_data_s [0:BANK_COUNT-1];
  logic              restart_apply_s;

  logic [LINE_W-1:0] occupancy_s [0:BANK_COUNT-1];
  logic              intake_fire_s [0:BANK_COUNT-1];
  logic              pair_flush_s [0:BANK_COUNT-1];
  logic              bank_done_s [0:BANK_COUNT-1];

  assign restart_apply_s = restart_pend_q && emitter_idle_s;

  always_comb begin
    for (int b = 0; b < BANK_COUNT; b++) begin
      occupancy_s[b] = tail_line_q[b] - head_line_q;
      // held off while a session restart is pending so the new session
      // cannot feed rows into the old session's pointers
      p_ready_o[b] = !restart_pend_q &&
        (occupancy_s[b] < LINE_W'(P_LINE_BUF_LINES));
      intake_fire_s[b] = p_valid_i[b] && p_ready_o[b];
      // odd final row: all rows accepted, one pending lo half, even it out
      pair_flush_s[b] =
        pair_lo_valid_q[b] && !intake_fire_s[b] &&
        (tail_row_q[b] >= valid_len_sess_q[b]);
      bank_done_s[b] =
        (tail_row_q[b] >= valid_len_sess_q[b]) && !pair_lo_valid_q[b];
      // buffer write: a completed pair, or the odd-tail hole flush; held
      // off on the restart-apply cycle (the intake block in the main
      // always_ff is skipped that cycle too)
      bank_wr_s[b] = !restart_apply_s &&
        ((intake_fire_s[b] && pair_lo_valid_q[b]) || pair_flush_s[b]);
      bank_wr_data_s[b] = pair_flush_s[b]
        ? {P_DATA_W'(0), pair_lo_q[b]} : {p_data_i[b], pair_lo_q[b]};
    end
  end

  // ---------------------------------------------------------------------
  // min-over-banks line scheduler (softmax_row_writer's ready_limit):
  // a bank reports its completed pairs, or session_max_lines once fully
  // written, so a finished bank never caps the schedule
  // ---------------------------------------------------------------------
  logic [LINE_W-1:0] valid_lines_s [0:BANK_COUNT-1];
  logic [LINE_W-1:0] bank_line_limit_s [0:BANK_COUNT-1];
  logic [ROW_W-1:0]  water_rows_s;
  logic [LINE_W-1:0] ready_limit_s;
  logic [LINE_W-1:0] ready_lines_s;
  logic              final_s;

  function automatic logic [LINE_W-1:0] min_line(
    input logic [LINE_W-1:0] lhs,
    input logic [LINE_W-1:0] rhs
  );
    return (lhs < rhs) ? lhs : rhs;
  endfunction

  function automatic logic [LINE_W-1:0] min4_line(
    input logic [LINE_W-1:0] a0,
    input logic [LINE_W-1:0] a1,
    input logic [LINE_W-1:0] a2,
    input logic [LINE_W-1:0] a3
  );
    return min_line(min_line(a0, a1), min_line(a2, a3));
  endfunction

  always_comb begin
    max_valid_len_s = bank_valid_len_i[0];
    for (int b = 1; b < BANK_COUNT; b++) begin
      if (bank_valid_len_i[b] > max_valid_len_s) begin
        max_valid_len_s = bank_valid_len_i[b];
      end
    end
    session_max_lines_s = LINE_W'((32'(max_valid_len_sess_q) + 1) >> 1);
    for (int b = 0; b < BANK_COUNT; b++) begin
      valid_lines_s[b] = LINE_W'((32'(valid_len_sess_q[b]) + 1) >> 1);
      // rows currently usable for pairing (intake is capped at valid_len
      // upstream by the engine's phantom guard)
      water_rows_s = (tail_row_q[b] > valid_len_sess_q[b])
        ? valid_len_sess_q[b] : tail_row_q[b];
      // a fully written bank reports the session max (never caps the
      // schedule); a streaming bank reports only completed pairs
      bank_line_limit_s[b] = bank_done_s[b]
        ? session_max_lines_s
        : LINE_W'(water_rows_s >> 1);
    end
    if (BANK_COUNT == 8) begin
      ready_limit_s = min_line(
        min4_line(bank_line_limit_s[0], bank_line_limit_s[1],
                  bank_line_limit_s[2], bank_line_limit_s[3]),
        min4_line(bank_line_limit_s[4], bank_line_limit_s[5],
                  bank_line_limit_s[6], bank_line_limit_s[7]));
    end else begin
      ready_limit_s = bank_line_limit_s[0];
      for (int b = 1; b < BANK_COUNT; b++) begin
        ready_limit_s = min_line(ready_limit_s, bank_line_limit_s[b]);
      end
    end
    ready_lines_s = ready_limit_s - head_line_q;
    final_s = 1'b1;
    for (int b = 0; b < BANK_COUNT; b++) begin
      final_s &= bank_done_s[b];
    end
  end

  // ---------------------------------------------------------------------
  // Burst scheduler + emitter. burst_start needs the threshold OR the
  // final flush; the burst walks consecutive lines from head_line_q.
  // Writer-isomorphic output stage: the BRAM read address is
  // forward-looking -- a pop with more lines remaining fetches the next
  // line in the SAME cycle (rd_fire_s/rd_addr_s), so data_valid_q (and
  // thus asm_valid_o) never drops between beats of a burst.
  // ---------------------------------------------------------------------
  assign burst_start_s =
    !burst_active_q && !restart_pend_q &&
    ((ready_lines_s >= LINE_W'(BURST_LINES)) ||
     (final_s && (ready_lines_s != '0)));

  assign asm_valid_o = data_valid_q;
  assign asm_count_o = 16'(burst_count_q);
  assign asm_row_o = row_base_sess_q + 32'(present_line_q);
  for (genvar gb = 0; gb < BANK_COUNT; gb++) begin : GEN_OUT_LINE
    assign asm_line_o[gb] = data_q[gb];
  end
  assign burst_pop_s = asm_valid_o && asm_ready_i;
  assign burst_busy_o = burst_active_q;

  // refill-on-pop: a burst start reads the head line; every non-final
  // pop reads the next line in the same cycle, staying one line ahead of
  // the output stage
  assign rd_fire_s =
    burst_start_s || (burst_pop_s && (burst_left_q > LINE_CNT_W'(1)));
  assign rd_addr_s = burst_start_s ? head_line_q : (present_line_q + 1'b1);

  assign drained_o =
    !burst_active_q && !data_valid_q && final_s &&
    (head_line_q >= session_max_lines_s);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      head_line_q <= '0;
      burst_active_q <= 1'b0;
      burst_left_q <= '0;
      burst_count_q <= '0;
      data_valid_q <= 1'b0;
      present_line_q <= '0;
      for (int b = 0; b < BANK_COUNT; b++) begin
        pair_lo_q[b] <= '0;
        pair_lo_valid_q[b] <= 1'b0;
        tail_row_q[b] <= '0;
        tail_line_q[b] <= '0;
      end
    end else if (restart_pend_q && emitter_idle_s) begin
      // apply the deferred session restart (pointers reset; ctx switches
      // to the holding registers in the session always_ff this same edge)
      head_line_q <= '0;
      for (int b = 0; b < BANK_COUNT; b++) begin
        pair_lo_valid_q[b] <= 1'b0;
        tail_row_q[b] <= '0;
        tail_line_q[b] <= '0;
      end
    end else begin
      // ---- per-bank intake (buffer writes ride bank_wr_s/bank_wr_data_s
      // into the GEN_BANK_BRAM arrays) ----
      for (int b = 0; b < BANK_COUNT; b++) begin
        if (intake_fire_s[b]) begin
          if (!pair_lo_valid_q[b]) begin
            pair_lo_q[b] <= p_data_i[b];
            pair_lo_valid_q[b] <= 1'b1;
          end else begin
            pair_lo_valid_q[b] <= 1'b0;
            tail_line_q[b] <= tail_line_q[b] + 1'b1;
          end
          tail_row_q[b] <= tail_row_q[b] + 1'b1;
        end else if (pair_flush_s[b]) begin
          pair_lo_valid_q[b] <= 1'b0;
          tail_line_q[b] <= tail_line_q[b] + 1'b1;
        end
      end

      // ---- burst start ----
      if (burst_start_s) begin
        burst_active_q <= 1'b1;
        burst_count_q <=
          (ready_lines_s >= LINE_CNT_W'(BURST_LINES))
            ? LINE_CNT_W'(BURST_LINES) : LINE_CNT_W'(ready_lines_s);
        burst_left_q <=
          (ready_lines_s >= LINE_CNT_W'(BURST_LINES))
            ? LINE_CNT_W'(BURST_LINES) : LINE_CNT_W'(ready_lines_s);
      end

      // ---- line index of the in-flight BRAM read (the masked read data
      // registers live in the GEN_BANK_BRAM arrays) ----
      if (rd_fire_s) begin
        present_line_q <= rd_addr_s;
      end

      // ---- output valid: a refill raises it (set priority over the pop
      // clear, exactly the row writer's head_data_valid discipline); only
      // a pop with no line behind it clears it, so valid is continuous
      // for the whole burst
      if (rd_fire_s) begin
        data_valid_q <= 1'b1;
      end else if (burst_pop_s) begin
        data_valid_q <= 1'b0;
      end

      // ---- entry pop accounting (bridge accepted the presented line) ----
      if (burst_pop_s) begin
        head_line_q <= present_line_q + 1'b1;
        if (burst_left_q <= LINE_CNT_W'(1)) begin
          burst_active_q <= 1'b0;
          burst_left_q <= '0;
        end else begin
          burst_left_q <= burst_left_q - 1'b1;
        end
      end
    end
  end

  // ---------------------------------------------------------------------
  // Per-bank pair-word BRAM (1r1w, registered read): one independent 1-D
  // array per bank, exactly the row writer's GEN_BANK_BRAM structure --
  // a single 2-D unpacked array of this size cannot be mapped to block
  // RAM (synth fails on the array as a whole). The read register doubles
  // as the output-stage data register (data_q), with the hole mask
  // applied on the way out.
  // ---------------------------------------------------------------------
  for (genvar gb = 0; gb < BANK_COUNT; gb++) begin : GEN_BANK_BRAM
    (* ram_style = "block" *) logic [31:0] mem_q [0:P_LINE_BUF_LINES-1];

    always_ff @(posedge clk) begin
      if (bank_wr_s[gb]) begin
        mem_q[tail_line_q[gb][PTR_W-1:0]] <= bank_wr_data_s[gb];
      end
      if (rd_fire_s) begin
        data_q[gb] <= (rd_addr_s < valid_lines_s[gb])
          ? mem_q[rd_addr_s[PTR_W-1:0]] : 32'h0;
      end
    end
  end

`ifndef SYNTHESIS
  // intake row-order check (the whole buffer scheme keys on in-order
  // production). The deferred session restart makes mid-burst restarts
  // benign, so no drain assertion is needed on the pop side.
  always_ff @(posedge clk) begin
    if (rst_n) begin
      for (int b = 0; b < BANK_COUNT; b++) begin
        if (intake_fire_s[b] && (p_addr_i[b] != ROW_W'(tail_row_q[b]))) begin
          $display("%t PASM-WARN: p_addr out of order bank=%0d addr=%0d tail_row=%0d",
            $time, b, p_addr_i[b], tail_row_q[b]);
        end
      end
      // the deferred restart is only safe on a fully retired session;
      // applying it with lines still buffered silently discards them
      // (their p region keeps stale data)
      if (restart_pend_q && emitter_idle_s &&
          !(final_s && (head_line_q >= session_max_lines_s))) begin
        $display("%t PASM-WARN: session restart discards unwritten lines head=%0d max=%0d final=%b",
          $time, head_line_q, session_max_lines_s, final_s);
      end
    end
  end
`endif

endmodule

`endif
