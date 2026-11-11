`ifndef FPGA_DISPATCHER_CORE_SV
`define FPGA_DISPATCHER_CORE_SV

`include "../common/segcore_pkg.sv"

module dispatcher_core #(
  parameter int ADDR_W = 4,
  parameter int DATA_W = `SEGCORE_DATA_W,
  parameter int FRAC_W = `SEGCORE_FRAC_W,
  parameter int COEFF_W = `SEGCORE_COEFF_W,
  parameter int COEFF_FRAC_W = `SEGCORE_COEFF_FRAC_W,
  parameter int BANK_DEPTH = 65536,
  parameter int FIFO_DEPTH = 512,
  parameter int FIFO_WARNING_THR = (FIFO_DEPTH <= 1) ? 1 : (FIFO_DEPTH - 1),
  parameter int PRE_ACT_IDLE_CYCLES = 0,
  parameter int POST_ACT_IDLE_CYCLES = 4
) (
  input  logic                              clk,
  input  logic                              rst_n,
  input  logic [1:0]                        mode_i,
  input  logic                              cmd_valid_i,
  input  segcore_pkg::banked_idx_t          segment_idx_i,
  input  logic [ADDR_W-1:0]                 segment_l_i,
  input  logic [ADDR_W-1:0]                 segment_r_i,
  input  logic signed [DATA_W-1:0]          segment_w_i,
  input  logic signed [COEFF_W-1:0]         segment_coeff_i,
  input  logic                              resp_valid_i,
  input  logic signed [DATA_W-1:0]          resp_value_i,
  input  logic                              fifo_warning_i,
  output segcore_pkg::banked_idx_t          upd_idx_o,
  output logic signed [DATA_W-1:0]          upd_w_o,
  output logic                              upd_valid_o,
  output logic [segcore_pkg::SEG_OP_W-1:0]  cmd_op_o,
  output logic [ADDR_W-1:0]                 cmd_l_o,
  output logic [ADDR_W-1:0]                 cmd_r_o,
  output logic signed [DATA_W-1:0]          cmd_w_o,
  output logic [2:0]                        state_o,
  output logic                              pending_o,
  output logic                              task_finish_o,
  output logic                              task_fifo_warning_o,
  // Sticky hard-error flag: set when a coeff segment had to be dropped
  // because the idx/range FIFO was full; cleared only by rst_n.
  output logic                              task_fifo_overflow_o,
  // Flattened overflow-datapath trace taps (8 x 32-bit, field f at
  // [f*32 +: 32]): 0=setw_op_count, 1=query_op_count, 2=last_setw_w,
  // 3=last_native_w, 4=last_resp, 5=last_upd, 6=upd_nonzero_count,
  // 7=last_upd_idx. Diagnostic only; cleared by rst_n.
  output logic [8*32-1:0]                   debug_trace_o
);
  import segcore_pkg::*;

  localparam int MODE_IDLE = 2'd0;
  localparam int MODE_NEW_TASK = 2'd1;
  localparam int MODE_SEGMENT = 2'd2;
  localparam int MODE_SETW = 2'd3;

  localparam int PTR_W = (FIFO_DEPTH <= 1) ? 1 : $clog2(FIFO_DEPTH);
  localparam int ACT_GROUP = 8;
  localparam int ACT_W = (ACT_GROUP <= 1) ? 1 : $clog2(ACT_GROUP);
  localparam int IDLE_CYCLE_MAX = (PRE_ACT_IDLE_CYCLES > POST_ACT_IDLE_CYCLES) ? PRE_ACT_IDLE_CYCLES : POST_ACT_IDLE_CYCLES;
  localparam int IDLE_W = (IDLE_CYCLE_MAX <= 1) ? 1 : $clog2(IDLE_CYCLE_MAX);

  typedef enum logic [2:0] {
    ST_WAIT_NEW   = 3'd0,
    ST_SEGMENT    = 3'd1,
    ST_SETW       = 3'd2,
    ST_PRE_ACT    = 3'd3,
    ST_AUTO_ACT   = 3'd4,
    ST_AUTO_IDLE  = 3'd5,
    ST_AUTO_QUERY = 3'd6,
    ST_DRAIN      = 3'd7
  } dispatcher_state_e;

  dispatcher_state_e state_q, state_d;
  logic [ACT_W-1:0] act_index_q, act_index_d;
  logic [IDLE_W-1:0] auto_idle_count_q, auto_idle_count_d;
  logic new_task_bubble_q, new_task_bubble_d;

  logic idx_push, idx_pop, range_push, range_pop;
  banked_idx_t idx_push_data;
  logic signed [COEFF_W-1:0] coeff_push_data;
  logic [ADDR_W-1:0] range_l_push_data, range_r_push_data;

  logic [BANKED_IDX_W-1:0] idx_fifo_input_data_s;
  logic [BANKED_IDX_W-1:0] idx_fifo_output_data_s;
  logic [COEFF_W-1:0] coeff_fifo_input_data_s;
  logic [COEFF_W-1:0] coeff_fifo_output_data_s;
  logic [ADDR_W-1:0] range_l_fifo_input_data_s;
  logic [ADDR_W-1:0] range_l_fifo_output_data_s;
  logic [ADDR_W-1:0] range_r_fifo_input_data_s;
  logic [ADDR_W-1:0] range_r_fifo_output_data_s;
  logic idx_fifo_valid_s;
  logic coeff_fifo_valid_s;
  logic range_l_fifo_valid_s;
  logic range_r_fifo_valid_s;
  logic idx_fifo_empty_s;
  logic coeff_fifo_empty_s;
  logic range_l_fifo_empty_s;
  logic range_r_fifo_empty_s;
  logic idx_fifo_full_warning_s;
  logic idx_fifo_full_s;
  logic range_l_fifo_full_warning_s;
  logic range_l_fifo_single_s;

  banked_idx_t idx_head_s;
  logic signed [COEFF_W-1:0] coeff_head_s;
  logic [ADDR_W-1:0] range_l_head_s, range_r_head_s;

  logic pipe_valid_q;
  logic task_fifo_overflow_d, task_fifo_overflow_q;
  banked_idx_t pipe_idx_q;
  logic signed [DATA_W-1:0] pipe_resp_q;
  logic signed [COEFF_W-1:0] pipe_coeff_q;

  // Trace tap registers (CMD_DEBUG trace readback; see debug_trace_o).
  logic [31:0] trace_setw_op_count_q;
  logic [31:0] trace_query_op_count_q;
  logic [31:0] trace_last_setw_w_q;
  logic [31:0] trace_last_native_w_q;
  logic [31:0] trace_last_resp_q;
  logic [31:0] trace_last_upd_q;
  logic [31:0] trace_upd_nonzero_count_q;
  logic [31:0] trace_last_upd_idx_q;

  initial begin
    assert ((FIFO_WARNING_THR >= 1) && (FIFO_WARNING_THR <= FIFO_DEPTH))
      else $error(1, "dispatcher_core: FIFO_WARNING_THR must be within [1, FIFO_DEPTH]");
    assert (BANK_DEPTH <= (1 << MEM_IDX_W))
      else $error(1, "dispatcher_core: BANK_DEPTH exceeds banked_idx_t.mem_idx width");
  end

  function automatic logic signed [DATA_W-1:0] fxp_mul_coeff(
    input logic signed [DATA_W-1:0] value_fxp,
    input logic signed [COEFF_W-1:0] coeff_fxp
  );
    logic signed [DATA_W+COEFF_W-1:0] prod;
    begin
      prod = $signed(value_fxp) * $signed(coeff_fxp);
      fxp_mul_coeff = prod >>> COEFF_FRAC_W;
    end
  endfunction

  assign idx_fifo_input_data_s = idx_push_data;
  assign coeff_fifo_input_data_s = coeff_push_data;
  assign range_l_fifo_input_data_s = range_l_push_data;
  assign range_r_fifo_input_data_s = range_r_push_data;
  assign idx_head_s = idx_fifo_output_data_s;
  assign coeff_head_s = coeff_fifo_output_data_s;
  assign range_l_head_s = range_l_fifo_output_data_s;
  assign range_r_head_s = range_r_fifo_output_data_s;

  fifo #(
    .DATA_W(BANKED_IDX_W),
    .FIFO_DEPTH(FIFO_DEPTH),
    .WARNING_THR(FIFO_WARNING_THR),
    .HAND_WRITTEN(1'b1)
  ) u_idx_fifo (
    .clk(clk),
    .rst_n(rst_n),
    .input_data_i(idx_fifo_input_data_s),
    .input_valid_i(idx_push),
    .pop_i(idx_pop),
    .output_data_o(idx_fifo_output_data_s),
    .output_valid_o(idx_fifo_valid_s),
    .empty_o(idx_fifo_empty_s),
    .full_warning_o(idx_fifo_full_warning_s),
    .full_o(idx_fifo_full_s),
    .single_o(),
    .ready_o()
  );

  fifo #(
    .DATA_W(COEFF_W),
    .FIFO_DEPTH(FIFO_DEPTH),
    .WARNING_THR(FIFO_WARNING_THR),
    .HAND_WRITTEN(1'b1)
  ) u_coeff_fifo (
    .clk(clk),
    .rst_n(rst_n),
    .input_data_i(coeff_fifo_input_data_s),
    .input_valid_i(idx_push),
    .pop_i(idx_pop),
    .output_data_o(coeff_fifo_output_data_s),
    .output_valid_o(coeff_fifo_valid_s),
    .empty_o(coeff_fifo_empty_s),
    .full_warning_o(),
    .full_o(),
    .single_o(),
    .ready_o()
  );

  fifo #(
    .DATA_W(ADDR_W),
    .FIFO_DEPTH(FIFO_DEPTH),
    .WARNING_THR(FIFO_WARNING_THR),
    .HAND_WRITTEN(1'b1)
  ) u_range_l_fifo (
    .clk(clk),
    .rst_n(rst_n),
    .input_data_i(range_l_fifo_input_data_s),
    .input_valid_i(range_push),
    .pop_i(range_pop),
    .output_data_o(range_l_fifo_output_data_s),
    .output_valid_o(range_l_fifo_valid_s),
    .empty_o(range_l_fifo_empty_s),
    .full_warning_o(range_l_fifo_full_warning_s),
    .full_o(),
    .single_o(range_l_fifo_single_s),
    .ready_o()
  );

  fifo #(
    .DATA_W(ADDR_W),
    .FIFO_DEPTH(FIFO_DEPTH),
    .WARNING_THR(FIFO_WARNING_THR),
    .HAND_WRITTEN(1'b1)
  ) u_range_r_fifo (
    .clk(clk),
    .rst_n(rst_n),
    .input_data_i(range_r_fifo_input_data_s),
    .input_valid_i(range_push),
    .pop_i(range_pop),
    .output_data_o(range_r_fifo_output_data_s),
    .output_valid_o(range_r_fifo_valid_s),
    .empty_o(range_r_fifo_empty_s),
    .full_warning_o(),
    .full_o(),
    .single_o(),
    .ready_o()
  );

  always_comb begin
    assert ($isunknown(segment_l_i) || $isunknown(segment_r_i) || (segment_l_i <= segment_r_i))
      else $fatal(1, "dispatcher_core: invalid input range l=0b%b r=0b%b", segment_l_i, segment_r_i);
    assert (($isunknown(resp_valid_i)) || (resp_valid_i !== 1'b1) || !idx_fifo_empty_s)
      else $fatal(1, "dispatcher_core: response arrived with empty idx fifo");
    assert (
      ($isunknown(mode_i)) ||
      (mode_i != MODE_SEGMENT) ||
      $isunknown(segment_idx_i.mem_idx) ||
      (segment_idx_i.mem_idx < BANK_DEPTH)
    ) else $fatal(1, "dispatcher_core: segment_idx.mem_idx exceeds BANK_DEPTH");
  end

  always_comb begin
    state_d = state_q;
    act_index_d = act_index_q;
    auto_idle_count_d = auto_idle_count_q;
    new_task_bubble_d = new_task_bubble_q;
    task_fifo_overflow_d = 1'b0;

    idx_push = 1'b0;
    idx_pop = 1'b0;
    range_push = 1'b0;
    range_pop = 1'b0;
    idx_push_data = banked_idx_zero();
    coeff_push_data = '0;
    range_l_push_data = '0;
    range_r_push_data = '0;

    cmd_op_o = SEG_OP_IDLE;
    cmd_l_o = '0;
    cmd_r_o = '0;
    cmd_w_o = fxp_zero();

    upd_valid_o = pipe_valid_q;
    upd_idx_o = banked_idx_zero();
    upd_w_o = fxp_zero();

    if (pipe_valid_q) begin
      upd_idx_o = pipe_idx_q;
      upd_w_o = fxp_mul_coeff(pipe_resp_q, pipe_coeff_q);
    end

    if (resp_valid_i === 1'b1) begin
      idx_pop = 1'b1;
    end

    unique case (state_q)
      ST_WAIT_NEW: begin
        if (cmd_valid_i && (mode_i == MODE_NEW_TASK)) begin
          cmd_op_o = SEG_OP_CLEAR;
          state_d = ST_SEGMENT;
          act_index_d = '0;
          auto_idle_count_d = '0;
          new_task_bubble_d = 1'b1;
        end else begin
          assert (!cmd_valid_i || (mode_i == MODE_IDLE))
            else $fatal(1, "dispatcher_core: expected idle/new_task while waiting for a task");
        end
      end

      ST_SEGMENT: begin
        if (!cmd_valid_i) begin
          new_task_bubble_d = new_task_bubble_q;
        end else if (mode_i == MODE_SEGMENT) begin
          new_task_bubble_d = 1'b0;
          cmd_op_o = SEG_OP_UPD;
          cmd_l_o = segment_l_i;
          cmd_r_o = segment_r_i;
          cmd_w_o = segment_w_i;
          if (segment_coeff_i != '0) begin
            if (idx_fifo_full_s) begin
              // Hard overflow: drop the bookkeeping for this segment so the
              // FIFO never wraps and the control flow stays intact. The
              // sticky flag surfaces it to the host via the status word.
              task_fifo_overflow_d = 1'b1;
            end else begin
              idx_push = 1'b1;
              idx_push_data = segment_idx_i;
              coeff_push_data = segment_coeff_i;
              range_push = 1'b1;
              range_l_push_data = segment_l_i;
              range_r_push_data = segment_r_i;
            end
          end
        end else if (mode_i == MODE_SETW) begin
          new_task_bubble_d = 1'b0;
          cmd_op_o = SEG_OP_SETW;
          cmd_l_o = segment_l_i;
          cmd_r_o = segment_r_i;
          cmd_w_o = segment_w_i;
          state_d = ST_SETW;
        end else if (mode_i == MODE_IDLE) begin
          new_task_bubble_d = 1'b0;
        end else if ((mode_i == MODE_NEW_TASK) && new_task_bubble_q) begin
          // After a `new_task` cycle is consumed on the posedge, the host-side
          // mode may remain stable for the rest of the cycle. Treat that tail
          // as a harmless bubble before the first real segment arrives.
          new_task_bubble_d = 1'b0;
        end else begin
          $fatal(
            1,
            "dispatcher_core: invalid mode during segment/setw handoff mode=%0d rst_n=%0d pending=%0d range_empty=%0d",
            mode_i,
            rst_n,
            !idx_fifo_empty_s,
            range_l_fifo_empty_s
          );
        end
      end

      ST_SETW: begin
        if (!cmd_valid_i) begin
          cmd_op_o = SEG_OP_IDLE;
        end else if (mode_i == MODE_SETW) begin
          cmd_op_o = SEG_OP_SETW;
          cmd_l_o = segment_l_i;
          cmd_r_o = segment_r_i;
          cmd_w_o = segment_w_i;
        end else if (mode_i == MODE_IDLE) begin
          cmd_op_o = SEG_OP_FLUSH;
          cmd_w_o = fxp_zero();
          auto_idle_count_d = '0;
          state_d = ST_AUTO_ACT;
          act_index_d = '0;
        end else begin
          $fatal(1, "dispatcher_core: invalid mode during setw phase");
        end
      end

      ST_AUTO_ACT: begin
        assert (!cmd_valid_i || (mode_i == MODE_IDLE))
          else $fatal(1, "dispatcher_core: auto act/query phase requires mode=idle");
        cmd_op_o = SEG_OP_ACT;
        cmd_l_o = act_index_q;
        cmd_r_o = act_index_q;
        cmd_w_o = fxp_zero();
        if (act_index_q == (ACT_GROUP - 1)) begin
          auto_idle_count_d = '0;
          if (POST_ACT_IDLE_CYCLES == 0) begin
            if (range_l_fifo_empty_s) begin
              state_d = ST_DRAIN;
            end else begin
              state_d = ST_AUTO_QUERY;
            end
          end else begin
            state_d = ST_AUTO_IDLE;
          end
        end else begin
          act_index_d = act_index_q + 1'b1;
        end
      end

      ST_AUTO_IDLE: begin
        assert (!cmd_valid_i || (mode_i == MODE_IDLE))
          else $fatal(1, "dispatcher_core: auto act/query phase requires mode=idle");
        cmd_op_o = SEG_OP_IDLE;
        if (auto_idle_count_q == (POST_ACT_IDLE_CYCLES - 1)) begin
          if (range_l_fifo_empty_s) begin
            state_d = ST_DRAIN;
          end else begin
            state_d = ST_AUTO_QUERY;
          end
        end else begin
          auto_idle_count_d = auto_idle_count_q + 1'b1;
        end
      end

      ST_AUTO_QUERY: begin
        assert (!cmd_valid_i || (mode_i == MODE_IDLE))
          else $fatal(1, "dispatcher_core: auto act/query phase requires mode=idle");
        if (!range_l_fifo_empty_s) begin
          if (fifo_warning_i === 1'b1) begin
            cmd_op_o = SEG_OP_IDLE;
          end else begin
            cmd_op_o = SEG_OP_QUERY;
            cmd_l_o = range_l_head_s;
            cmd_r_o = range_r_head_s;
            cmd_w_o = fxp_zero();
            range_pop = 1'b1;
            if (range_l_fifo_single_s) begin
              state_d = ST_DRAIN;
            end
          end
        end else begin
          state_d = ST_DRAIN;
        end
      end

      ST_DRAIN: begin
        if (idx_fifo_empty_s && !pipe_valid_q) begin
          if (cmd_valid_i && (mode_i == MODE_NEW_TASK)) begin
            cmd_op_o = SEG_OP_CLEAR;
            state_d = ST_SEGMENT;
            act_index_d = '0;
            auto_idle_count_d = '0;
            new_task_bubble_d = 1'b1;
          end else begin
            assert (!cmd_valid_i || (mode_i == MODE_IDLE))
              else $fatal(1, "dispatcher_core: next task must start with new_task");
            state_d = ST_WAIT_NEW;
          end
        end else begin
          assert (!cmd_valid_i || (mode_i == MODE_IDLE))
            else $fatal(1, "dispatcher_core: waiting for remaining responses requires mode=idle");
        end
      end

      default: begin
        state_d = ST_WAIT_NEW;
      end
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state_q <= ST_WAIT_NEW;
      act_index_q <= '0;
      auto_idle_count_q <= '0;
      new_task_bubble_q <= 1'b0;
      pipe_valid_q <= 1'b0;
      pipe_idx_q <= banked_idx_zero();
      pipe_resp_q <= '0;
      pipe_coeff_q <= '0;
      task_fifo_overflow_q <= 1'b0;
    end else begin
      state_q <= state_d;
      act_index_q <= act_index_d;
      auto_idle_count_q <= auto_idle_count_d;
      new_task_bubble_q <= new_task_bubble_d;
      pipe_valid_q <= (resp_valid_i === 1'b1);
      task_fifo_overflow_q <= task_fifo_overflow_q | task_fifo_overflow_d;
      if (resp_valid_i === 1'b1) begin
        pipe_idx_q <= idx_head_s;
        pipe_resp_q <= resp_value_i;
        pipe_coeff_q <= coeff_head_s;
      end
    end
  end

  // sim-only: trace dispatcher state transitions (stall forensics)
  always_ff @(posedge clk) begin
    if (rst_n && (state_q != state_d)) begin
      $display("%t DSP %m %0d -> %0d mode=%0d", $time, state_q, state_d, mode_i);
    end
  end

  // Trace tap capture: registered once per event, rst_n-cleared, 32-bit
  // natural wraparound. Taps observe the same combinational conditions that
  // drive the real datapath, so they cannot perturb it.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      trace_setw_op_count_q <= '0;
      trace_query_op_count_q <= '0;
      trace_last_setw_w_q <= '0;
      trace_last_native_w_q <= '0;
      trace_last_resp_q <= '0;
      trace_last_upd_q <= '0;
      trace_upd_nonzero_count_q <= '0;
      trace_last_upd_idx_q <= '0;
    end else begin
      if (
        cmd_valid_i &&
        (mode_i == MODE_SETW) &&
        ((state_q == ST_SEGMENT) || (state_q == ST_SETW))
      ) begin
        // The first SETW of a task is issued from ST_SEGMENT (the handoff
        // cycle that enters ST_SETW); later ones from ST_SETW itself.
        trace_setw_op_count_q <= trace_setw_op_count_q + 1'b1;
        trace_last_setw_w_q <= 32'(segment_w_i);
      end
      if (
        (state_q == ST_AUTO_QUERY) &&
        !range_l_fifo_empty_s &&
        (fifo_warning_i !== 1'b1)
      ) begin
        trace_query_op_count_q <= trace_query_op_count_q + 1'b1;
      end
      if (
        (state_q == ST_SEGMENT) &&
        cmd_valid_i &&
        (mode_i == MODE_SEGMENT) &&
        (segment_coeff_i != '0)
      ) begin
        // Native query segments are redispatched with the hardware-computed
        // weight as segment_w_i (grad_overflow_core_xN FE_WAIT_RESP path).
        trace_last_native_w_q <= 32'(segment_w_i);
      end
      if (resp_valid_i === 1'b1) begin
        trace_last_resp_q <= 32'(resp_value_i);
      end
      if (pipe_valid_q) begin
        trace_last_upd_q <= 32'(fxp_mul_coeff(pipe_resp_q, pipe_coeff_q));
        if (fxp_mul_coeff(pipe_resp_q, pipe_coeff_q) != '0) begin
          trace_upd_nonzero_count_q <= trace_upd_nonzero_count_q + 1'b1;
        end
        trace_last_upd_idx_q <= {{(32-BANKED_IDX_W){1'b0}}, pipe_idx_q};
      end
    end
  end

  assign debug_trace_o = {
    trace_last_upd_idx_q,
    trace_upd_nonzero_count_q,
    trace_last_upd_q,
    trace_last_resp_q,
    trace_last_native_w_q,
    trace_last_setw_w_q,
    trace_query_op_count_q,
    trace_setw_op_count_q
  };

  assign state_o = state_q;
  assign pending_o = !idx_fifo_empty_s || pipe_valid_q;
  assign task_finish_o =
    ((state_q == ST_DRAIN) && idx_fifo_empty_s && !pipe_valid_q) ||
    (state_q == ST_WAIT_NEW);
  assign task_fifo_warning_o = idx_fifo_full_warning_s || range_l_fifo_full_warning_s;
  assign task_fifo_overflow_o = task_fifo_overflow_q;

endmodule

`endif
