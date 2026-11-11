`ifndef FPGA_GRADP_CORE_XN_SV
`define FPGA_GRADP_CORE_XN_SV

`include "../common/segcore_pkg.sv"

module grad_overflow_core_xN #(
  parameter int CORE_COUNT_PER_FIFO = 1,
  parameter int ADDR_W = 4,
  parameter int DATA_W = `SEGCORE_DATA_W,
  parameter int FRAC_W = `SEGCORE_FRAC_W,
  parameter int COEFF_W = `SEGCORE_COEFF_W,
  parameter int COEFF_FRAC_W = `SEGCORE_COEFF_FRAC_W,
  parameter int P_DATA_W = 16,
  parameter int P_FRAC_W = 15,
  parameter int BANK_DEPTH = 65536,
  parameter int DISPATCHER_FIFO_DEPTH = 512,
  parameter int UPD_FIFO_DEPTH = 2048,
  parameter bit NATIVE_QUERY_ENABLE = 1'b1,
  parameter int QUERY_PREFETCH_FIFO_DEPTH = 16,
  parameter int QUERY_ACT_SCALE_FXP = (1 << (FRAC_W - 1)),
  parameter int ACT_GROUP = 8,
  parameter int ACT_PIPELINE = 2,
  parameter int PRE_ACT_IDLE_CYCLES = 0,
  parameter int POST_ACT_IDLE_CYCLES = ACT_PIPELINE + 2
) (
  input  logic                                             clk,
  input  logic                                             rst_n,
  input  logic [1:0]                                       mode_i [0:CORE_COUNT_PER_FIFO-1],
  input  logic                                             mode_valid_i [0:CORE_COUNT_PER_FIFO-1],
  input  segcore_pkg::banked_idx_t                         segment_idx_i [0:CORE_COUNT_PER_FIFO-1],
  input  logic [ADDR_W-1:0]                                segment_l_i [0:CORE_COUNT_PER_FIFO-1],
  input  logic [ADDR_W-1:0]                                segment_r_i [0:CORE_COUNT_PER_FIFO-1],
  input  logic signed [DATA_W-1:0]                         segment_w_i [0:CORE_COUNT_PER_FIFO-1],
  input  logic signed [COEFF_W-1:0]                        segment_coeff_i [0:CORE_COUNT_PER_FIFO-1],
  input  logic [2:0]                                       segment_type_i [0:CORE_COUNT_PER_FIFO-1],
  input  logic                                             pop_enable_i,
  output logic                                             host_ready_o [0:CORE_COUNT_PER_FIFO-1],
  output logic                                             p_req_valid_o [0:CORE_COUNT_PER_FIFO-1],
  output segcore_pkg::banked_idx_t                         p_req_idx_o [0:CORE_COUNT_PER_FIFO-1],
  input  logic                                             p_req_grant_i [0:CORE_COUNT_PER_FIFO-1],
  input  logic                                             p_resp_valid_i [0:CORE_COUNT_PER_FIFO-1],
  input  logic signed [P_DATA_W-1:0]                       p_resp_data_i [0:CORE_COUNT_PER_FIFO-1],
  output segcore_pkg::banked_idx_t                         upd_idx_o,
  output logic signed [DATA_W-1:0]                         upd_w_o,
  output logic                                             upd_valid_o,
  output logic                                             fifo_empty_o,
  output logic                                             dispatcher_pending_o [0:CORE_COUNT_PER_FIFO-1],
  output logic                                             pending_push_o [0:CORE_COUNT_PER_FIFO-1],
  output logic                                             task_finish_o [0:CORE_COUNT_PER_FIFO-1],
  output logic                                             task_fifo_warning_o [0:CORE_COUNT_PER_FIFO-1],
  output logic                                             task_fifo_overflow_o [0:CORE_COUNT_PER_FIFO-1],
  output logic [15:0]                                      debug_core_o [0:CORE_COUNT_PER_FIFO-1],
  // Per-lane flattened dispatcher trace taps (8 x 32-bit; see
  // dispatcher_core.debug_trace_o). Diagnostic only.
  output logic [8*32-1:0]                                  debug_trace_o [0:CORE_COUNT_PER_FIFO-1],
  // Per-lane flattened segcore trace taps (5 x 32-bit, field k at
  // [k*32 +: 32]): k0 = act_eval_count, k1 = act_last_out,
  // k2-4 = act_pe aligned last-valid triple (idx/deriv/w).
  output logic [5*32-1:0]                                  debug_seg_trace_o [0:CORE_COUNT_PER_FIFO-1]
);
  import segcore_pkg::*;

  localparam int MODE_IDLE = 2'd0;
  localparam int MODE_NEW_TASK = 2'd1;
  localparam int MODE_SEGMENT = 2'd2;
  localparam int MODE_SETW = 2'd3;
  localparam int DISPATCH_ST_WAIT_NEW = 3'd0;
  localparam int DISPATCH_ST_SEGMENT = 3'd1;
  localparam int DISPATCH_ST_SETW = 3'd2;
  localparam int DISPATCH_ST_DRAIN = 3'd7;
  localparam int IDX_W = BANKED_IDX_W;
  localparam int DISPATCHER_FIFO_WARNING_THR =
    (DISPATCHER_FIFO_DEPTH <= 1) ? 1 : (DISPATCHER_FIFO_DEPTH - 1);
  localparam int FIFO_WARNING_MARGIN = CORE_COUNT_PER_FIFO * ((2 * ADDR_W) - 1);
  localparam int FIFO_WARNING_THR = UPD_FIFO_DEPTH - FIFO_WARNING_MARGIN;
  localparam int QUERY_FIFO_WARNING_THR =
    (QUERY_PREFETCH_FIFO_DEPTH <= 1) ? 1 : (QUERY_PREFETCH_FIFO_DEPTH - 1);
  localparam int QUERY_SHIFT_W = P_FRAC_W + COEFF_FRAC_W;
  localparam int NATIVE_PROD_W = P_DATA_W + COEFF_W + DATA_W;
  localparam logic signed [DATA_W-1:0] DATA_MIN = {1'b1, {(DATA_W-1){1'b0}}};
  localparam logic signed [DATA_W-1:0] DATA_MAX = {1'b0, {(DATA_W-1){1'b1}}};

  typedef struct packed {
    logic [1:0]               mode;
    banked_idx_t              idx;
    logic [ADDR_W-1:0]        l;
    logic [ADDR_W-1:0]        r;
    logic signed [DATA_W-1:0] w;
    logic signed [COEFF_W-1:0] coeff;
  } host_cmd_t;

  // p native-query prefetch (方案 B): a fetch FIFO tapped at ingress_push
  // feeds the pipelined p-bank arbiter; the frontend pairs each native-query
  // ingress head with the head of the response FIFO instead of issuing a
  // blocking p read (old FE_IDLE/FE_WAIT_RESP serializer, retired).
  localparam int P_FETCH_FIFO_DEPTH = 16;
  localparam int P_RESP_FIFO_DEPTH = 32;
  // grant inhibit margin: worst-case in-flight per core = one granted request
  // per bank still in the response pipe (BANK_COUNT), plus RR slack
  localparam int P_RESP_FIFO_WARNING_THR = P_RESP_FIFO_DEPTH - 12;

  logic [SEG_OP_W-1:0] cmd_op_s [0:CORE_COUNT_PER_FIFO-1];
  logic [ADDR_W-1:0] cmd_l_s [0:CORE_COUNT_PER_FIFO-1];
  logic [ADDR_W-1:0] cmd_r_s [0:CORE_COUNT_PER_FIFO-1];
  logic signed [DATA_W-1:0] cmd_w_s [0:CORE_COUNT_PER_FIFO-1];
  logic resp_valid_s [0:CORE_COUNT_PER_FIFO-1];
  logic signed [DATA_W-1:0] resp_value_s [0:CORE_COUNT_PER_FIFO-1];
  logic [1:0] dispatch_mode_s [0:CORE_COUNT_PER_FIFO-1];
  logic dispatch_valid_s [0:CORE_COUNT_PER_FIFO-1];
  banked_idx_t dispatch_segment_idx_s [0:CORE_COUNT_PER_FIFO-1];
  logic [ADDR_W-1:0] dispatch_segment_l_s [0:CORE_COUNT_PER_FIFO-1];
  logic [ADDR_W-1:0] dispatch_segment_r_s [0:CORE_COUNT_PER_FIFO-1];
  logic signed [DATA_W-1:0] dispatch_segment_w_s [0:CORE_COUNT_PER_FIFO-1];
  logic signed [COEFF_W-1:0] dispatch_segment_coeff_s [0:CORE_COUNT_PER_FIFO-1];
  logic [2:0] dispatcher_state_s [0:CORE_COUNT_PER_FIFO-1];
  logic dispatcher_pending_internal_s [0:CORE_COUNT_PER_FIFO-1];
  logic dispatcher_task_finish_s [0:CORE_COUNT_PER_FIFO-1];
  logic dispatcher_task_fifo_warning_s [0:CORE_COUNT_PER_FIFO-1];
  logic dispatcher_task_fifo_overflow_s [0:CORE_COUNT_PER_FIFO-1];

  banked_idx_t dispatcher_upd_idx_s [0:CORE_COUNT_PER_FIFO-1];
  logic [IDX_W-1:0] dispatcher_upd_idx_bits_s [0:CORE_COUNT_PER_FIFO-1];
  logic signed [DATA_W-1:0] dispatcher_upd_w_s [0:CORE_COUNT_PER_FIFO-1];
  logic [IDX_W+DATA_W-1:0] dispatcher_upd_payload_s [0:CORE_COUNT_PER_FIFO-1];
  logic [IDX_W+DATA_W-1:0] fifo_upd_payload_s;
  logic dispatcher_upd_valid_s [0:CORE_COUNT_PER_FIFO-1];
  logic fifo_full_warning_s;
  logic [5*32-1:0] seg_act_trace_s [0:CORE_COUNT_PER_FIFO-1];

  host_cmd_t ingress_input_s [0:CORE_COUNT_PER_FIFO-1];
  host_cmd_t ingress_head_s [0:CORE_COUNT_PER_FIFO-1];
  logic [$bits(host_cmd_t)-1:0] ingress_output_bits_s [0:CORE_COUNT_PER_FIFO-1];
  logic ingress_push_s [0:CORE_COUNT_PER_FIFO-1];
  logic ingress_pop_s [0:CORE_COUNT_PER_FIFO-1];
  logic ingress_valid_s [0:CORE_COUNT_PER_FIFO-1];
  logic ingress_empty_s [0:CORE_COUNT_PER_FIFO-1];
  logic ingress_full_warning_s [0:CORE_COUNT_PER_FIFO-1];
  logic ingress_full_s [0:CORE_COUNT_PER_FIFO-1];
  logic p_fetch_pop_s [0:CORE_COUNT_PER_FIFO-1];
  logic p_fetch_empty_s [0:CORE_COUNT_PER_FIFO-1];
  banked_idx_t p_fetch_head_s [0:CORE_COUNT_PER_FIFO-1];
  logic p_resp_pop_s [0:CORE_COUNT_PER_FIFO-1];
  logic p_resp_empty_s [0:CORE_COUNT_PER_FIFO-1];
  logic p_resp_full_warning_s [0:CORE_COUNT_PER_FIFO-1];
  logic signed [P_DATA_W-1:0] p_resp_head_s [0:CORE_COUNT_PER_FIFO-1];

  initial begin
    assert (CORE_COUNT_PER_FIFO == 1)
      else $error(1, "grad_overflow_core_xN: CORE_COUNT_PER_FIFO must be 1");
    assert (FIFO_WARNING_THR >= 1)
      else $error(1, "grad_overflow_core_xN: UPD_FIFO_DEPTH is too small for warning margin");
    assert (QUERY_PREFETCH_FIFO_DEPTH >= 1)
      else $error(1, "grad_overflow_core_xN: QUERY_PREFETCH_FIFO_DEPTH must be >= 1");
    assert (BANK_DEPTH <= (1 << MEM_IDX_W))
      else $error(1, "grad_overflow_core_xN: BANK_DEPTH exceeds banked_idx_t.mem_idx width");
  end

  function automatic logic is_native_query_cmd(
    input host_cmd_t cmd
  );
    begin
      is_native_query_cmd =
        NATIVE_QUERY_ENABLE &&
        (cmd.mode == MODE_SEGMENT) &&
        (cmd.coeff != '0);
    end
  endfunction

  function automatic logic dispatcher_accepts_cmd(
    input logic [1:0] cmd_mode,
    input logic [2:0] dispatch_state,
    input logic dispatch_task_finish
  );
    begin
      unique case (dispatch_state)
        DISPATCH_ST_WAIT_NEW: begin
          dispatcher_accepts_cmd = (cmd_mode == MODE_IDLE) || (cmd_mode == MODE_NEW_TASK);
        end
        DISPATCH_ST_SEGMENT: begin
          dispatcher_accepts_cmd =
            (cmd_mode == MODE_IDLE) || (cmd_mode == MODE_SEGMENT) || (cmd_mode == MODE_SETW);
        end
        DISPATCH_ST_SETW: begin
          dispatcher_accepts_cmd = (cmd_mode == MODE_IDLE) || (cmd_mode == MODE_SETW);
        end
        DISPATCH_ST_DRAIN: begin
          dispatcher_accepts_cmd =
            (cmd_mode == MODE_IDLE) || (dispatch_task_finish && (cmd_mode == MODE_NEW_TASK));
        end
        default: begin
          dispatcher_accepts_cmd = (cmd_mode == MODE_IDLE);
        end
      endcase
    end
  endfunction

  // Dispatch pipeline: fixed-latency 3 stages for ALL commands (uniform
  // latency keeps dispatch order identical to ingress FIFO order; a bypass
  // for non-native commands would allow overtaking). The old single-cycle
  // native_query_weight (p * coeff * act_scale + round/clamp in one comb
  // cloud, 25+ logic levels into segcore) is split across the stages.
  localparam logic signed [DATA_W-1:0] QUERY_ACT_SCALE_S = QUERY_ACT_SCALE_FXP;

  // Round/clamp half of the retired native_query_weight: exact same
  // arithmetic, but takes the precomputed product so the two multiplies can
  // live in their own pipeline stages. prod_value = p * coeff * act_scale.
  function automatic logic signed [DATA_W-1:0] native_query_round(
    input logic signed [NATIVE_PROD_W-1:0] prod_value
  );
    logic signed [NATIVE_PROD_W-1:0] abs_prod_value;
    logic signed [NATIVE_PROD_W-1:0] rounded_value;
    logic signed [NATIVE_PROD_W-1:0] shifted_value;
    logic signed [NATIVE_PROD_W-1:0] data_min_ext;
    logic signed [NATIVE_PROD_W-1:0] data_max_ext;
    begin
      if (QUERY_SHIFT_W > 0) begin
        abs_prod_value = prod_value[NATIVE_PROD_W-1] ? -prod_value : prod_value;
        rounded_value = abs_prod_value + ({{(NATIVE_PROD_W-1){1'b0}}, 1'b1} <<< (QUERY_SHIFT_W - 1));
        shifted_value = rounded_value >>> QUERY_SHIFT_W;
        if (prod_value[NATIVE_PROD_W-1]) begin
          shifted_value = -shifted_value;
        end
      end else begin
        shifted_value = prod_value;
      end

      data_min_ext = {{(NATIVE_PROD_W-DATA_W){DATA_MIN[DATA_W-1]}}, DATA_MIN};
      data_max_ext = {{(NATIVE_PROD_W-DATA_W){DATA_MAX[DATA_W-1]}}, DATA_MAX};
      if (shifted_value < data_min_ext) begin
        native_query_round = DATA_MIN;
      end else if (shifted_value > data_max_ext) begin
        native_query_round = DATA_MAX;
      end else begin
        native_query_round = shifted_value[DATA_W-1:0];
      end
    end
  endfunction

  generate
    for (genvar core_idx = 0; core_idx < CORE_COUNT_PER_FIFO; core_idx++) begin : GEN_CORE
      assign ingress_input_s[core_idx].mode = mode_i[core_idx];
      assign ingress_input_s[core_idx].idx = segment_idx_i[core_idx];
      assign ingress_input_s[core_idx].l = segment_l_i[core_idx];
      assign ingress_input_s[core_idx].r = segment_r_i[core_idx];
      assign ingress_input_s[core_idx].w = segment_w_i[core_idx];
      assign ingress_input_s[core_idx].coeff = segment_coeff_i[core_idx];
      assign ingress_head_s[core_idx] = ingress_output_bits_s[core_idx];
      // The emit gate must be one slot more conservative than the push:
      // a fire commits the command now but the pulse (and the ingress push)
      // lands next cycle; with back-to-back emits the second push would land
      // on a full FIFO and be silently dropped (the f399 SETW lesson).
      // host_ready_o (emit gate) therefore uses full_warning (count <=
      // DEPTH-2), while the push itself only honors full.
      assign host_ready_o[core_idx] = !ingress_full_warning_s[core_idx];
      assign ingress_push_s[core_idx] = mode_valid_i[core_idx] && !ingress_full_s[core_idx];

      // sim-only: trace every ingress push and every frontend dispatch
      always_ff @(posedge clk) begin
        if (rst_n && ingress_push_s[core_idx]) begin
          $display("%t ING %m push mode=%0d valid=%0d", $time, mode_i[core_idx],
                   ingress_input_s[core_idx].mode);
        end
        if (rst_n && dispatch_valid_s[core_idx]) begin
          $display("%t ING %m disp mode=%0d", $time, dispatch_mode_s[core_idx]);
        end
      end

      fifo #(
        .DATA_W($bits(host_cmd_t)),
        .FIFO_DEPTH(QUERY_PREFETCH_FIFO_DEPTH),
        .WARNING_THR(QUERY_FIFO_WARNING_THR),
        .HAND_WRITTEN(1'b1)
      ) u_ingress_fifo (
        .clk(clk),
        .rst_n(rst_n),
        .input_data_i(ingress_input_s[core_idx]),
        .input_valid_i(ingress_push_s[core_idx]),
        .pop_i(ingress_pop_s[core_idx]),
        .output_data_o(ingress_output_bits_s[core_idx]),
        .output_valid_o(ingress_valid_s[core_idx]),
        .empty_o(ingress_empty_s[core_idx]),
        .full_warning_o(ingress_full_warning_s[core_idx]),
        .full_o(ingress_full_s[core_idx]),
        .single_o(),
        .ready_o()
      );

      // p native-query prefetch FIFO: tapped at the same push point with the
      // same predicate as the dispatch side (is_native_query_cmd), so the
      // fetch order is bit-identical to the dispatch order.
      fifo #(
        .DATA_W(BANKED_IDX_W),
        .FIFO_DEPTH(P_FETCH_FIFO_DEPTH),
        .WARNING_THR(P_FETCH_FIFO_DEPTH - 1),
        .HAND_WRITTEN(1'b1)
      ) u_p_fetch_fifo (
        .clk(clk),
        .rst_n(rst_n),
        .input_data_i(ingress_input_s[core_idx].idx),
        .input_valid_i(
          ingress_push_s[core_idx] && is_native_query_cmd(ingress_input_s[core_idx])
        ),
        .pop_i(p_fetch_pop_s[core_idx]),
        .output_data_o(p_fetch_head_s[core_idx]),
        .output_valid_o(),
        .empty_o(p_fetch_empty_s[core_idx]),
        .full_warning_o(),
        .full_o(),
        .single_o(),
        .ready_o()
      );

      // pipelined p-bank response FIFO: exactly one entry per granted fetch
      fifo #(
        .DATA_W(P_DATA_W),
        .FIFO_DEPTH(P_RESP_FIFO_DEPTH),
        .WARNING_THR(P_RESP_FIFO_WARNING_THR),
        .HAND_WRITTEN(1'b1)
      ) u_p_resp_fifo (
        .clk(clk),
        .rst_n(rst_n),
        .input_data_i(p_resp_data_i[core_idx]),
        .input_valid_i(p_resp_valid_i[core_idx]),
        .pop_i(p_resp_pop_s[core_idx]),
        .output_data_o(p_resp_head_s[core_idx]),
        .output_valid_o(),
        .empty_o(p_resp_empty_s[core_idx]),
        .full_warning_o(p_resp_full_warning_s[core_idx]),
        .full_o(),
        .single_o(),
        .ready_o()
      );

      // the fetch head is offered to the pipelined p-bank arbiter (grad_p);
      // a grant pops it. New requests are inhibited early enough that the
      // in-flight grants can never overflow the response FIFO.
      assign p_req_valid_o[core_idx] =
        !p_fetch_empty_s[core_idx] && !p_resp_full_warning_s[core_idx];
      assign p_req_idx_o[core_idx] = p_fetch_head_s[core_idx];
      assign p_fetch_pop_s[core_idx] = p_req_grant_i[core_idx];

      // -------------------- dispatch pipeline (3 stages) --------------------
      // S1: pop-capture + first multiply (mul_ab = p * coeff); both FIFOs pop
      //     here (p_resp only for native queries, keeping the pairing).
      // S2: second multiply (prod = mul_ab * act_scale).
      // S3: round/clamp + w select; fires into dispatcher_core only when
      //     dispatcher_accepts_cmd. A rejection stalls the whole pipe via
      //     valid/ready, so no command is ever dropped or reordered.
      // All handshake-relevant fields (mode/idx/l/r/w/coeff/valid)
      // travel together in s*_cmd_q; pops and fire are single-point.
      host_cmd_t s1_cmd_q, s2_cmd_q, s3_cmd_q;
      logic s1_native_q, s2_native_q;
      logic signed [P_DATA_W+COEFF_W-1:0] s1_mul_ab_q;
      logic signed [NATIVE_PROD_W-1:0] s2_prod_q;
      logic signed [DATA_W-1:0] s3_w_q;
      logic s1_v_q, s2_v_q, s3_v_q;
      logic s1_load, s1_shift, s2_shift, s3_fire;
      logic pipe_busy_s;

      assign s3_fire = s3_v_q && dispatcher_accepts_cmd(
          s3_cmd_q.mode,
          dispatcher_state_s[core_idx],
          dispatcher_task_finish_s[core_idx]
        );
      assign s2_shift = s2_v_q && (!s3_v_q || s3_fire);
      assign s1_shift = s1_v_q && (!s2_v_q || s2_shift);
      assign s1_load = (!s1_v_q || s1_shift) && ingress_valid_s[core_idx] &&
                       (!is_native_query_cmd(ingress_head_s[core_idx]) ||
                        !p_resp_empty_s[core_idx]);
      assign ingress_pop_s[core_idx] = s1_load;
      assign p_resp_pop_s[core_idx] =
        s1_load && is_native_query_cmd(ingress_head_s[core_idx]);
      assign pipe_busy_s = s1_v_q || s2_v_q || s3_v_q;

      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          s1_v_q <= 1'b0;
          s2_v_q <= 1'b0;
          s3_v_q <= 1'b0;
          s1_cmd_q <= '0;
          s2_cmd_q <= '0;
          s3_cmd_q <= '0;
          s1_native_q <= 1'b0;
          s2_native_q <= 1'b0;
          s1_mul_ab_q <= '0;
          s2_prod_q <= '0;
          s3_w_q <= '0;
        end else begin
          // S1: capture the command bundle and the first partial product.
          // ingress_head_s / p_resp_head_s are registered FWFT heads, so
          // sampling them on the pop edge reads the just-popped entries.
          if (s1_load) begin
            s1_v_q <= 1'b1;
            s1_cmd_q <= ingress_head_s[core_idx];
            s1_native_q <= is_native_query_cmd(ingress_head_s[core_idx]);
            s1_mul_ab_q <= $signed(p_resp_head_s[core_idx]) *
                           $signed(ingress_head_s[core_idx].coeff);
          end else if (s1_shift) begin
            s1_v_q <= 1'b0;
          end
          // S2: second multiply (constant act_scale).
          if (s1_shift) begin
            s2_v_q <= 1'b1;
            s2_cmd_q <= s1_cmd_q;
            s2_native_q <= s1_native_q;
            s2_prod_q <= s1_mul_ab_q * QUERY_ACT_SCALE_S;
          end else if (s2_shift) begin
            s2_v_q <= 1'b0;
          end
          // S3: round/clamp for native queries, plain passthrough otherwise.
          if (s2_shift) begin
            s3_v_q <= 1'b1;
            s3_cmd_q <= s2_cmd_q;
            s3_w_q <= s2_native_q ? native_query_round(s2_prod_q) : s2_cmd_q.w;
          end else if (s3_fire) begin
            s3_v_q <= 1'b0;
          end
        end
      end

      assign dispatch_mode_s[core_idx] = s3_v_q ? s3_cmd_q.mode : MODE_IDLE;
      assign dispatch_valid_s[core_idx] = s3_fire;
      assign dispatch_segment_idx_s[core_idx] = s3_cmd_q.idx;
      assign dispatch_segment_l_s[core_idx] = s3_cmd_q.l;
      assign dispatch_segment_r_s[core_idx] = s3_cmd_q.r;
      assign dispatch_segment_w_s[core_idx] = s3_w_q;
      assign dispatch_segment_coeff_s[core_idx] = s3_cmd_q.coeff;

      // pairing integrity: a finished task must leave both prefetch FIFOs
      // empty (any leftover silently misaligns the next task's pairings).
      // Clocked + reset-gated so reset-transient X flags never fire it.
      // pipe_busy_s counts as in-flight: commands popped into the dispatch
      // pipe have left the ingress FIFO but can still have prefetch traffic
      // outstanding (their own p_resp popped, but a following task's fetches
      // may already be in flight).
      always_ff @(posedge clk) begin
        if (rst_n) begin
          assert (
            !(dispatcher_task_finish_s[core_idx] && !ingress_valid_s[core_idx] &&
              !pipe_busy_s) ||
            (p_fetch_empty_s[core_idx] && p_resp_empty_s[core_idx])
          ) else $fatal(1, "grad_overflow_core_xN: task_finish with outstanding p prefetch");
        end
      end

      dispatcher_core #(
        .ADDR_W(ADDR_W),
        .DATA_W(DATA_W),
        .FRAC_W(FRAC_W),
        .COEFF_W(COEFF_W),
        .COEFF_FRAC_W(COEFF_FRAC_W),
        .BANK_DEPTH(BANK_DEPTH),
        .FIFO_DEPTH(DISPATCHER_FIFO_DEPTH),
        .FIFO_WARNING_THR(DISPATCHER_FIFO_WARNING_THR),
        .PRE_ACT_IDLE_CYCLES(PRE_ACT_IDLE_CYCLES),
        .POST_ACT_IDLE_CYCLES(POST_ACT_IDLE_CYCLES)
      ) u_dispatcher_core (
        .clk(clk),
        .rst_n(rst_n),
        .mode_i(dispatch_mode_s[core_idx]),
        .cmd_valid_i(dispatch_valid_s[core_idx]),
        .segment_idx_i(dispatch_segment_idx_s[core_idx]),
        .segment_l_i(dispatch_segment_l_s[core_idx]),
        .segment_r_i(dispatch_segment_r_s[core_idx]),
        .segment_w_i(dispatch_segment_w_s[core_idx]),
        .segment_coeff_i(dispatch_segment_coeff_s[core_idx]),
        .resp_valid_i(resp_valid_s[core_idx]),
        .resp_value_i(resp_value_s[core_idx]),
        .fifo_warning_i(fifo_full_warning_s),
        .upd_idx_o(dispatcher_upd_idx_s[core_idx]),
        .upd_w_o(dispatcher_upd_w_s[core_idx]),
        .upd_valid_o(dispatcher_upd_valid_s[core_idx]),
        .cmd_op_o(cmd_op_s[core_idx]),
        .cmd_l_o(cmd_l_s[core_idx]),
        .cmd_r_o(cmd_r_s[core_idx]),
        .cmd_w_o(cmd_w_s[core_idx]),
        .state_o(dispatcher_state_s[core_idx]),
        .pending_o(dispatcher_pending_internal_s[core_idx]),
        .task_finish_o(dispatcher_task_finish_s[core_idx]),
        .task_fifo_warning_o(dispatcher_task_fifo_warning_s[core_idx]),
        .task_fifo_overflow_o(dispatcher_task_fifo_overflow_s[core_idx]),
        .debug_trace_o(debug_trace_o[core_idx])
      );

      segcore_core #(
        .ADDR_W(ADDR_W),
        .DATA_W(DATA_W),
        .FRAC_W(FRAC_W),
        .ACT_GROUP(ACT_GROUP),
        .ACT_PIPELINE(ACT_PIPELINE)
      ) u_segcore_core (
        .clk(clk),
        .rst_n(rst_n),
        .cmd_op_i(cmd_op_s[core_idx]),
        .cmd_l_i(cmd_l_s[core_idx]),
        .cmd_r_i(cmd_r_s[core_idx]),
        .cmd_w_i(cmd_w_s[core_idx]),
        .resp_valid_o(resp_valid_s[core_idx]),
        .resp_value_o(resp_value_s[core_idx]),
        .debug_act_trace_o(seg_act_trace_s[core_idx])
      );

      assign debug_seg_trace_o[core_idx] = seg_act_trace_s[core_idx];

      assign dispatcher_upd_idx_bits_s[core_idx] = dispatcher_upd_idx_s[core_idx];
      assign dispatcher_upd_payload_s[core_idx] = {
        dispatcher_upd_idx_bits_s[core_idx],
        dispatcher_upd_w_s[core_idx]
      };
      assign dispatcher_pending_o[core_idx] =
        dispatcher_pending_internal_s[core_idx] ||
        ingress_valid_s[core_idx] ||
        pipe_busy_s ||
        !p_fetch_empty_s[core_idx] ||
        !p_resp_empty_s[core_idx];
      assign pending_push_o[core_idx] = dispatcher_upd_valid_s[core_idx];
      // task_finish must also wait for the dispatch pipe: commands popped
      // into the pipe have left the FIFOs but not yet reached the dispatcher.
      // dispatcher_task_finish_s is a level (DRAIN/WAIT_NEW), so the extra
      // term only delays the pulse by a few cycles, never suppresses it.
      assign task_finish_o[core_idx] =
        dispatcher_task_finish_s[core_idx] &&
        !ingress_valid_s[core_idx] &&
        !pipe_busy_s &&
        p_fetch_empty_s[core_idx] &&
        p_resp_empty_s[core_idx];
      assign task_fifo_warning_o[core_idx] =
       dispatcher_task_fifo_warning_s[core_idx] ||
       ingress_full_warning_s[core_idx];
      assign task_fifo_overflow_o[core_idx] = dispatcher_task_fifo_overflow_s[core_idx];
      assign debug_core_o[core_idx] = {
        4'b0,
        fifo_full_warning_s,
        pending_push_o[core_idx],
        (ingress_valid_s[core_idx] &&
         is_native_query_cmd(ingress_head_s[core_idx]) &&
         p_resp_empty_s[core_idx]),
        ingress_full_s[core_idx],
        ingress_valid_s[core_idx],
        ingress_empty_s[core_idx],
        dispatcher_task_finish_s[core_idx],
        dispatcher_pending_internal_s[core_idx],
        dispatcher_state_s[core_idx]
      };
    end
  endgenerate

  fifo #(
    .DATA_W(IDX_W + DATA_W),
    .FIFO_DEPTH(UPD_FIFO_DEPTH),
    .WARNING_THR(FIFO_WARNING_THR),
    .HAND_WRITTEN(1'b1)
  ) u_fifo (
    .clk(clk),
    .rst_n(rst_n),
    .input_data_i(dispatcher_upd_payload_s[0]),
    .input_valid_i(dispatcher_upd_valid_s[0]),
    .pop_i(pop_enable_i),
    .output_data_o(fifo_upd_payload_s),
    .output_valid_o(upd_valid_o),
    .empty_o(fifo_empty_o),
    .full_warning_o(fifo_full_warning_s),
    .full_o(),
    .single_o(),
    .ready_o()
  );

  assign upd_idx_o = fifo_upd_payload_s[DATA_W +: IDX_W];
  assign upd_w_o = fifo_upd_payload_s[0 +: DATA_W];

endmodule

`endif
