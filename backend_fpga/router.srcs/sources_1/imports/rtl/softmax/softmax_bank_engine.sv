`ifndef FPGA_SOFTMAX_BANK_ENGINE_SV
`define FPGA_SOFTMAX_BANK_ENGINE_SV

module softmax_stream_fifo #(
  parameter int DATA_W = 32,
  parameter int FIFO_DEPTH = 32
) (
  input  logic                 clk,
  input  logic                 rst_n,
  input  logic [DATA_W-1:0]    data_i,
  input  logic                 push_i,
  input  logic                 pop_i,
  output logic [DATA_W-1:0]    data_o,
  output logic                 valid_o,
  output logic                 empty_o,
  output logic                 full_o
);
  fifo #(
    .DATA_W(DATA_W),
    .FIFO_DEPTH(FIFO_DEPTH),
    .WARNING_THR((FIFO_DEPTH > 2) ? (FIFO_DEPTH - 2) : FIFO_DEPTH),
    .FWFT_MODE(1'b1),
    .COMMON_CLOCK_MODE(1'b1),
    .HAND_WRITTEN(1'b1)
  ) u_fifo (
    .clk(clk),
    .rst_n(rst_n),
    .input_data_i(data_i),
    .input_valid_i(push_i),
    .pop_i(pop_i),
    .output_data_o(data_o),
    .output_valid_o(valid_o),
    .empty_o(empty_o),
    .full_warning_o(full_o),
    .full_o(),
    .single_o(),
    .ready_o()
  );
endmodule

module softmax_bank_engine #(
  parameter int BANK_DEPTH = 65536,
  parameter int DATA_W = 32,
  parameter int FRAC_W = 16,
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
  parameter int OUT_FIFO_DEPTH = 32
) (
  input  logic                                       clk,
  input  logic                                       rst_n,
  input  logic                                       enable_i,
  input  logic                                       update_mode_i,
  input  logic                                       input_done_i,
  input  logic [$clog2(BANK_DEPTH + 1)-1:0]          bank_valid_len_i,
  input  logic [$clog2(BANK_DEPTH + 1)+DATA_W+RMS_V_W-1:0] in_entry_i,
  input  logic                                       in_valid_i,
  output logic                                       in_ready_o,
  output logic                                       local_rd_ce_o,
  output logic [$clog2(BANK_DEPTH + 1)-1:0]          local_rd_addr_o,
  input  logic signed [P_DATA_W-1:0]                 p_i,
  input  logic signed [ACC_W-1:0]                    grad_p_i,
  input  logic [GROUP_FLAG_WORD_W-1:0]               group_start_flag_word_i,
  output logic                                       p_wr_valid_o,
  output logic [$clog2(BANK_DEPTH + 1)-1:0]          p_wr_addr_o,
  output logic signed [P_DATA_W-1:0]                 p_wr_data_o,
  // v4: p sideband stage-1 vacancy; the pulse-style p write at step 7 must
  // hold while the demux/sideband path is stalled (no drop allowed: the
  // writeback FIFOs pop atomically with the pulse)
  input  logic                                       p_sb_ready_i,
  output logic [$clog2(BANK_DEPTH + 1)+DATA_W+RMS_V_W-1:0] out_entry_o,
  output logic                                       out_valid_o,
  input  logic                                       out_pop_i,
  output logic                                       finish_o
);
  localparam int ADDR_W = $clog2(BANK_DEPTH + 1);
  localparam int IN_ENTRY_W = ADDR_W + DATA_W + RMS_V_W;
  localparam int OUT_ENTRY_W = ADDR_W + DATA_W + RMS_V_W;
  localparam int GROUP_FLAG_WORD_SHIFT = (GROUP_FLAG_WORD_W <= 1) ? 0 : $clog2(GROUP_FLAG_WORD_W);
  localparam longint signed DATA_MIN = -(64'sd1 <<< (DATA_W - 1));
  localparam longint signed DATA_MAX = (64'sd1 <<< (DATA_W - 1)) - 1;
  localparam logic [DATA_W-1:0] ONE_FXP = (32'd1 <<< FRAC_W);
  localparam logic [DATA_W-1:0] ONE_MINUS_ALPHA_FXP = ONE_FXP - RMSPROP_ALPHA_FXP;
  localparam logic signed [DATA_W-1:0] FXP_EIGHT = 32'sd8 <<< FRAC_W;
  localparam int LUT_BUCKET_COUNT = 512;
  localparam int LUT_BUCKET_STEP = (8 << FRAC_W) / LUT_BUCKET_COUNT;
  localparam int GROUP_LEN_W = $clog2(MAX_GROUP_LEN + 1);
  localparam int GROUP_DESC_W = GROUP_LEN_W + ACC_W;
  localparam int PIPE_FIFO_DEPTH = IN_FIFO_DEPTH;
  localparam int CORDIC_INPUT_W = 48;
  localparam int CORDIC_OUTPUT_W = 32;
  localparam int SQRT_EXP_W = $clog2(RMS_V_W + 1);
  localparam int SQRT_CONTEXT_W = DATA_W + SQRT_EXP_W;
  localparam int SQRT_RESULT_W = 2 * DATA_W;
  localparam int PROB_PRODUCT_W = DATA_W + ACC_W;
  localparam logic [ACC_W-1:0] RECIP_NUMERATOR =
    {{(ACC_W-1){1'b0}}, 1'b1} << (FRAC_W + P_FRAC_W);

  typedef enum logic [1:0] {
    SM_WAIT,
    SM_COLLECT,
    SM_EXP,
    SM_EMIT
  } softmax_state_e;

  typedef struct packed {
    logic [ADDR_W-1:0]          addr;
    logic signed [DATA_W-1:0]   logits;
    logic signed [RMS_V_W-1:0]  rms;
  } in_entry_t;

  typedef struct packed {
    logic [ADDR_W-1:0]          addr;
    logic signed [P_DATA_W-1:0] p;
    logic signed [ACC_W-1:0]    grad_p;
    logic signed [DATA_W-1:0]   logits;
    logic signed [RMS_V_W-1:0]  rms;
    logic                       group_start;
  } full_sample_t;

  typedef struct packed {
    logic [GROUP_LEN_W-1:0]    len;
    logic signed [ACC_W-1:0]  sum_mul;
  } group_desc_t;

  typedef struct packed {
    logic signed [DATA_W-1:0] raw_grad;
    logic [SQRT_EXP_W-1:0]    exponent;
  } sqrt_context_t;

  typedef struct packed {
    logic signed [DATA_W-1:0] raw_grad;
    logic [DATA_W-1:0]        rms_grad;
  } sqrt_result_t;

  logic fetch_pending_q, fetch_pending_d;
  logic held_valid_q, held_valid_d;
  full_sample_t held_sample_q, held_sample_d;
  in_entry_t fetched_entry_q, fetched_entry_d;
  logic [GROUP_LEN_W-1:0] collect_count_q, collect_count_d;
  logic signed [ACC_W-1:0] collect_sum_q, collect_sum_d;

  logic [IN_ENTRY_W-1:0] in_fifo_out_s;
  logic in_fifo_valid_s;
  logic in_fifo_empty_s;
  logic in_fifo_full_warning_s;
  logic in_fifo_full_s;
  logic in_fifo_pop_s;
  in_entry_t in_head_s;

  logic [OUT_ENTRY_W-1:0] out_fifo_in_s;
  logic [OUT_ENTRY_W-1:0] out_fifo_out_s;
  logic out_fifo_push_s;
  logic out_fifo_valid_s;
  logic out_fifo_empty_s;
  logic out_fifo_full_warning_s;
  logic out_fifo_full_s;

  logic [ADDR_W-1:0] addr_fifo_in_s, addr_fifo_out_s;
  logic addr_fifo_push_s, addr_fifo_pop_s, addr_fifo_valid_s, addr_fifo_empty_s, addr_fifo_full_s;
  logic signed [DATA_W-1:0] logits_fifo_in_s, logits_fifo_out_s;
  logic logits_fifo_push_s, logits_fifo_pop_s, logits_fifo_valid_s, logits_fifo_empty_s, logits_fifo_full_s;
  logic signed [RMS_V_W-1:0] rms_fifo_in_s, rms_fifo_out_s;
  logic rms_fifo_push_s, rms_fifo_pop_s, rms_fifo_valid_s, rms_fifo_empty_s, rms_fifo_full_s;
  logic signed [P_DATA_W-1:0] p_fifo_in_s, p_fifo_out_s;
  logic p_fifo_push_s, p_fifo_pop_s, p_fifo_valid_s, p_fifo_empty_s, p_fifo_full_s;
  logic signed [ACC_W-1:0] grad_fifo_in_s, grad_fifo_out_s;
  logic grad_fifo_push_s, grad_fifo_pop_s, grad_fifo_valid_s, grad_fifo_empty_s, grad_fifo_full_s;

  logic [GROUP_DESC_W-1:0] raw_desc_fifo_in_s, raw_desc_fifo_out_s;
  logic raw_desc_fifo_push_s, raw_desc_fifo_pop_s, raw_desc_fifo_valid_s;
  logic raw_desc_fifo_empty_s, raw_desc_fifo_full_s;
  group_desc_t raw_desc_head_s;
  logic [GROUP_LEN_W-1:0] final_len_fifo_in_s, final_len_fifo_out_s;
  logic final_len_fifo_push_s, final_len_fifo_pop_s, final_len_fifo_valid_s;
  logic final_len_fifo_empty_s, final_len_fifo_full_s;

  logic signed [DATA_W-1:0] raw_fifo_in_s, raw_fifo_out_s;
  logic raw_fifo_push_s, raw_fifo_pop_s, raw_fifo_valid_s, raw_fifo_empty_s, raw_fifo_full_s;
  logic signed [DATA_W-1:0] raw_norm_fifo_in_s, raw_norm_fifo_out_s;
  logic raw_norm_fifo_push_s, raw_norm_fifo_pop_s, raw_norm_fifo_valid_s;
  logic raw_norm_fifo_empty_s, raw_norm_fifo_full_s;
  logic signed [RMS_V_W-1:0] rms_next_fifo_in_s, rms_next_fifo_out_s;
  logic rms_next_fifo_push_s, rms_next_fifo_pop_s, rms_next_fifo_valid_s;
  logic rms_next_fifo_empty_s, rms_next_fifo_full_s;
  logic signed [RMS_V_W-1:0] rms_wb_fifo_in_s, rms_wb_fifo_out_s;
  logic rms_wb_fifo_push_s, rms_wb_fifo_pop_s, rms_wb_fifo_valid_s;
  logic rms_wb_fifo_empty_s, rms_wb_fifo_full_s;
  logic [SQRT_CONTEXT_W-1:0] sqrt_context_fifo_in_s, sqrt_context_fifo_out_s;
  logic sqrt_context_fifo_push_s, sqrt_context_fifo_pop_s, sqrt_context_fifo_valid_s;
  logic sqrt_context_fifo_empty_s, sqrt_context_fifo_full_s;
  sqrt_context_t sqrt_context_head_s;
  logic [SQRT_RESULT_W-1:0] sqrt_result_fifo_in_s, sqrt_result_fifo_out_s;
  logic sqrt_result_fifo_push_s, sqrt_result_fifo_pop_s, sqrt_result_fifo_valid_s;
  logic sqrt_result_fifo_empty_s, sqrt_result_fifo_full_s;
  sqrt_result_t sqrt_result_head_s;
  logic signed [DATA_W-1:0] norm_fifo_in_s, norm_fifo_out_s;
  logic norm_fifo_push_s, norm_fifo_pop_s, norm_fifo_valid_s, norm_fifo_empty_s, norm_fifo_full_s;
  logic signed [DATA_W-1:0] logits_new_fifo_in_s, logits_new_fifo_out_s;
  logic logits_new_fifo_push_s, logits_new_fifo_pop_s, logits_new_fifo_valid_s;
  logic logits_new_fifo_empty_s, logits_new_fifo_full_s;
  logic signed [DATA_W-1:0] logits_wb_fifo_in_s, logits_wb_fifo_out_s;
  logic logits_wb_fifo_push_s, logits_wb_fifo_pop_s, logits_wb_fifo_valid_s;
  logic logits_wb_fifo_empty_s, logits_wb_fifo_full_s;
  logic signed [P_DATA_W-1:0] p_new_fifo_in_s, p_new_fifo_out_s;
  logic p_new_fifo_push_s, p_new_fifo_pop_s, p_new_fifo_valid_s;
  logic p_new_fifo_empty_s, p_new_fifo_full_s;

  logic signed [DATA_W-1:0] recip_context_fifo_in_s, recip_context_fifo_out_s;
  logic recip_context_fifo_push_s, recip_context_fifo_pop_s, recip_context_fifo_valid_s;
  logic recip_context_fifo_empty_s, recip_context_fifo_full_s;
  logic recip_dividend_sent_q, recip_dividend_sent_d;
  logic recip_divisor_sent_q, recip_divisor_sent_d;
  logic recip_dividend_tvalid_s, recip_dividend_tready_s;
  logic recip_divisor_tvalid_s, recip_divisor_tready_s;
  logic [ACC_W-1:0] recip_dividend_tdata_s, recip_divisor_tdata_s;
  logic recip_m_tvalid_s, recip_m_tready_s;
  logic [2*ACC_W-1:0] recip_m_tdata_s;
  logic signed [PROB_PRODUCT_W-1:0] prob_mul_product_q, prob_mul_product_d;
  logic prob_mul_valid_q, prob_mul_valid_d;
  logic signed [P_DATA_W-1:0] prob_format_data_q, prob_format_data_d;
  logic prob_format_valid_q, prob_format_valid_d;

  logic [CORDIC_INPUT_W-1:0] cordic_s_tdata_s;
  logic cordic_s_tvalid_s, cordic_s_tready_s;
  logic [CORDIC_OUTPUT_W-1:0] cordic_m_tdata_s;
  logic cordic_m_tvalid_s, cordic_m_tready_s;

  logic grad_div_context_fifo_in_s, grad_div_context_fifo_out_s;
  logic grad_div_context_fifo_push_s, grad_div_context_fifo_pop_s;
  logic grad_div_context_fifo_valid_s, grad_div_context_fifo_empty_s;
  logic grad_div_context_fifo_full_s;
  logic grad_dividend_sent_q, grad_dividend_sent_d;
  logic grad_divisor_sent_q, grad_divisor_sent_d;
  logic grad_dividend_tvalid_s, grad_dividend_tready_s;
  logic grad_divisor_tvalid_s, grad_divisor_tready_s;
  logic signed [(2*DATA_W)-1:0] grad_dividend_tdata_s;
  logic signed [(2*DATA_W)-1:0] grad_divisor_tdata_s;
  logic grad_div_m_tvalid_s, grad_div_m_tready_s;
  logic [(4*DATA_W)-1:0] grad_div_m_tdata_s;

  logic raw_group_active_q, raw_group_active_d;
  logic [GROUP_LEN_W-1:0] raw_group_len_q, raw_group_len_d;
  logic [GROUP_LEN_W-1:0] raw_group_count_q, raw_group_count_d;
  logic signed [ACC_W-1:0] raw_group_sum_q, raw_group_sum_d;
  logic raw_pipe_valid_q, raw_pipe_valid_d;
  logic raw_pipe_singleton_q, raw_pipe_singleton_d;
  logic signed [ACC_W-1:0] raw_pipe_p_grad_q, raw_pipe_p_grad_d;
  logic signed [ACC_W-1:0] raw_pipe_sum_p_q, raw_pipe_sum_p_d;

  logic final_group_active_q, final_group_active_d;
  logic [GROUP_LEN_W-1:0] final_group_len_q, final_group_len_d;

  softmax_state_e sm_state_q, sm_state_d;
  logic [GROUP_LEN_W-1:0] sm_idx_q, sm_idx_d;
  logic signed [DATA_W-1:0] sm_max_q, sm_max_d;
  logic signed [ACC_W-1:0] sm_sum_q, sm_sum_d;
  logic signed [DATA_W-1:0] sm_logits_q [0:MAX_GROUP_LEN-1];
  logic signed [DATA_W-1:0] sm_exp_q [0:MAX_GROUP_LEN-1];
  logic sm_logits_we_s, sm_exp_we_s;
  logic [GROUP_LEN_W-1:0] sm_buf_widx_s;
  logic signed [DATA_W-1:0] sm_logits_wdata_s, sm_exp_wdata_s;

  // Kept as observable compatibility signals for the existing cocotb/TB
  // backpressure statistics. They describe the new final emit stage.
  logic state_q;
  logic [GROUP_LEN_W-1:0] emit_idx_q;
  logic [GROUP_LEN_W-1:0] group_count_q;

  localparam logic signed [DATA_W-1:0] LUT_EXP [0:LUT_BUCKET_COUNT-1] = '{
`include "softmax_exp_lut_512x8.svh"
  };

  initial begin
    assert (GROUP_FLAG_WORD_W >= 1)
      else $error(1, "softmax_bank_engine: GROUP_FLAG_WORD_W must be >= 1");
    assert ((GROUP_FLAG_WORD_W & (GROUP_FLAG_WORD_W - 1)) == 0)
      else $error(1, "softmax_bank_engine: GROUP_FLAG_WORD_W must be a power-of-two");
    assert (MAX_GROUP_LEN >= 1)
      else $error(1, "softmax_bank_engine: MAX_GROUP_LEN must be >= 1");
    assert (RMS_V_W == 64)
      else $error(1, "softmax_bank_engine: CORDIC normalization requires RMS_V_W=64");
    assert (DATA_W == CORDIC_OUTPUT_W)
      else $error(1, "softmax_bank_engine: CORDIC output requires DATA_W=32");
    assert (ACC_W == 64)
      else $error(1, "softmax_bank_engine: reciprocal Divider Generator requires ACC_W=64");
    assert ((FRAC_W + P_FRAC_W) < ACC_W)
      else $error(1, "softmax_bank_engine: reciprocal numerator shift exceeds ACC_W");
  end

  function automatic logic signed [ACC_W-1:0] fxp_mul_p_grad_acc(
    input logic signed [P_DATA_W-1:0] p_value,
    input logic signed [ACC_W-1:0] grad_value
  );
    logic signed [ACC_W+P_DATA_W-1:0] prod;
    begin
      prod = p_value * grad_value;
      return $signed(prod >>> P_FRAC_W);
    end
  endfunction

  function automatic logic signed [ACC_W-1:0] fxp_mul_acc_p(
    input logic signed [ACC_W-1:0] acc_value,
    input logic signed [P_DATA_W-1:0] p_value
  );
    logic signed [ACC_W+P_DATA_W-1:0] prod;
    begin
      prod = acc_value * p_value;
      return $signed(prod >>> P_FRAC_W);
    end
  endfunction

  function automatic logic signed [DATA_W-1:0] saturate_data(
    input logic signed [ACC_W-1:0] value
  );
    begin
      if (value < DATA_MIN) begin
        return {1'b1, {(DATA_W-1){1'b0}}};
      end
      if (value > DATA_MAX) begin
        return {1'b0, {(DATA_W-1){1'b1}}};
      end
      return value[DATA_W-1:0];
    end
  endfunction

  function automatic logic signed [RMS_V_W-1:0] data_square_to_state(
    input logic signed [DATA_W-1:0] value
  );
    logic signed [(2*DATA_W)-1:0] product;
    logic signed [(2*DATA_W)-1:0] overflow_bits;
    begin
      product = value * value;
      if (RMS_V_W >= (2 * DATA_W)) begin
        return {{(RMS_V_W-(2*DATA_W)){1'b0}}, product};
      end
      overflow_bits = product >>> RMS_V_W;
      if (overflow_bits != '0) begin
        return {1'b0, {(RMS_V_W-1){1'b1}}};
      end
      return product[RMS_V_W-1:0];
    end
  endfunction

  function automatic logic signed [RMS_V_W-1:0] fxp_mul_state_coeff(
    input logic signed [RMS_V_W-1:0] state_value,
    input logic [DATA_W-1:0] coeff_fxp
  );
    logic signed [RMS_V_W+DATA_W-1:0] product;
    begin
      product = state_value * $signed({1'b0, coeff_fxp});
      return $signed(product >>> FRAC_W);
    end
  endfunction

  function automatic logic signed [RMS_V_W-1:0] add_eps_to_state(
    input logic signed [RMS_V_W-1:0] state_value
  );
    logic signed [RMS_V_W:0] sum_value;
    begin
      sum_value = $signed({1'b0, state_value}) + $signed({1'b0, RMSPROP_EPS_FXP});
      if (sum_value[RMS_V_W]) begin
        return {1'b0, {(RMS_V_W-1){1'b1}}};
      end
      return sum_value[RMS_V_W-1:0];
    end
  endfunction

  function automatic logic [SQRT_EXP_W-1:0] state_sqrt_even_exponent(
    input logic signed [RMS_V_W-1:0] state_value
  );
    int unsigned bit_length;
    begin
      bit_length = 0;
      if (state_value > 0) begin
        for (int bit_idx = 0; bit_idx < RMS_V_W; bit_idx++) begin
          if (state_value[bit_idx]) begin
            bit_length = bit_idx + 1;
          end
        end
      end
      return bit_length & ~1;
    end
  endfunction

  function automatic logic [CORDIC_INPUT_W-1:0] state_sqrt_mantissa(
    input logic signed [RMS_V_W-1:0] state_value,
    input logic [SQRT_EXP_W-1:0] exponent
  );
    logic [RMS_V_W-1:0] positive_value;
    int signed input_shift;
    begin
      positive_value = (state_value > 0) ? state_value : '0;
      input_shift = int'(exponent) - (CORDIC_INPUT_W - 1);
      if (input_shift >= 0) begin
        return positive_value >> input_shift;
      end
      return positive_value << (-input_shift);
    end
  endfunction

  function automatic logic [DATA_W-1:0] cordic_sqrt_to_data(
    input logic [CORDIC_OUTPUT_W-1:0] output_code,
    input logic [SQRT_EXP_W-1:0] exponent
  );
    logic [RMS_V_W:0] restored_value;
    int signed output_shift;
    begin
      output_shift = (int'(exponent) / 2) - (CORDIC_OUTPUT_W - 1);
      if (output_shift >= 0) begin
        restored_value = output_code << output_shift;
      end else begin
        restored_value = output_code >> (-output_shift);
      end
      if (restored_value > {DATA_W{1'b1}}) begin
        return {DATA_W{1'b1}};
      end
      return restored_value[DATA_W-1:0];
    end
  endfunction

  function automatic logic signed [DATA_W-1:0] grad_div_quotient_to_data(
    input logic signed [(2*DATA_W)-1:0] quotient
  );
    begin
      if (quotient < DATA_MIN) begin
        return {1'b1, {(DATA_W-1){1'b0}}};
      end
      if (quotient > DATA_MAX) begin
        return {1'b0, {(DATA_W-1){1'b1}}};
      end
      return quotient[DATA_W-1:0];
    end
  endfunction

  function automatic logic signed [DATA_W-1:0] fxp_mul_lr_grad(
    input logic signed [DATA_W-1:0] lr_value,
    input logic signed [DATA_W-1:0] grad_value
  );
    logic signed [(2*DATA_W)-1:0] product;
    logic signed [(2*DATA_W)-1:0] shifted;
    begin
      product = lr_value * grad_value;
      shifted = product >>> FRAC_W;
      if (shifted < DATA_MIN) begin
        return {1'b1, {(DATA_W-1){1'b0}}};
      end
      if (shifted > DATA_MAX) begin
        return {1'b0, {(DATA_W-1){1'b1}}};
      end
      return shifted[DATA_W-1:0];
    end
  endfunction

  function automatic logic signed [DATA_W-1:0] exp_lut_lookup(
    input logic signed [DATA_W-1:0] logits_delta
  );
    int unsigned lut_idx;
    logic signed [DATA_W-1:0] clipped_delta;
    begin
      if (logits_delta <= 0) begin
        return ONE_FXP;
      end
      clipped_delta = logits_delta;
      if (clipped_delta >= FXP_EIGHT) begin
        return '0;
      end
      lut_idx = clipped_delta / LUT_BUCKET_STEP;
      return LUT_EXP[lut_idx];
    end
  endfunction

  function automatic logic signed [ACC_W-1:0] data_to_acc(
    input logic signed [DATA_W-1:0] value
  );
    begin
      return $signed(value);
    end
  endfunction

  function automatic logic signed [P_DATA_W-1:0] prob_product_to_value(
    input logic signed [PROB_PRODUCT_W-1:0] product
  );
    longint signed shifted_val;
    longint signed max_positive;
    begin
      shifted_val = $signed(product >>> FRAC_W);
      max_positive = (64'sd1 <<< (P_DATA_W - 1)) - 1;
      if (shifted_val <= 0) begin
        return '0;
      end
      if (shifted_val > max_positive) begin
        return {1'b0, {(P_DATA_W-1){1'b1}}};
      end
      return shifted_val[P_DATA_W-1:0];
    end
  endfunction

  assign in_head_s = in_fifo_out_s;
  assign raw_desc_head_s = raw_desc_fifo_out_s;
  assign sqrt_context_head_s = sqrt_context_fifo_out_s;
  assign sqrt_result_head_s = sqrt_result_fifo_out_s;
  assign state_q = (sm_state_q == SM_EMIT);
  assign emit_idx_q = sm_idx_q;
  assign group_count_q = final_group_len_q;
  assign cordic_m_tready_s = sqrt_context_fifo_valid_s && !sqrt_result_fifo_full_s;

  fifo #(
    .DATA_W(IN_ENTRY_W),
    .FIFO_DEPTH(IN_FIFO_DEPTH),
    .WARNING_THR((IN_FIFO_DEPTH > 4) ? (IN_FIFO_DEPTH - 4) : IN_FIFO_DEPTH),
    .FWFT_MODE(1'b1),
    .COMMON_CLOCK_MODE(1'b1)
  ) u_in_fifo (
    .clk(clk),
    .rst_n(rst_n),
    .input_data_i(in_entry_i),
    .input_valid_i(in_valid_i),
    .pop_i(in_fifo_pop_s),
    .output_data_o(in_fifo_out_s),
    .output_valid_o(in_fifo_valid_s),
    .empty_o(in_fifo_empty_s),
    .full_warning_o(in_fifo_full_warning_s),
    .full_o(in_fifo_full_s),
    .single_o(),
    .ready_o()
  );

  fifo #(
    .DATA_W(OUT_ENTRY_W),
    .FIFO_DEPTH(OUT_FIFO_DEPTH),
    .WARNING_THR((OUT_FIFO_DEPTH > 4) ? (OUT_FIFO_DEPTH - 4) : OUT_FIFO_DEPTH),
    .FWFT_MODE(1'b1),
    .COMMON_CLOCK_MODE(1'b1)
  ) u_out_fifo (
    .clk(clk),
    .rst_n(rst_n),
    .input_data_i(out_fifo_in_s),
    .input_valid_i(out_fifo_push_s),
    .pop_i(out_pop_i),
    .output_data_o(out_fifo_out_s),
    .output_valid_o(out_fifo_valid_s),
    .empty_o(out_fifo_empty_s),
    .full_warning_o(out_fifo_full_warning_s),
    .full_o(out_fifo_full_s),
    .single_o(),
    .ready_o()
  );

  softmax_stream_fifo #(.DATA_W(ADDR_W), .FIFO_DEPTH(PIPE_FIFO_DEPTH)) u_addr_fifo (
    .clk, .rst_n, .data_i(addr_fifo_in_s), .push_i(addr_fifo_push_s), .pop_i(addr_fifo_pop_s),
    .data_o(addr_fifo_out_s), .valid_o(addr_fifo_valid_s), .empty_o(addr_fifo_empty_s), .full_o(addr_fifo_full_s)
  );
  softmax_stream_fifo #(.DATA_W(DATA_W), .FIFO_DEPTH(PIPE_FIFO_DEPTH)) u_logits_fifo (
    .clk, .rst_n, .data_i(logits_fifo_in_s), .push_i(logits_fifo_push_s), .pop_i(logits_fifo_pop_s),
    .data_o(logits_fifo_out_s), .valid_o(logits_fifo_valid_s), .empty_o(logits_fifo_empty_s), .full_o(logits_fifo_full_s)
  );
  softmax_stream_fifo #(.DATA_W(RMS_V_W), .FIFO_DEPTH(PIPE_FIFO_DEPTH)) u_rms_fifo (
    .clk, .rst_n, .data_i(rms_fifo_in_s), .push_i(rms_fifo_push_s), .pop_i(rms_fifo_pop_s),
    .data_o(rms_fifo_out_s), .valid_o(rms_fifo_valid_s), .empty_o(rms_fifo_empty_s), .full_o(rms_fifo_full_s)
  );
  softmax_stream_fifo #(.DATA_W(P_DATA_W), .FIFO_DEPTH(PIPE_FIFO_DEPTH)) u_p_fifo (
    .clk, .rst_n, .data_i(p_fifo_in_s), .push_i(p_fifo_push_s), .pop_i(p_fifo_pop_s),
    .data_o(p_fifo_out_s), .valid_o(p_fifo_valid_s), .empty_o(p_fifo_empty_s), .full_o(p_fifo_full_s)
  );
  softmax_stream_fifo #(.DATA_W(ACC_W), .FIFO_DEPTH(PIPE_FIFO_DEPTH)) u_grad_fifo (
    .clk, .rst_n, .data_i(grad_fifo_in_s), .push_i(grad_fifo_push_s), .pop_i(grad_fifo_pop_s),
    .data_o(grad_fifo_out_s), .valid_o(grad_fifo_valid_s), .empty_o(grad_fifo_empty_s), .full_o(grad_fifo_full_s)
  );
  softmax_stream_fifo #(.DATA_W(GROUP_DESC_W), .FIFO_DEPTH(PIPE_FIFO_DEPTH)) u_raw_desc_fifo (
    .clk, .rst_n, .data_i(raw_desc_fifo_in_s), .push_i(raw_desc_fifo_push_s), .pop_i(raw_desc_fifo_pop_s),
    .data_o(raw_desc_fifo_out_s), .valid_o(raw_desc_fifo_valid_s), .empty_o(raw_desc_fifo_empty_s), .full_o(raw_desc_fifo_full_s)
  );
  softmax_stream_fifo #(.DATA_W(GROUP_LEN_W), .FIFO_DEPTH(PIPE_FIFO_DEPTH)) u_final_len_fifo (
    .clk, .rst_n, .data_i(final_len_fifo_in_s), .push_i(final_len_fifo_push_s), .pop_i(final_len_fifo_pop_s),
    .data_o(final_len_fifo_out_s), .valid_o(final_len_fifo_valid_s), .empty_o(final_len_fifo_empty_s), .full_o(final_len_fifo_full_s)
  );
  softmax_stream_fifo #(.DATA_W(DATA_W), .FIFO_DEPTH(PIPE_FIFO_DEPTH)) u_raw_fifo (
    .clk, .rst_n, .data_i(raw_fifo_in_s), .push_i(raw_fifo_push_s), .pop_i(raw_fifo_pop_s),
    .data_o(raw_fifo_out_s), .valid_o(raw_fifo_valid_s), .empty_o(raw_fifo_empty_s), .full_o(raw_fifo_full_s)
  );
  softmax_stream_fifo #(.DATA_W(DATA_W), .FIFO_DEPTH(PIPE_FIFO_DEPTH)) u_raw_norm_fifo (
    .clk, .rst_n, .data_i(raw_norm_fifo_in_s), .push_i(raw_norm_fifo_push_s), .pop_i(raw_norm_fifo_pop_s),
    .data_o(raw_norm_fifo_out_s), .valid_o(raw_norm_fifo_valid_s), .empty_o(raw_norm_fifo_empty_s), .full_o(raw_norm_fifo_full_s)
  );
  softmax_stream_fifo #(.DATA_W(RMS_V_W), .FIFO_DEPTH(PIPE_FIFO_DEPTH)) u_rms_next_fifo (
    .clk, .rst_n, .data_i(rms_next_fifo_in_s), .push_i(rms_next_fifo_push_s), .pop_i(rms_next_fifo_pop_s),
    .data_o(rms_next_fifo_out_s), .valid_o(rms_next_fifo_valid_s), .empty_o(rms_next_fifo_empty_s), .full_o(rms_next_fifo_full_s)
  );
  softmax_stream_fifo #(.DATA_W(RMS_V_W), .FIFO_DEPTH(PIPE_FIFO_DEPTH)) u_rms_wb_fifo (
    .clk, .rst_n, .data_i(rms_wb_fifo_in_s), .push_i(rms_wb_fifo_push_s), .pop_i(rms_wb_fifo_pop_s),
    .data_o(rms_wb_fifo_out_s), .valid_o(rms_wb_fifo_valid_s), .empty_o(rms_wb_fifo_empty_s), .full_o(rms_wb_fifo_full_s)
  );
  softmax_stream_fifo #(.DATA_W(SQRT_CONTEXT_W), .FIFO_DEPTH(PIPE_FIFO_DEPTH)) u_sqrt_context_fifo (
    .clk, .rst_n, .data_i(sqrt_context_fifo_in_s), .push_i(sqrt_context_fifo_push_s), .pop_i(sqrt_context_fifo_pop_s),
    .data_o(sqrt_context_fifo_out_s), .valid_o(sqrt_context_fifo_valid_s), .empty_o(sqrt_context_fifo_empty_s), .full_o(sqrt_context_fifo_full_s)
  );
  softmax_stream_fifo #(.DATA_W(SQRT_RESULT_W), .FIFO_DEPTH(PIPE_FIFO_DEPTH)) u_sqrt_result_fifo (
    .clk, .rst_n, .data_i(sqrt_result_fifo_in_s), .push_i(sqrt_result_fifo_push_s), .pop_i(sqrt_result_fifo_pop_s),
    .data_o(sqrt_result_fifo_out_s), .valid_o(sqrt_result_fifo_valid_s), .empty_o(sqrt_result_fifo_empty_s), .full_o(sqrt_result_fifo_full_s)
  );
  softmax_stream_fifo #(.DATA_W(DATA_W), .FIFO_DEPTH(PIPE_FIFO_DEPTH)) u_norm_fifo (
    .clk, .rst_n, .data_i(norm_fifo_in_s), .push_i(norm_fifo_push_s), .pop_i(norm_fifo_pop_s),
    .data_o(norm_fifo_out_s), .valid_o(norm_fifo_valid_s), .empty_o(norm_fifo_empty_s), .full_o(norm_fifo_full_s)
  );
  softmax_stream_fifo #(.DATA_W(DATA_W), .FIFO_DEPTH(PIPE_FIFO_DEPTH)) u_logits_new_fifo (
    .clk, .rst_n, .data_i(logits_new_fifo_in_s), .push_i(logits_new_fifo_push_s), .pop_i(logits_new_fifo_pop_s),
    .data_o(logits_new_fifo_out_s), .valid_o(logits_new_fifo_valid_s), .empty_o(logits_new_fifo_empty_s), .full_o(logits_new_fifo_full_s)
  );
  softmax_stream_fifo #(.DATA_W(DATA_W), .FIFO_DEPTH(PIPE_FIFO_DEPTH)) u_logits_wb_fifo (
    .clk, .rst_n, .data_i(logits_wb_fifo_in_s), .push_i(logits_wb_fifo_push_s), .pop_i(logits_wb_fifo_pop_s),
    .data_o(logits_wb_fifo_out_s), .valid_o(logits_wb_fifo_valid_s), .empty_o(logits_wb_fifo_empty_s), .full_o(logits_wb_fifo_full_s)
  );
  softmax_stream_fifo #(.DATA_W(P_DATA_W), .FIFO_DEPTH(PIPE_FIFO_DEPTH)) u_p_new_fifo (
    .clk, .rst_n, .data_i(p_new_fifo_in_s), .push_i(p_new_fifo_push_s), .pop_i(p_new_fifo_pop_s),
    .data_o(p_new_fifo_out_s), .valid_o(p_new_fifo_valid_s), .empty_o(p_new_fifo_empty_s), .full_o(p_new_fifo_full_s)
  );
  softmax_stream_fifo #(.DATA_W(DATA_W), .FIFO_DEPTH(PIPE_FIFO_DEPTH)) u_recip_context_fifo (
    .clk, .rst_n, .data_i(recip_context_fifo_in_s), .push_i(recip_context_fifo_push_s), .pop_i(recip_context_fifo_pop_s),
    .data_o(recip_context_fifo_out_s), .valid_o(recip_context_fifo_valid_s), .empty_o(recip_context_fifo_empty_s), .full_o(recip_context_fifo_full_s)
  );
  softmax_stream_fifo #(.DATA_W(1), .FIFO_DEPTH(PIPE_FIFO_DEPTH)) u_grad_div_context_fifo (
    .clk, .rst_n,
    .data_i(grad_div_context_fifo_in_s),
    .push_i(grad_div_context_fifo_push_s),
    .pop_i(grad_div_context_fifo_pop_s),
    .data_o(grad_div_context_fifo_out_s),
    .valid_o(grad_div_context_fifo_valid_s),
    .empty_o(grad_div_context_fifo_empty_s),
    .full_o(grad_div_context_fifo_full_s)
  );

  softmax_sqrt_cordic_u48_o32 u_sqrt_cordic (
    .aclk(clk),
    .aresetn(rst_n),
    .s_axis_cartesian_tvalid(cordic_s_tvalid_s),
    .s_axis_cartesian_tready(cordic_s_tready_s),
    .s_axis_cartesian_tdata(cordic_s_tdata_s),
    .m_axis_dout_tvalid(cordic_m_tvalid_s),
    .m_axis_dout_tready(cordic_m_tready_s),
    .m_axis_dout_tdata(cordic_m_tdata_s)
  );

  softmax_recip_divider_u64 u_recip_divider (
    .aclk(clk),
    .aresetn(rst_n),
    .s_axis_divisor_tvalid(recip_divisor_tvalid_s),
    .s_axis_divisor_tready(recip_divisor_tready_s),
    .s_axis_divisor_tdata(recip_divisor_tdata_s),
    .s_axis_dividend_tvalid(recip_dividend_tvalid_s),
    .s_axis_dividend_tready(recip_dividend_tready_s),
    .s_axis_dividend_tdata(recip_dividend_tdata_s),
    .m_axis_dout_tvalid(recip_m_tvalid_s),
    .m_axis_dout_tready(recip_m_tready_s),
    .m_axis_dout_tdata(recip_m_tdata_s)
  );

  softmax_grad_rms_divider_s64 u_grad_rms_divider (
    .aclk(clk),
    .aresetn(rst_n),
    .s_axis_divisor_tvalid(grad_divisor_tvalid_s),
    .s_axis_divisor_tready(grad_divisor_tready_s),
    .s_axis_divisor_tdata(grad_divisor_tdata_s),
    .s_axis_dividend_tvalid(grad_dividend_tvalid_s),
    .s_axis_dividend_tready(grad_dividend_tready_s),
    .s_axis_dividend_tdata(grad_dividend_tdata_s),
    .m_axis_dout_tvalid(grad_div_m_tvalid_s),
    .m_axis_dout_tready(grad_div_m_tready_s),
    .m_axis_dout_tdata(grad_div_m_tdata_s)
  );

  always_comb begin : PIPELINE_CONTROL
    logic sample_fifo_space;
    logic descriptor_fifo_space;
    logic signed [ACC_W-1:0] raw_acc;
    logic signed [RMS_V_W-1:0] grad_sq;
    logic signed [RMS_V_W-1:0] rms_next_value;
    logic signed [RMS_V_W-1:0] sqrt_radicand;
    logic [SQRT_EXP_W-1:0] sqrt_exponent;
    logic [DATA_W-1:0] rms_grad;
    logic signed [DATA_W-1:0] logits_delta;
    logic signed [DATA_W-1:0] exp_value;
    logic prob_format_ready;
    logic prob_mul_ready;
    logic recip_dividend_done;
    logic recip_divisor_done;
    logic grad_dividend_done;
    logic grad_divisor_done;
    int unsigned group_flag_idx;

    fetch_pending_d = fetch_pending_q;
    held_valid_d = held_valid_q;
    held_sample_d = held_sample_q;
    fetched_entry_d = fetched_entry_q;
    collect_count_d = collect_count_q;
    collect_sum_d = collect_sum_q;

    raw_group_active_d = raw_group_active_q;
    raw_group_len_d = raw_group_len_q;
    raw_group_count_d = raw_group_count_q;
    raw_group_sum_d = raw_group_sum_q;
    raw_pipe_valid_d = raw_pipe_valid_q;
    raw_pipe_singleton_d = raw_pipe_singleton_q;
    raw_pipe_p_grad_d = raw_pipe_p_grad_q;
    raw_pipe_sum_p_d = raw_pipe_sum_p_q;

    final_group_active_d = final_group_active_q;
    final_group_len_d = final_group_len_q;
    sm_state_d = sm_state_q;
    sm_idx_d = sm_idx_q;
    sm_max_d = sm_max_q;
    sm_sum_d = sm_sum_q;
    recip_dividend_sent_d = recip_dividend_sent_q;
    recip_divisor_sent_d = recip_divisor_sent_q;
    grad_dividend_sent_d = grad_dividend_sent_q;
    grad_divisor_sent_d = grad_divisor_sent_q;
    prob_mul_valid_d = prob_mul_valid_q;
    prob_mul_product_d = prob_mul_product_q;
    prob_format_valid_d = prob_format_valid_q;
    prob_format_data_d = prob_format_data_q;

    in_fifo_pop_s = 1'b0;
    out_fifo_push_s = 1'b0;
    out_fifo_in_s = '0;
    local_rd_ce_o = 1'b0;
    local_rd_addr_o = bank_valid_len_i;
    p_wr_valid_o = 1'b0;
    p_wr_addr_o = bank_valid_len_i;
    p_wr_data_o = '0;

    addr_fifo_push_s = 1'b0;
    addr_fifo_pop_s = 1'b0;
    addr_fifo_in_s = '0;
    logits_fifo_push_s = 1'b0;
    logits_fifo_pop_s = 1'b0;
    logits_fifo_in_s = '0;
    rms_fifo_push_s = 1'b0;
    rms_fifo_pop_s = 1'b0;
    rms_fifo_in_s = '0;
    p_fifo_push_s = 1'b0;
    p_fifo_pop_s = 1'b0;
    p_fifo_in_s = '0;
    grad_fifo_push_s = 1'b0;
    grad_fifo_pop_s = 1'b0;
    grad_fifo_in_s = '0;
    raw_desc_fifo_push_s = 1'b0;
    raw_desc_fifo_pop_s = 1'b0;
    raw_desc_fifo_in_s = '0;
    final_len_fifo_push_s = 1'b0;
    final_len_fifo_pop_s = 1'b0;
    final_len_fifo_in_s = '0;
    raw_fifo_push_s = 1'b0;
    raw_fifo_pop_s = 1'b0;
    raw_fifo_in_s = '0;
    raw_norm_fifo_push_s = 1'b0;
    raw_norm_fifo_pop_s = 1'b0;
    raw_norm_fifo_in_s = '0;
    rms_next_fifo_push_s = 1'b0;
    rms_next_fifo_pop_s = 1'b0;
    rms_next_fifo_in_s = '0;
    rms_wb_fifo_push_s = 1'b0;
    rms_wb_fifo_pop_s = 1'b0;
    rms_wb_fifo_in_s = '0;
    sqrt_context_fifo_push_s = 1'b0;
    sqrt_context_fifo_pop_s = 1'b0;
    sqrt_context_fifo_in_s = '0;
    sqrt_result_fifo_push_s = 1'b0;
    sqrt_result_fifo_pop_s = 1'b0;
    sqrt_result_fifo_in_s = '0;
    norm_fifo_push_s = 1'b0;
    norm_fifo_pop_s = 1'b0;
    norm_fifo_in_s = '0;
    logits_new_fifo_push_s = 1'b0;
    logits_new_fifo_pop_s = 1'b0;
    logits_new_fifo_in_s = '0;
    logits_wb_fifo_push_s = 1'b0;
    logits_wb_fifo_pop_s = 1'b0;
    logits_wb_fifo_in_s = '0;
    p_new_fifo_push_s = 1'b0;
    p_new_fifo_pop_s = 1'b0;
    p_new_fifo_in_s = '0;
    recip_context_fifo_push_s = 1'b0;
    recip_context_fifo_pop_s = 1'b0;
    recip_context_fifo_in_s = '0;
    grad_div_context_fifo_push_s = 1'b0;
    grad_div_context_fifo_pop_s = 1'b0;
    grad_div_context_fifo_in_s = 1'b1;

    cordic_s_tvalid_s = 1'b0;
    cordic_s_tdata_s = '0;
    recip_dividend_tvalid_s = 1'b0;
    recip_divisor_tvalid_s = 1'b0;
    recip_dividend_tdata_s = RECIP_NUMERATOR + ($unsigned(sm_sum_q) >> 1);
    recip_divisor_tdata_s = $unsigned(sm_sum_q);
    recip_m_tready_s = 1'b0;
    grad_dividend_tvalid_s = 1'b0;
    grad_divisor_tvalid_s = 1'b0;
    grad_dividend_tdata_s = '0;
    grad_divisor_tdata_s = {{DATA_W{1'b0}}, sqrt_result_head_s.rms_grad};
    grad_div_m_tready_s = 1'b0;

    if (sqrt_result_head_s.rms_grad != '0) begin
      grad_dividend_tdata_s =
        $signed({{DATA_W{sqrt_result_head_s.raw_grad[DATA_W-1]}},
                 sqrt_result_head_s.raw_grad}) <<< FRAC_W;
    end else begin
      grad_divisor_tdata_s = {{((2*DATA_W)-1){1'b0}}, 1'b1};
    end

    sm_logits_we_s = 1'b0;
    sm_exp_we_s = 1'b0;
    sm_buf_widx_s = '0;
    sm_logits_wdata_s = '0;
    sm_exp_wdata_s = '0;

    sample_fifo_space =
      !addr_fifo_full_s && !logits_fifo_full_s && !rms_fifo_full_s &&
      !p_fifo_full_s && !grad_fifo_full_s;
    descriptor_fifo_space = !raw_desc_fifo_full_s && !final_len_fifo_full_s;

    // The Divider Generator supplies the reciprocal quotient. Its ordered
    // context stream feeds an elastic multiply stage and then a separate
    // shift/saturation stage before the existing p_new FIFO.
    prob_format_ready = !prob_format_valid_q || !p_new_fifo_full_s;
    prob_mul_ready = !prob_mul_valid_q || prob_format_ready;

    if (prob_format_valid_q && !p_new_fifo_full_s) begin
      p_new_fifo_push_s = 1'b1;
      p_new_fifo_in_s = prob_format_data_q;
    end

    if (prob_format_ready) begin
      prob_format_valid_d = prob_mul_valid_q;
      if (prob_mul_valid_q) begin
        prob_format_data_d = prob_product_to_value(prob_mul_product_q);
      end
    end

    if (prob_mul_ready) begin
      prob_mul_valid_d = 1'b0;
    end

    recip_m_tready_s = recip_context_fifo_valid_s && prob_mul_ready;
    if (recip_m_tvalid_s && recip_m_tready_s) begin
      recip_context_fifo_pop_s = 1'b1;
      prob_mul_valid_d = 1'b1;
      prob_mul_product_d =
        $signed(recip_context_fifo_out_s) * $signed(recip_m_tdata_s[2*ACC_W-1:ACC_W]);
    end

    // Step 1: fetch each local sample, preserve a next-group starter, and
    // push each data type only into its own FIFO.
    if (held_valid_q) begin
      if (held_sample_q.group_start && (collect_count_q != '0)) begin
        if (descriptor_fifo_space) begin
          raw_desc_fifo_push_s = 1'b1;
          raw_desc_fifo_in_s = {collect_count_q, collect_sum_q};
          final_len_fifo_push_s = 1'b1;
          final_len_fifo_in_s = collect_count_q;
          collect_count_d = '0;
          collect_sum_d = '0;
        end
      end else if (sample_fifo_space) begin
        addr_fifo_push_s = 1'b1;
        addr_fifo_in_s = held_sample_q.addr;
        logits_fifo_push_s = 1'b1;
        logits_fifo_in_s = held_sample_q.logits;
        rms_fifo_push_s = 1'b1;
        rms_fifo_in_s = held_sample_q.rms;
        p_fifo_push_s = 1'b1;
        p_fifo_in_s = held_sample_q.p;
        grad_fifo_push_s = 1'b1;
        grad_fifo_in_s = held_sample_q.grad_p;
        collect_sum_d =
          ((collect_count_q == '0) ? '0 : collect_sum_q) +
          fxp_mul_p_grad_acc(held_sample_q.p, held_sample_q.grad_p);
        collect_count_d = collect_count_q + 1'b1;
        held_valid_d = 1'b0;
      end
    end else if (fetch_pending_q) begin
      group_flag_idx = int'(fetched_entry_q.addr) & (GROUP_FLAG_WORD_W - 1);
      held_sample_d.addr = fetched_entry_q.addr;
      held_sample_d.p = p_i;
      held_sample_d.grad_p = grad_p_i;
      held_sample_d.logits = fetched_entry_q.logits;
      held_sample_d.rms = fetched_entry_q.rms;
      held_sample_d.group_start = group_start_flag_word_i[group_flag_idx];
      held_valid_d = 1'b1;
      fetch_pending_d = 1'b0;
    end else if (in_fifo_valid_s) begin
      local_rd_ce_o = 1'b1;
      local_rd_addr_o = in_head_s.addr;
      fetched_entry_d = in_head_s;
      fetch_pending_d = 1'b1;
      in_fifo_pop_s = 1'b1;
    end else if (
      input_done_i && in_fifo_empty_s && (collect_count_q != '0) &&
      descriptor_fifo_space
    ) begin
      raw_desc_fifo_push_s = 1'b1;
      raw_desc_fifo_in_s = {collect_count_q, collect_sum_q};
      final_len_fifo_push_s = 1'b1;
      final_len_fifo_in_s = collect_count_q;
      collect_count_d = '0;
      collect_sum_d = '0;
    end

    if (held_valid_q && !held_sample_q.group_start && (collect_count_q >= MAX_GROUP_LEN)) begin
      $fatal(1, "softmax_bank_engine: group length exceeds MAX_GROUP_LEN=%0d", MAX_GROUP_LEN);
    end

    // Step 2: latch one group descriptor and run the two-cycle raw-grad pipe.
    if (!raw_group_active_q && raw_desc_fifo_valid_s) begin
      raw_desc_fifo_pop_s = 1'b1;
      raw_group_active_d = 1'b1;
      raw_group_len_d = raw_desc_head_s.len;
      raw_group_count_d = '0;
      raw_group_sum_d = raw_desc_head_s.sum_mul;
    end

    if (raw_pipe_valid_q && !raw_fifo_full_s) begin
      raw_acc = raw_pipe_p_grad_q - raw_pipe_sum_p_q;
      raw_fifo_push_s = 1'b1;
      raw_fifo_in_s = raw_pipe_singleton_q ? '0 : saturate_data(raw_acc);
      raw_pipe_valid_d = 1'b0;
    end

    if (
      raw_group_active_q && p_fifo_valid_s && grad_fifo_valid_s &&
      !raw_pipe_valid_d
    ) begin
      p_fifo_pop_s = 1'b1;
      grad_fifo_pop_s = 1'b1;
      raw_pipe_p_grad_d = fxp_mul_p_grad_acc(p_fifo_out_s, grad_fifo_out_s);
      raw_pipe_sum_p_d = fxp_mul_acc_p(raw_group_sum_q, p_fifo_out_s);
      raw_pipe_singleton_d = (raw_group_len_q == 1);
      raw_pipe_valid_d = 1'b1;
      raw_group_count_d = raw_group_count_q + 1'b1;
      if ((raw_group_count_q + 1'b1) >= raw_group_len_q) begin
        raw_group_active_d = 1'b0;
      end
    end

    // Allocate the one group-local softmax workset. Other stages are no
    // longer coupled to this slot and may process later groups concurrently.
    if (!final_group_active_q && (sm_state_q == SM_WAIT) && final_len_fifo_valid_s) begin
      final_len_fifo_pop_s = 1'b1;
      final_group_active_d = 1'b1;
      final_group_len_d = final_len_fifo_out_s;
      sm_idx_d = '0;
      sm_max_d = '0;
      sm_sum_d = '0;
      sm_state_d = SM_COLLECT;
    end

    // Step 3: sample-stream RMS update. The writeback copy removes the old
    // single-group wb_rms_q dependency.
    if (
      raw_fifo_valid_s && rms_fifo_valid_s && !raw_norm_fifo_full_s &&
      !rms_next_fifo_full_s && !rms_wb_fifo_full_s
    ) begin
      grad_sq = data_square_to_state(raw_fifo_out_s);
      if (update_mode_i) begin
        rms_next_value =
          fxp_mul_state_coeff(rms_fifo_out_s, RMSPROP_ALPHA_FXP) +
          fxp_mul_state_coeff(grad_sq, ONE_MINUS_ALPHA_FXP);
      end else begin
        rms_next_value = rms_fifo_out_s;
      end
      raw_fifo_pop_s = 1'b1;
      rms_fifo_pop_s = 1'b1;
      raw_norm_fifo_push_s = 1'b1;
      raw_norm_fifo_in_s = raw_fifo_out_s;
      rms_next_fifo_push_s = 1'b1;
      rms_next_fifo_in_s = rms_next_value;
      rms_wb_fifo_push_s = 1'b1;
      rms_wb_fifo_in_s = rms_next_value;
    end

    // Step 4a: normalize one radicand and submit it to the AXI CORDIC. The
    // context FIFO tracks every accepted request across stalls and latency.
    if (
      raw_norm_fifo_valid_s && rms_next_fifo_valid_s && !sqrt_context_fifo_full_s
    ) begin
      sqrt_radicand = add_eps_to_state(rms_next_fifo_out_s);
      sqrt_exponent = state_sqrt_even_exponent(sqrt_radicand);
      cordic_s_tvalid_s = 1'b1;
      cordic_s_tdata_s = state_sqrt_mantissa(sqrt_radicand, sqrt_exponent);
      if (cordic_s_tready_s) begin
        raw_norm_fifo_pop_s = 1'b1;
        rms_next_fifo_pop_s = 1'b1;
        sqrt_context_fifo_push_s = 1'b1;
        sqrt_context_fifo_in_s = {raw_norm_fifo_out_s, sqrt_exponent};
      end
    end

    // Step 4b: receive the ordered CORDIC result and decouple it from the
    // divider with a handwritten result FIFO.
    if (cordic_m_tvalid_s && cordic_m_tready_s) begin
      rms_grad = cordic_sqrt_to_data(
        cordic_m_tdata_s,
        sqrt_context_head_s.exponent
      );
      sqrt_context_fifo_pop_s = 1'b1;
      sqrt_result_fifo_push_s = 1'b1;
      sqrt_result_fifo_in_s = {sqrt_context_head_s.raw_grad, rms_grad};
    end

    // Step 4c: bypass normalization outside update mode. Update requests use
    // an independent signed Divider Generator and a token FIFO for ordering.
    if (sqrt_result_fifo_valid_s && !update_mode_i && !norm_fifo_full_s) begin
      sqrt_result_fifo_pop_s = 1'b1;
      norm_fifo_push_s = 1'b1;
      norm_fifo_in_s = '0;
    end

    grad_dividend_done = grad_dividend_sent_q;
    grad_divisor_done = grad_divisor_sent_q;
    if (
      update_mode_i && sqrt_result_fifo_valid_s &&
      (grad_dividend_sent_q || grad_divisor_sent_q || !grad_div_context_fifo_full_s)
    ) begin
      if (!grad_dividend_sent_q) begin
        grad_dividend_tvalid_s = 1'b1;
        if (grad_dividend_tready_s) begin
          grad_dividend_done = 1'b1;
        end
      end
      if (!grad_divisor_sent_q) begin
        grad_divisor_tvalid_s = 1'b1;
        if (grad_divisor_tready_s) begin
          grad_divisor_done = 1'b1;
        end
      end
      if (grad_dividend_done && grad_divisor_done) begin
        grad_dividend_sent_d = 1'b0;
        grad_divisor_sent_d = 1'b0;
        sqrt_result_fifo_pop_s = 1'b1;
        grad_div_context_fifo_push_s = 1'b1;
      end else begin
        grad_dividend_sent_d = grad_dividend_done;
        grad_divisor_sent_d = grad_divisor_done;
      end
    end

    grad_div_m_tready_s = grad_div_context_fifo_valid_s && !norm_fifo_full_s;
    if (grad_div_m_tvalid_s && grad_div_m_tready_s) begin
      grad_div_context_fifo_pop_s = 1'b1;
      norm_fifo_push_s = 1'b1;
      norm_fifo_in_s = grad_div_quotient_to_data(
        $signed(grad_div_m_tdata_s[(4*DATA_W)-1 -: (2*DATA_W)])
      );
    end

    // Step 5: duplicate logits_new only for its two real consumers.
    if (
      logits_fifo_valid_s && norm_fifo_valid_s && !logits_new_fifo_full_s &&
      !logits_wb_fifo_full_s
    ) begin
      logits_delta = fxp_mul_lr_grad(LR_FXP, norm_fifo_out_s);
      logits_new_fifo_in_s = update_mode_i ?
        (logits_fifo_out_s - logits_delta) : logits_fifo_out_s;
      logits_fifo_pop_s = 1'b1;
      norm_fifo_pop_s = 1'b1;
      logits_new_fifo_push_s = 1'b1;
      logits_wb_fifo_push_s = 1'b1;
      logits_wb_fifo_in_s = logits_new_fifo_in_s;
    end

    // Step 7: final writeback is a separate sample stage. This can emit an
    // earlier group while group softmax starts work on the next one.
    // p_sb_ready_i gates the pulse: the demux/sideband path must be able to
    // hold the entry (the writeback FIFOs pop atomically with the pulse,
    // so a blocked pulse would silently lose the row's p write).
    if (
      addr_fifo_valid_s && p_new_fifo_valid_s && logits_wb_fifo_valid_s &&
      rms_wb_fifo_valid_s && p_sb_ready_i &&
      (!update_mode_i || (!out_fifo_full_warning_s && !out_fifo_full_s))
    ) begin
      addr_fifo_pop_s = 1'b1;
      p_new_fifo_pop_s = 1'b1;
      logits_wb_fifo_pop_s = 1'b1;
      rms_wb_fifo_pop_s = 1'b1;
      p_wr_valid_o = 1'b1;
      p_wr_addr_o = addr_fifo_out_s;
      p_wr_data_o = p_new_fifo_out_s;
      if (update_mode_i) begin
        out_fifo_push_s = 1'b1;
        out_fifo_in_s = {
          addr_fifo_out_s,
          logits_wb_fifo_out_s,
          rms_wb_fifo_out_s
        };
      end
    end

    // Step 6: retain the existing one-group COLLECT/EXP/EMIT algorithm, but
    // emit p_new into a FIFO instead of directly coupling to writeback.
    unique case (sm_state_q)
      SM_WAIT: begin
      end

      SM_COLLECT: begin
        if (logits_new_fifo_valid_s) begin
          logits_new_fifo_pop_s = 1'b1;
          sm_logits_we_s = 1'b1;
          sm_buf_widx_s = sm_idx_q;
          sm_logits_wdata_s = logits_new_fifo_out_s;
          if ((sm_idx_q == '0) || (logits_new_fifo_out_s > sm_max_q)) begin
            sm_max_d = logits_new_fifo_out_s;
          end
          if ((sm_idx_q + 1'b1) >= final_group_len_q) begin
            sm_idx_d = '0;
            sm_sum_d = '0;
            sm_state_d = SM_EXP;
          end else begin
            sm_idx_d = sm_idx_q + 1'b1;
          end
        end
      end

      SM_EXP: begin
        exp_value = exp_lut_lookup(sm_max_q - sm_logits_q[sm_idx_q]);
        sm_exp_we_s = 1'b1;
        sm_buf_widx_s = sm_idx_q;
        sm_exp_wdata_s = exp_value;
        sm_sum_d = sm_sum_q + data_to_acc(exp_value);
        if ((sm_idx_q + 1'b1) >= final_group_len_q) begin
          sm_idx_d = '0;
          sm_state_d = SM_EMIT;
        end else begin
          sm_idx_d = sm_idx_q + 1'b1;
        end
      end

      SM_EMIT: begin
        // Each sample owns one pair of independently handshaken AXI input
        // beats. Advance only after both operands are accepted, then enqueue
        // the matching exp value for the ordered Divider output stream.
        if (
          recip_dividend_sent_q || recip_divisor_sent_q ||
          !recip_context_fifo_full_s
        ) begin
          recip_dividend_done = recip_dividend_sent_q;
          recip_divisor_done = recip_divisor_sent_q;

          if (!recip_dividend_sent_q) begin
            recip_dividend_tvalid_s = 1'b1;
            if (recip_dividend_tready_s) begin
              recip_dividend_done = 1'b1;
            end
          end
          if (!recip_divisor_sent_q) begin
            recip_divisor_tvalid_s = 1'b1;
            if (recip_divisor_tready_s) begin
              recip_divisor_done = 1'b1;
            end
          end

          if (recip_dividend_done && recip_divisor_done) begin
            recip_dividend_sent_d = 1'b0;
            recip_divisor_sent_d = 1'b0;
            recip_context_fifo_push_s = 1'b1;
            recip_context_fifo_in_s = sm_exp_q[sm_idx_q];
            if ((sm_idx_q + 1'b1) >= final_group_len_q) begin
              sm_idx_d = '0;
              sm_state_d = SM_WAIT;
              final_group_active_d = 1'b0;
              final_group_len_d = '0;
            end else begin
              sm_idx_d = sm_idx_q + 1'b1;
            end
          end else begin
            recip_dividend_sent_d = recip_dividend_done;
            recip_divisor_sent_d = recip_divisor_done;
          end
        end
      end

      default: sm_state_d = SM_WAIT;
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      fetch_pending_q <= 1'b0;
      held_valid_q <= 1'b0;
      held_sample_q <= '0;
      fetched_entry_q <= '0;
      collect_count_q <= '0;
      collect_sum_q <= '0;
      raw_group_active_q <= 1'b0;
      raw_group_len_q <= '0;
      raw_group_count_q <= '0;
      raw_group_sum_q <= '0;
      raw_pipe_valid_q <= 1'b0;
      raw_pipe_singleton_q <= 1'b0;
      raw_pipe_p_grad_q <= '0;
      raw_pipe_sum_p_q <= '0;
      final_group_active_q <= 1'b0;
      final_group_len_q <= '0;
      sm_state_q <= SM_WAIT;
      sm_idx_q <= '0;
      sm_max_q <= '0;
      sm_sum_q <= '0;
      recip_dividend_sent_q <= 1'b0;
      recip_divisor_sent_q <= 1'b0;
      grad_dividend_sent_q <= 1'b0;
      grad_divisor_sent_q <= 1'b0;
      prob_mul_valid_q <= 1'b0;
      prob_mul_product_q <= '0;
      prob_format_valid_q <= 1'b0;
      prob_format_data_q <= '0;
      for (int idx = 0; idx < MAX_GROUP_LEN; idx++) begin
        sm_logits_q[idx] <= '0;
        sm_exp_q[idx] <= '0;
      end
    end else if (!enable_i) begin
      fetch_pending_q <= 1'b0;
      held_valid_q <= 1'b0;
      held_sample_q <= '0;
      fetched_entry_q <= '0;
      collect_count_q <= '0;
      collect_sum_q <= '0;
      raw_group_active_q <= 1'b0;
      raw_group_len_q <= '0;
      raw_group_count_q <= '0;
      raw_group_sum_q <= '0;
      raw_pipe_valid_q <= 1'b0;
      raw_pipe_singleton_q <= 1'b0;
      raw_pipe_p_grad_q <= '0;
      raw_pipe_sum_p_q <= '0;
      final_group_active_q <= 1'b0;
      final_group_len_q <= '0;
      sm_state_q <= SM_WAIT;
      sm_idx_q <= '0;
      sm_max_q <= '0;
      sm_sum_q <= '0;
      recip_dividend_sent_q <= 1'b0;
      recip_divisor_sent_q <= 1'b0;
      grad_dividend_sent_q <= 1'b0;
      grad_divisor_sent_q <= 1'b0;
      prob_mul_valid_q <= 1'b0;
      prob_mul_product_q <= '0;
      prob_format_valid_q <= 1'b0;
      prob_format_data_q <= '0;
      for (int idx = 0; idx < MAX_GROUP_LEN; idx++) begin
        sm_logits_q[idx] <= '0;
        sm_exp_q[idx] <= '0;
      end
    end else begin
      fetch_pending_q <= fetch_pending_d;
      held_valid_q <= held_valid_d;
      held_sample_q <= held_sample_d;
      fetched_entry_q <= fetched_entry_d;
      collect_count_q <= collect_count_d;
      collect_sum_q <= collect_sum_d;
      raw_group_active_q <= raw_group_active_d;
      raw_group_len_q <= raw_group_len_d;
      raw_group_count_q <= raw_group_count_d;
      raw_group_sum_q <= raw_group_sum_d;
      raw_pipe_valid_q <= raw_pipe_valid_d;
      raw_pipe_singleton_q <= raw_pipe_singleton_d;
      raw_pipe_p_grad_q <= raw_pipe_p_grad_d;
      raw_pipe_sum_p_q <= raw_pipe_sum_p_d;
      final_group_active_q <= final_group_active_d;
      final_group_len_q <= final_group_len_d;
      sm_state_q <= sm_state_d;
      sm_idx_q <= sm_idx_d;
      sm_max_q <= sm_max_d;
      sm_sum_q <= sm_sum_d;
      recip_dividend_sent_q <= recip_dividend_sent_d;
      recip_divisor_sent_q <= recip_divisor_sent_d;
      grad_dividend_sent_q <= grad_dividend_sent_d;
      grad_divisor_sent_q <= grad_divisor_sent_d;
      prob_mul_valid_q <= prob_mul_valid_d;
      prob_mul_product_q <= prob_mul_product_d;
      prob_format_valid_q <= prob_format_valid_d;
      prob_format_data_q <= prob_format_data_d;
      if (sm_logits_we_s) begin
        sm_logits_q[sm_buf_widx_s] <= sm_logits_wdata_s;
      end
      if (sm_exp_we_s) begin
        sm_exp_q[sm_buf_widx_s] <= sm_exp_wdata_s;
      end
      assert (!(cordic_m_tvalid_s && !sqrt_context_fifo_valid_s))
        else $fatal(1, "softmax_bank_engine: CORDIC output has no matching context");
      assert (!(recip_m_tvalid_s && !recip_context_fifo_valid_s))
        else $fatal(1, "softmax_bank_engine: reciprocal divider output has no matching context");
      assert (!(grad_div_m_tvalid_s && !grad_div_context_fifo_valid_s))
        else $fatal(1, "softmax_bank_engine: grad/rms divider output has no matching context");
      assert (!(recip_context_fifo_push_s && (sm_sum_q <= 0)))
        else $fatal(1, "softmax_bank_engine: reciprocal divisor must be positive");
    end
  end

  assign in_ready_o = !in_fifo_full_warning_s && (!update_mode_i || !out_fifo_full_warning_s);
  assign out_entry_o = out_fifo_out_s;
  assign out_valid_o = out_fifo_valid_s;
  assign finish_o =
    enable_i && input_done_i &&
    !fetch_pending_q && !held_valid_q && (collect_count_q == '0) &&
    in_fifo_empty_s && addr_fifo_empty_s && logits_fifo_empty_s &&
    rms_fifo_empty_s && p_fifo_empty_s && grad_fifo_empty_s &&
    raw_desc_fifo_empty_s && final_len_fifo_empty_s && raw_fifo_empty_s &&
    raw_norm_fifo_empty_s && rms_next_fifo_empty_s && rms_wb_fifo_empty_s &&
    sqrt_context_fifo_empty_s && sqrt_result_fifo_empty_s &&
    grad_div_context_fifo_empty_s && !grad_div_m_tvalid_s &&
    !grad_dividend_sent_q && !grad_divisor_sent_q &&
    !cordic_m_tvalid_s && norm_fifo_empty_s && logits_new_fifo_empty_s &&
    logits_wb_fifo_empty_s && p_new_fifo_empty_s &&
    recip_context_fifo_empty_s && !recip_m_tvalid_s &&
    !recip_dividend_sent_q && !recip_divisor_sent_q &&
    !prob_mul_valid_q && !prob_format_valid_q &&
    !raw_group_active_q && !raw_pipe_valid_q &&
    !final_group_active_q && (sm_state_q == SM_WAIT) && out_fifo_empty_s;

`ifndef SYNTHESIS
  // ---- v4 descriptor/sample accounting debug (sim only): proves whether
  // the group descriptors cover every sample pushed (stall forensics) ----
  logic [15:0] dbg_in_pop_cnt_q, dbg_sample_push_cnt_q, dbg_desc_push_cnt_q,
               dbg_desc_len_sum_q, dbg_raw_pop_cnt_q, dbg_flush_push_cnt_q;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dbg_in_pop_cnt_q <= '0;
      dbg_sample_push_cnt_q <= '0;
      dbg_desc_push_cnt_q <= '0;
      dbg_desc_len_sum_q <= '0;
      dbg_raw_pop_cnt_q <= '0;
      dbg_flush_push_cnt_q <= '0;
    end else begin
      if (in_fifo_pop_s) begin
        dbg_in_pop_cnt_q <= dbg_in_pop_cnt_q + 1'b1;
      end
      if (logits_fifo_push_s) begin
        dbg_sample_push_cnt_q <= dbg_sample_push_cnt_q + 1'b1;
      end
      if (raw_desc_fifo_push_s) begin
        dbg_desc_push_cnt_q <= dbg_desc_push_cnt_q + 1'b1;
        dbg_desc_len_sum_q <= dbg_desc_len_sum_q + 16'(collect_count_q);
        if (input_done_i) begin
          dbg_flush_push_cnt_q <= dbg_flush_push_cnt_q + 1'b1;
        end
      end
      if (p_fifo_pop_s) begin
        dbg_raw_pop_cnt_q <= dbg_raw_pop_cnt_q + 1'b1;
      end
    end
  end
`endif

endmodule

`endif
