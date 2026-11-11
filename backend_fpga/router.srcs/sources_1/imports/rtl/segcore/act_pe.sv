`ifndef FPGA_ACT_PE_SV
`define FPGA_ACT_PE_SV

`include "../common/segcore_pkg.sv"

(* DONT_TOUCH = "yes" *)
module act_pe #(
  parameter int DATA_W = `SEGCORE_DATA_W,
  parameter int FRAC_W = `SEGCORE_FRAC_W
) (
  input  logic                         clk,
  input  logic                         rst_n,
  input  logic                         valid_i,
  input  logic [1:0]                   leaf_sel_i,
  input  logic signed [DATA_W-1:0]     x0_raw_i,
  input  logic signed [DATA_W-1:0]     x1_raw_i,
  input  logic signed [DATA_W-1:0]     x2_raw_i,
  input  logic signed [DATA_W-1:0]     x3_raw_i,
  input  logic signed [DATA_W-1:0]     w0_raw_i,
  input  logic signed [DATA_W-1:0]     w1_raw_i,
  input  logic signed [DATA_W-1:0]     w2_raw_i,
  input  logic signed [DATA_W-1:0]     w3_raw_i,
  output logic                         valid_o,
  output logic signed [DATA_W-1:0]     x_act_o,
  // Flattened debug observability (diagnostic only): the aligned last-valid
  // triple of the most recent real evaluation (captured on valid_q1):
  // [31:0]=lut_idx_q1 (zero-ext), [63:32]=deriv_q1, [95:64]=w_q1.
  output logic [3*32-1:0]              debug_act_pe_o
);
  import segcore_pkg::*;

  localparam int LUT_BUCKET_COUNT =
`ifdef ACT_PE_LUT_64
    64
`elsif ACT_PE_LUT_128
    128
`elsif ACT_PE_LUT_256
    256
`elsif ACT_PE_LUT_512
    512
`elsif ACT_PE_LUT_2048
    2048
`else
    1024
`endif
  ;
  localparam int LUT_IDX_W = (LUT_BUCKET_COUNT <= 1) ? 1 : $clog2(LUT_BUCKET_COUNT);
  localparam logic signed [DATA_W-1:0] FXP_EIGHT = 32'sd8 <<< FRAC_W;
  localparam int LUT_BUCKET_STEP = (8 << FRAC_W) / LUT_BUCKET_COUNT;

  logic signed [DATA_W-1:0] x_sel_s0;
  logic signed [DATA_W-1:0] w_sel_s0;
  logic signed [DATA_W-1:0] abs_x_s0;
  logic signed [DATA_W-1:0] deriv_s0;
  logic [LUT_IDX_W-1:0] lut_idx_s0;

  logic                     valid_q1;
  logic signed [DATA_W-1:0] w_q1;
  logic signed [DATA_W-1:0] deriv_q1;
  logic [LUT_IDX_W-1:0]     lut_idx_q1;

  logic                     valid_q2;
  logic signed [DATA_W-1:0] x_act_q2;

  logic [31:0] dbg_last_idx_q;
  logic [31:0] dbg_last_deriv_q;
  logic [31:0] dbg_last_w_q;

  localparam logic signed [DATA_W-1:0] LUT_DERIV [0:LUT_BUCKET_COUNT-1] = '{
`ifdef ACT_PE_LUT_64
`include "act_pe_lut_64x8.svh"
`elsif ACT_PE_LUT_128
`include "act_pe_lut_128x8.svh"
`elsif ACT_PE_LUT_256
`include "act_pe_lut_256x8.svh"
`elsif ACT_PE_LUT_512
`include "act_pe_lut_512x8.svh"
`elsif ACT_PE_LUT_2048
`include "act_pe_lut_2048x8.svh"
`else
`include "act_pe_lut_1024x8.svh"
`endif
  };

  // LUT read in the same shape as softmax_bank_engine.exp_lut_lookup
  // (function + int unsigned index + self-contained module), which Vivado
  // 2018.2 synthesizes correctly; the previous always_comb direct read was
  // dissolved/restructured on hardware. Bit-equivalent to the direct read:
  // saturates to 0 for |x| >= 8.0 (Q16), otherwise LUT_DERIV[abs/step].
  function automatic logic signed [DATA_W-1:0] deriv_lut_lookup(
    input logic signed [DATA_W-1:0] abs_x
  );
    int unsigned lut_idx;
    begin
      if (abs_x >= FXP_EIGHT) begin
        return fxp_zero();
      end
      lut_idx = abs_x / LUT_BUCKET_STEP;
      return LUT_DERIV[lut_idx];
    end
  endfunction

  always_comb begin
    unique case (leaf_sel_i)
      2'd0: begin x_sel_s0 = x0_raw_i; w_sel_s0 = w0_raw_i; end
      2'd1: begin x_sel_s0 = x1_raw_i; w_sel_s0 = w1_raw_i; end
      2'd2: begin x_sel_s0 = x2_raw_i; w_sel_s0 = w2_raw_i; end
      default: begin x_sel_s0 = x3_raw_i; w_sel_s0 = w3_raw_i; end
    endcase
    abs_x_s0 = fxp_abs(x_sel_s0);
    if (abs_x_s0 >= FXP_EIGHT) begin
      lut_idx_s0 = '0;
      deriv_s0 = fxp_zero();
    end else begin
      lut_idx_s0 = abs_x_s0 / LUT_BUCKET_STEP;
      deriv_s0 = deriv_lut_lookup(abs_x_s0);
    end
  end

  // Last-valid capture (diagnostic): only on cycles where the registered
  // stage-1 result belongs to a real evaluation (valid_q1), record the
  // ALIGNED triple {lut_idx_q1, deriv_q1, w_q1} of that evaluation.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dbg_last_idx_q <= '0;
      dbg_last_deriv_q <= '0;
      dbg_last_w_q <= '0;
    end else if (valid_q1) begin
      dbg_last_idx_q <= {{(32-LUT_IDX_W){1'b0}}, lut_idx_q1};
      dbg_last_deriv_q <= 32'(deriv_q1);
      dbg_last_w_q <= 32'(w_q1);
    end
  end

  // Data-path registers intentionally have NO reset: on hardware the async
  // reset on the product register (x_act_q2) could not be absorbed by the
  // DSP48 (sync-reset only, cf. DRC DPOR-2) and Vivado 2018.2 hung the
  // resulting fabric FFs on an undriven "AR" net, pinning x_act_o at 0.
  // Only the valid chain keeps a reset; data registers are don't-care until
  // the first valid evaluation, which always precedes any consumer.
  always_ff @(posedge clk) begin
    w_q1 <= w_sel_s0;
    deriv_q1 <= deriv_s0;
    lut_idx_q1 <= lut_idx_s0;
    x_act_q2 <= fxp_mul(deriv_q1, w_q1);
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      valid_q1 <= 1'b0;
      valid_q2 <= 1'b0;
    end else begin
      valid_q1 <= valid_i;
      valid_q2 <= valid_q1;
    end
  end

  assign valid_o = valid_q2;
  assign x_act_o = x_act_q2;

  assign debug_act_pe_o = {
    dbg_last_w_q,
    dbg_last_deriv_q,
    dbg_last_idx_q
  };

endmodule

`endif
