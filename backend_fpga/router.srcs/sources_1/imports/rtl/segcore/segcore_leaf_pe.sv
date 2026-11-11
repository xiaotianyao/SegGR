`ifndef FPGA_SEGCORE_LEAF_PE_SV
`define FPGA_SEGCORE_LEAF_PE_SV

`include "../common/segcore_pkg.sv"

module segcore_leaf_pe #(
  parameter int DATA_W = `SEGCORE_DATA_W,
  parameter int FRAC_W = `SEGCORE_FRAC_W,
  parameter int ACT_PIPELINE = 2
) (
  input  logic                             clk,
  input  logic                             rst_n,
  input  logic [segcore_pkg::SEG_OP_W-1:0] cmd_op_i,
  input  logic                             cmd_l_i,
  input  logic                             cmd_r_i,
  input  logic signed [DATA_W-1:0]         cmd_w_i,
  input  logic signed [DATA_W-1:0]         x_act_i,
  output logic                             resp_valid_o,
  output logic signed [DATA_W-1:0]         resp_value_o,
  output logic signed [DATA_W-1:0]         x_raw_o,
  output logic signed [DATA_W-1:0]         w_raw_o
);
  import segcore_pkg::*;

  logic signed [DATA_W-1:0] x0_q, x1_q;
  logic signed [DATA_W-1:0] w0_q, w1_q;
  logic act_pipe_valid_q [0:ACT_PIPELINE-1];
  logic act_pipe_slot_q [0:ACT_PIPELINE-1];

  logic signed [DATA_W-1:0] x0_d, x1_d;
  logic signed [DATA_W-1:0] w0_d, w1_d;
  logic act_pipe_valid_d [0:ACT_PIPELINE-1];
  logic act_pipe_slot_d [0:ACT_PIPELINE-1];
  logic resp_valid_d;
  logic signed [DATA_W-1:0] resp_value_d;
  integer i_comb;
  integer i_ff;

  always_comb begin
    assert ($isunknown(cmd_l_i) || $isunknown(cmd_r_i) || (cmd_l_i <= cmd_r_i))
      else $fatal("segcore_leaf_pe: invalid command range l=0b%b r=0b%b", cmd_l_i, cmd_r_i);
    assert (
      (seg_op_e'(cmd_op_i) != SEG_OP_ACT) ||
      $isunknown(cmd_l_i) ||
      $isunknown(cmd_r_i) ||
      (cmd_l_i == cmd_r_i)
    ) else $fatal("segcore_leaf_pe: ACT requires l==r, got l=0b%b r=0b%b", cmd_l_i, cmd_r_i);
    x0_d = x0_q;
    x1_d = x1_q;
    w0_d = w0_q;
    w1_d = w1_q;
    resp_valid_d = 1'b0;
    resp_value_d = fxp_zero();

    for (i_comb = 0; i_comb < ACT_PIPELINE; i_comb++) begin
      act_pipe_valid_d[i_comb] = 1'b0;
      act_pipe_slot_d[i_comb] = 1'b0;
    end

    for (i_comb = 1; i_comb < ACT_PIPELINE; i_comb++) begin
      act_pipe_valid_d[i_comb] = act_pipe_valid_q[i_comb-1];
      act_pipe_slot_d[i_comb] = act_pipe_slot_q[i_comb-1];
    end

    if (act_pipe_valid_q[ACT_PIPELINE-1]) begin
      if (!act_pipe_slot_q[ACT_PIPELINE-1]) begin
        x0_d = x_act_i;
      end else begin
        x1_d = x_act_i;
      end
    end

    unique case (seg_op_e'(cmd_op_i))
      SEG_OP_CLEAR: begin
        x0_d = fxp_zero();
        x1_d = fxp_zero();
        w0_d = fxp_zero();
        w1_d = fxp_zero();
        for (i_comb = 0; i_comb < ACT_PIPELINE; i_comb++) begin
          act_pipe_valid_d[i_comb] = 1'b0;
          act_pipe_slot_d[i_comb] = 1'b0;
        end
      end
      SEG_OP_UPD: begin
        if (!cmd_l_i) begin
          x0_d = x0_d + cmd_w_i;
        end
        if (cmd_r_i) begin
          x1_d = x1_d + cmd_w_i;
        end
      end
      SEG_OP_FLUSH: begin
        x0_d = x0_d + cmd_w_i;
        x1_d = x1_d + cmd_w_i;
      end
      SEG_OP_SETW: begin
        if (!cmd_l_i) begin
          w0_d = cmd_w_i;
        end
        if (cmd_r_i) begin
          w1_d = cmd_w_i;
        end
      end
      SEG_OP_ACT: begin
        act_pipe_valid_d[0] = 1'b1;
        act_pipe_slot_d[0] = cmd_l_i;
      end
      SEG_OP_QUERY: begin
        resp_valid_d = 1'b1;
        if (!cmd_l_i) begin
          resp_value_d = resp_value_d + x0_q;
        end
        if (cmd_r_i) begin
          resp_value_d = resp_value_d + x1_q;
        end
      end
      SEG_OP_EMPTY_QUERY: begin
        resp_valid_d = 1'b1;
        resp_value_d = fxp_zero();
      end
      default: begin
      end
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      x0_q <= fxp_zero();
      x1_q <= fxp_zero();
      w0_q <= fxp_zero();
      w1_q <= fxp_zero();
      for (i_ff = 0; i_ff < ACT_PIPELINE; i_ff++) begin
        act_pipe_valid_q[i_ff] <= 1'b0;
        act_pipe_slot_q[i_ff] <= 1'b0;
      end
      resp_valid_o <= 1'b0;
      resp_value_o <= fxp_zero();
    end else begin
      x0_q <= x0_d;
      x1_q <= x1_d;
      w0_q <= w0_d;
      w1_q <= w1_d;
      for (i_ff = 0; i_ff < ACT_PIPELINE; i_ff++) begin
        act_pipe_valid_q[i_ff] <= act_pipe_valid_d[i_ff];
        act_pipe_slot_q[i_ff] <= act_pipe_slot_d[i_ff];
      end
      resp_valid_o <= resp_valid_d;
      resp_value_o <= resp_value_d;
    end
  end

  assign x_raw_o = cmd_l_i ? x1_q : x0_q;
  assign w_raw_o = cmd_l_i ? w1_q : w0_q;

endmodule

`endif
