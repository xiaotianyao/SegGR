`ifndef FPGA_SEGCORE_NONLEAF_PE_SV
`define FPGA_SEGCORE_NONLEAF_PE_SV

`include "../common/segcore_pkg.sv"

module segcore_nonleaf_pe #(
  parameter int ADDR_W = 3,
  parameter int DATA_W = `SEGCORE_DATA_W,
  parameter int FRAC_W = `SEGCORE_FRAC_W
) (
  input  logic                             clk,
  input  logic                             rst_n,
  input  logic [segcore_pkg::SEG_OP_W-1:0] cmd_op_i,
  input  logic [ADDR_W-1:0]                cmd_l_i,
  input  logic [ADDR_W-1:0]                cmd_r_i,
  input  logic signed [DATA_W-1:0]         cmd_w_i,
  input  logic                             resp_l_valid_i,
  input  logic signed [DATA_W-1:0]         resp_l_value_i,
  input  logic                             resp_r_valid_i,
  input  logic signed [DATA_W-1:0]         resp_r_value_i,
  output logic [segcore_pkg::SEG_OP_W-1:0] cmd_l_op_o,
  output logic [ADDR_W-2:0]                cmd_l_l_o,
  output logic [ADDR_W-2:0]                cmd_l_r_o,
  output logic signed [DATA_W-1:0]         cmd_l_w_o,
  output logic [segcore_pkg::SEG_OP_W-1:0] cmd_r_op_o,
  output logic [ADDR_W-2:0]                cmd_r_l_o,
  output logic [ADDR_W-2:0]                cmd_r_r_o,
  output logic signed [DATA_W-1:0]         cmd_r_w_o,
  output logic                             resp_valid_o,
  output logic signed [DATA_W-1:0]         resp_value_o
);
  import segcore_pkg::*;

  localparam int CHILD_W = ADDR_W - 1;

  logic signed [DATA_W-1:0] lazy_q, lazy_d;
  logic [SEG_OP_W-1:0] cmd_l_op_d, cmd_r_op_d;
  logic [CHILD_W-1:0] cmd_l_l_d, cmd_l_r_d;
  logic [CHILD_W-1:0] cmd_r_l_d, cmd_r_r_d;
  logic signed [DATA_W-1:0] cmd_l_w_d, cmd_r_w_d;
  logic resp_valid_d;
  logic signed [DATA_W-1:0] resp_value_d;

  function automatic logic is_full_cover(
    input logic [ADDR_W-1:0] l,
    input logic [ADDR_W-1:0] r
  );
    return (~(|l)) & (&r);
  endfunction

  always_comb begin
    assert ($isunknown(cmd_l_i) || $isunknown(cmd_r_i) || (cmd_l_i <= cmd_r_i))
      else $fatal("segcore_nonleaf_pe: invalid command range l=0b%b r=0b%b", cmd_l_i, cmd_r_i);
    assert (
      (seg_op_e'(cmd_op_i) != SEG_OP_ACT) ||
      $isunknown(cmd_l_i) ||
      $isunknown(cmd_r_i) ||
      (cmd_l_i == cmd_r_i)
    ) else $fatal("segcore_nonleaf_pe: ACT requires l==r, got l=0b%b r=0b%b", cmd_l_i, cmd_r_i);
    lazy_d = lazy_q;
    cmd_l_op_d = SEG_OP_IDLE;
    cmd_r_op_d = SEG_OP_IDLE;
    cmd_l_l_d = '0;
    cmd_l_r_d = '0;
    cmd_r_l_d = '0;
    cmd_r_r_d = '0;
    cmd_l_w_d = fxp_zero();
    cmd_r_w_d = fxp_zero();
    resp_valid_d = 1'b0;
    resp_value_d = fxp_zero();

    unique case (seg_op_e'(cmd_op_i))
      SEG_OP_CLEAR: begin
        lazy_d = fxp_zero();
        cmd_l_op_d = SEG_OP_CLEAR;
        cmd_r_op_d = SEG_OP_CLEAR;
      end
      SEG_OP_UPD: begin
        if (is_full_cover(cmd_l_i, cmd_r_i)) begin
          lazy_d = lazy_q + cmd_w_i;
        end else if (!cmd_l_i[ADDR_W-1] && !cmd_r_i[ADDR_W-1]) begin // to left child
          cmd_l_op_d = SEG_OP_UPD;
          cmd_l_l_d = cmd_l_i[CHILD_W-1:0];
          cmd_l_r_d = cmd_r_i[CHILD_W-1:0];
          cmd_l_w_d = cmd_w_i;
        end else if (cmd_l_i[ADDR_W-1] && cmd_r_i[ADDR_W-1]) begin   // to right child
          cmd_r_op_d = SEG_OP_UPD;
          cmd_r_l_d = cmd_l_i[CHILD_W-1:0];
          cmd_r_r_d = cmd_r_i[CHILD_W-1:0];
          cmd_r_w_d = cmd_w_i;
        end else begin
          cmd_l_op_d = SEG_OP_UPD;
          cmd_l_l_d = cmd_l_i[CHILD_W-1:0];
          cmd_l_r_d = {CHILD_W{1'b1}};
          cmd_l_w_d = cmd_w_i;
          cmd_r_op_d = SEG_OP_UPD;
          cmd_r_l_d = '0;
          cmd_r_r_d = cmd_r_i[CHILD_W-1:0];
          cmd_r_w_d = cmd_w_i;
        end
      end
      SEG_OP_FLUSH: begin
        cmd_l_op_d = SEG_OP_FLUSH;
        cmd_r_op_d = SEG_OP_FLUSH;
        cmd_l_w_d = cmd_w_i + lazy_q;
        cmd_r_w_d = cmd_w_i + lazy_q;
        lazy_d = fxp_zero();
      end
      SEG_OP_SETW: begin
        if (!cmd_l_i[ADDR_W-1] && !cmd_r_i[ADDR_W-1]) begin
          cmd_l_op_d = SEG_OP_SETW;
          cmd_l_l_d = cmd_l_i[CHILD_W-1:0];
          cmd_l_r_d = cmd_r_i[CHILD_W-1:0];
          cmd_l_w_d = cmd_w_i;
        end else if (cmd_l_i[ADDR_W-1] && cmd_r_i[ADDR_W-1]) begin
          cmd_r_op_d = SEG_OP_SETW;
          cmd_r_l_d = cmd_l_i[CHILD_W-1:0];
          cmd_r_r_d = cmd_r_i[CHILD_W-1:0];
          cmd_r_w_d = cmd_w_i;
        end else begin
          cmd_l_op_d = SEG_OP_SETW;
          cmd_l_l_d = cmd_l_i[CHILD_W-1:0];
          cmd_l_r_d = {CHILD_W{1'b1}};
          cmd_l_w_d = cmd_w_i;
          cmd_r_op_d = SEG_OP_SETW;
          cmd_r_l_d = '0;
          cmd_r_r_d = cmd_r_i[CHILD_W-1:0];
          cmd_r_w_d = cmd_w_i;
        end
      end
      SEG_OP_ACT: begin
        if (!cmd_l_i[ADDR_W-1]) begin
          cmd_l_op_d = SEG_OP_ACT;
          cmd_l_l_d = cmd_l_i[CHILD_W-1:0];
          cmd_l_r_d = cmd_r_i[CHILD_W-1:0];
        end else begin
          cmd_r_op_d = SEG_OP_ACT;
          cmd_r_l_d = cmd_l_i[CHILD_W-1:0];
          cmd_r_r_d = cmd_r_i[CHILD_W-1:0];
        end
      end
      SEG_OP_QUERY: begin
        if (!cmd_l_i[ADDR_W-1] && !cmd_r_i[ADDR_W-1]) begin
          cmd_l_op_d = SEG_OP_QUERY;
          cmd_l_l_d = cmd_l_i[CHILD_W-1:0];
          cmd_l_r_d = cmd_r_i[CHILD_W-1:0];
          cmd_r_op_d = SEG_OP_EMPTY_QUERY;
        end else if (cmd_l_i[ADDR_W-1] && cmd_r_i[ADDR_W-1]) begin
          cmd_l_op_d = SEG_OP_EMPTY_QUERY;
          cmd_r_op_d = SEG_OP_QUERY;
          cmd_r_l_d = cmd_l_i[CHILD_W-1:0];
          cmd_r_r_d = cmd_r_i[CHILD_W-1:0];
        end else begin
          cmd_l_op_d = SEG_OP_QUERY;
          cmd_l_l_d = cmd_l_i[CHILD_W-1:0];
          cmd_l_r_d = {CHILD_W{1'b1}};
          cmd_r_op_d = SEG_OP_QUERY;
          cmd_r_l_d = '0;
          cmd_r_r_d = cmd_r_i[CHILD_W-1:0];
        end
      end
      SEG_OP_EMPTY_QUERY: begin
        cmd_l_op_d = SEG_OP_EMPTY_QUERY;
        cmd_r_op_d = SEG_OP_EMPTY_QUERY;
      end
      default: begin
      end
    endcase

    if (resp_l_valid_i && resp_r_valid_i) begin
      resp_valid_d = 1'b1;
      resp_value_d = resp_l_value_i + resp_r_value_i;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      lazy_q <= fxp_zero();
      resp_valid_o <= 1'b0;
      resp_value_o <= fxp_zero();
    end else begin
      lazy_q <= lazy_d;
      resp_valid_o <= resp_valid_d;
      resp_value_o <= resp_value_d;
    end
  end

  assign cmd_l_op_o = cmd_l_op_d;
  assign cmd_l_l_o = cmd_l_l_d;
  assign cmd_l_r_o = cmd_l_r_d;
  assign cmd_l_w_o = cmd_l_w_d;
  assign cmd_r_op_o = cmd_r_op_d;
  assign cmd_r_l_o = cmd_r_l_d;
  assign cmd_r_r_o = cmd_r_r_d;
  assign cmd_r_w_o = cmd_r_w_d;

endmodule

`endif
