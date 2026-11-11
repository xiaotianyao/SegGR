`ifndef FPGA_SEGCORE_CORE_SV
`define FPGA_SEGCORE_CORE_SV

module segcore_core #(
  parameter int ADDR_W = 6,
  parameter int DATA_W = `SEGCORE_DATA_W,
  parameter int FRAC_W = `SEGCORE_FRAC_W,
  parameter int ACT_GROUP = 8,
  parameter int ACT_PIPELINE = 2
) (
  input  logic                             clk,
  input  logic                             rst_n,
  input  logic [segcore_pkg::SEG_OP_W-1:0] cmd_op_i,
  input  logic [ADDR_W-1:0]                cmd_l_i,
  input  logic [ADDR_W-1:0]                cmd_r_i,
  input  logic signed [DATA_W-1:0]         cmd_w_i,
  output logic                             resp_valid_o,
  output logic signed [DATA_W-1:0]         resp_value_o,
  // Flattened actgroup8 trace of the LEFTMOST subtree (the one holding
  // chunk leaf 0), forwarded from segcore_actgroup8.debug_act_trace_o.
  // Recursive instances tie this off. Diagnostic only.
  output logic [5*32-1:0]                  debug_act_trace_o
);
  import segcore_pkg::*;

  localparam int SIZE = 1 << ADDR_W;

  logic [SEG_OP_W-1:0] next_l_op, next_r_op;
  logic [ADDR_W-2:0] next_l_l, next_l_r, next_r_l, next_r_r;
  logic signed [DATA_W-1:0] next_l_w, next_r_w;
  logic [SEG_OP_W-1:0] child_l_op_q, child_r_op_q;
  logic [ADDR_W-2:0] child_l_l_q, child_l_r_q, child_r_l_q, child_r_r_q;
  logic signed [DATA_W-1:0] child_l_w_q, child_r_w_q;
  logic child_l_resp_valid, child_r_resp_valid;
  logic signed [DATA_W-1:0] child_l_resp_value, child_r_resp_value;


  always_comb begin
    assert ($isunknown(cmd_l_i) || $isunknown(cmd_r_i) || (cmd_l_i <= cmd_r_i))
      else $fatal("segcore_core: invalid command range l=0b%b r=0b%b", cmd_l_i, cmd_r_i);
    assert (
      (seg_op_e'(cmd_op_i) != SEG_OP_ACT) ||
      $isunknown(cmd_l_i) ||
      $isunknown(cmd_r_i) ||
      (cmd_l_i == cmd_r_i)
    ) else $fatal("segcore_core: ACT requires l==r, got l=0b%b r=0b%b", cmd_l_i, cmd_r_i);
  end

  segcore_highlevel_nonleaf_pe #(
    .ADDR_W(ADDR_W),
    .DATA_W(DATA_W),
    .FRAC_W(FRAC_W)
  ) u_root (
    .clk(clk),
    .rst_n(rst_n),
    .cmd_op_i(cmd_op_i),
    .cmd_l_i(cmd_l_i),
    .cmd_r_i(cmd_r_i),
    .cmd_w_i(cmd_w_i),
    .resp_l_valid_i(child_l_resp_valid),
    .resp_l_value_i(child_l_resp_value),
    .resp_r_valid_i(child_r_resp_valid),
    .resp_r_value_i(child_r_resp_value),
    .cmd_l_op_o(next_l_op),
    .cmd_l_l_o(next_l_l),
    .cmd_l_r_o(next_l_r),
    .cmd_l_w_o(next_l_w),
    .cmd_r_op_o(next_r_op),
    .cmd_r_l_o(next_r_l),
    .cmd_r_r_o(next_r_r),
    .cmd_r_w_o(next_r_w),
    .resp_valid_o(resp_valid_o),
    .resp_value_o(resp_value_o)
  );

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      child_l_op_q <= SEG_OP_IDLE;
      child_r_op_q <= SEG_OP_IDLE;
      child_l_l_q <= '0;
      child_l_r_q <= '0;
      child_r_l_q <= '0;
      child_r_r_q <= '0;
      child_l_w_q <= fxp_zero();
      child_r_w_q <= fxp_zero();
    end else begin
      child_l_op_q <= next_l_op;
      child_r_op_q <= next_r_op;
      child_l_l_q <= next_l_l;
      child_l_r_q <= next_l_r;
      child_r_l_q <= next_r_l;
      child_r_r_q <= next_r_r;
      child_l_w_q <= next_l_w;
      child_r_w_q <= next_r_w;
    end
  end

  generate
    if (SIZE == (ACT_GROUP * 2)) begin : GEN_BOTTOM
      segcore_actgroup8 #(
        .DATA_W(DATA_W),
        .FRAC_W(FRAC_W),
        .ACT_PIPELINE(ACT_PIPELINE)
      ) u_left (
        .clk(clk),
        .rst_n(rst_n),
        .cmd_op_i(child_l_op_q),
        .cmd_l_i(child_l_l_q),
        .cmd_r_i(child_l_r_q),
        .cmd_w_i(child_l_w_q),
        .resp_valid_o(child_l_resp_valid),
        .resp_value_o(child_l_resp_value),
        .debug_act_trace_o(debug_act_trace_o)
      );

      segcore_actgroup8 #(
        .DATA_W(DATA_W),
        .FRAC_W(FRAC_W),
        .ACT_PIPELINE(ACT_PIPELINE)
      ) u_right (
        .clk(clk),
        .rst_n(rst_n),
        .cmd_op_i(child_r_op_q),
        .cmd_l_i(child_r_l_q),
        .cmd_r_i(child_r_r_q),
        .cmd_w_i(child_r_w_q),
        .resp_valid_o(child_r_resp_valid),
        .resp_value_o(child_r_resp_value),
        .debug_act_trace_o()
      );
    end else begin : GEN_RECURSE
      segcore_core #(
        .ADDR_W(ADDR_W-1),
        .DATA_W(DATA_W),
        .FRAC_W(FRAC_W),
        .ACT_GROUP(ACT_GROUP),
        .ACT_PIPELINE(ACT_PIPELINE)
      ) u_left (
        .clk(clk),
        .rst_n(rst_n),
        .cmd_op_i(child_l_op_q),
        .cmd_l_i(child_l_l_q),
        .cmd_r_i(child_l_r_q),
        .cmd_w_i(child_l_w_q),
        .resp_valid_o(child_l_resp_valid),
        .resp_value_o(child_l_resp_value),
        .debug_act_trace_o(debug_act_trace_o)
      );

      segcore_core #(
        .ADDR_W(ADDR_W-1),
        .DATA_W(DATA_W),
        .FRAC_W(FRAC_W),
        .ACT_GROUP(ACT_GROUP),
        .ACT_PIPELINE(ACT_PIPELINE)
      ) u_right (
        .clk(clk),
        .rst_n(rst_n),
        .cmd_op_i(child_r_op_q),
        .cmd_l_i(child_r_l_q),
        .cmd_r_i(child_r_r_q),
        .cmd_w_i(child_r_w_q),
        .resp_valid_o(child_r_resp_valid),
        .resp_value_o(child_r_resp_value),
        .debug_act_trace_o()
      );
    end
  endgenerate

endmodule

`endif
