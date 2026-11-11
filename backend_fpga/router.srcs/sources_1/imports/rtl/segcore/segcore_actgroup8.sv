`ifndef FPGA_SEGCORE_ACTGROUP8_SV
`define FPGA_SEGCORE_ACTGROUP8_SV

module segcore_actgroup8 #(
  parameter int DATA_W = `SEGCORE_DATA_W,
  parameter int FRAC_W = `SEGCORE_FRAC_W,
  parameter int ACT_PIPELINE = 2
) (
  input  logic                             clk,
  input  logic                             rst_n,
  input  logic [segcore_pkg::SEG_OP_W-1:0] cmd_op_i,
  input  logic [2:0]                       cmd_l_i,
  input  logic [2:0]                       cmd_r_i,
  input  logic signed [DATA_W-1:0]         cmd_w_i,
  output logic                             resp_valid_o,
  output logic signed [DATA_W-1:0]         resp_value_o,
  // Flattened act-datapath trace taps (5 x 32-bit, field f at
  // [f*32 +: 32]): 0=act_eval_count, 1=act_last_out,
  // 2=act_last_idx, 3=act_last_deriv, 4=act_last_w (aligned
  // last-valid triple from act_pe). Diagnostic only; rst_n-cleared.
  output logic [5*32-1:0]                  debug_act_trace_o
);
  import segcore_pkg::*;

  logic [SEG_OP_W-1:0] root_l_op, root_r_op;
  logic [1:0] root_l_l, root_l_r, root_r_l, root_r_r;
  logic signed [DATA_W-1:0] root_l_w, root_r_w;

  logic [SEG_OP_W-1:0] root_l_op_q, root_r_op_q;
  logic [1:0] root_l_l_q, root_l_r_q, root_r_l_q, root_r_r_q;
  logic signed [DATA_W-1:0] root_l_w_q, root_r_w_q;

  logic [SEG_OP_W-1:0] mid_l_op [0:1], mid_r_op [0:1];
  logic [0:0] mid_l_l [0:1], mid_l_r [0:1], mid_r_l [0:1], mid_r_r [0:1];
  logic signed [DATA_W-1:0] mid_l_w [0:1], mid_r_w [0:1];

  logic [SEG_OP_W-1:0] mid_l_op_q [0:1], mid_r_op_q [0:1];
  logic [0:0] mid_l_l_q [0:1], mid_l_r_q [0:1], mid_r_l_q [0:1], mid_r_r_q [0:1];
  logic signed [DATA_W-1:0] mid_l_w_q [0:1], mid_r_w_q [0:1];

  logic mid_resp_valid [0:1];
  logic signed [DATA_W-1:0] mid_resp_value [0:1];
  logic leaf_resp_valid [0:3];
  logic signed [DATA_W-1:0] leaf_resp_value [0:3];

  logic signed [DATA_W-1:0] leaf_x_raw [0:3];
  logic signed [DATA_W-1:0] leaf_w_raw [0:3];

  logic act_valid_in_q [0:1];
  logic [1:0] act_leaf_in_q [0:1];
  logic act_valid_out;
  logic signed [DATA_W-1:0] x_act_out;

  logic leaf_act_wr_valid [0:3];
  logic [1:0] act_leaf_out_q [0:1];

  // Trace tap registers and act_pe debug bus (see debug_act_trace_o).
  logic [3*32-1:0] act_pe_debug_s;
  logic [31:0] trace_act_eval_count_q;
  logic [31:0] trace_act_last_out_q;

  integer i_ff;

  always_comb begin
    assert ($isunknown(cmd_l_i) || $isunknown(cmd_r_i) || (cmd_l_i <= cmd_r_i))
      else $fatal("segcore_actgroup8: invalid command range l=0b%b r=0b%b", cmd_l_i, cmd_r_i);
    assert (
      (seg_op_e'(cmd_op_i) != SEG_OP_ACT) ||
      $isunknown(cmd_l_i) ||
      $isunknown(cmd_r_i) ||
      (cmd_l_i == cmd_r_i)
    ) else $fatal("segcore_actgroup8: ACT requires l==r, got l=0b%b r=0b%b", cmd_l_i, cmd_r_i);
  end

  segcore_nonleaf_pe #(
    .ADDR_W(3),
    .DATA_W(DATA_W),
    .FRAC_W(FRAC_W)
  ) u_root (
    .clk(clk),
    .rst_n(rst_n),
    .cmd_op_i(cmd_op_i),
    .cmd_l_i(cmd_l_i),
    .cmd_r_i(cmd_r_i),
    .cmd_w_i(cmd_w_i),
    .resp_l_valid_i(mid_resp_valid[0]),
    .resp_l_value_i(mid_resp_value[0]),
    .resp_r_valid_i(mid_resp_valid[1]),
    .resp_r_value_i(mid_resp_value[1]),
    .cmd_l_op_o(root_l_op),
    .cmd_l_l_o(root_l_l),
    .cmd_l_r_o(root_l_r),
    .cmd_l_w_o(root_l_w),
    .cmd_r_op_o(root_r_op),
    .cmd_r_l_o(root_r_l),
    .cmd_r_r_o(root_r_r),
    .cmd_r_w_o(root_r_w),
    .resp_valid_o(resp_valid_o),
    .resp_value_o(resp_value_o)
  );

  genvar g_mid;
  generate
    for (g_mid = 0; g_mid < 2; g_mid++) begin : GEN_MID
      segcore_nonleaf_pe #(
        .ADDR_W(2),
        .DATA_W(DATA_W),
        .FRAC_W(FRAC_W)
      ) u_mid (
        .clk(clk),
        .rst_n(rst_n),
        .cmd_op_i(g_mid == 0 ? root_l_op_q : root_r_op_q),
        .cmd_l_i(g_mid == 0 ? root_l_l_q : root_r_l_q),
        .cmd_r_i(g_mid == 0 ? root_l_r_q : root_r_r_q),
        .cmd_w_i(g_mid == 0 ? root_l_w_q : root_r_w_q),
        .resp_l_valid_i(leaf_resp_valid[g_mid*2]),
        .resp_l_value_i(leaf_resp_value[g_mid*2]),
        .resp_r_valid_i(leaf_resp_valid[g_mid*2+1]),
        .resp_r_value_i(leaf_resp_value[g_mid*2+1]),
        .cmd_l_op_o(mid_l_op[g_mid]),
        .cmd_l_l_o(mid_l_l[g_mid]),
        .cmd_l_r_o(mid_l_r[g_mid]),
        .cmd_l_w_o(mid_l_w[g_mid]),
        .cmd_r_op_o(mid_r_op[g_mid]),
        .cmd_r_l_o(mid_r_l[g_mid]),
        .cmd_r_r_o(mid_r_r[g_mid]),
        .cmd_r_w_o(mid_r_w[g_mid]),
        .resp_valid_o(mid_resp_valid[g_mid]),
        .resp_value_o(mid_resp_value[g_mid])
      );
    end
  endgenerate

  genvar g_leaf;
  generate
    for (g_leaf = 0; g_leaf < 4; g_leaf++) begin : GEN_LEAF
      localparam int MID_IDX = g_leaf / 2;
      localparam bit IS_RIGHT = (g_leaf % 2) == 1;

      segcore_leaf_pe #(
        .DATA_W(DATA_W),
        .FRAC_W(FRAC_W),
        .ACT_PIPELINE(ACT_PIPELINE)
      ) u_leaf (
        .clk(clk),
        .rst_n(rst_n),
        .cmd_op_i(IS_RIGHT ? mid_r_op_q[MID_IDX] : mid_l_op_q[MID_IDX]),
        .cmd_l_i(IS_RIGHT ? mid_r_l_q[MID_IDX] : mid_l_l_q[MID_IDX]),
        .cmd_r_i(IS_RIGHT ? mid_r_r_q[MID_IDX] : mid_l_r_q[MID_IDX]),
        .cmd_w_i(IS_RIGHT ? mid_r_w_q[MID_IDX] : mid_l_w_q[MID_IDX]),
        .x_act_i(x_act_out),
        .resp_valid_o(leaf_resp_valid[g_leaf]),
        .resp_value_o(leaf_resp_value[g_leaf]),
        .x_raw_o(leaf_x_raw[g_leaf]),
        .w_raw_o(leaf_w_raw[g_leaf])
      );
    end
  endgenerate

  act_pe #(
    .DATA_W(DATA_W),
    .FRAC_W(FRAC_W)
  ) u_act (
    .clk(clk),
    .rst_n(rst_n),
    .valid_i(act_valid_in_q[1]),
    .leaf_sel_i(act_leaf_in_q[1]),
    .x0_raw_i(leaf_x_raw[0]),
    .x1_raw_i(leaf_x_raw[1]),
    .x2_raw_i(leaf_x_raw[2]),
    .x3_raw_i(leaf_x_raw[3]),
    .w0_raw_i(leaf_w_raw[0]),
    .w1_raw_i(leaf_w_raw[1]),
    .w2_raw_i(leaf_w_raw[2]),
    .w3_raw_i(leaf_w_raw[3]),
    .valid_o(act_valid_out),
    .x_act_o(x_act_out),
    .debug_act_pe_o(act_pe_debug_s)
  );

  always_comb begin
    for (int i = 0; i < 4; i++) begin
      leaf_act_wr_valid[i] = 1'b0;
    end
    if (act_valid_out) begin
      leaf_act_wr_valid[act_leaf_out_q[1]] = 1'b1;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      root_l_op_q <= SEG_OP_IDLE;
      root_l_l_q <= '0;
      root_l_r_q <= '0;
      root_l_w_q <= fxp_zero();
      root_r_op_q <= SEG_OP_IDLE;
      root_r_l_q <= '0;
      root_r_r_q <= '0;
      root_r_w_q <= fxp_zero();
      for (i_ff = 0; i_ff < 2; i_ff++) begin
        mid_l_op_q[i_ff] <= SEG_OP_IDLE;
        mid_l_l_q[i_ff] <= '0;
        mid_l_r_q[i_ff] <= '0;
        mid_l_w_q[i_ff] <= fxp_zero();
        mid_r_op_q[i_ff] <= SEG_OP_IDLE;
        mid_r_l_q[i_ff] <= '0;
        mid_r_r_q[i_ff] <= '0;
        mid_r_w_q[i_ff] <= fxp_zero();
      end
      act_valid_in_q[0] <= 1'b0;
      act_valid_in_q[1] <= 1'b0;
      act_leaf_in_q[0] <= '0;
      act_leaf_in_q[1] <= '0;
      act_leaf_out_q[0] <= '0;
      act_leaf_out_q[1] <= '0;
    end else begin
      root_l_op_q <= root_l_op;
      root_l_l_q <= root_l_l;
      root_l_r_q <= root_l_r;
      root_l_w_q <= root_l_w;
      root_r_op_q <= root_r_op;
      root_r_l_q <= root_r_l;
      root_r_r_q <= root_r_r;
      root_r_w_q <= root_r_w;

      for (i_ff = 0; i_ff < 2; i_ff++) begin
        mid_l_op_q[i_ff] <= mid_l_op[i_ff];
        mid_l_l_q[i_ff] <= mid_l_l[i_ff];
        mid_l_r_q[i_ff] <= mid_l_r[i_ff];
        mid_l_w_q[i_ff] <= mid_l_w[i_ff];
        mid_r_op_q[i_ff] <= mid_r_op[i_ff];
        mid_r_l_q[i_ff] <= mid_r_l[i_ff];
        mid_r_r_q[i_ff] <= mid_r_r[i_ff];
        mid_r_w_q[i_ff] <= mid_r_w[i_ff];
      end

      act_valid_in_q[0] <= (seg_op_e'(cmd_op_i) == SEG_OP_ACT);
      act_leaf_in_q[0] <= cmd_l_i[2:1];
      act_valid_in_q[1] <= act_valid_in_q[0];
      act_leaf_in_q[1] <= act_leaf_in_q[0];
      act_leaf_out_q[0] <= act_leaf_in_q[1];
      act_leaf_out_q[1] <= act_leaf_out_q[0];
    end
  end

  // Trace tap capture: registered once per event, rst_n-cleared, 32-bit
  // natural wraparound. Pure observers; the datapath is untouched.
  // act_eval_count / act_last_out are captured on the act_valid_out
  // cycle; the last-valid triple is forwarded from act_pe.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      trace_act_eval_count_q <= '0;
      trace_act_last_out_q <= '0;
    end else begin
      if (act_valid_out) begin
        trace_act_eval_count_q <= trace_act_eval_count_q + 1'b1;
        trace_act_last_out_q <= 32'(x_act_out);
      end
    end
  end

  assign debug_act_trace_o = {
    act_pe_debug_s[2*32 +: 32],
    act_pe_debug_s[1*32 +: 32],
    act_pe_debug_s[0*32 +: 32],
    trace_act_last_out_q,
    trace_act_eval_count_q
  };

endmodule

`endif
