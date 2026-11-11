`ifndef FPGA_GRAD_WLVIA_SV
`define FPGA_GRAD_WLVIA_SV

`include "../common/segcore_pkg.sv"

module grad_wlvia #(
  parameter int DATA_W = `SEGCORE_DATA_W,
  parameter int FRAC_W = `SEGCORE_FRAC_W,
  parameter int BANK_DEPTH = 65536,
  parameter int FIFO_DEPTH = 2048,
  // Leave headroom for segments already in flight when the warning asserts
  // (adapter emit pipeline + wlvia compute pipeline, a few entries).
  parameter int FIFO_WARNING_THR = (FIFO_DEPTH > 16) ? (FIFO_DEPTH - 16) : 1,
  parameter int SEGMENT_PER_FIFO = 1
) (
  input  logic                                             clk,
  input  logic                                             rst_n,
  input  logic [2:0]                                       mode_i,
  input  logic                                             pop_enable_i,
  input  segcore_pkg::banked_idx_t                         segment_idx_i [0:SEGMENT_PER_FIFO-1],
  input  logic signed [DATA_W-1:0]                         segment_len_i [0:SEGMENT_PER_FIFO-1],
  input  logic [2:0]                                       segment_type_i [0:SEGMENT_PER_FIFO-1],
  input  logic                                             segment_valid_i [0:SEGMENT_PER_FIFO-1],
  output segcore_pkg::banked_idx_t                         upd_idx_o,
  output logic signed [DATA_W-1:0]                         upd_w_o,
  output logic                                             upd_valid_o,
  output logic                                             fifo_empty_o,
  output logic                                             pending_push_o [0:SEGMENT_PER_FIFO-1],
  output logic                                             fifo_warning_o
);
  import segcore_pkg::*;

  localparam int MODE_IDLE                     = 3'b000;
  localparam int MODE_SET_LAMBDA_WL            = 3'b001;
  localparam int MODE_SET_LAMBDA_VIA           = 3'b010;
  localparam int MODE_SET_HOR_EDGE_LEN_NORMAL  = 3'b011;
  localparam int MODE_SET_VER_EDGE_LEN_NORMAL  = 3'b100;
  localparam int MODE_SET_HOR_EDGE_LEN_LAST    = 3'b101;
  localparam int MODE_SET_VER_EDGE_LEN_LAST    = 3'b110;
  localparam int MODE_COMPUTE                  = 3'b111;

  localparam int TYPE_HOR_WL_NORMAL = 3'd0;
  localparam int TYPE_HOR_WL_LAST   = 3'd1;
  localparam int TYPE_VER_WL_NORMAL = 3'd2;
  localparam int TYPE_VER_WL_LAST   = 3'd3;
  localparam int TYPE_HOR_VIA       = 3'd4;
  localparam int TYPE_VER_VIA       = 3'd5;

  localparam int IDX_W = BANKED_IDX_W;

  localparam logic signed [DATA_W-1:0] FXP_ONE = DATA_W'(1 <<< FRAC_W);
  localparam logic signed [DATA_W-1:0] FXP_TWO = DATA_W'(2 <<< FRAC_W);

  logic signed [DATA_W-1:0] hor_lambda_wl_q, ver_lambda_wl_q;
  logic signed [DATA_W-1:0] hor_lambda_via_q, ver_lambda_via_q;
  logic signed [DATA_W-1:0] hor_edge_len_normal_q, ver_edge_len_normal_q;
  logic signed [DATA_W-1:0] hor_edge_len_last_q, ver_edge_len_last_q;

  banked_idx_t pipe_idx_q [0:SEGMENT_PER_FIFO-1];
  logic signed [DATA_W-1:0] pipe_len_q [0:SEGMENT_PER_FIFO-1];
  logic signed [DATA_W-1:0] pipe_coeff_q [0:SEGMENT_PER_FIFO-1];
  logic pipe_valid_q [0:SEGMENT_PER_FIFO-1];

  banked_idx_t pipe_idx_d [0:SEGMENT_PER_FIFO-1];
  logic signed [DATA_W-1:0] pipe_len_d [0:SEGMENT_PER_FIFO-1];
  logic signed [DATA_W-1:0] pipe_coeff_d [0:SEGMENT_PER_FIFO-1];
  logic pipe_valid_d [0:SEGMENT_PER_FIFO-1];

  banked_idx_t lane_upd_idx_s [0:SEGMENT_PER_FIFO-1];
  logic [IDX_W-1:0] lane_upd_idx_bits_s [0:SEGMENT_PER_FIFO-1];
  logic signed [DATA_W-1:0] lane_upd_w_s [0:SEGMENT_PER_FIFO-1];
  logic [IDX_W+DATA_W-1:0] lane_upd_payload_s [0:SEGMENT_PER_FIFO-1];
  logic lane_upd_valid_s [0:SEGMENT_PER_FIFO-1];
  logic [IDX_W+DATA_W-1:0] fifo_upd_payload_s;
  logic fifo_empty_s;

  initial begin
    assert (SEGMENT_PER_FIFO == 1)
      else $error(1, "grad_wlvia: SEGMENT_PER_FIFO must be 1");
    assert (BANK_DEPTH <= (1 << MEM_IDX_W))
      else $error(1, "grad_wlvia: BANK_DEPTH exceeds banked_idx_t.mem_idx width");
  end

  always_comb begin
    assert (
      $isunknown(mode_i) ||
      (mode_i == MODE_IDLE) ||
      (mode_i == MODE_SET_LAMBDA_WL) ||
      (mode_i == MODE_SET_LAMBDA_VIA) ||
      (mode_i == MODE_SET_HOR_EDGE_LEN_NORMAL) ||
      (mode_i == MODE_SET_VER_EDGE_LEN_NORMAL) ||
      (mode_i == MODE_SET_HOR_EDGE_LEN_LAST) ||
      (mode_i == MODE_SET_VER_EDGE_LEN_LAST) ||
      (mode_i == MODE_COMPUTE)
    ) else $fatal(1, "grad_wlvia: invalid mode=0b%b", mode_i);
  end

  always_comb begin
    for (int lane = 0; lane < SEGMENT_PER_FIFO; lane++) begin
      pipe_idx_d[lane] = banked_idx_zero();
      pipe_len_d[lane] = fxp_zero();
      pipe_coeff_d[lane] = fxp_zero();
      pipe_valid_d[lane] = 1'b0;

      lane_upd_idx_s[lane] = pipe_idx_q[lane];
      lane_upd_w_s[lane] = fxp_mul(pipe_len_q[lane], pipe_coeff_q[lane]);
      lane_upd_valid_s[lane] = pipe_valid_q[lane];

      if (mode_i == MODE_COMPUTE) begin
        assert (
          $isunknown(segment_valid_i[lane]) ||
          (segment_valid_i[lane] !== 1'b1) ||
          (!$isunknown(segment_idx_i[lane].mem_idx) && (segment_idx_i[lane].mem_idx < BANK_DEPTH))
        ) else $fatal(1, "grad_wlvia: segment_idx.mem_idx exceeds BANK_DEPTH on lane %0d", lane);

        if (segment_valid_i[lane] === 1'b1) begin
          assert (!$isunknown(segment_len_i[lane]) && (segment_len_i[lane] > fxp_zero()))
            else $fatal(1, "grad_wlvia: segment_len must be positive on lane %0d", lane);

          pipe_idx_d[lane] = segment_idx_i[lane];
          pipe_len_d[lane] = segment_len_i[lane];
          pipe_valid_d[lane] = 1'b1;

          unique case (segment_type_i[lane])
            TYPE_HOR_WL_NORMAL: begin
              pipe_coeff_d[lane] = fxp_mul(hor_edge_len_normal_q, hor_lambda_wl_q);
            end
            TYPE_HOR_WL_LAST: begin
              assert (segment_len_i[lane] == FXP_ONE)
                else $fatal(1, "grad_wlvia: hor_wl_last requires len=1 on lane %0d", lane);
              pipe_coeff_d[lane] = fxp_mul(hor_edge_len_last_q, hor_lambda_wl_q);
            end
            TYPE_VER_WL_NORMAL: begin
              pipe_coeff_d[lane] = fxp_mul(ver_edge_len_normal_q, ver_lambda_wl_q);
            end
            TYPE_VER_WL_LAST: begin
              assert (segment_len_i[lane] == FXP_ONE)
                else $fatal(1, "grad_wlvia: ver_wl_last requires len=1 on lane %0d", lane);
              pipe_coeff_d[lane] = fxp_mul(ver_edge_len_last_q, ver_lambda_wl_q);
            end
            TYPE_HOR_VIA: begin
              assert (segment_len_i[lane] == FXP_TWO)
                else $fatal(1, "grad_wlvia: hor_via requires len=2 on lane %0d", lane);
              pipe_coeff_d[lane] = hor_lambda_via_q;
            end
            TYPE_VER_VIA: begin
              assert (segment_len_i[lane] == FXP_TWO)
                else $fatal(1, "grad_wlvia: ver_via requires len=2 on lane %0d", lane);
              pipe_coeff_d[lane] = ver_lambda_via_q;
            end
            default: begin
              $fatal(1, "grad_wlvia: invalid segment_type=0b%b on lane %0d", segment_type_i[lane], lane);
            end
          endcase
        end
      end
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      hor_lambda_wl_q <= fxp_zero();
      ver_lambda_wl_q <= fxp_zero();
      hor_lambda_via_q <= fxp_zero();
      ver_lambda_via_q <= fxp_zero();
      hor_edge_len_normal_q <= fxp_zero();
      ver_edge_len_normal_q <= fxp_zero();
      hor_edge_len_last_q <= fxp_zero();
      ver_edge_len_last_q <= fxp_zero();
      for (int lane = 0; lane < SEGMENT_PER_FIFO; lane++) begin
        pipe_idx_q[lane] <= banked_idx_zero();
        pipe_len_q[lane] <= fxp_zero();
        pipe_coeff_q[lane] <= fxp_zero();
        pipe_valid_q[lane] <= 1'b0;
      end
    end else begin
      unique case (mode_i)
        MODE_SET_LAMBDA_WL: begin
          hor_lambda_wl_q <= segment_len_i[0];
          ver_lambda_wl_q <= segment_len_i[0];
        end
        MODE_SET_LAMBDA_VIA: begin
          hor_lambda_via_q <= segment_len_i[0];
          ver_lambda_via_q <= segment_len_i[0];
        end
        MODE_SET_HOR_EDGE_LEN_NORMAL: begin
          hor_edge_len_normal_q <= segment_len_i[0];
        end
        MODE_SET_VER_EDGE_LEN_NORMAL: begin
          ver_edge_len_normal_q <= segment_len_i[0];
        end
        MODE_SET_HOR_EDGE_LEN_LAST: begin
          hor_edge_len_last_q <= segment_len_i[0];
        end
        MODE_SET_VER_EDGE_LEN_LAST: begin
          ver_edge_len_last_q <= segment_len_i[0];
        end
        default: begin
        end
      endcase

      for (int lane = 0; lane < SEGMENT_PER_FIFO; lane++) begin
        pipe_idx_q[lane] <= pipe_idx_d[lane];
        pipe_len_q[lane] <= pipe_len_d[lane];
        pipe_coeff_q[lane] <= pipe_coeff_d[lane];
        pipe_valid_q[lane] <= pipe_valid_d[lane];
      end
    end
  end

  generate
    genvar lane_idx;
    for (lane_idx = 0; lane_idx < SEGMENT_PER_FIFO; lane_idx++) begin : GEN_IDX_BITS
      assign lane_upd_idx_bits_s[lane_idx] = lane_upd_idx_s[lane_idx];
      assign lane_upd_payload_s[lane_idx] = {
        lane_upd_idx_bits_s[lane_idx],
        lane_upd_w_s[lane_idx]
      };
      assign pending_push_o[lane_idx] = lane_upd_valid_s[lane_idx];
    end
  endgenerate

  fifo #(
    .DATA_W(IDX_W + DATA_W),
    .FIFO_DEPTH(FIFO_DEPTH),
    .WARNING_THR(FIFO_WARNING_THR),
    .HAND_WRITTEN(1'b1)
  ) u_fifo (
    .clk(clk),
    .rst_n(rst_n),
    .input_data_i(lane_upd_payload_s[0]),
    .input_valid_i(lane_upd_valid_s[0]),
    .pop_i(pop_enable_i),
    .output_data_o(fifo_upd_payload_s),
    .output_valid_o(upd_valid_o),
    .empty_o(fifo_empty_s),
    .full_warning_o(fifo_warning_o),
    .full_o(),
    .single_o(),
    .ready_o()
  );

  assign upd_idx_o = fifo_upd_payload_s[DATA_W +: IDX_W];
  assign upd_w_o = fifo_upd_payload_s[0 +: DATA_W];
  assign fifo_empty_o = fifo_empty_s;

endmodule

`endif
