`ifndef FPGA_SEGCORE_PKG_SV
`define FPGA_SEGCORE_PKG_SV

package segcore_pkg;

`ifndef SEGCORE_ADDR_W
`define SEGCORE_ADDR_W 16
`endif
`ifndef SEGCORE_DATA_W
`define SEGCORE_DATA_W 32
`endif
`ifndef SEGCORE_FRAC_W
`define SEGCORE_FRAC_W 16
`endif
`ifndef SEGCORE_COEFF_W
`define SEGCORE_COEFF_W 16
`endif
`ifndef SEGCORE_COEFF_FRAC_W
`define SEGCORE_COEFF_FRAC_W 12
`endif
`ifndef SEGCORE_BANK_IDX_W
`define SEGCORE_BANK_IDX_W 8
`endif
`ifndef SEGCORE_MEM_IDX_W
`define SEGCORE_MEM_IDX_W 16
`endif

  localparam int ADDR_W = `SEGCORE_ADDR_W;
  localparam int DATA_W = `SEGCORE_DATA_W;
  localparam int FRAC_W = `SEGCORE_FRAC_W;
  localparam int COEFF_W = `SEGCORE_COEFF_W;
  localparam int COEFF_FRAC_W = `SEGCORE_COEFF_FRAC_W;
  localparam int BANK_IDX_W = `SEGCORE_BANK_IDX_W;
  localparam int MEM_IDX_W = `SEGCORE_MEM_IDX_W;
  localparam int BANKED_IDX_W = BANK_IDX_W + MEM_IDX_W;

  typedef struct packed {
    logic [BANK_IDX_W-1:0] bank_idx;
    logic [MEM_IDX_W-1:0]  mem_idx;
  } banked_idx_t;

  typedef enum logic [2:0] {
    SEG_OP_IDLE        = 3'd0,
    SEG_OP_CLEAR       = 3'd1,
    SEG_OP_UPD         = 3'd2,
    SEG_OP_SETW        = 3'd3,
    SEG_OP_FLUSH       = 3'd4,
    SEG_OP_ACT         = 3'd5,
    SEG_OP_QUERY       = 3'd6,
    SEG_OP_EMPTY_QUERY = 3'd7
  } seg_op_e;

  localparam int SEG_OP_W = $bits(seg_op_e);

  function automatic logic signed [DATA_W-1:0] fxp_zero();
    return '0;
  endfunction

  function automatic logic signed [DATA_W-1:0] fxp_abs(
    input logic signed [DATA_W-1:0] value
  );
    if (value[DATA_W-1]) begin
      return -value;
    end
    return value;
  endfunction

  function automatic logic signed [DATA_W-1:0] fxp_mul(
    input logic signed [DATA_W-1:0] a,
    input logic signed [DATA_W-1:0] b
  );
    logic signed [(2*DATA_W)-1:0] prod;
    prod = a * b;
    return prod >>> FRAC_W;
  endfunction

  function automatic banked_idx_t banked_idx_zero();
    banked_idx_t result;
    result = '0;
    return result;
  endfunction

endpackage

`endif
