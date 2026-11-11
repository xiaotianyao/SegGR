`ifndef FPGA_FIFO_SV
`define FPGA_FIFO_SV

`include "segcore_pkg.sv"

module fifo #(
  parameter int DATA_W = `SEGCORE_DATA_W,
  parameter int FIFO_DEPTH = 16,
  parameter int WARNING_THR = 12,
  parameter bit FWFT_MODE = 1'b0,
  parameter bit COMMON_CLOCK_MODE = 1'b0,
  parameter bit HAND_WRITTEN = 1'b0,
  // Handwritten standard-mode (read-latency) ring: pop -> next cycle
  // dout = popped word / valid = 1, exact empty/full flags. The internal
  // pop is gated by occupancy, so underflow reads (the IP fifo phantom
  // source, see the M-series debug history) can never produce a bogus
  // valid entry.
  parameter bit HAND_WRITTEN_STD = 1'b0
) (
  input  logic                               clk,
  input  logic                               rst_n,
  input  logic [DATA_W-1:0]                  input_data_i,
  input  logic                               input_valid_i,
  input  logic                               pop_i,
  output logic [DATA_W-1:0]                  output_data_o,
  output logic                               output_valid_o,
  output logic                               empty_o,
  output logic                               full_warning_o,
  output logic                               full_o,
  output logic                               single_o,
  output logic                               ready_o
);
  localparam int PTR_W = (FIFO_DEPTH <= 1) ? 1 : $clog2(FIFO_DEPTH);
  localparam int CNT_W = $clog2(FIFO_DEPTH + 1);

  localparam int IMPL_FIFO_DEPTH =
    ((FIFO_DEPTH <= 8) ? 16 : FIFO_DEPTH);

  logic [DATA_W-1:0] fifo_dout_s;
  logic fifo_full_s;
  logic fifo_empty_s;
  logic fifo_valid_s;
  logic fifo_prog_full_s;
  logic fifo_wr_rst_busy_s;
  logic fifo_rd_rst_busy_s;
  logic [CNT_W-1:0] count_q, count_d;
  logic push_fire_s;
  logic pop_fire_s;
  logic fifo_wr_en_s;
  logic fifo_rd_en_s;
  logic fifo_ready_raw_s;
  logic [2:0] fifo_ready_pipe_q;
  logic fifo_ready_s;

  initial begin
    assert (FIFO_DEPTH >= 1)
      else $error(1, "fifo: FIFO_DEPTH must be >= 1");
    assert ((WARNING_THR >= 1) && (WARNING_THR <= FIFO_DEPTH))
      else $error(1, "fifo: WARNING_THR must be within [1, FIFO_DEPTH]");
  end

  generate
    if (HAND_WRITTEN_STD) begin : GEN_HAND_WRITTEN_STD
      // Handwritten standard-mode ring (see the parameter note).
      (* ram_style = "block" *) logic [DATA_W-1:0] data_mem [0:FIFO_DEPTH-1];
      logic [PTR_W-1:0] rd_ptr_q, wr_ptr_q;

      function automatic logic [PTR_W-1:0] ptr_inc(
        input logic [PTR_W-1:0] ptr
      );
        if (ptr == (FIFO_DEPTH - 1)) begin
          return '0;
        end
        return ptr + 1'b1;
      endfunction

      assign fifo_ready_raw_s = 1'b1;
      assign fifo_ready_s = 1'b1;
      // Occupancy-gated pop: an underflow read is refused internally, so a
      // lagging upstream flag can never manufacture a phantom entry.
      assign pop_fire_s = (pop_i == 1'b1) && (count_q != '0);
      // No handshake for push. Upstream must closely watch full_warning.
      assign push_fire_s = (input_valid_i == 1'b1);

      assign fifo_full_s = (count_q == FIFO_DEPTH);
      assign fifo_empty_s = (count_q == '0);
      assign fifo_prog_full_s = (count_q >= WARNING_THR);
      assign fifo_wr_rst_busy_s = 1'b0;
      assign fifo_rd_rst_busy_s = 1'b0;

      always_ff @(posedge clk) begin
        if (!rst_n) begin
          rd_ptr_q <= '0;
          wr_ptr_q <= '0;
          fifo_dout_s <= '0;
          fifo_valid_s <= 1'b0;
        end else begin
          if (push_fire_s) begin
            data_mem[wr_ptr_q] <= input_data_i;
            wr_ptr_q <= ptr_inc(wr_ptr_q);
          end
          if (pop_fire_s) begin
            fifo_dout_s <= data_mem[rd_ptr_q];
            rd_ptr_q <= ptr_inc(rd_ptr_q);
            fifo_valid_s <= 1'b1;
          end else begin
            fifo_valid_s <= 1'b0;
          end
        end
      end
    end else if (HAND_WRITTEN) begin : GEN_HAND_WRITTEN
      (* ram_style = "block" *) logic [DATA_W-1:0] data_mem [0:FIFO_DEPTH-1];
      logic [PTR_W-1:0] rd_ptr_q, rd_ptr_d;
      logic [PTR_W-1:0] wr_ptr_q, wr_ptr_d;

      function automatic logic [PTR_W-1:0] ptr_inc(
        input logic [PTR_W-1:0] ptr
      );
        if (ptr == (FIFO_DEPTH - 1)) begin
          return '0;
        end
        return ptr + 1'b1;
      endfunction

      assign fifo_ready_raw_s = 1'b1;
      assign fifo_ready_s = 1'b1;
      assign pop_fire_s = (pop_i == 1'b1) && (count_q != '0);
      assign push_fire_s = (input_valid_i == 1'b1);

      assign fifo_full_s = (count_q == FIFO_DEPTH);
      assign fifo_empty_s = (count_q == '0);
      assign fifo_prog_full_s = (count_q >= WARNING_THR);
      assign fifo_wr_rst_busy_s = 1'b0;
      assign fifo_rd_rst_busy_s = 1'b0;

      always_comb begin
        rd_ptr_d = rd_ptr_q;
        wr_ptr_d = wr_ptr_q;

        if (pop_fire_s) begin
          rd_ptr_d = ptr_inc(rd_ptr_q);
        end

        if (push_fire_s) begin
          wr_ptr_d = ptr_inc(wr_ptr_q);
        end
      end

      always_ff @(posedge clk) begin
        if (!rst_n) begin
          rd_ptr_q <= '0;
          wr_ptr_q <= '0;
          fifo_dout_s <= '0;
          fifo_valid_s <= 1'b0;
        end else begin
          rd_ptr_q <= rd_ptr_d;
          wr_ptr_q <= wr_ptr_d;

          // No handshake for push. Upstream must closely watch full_warning
          if (push_fire_s) begin 
            data_mem[wr_ptr_q] <= input_data_i;
          end

          // Registered FWFT head: after a pop, expose the next logical head
          // on the following cycle instead of replaying the just-popped entry.
          unique case ({pop_fire_s, push_fire_s})
            2'b00: begin
              if (count_q == '0) begin
                fifo_dout_s <= '0;
                fifo_valid_s <= 1'b0;
              end
            end
            2'b01: begin
              if (count_q == '0) begin
                fifo_dout_s <= input_data_i;
              end
              fifo_valid_s <= 1'b1;
            end
            2'b10: begin
              if (count_q > 1) begin
                fifo_dout_s <= data_mem[ptr_inc(rd_ptr_q)];
                fifo_valid_s <= 1'b1;
              end else begin
                fifo_dout_s <= '0;
                fifo_valid_s <= 1'b0;
              end
            end
            2'b11: begin
              if (count_q > 1) begin
                fifo_dout_s <= data_mem[ptr_inc(rd_ptr_q)];
              end else begin
                fifo_dout_s <= input_data_i;
              end
              fifo_valid_s <= 1'b1;
            end
            default: begin
              fifo_dout_s <= '0;
              fifo_valid_s <= 1'b0;
            end
          endcase
        end
      end
    end else if (((DATA_W == 113) || (DATA_W == 111)) && (IMPL_FIFO_DEPTH == 32) && FWFT_MODE && COMMON_CLOCK_MODE) begin : GEN_FIFO_W113_D32_FWFT_CC
      // DATA_W == 113: monolithic BANK_DEPTH=65536 softmax in/out entries
      //   (17b row addr + 32b logits + 64b rms).
      // DATA_W == 111: blocked-GRADP BANK_DEPTH=16384 window geometry
      //   (15b row addr + 32b logits + 64b rms). Both pad into the w128 IP.
      logic [127:0] fifo_din_128_s;
      logic [127:0] fifo_dout_128_s;

      assign fifo_ready_raw_s = (!fifo_wr_rst_busy_s && !fifo_rd_rst_busy_s);
      assign fifo_ready_s = (fifo_ready_pipe_q[1] === 1'b1);
      assign pop_fire_s = (fifo_ready_s && (pop_i == 1'b1) && !fifo_empty_s);
      assign push_fire_s = (fifo_ready_s &&
          (input_valid_i == 1'b1) &&
          (!fifo_full_s || pop_fire_s));
      assign fifo_prog_full_s = (count_q >= WARNING_THR);
      assign fifo_din_128_s = {{(128-DATA_W){1'b0}}, input_data_i};
      assign fifo_dout_s = fifo_dout_128_s[DATA_W-1:0];

      ts_fifo_w128_d32_fwft_cc u_fifo (
        .clk(clk),
        .rst(!rst_n),
        .din(fifo_din_128_s),
        .wr_en(fifo_wr_en_s),
        .rd_en(fifo_rd_en_s),
        .dout(fifo_dout_128_s),
        .full(fifo_full_s),
        .empty(fifo_empty_s),
        .valid(fifo_valid_s),
        .wr_rst_busy(fifo_wr_rst_busy_s),
        .rd_rst_busy(fifo_rd_rst_busy_s)
      );
    end else begin : GEN_UNSUPPORTED
      initial begin
        $error(
          1,
          "fifo: unsupported DATA_W=%0d FIFO_DEPTH=%0d FWFT_MODE=%0d",
          DATA_W,
          FIFO_DEPTH,
          FWFT_MODE
        );
      end

      assign fifo_dout_s = '0;
      assign fifo_full_s = 1'b0;
      assign fifo_empty_s = 1'b1;
      assign fifo_valid_s = 1'b0;
      assign fifo_prog_full_s = 1'b0;
      assign fifo_wr_rst_busy_s = 1'b0;
      assign fifo_rd_rst_busy_s = 1'b0;
    end
  endgenerate

  always_comb begin
    count_d = count_q;

    if (pop_fire_s) begin
      count_d = count_d - 1'b1;
    end

    if (push_fire_s) begin
      count_d = count_d + 1'b1;
    end

  end

  // Sample the overflow check on the clock edge: a combinational assertion
  // races with the upstream always_comb that drives input_valid_i (the fifo
  // can read a stale input_valid_i for a few deltas after the edge that
  // raises fifo_full_s, misfiring on a legal last-slot push).
  always_ff @(posedge clk) begin
    if (rst_n == 1'b1) begin
      assert (!(input_valid_i && fifo_full_s && !pop_fire_s))
        else $fatal(1, "fifo: fifo overflow");
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      count_q <= '0;
      fifo_ready_pipe_q <= '0;
    end else begin
      count_q <= count_d;
      fifo_ready_pipe_q <= {fifo_ready_pipe_q[1:0], fifo_ready_raw_s};
    end
  end

  assign fifo_wr_en_s = push_fire_s;
  assign fifo_rd_en_s = pop_fire_s;

  assign output_data_o = fifo_dout_s;
  assign output_valid_o = fifo_valid_s;
  assign empty_o = fifo_empty_s;
  assign full_warning_o = fifo_prog_full_s;
  assign full_o = fifo_full_s;
  assign single_o = (count_q == 1);
  assign ready_o = fifo_ready_s;

endmodule

`endif
