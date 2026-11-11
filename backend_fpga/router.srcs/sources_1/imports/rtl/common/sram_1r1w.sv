`ifndef FPGA_SRAM_1R1W_SV
`define FPGA_SRAM_1R1W_SV

`include "segcore_pkg.sv"

module sram_1r1w #(
  parameter int DATA_W = `SEGCORE_DATA_W,
  parameter int BANK_COUNT = 1,
  parameter int BANK_DEPTH = 2048
) (
  input  logic                                clk,
  input  logic                                rd_ce_i [0:BANK_COUNT-1],
  input  logic [$clog2(BANK_DEPTH)-1:0]       rd_addr_i [0:BANK_COUNT-1],
  output logic signed [DATA_W-1:0]            rd_data_o [0:BANK_COUNT-1],
  input  logic                                wr_ce_i [0:BANK_COUNT-1],
  input  logic [$clog2(BANK_DEPTH)-1:0]       wr_addr_i [0:BANK_COUNT-1],
  input  logic signed [DATA_W-1:0]            wr_data_i [0:BANK_COUNT-1]
);
  logic signed [DATA_W-1:0] mem_dout_s [0:BANK_COUNT-1];
  logic signed [DATA_W-1:0] bypass_data_q [0:BANK_COUNT-1];
  logic                     bypass_hold_q [0:BANK_COUNT-1];
  logic                     read_seen_q [0:BANK_COUNT-1];
  localparam bit CONFIG_SUPPORTED =
    ((DATA_W == 16) && ((BANK_DEPTH == 16384) || (BANK_DEPTH == 32768))) ||
    ((DATA_W == 32) && ((BANK_DEPTH == 512) || (BANK_DEPTH == 16384))) ||
    ((DATA_W == 64) && ((BANK_DEPTH == 512) || (BANK_DEPTH == 16384))) ||
    ((DATA_W == 512) && (BANK_DEPTH == 64));

  initial begin
    assert (BANK_COUNT >= 1)
      else $error(1, "sram_1r1w: BANK_COUNT must be >= 1");
    assert (BANK_DEPTH >= 2)
      else $error(1, "sram_1r1w: BANK_DEPTH must be >= 2");
    assert (CONFIG_SUPPORTED)
      else $error(
        1,
        "sram_1r1w: unsupported DATA_W=%0d BANK_DEPTH=%0d",
        DATA_W,
        BANK_DEPTH
      );
    for (int bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin
      bypass_data_q[bank_idx] = '0;
      bypass_hold_q[bank_idx] = 1'b0;
      read_seen_q[bank_idx] = 1'b0;
    end
  end

  generate
    for (genvar bank_idx = 0; bank_idx < BANK_COUNT; bank_idx++) begin : GEN_BRAM_BANK
      if ((DATA_W == 16) && (BANK_DEPTH == 16384)) begin : GEN_W16_D16384
        logic [0:0] wea_s;

        assign wea_s = {wr_ce_i[bank_idx]};

        ts_bram_sdp_w16_d16384 u_mem (
          .clka(clk),
          .ena(wr_ce_i[bank_idx]),
          .wea(wea_s),
          .addra(wr_addr_i[bank_idx]),
          .dina(wr_data_i[bank_idx]),
          .clkb(clk),
          .enb(rd_ce_i[bank_idx]),
          .addrb(rd_addr_i[bank_idx]),
          .doutb(mem_dout_s[bank_idx])
        );
      end else if ((DATA_W == 32) && (BANK_DEPTH == 16384)) begin : GEN_W32_D16384
        logic [0:0] wea_s;

        assign wea_s = {wr_ce_i[bank_idx]};

        ts_bram_sdp_w32_d16384 u_mem (
          .clka(clk),
          .ena(wr_ce_i[bank_idx]),
          .wea(wea_s),
          .addra(wr_addr_i[bank_idx]),
          .dina(wr_data_i[bank_idx]),
          .clkb(clk),
          .enb(rd_ce_i[bank_idx]),
          .addrb(rd_addr_i[bank_idx]),
          .doutb(mem_dout_s[bank_idx])
        );
      end else if ((DATA_W == 16) && (BANK_DEPTH == 32768)) begin : GEN_W16_D32768
        logic [0:0] wea_s;
        logic wr_ce_mux_s;
        logic [$clog2(BANK_DEPTH)-1:0] wr_addr_mux_s;
        logic signed [DATA_W-1:0] wr_data_mux_s;

        assign wr_ce_mux_s = wr_ce_i[bank_idx];
        assign wr_addr_mux_s = wr_addr_i[bank_idx];
        assign wr_data_mux_s = wr_data_i[bank_idx];
        assign wea_s = {wr_ce_mux_s};

        ts_bram_sdp_w16_d32768 u_mem (
          .clka(clk),
          .ena(wr_ce_mux_s),
          .wea(wea_s),
          .addra(wr_addr_mux_s),
          .dina(wr_data_mux_s),
          .clkb(clk),
          .enb(rd_ce_i[bank_idx]),
          .addrb(rd_addr_i[bank_idx]),
          .doutb(mem_dout_s[bank_idx])
        );
      end else if ((DATA_W == 512) && (BANK_DEPTH == 64)) begin : GEN_W512_D64
        logic [0:0] wea_s;
        logic wr_ce_mux_s;
        logic [$clog2(BANK_DEPTH)-1:0] wr_addr_mux_s;
        logic signed [DATA_W-1:0] wr_data_mux_s;

        assign wr_ce_mux_s = wr_ce_i[bank_idx];
        assign wr_addr_mux_s = wr_addr_i[bank_idx];
        assign wr_data_mux_s = wr_data_i[bank_idx];
        assign wea_s = {wr_ce_mux_s};

        router_bridge_line_bram_w512_d64 u_mem (
          .clka(clk),
          .ena(wr_ce_mux_s),
          .wea(wea_s),
          .addra(wr_addr_mux_s),
          .dina(wr_data_mux_s),
          .clkb(clk),
          .enb(rd_ce_i[bank_idx]),
          .addrb(rd_addr_i[bank_idx]),
          .doutb(mem_dout_s[bank_idx])
        );
      end else if ((DATA_W == 32) && (BANK_DEPTH == 512)) begin : GEN_W32_D512
        logic [0:0] wea_s;
        logic wr_ce_mux_s;
        logic [$clog2(BANK_DEPTH)-1:0] wr_addr_mux_s;
        logic signed [DATA_W-1:0] wr_data_mux_s;

        assign wr_ce_mux_s = wr_ce_i[bank_idx];
        assign wr_addr_mux_s = wr_addr_i[bank_idx];
        assign wr_data_mux_s = wr_data_i[bank_idx];
        assign wea_s = {wr_ce_mux_s};

        router_bridge_batch_bram_w32_d512 u_mem (
          .clka(clk),
          .ena(wr_ce_mux_s),
          .wea(wea_s),
          .addra(wr_addr_mux_s),
          .dina(wr_data_mux_s),
          .clkb(clk),
          .enb(rd_ce_i[bank_idx]),
          .addrb(rd_addr_i[bank_idx]),
          .doutb(mem_dout_s[bank_idx])
        );
      end else if ((DATA_W == 64) && (BANK_DEPTH == 512)) begin : GEN_W64_D512
        logic [0:0] wea_s;
        logic wr_ce_mux_s;
        logic [$clog2(BANK_DEPTH)-1:0] wr_addr_mux_s;
        logic signed [DATA_W-1:0] wr_data_mux_s;

        assign wr_ce_mux_s = wr_ce_i[bank_idx];
        assign wr_addr_mux_s = wr_addr_i[bank_idx];
        assign wr_data_mux_s = wr_data_i[bank_idx];
        assign wea_s = {wr_ce_mux_s};

        router_bridge_batch_bram_w64_d512 u_mem (
          .clka(clk),
          .ena(wr_ce_mux_s),
          .wea(wea_s),
          .addra(wr_addr_mux_s),
          .dina(wr_data_mux_s),
          .clkb(clk),
          .enb(rd_ce_i[bank_idx]),
          .addrb(rd_addr_i[bank_idx]),
          .doutb(mem_dout_s[bank_idx])
        );
      end else if ((DATA_W == 64) && (BANK_DEPTH == 16384)) begin : GEN_W64_D16384
        // Compose the 64-bit word from two w32d16384 IPs (lo/hi halves),
        // mirroring the GEN_W64_D65536 approach.
        logic [0:0] wea_s;

        assign wea_s = {wr_ce_i[bank_idx]};

        ts_bram_sdp_w32_d16384 u_mem_lo (
          .clka(clk),
          .ena(wr_ce_i[bank_idx]),
          .wea(wea_s),
          .addra(wr_addr_i[bank_idx]),
          .dina(wr_data_i[bank_idx][31:0]),
          .clkb(clk),
          .enb(rd_ce_i[bank_idx]),
          .addrb(rd_addr_i[bank_idx]),
          .doutb(mem_dout_s[bank_idx][31:0])
        );

        ts_bram_sdp_w32_d16384 u_mem_hi (
          .clka(clk),
          .ena(wr_ce_i[bank_idx]),
          .wea(wea_s),
          .addra(wr_addr_i[bank_idx]),
          .dina(wr_data_i[bank_idx][63:32]),
          .clkb(clk),
          .enb(rd_ce_i[bank_idx]),
          .addrb(rd_addr_i[bank_idx]),
          .doutb(mem_dout_s[bank_idx][63:32])
        );
      end else begin : GEN_UNSUPPORTED
        initial begin
          $error(
            1,
            "sram_1r1w: unsupported DATA_W=%0d BANK_DEPTH=%0d",
            DATA_W,
            BANK_DEPTH
          );
        end
        assign mem_dout_s[bank_idx] = '0;
      end

      always @(posedge clk) begin
        if (rd_ce_i[bank_idx]) begin
          read_seen_q[bank_idx] <= 1'b1;
          if (wr_ce_i[bank_idx] && (wr_addr_i[bank_idx] == rd_addr_i[bank_idx])) begin
            bypass_hold_q[bank_idx] <= 1'b1;
            bypass_data_q[bank_idx] <= wr_data_i[bank_idx];
          end else begin
            bypass_hold_q[bank_idx] <= 1'b0;
          end
        end
      end

      assign rd_data_o[bank_idx] =
        !read_seen_q[bank_idx] ? '0 :
        (bypass_hold_q[bank_idx] ? bypass_data_q[bank_idx] : mem_dout_s[bank_idx]);
    end
  endgenerate

endmodule

`endif
