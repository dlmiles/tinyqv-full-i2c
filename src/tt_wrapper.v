/*
 * Copyright (c) 2025 Michael Bell
 * Copyright (c) 2025 Darryl L. Miles
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

`define UI_IN_SYNCHRONIZER 1
// Unclear what benefit 2 stage (the template default) brings,
//   the signal are externally driven as sync to (sent with) clock.
// Introduces input latency cycle, reducing maximum SPI clock rate
// TT space is a premium, FlipFlops are relatively large.
// Understand it makes sense to retime all input signal to clock
//    arrival at module boundary, as STA will correctly handle
//    correct timing across the project from that point.
`define UI_IN_SYNCHRONIZER_STAGES 1
// This is the standalone SPI slave testing interface
`define SPI_SLAVE_IFACE 1
// This is a more direct signal testing interface (in my case there are
//    enough unused signals and space to a more direct access in standalone).
`define DIRECT_TEST_IFACE 1

/** TinyQV peripheral harness test using SPI */
module tt_um_dlmiles_tqvph_i2c (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // SPI access to registers
  wire  [5:0] address;
  wire [31:0] data_in;  // Data in to peripheral
  wire [31:0] data_out; // Data out from peripheral
  wire  [1:0] data_write_n;
  wire  [1:0] data_read_n;
  wire        data_ready;
  wire        user_interrupt;

  wire  [7:0] ui_in_signal;
`ifdef UI_IN_SYNCHRONIZER
  // Peripherals get synchronized ui_in.
  reg [7:0] ui_in_sync;
  synchronizer #(.STAGES(`UI_IN_SYNCHRONIZER_STAGES), .WIDTH(8)) synchronizer_ui_in_inst (.clk(clk), .data_in(ui_in), .data_out(ui_in_sync));
  assign ui_in_signal[7:0] = ui_in_sync[7:0];
`else
  assign ui_in_signal[7:0] = ui_in[7:0];
`endif

  // Register reset as in TinyQV
  /* verilator lint_off SYNCASYNCNET */
  reg rst_reg_n;
  /* verilator lint_on SYNCASYNCNET */
  always @(negedge clk) rst_reg_n <= rst_n;

`ifdef DIRECT_TEST_IFACE
  localparam MODE_IDLE  = 2'b00;
  localparam MODE_READ  = 2'b01;
  localparam MODE_WRITE = 2'b10;

  localparam UI_IN0_ADDR = 0;  // 2bit ADDRESS (LSB)
  localparam UI_IN2_MODE = 2;  // 2bit MODE (LSB)
  localparam UO_OUT0_INTR = 0; // INTERRUPT (also UART TXD for TinyQV)

  `ifdef SPI_SLAVE_IFACE
    // Only if both interfaces are built into hardware do we need this extra logic
    reg regtestmux; // 0=SPI_SLAVE_IFACE 1=DIRECT_TEST_IFACE
    always @(posedge clk) begin
      if (!rst_reg_n) begin
        // Synchronous sample, while under reset, so:
        // rst_n=0; ui_in[UI_IN2_MODE]=?; clk=~clk; clk=~clk; clk=~clk; clk=~clk; rst_n=1;
        regtestmux <= ui_in_signal[UI_IN2_MODE]; // use LSB of this setting
      end
    end
  `endif
`endif

  wire [7:0] uo_out_dut;
  // The peripheral under test.
  // **** Change the module name from tqvp_example to match your peripheral. ****
  tqvp_dlmiles_i2c_top user_peripheral_dlmiles_i2c(
    .clk(clk),
    .rst_n(rst_reg_n),
    .ui_in(ui_in_signal),
    .uo_out(uo_out_dut),
    .address(address),
    .data_in(data_in),
    .data_write_n(data_write_n),
    .data_read_n(data_read_n),
    .data_out(data_out),
    .data_ready(data_ready),
    .user_interrupt(user_interrupt)
  );

`ifdef SPI_SLAVE_IFACE
  // SPI data indications
  wire        addr_valid;
  wire        data_valid;
  wire        data_rw;
  wire  [1:0] txn_n;
  reg  [31:0] data_out_masked;

  // SPI interface
  wire spi_cs_n;
  wire spi_clk;
  wire spi_miso;
  wire spi_mosi;

  // Synchronized SPI inputs
  wire spi_cs_n_sync;
  wire spi_clk_sync;
  wire spi_mosi_sync;

  assign spi_cs_n  = uio_in[4];
  assign spi_clk   = uio_in[5];
  assign spi_mosi  = uio_in[6];

  synchronizer #(.STAGES(2), .WIDTH(1)) synchronizer_spi_cs_n_inst (.clk(clk), .data_in(spi_cs_n), .data_out(spi_cs_n_sync));
  synchronizer #(.STAGES(2), .WIDTH(1)) synchronizer_spi_clk_inst  (.clk(clk), .data_in(spi_clk),  .data_out(spi_clk_sync));
  synchronizer #(.STAGES(2), .WIDTH(1)) synchronizer_spi_mosi_inst (.clk(clk), .data_in(spi_mosi), .data_out(spi_mosi_sync));  

  wire  [5:0] address_spislv;
  wire [31:0] data_in_spislv;

  // The SPI instance
  spi_reg #(.ADDR_W(6), .REG_W(32)) i_spi_reg(
    .clk(clk),
    .rstb(rst_reg_n),
    .ena(1'b1),  // template SPI logic makes use of ena==1'b1 so this allow synth to remove
    .spi_mosi(spi_mosi_sync),
    .spi_miso(spi_miso),
    .spi_clk(spi_clk_sync),
    .spi_cs_n(spi_cs_n_sync),
    .reg_addr(address_spislv),
    .reg_data_i(data_out_masked),
    .reg_data_o(data_in_spislv),
    .reg_addr_v(addr_valid),
    .reg_data_i_dv(data_ready),
    .reg_data_o_dv(data_valid),
    .reg_rw(data_rw),
    .txn_width(txn_n)
  );

  reg [1:0] data_write_n_spislv;
  //assign data_write_n_spislv = (data_valid &&  data_rw) ? txn_n : 2'b11;
  reg [1:0] data_read_n_spislv;
  //assign data_read_n_spislv  = (addr_valid && !data_rw) ? txn_n : 2'b11;

  always @(*) begin
      data_write_n_spislv = 2'b11;
      data_read_n_spislv = 2'b11;

      if (data_valid && data_rw) begin
        data_write_n_spislv = txn_n;
      end
      if (addr_valid && !data_rw) begin
        data_read_n_spislv = txn_n;
      end

      data_out_masked = data_out;
      if (txn_n[1] == 1'b0) data_out_masked[31:16] = 0;
      if (txn_n == 2'b00) data_out_masked[15:8] = 0;
  end

  // Assign outputs
  wire [7:0] uio_oe_spislv;
  wire [7:0] uio_out_spislv;
  wire [7:0] uo_out_spislv;
  assign uo_out_spislv[7:0] = uo_out_dut[7:0];

  assign uio_out_spislv[3] = spi_miso;
  assign uio_oe_spislv[3] = 1;
  assign uio_out_spislv[0] = user_interrupt;
  assign uio_oe_spislv[0] = 1;
  assign uio_out_spislv[1] = data_ready;
  assign uio_oe_spislv[1] = 1;

  // tie unused
  assign uio_out_spislv[7:4] = 0;
  assign uio_out_spislv[2] = 0;
  assign uio_oe_spislv[7:4] = 0;
  assign uio_oe_spislv[2] = 0;
`endif

`ifdef DIRECT_TEST_IFACE
  wire  [7:0] uio_oe_test;
  wire  [7:0] uio_out_test;
  wire  [7:0] uo_out_test;
  wire  [1:0] data_write_n_test;
  wire  [1:0] data_read_n_test;
  wire  [5:0] address_test;
  wire [31:0] data_in_test;
  assign address_test[5:0]        = {2'b0,ui_in_signal[UI_IN0_ADDR +: 2],2'b0};
  assign data_in_test[31:0]       = {24'b0,uio_in[7:0]};
  assign data_write_n_test        = !(ui_in_signal[UI_IN2_MODE +: 2] == MODE_WRITE);
  assign data_read_n_test         = !(ui_in_signal[UI_IN2_MODE +: 2] == MODE_READ);
  assign uio_oe_test[7:0]         = (ui_in_signal[UI_IN2_MODE +: 2] == MODE_READ) ? 8'hff : 8'h00;
  assign uio_out_test[7:0]        = data_out[7:0]; // ignores top 24bit MSB
  // vvv MUST BE CHECKED AND FIXED UP MANUALLY
  assign uo_out_test[7:1]         = uo_out_dut[7:1];
  assign uo_out_test[UO_OUT0_INTR] = user_interrupt;  // SPI uses uio_out[0] we use uo_out[0] which is also UART TXD
`endif

// Wire the outputs based on the ifdefs
`ifdef DIRECT_TEST_IFACE
  `ifdef SPI_SLAVE_IFACE
    // Use MUXed
    assign data_write_n  = (regtestmux) ? data_write_n_test  : data_write_n_spislv;
    assign data_read_n   = (regtestmux) ? data_read_n_test   : data_read_n_spislv;
    assign address[5:0]  = (regtestmux) ? address_test[5:0]  : address_spislv[5:0];
    assign data_in[31:0] = (regtestmux) ? data_in_test[31:0] : data_in_spislv[31:0];
    assign uio_oe[7:0]   = (regtestmux) ? uio_oe_test[7:0]   : uio_oe_spislv[7:0];
    assign uio_out[7:0]  = (regtestmux) ? uio_out_test[7:0]  : uio_out_spislv[7:0];
    assign uo_out[7:0]   = (regtestmux) ? uo_out_test[7:0]   : uo_out_spislv[7:0];
  `else
    // Just DIRECT_TEST_IFACE
    assign data_write_n  = data_write_n_test;
    assign data_read_n   = data_read_n_test;
    assign address[5:0]  = address_test[5:0];
    assign data_in[31:0] = data_in_test[31:0];
    assign uio_oe[7:0]   = uio_oe_test[7:0];
    assign uio_out[7:0]  = uio_out_test[7:0];
    assign uo_out[7:0]   = uo_out_test[7:0];
  `endif
`else
  // Just SPI_SLAVE_IFACE
  assign data_write_n  = data_write_n_spislv;
  assign data_read_n   = data_read_n_spislv;
  assign address[5:0]  = address_spislv[5:0];
  assign data_in[31:0] = data_in_spislv[31:0];
  assign uio_oe[7:0]   = uio_oe_spislv[7:0];
  assign uio_out[7:0]  = uio_out_spislv[7:0];
  assign uo_out[7:0]   = uo_out_spislv[7:0];
`endif

  // Ignore unused inputs
  wire _unused = &{uio_in[7:0], ui_in[7:0], ena, 1'b0};

endmodule
