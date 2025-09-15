/*
 * Copyright (c) 2025 Darryl L. Miles
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

`include "global.vh"

module tqvp_dlmiles_i2c_fifo (
    input           clk,          // Clock - the TinyQV project clock is normally set to 64MHz.
    input           rst_tx_n_i,   // Reset active LOW - TX side active HIGH (revoke valid, set which_r=0, which_w=0)
    input           rst_rx_n_i,   // Reset active LOW - RX side active HIGH (revoke valid, set which_r=0, which_w=0)

    output    [8:0] reg_data_recv_o,  // MSB is inverted reg_data_recv_valid_o
    output          reg_data_recv_valid_o,
    input           stb_data_recv_ready_i,

    input    [11:0] reg_data_send_i,  
    input           reg_data_send_valid_i,

    output    [8:0] i2c_txd_data_o,
    output          i2c_txd_valid_o,
    input           i2c_txd_ready_i,

    input     [7:0] i2c_rxd_data_i,
    input           i2c_rxd_valid_i,

    output          st_tx_full_o,
    output          st_tx_empty_o,
    output          st_tx_overrun_o,
    output          st_rx_full_o,
    output          st_rx_empty_o,
    output          st_rx_overrun_o
);

    localparam RX_FIFO_DEPTH = 8; // pow2
    localparam TX_FIFO_DEPTH = 8; // pow2
    localparam RX_FIFO_COUNT_WIDTH = 3;
    localparam TX_FIFO_COUNT_WIDTH = 3;

    localparam DIR_TXD = `DIR_TXD;
    localparam DIR_RXD = `DIR_RXD;

`ifndef SYNTHESIS_OPENLANE
    initial assert(RX_FIFO_DEPTH == 2 ** RX_FIFO_COUNT_WIDTH);
    initial assert(TX_FIFO_DEPTH == 2 ** TX_FIFO_COUNT_WIDTH);
`endif

    // TODO merge these into one for the I2C half-duplex situation so the area can
    //  be used for a single large FIFO
    reg r_rxe_overrun;
    reg       [RX_FIFO_DEPTH-1:0] r_rxd_valid;
    reg                     [7:0] r_rxd_data [RX_FIFO_DEPTH-1:0];
    reg [RX_FIFO_COUNT_WIDTH-1:0] r_rxd_which_w;
    reg [RX_FIFO_COUNT_WIDTH-1:0] r_rxd_which_r;

    reg r_txe_overrun;
    reg       [TX_FIFO_DEPTH-1:0] r_txd_valid;
    reg                     [7:0] r_txd_data [TX_FIFO_DEPTH-1:0];
    reg                     [0:0] r_txd_dir  [TX_FIFO_DEPTH-1:0];
    reg [TX_FIFO_COUNT_WIDTH-1:0] r_txd_which_w;        // which to write
    reg [TX_FIFO_COUNT_WIDTH-1:0] r_txd_which_r;        // which to read

    always @(posedge clk) begin
        if (reg_data_send_valid_i) begin        // CPU to device
            r_txd_data [r_txd_which_w][7:0] <= reg_data_send_i[7:0];
            r_txd_dir  [r_txd_which_w][0]   <= reg_data_send_i[8];
            r_txd_valid[r_txd_which_w] <= 1'b1;
            r_txd_which_w <= r_txd_which_w + 1'h1;
            if (r_txd_valid[r_txd_which_w])
                r_txe_overrun <= 1'b1;
        end
        if (stb_data_recv_ready_i) begin        // device to CPU
            // data always has current (muxed) view
            r_rxd_which_r <= r_rxd_which_r + 1'd1;
            r_rxd_valid[r_rxd_which_r] <= 1'b0;
        end
        if (i2c_rxd_valid_i) begin      // I2C to device
            r_rxd_data [r_rxd_which_w][7:0] <= i2c_rxd_data_i[7:0];
            r_rxd_valid[r_rxd_which_w] <= 1'b1;
            r_rxd_which_w <= r_rxd_which_w + 1'd1;
            if (r_rxd_valid[r_rxd_which_w])
                r_rxe_overrun <= 1'b1;
            
        end
        if (i2c_txd_ready_i) begin      // device to I2C
            // data always has current (muxed) view
            r_txd_which_r <= r_txd_which_r + 1'd1;
            r_txd_valid[r_txd_which_r] <= 1'b0;
        end
        // These are at the bottom so they have the highest precedence
        if (!rst_rx_n_i) begin
            r_rxd_valid   <= {RX_FIFO_DEPTH{1'b0}};
            r_rxe_overrun <= 1'b0;
            r_rxd_which_r <= {RX_FIFO_COUNT_WIDTH{1'd0}};
            r_rxd_which_w <= {RX_FIFO_COUNT_WIDTH{1'd0}}; //RX_FIFO_DEPTH - 1'd1; // one before write position
        end
        if (!rst_tx_n_i) begin
            r_txd_valid   <= {TX_FIFO_DEPTH{1'b0}};
            r_txe_overrun <= 1'b0;
            r_txd_which_r <= {TX_FIFO_COUNT_WIDTH{1'd0}};
            r_txd_which_w <= {TX_FIFO_COUNT_WIDTH{1'd0}}; //TX_FIFO_DEPTH - 1'b1; // one before write position
        end
    end

    // device to CPU (always has view)
    assign reg_data_recv_o       = {~r_rxd_valid[r_rxd_which_r], r_rxd_data[r_rxd_which_r][7:0]}; // inverted for CPU register interface
    assign reg_data_recv_valid_o = r_rxd_valid[r_rxd_which_r];  // non-inverted

    // device to I2C (always has view)
    assign i2c_txd_data_o   = {r_txd_dir[r_txd_which_r][0], r_txd_data[r_txd_which_r][7:0]};
    assign i2c_txd_valid_o  = r_txd_valid[r_txd_which_r];

    assign st_tx_full_o     =  &{r_txd_valid}; // AND
    assign st_tx_empty_o    = ~|{r_txd_valid}; // NOR
    assign st_tx_overrun_o  = r_txe_overrun;

    assign st_rx_full_o     =  &{r_rxd_valid}; // AND
    assign st_rx_empty_o    = ~|{r_rxd_valid}; // NOR
    assign st_rx_overrun_o  = r_rxe_overrun;

endmodule
