/*
 * Copyright (c) 2025 Darryl L. Miles
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// Interrupt / Error state unit
module tqvp_dlmiles_i2c_interr (
    input           clk,            // Clock - the TinyQV project clock is normally set to 64MHz.
    input           rst_n,          // Reset_n - low to reset.

    input           reg_stat_i,     // write data
    input           stb_stat_i,	    // write txn

    output [5:0]    reg_stat_o,     // TIMEOUT, IO, GENERIC, INTR_RAW, INTR_EN, INTR_EDGE

    input [2:0]     stb_error_i,    // TIMEOUT, IO, GENERIC

    output          interrupt_raw_o,
    output          interrupt_o     // Dedicated interrupt request for this peripheral
);


    reg [2:0] r_err;
    reg       r_intr_edge;
    reg       r_intr_en;

    wire intr_raw;
    assign intr_raw = |{r_err};

    always @(posedge clk) begin
        if (!rst_n) begin
            r_intr_edge <= 1'b0;
            r_intr_en <= 1'b0;
            r_err <= 3'b0;
        end else if (stb_stat_i) begin
            r_intr_edge <= 1'b0;
            r_intr_en <= reg_stat_i;
            r_err <= 3'b0;
        end else begin
            // synchronous latching
            r_err       <= r_err       | stb_error_i; // 3bits
            r_intr_edge <= r_intr_edge | intr_raw;
        end
    end

    assign reg_stat_o = {
      r_err[2:0],	// TIMEOUT, IO, GENERIC
      intr_raw,

      r_intr_en,
      r_intr_edge
    };

    assign interrupt_raw_o = intr_raw;

    assign interrupt_o = r_intr_en & r_intr_edge;

endmodule
