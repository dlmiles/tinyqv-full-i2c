/*
 * Copyright (c) 2025 Darryl L. Miles
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tqvp_dlmiles_i2c_timer (
    input           clk,          // Clock
    input           rst_n,        // Reset_n - low to reset

    input           timer_run_i,

    input    [11:0] reg_conf_i,

    output          stb_tick_first_o,		// first after reset
    output          stb_tick_edgewait_o,	// LIMIT0
    output          stb_tick_prewait_o,		// LIMIT1
    output          stb_tick_sclhigh_o,		// LIMIT2
    output          stb_tick_scllow_o,		// LIMIT3
    output          stb_tick_idlescl_o,		// LIMIT4
    output          stb_tick_overflow_o,	// MSB overflow

    input           scl_idle_monitor_reset_i,
    input           scl_idle_monitor_arm_i,
    output          scl_idle_monitor_strobe_o,
    output          scl_idle_monitor_notidle_o,

    input           scl_i,
    input           sda_i
);

    localparam WIDTH = 12;

    reg [WIDTH-1:0] timer_count;
    reg       [1:0] timer_count_idle_scl; // 

    // At 64MHz master clock:
    //   FAST+    1MHz   = 4
    //   FAST     400Khz = 10 to provide 1/16 ticks
    //   STANDARD 100KHz = 40 to provide 1/16 ticks
    //   SLOW      10KHz = 400 to provide 1/16 ticks

    wire [1:0] reg_conf_clkdiv;
    //assign reg_conf_clkdiv = reg_conf_i[9:8];
    // Originally had 2 bits to hold fixed value, modified to allow future update to allow programmable value
    //assign reg_conf_clkdiv = {
    //    reg_conf_i[5] | reg_conf_i[4], // see table in comments below (actual count)
    //    reg_conf_i[4] | reg_conf_i[1]  // see table in comments below (actual count)
    //};
    assign reg_conf_clkdiv = {
        reg_conf_i[7] | reg_conf_i[5], // see table in comments below (minus one)
        reg_conf_i[3]                  // see table in comments below (minus one)
    };

    localparam CLKDIV_FASTPLUS = 9'd4 - 1;	// subtract 1 as zero based count
    localparam CLKDIV_FAST     = 9'd10 - 1;
    localparam CLKDIV_SLOW     = 9'd400 - 1;
    localparam CLKDIV_STANDARD = 9'd40 - 1;

    wire [8:0] clkdiv_limit;
    // 
    assign clkdiv_limit = (reg_conf_clkdiv[1]) ?	// .............. (actual) 8 76543210    8 76543210 (minus one)
        ((reg_conf_clkdiv[0]) ? (CLKDIV_SLOW) : 	// 2'b11 SLOW     400 = 9'b1_10010000 9'b1_10001111
                                (CLKDIV_STANDARD)) :	// 2'b10 STANDARD  40 = 9'b0_00101000 9'b0_00100111
        ((reg_conf_clkdiv[0]) ? (CLKDIV_FAST) :		// 2'b01 FAST      10 = 9'b0_00001010 9'b0_00001001
                                (CLKDIV_FASTPLUS));	// 2'b00 FASTPLUS   4 = 9'b0_00000100 9'b0_00000011 // expected reset default

    reg  [8:0] clkdiv_count;
`ifdef COCOTB_SIM
    // FIXME why does this not work, I'm sure it used to work
    initial clkdiv_count = 9'b0;	// SIM only reset to known value
`endif

`ifndef SYNTHESIS_OPENLANE
    //assert({9{1'b1}} > CLKDIV_SLOW);	// check counter can store largest value
`endif

    wire clkdiv_stb;
    assign clkdiv_stb = clkdiv_count == 9'b0;

    always @(posedge clk) begin
        if (clkdiv_stb)
            clkdiv_count <= clkdiv_limit;
        else
            clkdiv_count <= clkdiv_count - 1;
`ifdef COCOTB_SIM
        if (!rst_n)
            clkdiv_count <= 9'b0;
`endif
    end

    // START_PRE/START_POST STOP_PRE/STOP_POST
    localparam TIMER_LIMIT0 = 2;   // 2/16 of OE disconnect            1.250uS @100KHz
    localparam TIMER_LIMIT1 = 6;   // 6/16 of SCL rate (HD:STA SU:STO) 3.750uS @100KHz
    localparam TIMER_LIMIT2 = 7;   // 7/16 of SCL rate (SCL HIGH time) 4.375uS @100KHz
    localparam TIMER_LIMIT3 = 9; //   9/16 of SCL rate (SCL LOW time)  5.625uS @100KHz
    localparam TIMER_LIMIT4 = 511; // 511/16 (almost 32bits of data) 319.375uS @100KHz

    reg r_tick_overflow;
    reg r_tick_sclhigh;
    reg r_tick_first;

    reg r_scl_idle_monitor_notidle;

    wire stb_tick_sclhigh_condx;
    assign stb_tick_sclhigh_condx  = timer_count == TIMER_LIMIT2;

    // timer with strobe outputs
    always @(posedge clk) begin
        if (!rst_n) begin
            timer_count          <= '0;
            timer_count_idle_scl <= '0;
            r_tick_overflow <= 1'b0;
            r_tick_sclhigh  <= 1'b0;
            r_tick_first    <= 1'b1;
        end else begin
            // Main count (clkdiv_stb is 1/16 of SCL period)
            if (timer_run_i & clkdiv_stb) begin
                timer_count <= timer_count + 1;
            end

            // This is always running
            if (!scl_i) begin  // something is using SCL (the pull-up has been disrupted)
                timer_count_idle_scl <= '0;
            end else if (!timer_count_idle_scl[0] && timer_count == '0) begin // wait for wrap
                timer_count_idle_scl[0] <= 1'b1; // bit0 set
            end else if ( timer_count_idle_scl[0] && timer_count[6] == 1'b1) begin // 4*SCL period (which is a 16 count)
                timer_count_idle_scl[1] <= 1'b1; // bit1 set, count up until saturation
            end

            // Output only shown when armed
            if (scl_idle_monitor_arm_i) begin // Only output if its armed
                if (timer_count_idle_scl[1:0] == 2'b00) // was reset above (while armed) due to !scl_i
                    r_scl_idle_monitor_notidle <= 1'b1;
            end else begin
                r_scl_idle_monitor_notidle <= 1'b0;
            end

            // 
            if (timer_count[WIDTH-1]) begin	// MSB set is overflow
                r_tick_overflow <= 1'b1;
            end

            // This is made to latch until reset, to help clock stretch logic support
            //  output from module has a MUX bypass to ensure !rst_n ensure it shows reset
            if (stb_tick_sclhigh_condx) begin
                r_tick_sclhigh <= 1'b1;
            end

            if (scl_idle_monitor_reset_i) begin
                timer_count_idle_scl <= '0;
            end

            r_tick_first <= 1'b0;
        end
    end

    // FIXME see if these comparisons work at smaller width (many use cases only care for the first tick since reset)
    assign stb_tick_first_o    = /*rst_n && */r_tick_first;		// MUX bypass when reset
    assign stb_tick_edgewait_o = clkdiv_stb && timer_count == TIMER_LIMIT0;   // Used to drop OE by driving the edge to pull-up then disconnecting OE
    assign stb_tick_prewait_o  = clkdiv_stb && timer_count == TIMER_LIMIT1;   // START hold, STOP setup
    assign stb_tick_scllow_o   = clkdiv_stb && timer_count == TIMER_LIMIT3;
    assign stb_tick_sclhigh_o  = /*rst_n && */r_tick_sclhigh;        // MUX bypass when reset
    assign stb_tick_idlescl_o  = clkdiv_stb && timer_count == TIMER_LIMIT3;   // happens to be same as above
    assign stb_tick_overflow_o = r_tick_overflow;

    assign scl_idle_monitor_strobe_o   = /*!scl_idle_monitor_reset_i && */timer_count_idle_scl[1];
    assign scl_idle_monitor_notidle_o  = r_scl_idle_monitor_notidle;

endmodule
