/*
 * Copyright (c) 2025 Darryl L. Miles
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// Manage the mapping of MUXed IO options
module tqvp_dlmiles_i2c_io (
    input  [11:0]  reg_cmux,

    input   [7:0]  ui_in,
    output  [7:0]  uo_out,

    output         scl_i,
    input          scl_o,
    input          scl_oe,

    output         sda_i,
    input          sda_o,
    input          sda_oe
);

    // reg_cmux
    // ....................0
    //                    10 See "SCL input source" comment near verilog below
    //                  32   See "SDA input source" comment near verilog below
    //                54     uo_out mapping 01=without OE, 10=I2C0 with OE, 11=I2C1 with OE
    //                       See ""uo_out mapping" comment near verilog below
    //               6       SCL mode (0=pull-down only, 1=direct)
    //                       pull-down only, means only drive SCL LOW, with external pull-up open-drain expected to provide SCL HIGH
    //                       direct, means drive at all times we are not capturing signal input
    //              7        SDA mode (0=pull-down only, 1=direct)
    //                       pull-down only, means only drive SCL LOW, with external pull-up open-drain expected to provide SCL HIGH
    //                       direct, means drive at all times we are not capturing signal input
    //             8         SCL invert (0=non-inverted)
    //            9          SDA invert (0=non-inverted)
    //          10           SCL OE invert (0=non-inverted)
    //         11            SDA OE invert (0=non-inverted)
    //                       Inverts are designed to help out external NPN/PNP combinations
    //

`ifdef NOT_SUPPORTED_BY_IVERILOG // easier to read verilog at top, is unsupported by IVERILOG
    always_comb begin
        case (reg_cmux[1:0])	 // SCL input source
        2'b00: scl_i = ui_in[0]; // GPIO09 I2C0 SCL
        2'b01: scl_i = ui_in[2]; // GPIO11 I2C1 SCL
        2'b10: scl_i = ui_in[4]; // GPIO17 I2C0 SCL
        2'b11: scl_i = ui_in[6]; // GPIO19 I2C1 SCL
        endcase
    end
`else
    // same as above but using IVERLOG support expressions
    assign scl_i = (reg_cmux[1]) ? // SCL input source
        ((reg_cmux[0]) ?	(ui_in[6]) :	// 2'b11 // GPIO19 I2C1 SCL
                                (ui_in[4])) :	// 2'b10 // GPIO17 I2C0 SCL
        ((reg_cmux[0]) ?	(ui_in[2]) :	// 2'b01 // GPIO11 I2C1 SCL
                                (ui_in[0]));	// 2'b00 // GPIO09 I2C0 SCL
`endif

`ifdef NOT_SUPPORTED_BY_IVERILOG // easier to read verilog at top, is unsupported by IVERILOG
    always_comb begin
        case (reg_cmux[3:2])     // SDA input source
        2'b00: sda_i = ui_in[7]; // GPIO20 I2C0 SDA
        2'b01: sda_i = ui_in[5]; // GPIO18 I2C1 SDA
        2'b10: sda_i = ui_in[3]; // GPIO12 I2C0 SDA
        2'b11: sda_i = ui_in[1]; // GPIO10 I2C1 SDA
        endcase
    end
`else
    // same as above but using IVERLOG support expressions
    assign sda_i = (reg_cmux[3]) ? // SDA input source
        ((reg_cmux[2]) ?	(ui_in[1]) :	// 2'b11 // GPIO10 I2C1 SDA
                                (ui_in[3])) :	// 2'b10 // GPIO12 I2C0 SDA
        ((reg_cmux[2]) ?	(ui_in[5]) :	// 2'b01 // GPIO18 I2C1 SDA
                                (ui_in[7]));	// 2'b00 // GPIO20 I2C0 SDA
`endif

    wire scl_o_direct;
    wire sda_o_direct;
    wire scl_o_signal;
    wire sda_o_signal;
    wire scl_oe_signal;
    wire sda_oe_signal;

    // direct output, or via NPN to pull-down (default)
    assign scl_o_direct  = (reg_cmux[6])  ? scl_o         : (scl_oe && !scl_o); // only when demand drive low
    assign sda_o_direct  = (reg_cmux[7])  ? sda_o         : (sda_oe && !sda_o); // only when demand drive low
    assign scl_o_signal  = (reg_cmux[8])  ? ~scl_o_direct : scl_o_direct;
    assign sda_o_signal  = (reg_cmux[9])  ? ~sda_o_direct : sda_o_direct;
    assign scl_oe_signal = (reg_cmux[10]) ? ~scl_oe       : scl_oe;
    assign sda_oe_signal = (reg_cmux[11]) ? ~sda_oe       : sda_oe;

    wire [7:0] i2c0and1_without_oe;
    assign i2c0and1_without_oe = {
        sda_o_signal,	// out[7] GPIO16 I2C0 SDA
        scl_o_signal,	// out[6] GPIO15 I2C1 SCL
        sda_o_signal,	// out[5] GPIO14 I2C1 SDA
        scl_o_signal,	// out[4] GPIO13 I2C0 SCL
        sda_o_signal,	// out[3] GPIO08 I2C0 SDA
        scl_o_signal,	// out[2] GPIO07 I2C1 SCL
        sda_o_signal,	// out[1] GPIO06 I2C1 SDA
        scl_o_signal	// out[0] GPIO05 I2C0 SCL
    };

    wire [7:0] i2c0_with_oe;
    assign i2c0_with_oe = {
        sda_o_signal,	// out[7] GPIO16 I2C0 SDA
        scl_oe_signal,	// out[6] GPIO15
        sda_oe_signal,	// out[5] GPIO14
        scl_o_signal,	// out[4] GPIO13 I2C0 SCL
        sda_o_signal,	// out[3] GPIO08 I2C0 SDA
        scl_oe_signal,	// out[2] GPIO07
        sda_oe_signal,	// out[1] GPIO06
        scl_o_signal	// out[0] GPIO05 I2C0 SCL
    };

    wire [7:0] i2c1_with_oe;
    assign i2c1_with_oe = {
        sda_oe_signal,	// out[7] GPIO16
        scl_o_signal,	// out[6] GPIO15 I2C1 SCL
        sda_o_signal,	// out[5] GPIO14 I2C1 SDA
        scl_oe_signal,	// out[4] GPIO13 
        sda_oe_signal,	// out[3] GPIO08
        scl_o_signal,	// out[2] GPIO07 I2C1 SCL
        sda_o_signal,	// out[1] GPIO06 I2C1 SDA
        scl_oe_signal	// out[0] GPIO05 
    };

`ifdef NOT_SUPPORTED_BY_IVERILOG
    always_comb begin
        case (reg_cmux[5:4]) // uo_out mapping
        2'b00: uo_out[7:0] = i2c0and1_without_oe[7:0];
        2'b01: uo_out[7:0] = i2c0and1_without_oe[7:0];
        2'b10: uo_out[7:0] = i2c0_with_oe[7:0];
        2'b11: uo_out[7:0] = i2c1_with_oe[7:0];
        endcase
    end
`else
    assign uo_out[7:0] = (reg_cmux[5]) ? // uo_out mapping
        ((reg_cmux[4]) ?    (i2c1_with_oe[7:0]) : 		// 2'b11
                            (i2c0_with_oe[7:0])) :		// 2'b10
        ((reg_cmux[4]) ?    (i2c0and1_without_oe[7:0]) :	// 2'b01
                            (i2c0and1_without_oe[7:0]));	// 2'b00
`endif

endmodule
