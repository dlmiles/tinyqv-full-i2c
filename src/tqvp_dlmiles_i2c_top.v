/*
 * Copyright (c) 2025 Darryl L. Miles
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

`include "global.vh"

module tqvp_dlmiles_i2c_top (
    input         clk,          // Clock - the TinyQV project clock is normally set to 64MHz.
    input         rst_n,        // Reset_n - low to reset.

    input  [7:0]  ui_in,        // The input PMOD, always available.  Note that ui_in[7] is normally used for UART RX.
                                // The inputs are synchronized to the clock, note this will introduce 2 cycles of delay on the inputs.

    output [7:0]  uo_out,       // The output PMOD.  Each wire is only connected if this peripheral is selected.
                                // Note that uo_out[0] is normally used for UART TX.

    input [5:0]   address,      // Address within this peripheral's address space
    input [31:0]  data_in,      // Data in to the peripheral, bottom 8, 16 or all 32 bits are valid on write.

    // Data read and write requests from the TinyQV core.
    input [1:0]   data_write_n, // 11 = no write, 00 = 8-bits, 01 = 16-bits, 10 = 32-bits
    input [1:0]   data_read_n,  // 11 = no read,  00 = 8-bits, 01 = 16-bits, 10 = 32-bits
    
    output [31:0] data_out,     // Data out from the peripheral, bottom 8, 16 or all 32 bits are valid on read when data_ready is high.
    output        data_ready,

    output        user_interrupt  // Dedicated interrupt request for this peripheral
);

    // 32bit word aligned, byte addressing
    localparam ADR00_DATA       = `ADR00_DATA;  // 8bit data
    localparam ADR01_STAT       = `ADR01_STAT;
    localparam ADR02_CTRL       = `ADR02_CTRL;
    localparam ADR03_CONF       = `ADR03_CONF;
    localparam ADR04_CMUX       = `ADR04_CMUX;

    localparam DATA_READ_IDLE   = `DATA_READ_IDLE;

    localparam DATA_WRITE_IDLE  = `DATA_WRITE_IDLE;

    wire reg_data_write;
    assign reg_data_write = (address[4:2] == ADR00_DATA[4:2]) && (data_write_n != DATA_WRITE_IDLE);
    wire reg_data_read;
    assign reg_data_read  = (address[4:2] == ADR00_DATA[4:2]) && (data_read_n != DATA_READ_IDLE);

    wire reg_stat_write;
    assign reg_stat_write = (address[4:2] == ADR01_STAT[4:2]) && (data_write_n != DATA_WRITE_IDLE);

    wire reg_ctrl_write;
    assign reg_ctrl_write = (address[4:2] == ADR02_CTRL[4:2]) && (data_write_n != DATA_WRITE_IDLE);
    wire reg_ctrl_read;
    assign reg_ctrl_read  = (address[4:2] == ADR02_CTRL[4:2]) && (data_read_n != DATA_READ_IDLE);

    wire reg_conf_write;
    assign reg_conf_write = (address[4:2] == ADR03_CONF[4:2]) && (data_write_n != DATA_WRITE_IDLE);

    wire reg_cmux_write;
    assign reg_cmux_write = (address[4:2] == ADR04_CMUX[4:2]) && (data_write_n != DATA_WRITE_IDLE);

    reg [11:0] reg_cmux;
    reg [11:0] reg_conf;

    wire  [3:0] fsm_state;
    wire        fsm_run;

    always @(posedge clk) begin
        if (!rst_n) begin
            reg_conf <= '0;
            reg_cmux <= '0;
        end else begin
            // Can only change when FSM_RUN==0 because it invalidates timer integrity
            if (reg_conf_write && !fsm_run) begin
                reg_conf <= data_in[11:0];
            end
            // Recommended to only change when FSM_RUN==0 (but not going to stop you)
            if (reg_cmux_write) begin
                reg_cmux <= data_in[11:0];
            end
        end
    end

    // Timer unit
    wire timer_reset;
    wire timer_run;
    wire tick_first;
    wire tick_edgewait;
    wire tick_prewait;
    wire tick_sclhigh;
    wire tick_scllow;
    wire tick_idlescl;
    wire tick_overflow;
    wire scl_idle_monitor_reset;
    wire scl_idle_monitor_arm;
    wire scl_idle_monitor_strobe;
    wire scl_idle_monitor_notidle;

    wire scl_i; // driven by tqvp_dlmiles_i2c_io
    wire sda_i; // driven by tqvp_dlmiles_i2c_io

    tqvp_dlmiles_i2c_timer tqvp_dlmiles_i2c_timer(
        .clk                        (clk),
        .rst_n                      (rst_n & ~timer_reset),

        .timer_run_i                (timer_run),

        .reg_conf_i                 (reg_conf),

        // Output
        .stb_tick_first_o           (tick_first),
        .stb_tick_edgewait_o        (tick_edgewait),
        .stb_tick_prewait_o         (tick_prewait),
        .stb_tick_sclhigh_o         (tick_sclhigh),
        .stb_tick_scllow_o          (tick_scllow),
        .stb_tick_idlescl_o         (tick_idlescl),
        .stb_tick_overflow_o        (tick_overflow),

        // Maybe in future the monitor can be split from timer
        .scl_idle_monitor_reset_i   (scl_idle_monitor_reset),
        .scl_idle_monitor_arm_i     (scl_idle_monitor_arm),
        .scl_idle_monitor_strobe_o  (scl_idle_monitor_strobe),
        .scl_idle_monitor_notidle_o (scl_idle_monitor_notidle),

        // Visibility of the input state
        .scl_i                      (scl_i),
        .sda_i                      (sda_i)
    );


    wire scl_o;
    wire scl_oe;
    wire sda_o;
    wire sda_oe;

    wire st_err_timeout;    
    wire st_err_io;
    wire st_err_generic;
    wire st_int_raw;
    wire st_int_en;
    wire st_int_edge;

    wire i2c_error_timeout; // signals from FSM to indicate error
    wire i2c_error_io;
    wire i2c_error_generic;

    wire interrupt;

    localparam STAT7_INTR_EN = 7; // bit7

    // Interrupt and Error control unit
    tqvp_dlmiles_i2c_interr tqvp_dlmiles_i2c_interr(
        .clk                (clk),
        .rst_n              (rst_n),

        .reg_stat_i         (data_in[STAT7_INTR_EN]), // INTR_EN
        .stb_stat_i         (reg_stat_write),

        .reg_stat_o         ({st_err_timeout,st_err_io,st_err_generic,st_int_raw,st_int_en,st_int_edge}),

        // strobes that latch error condx
        .stb_error_i        ({i2c_error_timeout,i2c_error_io,i2c_error_generic}),

        /* verilator lint_off PINCONNECTEMPTY */
        .interrupt_raw_o    (/*nc*/),
        /* verilator lint_on PINCONNECTEMPTY */

        // interrupt output line
        .interrupt_o        (interrupt)
    );

    wire        i2c_condx_start_stop;
    wire        i2c_acknack;
    wire        i2c_acknack_valid;

    wire        st_tx_overrun;
    wire        st_tx_full;
    wire        st_tx_empty;
    wire        st_rx_overrun;
    wire        st_rx_full;
    wire        st_rx_empty;

    localparam STAT3_TX_EMPTY = 3;
    localparam STAT0_RX_EMPTY = 0;

    wire  [8:0] reg_data_recv;  // 9bits (includes !is_valid)

    wire [11:0] reg_data_r;
    assign reg_data_r = {
        i2c_condx_start_stop,
        i2c_acknack_valid,
        i2c_acknack,
        reg_data_recv[8],       // inverted reg_data_recv_valid

        reg_data_recv[7:0]
    };
    wire [11:0] reg_data_w;
    assign reg_data_w[11:0] = data_in[11:0];

    wire  [8:0] i2c_txd_data;   // 9bit MSB is direction see DIR_TXD/DIR_RXD
    wire        i2c_txd_valid;
    wire        i2c_txd_ready;

    wire  [7:0] i2c_rxd_data;
    assign i2c_rxd_data = 0;    // FIXME use txd_data+direction
    wire        i2c_rxd_valid;
    assign i2c_rxd_valid = 0;   // FIXME use txd_data+direction

    tqvp_dlmiles_i2c_fifo tqvp_dlmiles_i2c_fifo(
        .clk                    (clk),
        .rst_tx_n_i             (rst_n && ~(reg_stat_write & data_in[STAT3_TX_EMPTY])),
        .rst_rx_n_i             (rst_n && ~(reg_stat_write & data_in[STAT0_RX_EMPTY])),

        .reg_data_recv_o        (reg_data_recv), // 9bits MSB is inverted reg_data_recv_valid_o
        /* verilator lint_off PINCONNECTEMPTY */
        .reg_data_recv_valid_o  (/*nc*/),
        /* verilator lint_on PINCONNECTEMPTY */
        .stb_data_recv_ready_i  (reg_data_read),

        .reg_data_send_i        (reg_data_w),
        .reg_data_send_valid_i  (reg_data_write),

        .i2c_txd_data_o         (i2c_txd_data),
        .i2c_txd_valid_o        (i2c_txd_valid),
        .i2c_txd_ready_i        (i2c_txd_ready),

        .i2c_rxd_data_i         (i2c_rxd_data),
        .i2c_rxd_valid_i        (i2c_rxd_valid),

        .st_tx_overrun_o        (st_tx_overrun),
        .st_tx_full_o           (st_tx_full),
        .st_tx_empty_o          (st_tx_empty),
        .st_rx_overrun_o        (st_rx_overrun),
        .st_rx_full_o           (st_rx_full),
        .st_rx_empty_o          (st_rx_empty)
    );


    wire [11:0] reg_ctrl_r;
    wire [11:0] reg_ctrl_w;
    assign reg_ctrl_w[11:0] = data_in[11:0];

    tqvp_dlmiles_i2c_fsm tqvp_dlmiles_i2c_fsm(
        .clk                        (clk),
        .rst_n                      (rst_n),

        .reg_ctrl_o                 (reg_ctrl_r),
        .stb_ctrl_read_i            (reg_ctrl_read),

        .reg_ctrl_i                 (reg_ctrl_w),
        .stb_ctrl_write_i           (reg_ctrl_write),

        .fsm_run_o                  (fsm_run),
        .fsm_state_o                (fsm_state),

        .tick_first_i               (tick_first),
        .tick_edgewait_i            (tick_edgewait),
        .tick_prewait_i             (tick_prewait),
        .tick_sclhigh_i             (tick_sclhigh),
        .tick_scllow_i              (tick_scllow),
        .tick_idlescl_i             (tick_idlescl),
        .tick_overflow_i            (tick_overflow),

        .stb_i2c_error_timeout_o    (i2c_error_timeout),
        .stb_i2c_error_io_o         (i2c_error_io),
        .stb_i2c_error_generic_o    (i2c_error_generic),

        .scl_idle_monitor_reset_o   (scl_idle_monitor_reset),
        .scl_idle_monitor_arm_o     (scl_idle_monitor_arm),
        .scl_idle_monitor_strobe_i  (scl_idle_monitor_strobe),
        .scl_idle_monitor_notidle_i (scl_idle_monitor_notidle),

        .timer_reset_o              (timer_reset),
        .timer_run_o                (timer_run),

        .stb_data_read_i            (reg_data_read),

        .i2c_acknack_o              (i2c_acknack),
        .i2c_acknack_valid_o        (i2c_acknack_valid),
        .i2c_condx_start_stop_o     (i2c_condx_start_stop),

        .i2c_txd_data_i             (i2c_txd_data),
        .i2c_txd_valid_i            (i2c_txd_valid),
        .i2c_txd_ready_o            (i2c_txd_ready),

        .scl_i                      (scl_i),
        .sda_i                      (sda_i),

        .scl_o                      (scl_o),
        .scl_oe_o                   (scl_oe),
        .sda_o                      (sda_o),
        .sda_oe_o                   (sda_oe)
    );

    // REG_STAT view
    wire [11:0] reg_stat_r;
    assign reg_stat_r = {
        st_err_timeout,
        st_err_io,
        st_err_generic,
        st_int_raw,

        st_int_en,      // bit7 so as to be accessible from all read/write sizes
        st_int_edge,
        st_tx_overrun,  // CPU side error
        st_tx_full,     // most interested in full
        st_tx_empty,
        st_rx_overrun,  // CPU too slow to service
        st_rx_full,     // less interested in !full
        st_rx_empty     // most interested in !empty
    };

    wire scl_i_raw;
    wire sda_i_raw;

    // IO Mapping MUX control unit (you could just bypass this to reduce resource usage)
    tqvp_dlmiles_i2c_io tqvp_dlmiles_i2c_io(
        // no clock or reset as combinational muxing only

        .reg_cmux   (reg_cmux[11:0]),   // Control MUX reg

        .ui_in      (ui_in[7:0]),       //i
        .uo_out     (uo_out[7:0]),      //o

        // external view nomenclature   //verilog signal direction
        .scl_i      (scl_i_raw),        //o
        .scl_o      (scl_o),            //i
        .scl_oe     (scl_oe),           //i

        .sda_i      (sda_i_raw),        //o
        .sda_o      (sda_o),            //i
        .sda_oe     (sda_oe)            //i
    );

    synchronizer #(.STAGES(2), .WIDTH(1)) synchronizer_scl_i_inst (.clk(clk), .data_in(scl_i_raw), .data_out(scl_i));
    synchronizer #(.STAGES(2), .WIDTH(1)) synchronizer_sda_i_inst (.clk(clk), .data_in(sda_i_raw), .data_out(sda_i));

`ifdef NOT_SUPPORTED_BY_IVERILOG
    always_comb begin
        case  (address[4:2]) // 32bit alignment of byte aligned address
            ADR00_DATA[4:2]:    data_out = {20'b0, reg_data_r[11:0]};
            ADR01_STAT[4:2]:    data_out = {20'b0, reg_stat_r[11:0]};
            ADR02_CTRL[4:2]:    data_out = {20'b0, reg_ctrl_r[11:0]};
            ADR03_CONF[4:2]:    data_out = {20'b0, reg_conf  [11:0]};
            ADR04_CMUX[4:2]:    data_out = {20'b0, reg_cmux  [11:0]};
            default:            data_out = 32'h0;
        endcase
    end
`else
    wire [2:0] addr32;
    assign addr32 = address[4:2]; // 32bit aligned
    assign data_out =
         (addr32 == ADR04_CMUX[4:2]) ? ({20'b0, reg_cmux  [11:0]}) :
        ((addr32 == ADR03_CONF[4:2]) ? ({20'b0, reg_conf  [11:0]}) :
        ((addr32 == ADR02_CTRL[4:2]) ? ({20'b0, reg_ctrl_r[11:0]}) :
        ((addr32 == ADR01_STAT[4:2]) ? ({20'b0, reg_stat_r[11:0]}) :
        ((addr32 == ADR00_DATA[4:2]) ? ({20'b0, reg_data_r[11:0]}) :
                                       ( 32'h0 )
        ))));
`endif

    assign data_ready = 1'b1;   // always ready no delay

    assign user_interrupt = interrupt;

endmodule
