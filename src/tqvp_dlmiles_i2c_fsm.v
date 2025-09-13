/*
 * Copyright (c) 2025 Darryl L. Miles
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

`include "global.vh"

module tqvp_dlmiles_i2c_fsm (
    input           clk,          // Clock
    input           rst_n,        // Reset_n - low to reset

    output   [11:0] reg_ctrl_o,
    input           stb_ctrl_read_i,

    input    [11:0] reg_ctrl_i,
    input           stb_ctrl_write_i,

    input           stb_data_read_i,

    output          fsm_run_o,
    output    [3:0] fsm_state_o,

    input           tick_first_i,
    input           tick_edgewait_i,
    input           tick_prewait_i,
    input           tick_sclhigh_i,
    input           tick_scllow_i,
    input           tick_idlescl_i,
    input           tick_overflow_i,

    output reg      stb_i2c_error_timeout_o,
    output reg      stb_i2c_error_io_o,
    output reg      stb_i2c_error_generic_o,

    output reg      scl_idle_monitor_reset_o,
    output reg      scl_idle_monitor_arm_o,
    input           scl_idle_monitor_strobe_i,
    input           scl_idle_monitor_notidle_i,

    output          timer_reset_o,
    output          timer_run_o,

    output          i2c_acknack_o,
    output          i2c_acknack_valid_o,
    output          i2c_condx_start_stop_o,

    input     [8:0] i2c_txd_data_i,             // MSB is direction DIR_TXD or DIR_RXD
    input           i2c_txd_valid_i,
    output reg      i2c_txd_ready_o,

    input           scl_i,
    input           sda_i,

    output          scl_o,
    output          scl_oe_o,
    output          sda_o,
    output          sda_oe_o
);

    // Maybe this should be moved to *.vh as copied from tqvp_dlmiles_i2c_fifo module
    localparam DIR_TXD = `DIR_TXD;
    localparam DIR_RXD = `DIR_RXD;

    reg r_scl_o;
    reg r_scl_oe;
    reg r_sda_o;
    reg r_sda_oe;

    reg   [3:0] fsm_state;
    /* verilator lint_off UNUSEDSIGNAL */
    reg   [3:0] fsm_next_state; // FIXME verification assert, 1 clock delay of w_timer_reset_o
    /* verilator lint_on  UNUSEDSIGNAL */
`ifdef SIM
    reg   [3:0] fsm_last_state;
`endif

    reg  [11:0] reg_ctrl;

`ifndef SYNTHESIS
    reg [175:0] fsm_state_string;
`endif

    localparam ST_RESET                 = 4'd0;
    localparam ST_IDLE_NEED_RESTART     = 4'd1;
    localparam ST_IDLE                  = 4'd2;
    localparam ST_WAIT_LINE_IDLE        = 4'd3;
    localparam ST_PREWAIT               = 4'd4;
    localparam ST_WAIT_SCL_LOW          = 4'd5;
    localparam ST_WAIT_SCL_HIGH         = 4'd6;
    localparam ST_SEND_BIT              = 4'd7;
    localparam ST_RECV_ACKNACK          = 4'd8;
    localparam ST_WAIT_ACKNACK          = 4'd9;
    localparam ST_RECV_BIT              = 4'd10;
    localparam ST_SEND_ACKNACK          = 4'd11;
    localparam ST_PREPARE_STOP          = 4'd12;
    localparam ST_SEND_STOP             = 4'd13;
    localparam ST_SEND_STOP_WAIT        = 4'd14;
    localparam ST_BEFORE_RUN            = 4'd15; // FIXME unused

`ifndef SYNTHESIS
    always @(*) begin
        case (fsm_state)
            ST_RESET :                fsm_state_string = "ST_RESET              ";
            ST_IDLE_NEED_RESTART :    fsm_state_string = "ST_IDLE_NEED_RESTART  ";
            ST_IDLE :                 fsm_state_string = "ST_IDLE               ";
            ST_WAIT_LINE_IDLE :       fsm_state_string = "ST_WAIT_LINE_IDLE     ";
            ST_PREWAIT :              fsm_state_string = "ST_PREWAIT            ";
            ST_WAIT_SCL_LOW :         fsm_state_string = "ST_WAIT_SCL_LOW       ";
            ST_WAIT_SCL_HIGH :        fsm_state_string = "ST_WAIT_SCL_HIGH      ";
            ST_SEND_BIT :             fsm_state_string = "ST_SEND_BIT           ";
            ST_RECV_ACKNACK :         fsm_state_string = "ST_RECV_ACKNACK       ";
            ST_WAIT_ACKNACK :         fsm_state_string = "ST_WAIT_ACKNACK       ";
            ST_RECV_BIT  :            fsm_state_string = "ST_RECV_BIT           ";
            ST_SEND_ACKNACK :         fsm_state_string = "ST_SEND_ACKNACK       ";
            ST_PREPARE_STOP :         fsm_state_string = "ST_PREPARE_STOP       ";
            ST_SEND_STOP :            fsm_state_string = "ST_SEND_STOP          ";
            ST_SEND_STOP_WAIT :       fsm_state_string = "ST_SEND_STOP_WAIT     ";
            ST_BEFORE_RUN :           fsm_state_string = "ST_BEFORE_RUN         ";
            default :                 fsm_state_string = "??????????????????????";
        endcase
    end
`endif

    localparam CTRL0_FSM_RUN        = 0; // FSM_START
    localparam CTRL1_FSM_STOP       = 1; // both to RESET
    localparam CTRL2_START          = 2;
    localparam CTRL3_STOP           = 3; // nominal STOP
    localparam CTRL4_FORCE_STOP     = 4;
    localparam CTRL6_ACKNACK        = 6; // value
    localparam CTRL7_ACKNACK_VALID  = 7; // is_valid

    wire fsm_reset;
    assign fsm_reset = reg_ctrl_i[CTRL0_FSM_RUN] && reg_ctrl_i[CTRL1_FSM_STOP]; // stb_ctrl_write_i==1 guard when valid

    // Optional feedback, could be ifdef out of design
    reg i2c_clock_stretch;
    reg i2c_clock_stretch_acknack;

    wire fsm_run;
    assign fsm_run = reg_ctrl[CTRL0_FSM_RUN];   // rename of a bit
    wire send_start;
    assign send_start = reg_ctrl[CTRL2_START];
    wire send_stop;
    assign send_stop = reg_ctrl[CTRL3_STOP];
    wire force_stop;
    assign force_stop = reg_ctrl[CTRL4_FORCE_STOP];

    reg       need_start;
    // The bit_count[MSB] has additional meaning, it is used to indicate
    // that a character is loaded, but the counting goes 6 downto -1 (MSB to LSB)
    // bit_count[MSB] can be checked to see if another bit is needs to be
    // sent
    //  bit_count value:  6  5  4  3  2  1  0 -1 (relating to value of bit_count)
    //  send bit ID    :  7  6  5  4  3  2  1  0 (relating to I2C data bit ID)
    //  has more bits  :  1  1  1  1  1  1  1  0 (follow inverted MSB)
    //  MSB            :  0  0  0  0  0  0  0  1
    reg [3:0] bit_count;        // 0..8
    wire has_moredata;
    assign has_moredata = bit_count[2:0] != 3'b111;     // roll over
    wire has_acknack;
    assign has_acknack = bit_count[3];  // MSB from wrap-under-decrement
    reg       direction;
    // data[MSB] is also used for START/STOP condition SDA value
    reg [7:0] data;             // current send data MSB sent first

    reg       i2c_acknack;
    assign i2c_acknack_o       = i2c_acknack;
    reg       i2c_acknack_edge;
    //assign i2c_acknack_o       = i2c_acknack_edge;
    reg       i2c_acknack_valid;
    assign i2c_acknack_valid_o = i2c_acknack_valid;
    reg       i2c_condx_start_stop;
    assign i2c_condx_start_stop_o = i2c_condx_start_stop;

    wire scl_clock_stretch;
    assign scl_clock_stretch            = (0) ? 1'b1 : scl_i;   // FIXME enable/disable CLOCK_STRETCH support
    wire scl_clock_stretch_acknack;
    assign scl_clock_stretch_acknack    = (0) ? 1'b1 : scl_i;   // FIXME enable/disable CLOCK_STRETCH support

`ifdef SIM
    initial $info("SIM is defined");
    localparam HIGHX = 1'bx;
`else
`ifndef SYNTHESIS_OPENLANE
    // Hmm the verilator used in flow, or flow itself does not handle $info FIXME remove ifndef
//    initial $info("SIM is undefined");
`endif
    localparam HIGHX = 1'b1;
`endif
`ifndef SYNTHESIS_OPENLANE
    // Hmm the verilator used in flow, or flow itself does not handle $info FIXME remove ifndef
//    initial $info("HIGHX=%b (showing X when -DSIM is active in simulation)", HIGHX);
`endif

`ifdef SIM
    reg on_entry;       // strobe when FSM is the first cycle, state machine validation
`endif

    // really want $isvalid() did not like name of $isknown() as too close
    //  to official $isunknown() and could be misread at a glance
    function automatic logic isvalid1 (input logic v);
        // FIXME trying to say any width is allowed for 'v'
        isvalid1 = !$isunknown(v);
    endfunction
    function automatic logic isvalid2 (input logic [1:0] v);
        isvalid2 = !$isunknown(v);
    endfunction
    function automatic logic isvalid4 (input logic [3:0] v);
        isvalid4 = !$isunknown(v);
    endfunction
    function automatic logic isvalid8 (input logic [7:0] v);
        isvalid8 = !$isunknown(v);
    endfunction
    function automatic logic isvalid9 (input logic [8:0] v);
        isvalid9 = !$isunknown(v);
    endfunction
`ifndef SYNTHESIS_OPENLANE
    // sanity check the simulation feature
    initial assert(isvalid1({1'b0}));
    initial assert(isvalid1({1'b1}));
    initial assert(isvalid2({1'b0, 1'b0}));
    initial assert(isvalid2({1'b0, 1'b1}));
    initial assert(isvalid2({1'b1, 1'b0}));
    initial assert(isvalid2({1'b1, 1'b1}));
    initial assert(!isvalid1({1'bx}));
    initial assert(!isvalid1({1'bz}));
    initial assert(!isvalid2({1'bx, 1'b0}));    // FAILED with (input logic v)
    initial assert(!isvalid2({1'bx, 1'b1}));    // FAILED
    initial assert(!isvalid2({1'bx, 1'b0}));    // FAILED
    initial assert(!isvalid2({1'bx, 1'b1}));    // FAILED
    initial assert(!isvalid2({1'b0, 1'bx}));
    initial assert(!isvalid2({1'b0, 1'bx}));
    initial assert(!isvalid2({1'b1, 1'bx}));
    initial assert(!isvalid2({1'b1, 1'bx}));
    initial assert(!isvalid2({1'bx, 1'bx}));
`endif

    reg   [3:0] w_fsm_next_state;       // wire
    //initial assert($width(w_fsm_next_state) == $width(fsm_state));
    reg         w_timer_reset_o;    // wire
    /* verilator lint_off UNUSEDSIGNAL */
    reg         r_timer_reset_o;    // FIXME verification assert, 1 clock delay of w_timer_reset_o
    /* verilator lint_on UNUSEDSIGNAL */
    assign timer_reset_o = w_timer_reset_o;
    reg          w_timer_run;
    assign timer_run_o = w_timer_run;

    always @(*) begin
        w_timer_reset_o = 1'b0;
        w_fsm_next_state = fsm_state;
        w_timer_run = 1'b0;
        if (!rst_n) begin
            w_timer_reset_o = 1'b1;
            w_fsm_next_state = ST_RESET;
        end else if (stb_ctrl_write_i) begin
            // This is so it doesn't miss a tick (inbound strobe) during other kinds of IO access
            // Which at the moment stops the FSM for a single cycle per write access
            w_timer_run = 1'b0;
            if (fsm_reset) begin
                w_fsm_next_state = ST_RESET;
            end
        end else if (fsm_run) begin
            w_timer_run = 1'b1;
            case (fsm_state)
                ST_RESET: begin
                    w_timer_reset_o = 1'b1;
                    w_fsm_next_state = ST_IDLE_NEED_RESTART;
                end
                ST_BEFORE_RUN: begin
//                    if (r_scl_oe) begin
//                        w_timer_reset_o = 1'b1;
//                        w_fsm_next_state = ST_IDLE;
//                    end else begin
//                        w_fsm_next_state = ST_IDLE_NEED_RESTART;
//                    end
                end
                ST_IDLE_NEED_RESTART: begin
                    w_timer_reset_o = 1'b1;         // no timers are running, only line monitors
                    if (force_stop) begin
                        w_fsm_next_state = ST_PREPARE_STOP;
                    end if (i2c_txd_valid_i || (send_start && need_start)) begin
                        w_fsm_next_state = ST_WAIT_LINE_IDLE;
                    end else if (send_stop) begin
                        w_fsm_next_state = ST_PREPARE_STOP;
                    end else begin
                        //w_fsm_next_state = ST_RESET;
                    end
                end
                ST_IDLE: begin
                    w_timer_reset_o = 1'b0;      // timers running
                    if (force_stop) begin
                        w_timer_reset_o = 1'b1;  // reset for next state
                        w_fsm_next_state = ST_PREPARE_STOP;
                    end else if (has_acknack && direction == DIR_TXD) begin
                        w_timer_reset_o = 1'b1;  // reset for next state
                        w_fsm_next_state = ST_RECV_ACKNACK;
                    end else if (has_acknack && direction == DIR_RXD) begin
                        w_timer_reset_o = 1'b1;  // reset for next state
                        w_fsm_next_state = ST_SEND_ACKNACK;
                    end else if (has_moredata && direction == DIR_TXD) begin
                        w_fsm_next_state = ST_SEND_BIT;
                    end else if (has_moredata && direction == DIR_RXD) begin
                        w_fsm_next_state = ST_RECV_BIT;
                    end else if (i2c_txd_valid_i && i2c_txd_data_i[8] == DIR_TXD) begin
                        w_fsm_next_state = ST_SEND_BIT;
                    end else if (i2c_txd_valid_i && i2c_txd_data_i[8] == DIR_RXD) begin
                        w_fsm_next_state = ST_RECV_BIT;
                    end else if (send_stop) begin
                        w_timer_reset_o = 1'b1;  // reset for next state
                        w_fsm_next_state = ST_PREPARE_STOP;
                    end if (!tick_scllow_i) begin
                        /* nop */
                    end else begin
                        w_fsm_next_state = ST_IDLE_NEED_RESTART;
                    end
                end
                ST_WAIT_LINE_IDLE: begin
                    w_timer_reset_o = 1'b0;      // timers running
                    if (need_start) begin
                        if (tick_idlescl_i) begin // read SCL check has been HI for timer
                            w_timer_reset_o = 1'b1; // reset for next state
                            w_fsm_next_state = ST_PREWAIT;
                        end
                    end else begin
                        if (0) begin    // FIXME scl_idle_monitor_strobe
                            w_fsm_next_state = ST_RESET;
                        end else begin
                            if (tick_edgewait_i) begin
                                w_timer_reset_o = 1'b1;
                                w_fsm_next_state = ST_IDLE;
                            end
                        end
                    end
                end
                ST_PREWAIT: begin
                    if (need_start) begin
                        w_timer_reset_o = 1'b1;     // reset for next state
                        w_fsm_next_state = ST_WAIT_SCL_HIGH;
                    end if (tick_prewait_i /*&& scl_i == !has_moredata*/) begin // FIXME uncomment
                        // NO TIMER RESET must run on for correct SCL_LOW period
                        w_fsm_next_state = ST_WAIT_SCL_LOW;
`ifdef unused
                    end if (tick_prewait_i) begin
                        w_fsm_next_state = ST_RESET;
`endif
                    end
                end
                ST_WAIT_SCL_LOW: begin
                    if (tick_scllow_i) begin
                        w_timer_reset_o = 1'b1;     // reset for next state
                        w_fsm_next_state = ST_WAIT_SCL_HIGH;
                    end
                end
                ST_WAIT_SCL_HIGH: begin
                    if (tick_sclhigh_i) begin
                        if (scl_clock_stretch) begin
                            w_timer_reset_o = 1'b1; // reset for next state
                            w_fsm_next_state = ST_IDLE;
                        end
                    end
                end
                ST_SEND_BIT: begin
                    w_timer_reset_o = 1'b1;      // reset for next state
                    w_fsm_next_state = ST_PREWAIT;
                end
                ST_RECV_ACKNACK: begin
                    if (tick_edgewait_i) begin
                    end
                    if (tick_prewait_i) begin
                    end
                    if (tick_scllow_i) begin
                        w_timer_reset_o = 1'b1;     // reset for next state
                        w_fsm_next_state = ST_WAIT_ACKNACK;
                    end
                end
                ST_WAIT_ACKNACK: begin
                    if (tick_edgewait_i) begin
                    end
                    if (tick_sclhigh_i) begin // tick_sclhigh_i must latch
                        if (scl_clock_stretch_acknack) begin
                            w_timer_reset_o = 1'b1; // reset for next state
                            w_fsm_next_state = ST_IDLE_NEED_RESTART;
                        end
                    end
                end
                ST_RECV_BIT: begin
                    w_timer_reset_o = 1'b1;      // reset for next state
                    w_fsm_next_state = ST_WAIT_SCL_LOW;
                end
                ST_SEND_ACKNACK: begin
                    // FIXME
                end
                ST_PREPARE_STOP: begin
                    if (r_sda_o || !r_sda_oe) begin
                        if (r_scl_o || !r_scl_oe) begin
                        end
                        if (tick_prewait_i) begin
                            w_timer_reset_o = 1'b1;  // reset for iteration of next state
                            w_fsm_next_state = ST_SEND_STOP;
                        end
                    end else begin
                        w_timer_reset_o = 1'b1;  // reset for iteration of next state
                        w_fsm_next_state = ST_SEND_STOP;
                    end
                end
                ST_SEND_STOP: begin
                    if (tick_edgewait_i) begin
                        w_timer_reset_o = 1'b1;      // reset for next state
                        w_fsm_next_state = ST_SEND_STOP_WAIT;
                    end
                end
                ST_SEND_STOP_WAIT: begin
                    if (tick_sclhigh_i) begin
                        w_timer_reset_o = 1'b1;     // reset for next state
                        w_fsm_next_state = ST_WAIT_SCL_HIGH;
                    end
                end
            endcase
        end
    end

    // FSM (this controls the logic view of SCL/SDA lines and OEs)
    always @(posedge clk) begin
        // default low precedece defaults
        r_timer_reset_o <= 1'b0;
        i2c_txd_ready_o <= 1'b0;
        fsm_state <= w_fsm_next_state;  // wire to reg

        if (stb_ctrl_read_i) begin
            i2c_acknack_valid <= 1'b0; // RC (read will clear)
        end

        // FIXME move items to block below reset, check rst_n==0 interaction
        if (stb_ctrl_write_i) begin
            reg_ctrl[CTRL2_START]      <= reg_ctrl_i[CTRL2_START];
            reg_ctrl[CTRL3_STOP]       <= reg_ctrl_i[CTRL3_STOP];
            reg_ctrl[CTRL4_FORCE_STOP] <= reg_ctrl_i[CTRL4_FORCE_STOP] | reg_ctrl[CTRL4_FORCE_STOP];    // W1S (write one to set)

            // By default FSM_RESET will cause FSM_STOP
            if (!fsm_reset && reg_ctrl_i[CTRL0_FSM_RUN]) begin
                reg_ctrl[CTRL0_FSM_RUN] <= 1'b1;        // W1S (write one to set)
            end
            if (reg_ctrl_i[CTRL1_FSM_STOP]) begin // including FSM_RESET
                reg_ctrl[CTRL0_FSM_RUN] <= 1'b0;        // W1C (write one to clear)
            end
            if (reg_ctrl_i[CTRL7_ACKNACK_VALID]) begin  //
                reg_ctrl[CTRL6_ACKNACK]       <= reg_ctrl_i[CTRL6_ACKNACK];
                reg_ctrl[CTRL7_ACKNACK_VALID] <= reg_ctrl_i[CTRL7_ACKNACK_VALID];
            end
        end
        if (stb_data_read_i) begin
            reg_ctrl[CTRL7_ACKNACK_VALID] <= 1'b0;
        end

        if (!rst_n) begin
            // ST_RESET stuff
            r_scl_oe   <= 1'b0;
            r_scl_o    <= HIGHX;  // align with pull-up state
            r_sda_oe   <= 1'b0;
            r_sda_o    <= HIGHX;  // align with pull-up state
            need_start     <= 1'b1;
            data[7:0]      <= 8'bxxxxxxxx;
            bit_count[3:0] <= 4'b0111;  // bit_count==0 indicates we are not sending or recving
// TinyQV SPI interface does not handle X data_in/data_out
//`ifdef SIM
//            i2c_acknack                 <= 1'bx;        // this is exposed to CPU interface so make is consistent
//`else
            i2c_acknack                 <= 1'b0;        // LOWX
//`endif
            i2c_acknack_edge            <= 1'b0;        // ACK
            i2c_acknack_valid           <= 1'b0;
            i2c_clock_stretch           <= 1'b0;
            i2c_clock_stretch_acknack   <= 1'b0;
            i2c_condx_start_stop        <= 1'b0;
            scl_idle_monitor_arm_o      <= 1'b0;
            scl_idle_monitor_reset_o    <= 1'b0;
            direction                   <= DIR_TXD;     // assume TX

            // controller reset stuff
            stb_i2c_error_timeout_o <= 1'b0;    // hardware reset additionally resets
            stb_i2c_error_io_o      <= 1'b0;
            stb_i2c_error_generic_o <= 1'b0;
            i2c_txd_ready_o         <= 1'b0;

            // hardware reset stuff
            reg_ctrl[CTRL0_FSM_RUN]    <= 1'b0; // fsm_run
            reg_ctrl[CTRL3_STOP]       <= 1'b0; // send_stop
            reg_ctrl[CTRL4_FORCE_STOP] <= 1'b0; // force_stop

            r_timer_reset_o <= 1'b1;        // ST_RESET does not care for timer
            fsm_next_state <= ST_RESET;
`ifdef SIM
            on_entry <= 1'b1; // VALIDATION
`endif
        end else if (stb_ctrl_write_i) begin    // CPU control has high priority
            if(fsm_state == ST_IDLE_NEED_RESTART) begin
                // kicks state machine
                //fsm_next_state <= ST_BEFORE_RUN;
            end
            // precedence here to inhibit the current fsm_run this cycle
            if (fsm_reset) begin // reset
                stb_i2c_error_timeout_o <= 1'b0;        // hardware reset additionally resets
                stb_i2c_error_io_o      <= 1'b0;
                stb_i2c_error_generic_o <= 1'b0;
                i2c_txd_ready_o         <= 1'b0;
                
                // We don't change the current fsm_run state
                // We assume CPU can issue FSM_STOP before FSM_RESET
                // We assume CPU can issue FMS_RESET then FSM_START
                
                // The FSM_SEND_START and FSM_SEND_STOP and DSM_FORCE_STOP
                //  are updated as the same time to a new state from the CPU write transaction

                need_start <= 1'b1;             // here as was moved from ST_RESET due to its use when transaction is active to release OE
                fsm_next_state <= ST_RESET;
            end
            if (1) begin        // FIXME consider this better
                i2c_clock_stretch           <= 1'b0;
                i2c_clock_stretch_acknack   <= 1'b0;
            end
        end else if (fsm_run) begin
            // counter
            // FIXME maybe we don't allow immediate stop while SCL==0 (ST_PREWAIT/ST_WAIT_SCL_LOW) then release then stop
            case (fsm_state)
                ST_RESET: begin
                    r_scl_oe   <= 1'b0;
                    r_scl_o    <= HIGHX;  // align with pull-up state
                    r_sda_oe   <= 1'b0;
                    r_sda_o    <= HIGHX;  // align with pull-up state
                    data[7:0]  <= 8'bxxxxxxxx;
                    bit_count[3:0] <= 4'b0111;  // bit_count==0 indicates we are not sending or recving
`ifdef SIM
                    i2c_acknack            <= 1'b0; // FIXME TinyQV does not handle X
`else
                    i2c_acknack            <= 1'bx;
`endif
                    i2c_acknack_valid      <= 1'b0;
                    // no i2c_acknack_edge reset here as ST_REST can occur inside transaction
                    i2c_condx_start_stop   <= 1'b0;
                    scl_idle_monitor_arm_o <= 1'b0;
                    direction              <= DIR_TXD;  // assume TX

                    r_timer_reset_o <= 1'b1;
                    fsm_next_state <= ST_IDLE_NEED_RESTART;
                end
                ST_BEFORE_RUN: begin
//`ifndef SYNTHESIS_OPENLANE
//                    assert(!need_start);
//`endif
//                    scl_idle_monitor_arm_o <= 1'b0;
//                    if (r_scl_oe) begin
//`ifndef SYNTHESIS_OPENLANE
//                        assert(isvalid1(r_scl_o));
//                        assert(isvalid1(r_sda_o));
//`endif
//                        r_timer_reset_o <= 1'b1;
//                        fsm_next_state <= ST_IDLE;
//                    end else begin
//`ifndef SYNTHESIS_OPENLANE
//                        assert(r_scl_o === HIGHX);
//                        assert(r_sda_o === HIGHX);
//`endif
//                        fsm_next_state <= ST_IDLE_NEED_RESTART;
//                    end
                end
                ST_IDLE_NEED_RESTART: begin     // entry has OE=off
`ifndef SYNTHESIS_OPENLANE
                    assert(!r_scl_oe);
                    assert(r_scl_o === HIGHX);  // HIGH
                    //if ($isunknown(r_scl_o)) $info("ST_IDLE_NEED_RESTART r_scl_o=%b (HIGHX=%b)", r_scl_o, HIGHX); // DEBUG
                    assert(!r_sda_oe);
                    assert(r_sda_o === HIGHX);
                    //if ($isunknown(r_sda_o)) $info("ST_IDLE_NEED_RESTART r_sda_o=%b (HIGHX=%b)", r_sda_o, HIGHX); // DEBUG
`endif
                    r_timer_reset_o <= 1'b1;        // no timers are running, only line monitors
                    if (force_stop) begin
                        // already sets all output signals
                        fsm_next_state <= ST_PREPARE_STOP;
                    end if (i2c_txd_valid_i || (send_start && need_start)) begin        // waiting for data to appear
`ifdef SIM
                        r_scl_o <= 1'b1;                // replace X (HIGHX) with 1 (matching external pull-up)
                        r_sda_o <= 1'b1;                // replace X (HIGHX) with 1 (matching external pull-up)
`endif
                        fsm_next_state <= ST_WAIT_LINE_IDLE;
                    end else if (send_stop) begin
                        // already sets all output signals
                        fsm_next_state <= ST_PREPARE_STOP;
                    end else begin
                        //fsm_next_state <= ST_RESET;
                    end
                end
                ST_IDLE: begin  // active idle, looking for work, when we still have line aquisition
`ifndef SYNTHESIS_OPENLANE
`ifdef SIM
                    assert(on_entry == tick_first_i);   // check timer was reset on_entry
`endif
                    assert(r_scl_oe);
                    assert(r_scl_o);            // HIGH
                    assert(r_sda_oe == ~direction);
                    assert(isvalid1(r_sda_o));
                    //if ($isunknown(r_sda_o)) $info("ST_IDLE r_sda_o=%b", r_sda_o); // DEBUG
                    assert(!need_start);        // indicates controller has line acquisition
`endif
                    // FIXME work through these to see which need a timer reset
                    if (force_stop) begin
                        r_timer_reset_o <= 1'b1;  // reset for next state
                        fsm_next_state <= ST_PREPARE_STOP;
                    end else if (has_acknack && direction == DIR_TXD) begin
                        r_scl_o <= 1'b0;
`ifndef SYNTHESIS_OPENLANE
                        assert($isunknown(data[7]));
`endif
                        r_timer_reset_o <= 1'b1;  // reset for next state
                        fsm_next_state <= ST_RECV_ACKNACK;
                    end else if (has_acknack && direction == DIR_RXD) begin
                        r_scl_o <= 1'b0;
                        r_timer_reset_o <= 1'b1;  // reset for next state
                        fsm_next_state <= ST_SEND_ACKNACK;
                    end else if (has_moredata && direction == DIR_TXD) begin    // sending
`ifndef SYNTHESIS_OPENLANE
                        assert(r_sda_oe);
`endif
                        fsm_next_state <= ST_SEND_BIT;
                    end else if (has_moredata && direction == DIR_RXD) begin    // sending
`ifndef SYNTHESIS_OPENLANE
                        assert(!r_sda_oe);
`endif
                        fsm_next_state <= ST_RECV_BIT;
                    end else if (i2c_txd_valid_i && i2c_txd_data_i[8] == DIR_TXD) begin
`ifndef SYNTHESIS_OPENLANE
                        assert(isvalid9(i2c_txd_data_i));       // all bits valid
`endif
                        bit_count <= 4'd7; // zero-based count of 8
                        data[7:0] <= i2c_txd_data_i[7:0];
                        direction <= i2c_txd_data_i[8];
                        i2c_txd_ready_o <= 1'b1;        // strobe as we load
                        fsm_next_state <= ST_SEND_BIT;
                    end else if (i2c_txd_valid_i && i2c_txd_data_i[8] == DIR_RXD) begin
`ifndef SYNTHESIS_OPENLANE
                        assert(isvalid9(i2c_txd_data_i));       // all bits valid
`endif
                        bit_count <= 4'd7; // zero-based count of 8
                        // i2c_txd_data_i[0]  FIXME TODO force_stop and restart (not need, bad idea, I2C master must send first (address) byte as a minimum)
                        // i2c_txd_data_i[1]  FIXME TODO force_acknack_enable (otherwise use setting)
                        // i2c_txd_data_i[2]  FIXME TODO force_acknack_value (requires force_acknack_enable==1 otherwise ignored)
                        // i2c_txd_data_i[3]  FIXME TODO send_stop (after this byte)
                        //i2c_txd_ready_o <= 1'b1;      // strobe as we load  FIXME move to strobe FIFO after recvd
                        direction <= i2c_txd_data_i[8];
                        r_sda_oe <= 1'b0;
                        fsm_next_state <= ST_RECV_BIT;
                    end else if (send_stop) begin
                        r_timer_reset_o <= 1'b1;  // reset for next state
                        fsm_next_state <= ST_PREPARE_STOP;
                    end if (!tick_scllow_i) begin       // TIMER !tick_scllow_i overhang ? before release
                        /* nop */
                    end else begin
`ifdef SIM
                        r_scl_o <= HIGHX;
`endif
                        r_sda_o <= HIGHX;
                        r_scl_oe <= 1'b0;
                        r_sda_oe <= 1'b0;
//                        need_start <= 1'b1; // FIXME this is not true, we are midtransaction
                        // trying to give synthesis degree of freedom
                        data[7:0]  <= 8'bxxxxxxxx;
`ifndef SYNTHESIS_OPENLANE
                        assert(!has_moredata);
`endif
///                        bit_count[2:0] <= 3'bxxx;
                        // FIXME check this scl_idle_monitor_arm_o
                        scl_idle_monitor_arm_o <= 1'b1; // we're mid transaction releasing line
                        fsm_next_state <= ST_IDLE_NEED_RESTART;
                    end
                end
                ST_WAIT_LINE_IDLE: begin        // waiting to confirm SCL is idle
                    if (need_start) begin
`ifndef SYNTHESIS_OPENLANE
                        assert(!r_scl_oe);
                        assert(isvalid1(r_scl_o));
                        assert(!r_sda_oe);
                        assert(isvalid1(r_sda_o));
                        assert(need_start);     // FIXME to remove
`endif
                        if (tick_idlescl_i) begin // read SCL check has been HI for timer
                            r_timer_reset_o <= 1'b1;        // reset for next state
                            r_scl_oe <= 1'b1;   // time to drive (r_scl_o==1)
                            r_scl_o  <= 1'b1;   // mimic pull-up
                            r_sda_oe <= 1'b1;   // START condition SDA negedge
                            r_sda_o  <= 1'b1;   // mimic pull-up
                            data[7]  <= 1'b0;   // will become SDA FALL for START condx
                            reg_ctrl[CTRL2_START] <= 1'b0;      // send_start
                            fsm_next_state <= ST_PREWAIT;
                        end
                    end else begin
                        r_scl_oe <= 1'b1;       // replace HIGHX
                        r_scl_o  <= 1'b1;       // mimic pull-up
                        r_sda_oe <= 1'b1;       // replace HIGHX
                        r_sda_o  <= 1'b1;       // mimic pull-up
                        if (scl_idle_monitor_strobe_i) begin    // FIXME scl_idle_monitor_strobe
                            stb_i2c_error_io_o <= 1'b1;
                            fsm_next_state <= ST_RESET;
                        end else begin
                            if (tick_edgewait_i) begin
                                r_timer_reset_o <= 1'b1;
                                fsm_next_state <= ST_IDLE;
                            end
                        end
                    end
                end
                ST_PREWAIT: begin       // wait HOLD:SCL / SETUP:SDA to prevent false START situation
`ifndef SYNTHESIS_OPENLANE
`ifdef SIM
                    assert(on_entry == tick_first_i);   // check timer was reset on_entry
`endif
                    assert(r_scl_oe == 1'b1);
// FIXME this breaks since bit_count/has_moredata change
//                    assert(r_scl_o  == !has_moredata || (r_scl_o && has_acknack));    // matches HIGH for START, but LOW for DATABIT
                    assert(r_sda_oe == 1'b1);
                    assert(isvalid1(r_sda_o));  // any state on entry
`endif
                    if (need_start) begin
                        i2c_clock_stretch           <= 1'b0;
                        i2c_clock_stretch_acknack   <= 1'b0;
                        r_sda_o <= 1'b0;        // START condx
                        r_timer_reset_o <= 1'b1;    // reset for next state
                        fsm_next_state <= ST_WAIT_SCL_HIGH;
                    end if (tick_prewait_i && direction == DIR_TXD /*&& scl_i == !has_moredata*/) begin // FIXME uncomment
`ifndef SYNTHESIS_OPENLANE
                        assert(isvalid1(data[7]));
`endif
                        // no timer reset here to run on
                        r_sda_o <= data[7];
                        data[7:0] <= {data[6:0], 1'bx}; // {data[6:0], data[7]} // MSB first
                        bit_count <= bit_count - 4'd1;
                        // NO TIMER RESET must run on for correct SCL_LOW period
                        fsm_next_state <= ST_WAIT_SCL_LOW;
`ifdef unused
                    end if (tick_prewait_i) begin       // abort due to SCL not in expected state
                        // this indicates another controller is present
                        stb_i2c_error_generic_o <= 1'b1;
                        fsm_next_state <= ST_RESET;
`endif
                    end
                end
                ST_WAIT_SCL_LOW: begin          // wait rest of SCL low period
`ifndef SYNTHESIS_OPENLANE
`ifdef SIM
                    // ST_PREWAIT does not timer_reset (it is designed to run on for correct SCL_LOW period)
                    assert(on_entry == tick_first_i || fsm_last_state == ST_PREWAIT);   // check timer was reset on_entry
`endif
                    assert(r_scl_oe == 1'b1);
                    assert(r_scl_o  == 1'b0);   // LOW
                    assert(r_sda_oe == ~direction);   // 1=TX 0=RX
                    assert(isvalid1(r_sda_o));  // any state
`endif
                    if (tick_scllow_i) begin
                        r_scl_o <= 1'b1;        // RISE
                        r_timer_reset_o <= 1'b1;    // reset for next state
                        fsm_next_state <= ST_WAIT_SCL_HIGH;
                    end
                end
                ST_WAIT_SCL_HIGH: begin	        // wait rest of SCL high period
`ifndef SYNTHESIS_OPENLANE
`ifdef SIM
                    assert(on_entry == tick_first_i);   // check timer was reset on_entry
`endif
                    assert(r_scl_oe == 1'b1);
                    assert(r_scl_o  == 1'b1);   // HIGH after START or BITS
                    assert(r_sda_oe == ~direction);   // 1=TX 0=RX
                    assert(isvalid1(r_sda_o));  // any state
`endif
                    // FIXME The I2C specification allows an infinite timeout here but we are currently limited to tick_overflow_i
                    //  tick_sclhigh_i is latching so we could inhibit tick_overflow_i processing and we should confirm to spec ?
                    if (tick_sclhigh_i) begin
                        if (direction == DIR_TXD) begin
                            if (scl_clock_stretch) begin    // CLOCK STRETCH (we need to see the scl_i==1 to continue)
                                i2c_acknack_edge <= 1'b0;   // RESET for new transaction
                                need_start <= 1'b0;
                                r_timer_reset_o <= 1'b1;        // reset for next state
                                fsm_next_state <= ST_IDLE;
                            end else begin
                                i2c_clock_stretch <= 1'b1;
`ifndef SYNTHESIS_OPENLANE
                                // Hmm the verilator used in flow, or flow itself does not handle $info FIXME remove ifndef
//                                $info("I2C ST_WAIT_SCL_HIGH CLOCK_STRETCH tick_sclhigh_i=%b scl_i=%b sda_i=%b", tick_sclhigh_i, scl_i, sda_i);
`endif
                            end
                        end if (direction == DIR_RXD) begin
                            // FIXME this is the wrong sampel point, for RXD it is end of SCL_HIGH
                            data[7] <= sda_i;
                            data[6:0] <= {data[5:0], data[7]};
                            bit_count <= bit_count - 4'd1;
                            fsm_next_state <= ST_IDLE;
                        end
                    end
                end
                ST_SEND_BIT: begin
`ifndef SYNTHESIS_OPENLANE
                    assert(r_scl_oe);
                    assert(r_scl_o);            // HIGH
                    assert(r_sda_oe);
                    assert(isvalid1(r_sda_o));
                    //if ($isunknown(r_sda_o)) $info("ST_SEND_BIT r_sda_o=%b", r_sda_o); // DEBUG
                    assert(isvalid4(bit_count) && (/*bit_count >= 4'd0 &&*/ bit_count <= 4'd7)); // bit_count unsigned so >=0 always true
`endif
                    r_scl_o <= 1'b0;            // FALL
`ifndef SYNTHESIS_OPENLANE
                    assert(isvalid1(data[7]));  // the bit we're about to emit
`endif
                    r_timer_reset_o <= 1'b1;        // reset for next state
                    fsm_next_state <= ST_PREWAIT;       // data[7] is the next bit to send
                end
                ST_RECV_ACKNACK: begin
`ifndef SYNTHESIS_OPENLANE
`ifdef SIM
                    assert(on_entry == tick_first_i);   // check timer was reset on_entry
`endif
                    assert(r_scl_oe == 1'b1);
                    assert(r_scl_o  == 1'b0);   // LOW (so we can change SDA
//                    assert(r_sda_oe == 1'b1);
///                    assert(isvalid1(r_sda_o)); // we set HIGHX
                    assert(bit_count == 4'b1111);
                    //if ($isunknown(r_sda_o)) $info("ST_SEND_BIT r_sda_o=%b", r_sda_o); // DEBUG
`endif
                    if (tick_edgewait_i) begin
`ifndef SYNTHESIS_OPENLANE
                        assert(isvalid1(r_sda_o));   // only works as tick_edgewait_i is a single strobe not repeated
`endif
`ifndef SYNTHESIS_OPENLANE
                        // Hmm the verilator used in flow, or flow itself does not handle $info FIXME remove ifndef
//                        if ($isunknown(r_sda_o)) $info("ST_RECV_ACKNACK r_sda_o=%b", r_sda_o); // DEBUG
`endif
                        r_sda_o  <= HIGHX;      // HIGH to match pull-up (before sda_oe=0)
                    end
                    if (tick_prewait_i) begin
`ifndef SYNTHESIS_OPENLANE
                        assert(r_sda_o === HIGHX);    // only works as tick_prewait_i is a single strobe not repeated
                        assert(r_sda_oe);       // only works as tick_prewait_i is a single strobe not repeated
`endif
                        // This also keeps the SDA handover away from the SCL RISE edge so as not to be interpreted as a START/STOP condx
                        r_sda_oe <= 1'b0;       // delayed until after SDA=1 RISE so ASIC drives it
                    end
                    if (tick_scllow_i) begin
`ifndef SYNTHESIS_OPENLANE
                        assert(r_sda_o === HIGHX);
                        assert(!r_sda_oe);
`endif
                        r_scl_o  <= HIGHX;      // RISE
                        r_timer_reset_o <= 1'b1;    // reset for next state
                        fsm_next_state <= ST_WAIT_ACKNACK;
                    end
                end
                ST_WAIT_ACKNACK: begin  // copy ST_WAIT_SCL_HIGH
`ifndef SYNTHESIS_OPENLANE
`ifdef SIM
                    assert(on_entry == tick_first_i);   // check timer was reset on_entry
`endif
                    assert(isvalid1(r_scl_oe)); // is modified while in this state for CLOCK_STRETCH support
                    assert(r_scl_o === HIGHX);  // HIGH after RECV_ACKNACK to drive edge
                    assert(r_sda_oe == 1'b0);   // LOW
                    assert(r_sda_o === HIGHX);  // HIGH due to pull-up matching before disconnect (SDA_OE=0)
                    assert(bit_count == 4'b1111);
`endif
                    // This small delay is to ensure we drive the SCL high, not reply on pull-up but now we hand off to pull-up
                    if (tick_edgewait_i) begin
`ifndef SYNTHESIS_OPENLANE
                        assert(r_scl_oe);       // only works as tick_edgewait_i is a single strobe not repeated
`endif
                        r_scl_oe <= 1'b0;       // pull-up now takes over
                    end
                    // A clock stretch, would observe the scl_i low here, as the external pull-up
                    //  is being opposed by a device stretching the SCL line, which will delay the
                    //  ACKNACK sample made below. 
                    // The tick_overflow_i will handle maximum timeout.
                    if (tick_sclhigh_i) begin // tick_sclhigh_i must latch
`ifndef SYNTHESIS_OPENLANE
                        assert(r_scl_o === HIGHX);
                        assert(!r_scl_oe);
                        assert(r_sda_o === HIGHX);
                        assert(!r_sda_oe);
`endif
                        if (scl_clock_stretch_acknack) begin
                            if (sda_i)
                                i2c_acknack_edge <= sda_i;  // only stores NACKs
                            i2c_acknack <= sda_i;           // FIXME emit/make-available this as data for CPU
                            i2c_acknack_valid <= 1'b1;      // FIXME process latching NACK
                            r_timer_reset_o <= 1'b1;    // reset for next state
                            //r_scl_oe <= 1'b1;         // no need to delay this
                            //r_sda_oe <= 1'b1;         // FIXME maybe this should be delayed ?
                            //bit_count <= 4'b0111;     // has_moredata=0 has_acknack=0
                            bit_count[3] <= 1'b0;
                            if (sda_i) begin
                                fsm_next_state <= ST_PREPARE_STOP; // auto-STOP
                            end else begin
                                fsm_next_state <= ST_IDLE_NEED_RESTART;
                            end
                        end else begin
                            i2c_clock_stretch_acknack <= 1'b1;
`ifndef SYNTHESIS_OPENLANE
                            // Hmm the verilator used in flow, or flow itself does not handle $info FIXME remove ifndef
//                            $info("I2C CLOCK_STRETCH tick_sclhigh_i=%b scl_i=%b sda_i=%b", tick_sclhigh_i, scl_i, sda_i);
`endif
                        end
                    end
                end
                ST_RECV_BIT: begin // FIXME not tested
                    // only start if FIFO can take a byte ?  data input with direction bit with ACKNACK bit ?
`ifndef SYNTHESIS_OPENLANE
                    assert(r_scl_oe);
                    assert(r_scl_o);            // HIGH
                    assert(!r_sda_oe);
                    assert(isvalid1(r_sda_o));
                    //if ($isunknown(r_sda_o)) $info("ST_RECV_BIT r_sda_o=%b", r_sda_o); // DEBUG
                    assert(isvalid4(bit_count) && (/*bit_count >= 4'd0 &&*/ bit_count <= 4'd7)); // bit_count unsigned so >=0 always true
`endif
                    r_scl_o <= 1'b0;            // FALL
                    r_timer_reset_o <= 1'b1;      // reset for next state
                    fsm_next_state <= ST_WAIT_SCL_LOW;
                end
                ST_SEND_ACKNACK: begin
                    // This is based on current setting
                    //  ALWAYS ACK
                    //  CAN NACK ON RXE_OVERFLOW
                    //  HALT ON RXE_OVERFLOW (with interrupt)
                end
                ST_PREPARE_STOP: begin  // can enter this state at any time
`ifndef SYNTHESIS_OPENLANE
`ifdef SIM
                    assert(on_entry == tick_first_i);   // check timer was reset on_entry
`endif
`endif
                    if (r_sda_o || !r_sda_oe) begin
                        if (r_scl_o || !r_scl_oe) begin
                            r_scl_o  <= 1'b0;           // FALL
                            r_scl_oe <= 1'b1;
                        end
                        if (tick_prewait_i) begin
                            r_sda_o  <= 1'b0;           // SDA FALL while SCL low is not a STOP condx
                            r_sda_oe <= 1'b1;
                            r_timer_reset_o <= 1'b1;  // reset for iteration of next state
                            fsm_next_state <= ST_SEND_STOP;
                        end
                    end else begin
`ifndef SYNTHESIS_OPENLANE
                        assert(!r_scl_o);
                        assert(r_scl_oe);
`endif
                        r_timer_reset_o <= 1'b1;  // reset for iteration of next state
                        fsm_next_state <= ST_SEND_STOP;
                    end
                end
                ST_SEND_STOP: begin     // can enter this state at any time
                    // FIXME do we need to check entry condition and do something special ?
                    //  for example of an OE==0 || (SDA==1 && SCL==1)
                    //  if SDA==1 || SDA_OE==0 (implicitly making SDA==1)
                    //     set SDA_OE=1
                    //     if SCL==1 || SCL_OE==0
                    //        set SCL_OE=1 SCL=0
                    //        wait
                    //     SDA=0 (prepare for STOP)
                    //     wait
                    //  set SCL=1
                    //  goto ST_SEND_STOP_WAIT
`ifndef SYNTHESIS_OPENLANE
`ifdef SIM
                    assert(on_entry == tick_first_i);   // check timer was reset on_entry
`endif
`endif
                    r_scl_oe <= 1'b1;
                    r_scl_o  <= 1'b1;
                    r_sda_oe <= 1'b1;
                    r_sda_o  <= 1'b0;
                    if (tick_edgewait_i) begin  // is this delay needed ?
                        r_sda_o  <= 1'b0;
                        r_timer_reset_o <= 1'b1;      // reset for next state
                        fsm_next_state <= ST_SEND_STOP_WAIT;
                    end
                end
                ST_SEND_STOP_WAIT: begin        // waiting SETUP:STOP
`ifndef SYNTHESIS_OPENLANE
`ifdef SIM
                    assert(on_entry == tick_first_i);   // check timer was reset on_entry
`endif
                    assert(r_scl_oe == 1'b1);
                    assert(r_scl_o  == 1'b1);
                    assert(r_sda_oe == 1'b1);
                    assert(r_sda_o  == 1'b0);
`endif
                    if (tick_sclhigh_i) begin   // SETUP:STOP
                        r_sda_o <= 1'b1;        // STOP condition SDA rise
                        need_start <= 1'b1;     // FIXME ST_WAIT_SCL_HIGH will FALL this! need to inhibit that
                        reg_ctrl[CTRL3_STOP]       <= 1'b0; // send_stop
                        reg_ctrl[CTRL4_FORCE_STOP] <= 1'b0; // force_stop
                        i2c_condx_start_stop       <= 1'b1;
                        r_timer_reset_o <= 1'b1;    // reset for next state
                        // ideally want ST_WAIT_SCL_HIGH but with tick_scllow_i timer
                        fsm_next_state <= ST_WAIT_SCL_HIGH;     // HOLD:STOP before ST_RESET ?
                    end
                end
            endcase

            // highest priority precedence
            if (tick_overflow_i) begin
                stb_i2c_error_timeout_o <= 1'b1; // strobe latchable error condx
                fsm_next_state <= ST_RESET;
            end

`ifdef SIM
            // fsm_last_state only exists to validate with assert() usage
            if (fsm_last_state !== w_fsm_next_state) begin
                fsm_last_state <= fsm_state;
            end
`endif

`ifdef SIM
            on_entry <= fsm_state !== w_fsm_next_state; // VALIDATION
`endif
        end // fsm_run
    end

    localparam FSM_ST_IDLE  = 2'b00;
    localparam FSM_ST_RECV  = 2'b01;
    localparam FSM_ST_SEND  = 2'b10;
    localparam FSM_ST_ERROR = 2'b11;

    reg [1:0] w_fsm_state_2bit;
    always @(*) begin
        if(direction == DIR_RXD)
            w_fsm_state_2bit = FSM_ST_RECV;
        else
            w_fsm_state_2bit = FSM_ST_SEND;

        case (fsm_state)
            ST_RESET :              w_fsm_state_2bit = FSM_ST_IDLE;
            ST_IDLE_NEED_RESTART :  w_fsm_state_2bit = FSM_ST_IDLE;
            //ST_IDLE :               w_fsm_state_2bit = FSM_ST_IDLE;
            ST_WAIT_LINE_IDLE :     w_fsm_state_2bit = FSM_ST_IDLE;
            //ST_PREWAIT :
            //ST_WAIT_SCL_LOW :
            //ST_WAIT_SCL_HIGH :
            //ST_SEND_BIT :
            //ST_RECV_ACKNACK :
            //ST_WAIT_ACKNACK :
            //ST_RECV_BIT :
            //ST_SEND_ACKNACK :
            ST_PREPARE_STOP :       w_fsm_state_2bit = FSM_ST_IDLE;
            ST_SEND_STOP :          w_fsm_state_2bit = FSM_ST_IDLE;
            ST_SEND_STOP_WAIT :     w_fsm_state_2bit = FSM_ST_IDLE;
            ST_BEFORE_RUN :         w_fsm_state_2bit = FSM_ST_IDLE;
            default : begin
                /* nop */
            end
        endcase

        // Highest precedence
        if (stb_i2c_error_timeout_o || stb_i2c_error_io_o || stb_i2c_error_generic_o) begin
            w_fsm_state_2bit = FSM_ST_ERROR;
        end
    end

    // REG_CTRL view
    assign reg_ctrl_o = {
        // Can we sequeeze this into 4 states ?
        //  IDLE, SENDING, RECVING, ...
        // We have need_start to indicate inside a transaction
        w_fsm_state_2bit,       // debugging ?
        i2c_clock_stretch_acknack,
        i2c_clock_stretch,

        i2c_acknack_valid,
        i2c_acknack,
        {1{1'b0}},
        force_stop,
        send_stop,
        send_start,
        need_start,
        fsm_run
    };

    assign fsm_run_o    = reg_ctrl[0];  // FSM_RUN
    assign fsm_state_o  = fsm_state;

    assign scl_o        = r_scl_o;
    assign scl_oe_o     = r_scl_oe;
    assign sda_o        = r_sda_o;
    assign sda_oe_o     = r_sda_oe;

`ifdef SYNTHESIS_OPENLANE
    initial begin
        // This errored as expected in openlan2.2.9 yosys0.46 flow to prove ifdef is being selected
        //$warning("SYNTHESIS_OPENLANE is defined");
    end
`endif

endmodule
