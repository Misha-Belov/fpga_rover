`timescale 1ns / 1ps
`default_nettype none

module rover (
    input  wire       CLK_50M,
    input  wire       RST_N,
    inout  wire       I2C_SCL,
    inout  wire       I2C_SDA,
    output wire [7:0] SEG,
    output wire [3:0] DIG,
    output wire       LED1,
    output wire       LED2,
    output wire       LED3,
    output wire       LED4
);
    parameter integer CLK_HZ                     = 50000000;
    parameter integer I2C_HZ                     = 50000;
    parameter integer TOF_SENSOR_KIND            = 2;       // 2 = VL53L1X
    parameter integer PWRUP_DELAY_MS             = 100;
    parameter integer INTER_OP_DELAY_MS          = 5;
    parameter integer MOTOR_SPEED_CMD            = 8'd50;
    parameter integer SENSOR_REINIT_AFTER_ERRORS = 4;
    parameter integer MOTOR_WAIT_TIMEOUT_MS      = 40;
    parameter integer TOF_INIT_TIMEOUT_MS        = 300;
    parameter integer TOF_SAMP_TIMEOUT_MS        = 150;
    parameter integer TOF_FAR_DISTANCE_MM        = 4000;

    function integer clog2;
        input integer value;
        integer i;
        begin
            value = value - 1;
            for (i = 0; value > 0; i = i + 1)
                value = value >> 1;
            clog2 = (i < 1) ? 1 : i;
        end
    endfunction

    localparam integer PWRUP_DELAY_CLKS        = (CLK_HZ / 1000) * PWRUP_DELAY_MS;
    localparam integer INTER_OP_DELAY_CLKS     = (CLK_HZ / 1000) * INTER_OP_DELAY_MS;
    localparam integer MOTOR_WAIT_TIMEOUT_CLKS = (CLK_HZ / 1000) * MOTOR_WAIT_TIMEOUT_MS;
    localparam integer TOF_INIT_TIMEOUT_CLKS   = (CLK_HZ / 1000) * TOF_INIT_TIMEOUT_MS;
    localparam integer TOF_SAMP_TIMEOUT_CLKS   = (CLK_HZ / 1000) * TOF_SAMP_TIMEOUT_MS;

    localparam integer PWRUP_CNT_W    = clog2(PWRUP_DELAY_CLKS + 1);
    localparam integer PAUSE_CNT_W    = clog2(INTER_OP_DELAY_CLKS + 1);
    localparam integer MOTOR_TMO_W    = clog2(MOTOR_WAIT_TIMEOUT_CLKS + 1);
    localparam integer TOF_INIT_TMO_W = clog2(TOF_INIT_TIMEOUT_CLKS + 1);
    localparam integer TOF_SAMP_TMO_W = clog2(TOF_SAMP_TIMEOUT_CLKS + 1);

    localparam [3:0]
        S_PWRUP_WAIT     = 4'd0,
        S_INTER_OP_WAIT  = 4'd1,
        S_MOTOR_START    = 4'd2,
        S_MOTOR_WAIT     = 4'd3,
        S_TOF_INIT_START = 4'd4,
        S_TOF_INIT_WAIT  = 4'd5,
        S_TOF_SAMP_START = 4'd6,
        S_TOF_SAMP_WAIT  = 4'd7;

    reg [3:0] state;
    reg [3:0] state_after_pause;
    reg [PWRUP_CNT_W-1:0]    pwrup_cnt;
    reg [PAUSE_CNT_W-1:0]    pause_cnt;
    reg [MOTOR_TMO_W-1:0]    motor_wait_cnt;
    reg [TOF_INIT_TMO_W-1:0] tof_init_wait_cnt;
    reg [TOF_SAMP_TMO_W-1:0] tof_samp_wait_cnt;

    reg [15:0] display_value;
    reg [24:0] heartbeat_cnt;

    reg tof_init_start;
    reg tof_sample_start;
    reg motor_start;

    reg sensor_init_done;
    reg sensor_error_latched;
    reg motor_error_latched;
    reg sensor_valid_seen;
    reg [7:0] sensor_sample_error_count;

    wire heartbeat;
    wire display_show_err;

    wire [7:0] motor0_speed;
    wire [7:0] motor1_speed;
    wire [7:0] motor2_speed;
    wire [7:0] motor3_speed;

    // Вынесенная логика движения. clk и rst_n заведены внутрь,
    // но сам алгоритм движения оставлен таким же, как в последней рабочей версии.
    distance_motion_ctrl #(
        .DIST_STOP_MM(200),
        .MOTOR_SPEED_CMD(MOTOR_SPEED_CMD)
    ) u_distance_motion_ctrl (
        .clk(CLK_50M),
        .rst_n(RST_N),
        .sensor_valid(sensor_valid_seen),
        .distance_mm(display_value),
        .motor0_speed(motor0_speed),
        .motor1_speed(motor1_speed),
        .motor2_speed(motor2_speed),
        .motor3_speed(motor3_speed)
    );

    // I2C physical lines
    wire scl_drive_low;
    wire sda_drive_low;
    assign I2C_SCL = scl_drive_low ? 1'b0 : 1'bz;
    assign I2C_SDA = sda_drive_low ? 1'b0 : 1'bz;

    // Shared I2C master
    wire        txn_core_start;
    wire        txn_core_is_read;
    wire [6:0]  txn_core_dev_addr;
    wire [15:0] txn_core_reg_addr;
    wire        txn_core_reg_addr_16b;
    wire [7:0]  txn_core_wr_data;
    wire [1:0]  txn_core_rd_len;
    wire [15:0] txn_core_rd_data;
    wire        txn_core_busy;
    wire        txn_core_done;
    wire        txn_core_error;
    wire        txn_core_nack;

    i2c_master #(
        .CLK_HZ(CLK_HZ),
        .I2C_HZ(I2C_HZ)
    ) u_i2c_master (
        .clk(CLK_50M),
        .rst_n(RST_N),
        .start(txn_core_start),
        .is_read(txn_core_is_read),
        .dev_addr(txn_core_dev_addr),
        .reg_addr(txn_core_reg_addr),
        .reg_addr_16b(txn_core_reg_addr_16b),
        .wr_data(txn_core_wr_data),
        .rd_len(txn_core_rd_len),
        .rd_data(txn_core_rd_data),
        .busy(txn_core_busy),
        .done(txn_core_done),
        .error(txn_core_error),
        .nack(txn_core_nack),
        .scl_drive_low(scl_drive_low),
        .sda_drive_low(sda_drive_low),
        .scl_in(I2C_SCL),
        .sda_in(I2C_SDA)
    );

    // TOF controller
    wire [15:0] tof_distance_mm;
    wire        tof_distance_valid;
    wire        tof_busy;
    wire        tof_done;
    wire        tof_error;
    wire        tof_nack;

    wire        tof_txn_req_valid;
    wire        tof_txn_req_ready;
    wire        tof_txn_req_is_read;
    wire [6:0]  tof_txn_req_dev_addr;
    wire [15:0] tof_txn_req_reg_addr;
    wire        tof_txn_req_reg_addr_16b;
    wire [7:0]  tof_txn_req_wr_data;
    wire [1:0]  tof_txn_req_rd_len;

    wire        tof_txn_rsp_done;
    wire        tof_txn_rsp_error;
    wire        tof_txn_rsp_nack;
    wire [15:0] tof_txn_rsp_rd_data;

    tof_ctrl #(
        .SENSOR_KIND(TOF_SENSOR_KIND),
        .CLK_HZ(CLK_HZ)
    ) u_tof_ctrl (
        .clk(CLK_50M),
        .rst_n(RST_N),
        .init_start(tof_init_start),
        .sample_start(tof_sample_start),
        .distance_mm(tof_distance_mm),
        .distance_valid(tof_distance_valid),
        .busy(tof_busy),
        .done(tof_done),
        .error(tof_error),
        .nack(tof_nack),
        .txn_req_valid(tof_txn_req_valid),
        .txn_req_ready(tof_txn_req_ready),
        .txn_req_is_read(tof_txn_req_is_read),
        .txn_req_dev_addr(tof_txn_req_dev_addr),
        .txn_req_reg_addr(tof_txn_req_reg_addr),
        .txn_req_reg_addr_16b(tof_txn_req_reg_addr_16b),
        .txn_req_wr_data(tof_txn_req_wr_data),
        .txn_req_rd_len(tof_txn_req_rd_len),
        .txn_rsp_done(tof_txn_rsp_done),
        .txn_rsp_error(tof_txn_rsp_error),
        .txn_rsp_nack(tof_txn_rsp_nack),
        .txn_rsp_rd_data(tof_txn_rsp_rd_data)
    );

    // Motor controller
    wire        motor_busy;
    wire        motor_done;
    wire        motor_error;
    wire        motor_nack;
    wire [2:0]  motor_step_dbg;

    wire        motor_txn_req_valid;
    wire        motor_txn_req_ready;
    wire        motor_txn_req_is_read;
    wire [6:0]  motor_txn_req_dev_addr;
    wire [15:0] motor_txn_req_reg_addr;
    wire        motor_txn_req_reg_addr_16b;
    wire [7:0]  motor_txn_req_wr_data;
    wire [1:0]  motor_txn_req_rd_len;

    wire        motor_txn_rsp_done;
    wire        motor_txn_rsp_error;
    wire        motor_txn_rsp_nack;
    wire [15:0] motor_txn_rsp_rd_data;

    roverc_motor_ctrl u_roverc_motor_ctrl (
        .clk(CLK_50M),
        .rst_n(RST_N),
        .start(motor_start),
        .motor0_speed(motor0_speed),
        .motor1_speed(motor1_speed),
        .motor2_speed(motor2_speed),
        .motor3_speed(motor3_speed),
        .busy(motor_busy),
        .done(motor_done),
        .error(motor_error),
        .nack(motor_nack),
        .step_dbg(motor_step_dbg),
        .txn_req_valid(motor_txn_req_valid),
        .txn_req_ready(motor_txn_req_ready),
        .txn_req_is_read(motor_txn_req_is_read),
        .txn_req_dev_addr(motor_txn_req_dev_addr),
        .txn_req_reg_addr(motor_txn_req_reg_addr),
        .txn_req_reg_addr_16b(motor_txn_req_reg_addr_16b),
        .txn_req_wr_data(motor_txn_req_wr_data),
        .txn_req_rd_len(motor_txn_req_rd_len),
        .txn_rsp_done(motor_txn_rsp_done),
        .txn_rsp_error(motor_txn_rsp_error),
        .txn_rsp_nack(motor_txn_rsp_nack),
        .txn_rsp_rd_data(motor_txn_rsp_rd_data)
    );

    // Single-owner I2C mux
    wire owner_tof;
    wire owner_motor;

    assign owner_tof   = (state == S_TOF_INIT_WAIT) || (state == S_TOF_SAMP_WAIT);
    assign owner_motor = (state == S_MOTOR_WAIT);

    assign tof_txn_req_ready      = owner_tof   ? (~txn_core_busy) : 1'b0;
    assign motor_txn_req_ready    = owner_motor ? (~txn_core_busy) : 1'b0;

    assign txn_core_start         = owner_tof   ? (tof_txn_req_valid   & tof_txn_req_ready) :
                                    owner_motor ? (motor_txn_req_valid & motor_txn_req_ready) :
                                                  1'b0;

    assign txn_core_is_read       = owner_tof   ? tof_txn_req_is_read       : motor_txn_req_is_read;
    assign txn_core_dev_addr      = owner_tof   ? tof_txn_req_dev_addr      : motor_txn_req_dev_addr;
    assign txn_core_reg_addr      = owner_tof   ? tof_txn_req_reg_addr      : motor_txn_req_reg_addr;
    assign txn_core_reg_addr_16b  = owner_tof   ? tof_txn_req_reg_addr_16b  : motor_txn_req_reg_addr_16b;
    assign txn_core_wr_data       = owner_tof   ? tof_txn_req_wr_data       : motor_txn_req_wr_data;
    assign txn_core_rd_len        = owner_tof   ? tof_txn_req_rd_len        : motor_txn_req_rd_len;

    assign tof_txn_rsp_done       = owner_tof   ? txn_core_done    : 1'b0;
    assign tof_txn_rsp_error      = owner_tof   ? txn_core_error   : 1'b0;
    assign tof_txn_rsp_nack       = owner_tof   ? txn_core_nack    : 1'b0;
    assign tof_txn_rsp_rd_data    = owner_tof   ? txn_core_rd_data : 16'h0000;

    assign motor_txn_rsp_done     = owner_motor ? txn_core_done    : 1'b0;
    assign motor_txn_rsp_error    = owner_motor ? txn_core_error   : 1'b0;
    assign motor_txn_rsp_nack     = owner_motor ? txn_core_nack    : 1'b0;
    assign motor_txn_rsp_rd_data  = owner_motor ? txn_core_rd_data : 16'h0000;

    assign display_show_err = motor_error_latched | (sensor_error_latched & ~sensor_valid_seen);

    seg7_display u_seg7_display (
        .clk(CLK_50M),
        .rst_n(RST_N),
        .value(display_value),
        .show_err(display_show_err),
        .seg(SEG),
        .dig(DIG)
    );

    always @(posedge CLK_50M or negedge RST_N) begin
        if (!RST_N) begin
            state                     <= S_PWRUP_WAIT;
            state_after_pause         <= S_MOTOR_START;
            pwrup_cnt                 <= {PWRUP_CNT_W{1'b0}};
            pause_cnt                 <= {PAUSE_CNT_W{1'b0}};
            motor_wait_cnt            <= {MOTOR_TMO_W{1'b0}};
            tof_init_wait_cnt         <= {TOF_INIT_TMO_W{1'b0}};
            tof_samp_wait_cnt         <= {TOF_SAMP_TMO_W{1'b0}};
            display_value             <= 16'd0;
            heartbeat_cnt             <= 25'd0;
            tof_init_start            <= 1'b0;
            tof_sample_start          <= 1'b0;
            motor_start               <= 1'b0;
            sensor_init_done          <= 1'b0;
            sensor_error_latched      <= 1'b0;
            motor_error_latched       <= 1'b0;
            sensor_valid_seen         <= 1'b0;
            sensor_sample_error_count <= 8'd0;
        end else begin
            heartbeat_cnt    <= heartbeat_cnt + 25'd1;
            tof_init_start   <= 1'b0;
            tof_sample_start <= 1'b0;
            motor_start      <= 1'b0;

            if (tof_distance_valid && (tof_distance_mm != 16'd0)) begin
                display_value             <= tof_distance_mm;
                sensor_valid_seen         <= 1'b1;
                sensor_error_latched      <= 1'b0;
                sensor_sample_error_count <= 8'd0;
            end

            case (state)
                S_PWRUP_WAIT: begin
                    if (pwrup_cnt >= PWRUP_DELAY_CLKS - 1) begin
                        pwrup_cnt <= {PWRUP_CNT_W{1'b0}};
                        state     <= S_MOTOR_START;
                    end else begin
                        pwrup_cnt <= pwrup_cnt + 1'b1;
                    end
                end

                S_INTER_OP_WAIT: begin
                    if (pause_cnt >= INTER_OP_DELAY_CLKS - 1) begin
                        pause_cnt <= {PAUSE_CNT_W{1'b0}};
                        state     <= state_after_pause;
                    end else begin
                        pause_cnt <= pause_cnt + 1'b1;
                    end
                end

                S_MOTOR_START: begin
                    motor_wait_cnt <= {MOTOR_TMO_W{1'b0}};
                    if (!motor_busy && !txn_core_busy) begin
                        motor_start <= 1'b1;
                        state       <= S_MOTOR_WAIT;
                    end
                end

                S_MOTOR_WAIT: begin
                    if (motor_done) begin
                        motor_error_latched <= motor_error | motor_nack;
                        pause_cnt           <= {PAUSE_CNT_W{1'b0}};
                        state_after_pause   <= sensor_init_done ? S_TOF_SAMP_START : S_TOF_INIT_START;
                        state               <= S_INTER_OP_WAIT;
                    end else if (motor_wait_cnt >= MOTOR_WAIT_TIMEOUT_CLKS - 1) begin
                        motor_error_latched <= 1'b1;
                        pause_cnt           <= {PAUSE_CNT_W{1'b0}};
                        state_after_pause   <= sensor_init_done ? S_TOF_SAMP_START : S_TOF_INIT_START;
                        state               <= S_INTER_OP_WAIT;
                    end else begin
                        motor_wait_cnt <= motor_wait_cnt + 1'b1;
                    end
                end

                S_TOF_INIT_START: begin
                    tof_init_wait_cnt <= {TOF_INIT_TMO_W{1'b0}};
                    if (!tof_busy && !txn_core_busy) begin
                        tof_init_start <= 1'b1;
                        state          <= S_TOF_INIT_WAIT;
                    end
                end

                S_TOF_INIT_WAIT: begin
                    if (tof_done) begin
                        if (tof_error || tof_nack) begin
                            sensor_error_latched      <= 1'b1;
                            sensor_init_done          <= 1'b0;
                            sensor_valid_seen         <= 1'b0;
                            sensor_sample_error_count <= 8'd0;
                        end else begin
                            sensor_error_latched      <= 1'b0;
                            sensor_init_done          <= 1'b1;
                            sensor_sample_error_count <= 8'd0;
                        end
                        pause_cnt         <= {PAUSE_CNT_W{1'b0}};
                        state_after_pause <= S_MOTOR_START;
                        state             <= S_INTER_OP_WAIT;
                    end else if (tof_init_wait_cnt >= TOF_INIT_TIMEOUT_CLKS - 1) begin
                        sensor_error_latched      <= 1'b1;
                        sensor_init_done          <= 1'b0;
                        sensor_valid_seen         <= 1'b0;
                        sensor_sample_error_count <= 8'd0;
                        pause_cnt                 <= {PAUSE_CNT_W{1'b0}};
                        state_after_pause         <= S_MOTOR_START;
                        state                     <= S_INTER_OP_WAIT;
                    end else begin
                        tof_init_wait_cnt <= tof_init_wait_cnt + 1'b1;
                    end
                end

                S_TOF_SAMP_START: begin
                    tof_samp_wait_cnt <= {TOF_SAMP_TMO_W{1'b0}};
                    if (!tof_busy && !txn_core_busy) begin
                        tof_sample_start <= 1'b1;
                        state            <= S_TOF_SAMP_WAIT;
                    end
                end

                S_TOF_SAMP_WAIT: begin
                    if (tof_done) begin
                        if (tof_error || tof_nack) begin
                            sensor_error_latched <= 1'b1;
                            sensor_valid_seen    <= 1'b0;
                            if (sensor_sample_error_count >= SENSOR_REINIT_AFTER_ERRORS - 1) begin
                                sensor_sample_error_count <= 8'd0;
                                sensor_init_done          <= 1'b0;
                            end else begin
                                sensor_sample_error_count <= sensor_sample_error_count + 8'd1;
                            end
                        end else if (tof_distance_valid && (tof_distance_mm != 16'd0)) begin
                            sensor_error_latched      <= 1'b0;
                            sensor_valid_seen         <= 1'b1;
                            sensor_sample_error_count <= 8'd0;
                        end else begin
                            // Успешный цикл измерения без пригодной дальности:
                            // объект слишком далеко, сцена пустая или датчик вернул
                            // "out of range". Это не считаем ошибкой датчика.
                            display_value             <= TOF_FAR_DISTANCE_MM[15:0];
                            sensor_error_latched      <= 1'b0;
                            sensor_valid_seen         <= 1'b1;
                            sensor_sample_error_count <= 8'd0;
                        end
                        pause_cnt         <= {PAUSE_CNT_W{1'b0}};
                        state_after_pause <= S_MOTOR_START;
                        state             <= S_INTER_OP_WAIT;
                    end else if (tof_samp_wait_cnt >= TOF_SAMP_TIMEOUT_CLKS - 1) begin
                        sensor_error_latched <= 1'b1;
                        sensor_valid_seen    <= 1'b0;
                        if (sensor_sample_error_count >= SENSOR_REINIT_AFTER_ERRORS - 1) begin
                            sensor_sample_error_count <= 8'd0;
                            sensor_init_done          <= 1'b0;
                        end else begin
                            sensor_sample_error_count <= sensor_sample_error_count + 8'd1;
                        end
                        pause_cnt         <= {PAUSE_CNT_W{1'b0}};
                        state_after_pause <= S_MOTOR_START;
                        state             <= S_INTER_OP_WAIT;
                    end else begin
                        tof_samp_wait_cnt <= tof_samp_wait_cnt + 1'b1;
                    end
                end

                default: begin
                    state <= S_PWRUP_WAIT;
                end
            endcase
        end
    end

    assign heartbeat = heartbeat_cnt[24];

    // active-low LEDs on board
    assign LED1 = ~heartbeat;
    assign LED2 = ~sensor_valid_seen;
    assign LED3 = ~sensor_error_latched;
    assign LED4 = ~motor_error_latched;

endmodule

`default_nettype wire