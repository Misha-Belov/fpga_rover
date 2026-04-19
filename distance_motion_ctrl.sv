`timescale 1ns / 1ps
`default_nettype none

module distance_motion_ctrl #(
    parameter integer CLK_HZ               = 50000000,
    parameter integer DIST_STOP_MM         = 100,
    parameter integer MOTOR_SPEED_CMD      = 8'd100,
    parameter integer SIDE_DELAY_MS        = 500,
    parameter integer SIDE_EXTRA_SCALE_PCT = 20
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sensor_valid,
    input  wire [15:0] distance_mm,
    output reg  [7:0]  motor0_speed,
    output reg  [7:0]  motor1_speed,
    output reg  [7:0]  motor2_speed,
    output reg  [7:0]  motor3_speed
);
    localparam integer SIDE_DELAY_CLKS = (CLK_HZ / 1000) * SIDE_DELAY_MS;

    localparam [2:0]
        S_STOP            = 3'd0,
        S_FORWARD         = 3'd1,
        S_PRE_SIDE_DELAY  = 3'd2,
        S_SIDE_MEASURE    = 3'd3,
        S_SIDE_EXTRA      = 3'd4,
        S_POST_SIDE_DELAY = 3'd5;

    reg [2:0]  state;
    reg [31:0] delay_cnt;
    reg [31:0] side_meas_cnt;
    reg [31:0] side_extra_cnt;

    wire motion_enable;
    wire sensor_too_close;

    assign motion_enable    = sensor_valid;
    assign sensor_too_close = motion_enable && (distance_mm < DIST_STOP_MM);

    function automatic [31:0] calc_extra_target_cnt;
        input [31:0] measured_cnt;
        reg   [63:0] scaled_cnt;
        begin
            if ((SIDE_EXTRA_SCALE_PCT <= 0) || (measured_cnt == 32'd0)) begin
                calc_extra_target_cnt = 32'd0;
            end else begin
                scaled_cnt = measured_cnt;
                scaled_cnt = (scaled_cnt * SIDE_EXTRA_SCALE_PCT + 64'd99) / 64'd100;
                if (scaled_cnt > 64'hFFFF_FFFF) begin
                    calc_extra_target_cnt = 32'hFFFF_FFFF;
                end else begin
                    calc_extra_target_cnt = scaled_cnt[31:0];
                end
            end
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_STOP;
            delay_cnt      <= 32'd0;
            side_meas_cnt  <= 32'd0;
            side_extra_cnt <= 32'd0;
        end else begin
            case (state)
                S_STOP: begin
                    delay_cnt      <= 32'd0;
                    side_meas_cnt  <= 32'd0;
                    side_extra_cnt <= 32'd0;
                    if (motion_enable) begin
                        if (sensor_too_close) begin
                            state <= S_PRE_SIDE_DELAY;
                        end else begin
                            state <= S_FORWARD;
                        end
                    end
                end

                S_FORWARD: begin
                    delay_cnt      <= 32'd0;
                    side_meas_cnt  <= 32'd0;
                    side_extra_cnt <= 32'd0;
                    if (!motion_enable) begin
                        state <= S_STOP;
                    end else if (sensor_too_close) begin
                        state <= S_PRE_SIDE_DELAY;
                    end
                end

                S_PRE_SIDE_DELAY: begin
                    side_meas_cnt  <= 32'd0;
                    side_extra_cnt <= 32'd0;
                    if (!motion_enable) begin
                        state     <= S_STOP;
                        delay_cnt <= 32'd0;
                    end else if (!sensor_too_close) begin
                        state     <= S_FORWARD;
                        delay_cnt <= 32'd0;
                    end else if (delay_cnt >= (SIDE_DELAY_CLKS - 1)) begin
                        state         <= S_SIDE_MEASURE;
                        delay_cnt     <= 32'd0;
                        side_meas_cnt <= 32'd0;
                    end else begin
                        delay_cnt <= delay_cnt + 32'd1;
                    end
                end

                S_SIDE_MEASURE: begin
                    delay_cnt      <= 32'd0;
                    side_extra_cnt <= 32'd0;
                    if (!motion_enable) begin
                        state          <= S_STOP;
                        side_meas_cnt  <= 32'd0;
                        side_extra_cnt <= 32'd0;
                    end else if (sensor_too_close) begin
                        if (side_meas_cnt != 32'hFFFF_FFFF) begin
                            side_meas_cnt <= side_meas_cnt + 32'd1;
                        end
                    end else begin
                        state          <= S_SIDE_EXTRA;
                        side_extra_cnt <= 32'd0;
                    end
                end

                S_SIDE_EXTRA: begin
                    delay_cnt <= 32'd0;
                    if (!motion_enable) begin
                        state          <= S_STOP;
                        side_meas_cnt  <= 32'd0;
                        side_extra_cnt <= 32'd0;
                    end else if (sensor_too_close) begin
                        state <= S_SIDE_MEASURE;
                    end else if (calc_extra_target_cnt(side_meas_cnt) == 32'd0) begin
                        state          <= S_POST_SIDE_DELAY;
                        delay_cnt      <= 32'd0;
                        side_meas_cnt  <= 32'd0;
                        side_extra_cnt <= 32'd0;
                    end else if (side_extra_cnt >= (calc_extra_target_cnt(side_meas_cnt) - 32'd1)) begin
                        state          <= S_POST_SIDE_DELAY;
                        delay_cnt      <= 32'd0;
                        side_meas_cnt  <= 32'd0;
                        side_extra_cnt <= 32'd0;
                    end else begin
                        side_extra_cnt <= side_extra_cnt + 32'd1;
                    end
                end

                S_POST_SIDE_DELAY: begin
                    side_meas_cnt  <= 32'd0;
                    side_extra_cnt <= 32'd0;
                    if (!motion_enable) begin
                        state     <= S_STOP;
                        delay_cnt <= 32'd0;
                    end else if (sensor_too_close) begin
                        state     <= S_PRE_SIDE_DELAY;
                        delay_cnt <= 32'd0;
                    end else if (delay_cnt >= (SIDE_DELAY_CLKS - 1)) begin
                        state     <= S_FORWARD;
                        delay_cnt <= 32'd0;
                    end else begin
                        delay_cnt <= delay_cnt + 32'd1;
                    end
                end

                default: begin
                    state          <= S_STOP;
                    delay_cnt      <= 32'd0;
                    side_meas_cnt  <= 32'd0;
                    side_extra_cnt <= 32'd0;
                end
            endcase
        end
    end

    always @(*) begin
        motor0_speed = 8'd0;
        motor1_speed = 8'd0;
        motor2_speed = 8'd0;
        motor3_speed = 8'd0;

        case (state)
            S_FORWARD: begin
                motor0_speed = (8'd127); // 
                motor1_speed = (8'd127);
                motor2_speed = (8'd127);
                motor3_speed = (8'd127);
            end

            S_SIDE_MEASURE,
            S_SIDE_EXTRA: begin
                //motor0_speed =  (MOTOR_SPEED_CMD[7:0] + 8'd0);
                //motor1_speed = -(MOTOR_SPEED_CMD[7:0] - 8'd0);
                //motor2_speed = -(MOTOR_SPEED_CMD[7:0] + 8'd35); 
                //motor3_speed =  (MOTOR_SPEED_CMD[7:0] - 8'd0);
					 motor0_speed = -(8'd100); // переднее левое
                motor1_speed =  (8'd100); // переднее правое, prev: 
                motor2_speed =  (8'd127); // заднее левое
                motor3_speed = -(8'd127); // заднее правое
            end

            default: begin
                motor0_speed = 8'd0;
                motor1_speed = 8'd0;
                motor2_speed = 8'd0;
                motor3_speed = 8'd0;
            end
        endcase
    end
endmodule

`default_nettype wire