`timescale 1ns / 1ps
`default_nettype none

module distance_motion_ctrl #(
    parameter integer DIST_STOP_MM    = 100,
    parameter integer MOTOR_SPEED_CMD = 8'd127
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sensor_valid,
    input  wire [15:0] distance_mm,
    output wire [7:0]  motor0_speed,
    output wire [7:0]  motor1_speed,
    output wire [7:0]  motor2_speed,
    output wire [7:0]  motor3_speed
);
    // clk и rst_n заведены сюда специально, чтобы дальше можно было
    // без изменения интерфейса добавить гистерезис, фильтрацию или FSM.
    // В текущей версии логика оставлена комбинаторной, чтобы не менять
    // поведение последней рабочей схемы.
    wire _unused_ok;
    assign _unused_ok = clk ^ rst_n;

    wire motion_enable;
    wire sensor_too_close;

    assign motion_enable    = sensor_valid;
    assign sensor_too_close = motion_enable && (distance_mm < DIST_STOP_MM);

    // assign motor0_speed = !motion_enable   ? 8'd0 :  MOTOR_SPEED_CMD[7:0];
    // assign motor1_speed = !motion_enable   ? 8'd0 : (sensor_too_close ? -MOTOR_SPEED_CMD[7:0] :  MOTOR_SPEED_CMD[7:0]);
    // assign motor2_speed = !motion_enable   ? 8'd0 : (sensor_too_close ? -MOTOR_SPEED_CMD[7:0] :  MOTOR_SPEED_CMD[7:0]);
    // assign motor3_speed = !motion_enable   ? 8'd0 :  MOTOR_SPEED_CMD[7:0];
	 assign motor0_speed = !motion_enable   ? 8'd0 :  MOTOR_SPEED_CMD[7:0];
    assign motor1_speed = !motion_enable   ? 8'd0 : (sensor_too_close ? 8'd0 :  MOTOR_SPEED_CMD[7:0]);
    assign motor2_speed = !motion_enable   ? 8'd0 : (sensor_too_close ? 8'd0 :  MOTOR_SPEED_CMD[7:0]);
    assign motor3_speed = !motion_enable   ? 8'd0 :  MOTOR_SPEED_CMD[7:0];
endmodule

`default_nettype wire