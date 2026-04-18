`timescale 1ns / 1ps
`default_nettype none

module roverc_motor_ctrl #(
    parameter [6:0] I2C_ADDR = 7'h38,
    parameter [23:0] STARTUP_HOLDOFF_CLKS = 24'd1000000 // 20 ms @ 50 MHz
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,

    input  wire [7:0]  motor0_speed,
    input  wire [7:0]  motor1_speed,
    input  wire [7:0]  motor2_speed,
    input  wire [7:0]  motor3_speed,

    output reg         busy,
    output reg         done,
    output reg         error,
    output reg         nack,
    output reg  [2:0]  step_dbg,

    output reg         txn_req_valid,
    input  wire        txn_req_ready,
    output reg         txn_req_is_read,
    output reg  [6:0]  txn_req_dev_addr,
    output reg  [15:0] txn_req_reg_addr,
    output reg         txn_req_reg_addr_16b,
    output reg  [7:0]  txn_req_wr_data,
    output reg  [1:0]  txn_req_rd_len,

    input  wire        txn_rsp_done,
    input  wire        txn_rsp_error,
    input  wire        txn_rsp_nack,
    input  wire [15:0] txn_rsp_rd_data
);
    localparam [3:0]
        S_HOLDOFF = 4'd0,
        S_IDLE    = 4'd1,
        S_SEND_M0 = 4'd2,
        S_WAIT_M0 = 4'd3,
        S_SEND_M1 = 4'd4,
        S_WAIT_M1 = 4'd5,
        S_SEND_M2 = 4'd6,
        S_WAIT_M2 = 4'd7,
        S_SEND_M3 = 4'd8,
        S_WAIT_M3 = 4'd9;

    reg [3:0] state;
    reg       start_d;
    reg [23:0] holdoff_cnt;

    reg [7:0] motor0_speed_l;
    reg [7:0] motor1_speed_l;
    reg [7:0] motor2_speed_l;
    reg [7:0] motor3_speed_l;

    wire start_pulse;
    wire unused_rd_data;
    assign start_pulse   = start & ~start_d;
    assign unused_rd_data = txn_rsp_rd_data[0];

    always @(*) begin
        txn_req_valid        = 1'b0;
        txn_req_is_read      = 1'b0;
        txn_req_dev_addr     = I2C_ADDR;
        txn_req_reg_addr     = 16'h0000;
        txn_req_reg_addr_16b = 1'b0;
        txn_req_wr_data      = 8'h00;
        txn_req_rd_len       = 2'd0;
        step_dbg             = 3'd0;

        case (state)
            S_SEND_M0: begin
                txn_req_valid   = 1'b1;
                txn_req_reg_addr = 16'h0000;
                txn_req_wr_data = motor0_speed_l;
                step_dbg        = 3'd1;
            end

            S_SEND_M1: begin
                txn_req_valid   = 1'b1;
                txn_req_reg_addr = 16'h0001;
                txn_req_wr_data = motor1_speed_l;
                step_dbg        = 3'd2;
            end

            S_SEND_M2: begin
                txn_req_valid   = 1'b1;
                txn_req_reg_addr = 16'h0002;
                txn_req_wr_data = motor2_speed_l;
                step_dbg        = 3'd3;
            end

            S_SEND_M3: begin
                txn_req_valid   = 1'b1;
                txn_req_reg_addr = 16'h0003;
                txn_req_wr_data = motor3_speed_l;
                step_dbg        = 3'd4;
            end

            S_WAIT_M0: step_dbg = 3'd1;
            S_WAIT_M1: step_dbg = 3'd2;
            S_WAIT_M2: step_dbg = 3'd3;
            S_WAIT_M3: step_dbg = 3'd4;
            default:   step_dbg = 3'd0;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                <= S_HOLDOFF;
            start_d              <= 1'b0;
            holdoff_cnt          <= 24'd0;
            motor0_speed_l       <= 8'h00;
            motor1_speed_l       <= 8'h00;
            motor2_speed_l       <= 8'h00;
            motor3_speed_l       <= 8'h00;
            busy                 <= 1'b0;
            done                 <= 1'b0;
            error                <= 1'b0;
            nack                 <= 1'b0;
        end else begin
            start_d <= start;
            done    <= 1'b0;

            case (state)
                S_HOLDOFF: begin
                    busy  <= 1'b0;
                    error <= 1'b0;
                    nack  <= 1'b0;
                    if (holdoff_cnt >= STARTUP_HOLDOFF_CLKS - 1) begin
                        state <= S_IDLE;
                    end else begin
                        holdoff_cnt <= holdoff_cnt + 24'd1;
                    end
                end

                S_IDLE: begin
                    busy <= 1'b0;
                    if (start_pulse) begin
                        motor0_speed_l <= motor0_speed;
                        motor1_speed_l <= motor1_speed;
                        motor2_speed_l <= motor2_speed;
                        motor3_speed_l <= motor3_speed;
                        error          <= 1'b0;
                        nack           <= 1'b0;
                        busy           <= 1'b1;
                        state          <= S_SEND_M0;
                    end
                end

                S_SEND_M0: begin
                    if (txn_req_ready) begin
                        state <= S_WAIT_M0;
                    end
                end

                S_WAIT_M0: begin
                    if (txn_rsp_done) begin
                        if (txn_rsp_error || txn_rsp_nack) begin
                            error <= txn_rsp_error;
                            nack  <= txn_rsp_nack;
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            state <= S_IDLE;
                        end else begin
                            state <= S_SEND_M1;
                        end
                    end
                end

                S_SEND_M1: begin
                    if (txn_req_ready) begin
                        state <= S_WAIT_M1;
                    end
                end

                S_WAIT_M1: begin
                    if (txn_rsp_done) begin
                        if (txn_rsp_error || txn_rsp_nack) begin
                            error <= txn_rsp_error;
                            nack  <= txn_rsp_nack;
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            state <= S_IDLE;
                        end else begin
                            state <= S_SEND_M2;
                        end
                    end
                end

                S_SEND_M2: begin
                    if (txn_req_ready) begin
                        state <= S_WAIT_M2;
                    end
                end

                S_WAIT_M2: begin
                    if (txn_rsp_done) begin
                        if (txn_rsp_error || txn_rsp_nack) begin
                            error <= txn_rsp_error;
                            nack  <= txn_rsp_nack;
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            state <= S_IDLE;
                        end else begin
                            state <= S_SEND_M3;
                        end
                    end
                end

                S_SEND_M3: begin
                    if (txn_req_ready) begin
                        state <= S_WAIT_M3;
                    end
                end

                S_WAIT_M3: begin
                    if (txn_rsp_done) begin
                        error <= txn_rsp_error;
                        nack  <= txn_rsp_nack;
                        busy  <= 1'b0;
                        done  <= 1'b1;
                        state <= S_IDLE;
                    end
                end

                default: begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    error <= 1'b1;
                    nack  <= 1'b0;
                    state <= S_IDLE;
                end
            endcase
        end
    end
endmodule

`default_nettype wire