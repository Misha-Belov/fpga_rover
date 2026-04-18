module roverc_pro_ctrl #(
    parameter integer CLK_HZ = 50000000,
    parameter [6:0] I2C_ADDR = 7'h38,
    parameter signed [7:0] MOTOR1_SPEED = 8'sd100,
    parameter signed [7:0] MOTOR2_SPEED = 8'sd100,
    parameter signed [7:0] MOTOR3_SPEED = 8'sd100,
    parameter signed [7:0] MOTOR4_SPEED = 8'sd100,
    parameter integer STARTUP_DELAY_MS = 800,
    parameter integer UPDATE_PERIOD_MS = 100
) (
    input  wire        clk,
    input  wire        rst_n,

    output reg         busy,
    output reg         done,
    output reg         error,
    output reg         nack,
    output reg  [2:0]  step_dbg,

    output reg         txn_req_valid,
    input  wire        txn_req_ready,
    output reg         txn_req_is_read,
    output reg [6:0]   txn_req_dev_addr,
    output reg [15:0]  txn_req_reg_addr,
    output reg         txn_req_reg_addr_16b,
    output reg [7:0]   txn_req_wr_data,
    output reg [1:0]   txn_req_rd_len,
    input  wire        txn_rsp_done,
    input  wire        txn_rsp_error,
    input  wire        txn_rsp_nack,
    input  wire [15:0] txn_rsp_rd_data
);

    localparam integer STARTUP_DELAY_CLKS = (CLK_HZ / 1000) * STARTUP_DELAY_MS;
    localparam integer UPDATE_PERIOD_CLKS = (CLK_HZ / 1000) * UPDATE_PERIOD_MS;

    localparam [3:0]
        S_STARTUP_WAIT = 4'd0,
        S_SEND_M0      = 4'd1,
        S_WAIT_M0      = 4'd2,
        S_SEND_M1      = 4'd3,
        S_WAIT_M1      = 4'd4,
        S_SEND_M2      = 4'd5,
        S_WAIT_M2      = 4'd6,
        S_SEND_M3      = 4'd7,
        S_WAIT_M3      = 4'd8,
        S_HOLD         = 4'd9;

    reg [3:0] state;
    reg [31:0] delay_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                <= S_STARTUP_WAIT;
            delay_cnt            <= 32'd0;
            busy                 <= 1'b1;
            done                 <= 1'b0;
            error                <= 1'b0;
            nack                 <= 1'b0;
            step_dbg             <= 3'd0;
            txn_req_valid        <= 1'b0;
            txn_req_is_read      <= 1'b0;
            txn_req_dev_addr     <= I2C_ADDR;
            txn_req_reg_addr     <= 16'h0000;
            txn_req_reg_addr_16b <= 1'b0;
            txn_req_wr_data      <= 8'h00;
            txn_req_rd_len       <= 2'd0;
        end else begin
            done <= 1'b0;

            case (state)
                S_STARTUP_WAIT: begin
                    busy <= 1'b1;
                    step_dbg <= 3'd0;
                    if (delay_cnt < STARTUP_DELAY_CLKS - 1) begin
                        delay_cnt <= delay_cnt + 32'd1;
                    end else begin
                        delay_cnt <= 32'd0;
                        state <= S_SEND_M0;
                    end
                end

                S_SEND_M0: begin
                    step_dbg              <= 3'd1;
                    txn_req_valid         <= 1'b1;
                    txn_req_is_read       <= 1'b0;
                    txn_req_dev_addr      <= I2C_ADDR;
                    txn_req_reg_addr      <= 16'h0000;
                    txn_req_reg_addr_16b  <= 1'b0;
                    txn_req_wr_data       <= MOTOR1_SPEED;
                    txn_req_rd_len        <= 2'd0;
                    if (txn_req_ready) begin
                        txn_req_valid <= 1'b0;
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
                            state <= S_HOLD;
                        end else begin
                            state <= S_SEND_M1;
                        end
                    end
                end

                S_SEND_M1: begin
                    step_dbg              <= 3'd2;
                    txn_req_valid         <= 1'b1;
                    txn_req_is_read       <= 1'b0;
                    txn_req_dev_addr      <= I2C_ADDR;
                    txn_req_reg_addr      <= 16'h0001;
                    txn_req_reg_addr_16b  <= 1'b0;
                    txn_req_wr_data       <= MOTOR2_SPEED;
                    txn_req_rd_len        <= 2'd0;
                    if (txn_req_ready) begin
                        txn_req_valid <= 1'b0;
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
                            state <= S_HOLD;
                        end else begin
                            state <= S_SEND_M2;
                        end
                    end
                end

                S_SEND_M2: begin
                    step_dbg              <= 3'd3;
                    txn_req_valid         <= 1'b1;
                    txn_req_is_read       <= 1'b0;
                    txn_req_dev_addr      <= I2C_ADDR;
                    txn_req_reg_addr      <= 16'h0002;
                    txn_req_reg_addr_16b  <= 1'b0;
                    txn_req_wr_data       <= MOTOR3_SPEED;
                    txn_req_rd_len        <= 2'd0;
                    if (txn_req_ready) begin
                        txn_req_valid <= 1'b0;
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
                            state <= S_HOLD;
                        end else begin
                            state <= S_SEND_M3;
                        end
                    end
                end

                S_SEND_M3: begin
                    step_dbg              <= 3'd4;
                    txn_req_valid         <= 1'b1;
                    txn_req_is_read       <= 1'b0;
                    txn_req_dev_addr      <= I2C_ADDR;
                    txn_req_reg_addr      <= 16'h0003;
                    txn_req_reg_addr_16b  <= 1'b0;
                    txn_req_wr_data       <= MOTOR4_SPEED;
                    txn_req_rd_len        <= 2'd0;
                    if (txn_req_ready) begin
                        txn_req_valid <= 1'b0;
                        state <= S_WAIT_M3;
                    end
                end

                S_WAIT_M3: begin
                    if (txn_rsp_done) begin
                        busy <= 1'b0;
                        done <= 1'b1;
                        if (txn_rsp_error || txn_rsp_nack) begin
                            error <= txn_rsp_error;
                            nack  <= txn_rsp_nack;
                        end
                        state <= S_HOLD;
                        delay_cnt <= 32'd0;
                    end
                end

                S_HOLD: begin
                    step_dbg <= (error || nack) ? 3'd7 : 3'd5;
                    if (delay_cnt < UPDATE_PERIOD_CLKS - 1) begin
                        delay_cnt <= delay_cnt + 32'd1;
                    end else begin
                        delay_cnt <= 32'd0;
                        busy <= 1'b1;
                        error <= 1'b0;
                        nack <= 1'b0;
                        state <= S_SEND_M0;
                    end
                end

                default: begin
                    state <= S_STARTUP_WAIT;
                end
            endcase
        end
    end
endmodule