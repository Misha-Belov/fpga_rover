`timescale 1ns / 1ps
`default_nettype none

module i2c_master #(
    parameter int unsigned CLK_HZ      = 50_000_000,
    parameter int unsigned I2C_HZ      = 100_000
) (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         start,
    input  wire         is_read,
    input  wire [6:0]   dev_addr,
    input  wire [15:0]  reg_addr,
    input  wire         reg_addr_16b,
    input  wire [7:0]   wr_data,
    input  wire [1:0]   rd_len,

    output logic [15:0] rd_data,
    output logic        busy,
    output logic        done,
    output logic        error,
    output logic        nack,

    output logic        scl_drive_low,
    output logic        sda_drive_low,
    input  wire         scl_in,
    input  wire         sda_in
);
    typedef enum logic [3:0] {
        ST_IDLE         = 4'd0,
        ST_WR_REQ       = 4'd1,
        ST_WR_WAIT      = 4'd2,
        ST_RD_PTR_REQ   = 4'd3,
        ST_RD_PTR_WAIT  = 4'd4,
        ST_RD_DATA_REQ  = 4'd5,
        ST_RD_DATA_WAIT = 4'd6
    } state_t;

    state_t state;
    logic   start_d;

    logic [6:0] req_dev_addr_l;
    logic [15:0] req_reg_addr_l;
    logic        req_reg_addr_16b_l;
    logic [7:0] req_wr_data_l;
    logic [1:0] req_rd_len_l;

    logic        eng_start;
    logic        eng_is_read;
    logic [2:0]  eng_write_len;
    logic [1:0]  eng_read_len;
    logic [7:0]  eng_byte0;
    logic [7:0]  eng_byte1;
    logic [7:0]  eng_byte2;
    logic [7:0]  eng_byte3;
    logic [15:0] eng_rd_data;
    logic        eng_done;
    logic        eng_error;
    logic        eng_nack;

    i2c_driver #(
        .CLK_HZ(CLK_HZ),
        .I2C_HZ(I2C_HZ)
    ) u_i2c_driver (
        .clk(clk),
        .rst_n(rst_n),
        .op_start(eng_start),
        .op_is_read(eng_is_read),
        .op_write_len(eng_write_len),
        .op_read_len(eng_read_len),
        .op_byte0(eng_byte0),
        .op_byte1(eng_byte1),
        .op_byte2(eng_byte2),
        .op_byte3(eng_byte3),
        .op_rd_data(eng_rd_data),
        .busy(),
        .done(eng_done),
        .error(eng_error),
        .nack(eng_nack),
        .scl_drive_low(scl_drive_low),
        .sda_drive_low(sda_drive_low),
        .scl_in(scl_in),
        .sda_in(sda_in)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= ST_IDLE;
            start_d        <= 1'b0;
            req_dev_addr_l <= 7'h00;
            req_reg_addr_l <= 16'h0000;
            req_reg_addr_16b_l <= 1'b0;
            req_wr_data_l  <= 8'h00;
            req_rd_len_l   <= 2'd1;
            eng_start      <= 1'b0;
            eng_is_read    <= 1'b0;
            eng_write_len  <= 3'd0;
            eng_read_len   <= 2'd0;
            eng_byte0      <= 8'h00;
            eng_byte1      <= 8'h00;
            eng_byte2      <= 8'h00;
            eng_byte3      <= 8'h00;
            rd_data        <= 16'h0000;
            busy           <= 1'b0;
            done           <= 1'b0;
            error          <= 1'b0;
            nack           <= 1'b0;
        end else begin
            start_d   <= start;
            done      <= 1'b0;
            eng_start <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;

                    if (start && !start_d) begin
                        if (is_read && ((rd_len == 2'd0) || (rd_len > 2'd2))) begin
                            rd_data <= 16'h0000;
                            done    <= 1'b1;
                            error   <= 1'b1;
                            nack    <= 1'b0;
                        end else begin
                            req_dev_addr_l <= dev_addr;
                            req_reg_addr_l <= reg_addr;
                            req_reg_addr_16b_l <= reg_addr_16b;
                            req_wr_data_l  <= wr_data;
                            req_rd_len_l   <= is_read ? rd_len : 2'd0;
                            rd_data        <= 16'h0000;
                            busy           <= 1'b1;
                            error          <= 1'b0;
                            nack           <= 1'b0;

                            if (is_read) begin
                                state <= ST_RD_PTR_REQ;
                            end else begin
                                state <= ST_WR_REQ;
                            end
                        end
                    end
                end

                ST_WR_REQ: begin
                    eng_is_read   <= 1'b0;
                    eng_write_len <= req_reg_addr_16b_l ? 3'd4 : 3'd3;
                    eng_read_len  <= 2'd0;
                    eng_byte0     <= {req_dev_addr_l, 1'b0};
                    eng_byte1     <= req_reg_addr_16b_l ? req_reg_addr_l[15:8] : req_reg_addr_l[7:0];
                    eng_byte2     <= req_reg_addr_16b_l ? req_reg_addr_l[7:0] : req_wr_data_l;
                    eng_byte3     <= req_reg_addr_16b_l ? req_wr_data_l : 8'h00;
                    eng_start     <= 1'b1;
                    state         <= ST_WR_WAIT;
                end

                ST_WR_WAIT: begin
                    if (eng_done) begin
                        busy  <= 1'b0;
                        done  <= 1'b1;
                        error <= eng_error;
                        nack  <= eng_nack;
                        state <= ST_IDLE;
                    end
                end

                ST_RD_PTR_REQ: begin
                    eng_is_read   <= 1'b0;
                    eng_write_len <= req_reg_addr_16b_l ? 3'd3 : 3'd2;
                    eng_read_len  <= 2'd0;
                    eng_byte0     <= {req_dev_addr_l, 1'b0};
                    eng_byte1     <= req_reg_addr_16b_l ? req_reg_addr_l[15:8] : req_reg_addr_l[7:0];
                    eng_byte2     <= req_reg_addr_16b_l ? req_reg_addr_l[7:0] : 8'h00;
                    eng_byte3     <= 8'h00;
                    eng_start     <= 1'b1;
                    state         <= ST_RD_PTR_WAIT;
                end

                ST_RD_PTR_WAIT: begin
                    if (eng_done) begin
                        if (eng_error || eng_nack) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            error <= eng_error;
                            nack  <= eng_nack;
                            state <= ST_IDLE;
                        end else begin
                            state <= ST_RD_DATA_REQ;
                        end
                    end
                end

                ST_RD_DATA_REQ: begin
                    eng_is_read   <= 1'b1;
                    eng_write_len <= 3'd1;
                    eng_read_len  <= req_rd_len_l;
                    eng_byte0     <= {req_dev_addr_l, 1'b1};
                    eng_byte1     <= 8'h00;
                    eng_byte2     <= 8'h00;
                    eng_byte3     <= 8'h00;
                    eng_start     <= 1'b1;
                    state         <= ST_RD_DATA_WAIT;
                end

                ST_RD_DATA_WAIT: begin
                    if (eng_done) begin
                        rd_data <= eng_rd_data;
                        busy    <= 1'b0;
                        done    <= 1'b1;
                        error   <= eng_error;
                        nack    <= eng_nack;
                        state   <= ST_IDLE;
                    end
                end

                default: begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    error <= 1'b1;
                    nack  <= 1'b0;
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
