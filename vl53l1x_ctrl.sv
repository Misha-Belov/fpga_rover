`timescale 1ns / 1ps
`default_nettype none

module vl53l1x_ctrl #(
    parameter int unsigned CLK_HZ = 50_000_000
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         init_start,
    input  wire         sample_start,
    output logic [15:0] distance_mm,
    output logic        distance_valid,
    output logic        busy,
    output logic        done,
    output logic        error,
    output logic        nack,
    output logic        txn_req_valid,
    input  wire         txn_req_ready,
    output logic        txn_req_is_read,
    output logic [6:0]  txn_req_dev_addr,
    output logic [15:0] txn_req_reg_addr,
    output logic        txn_req_reg_addr_16b,
    output logic [7:0]  txn_req_wr_data,
    output logic [1:0]  txn_req_rd_len,
    input  wire         txn_rsp_done,
    input  wire         txn_rsp_error,
    input  wire         txn_rsp_nack,
    input  wire [15:0]  txn_rsp_rd_data
);
    logic init_d, sample_d;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            init_d <= 1'b0;
            sample_d <= 1'b0;
            distance_mm <= 16'h0000;
            distance_valid <= 1'b0;
            busy <= 1'b0;
            done <= 1'b0;
            error <= 1'b0;
            nack <= 1'b0;
            txn_req_valid <= 1'b0;
            txn_req_is_read <= 1'b0;
            txn_req_dev_addr <= 7'h00;
            txn_req_reg_addr <= 16'h0000;
            txn_req_reg_addr_16b <= 1'b0;
            txn_req_wr_data <= 8'h00;
            txn_req_rd_len <= 2'd0;
        end else begin
            init_d <= init_start;
            sample_d <= sample_start;
            done <= 1'b0;
            distance_valid <= 1'b0;
            busy <= 1'b0;
            txn_req_valid <= 1'b0;
            if ((init_start && !init_d) || (sample_start && !sample_d)) begin
                done <= 1'b1;
                error <= 1'b1;
                nack <= 1'b0;
            end
        end
    end
endmodule

`default_nettype wire
