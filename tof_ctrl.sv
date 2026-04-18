`timescale 1ns / 1ps
`default_nettype none

module tof_ctrl #(
    parameter int unsigned SENSOR_KIND = 0,
    parameter int unsigned CLK_HZ      = 50_000_000
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
    localparam int unsigned SENSOR_AUTO    = 0;
    localparam int unsigned SENSOR_VL53L0X = 1;
    localparam int unsigned SENSOR_VL53L1X = 2;

    typedef enum logic [2:0] {
        WR_IDLE           = 3'd0,
        WR_L0_INIT_WAIT   = 3'd1,
        WR_L1_INIT_WAIT   = 3'd2,
        WR_L0_SAMPLE_WAIT = 3'd3,
        WR_L1_SAMPLE_WAIT = 3'd4
    } wrapper_state_t;

    typedef enum logic [1:0] {
        KIND_NONE = 2'd0,
        KIND_L0   = 2'd1,
        KIND_L1   = 2'd2
    } sensor_kind_t;

    wrapper_state_t state;
    sensor_kind_t   active_kind;
    sensor_kind_t   current_kind;

    logic init_start_d;
    logic sample_start_d;

    logic l0_init_start;
    logic l0_sample_start;
    logic l1_init_start;
    logic l1_sample_start;

    logic [15:0] l0_distance_mm;
    logic        l0_distance_valid;
    logic        l0_busy;
    logic        l0_done;
    logic        l0_error;
    logic        l0_nack;
    logic        l0_txn_req_valid;
    logic        l0_txn_req_ready;
    logic        l0_txn_req_is_read;
    logic [6:0]  l0_txn_req_dev_addr;
    logic [15:0] l0_txn_req_reg_addr;
    logic        l0_txn_req_reg_addr_16b;
    logic [7:0]  l0_txn_req_wr_data;
    logic [1:0]  l0_txn_req_rd_len;
    logic        l0_txn_rsp_done;
    logic        l0_txn_rsp_error;
    logic        l0_txn_rsp_nack;
    logic [15:0] l0_txn_rsp_rd_data;

    logic [15:0] l1_distance_mm;
    logic        l1_distance_valid;
    logic        l1_busy;
    logic        l1_done;
    logic        l1_error;
    logic        l1_nack;
    logic        l1_txn_req_valid;
    logic        l1_txn_req_ready;
    logic        l1_txn_req_is_read;
    logic [6:0]  l1_txn_req_dev_addr;
    logic [15:0] l1_txn_req_reg_addr;
    logic        l1_txn_req_reg_addr_16b;
    logic [7:0]  l1_txn_req_wr_data;
    logic [1:0]  l1_txn_req_rd_len;
    logic        l1_txn_rsp_done;
    logic        l1_txn_rsp_error;
    logic        l1_txn_rsp_nack;
    logic [15:0] l1_txn_rsp_rd_data;

    assign l0_txn_req_ready   = (current_kind == KIND_L0) ? txn_req_ready   : 1'b0;
    assign l0_txn_rsp_done    = (current_kind == KIND_L0) ? txn_rsp_done    : 1'b0;
    assign l0_txn_rsp_error   = (current_kind == KIND_L0) ? txn_rsp_error   : 1'b0;
    assign l0_txn_rsp_nack    = (current_kind == KIND_L0) ? txn_rsp_nack    : 1'b0;
    assign l0_txn_rsp_rd_data = (current_kind == KIND_L0) ? txn_rsp_rd_data : 16'h0000;

    assign l1_txn_req_ready   = (current_kind == KIND_L1) ? txn_req_ready   : 1'b0;
    assign l1_txn_rsp_done    = (current_kind == KIND_L1) ? txn_rsp_done    : 1'b0;
    assign l1_txn_rsp_error   = (current_kind == KIND_L1) ? txn_rsp_error   : 1'b0;
    assign l1_txn_rsp_nack    = (current_kind == KIND_L1) ? txn_rsp_nack    : 1'b0;
    assign l1_txn_rsp_rd_data = (current_kind == KIND_L1) ? txn_rsp_rd_data : 16'h0000;

    always_comb begin
        txn_req_valid        = 1'b0;
        txn_req_is_read      = 1'b0;
        txn_req_dev_addr     = 7'h00;
        txn_req_reg_addr     = 16'h0000;
        txn_req_reg_addr_16b = 1'b0;
        txn_req_wr_data      = 8'h00;
        txn_req_rd_len       = 2'd0;

        case (current_kind)
            KIND_L0: begin
                txn_req_valid        = l0_txn_req_valid;
                txn_req_is_read      = l0_txn_req_is_read;
                txn_req_dev_addr     = l0_txn_req_dev_addr;
                txn_req_reg_addr     = l0_txn_req_reg_addr;
                txn_req_reg_addr_16b = l0_txn_req_reg_addr_16b;
                txn_req_wr_data      = l0_txn_req_wr_data;
                txn_req_rd_len       = l0_txn_req_rd_len;
            end

            KIND_L1: begin
                txn_req_valid        = l1_txn_req_valid;
                txn_req_is_read      = l1_txn_req_is_read;
                txn_req_dev_addr     = l1_txn_req_dev_addr;
                txn_req_reg_addr     = l1_txn_req_reg_addr;
                txn_req_reg_addr_16b = l1_txn_req_reg_addr_16b;
                txn_req_wr_data      = l1_txn_req_wr_data;
                txn_req_rd_len       = l1_txn_req_rd_len;
            end

            default: begin
            end
        endcase
    end

    vl5310x_ctrl #(
        .CLK_HZ(CLK_HZ)
    ) u_vl53l0x_ctrl (
        .clk(clk),
        .rst_n(rst_n),
        .init_start(l0_init_start),
        .sample_start(l0_sample_start),
        .distance_mm(l0_distance_mm),
        .distance_valid(l0_distance_valid),
        .busy(l0_busy),
        .done(l0_done),
        .error(l0_error),
        .nack(l0_nack),
        .txn_req_valid(l0_txn_req_valid),
        .txn_req_ready(l0_txn_req_ready),
        .txn_req_is_read(l0_txn_req_is_read),
        .txn_req_dev_addr(l0_txn_req_dev_addr),
        .txn_req_reg_addr(l0_txn_req_reg_addr),
        .txn_req_reg_addr_16b(l0_txn_req_reg_addr_16b),
        .txn_req_wr_data(l0_txn_req_wr_data),
        .txn_req_rd_len(l0_txn_req_rd_len),
        .txn_rsp_done(l0_txn_rsp_done),
        .txn_rsp_error(l0_txn_rsp_error),
        .txn_rsp_nack(l0_txn_rsp_nack),
        .txn_rsp_rd_data(l0_txn_rsp_rd_data)
    );

    vl53l1x_ctrl #(
        .CLK_HZ(CLK_HZ)
    ) u_vl53l1x_ctrl (
        .clk(clk),
        .rst_n(rst_n),
        .init_start(l1_init_start),
        .sample_start(l1_sample_start),
        .distance_mm(l1_distance_mm),
        .distance_valid(l1_distance_valid),
        .busy(l1_busy),
        .done(l1_done),
        .error(l1_error),
        .nack(l1_nack),
        .txn_req_valid(l1_txn_req_valid),
        .txn_req_ready(l1_txn_req_ready),
        .txn_req_is_read(l1_txn_req_is_read),
        .txn_req_dev_addr(l1_txn_req_dev_addr),
        .txn_req_reg_addr(l1_txn_req_reg_addr),
        .txn_req_reg_addr_16b(l1_txn_req_reg_addr_16b),
        .txn_req_wr_data(l1_txn_req_wr_data),
        .txn_req_rd_len(l1_txn_req_rd_len),
        .txn_rsp_done(l1_txn_rsp_done),
        .txn_rsp_error(l1_txn_rsp_error),
        .txn_rsp_nack(l1_txn_rsp_nack),
        .txn_rsp_rd_data(l1_txn_rsp_rd_data)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= WR_IDLE;
            active_kind    <= KIND_NONE;
            current_kind   <= KIND_NONE;
            init_start_d   <= 1'b0;
            sample_start_d <= 1'b0;
            l0_init_start  <= 1'b0;
            l0_sample_start <= 1'b0;
            l1_init_start  <= 1'b0;
            l1_sample_start <= 1'b0;
            distance_mm    <= 16'h0000;
            distance_valid <= 1'b0;
            busy           <= 1'b0;
            done           <= 1'b0;
            error          <= 1'b0;
            nack           <= 1'b0;
        end else begin
            init_start_d    <= init_start;
            sample_start_d  <= sample_start;
            l0_init_start   <= 1'b0;
            l0_sample_start <= 1'b0;
            l1_init_start   <= 1'b0;
            l1_sample_start <= 1'b0;
            done            <= 1'b0;

            case (state)
                WR_IDLE: begin
                    busy         <= 1'b0;
                    current_kind <= KIND_NONE;

                    if (init_start && !init_start_d) begin
                        busy           <= 1'b1;
                        error          <= 1'b0;
                        nack           <= 1'b0;
                        distance_valid <= 1'b0;

                        if (SENSOR_KIND == SENSOR_VL53L1X) begin
                            current_kind  <= KIND_L1;
                            l1_init_start <= 1'b1;
                            state         <= WR_L1_INIT_WAIT;
                        end else begin
                            current_kind  <= KIND_L0;
                            l0_init_start <= 1'b1;
                            state         <= WR_L0_INIT_WAIT;
                        end
                    end else if (sample_start && !sample_start_d) begin
                        busy           <= 1'b1;
                        error          <= 1'b0;
                        nack           <= 1'b0;
                        distance_valid <= 1'b0;

                        case (active_kind)
                            KIND_L0: begin
                                current_kind    <= KIND_L0;
                                l0_sample_start <= 1'b1;
                                state           <= WR_L0_SAMPLE_WAIT;
                            end

                            KIND_L1: begin
                                current_kind    <= KIND_L1;
                                l1_sample_start <= 1'b1;
                                state           <= WR_L1_SAMPLE_WAIT;
                            end

                            default: begin
                                busy        <= 1'b0;
                                done        <= 1'b1;
                                error       <= 1'b1;
                                current_kind <= KIND_NONE;
                            end
                        endcase
                    end
                end

                WR_L0_INIT_WAIT: begin
                    if (l0_done) begin
                        if ((SENSOR_KIND == SENSOR_AUTO) &&
                            (active_kind == KIND_NONE) &&
                            (l0_error || l0_nack)) begin
                            current_kind  <= KIND_L1;
                            l1_init_start <= 1'b1;
                            state         <= WR_L1_INIT_WAIT;
                        end else begin
                            busy        <= 1'b0;
                            done        <= 1'b1;
                            error       <= l0_error;
                            nack        <= l0_nack;
                            current_kind <= KIND_NONE;

                            if (!(l0_error || l0_nack)) begin
                                active_kind <= KIND_L0;
                            end

                            state <= WR_IDLE;
                        end
                    end
                end

                WR_L1_INIT_WAIT: begin
                    if (l1_done) begin
                        busy        <= 1'b0;
                        done        <= 1'b1;
                        error       <= l1_error;
                        nack        <= l1_nack;
                        current_kind <= KIND_NONE;

                        if (!(l1_error || l1_nack)) begin
                            active_kind <= KIND_L1;
                        end

                        state <= WR_IDLE;
                    end
                end

                WR_L0_SAMPLE_WAIT: begin
                    if (l0_done) begin
                        busy           <= 1'b0;
                        done           <= 1'b1;
                        error          <= l0_error;
                        nack           <= l0_nack;
                        distance_mm    <= l0_distance_mm;
                        distance_valid <= l0_distance_valid;
                        current_kind   <= KIND_NONE;
                        state          <= WR_IDLE;
                    end
                end

                WR_L1_SAMPLE_WAIT: begin
                    if (l1_done) begin
                        busy           <= 1'b0;
                        done           <= 1'b1;
                        error          <= l1_error;
                        nack           <= l1_nack;
                        distance_mm    <= l1_distance_mm;
                        distance_valid <= l1_distance_valid;
                        current_kind   <= KIND_NONE;
                        state          <= WR_IDLE;
                    end
                end

                default: begin
                    busy         <= 1'b0;
                    done         <= 1'b1;
                    error        <= 1'b1;
                    current_kind <= KIND_NONE;
                    state        <= WR_IDLE;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
