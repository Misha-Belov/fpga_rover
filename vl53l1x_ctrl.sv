`timescale 1ns / 1ps
`default_nettype none

module vl53l1x_ctrl #(
    parameter int unsigned CLK_HZ            = 50_000_000,
    parameter int unsigned TIMEOUT_MS        = 20,
    parameter int unsigned STATUS_POLL_LIMIT = 512
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
    localparam logic [6:0] TOF_ADDR             = 7'h29;
    localparam logic [15:0] REG_BOOT_STATE      = 16'h00E5;
    localparam logic [15:0] REG_IDENT_MODEL_ID  = 16'h010F;
    localparam logic [15:0] REG_CONFIG_BASE      = 16'h002D;
    localparam logic [15:0] REG_GPIO_HV_MUX_CTRL = 16'h0030;
    localparam logic [15:0] REG_GPIO_STATUS      = 16'h0031;
    localparam logic [15:0] REG_VHV_LOOP_BOUND   = 16'h0008;
    localparam logic [15:0] REG_PHASECAL_CONFIG  = 16'h000B;
    localparam logic [15:0] REG_INTERRUPT_CLEAR = 16'h0086;
    localparam logic [15:0] REG_MODE_START      = 16'h0087;
    localparam logic [15:0] REG_RANGE_STATUS    = 16'h0089;
    localparam logic [15:0] REG_RANGE_MM        = 16'h0096;

    localparam logic [7:0] START_RANGING_BACK_TO_BACK = 8'h40;
    localparam logic [7:0] STOP_RANGING                = 8'h00;
    localparam logic [7:0] CLEAR_RANGE_INTERRUPT      = 8'h01;
    localparam logic [7:0] VHV_LOOP_BOUND_INIT        = 8'h09;
    localparam logic [7:0] PHASECAL_INIT              = 8'h00;
    localparam logic [15:0] EXPECT_MODEL_ID_0         = 16'hEACC;
    localparam logic [15:0] EXPECT_MODEL_ID_1         = 16'hEBAA;
    localparam int unsigned CONFIG_COUNT              = 91;

    localparam int unsigned CLKS_PER_MS               = (CLK_HZ < 1000) ? 1 : (CLK_HZ / 1000);
    localparam int unsigned TIMEOUT_CLKS_R            = CLKS_PER_MS * TIMEOUT_MS;
    localparam int unsigned TIMEOUT_CLKS              = (TIMEOUT_CLKS_R < 1) ? 1 : TIMEOUT_CLKS_R;
    localparam int unsigned TIMEOUT_W                 = (TIMEOUT_CLKS <= 1) ? 1 : $clog2(TIMEOUT_CLKS);
    localparam int unsigned STATUS_POLL_LIMIT_E       = (STATUS_POLL_LIMIT < 1) ? 1 : STATUS_POLL_LIMIT;
    localparam int unsigned STATUS_POLL_W             = (STATUS_POLL_LIMIT_E <= 1) ? 1 : $clog2(STATUS_POLL_LIMIT_E);
    localparam int unsigned CONFIG_IDX_W              = (CONFIG_COUNT <= 1) ? 1 : $clog2(CONFIG_COUNT);

    typedef enum logic [4:0] {
        ST_IDLE           = 5'd0,
        ST_INIT_BOOT_REQ  = 5'd1,
        ST_INIT_BOOT_WAIT = 5'd2,
        ST_INIT_BOOT_EVAL = 5'd3,
        ST_INIT_ID_REQ    = 5'd4,
        ST_INIT_ID_WAIT   = 5'd5,
        ST_INIT_CFG_REQ    = 5'd6,
        ST_INIT_CFG_WAIT   = 5'd7,
        ST_INIT_POL_REQ    = 5'd8,
        ST_INIT_POL_WAIT   = 5'd9,
        ST_INIT_START_REQ  = 5'd10,
        ST_INIT_START_WAIT = 5'd11,
        ST_INIT_CLEAR_REQ  = 5'd12,
        ST_INIT_CLEAR_WAIT = 5'd13,
        ST_INIT_STOP_REQ   = 5'd14,
        ST_INIT_STOP_WAIT  = 5'd15,
        ST_INIT_VHV_REQ    = 5'd16,
        ST_INIT_VHV_WAIT   = 5'd17,
        ST_INIT_PHASE_REQ  = 5'd18,
        ST_INIT_PHASE_WAIT = 5'd19,
        ST_INIT_FINAL_START_REQ  = 5'd20,
        ST_INIT_FINAL_START_WAIT = 5'd21,
        ST_STATUS_REQ        = 5'd22,
        ST_STATUS_WAIT       = 5'd23,
        ST_STATUS_EVAL       = 5'd24,
        ST_RANGE_STATUS_REQ  = 5'd25,
        ST_RANGE_STATUS_WAIT = 5'd26,
        ST_RANGE_REQ         = 5'd27,
        ST_RANGE_WAIT        = 5'd28,
        ST_CLEAR_REQ         = 5'd29,
        ST_CLEAR_WAIT        = 5'd30
    } state_t;

    state_t state;

    logic init_start_d;
    logic sample_start_d;

    logic [7:0]  status_l;
    logic [7:0]  range_status_l;
    logic        irq_ready_level_l;
    logic        discard_next_range_l;
    logic [TIMEOUT_W-1:0] wait_cnt;
    logic [STATUS_POLL_W-1:0] poll_cnt;
    logic [CONFIG_IDX_W-1:0] cfg_idx;

    function automatic logic [7:0] default_cfg_byte(input logic [CONFIG_IDX_W-1:0] idx);
        case (idx)
            7'd0:  default_cfg_byte = 8'h00;
            7'd1:  default_cfg_byte = 8'h00;
            7'd2:  default_cfg_byte = 8'h00;
            7'd3:  default_cfg_byte = 8'h01;
            7'd4:  default_cfg_byte = 8'h02;
            7'd5:  default_cfg_byte = 8'h00;
            7'd6:  default_cfg_byte = 8'h02;
            7'd7:  default_cfg_byte = 8'h08;
            7'd8:  default_cfg_byte = 8'h00;
            7'd9:  default_cfg_byte = 8'h08;
            7'd10: default_cfg_byte = 8'h10;
            7'd11: default_cfg_byte = 8'h01;
            7'd12: default_cfg_byte = 8'h01;
            7'd13: default_cfg_byte = 8'h00;
            7'd14: default_cfg_byte = 8'h00;
            7'd15: default_cfg_byte = 8'h00;
            7'd16: default_cfg_byte = 8'h00;
            7'd17: default_cfg_byte = 8'hFF;
            7'd18: default_cfg_byte = 8'h00;
            7'd19: default_cfg_byte = 8'h0F;
            7'd20: default_cfg_byte = 8'h00;
            7'd21: default_cfg_byte = 8'h00;
            7'd22: default_cfg_byte = 8'h00;
            7'd23: default_cfg_byte = 8'h00;
            7'd24: default_cfg_byte = 8'h00;
            7'd25: default_cfg_byte = 8'h20;
            7'd26: default_cfg_byte = 8'h0B;
            7'd27: default_cfg_byte = 8'h00;
            7'd28: default_cfg_byte = 8'h00;
            7'd29: default_cfg_byte = 8'h02;
            7'd30: default_cfg_byte = 8'h0A;
            7'd31: default_cfg_byte = 8'h21;
            7'd32: default_cfg_byte = 8'h00;
            7'd33: default_cfg_byte = 8'h00;
            7'd34: default_cfg_byte = 8'h05;
            7'd35: default_cfg_byte = 8'h00;
            7'd36: default_cfg_byte = 8'h00;
            7'd37: default_cfg_byte = 8'h00;
            7'd38: default_cfg_byte = 8'h00;
            7'd39: default_cfg_byte = 8'hC8;
            7'd40: default_cfg_byte = 8'h00;
            7'd41: default_cfg_byte = 8'h00;
            7'd42: default_cfg_byte = 8'h38;
            7'd43: default_cfg_byte = 8'hFF;
            7'd44: default_cfg_byte = 8'h01;
            7'd45: default_cfg_byte = 8'h00;
            7'd46: default_cfg_byte = 8'h08;
            7'd47: default_cfg_byte = 8'h00;
            7'd48: default_cfg_byte = 8'h00;
            7'd49: default_cfg_byte = 8'h01;
            7'd50: default_cfg_byte = 8'hCC;
            7'd51: default_cfg_byte = 8'h0F;
            7'd52: default_cfg_byte = 8'h01;
            7'd53: default_cfg_byte = 8'hF1;
            7'd54: default_cfg_byte = 8'h0D;
            7'd55: default_cfg_byte = 8'h01;
            7'd56: default_cfg_byte = 8'h68;
            7'd57: default_cfg_byte = 8'h00;
            7'd58: default_cfg_byte = 8'h80;
            7'd59: default_cfg_byte = 8'h08;
            7'd60: default_cfg_byte = 8'hB8;
            7'd61: default_cfg_byte = 8'h00;
            7'd62: default_cfg_byte = 8'h00;
            7'd63: default_cfg_byte = 8'h00;
            7'd64: default_cfg_byte = 8'h00;
            7'd65: default_cfg_byte = 8'h0F;
            7'd66: default_cfg_byte = 8'h89;
            7'd67: default_cfg_byte = 8'h00;
            7'd68: default_cfg_byte = 8'h00;
            7'd69: default_cfg_byte = 8'h00;
            7'd70: default_cfg_byte = 8'h00;
            7'd71: default_cfg_byte = 8'h00;
            7'd72: default_cfg_byte = 8'h00;
            7'd73: default_cfg_byte = 8'h00;
            7'd74: default_cfg_byte = 8'h01;
            7'd75: default_cfg_byte = 8'h0F;
            7'd76: default_cfg_byte = 8'h0D;
            7'd77: default_cfg_byte = 8'h0E;
            7'd78: default_cfg_byte = 8'h0E;
            7'd79: default_cfg_byte = 8'h00;
            7'd80: default_cfg_byte = 8'h00;
            7'd81: default_cfg_byte = 8'h02;
            7'd82: default_cfg_byte = 8'hC7;
            7'd83: default_cfg_byte = 8'hFF;
            7'd84: default_cfg_byte = 8'h9B;
            7'd85: default_cfg_byte = 8'h00;
            7'd86: default_cfg_byte = 8'h00;
            7'd87: default_cfg_byte = 8'h00;
            7'd88: default_cfg_byte = 8'h01;
            7'd89: default_cfg_byte = 8'h00;
            7'd90: default_cfg_byte = 8'h00;
            default: default_cfg_byte = 8'h00;
        endcase
    endfunction

    function automatic logic [7:0] normalize_range_status(input logic [7:0] raw_status);
        logic [7:0] rgst;
        begin
            rgst = raw_status & 8'h1F;
            case (rgst)
                8'd9:  normalize_range_status = 8'd0;
                8'd6:  normalize_range_status = 8'd1;
                8'd4:  normalize_range_status = 8'd2;
                8'd8:  normalize_range_status = 8'd3;
                8'd5:  normalize_range_status = 8'd4;
                8'd3:  normalize_range_status = 8'd5;
                8'd19: normalize_range_status = 8'd6;
                8'd7:  normalize_range_status = 8'd7;
                8'd12: normalize_range_status = 8'd9;
                8'd18: normalize_range_status = 8'd10;
                8'd22: normalize_range_status = 8'd11;
                8'd23: normalize_range_status = 8'd12;
                8'd13: normalize_range_status = 8'd13;
                default: normalize_range_status = 8'hFF;
            endcase
        end
    endfunction


    always_comb begin
        txn_req_valid        = 1'b0;
        txn_req_is_read      = 1'b0;
        txn_req_dev_addr     = TOF_ADDR;
        txn_req_reg_addr     = 16'h0000;
        txn_req_reg_addr_16b = 1'b1;
        txn_req_wr_data      = 8'h00;
        txn_req_rd_len       = 2'd1;

        case (state)
            ST_INIT_BOOT_REQ: begin
                txn_req_valid   = 1'b1;
                txn_req_is_read = 1'b1;
                txn_req_reg_addr = REG_BOOT_STATE;
            end

            ST_INIT_ID_REQ: begin
                txn_req_valid   = 1'b1;
                txn_req_is_read = 1'b1;
                txn_req_reg_addr = REG_IDENT_MODEL_ID;
                txn_req_rd_len  = 2'd2;
            end

            ST_INIT_CFG_REQ: begin
                txn_req_valid   = 1'b1;
                txn_req_reg_addr = REG_CONFIG_BASE + {{(16-CONFIG_IDX_W){1'b0}}, cfg_idx};
                txn_req_wr_data = default_cfg_byte(cfg_idx);
            end

            ST_INIT_POL_REQ: begin
                txn_req_valid   = 1'b1;
                txn_req_is_read = 1'b1;
                txn_req_reg_addr = REG_GPIO_HV_MUX_CTRL;
            end

            ST_INIT_START_REQ: begin
                txn_req_valid   = 1'b1;
                txn_req_reg_addr = REG_MODE_START;
                txn_req_wr_data = START_RANGING_BACK_TO_BACK;
            end

            ST_INIT_CLEAR_REQ: begin
                txn_req_valid   = 1'b1;
                txn_req_reg_addr = REG_INTERRUPT_CLEAR;
                txn_req_wr_data = CLEAR_RANGE_INTERRUPT;
            end

            ST_INIT_STOP_REQ: begin
                txn_req_valid   = 1'b1;
                txn_req_reg_addr = REG_MODE_START;
                txn_req_wr_data = STOP_RANGING;
            end

            ST_INIT_VHV_REQ: begin
                txn_req_valid   = 1'b1;
                txn_req_reg_addr = REG_VHV_LOOP_BOUND;
                txn_req_wr_data = VHV_LOOP_BOUND_INIT;
            end

            ST_INIT_PHASE_REQ: begin
                txn_req_valid   = 1'b1;
                txn_req_reg_addr = REG_PHASECAL_CONFIG;
                txn_req_wr_data = PHASECAL_INIT;
            end

            ST_INIT_FINAL_START_REQ: begin
                txn_req_valid   = 1'b1;
                txn_req_reg_addr = REG_MODE_START;
                txn_req_wr_data = START_RANGING_BACK_TO_BACK;
            end

            ST_STATUS_REQ: begin
                txn_req_valid   = 1'b1;
                txn_req_is_read = 1'b1;
                txn_req_reg_addr = REG_GPIO_STATUS;
            end

            ST_RANGE_STATUS_REQ: begin
                txn_req_valid   = 1'b1;
                txn_req_is_read = 1'b1;
                txn_req_reg_addr = REG_RANGE_STATUS;
            end

            ST_RANGE_REQ: begin
                txn_req_valid   = 1'b1;
                txn_req_is_read = 1'b1;
                txn_req_reg_addr = REG_RANGE_MM;
                txn_req_rd_len  = 2'd2;
            end

            ST_CLEAR_REQ: begin
                txn_req_valid   = 1'b1;
                txn_req_reg_addr = REG_INTERRUPT_CLEAR;
                txn_req_wr_data = CLEAR_RANGE_INTERRUPT;
            end

            default: begin
            end
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= ST_IDLE;
            init_start_d   <= 1'b0;
            sample_start_d <= 1'b0;
            status_l       <= 8'h00;
            range_status_l <= 8'hFF;
            irq_ready_level_l <= 1'b1;
            discard_next_range_l <= 1'b0;
            wait_cnt       <= '0;
            poll_cnt       <= '0;
            cfg_idx        <= '0;
            distance_mm    <= 16'h0000;
            distance_valid <= 1'b0;
            busy           <= 1'b0;
            done           <= 1'b0;
            error          <= 1'b0;
            nack           <= 1'b0;
        end else begin
            init_start_d   <= init_start;
            sample_start_d <= sample_start;
            done           <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy           <= 1'b0;
                    wait_cnt       <= '0;
                    poll_cnt       <= '0;

                    if (init_start && !init_start_d) begin
                        busy           <= 1'b1;
                        error          <= 1'b0;
                        nack           <= 1'b0;
                        distance_valid <= 1'b0;
                        range_status_l <= 8'hFF;
                        cfg_idx        <= '0;
                        state          <= ST_INIT_BOOT_REQ;
                    end else if (sample_start && !sample_start_d) begin
                        busy           <= 1'b1;
                        error          <= 1'b0;
                        nack           <= 1'b0;
                        distance_valid <= 1'b0;
                        range_status_l <= 8'hFF;
                        poll_cnt       <= '0;
                        state          <= ST_STATUS_REQ;
                    end
                end

                ST_INIT_BOOT_REQ,
                ST_INIT_ID_REQ,
                ST_INIT_CFG_REQ,
                ST_INIT_POL_REQ,
                ST_INIT_START_REQ,
                ST_INIT_CLEAR_REQ,
                ST_INIT_STOP_REQ,
                ST_INIT_VHV_REQ,
                ST_INIT_PHASE_REQ,
                ST_INIT_FINAL_START_REQ,
                ST_STATUS_REQ,
                ST_RANGE_STATUS_REQ,
                ST_RANGE_REQ,
                ST_CLEAR_REQ: begin
                    if (txn_req_ready) begin
                        wait_cnt <= '0;

                        case (state)
                            ST_INIT_BOOT_REQ:  state <= ST_INIT_BOOT_WAIT;
                            ST_INIT_ID_REQ:    state <= ST_INIT_ID_WAIT;
                            ST_INIT_CFG_REQ:   state <= ST_INIT_CFG_WAIT;
                            ST_INIT_POL_REQ:   state <= ST_INIT_POL_WAIT;
                            ST_INIT_START_REQ: state <= ST_INIT_START_WAIT;
                            ST_INIT_CLEAR_REQ: state <= ST_INIT_CLEAR_WAIT;
                            ST_INIT_STOP_REQ:  state <= ST_INIT_STOP_WAIT;
                            ST_INIT_VHV_REQ:   state <= ST_INIT_VHV_WAIT;
                            ST_INIT_PHASE_REQ: state <= ST_INIT_PHASE_WAIT;
                            ST_INIT_FINAL_START_REQ: state <= ST_INIT_FINAL_START_WAIT;
                            ST_STATUS_REQ:       state <= ST_STATUS_WAIT;
                            ST_RANGE_STATUS_REQ: state <= ST_RANGE_STATUS_WAIT;
                            ST_RANGE_REQ:        state <= ST_RANGE_WAIT;
                            ST_CLEAR_REQ:        state <= ST_CLEAR_WAIT;
                            default:           state <= ST_IDLE;
                        endcase
                    end else if (wait_cnt == (TIMEOUT_CLKS - 1)) begin
                        wait_cnt       <= '0;
                        busy           <= 1'b0;
                        done           <= 1'b1;
                        error          <= 1'b1;
                        distance_valid <= 1'b0;
                        state          <= ST_IDLE;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                ST_INIT_BOOT_WAIT: begin
                    if (txn_rsp_done) begin
                        wait_cnt <= '0;

                        if (txn_rsp_error) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            error <= 1'b1;
                            state <= ST_IDLE;
                        end else if (txn_rsp_nack) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            nack <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            status_l <= txn_rsp_rd_data[7:0];
                            state    <= ST_INIT_BOOT_EVAL;
                        end
                    end else if (wait_cnt == (TIMEOUT_CLKS - 1)) begin
                        wait_cnt <= '0;
                        busy     <= 1'b0;
                        done     <= 1'b1;
                        error    <= 1'b1;
                        state    <= ST_IDLE;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                ST_INIT_BOOT_EVAL: begin
                    if (status_l[0]) begin
                        poll_cnt <= '0;
                        state    <= ST_INIT_ID_REQ;
                    end else if (poll_cnt == (STATUS_POLL_LIMIT_E - 1)) begin
                        busy  <= 1'b0;
                        done  <= 1'b1;
                        error <= 1'b1;
                        state <= ST_IDLE;
                    end else begin
                        poll_cnt <= poll_cnt + {{(STATUS_POLL_W-1){1'b0}}, 1'b1};
                        state    <= ST_INIT_BOOT_REQ;
                    end
                end

                ST_INIT_ID_WAIT: begin
                    if (txn_rsp_done) begin
                        wait_cnt <= '0;

                        if (txn_rsp_error) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            error <= 1'b1;
                            state <= ST_IDLE;
                        end else if (txn_rsp_nack) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            nack <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            if ((txn_rsp_rd_data == EXPECT_MODEL_ID_0) ||
                                (txn_rsp_rd_data == EXPECT_MODEL_ID_1)) begin
                                cfg_idx <= '0;
                                state   <= ST_INIT_CFG_REQ;
                            end else begin
                                busy  <= 1'b0;
                                done  <= 1'b1;
                                error <= 1'b1;
                                state <= ST_IDLE;
                            end
                        end
                    end else if (wait_cnt == (TIMEOUT_CLKS - 1)) begin
                        wait_cnt <= '0;
                        busy     <= 1'b0;
                        done     <= 1'b1;
                        error    <= 1'b1;
                        state    <= ST_IDLE;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                ST_INIT_CFG_WAIT: begin
                    if (txn_rsp_done) begin
                        wait_cnt <= '0;

                        if (txn_rsp_error) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            error <= 1'b1;
                            state <= ST_IDLE;
                        end else if (txn_rsp_nack) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            nack <= 1'b1;
                            state <= ST_IDLE;
                        end else if (cfg_idx == (CONFIG_COUNT - 1)) begin
                            state <= ST_INIT_POL_REQ;
                        end else begin
                            cfg_idx <= cfg_idx + {{(CONFIG_IDX_W-1){1'b0}}, 1'b1};
                            state   <= ST_INIT_CFG_REQ;
                        end
                    end else if (wait_cnt == (TIMEOUT_CLKS - 1)) begin
                        wait_cnt <= '0;
                        busy     <= 1'b0;
                        done     <= 1'b1;
                        error    <= 1'b1;
                        state    <= ST_IDLE;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                ST_INIT_POL_WAIT: begin
                    if (txn_rsp_done) begin
                        wait_cnt <= '0;

                        if (txn_rsp_error) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            error <= 1'b1;
                            state <= ST_IDLE;
                        end else if (txn_rsp_nack) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            nack <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            // ST ULD derives the expected "data ready" level from bit4 of
                            // GPIO_HV_MUX__CTRL: IntPol = !((reg & 0x10) >> 4).
                            irq_ready_level_l <= ~txn_rsp_rd_data[4];
                            state             <= ST_INIT_START_REQ;
                        end
                    end else if (wait_cnt == (TIMEOUT_CLKS - 1)) begin
                        wait_cnt <= '0;
                        busy     <= 1'b0;
                        done     <= 1'b1;
                        error    <= 1'b1;
                        state    <= ST_IDLE;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                ST_INIT_START_WAIT: begin
                    if (txn_rsp_done) begin
                        wait_cnt <= '0;

                        if (txn_rsp_error) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            error <= 1'b1;
                            state <= ST_IDLE;
                        end else if (txn_rsp_nack) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            nack <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            state <= ST_INIT_CLEAR_REQ;
                        end
                    end else if (wait_cnt == (TIMEOUT_CLKS - 1)) begin
                        wait_cnt <= '0;
                        busy     <= 1'b0;
                        done     <= 1'b1;
                        error    <= 1'b1;
                        state    <= ST_IDLE;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                ST_INIT_CLEAR_WAIT: begin
                    if (txn_rsp_done) begin
                        wait_cnt <= '0;

                        if (txn_rsp_error) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            error <= 1'b1;
                            state <= ST_IDLE;
                        end else if (txn_rsp_nack) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            nack <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            state <= ST_INIT_STOP_REQ;
                        end
                    end else if (wait_cnt == (TIMEOUT_CLKS - 1)) begin
                        wait_cnt <= '0;
                        busy     <= 1'b0;
                        done     <= 1'b1;
                        error    <= 1'b1;
                        state    <= ST_IDLE;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                ST_INIT_STOP_WAIT: begin
                    if (txn_rsp_done) begin
                        wait_cnt <= '0;

                        if (txn_rsp_error) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            error <= 1'b1;
                            state <= ST_IDLE;
                        end else if (txn_rsp_nack) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            nack <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            state <= ST_INIT_VHV_REQ;
                        end
                    end else if (wait_cnt == (TIMEOUT_CLKS - 1)) begin
                        wait_cnt <= '0;
                        busy     <= 1'b0;
                        done     <= 1'b1;
                        error    <= 1'b1;
                        state    <= ST_IDLE;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                ST_INIT_VHV_WAIT: begin
                    if (txn_rsp_done) begin
                        wait_cnt <= '0;

                        if (txn_rsp_error) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            error <= 1'b1;
                            state <= ST_IDLE;
                        end else if (txn_rsp_nack) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            nack <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            state <= ST_INIT_PHASE_REQ;
                        end
                    end else if (wait_cnt == (TIMEOUT_CLKS - 1)) begin
                        wait_cnt <= '0;
                        busy     <= 1'b0;
                        done     <= 1'b1;
                        error    <= 1'b1;
                        state    <= ST_IDLE;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                ST_INIT_PHASE_WAIT: begin
                    if (txn_rsp_done) begin
                        wait_cnt <= '0;

                        if (txn_rsp_error) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            error <= 1'b1;
                            state <= ST_IDLE;
                        end else if (txn_rsp_nack) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            nack <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            state <= ST_INIT_FINAL_START_REQ;
                        end
                    end else if (wait_cnt == (TIMEOUT_CLKS - 1)) begin
                        wait_cnt <= '0;
                        busy     <= 1'b0;
                        done     <= 1'b1;
                        error    <= 1'b1;
                        state    <= ST_IDLE;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                ST_INIT_FINAL_START_WAIT: begin
                    if (txn_rsp_done) begin
                        wait_cnt <= '0;
                        busy     <= 1'b0;
                        done     <= 1'b1;

                        if (txn_rsp_error) begin
                            error <= 1'b1;
                        end else if (txn_rsp_nack) begin
                            nack <= 1'b1;
                        end else begin
                            // ST documents that the first range after (re)start is not
                            // guaranteed valid, so the host should discard it.
                            discard_next_range_l <= 1'b1;
                        end

                        state <= ST_IDLE;
                    end else if (wait_cnt == (TIMEOUT_CLKS - 1)) begin
                        wait_cnt <= '0;
                        busy     <= 1'b0;
                        done     <= 1'b1;
                        error    <= 1'b1;
                        state    <= ST_IDLE;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                ST_STATUS_WAIT: begin
                    if (txn_rsp_done) begin
                        wait_cnt <= '0;

                        if (txn_rsp_error) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            error <= 1'b1;
                            state <= ST_IDLE;
                        end else if (txn_rsp_nack) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            nack <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            status_l <= txn_rsp_rd_data[7:0];
                            state    <= ST_STATUS_EVAL;
                        end
                    end else if (wait_cnt == (TIMEOUT_CLKS - 1)) begin
                        wait_cnt       <= '0;
                        busy           <= 1'b0;
                        done           <= 1'b1;
                        error          <= 1'b1;
                        distance_valid <= 1'b0;
                        state          <= ST_IDLE;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                ST_STATUS_EVAL: begin
                    if (status_l[0] == irq_ready_level_l) begin
                        state <= ST_RANGE_STATUS_REQ;
                    end else if (poll_cnt == (STATUS_POLL_LIMIT_E - 1)) begin
                        busy           <= 1'b0;
                        done           <= 1'b1;
                        distance_valid <= 1'b0;
                        state          <= ST_IDLE;
                    end else begin
                        poll_cnt <= poll_cnt + {{(STATUS_POLL_W-1){1'b0}}, 1'b1};
                        state    <= ST_STATUS_REQ;
                    end
                end

                ST_RANGE_STATUS_WAIT: begin
                    if (txn_rsp_done) begin
                        wait_cnt <= '0;

                        if (txn_rsp_error) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            error <= 1'b1;
                            state <= ST_IDLE;
                        end else if (txn_rsp_nack) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            nack <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            range_status_l <= normalize_range_status(txn_rsp_rd_data[7:0]);
                            state          <= ST_RANGE_REQ;
                        end
                    end else if (wait_cnt == (TIMEOUT_CLKS - 1)) begin
                        wait_cnt       <= '0;
                        busy           <= 1'b0;
                        done           <= 1'b1;
                        error          <= 1'b1;
                        distance_valid <= 1'b0;
                        state          <= ST_IDLE;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                ST_RANGE_WAIT: begin
                    if (txn_rsp_done) begin
                        wait_cnt <= '0;

                        if (txn_rsp_error) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            error <= 1'b1;
                            state <= ST_IDLE;
                        end else if (txn_rsp_nack) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            nack <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            if (discard_next_range_l) begin
                                distance_valid        <= 1'b0;
                                discard_next_range_l  <= 1'b0;
                            end else if ((range_status_l == 8'd0) && (txn_rsp_rd_data != 16'h0000)) begin
                                distance_mm    <= txn_rsp_rd_data;
                                distance_valid <= 1'b1;
                            end else begin
                                distance_valid <= 1'b0;
                            end
                            state          <= ST_CLEAR_REQ;
                        end
                    end else if (wait_cnt == (TIMEOUT_CLKS - 1)) begin
                        wait_cnt       <= '0;
                        busy           <= 1'b0;
                        done           <= 1'b1;
                        error          <= 1'b1;
                        distance_valid <= 1'b0;
                        state          <= ST_IDLE;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                ST_CLEAR_WAIT: begin
                    if (txn_rsp_done) begin
                        wait_cnt <= '0;
                        busy     <= 1'b0;
                        done     <= 1'b1;

                        if (txn_rsp_error) begin
                            error          <= 1'b1;
                            distance_valid <= 1'b0;
                        end else if (txn_rsp_nack) begin
                            nack           <= 1'b1;
                            distance_valid <= 1'b0;
                        end

                        state <= ST_IDLE;
                    end else if (wait_cnt == (TIMEOUT_CLKS - 1)) begin
                        wait_cnt       <= '0;
                        busy           <= 1'b0;
                        done           <= 1'b1;
                        error          <= 1'b1;
                        distance_valid <= 1'b0;
                        state          <= ST_IDLE;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                default: begin
                    wait_cnt       <= '0;
                    poll_cnt       <= '0;
                    busy           <= 1'b0;
                    done           <= 1'b1;
                    error          <= 1'b1;
                    distance_valid <= 1'b0;
                    state          <= ST_IDLE;
                end
            endcase
        end
    end
endmodule

`default_nettype wire