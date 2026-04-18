`timescale 1ns / 1ps
`default_nettype none

module vl5310x_ctrl #(
    parameter int unsigned CLK_HZ            = 50_000_000,
    parameter int unsigned TIMEOUT_MS        = 20,
    parameter int unsigned STATUS_POLL_LIMIT = 128
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
    localparam logic [6:0] TOF_ADDR                = 7'h29; // I2C-адрес датчика VL53L0X.
    localparam logic [7:0] REG_SYSRANGE_START      = 8'h00; // Регистр запуска single-shot измерения.
    localparam logic [7:0] REG_SYSTEM_SEQUENCE_CFG = 8'h01; // Регистр включения стандартной последовательности измерения.
    localparam logic [7:0] REG_SYSTEM_INT_CFG_GPIO = 8'h0A; // Регистр выбора источника interrupt "new sample ready".
    localparam logic [7:0] REG_INTERRUPT_CLEAR     = 8'h0B; // Регистр очистки interrupt после чтения результата.
    localparam logic [7:0] REG_RESULT_INT_STATUS   = 8'h13; // Регистр статуса готовности результата.
    localparam logic [7:0] REG_RESULT_RANGE_MM_MSB = 8'h1E; // Старший байт регистра результата расстояния.
    localparam logic [7:0] REG_GPIO_HV_MUX_ACTIVE  = 8'h84; // Регистр полярности GPIO interrupt-вывода.
    localparam logic [7:0] REG_IDENT_C0            = 8'hC0; // Первый ID-регистр датчика VL53L0X.
    localparam logic [7:0] REG_IDENT_C1            = 8'hC1; // Второй ID-регистр датчика VL53L0X.
    localparam logic [7:0] REG_IDENT_C2            = 8'hC2; // Третий ID-регистр датчика VL53L0X.
    localparam logic [7:0] START_CONTINUOUS        = 8'h02; // Минимальный запуск непрерывного измерения, как в старой архитектуре проекта.
    localparam logic [7:0] SYSTEM_SEQUENCE_ALL     = 8'hFF; // Включение стандартной sequence-конфигурации из ST DataInit.
    localparam logic [7:0] INT_NEW_SAMPLE_READY    = 8'h04; // Interrupt "new sample ready", как в ST StaticInit.
    localparam logic [7:0] CLEAR_RANGE_INTERRUPT   = 8'h01; // Значение для очистки прерывания по завершенному измерению.
    localparam logic [7:0] EXPECT_ID_C0            = 8'hEE; // Ожидаемое значение первого ID-регистра.
    localparam logic [7:0] EXPECT_ID_C1            = 8'hAA; // Ожидаемое значение второго ID-регистра.
    localparam logic [7:0] EXPECT_ID_C2            = 8'h10; // Ожидаемое значение третьего ID-регистра.

    localparam int unsigned CLKS_PER_MS            = (CLK_HZ < 1000) ? 1 : (CLK_HZ / 1000); // Количество тактов `clk` в одной миллисекунде.
    localparam int unsigned TIMEOUT_CLKS_R         = CLKS_PER_MS * TIMEOUT_MS; // Сырой таймаут ожидания ответа от `i2c_master`.
    localparam int unsigned TIMEOUT_CLKS           = (TIMEOUT_CLKS_R < 1) ? 1 : TIMEOUT_CLKS_R; // Нормализованный таймаут ожидания транзакции.
    localparam int unsigned TIMEOUT_W              = (TIMEOUT_CLKS <= 1) ? 1 : $clog2(TIMEOUT_CLKS); // Разрядность счетчика таймаута транзакции.
    localparam int unsigned STATUS_POLL_LIMIT_E    = (STATUS_POLL_LIMIT < 1) ? 1 : STATUS_POLL_LIMIT; // Эффективный лимит опросов статуса готовности измерения.
    localparam int unsigned STATUS_POLL_W          = (STATUS_POLL_LIMIT_E <= 1) ? 1 : $clog2(STATUS_POLL_LIMIT_E); // Разрядность счетчика опросов статуса измерения.

    typedef enum logic [4:0] {
        ST_IDLE          = 5'd0,
        ST_INIT_ID0_REQ  = 5'd1,
        ST_INIT_ID0_WAIT = 5'd2,
        ST_INIT_ID1_REQ  = 5'd3,
        ST_INIT_ID1_WAIT = 5'd4,
        ST_INIT_ID2_REQ  = 5'd5,
        ST_INIT_ID2_WAIT = 5'd6,
        ST_INIT_SEQ_REQ  = 5'd7,
        ST_INIT_SEQ_WAIT = 5'd8,
        ST_INIT_INT_REQ  = 5'd9,
        ST_INIT_INT_WAIT = 5'd10,
        ST_INIT_GPIO_RD_REQ  = 5'd11,
        ST_INIT_GPIO_RD_WAIT = 5'd12,
        ST_INIT_GPIO_WR_REQ  = 5'd13,
        ST_INIT_GPIO_WR_WAIT = 5'd14,
        ST_INIT_CLEAR_REQ    = 5'd15,
        ST_INIT_CLEAR_WAIT   = 5'd16,
        ST_START_REQ     = 5'd17,
        ST_START_WAIT    = 5'd18,
        ST_STATUS_REQ    = 5'd19,
        ST_STATUS_WAIT   = 5'd20,
        ST_STATUS_EVAL   = 5'd21,
        ST_RANGE_REQ     = 5'd22,
        ST_RANGE_WAIT    = 5'd23,
        ST_CLEAR_REQ     = 5'd24,
        ST_CLEAR_WAIT    = 5'd25
    } state_t;

    state_t state; // Текущее состояние автомата init и single-shot измерения VL53L0X.

    logic init_start_d; // Задержанная версия `init_start` для детекции фронта инициализации.
    logic sample_start_d; // Задержанная версия `sample_start` для детекции фронта измерения.
    logic [7:0] id_c0_l; // Защелкнутое значение первого ID-регистра датчика.
    logic [7:0] id_c1_l; // Защелкнутое значение второго ID-регистра датчика.
    logic [7:0] gpio_hv_mux_l; // Защелкнутый байт GPIO-конфигурации для read-modify-write.
    logic [7:0] status_l; // Защелкнутое значение регистра статуса готовности измерения.

    logic       req_valid_int; // Внутренний признак активного запроса на I2C-транзакцию к VL53L0X.
    logic       req_is_read_int; // Внутренний тип текущей транзакции к VL53L0X: чтение или запись.
    logic [7:0] req_reg_addr; // Адрес регистра VL53L0X для текущего запроса.
    logic [7:0] req_wr_data; // Данные записи VL53L0X для текущего запроса.
    logic [1:0] req_rd_len; // Длина чтения VL53L0X для текущего запроса.
    logic [TIMEOUT_W-1:0] wait_cnt; // Счетчик ожидания handshake и ответа от `i2c_master`.
    logic [STATUS_POLL_W-1:0] status_poll_cnt; // Счетчик числа опросов регистра статуса измерения.

    always_comb begin
        req_valid_int   = 1'b0;
        req_is_read_int = 1'b0;
        req_reg_addr    = 8'h00;
        req_wr_data     = 8'h00;
        req_rd_len      = 2'd1;

        case (state)
            ST_INIT_ID0_REQ: begin
                req_valid_int   = 1'b1;
                req_is_read_int = 1'b1;
                req_reg_addr    = REG_IDENT_C0;
            end

            ST_INIT_ID1_REQ: begin
                req_valid_int   = 1'b1;
                req_is_read_int = 1'b1;
                req_reg_addr    = REG_IDENT_C1;
            end

            ST_INIT_ID2_REQ: begin
                req_valid_int   = 1'b1;
                req_is_read_int = 1'b1;
                req_reg_addr    = REG_IDENT_C2;
            end

            ST_INIT_SEQ_REQ: begin
                req_valid_int   = 1'b1;
                req_is_read_int = 1'b0;
                req_reg_addr    = REG_SYSTEM_SEQUENCE_CFG;
                req_wr_data     = SYSTEM_SEQUENCE_ALL;
            end

            ST_INIT_INT_REQ: begin
                req_valid_int   = 1'b1;
                req_is_read_int = 1'b0;
                req_reg_addr    = REG_SYSTEM_INT_CFG_GPIO;
                req_wr_data     = INT_NEW_SAMPLE_READY;
            end

            ST_INIT_GPIO_RD_REQ: begin
                req_valid_int   = 1'b1;
                req_is_read_int = 1'b1;
                req_reg_addr    = REG_GPIO_HV_MUX_ACTIVE;
            end

            ST_INIT_GPIO_WR_REQ: begin
                req_valid_int   = 1'b1;
                req_is_read_int = 1'b0;
                req_reg_addr    = REG_GPIO_HV_MUX_ACTIVE;
                req_wr_data     = gpio_hv_mux_l & 8'hEF;
            end

            ST_INIT_CLEAR_REQ: begin
                req_valid_int   = 1'b1;
                req_is_read_int = 1'b0;
                req_reg_addr    = REG_INTERRUPT_CLEAR;
                req_wr_data     = CLEAR_RANGE_INTERRUPT;
            end

            ST_START_REQ: begin
                req_valid_int   = 1'b1;
                req_is_read_int = 1'b0;
                req_reg_addr    = REG_SYSRANGE_START;
                req_wr_data     = START_CONTINUOUS;
            end

            ST_STATUS_REQ: begin
                req_valid_int   = 1'b1;
                req_is_read_int = 1'b1;
                req_reg_addr    = REG_RESULT_INT_STATUS;
            end

            ST_RANGE_REQ: begin
                req_valid_int   = 1'b1;
                req_is_read_int = 1'b1;
                req_reg_addr    = REG_RESULT_RANGE_MM_MSB;
                req_rd_len      = 2'd2;
            end

            ST_CLEAR_REQ: begin
                req_valid_int   = 1'b1;
                req_is_read_int = 1'b0;
                req_reg_addr    = REG_INTERRUPT_CLEAR;
                req_wr_data     = CLEAR_RANGE_INTERRUPT;
            end

            default: begin
            end
        endcase
    end

    assign txn_req_valid    = req_valid_int;
    assign txn_req_is_read  = req_is_read_int;
    assign txn_req_dev_addr     = TOF_ADDR;
    assign txn_req_reg_addr     = {8'h00, req_reg_addr};
    assign txn_req_reg_addr_16b = 1'b0;
    assign txn_req_wr_data      = req_wr_data;
    assign txn_req_rd_len       = req_rd_len;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= ST_IDLE;
            init_start_d    <= 1'b0;
            sample_start_d  <= 1'b0;
            id_c0_l         <= 8'h00;
            id_c1_l         <= 8'h00;
            gpio_hv_mux_l   <= 8'h00;
            status_l        <= 8'h00;
            wait_cnt        <= '0;
            status_poll_cnt <= '0;
            distance_mm     <= 16'h0000;
            distance_valid  <= 1'b0;
            busy            <= 1'b0;
            done            <= 1'b0;
            error           <= 1'b0;
            nack            <= 1'b0;
        end else begin
            init_start_d   <= init_start;
            sample_start_d <= sample_start;
            done           <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy            <= 1'b0;
                    wait_cnt        <= '0;
                    status_poll_cnt <= '0;

                    if (init_start && !init_start_d) begin
                        busy           <= 1'b1;
                        error          <= 1'b0;
                        nack           <= 1'b0;
                        distance_valid <= 1'b0;
                        state          <= ST_INIT_ID0_REQ;
                    end else if (sample_start && !sample_start_d) begin
                        busy           <= 1'b1;
                        error          <= 1'b0;
                        nack           <= 1'b0;
                        distance_valid <= 1'b0;
                        status_poll_cnt <= '0;
                        state          <= ST_STATUS_REQ;
                    end
                end

                ST_INIT_ID0_REQ,
                ST_INIT_ID1_REQ,
                ST_INIT_ID2_REQ,
                ST_INIT_SEQ_REQ,
                ST_INIT_INT_REQ,
                ST_INIT_GPIO_RD_REQ,
                ST_INIT_GPIO_WR_REQ,
                ST_INIT_CLEAR_REQ,
                ST_START_REQ,
                ST_STATUS_REQ,
                ST_RANGE_REQ,
                ST_CLEAR_REQ: begin
                    if (txn_req_ready) begin
                        wait_cnt <= '0;

                        case (state)
                            ST_INIT_ID0_REQ: state <= ST_INIT_ID0_WAIT;
                            ST_INIT_ID1_REQ: state <= ST_INIT_ID1_WAIT;
                            ST_INIT_ID2_REQ: state <= ST_INIT_ID2_WAIT;
                            ST_INIT_SEQ_REQ: state <= ST_INIT_SEQ_WAIT;
                            ST_INIT_INT_REQ: state <= ST_INIT_INT_WAIT;
                            ST_INIT_GPIO_RD_REQ: state <= ST_INIT_GPIO_RD_WAIT;
                            ST_INIT_GPIO_WR_REQ: state <= ST_INIT_GPIO_WR_WAIT;
                            ST_INIT_CLEAR_REQ: state <= ST_INIT_CLEAR_WAIT;
                            ST_START_REQ:    state <= ST_START_WAIT;
                            ST_STATUS_REQ:   state <= ST_STATUS_WAIT;
                            ST_RANGE_REQ:    state <= ST_RANGE_WAIT;
                            ST_CLEAR_REQ:    state <= ST_CLEAR_WAIT;
                            default:         state <= ST_IDLE;
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

                ST_INIT_ID0_WAIT: begin
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
                            id_c0_l <= txn_rsp_rd_data[7:0];
                            state   <= ST_INIT_ID1_REQ;
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

                ST_INIT_ID1_WAIT: begin
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
                            id_c1_l <= txn_rsp_rd_data[7:0];
                            state   <= ST_INIT_ID2_REQ;
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

                ST_INIT_ID2_WAIT: begin
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
                            if ((id_c0_l == EXPECT_ID_C0) &&
                                (id_c1_l == EXPECT_ID_C1) &&
                                (txn_rsp_rd_data[7:0] == EXPECT_ID_C2)) begin
                                state <= ST_INIT_SEQ_REQ;
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

                ST_INIT_SEQ_WAIT: begin
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
                            state <= ST_INIT_INT_REQ;
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

                ST_INIT_INT_WAIT: begin
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
                            state <= ST_INIT_GPIO_RD_REQ;
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

                ST_INIT_GPIO_RD_WAIT: begin
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
                            gpio_hv_mux_l <= txn_rsp_rd_data[7:0];
                            state         <= ST_INIT_GPIO_WR_REQ;
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

                ST_INIT_GPIO_WR_WAIT: begin
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
                            state <= ST_START_REQ;
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

                ST_START_WAIT: begin
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
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            state <= ST_IDLE;
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
                    if (status_l[2:0] != 3'b000) begin
                        state <= ST_RANGE_REQ;
                    end else if (status_poll_cnt == (STATUS_POLL_LIMIT_E - 1)) begin
                        busy           <= 1'b0;
                        done           <= 1'b1;
                        error          <= 1'b1;
                        distance_valid <= 1'b0;
                        state          <= ST_IDLE;
                    end else begin
                        status_poll_cnt <= status_poll_cnt + 1'b1;
                        state           <= ST_STATUS_REQ;
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
                            distance_mm    <= txn_rsp_rd_data;
                            distance_valid <= 1'b1;
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
                    status_poll_cnt <= '0;
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
