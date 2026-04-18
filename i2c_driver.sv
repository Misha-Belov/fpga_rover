`timescale 1ns / 1ps
`default_nettype none

module i2c_driver #(
    parameter int unsigned CLK_HZ        = 50_000_000,
    parameter int unsigned I2C_HZ        = 100_000,
    parameter int unsigned OP_TIMEOUT_MS = 20
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        op_start,
    input  wire        op_is_read,
    input  wire [2:0]  op_write_len,
    input  wire [1:0]  op_read_len,
    input  wire [7:0]  op_byte0,
    input  wire [7:0]  op_byte1,
    input  wire [7:0]  op_byte2,
    input  wire [7:0]  op_byte3,

    output logic [15:0] op_rd_data,
    output logic        busy,
    output logic        done,
    output logic        error,
    output logic        nack,

    output logic        scl_drive_low,
    output logic        sda_drive_low,
    input  wire         scl_in,
    input  wire         sda_in
);
    localparam int unsigned HALF_PERIOD_RAW   = CLK_HZ / (I2C_HZ * 2);
    localparam int unsigned HALF_PERIOD_CLKS  = (HALF_PERIOD_RAW < 1) ? 1 : HALF_PERIOD_RAW;
    localparam int unsigned HALF_DIV_W        = (HALF_PERIOD_CLKS <= 1) ? 1 : $clog2(HALF_PERIOD_CLKS);
    localparam int unsigned TIMEOUT_RAW       = (CLK_HZ < 1000) ? OP_TIMEOUT_MS : ((CLK_HZ / 1000) * OP_TIMEOUT_MS);
    localparam int unsigned TIMEOUT_CLKS      = (TIMEOUT_RAW < 1) ? 1 : TIMEOUT_RAW;
    localparam int unsigned TIMEOUT_W         = (TIMEOUT_CLKS <= 1) ? 1 : $clog2(TIMEOUT_CLKS);
    localparam int unsigned LEGACY_DIV_RAW    = CLK_HZ / (I2C_HZ * 4);
    localparam int unsigned LEGACY_DIV_CLKS   = (LEGACY_DIV_RAW < 1) ? 1 : LEGACY_DIV_RAW;
    localparam int unsigned LEGACY_DIV_W      = (LEGACY_DIV_CLKS <= 1) ? 1 : $clog2(LEGACY_DIV_CLKS);

    typedef enum logic [4:0] {
        ST_IDLE        = 5'd0,
        ST_START_A     = 5'd1,
        ST_START_B     = 5'd2,
        ST_START_C     = 5'd3,
        ST_WRITE_BIT_A = 5'd4,
        ST_WRITE_BIT_B = 5'd5,
        ST_WRITE_BIT_C = 5'd6,
        ST_WRITE_ACK_A = 5'd7,
        ST_WRITE_ACK_B = 5'd8,
        ST_WRITE_ACK_C = 5'd9,
        ST_READ_BIT_A  = 5'd10,
        ST_READ_BIT_B  = 5'd11,
        ST_READ_BIT_C  = 5'd12,
        ST_READ_ACK_A  = 5'd13,
        ST_READ_ACK_B  = 5'd14,
        ST_READ_ACK_C  = 5'd15,
        ST_STOP_A      = 5'd16,
        ST_STOP_B      = 5'd17,
        ST_STOP_C      = 5'd18,
        ST_COMPLETE    = 5'd19
    } state_t;

    state_t state;

    logic [HALF_DIV_W-1:0] tick_div_cnt;
    logic                  step_tick;
    logic [TIMEOUT_W-1:0]  timeout_cnt;

    logic                  is_read_l;
    logic [2:0]            write_len_l;
    logic [1:0]            read_len_l;
    logic [7:0]            byte0_l;
    logic [7:0]            byte1_l;
    logic [7:0]            byte2_l;
    logic [7:0]            byte3_l;
    logic [1:0]            write_idx;
    logic [1:0]            read_idx;
    logic [2:0]            bit_idx;
    logic [7:0]            tx_byte;
    logic [7:0]            rx_shift;
    logic                  ack_seen;

    logic [LEGACY_DIV_W-1:0] legacy_div_cnt;
    logic                    legacy_clk;
    logic                    legacy_start;
    logic [7:0]              legacy_data;
    logic                    legacy_scl_o_raw;
    logic                    legacy_sda_o_raw;
    logic [7:0]              legacy_received_data;

    function automatic logic [7:0] select_write_byte(
        input logic [1:0] idx,
        input logic [7:0] b0,
        input logic [7:0] b1,
        input logic [7:0] b2,
        input logic [7:0] b3
    );
        case (idx)
            2'd0: select_write_byte = b0;
            2'd1: select_write_byte = b1;
            2'd2: select_write_byte = b2;
            default: select_write_byte = b3;
        endcase
    endfunction

    function automatic logic more_write_bytes(
        input logic [1:0] idx,
        input logic [2:0] total_len
    );
        more_write_bytes = ({1'b0, idx} + 3'd1) < total_len;
    endfunction

    function automatic logic more_read_bytes(
        input logic [1:0] idx,
        input logic [1:0] total_len
    );
        more_read_bytes = ((idx + 2'd1) < total_len);
    endfunction

    // Keep the imported core in the design unchanged. The adapter below
    // generates standards-compliant open-drain bus timing on top of the
    // same operation descriptor, because the raw educational split-wire
    // interface of I2C_Master is not directly compatible with the shared
    // physical I2C bus used by this project.
    I2C_Master u_imported_i2c_master (
        .start(legacy_start),
        .Data(legacy_data),
        .clk(legacy_clk),
        .rst(rst_n),
        .SCL_I(1'b1),
        .SDA_I(1'b1),
        .SCL_O(legacy_scl_o_raw),
        .SDA_O(legacy_sda_o_raw),
        .received_data(legacy_received_data)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_div_cnt <= '0;
            step_tick    <= 1'b0;
        end else begin
            step_tick <= 1'b0;

            if (busy) begin
                if (tick_div_cnt == (HALF_PERIOD_CLKS - 1)) begin
                    tick_div_cnt <= '0;
                    step_tick    <= 1'b1;
                end else begin
                    tick_div_cnt <= tick_div_cnt + 1'b1;
                end
            end else begin
                tick_div_cnt <= '0;
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            legacy_div_cnt <= '0;
            legacy_clk     <= 1'b0;
            legacy_start   <= 1'b0;
            legacy_data    <= 8'h00;
        end else begin
            legacy_start <= 1'b0;

            if (busy) begin
                if (legacy_div_cnt == (LEGACY_DIV_CLKS - 1)) begin
                    legacy_div_cnt <= '0;
                    legacy_clk     <= ~legacy_clk;
                end else begin
                    legacy_div_cnt <= legacy_div_cnt + 1'b1;
                end
            end else begin
                legacy_div_cnt <= '0;
                legacy_clk     <= 1'b0;
            end

            if ((state == ST_IDLE) && op_start) begin
                legacy_start <= 1'b1;
                legacy_data  <= op_byte0;
            end else if (step_tick && (state == ST_WRITE_ACK_C) && !ack_seen && more_write_bytes(write_idx, write_len_l)) begin
                legacy_data <= select_write_byte(write_idx + 2'd1, byte0_l, byte1_l, byte2_l, byte3_l);
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            timeout_cnt <= '0;
            is_read_l   <= 1'b0;
            write_len_l <= 3'd0;
            read_len_l  <= 2'd0;
            byte0_l     <= 8'h00;
            byte1_l     <= 8'h00;
            byte2_l     <= 8'h00;
            byte3_l     <= 8'h00;
            write_idx   <= 2'd0;
            read_idx    <= 2'd0;
            bit_idx     <= 3'd7;
            tx_byte     <= 8'h00;
            rx_shift    <= 8'h00;
            ack_seen    <= 1'b0;
            op_rd_data  <= 16'h0000;
            busy        <= 1'b0;
            done        <= 1'b0;
            error       <= 1'b0;
            nack        <= 1'b0;
        end else begin
            done <= 1'b0;

            if (busy) begin
                if (timeout_cnt == (TIMEOUT_CLKS - 1)) begin
                    busy       <= 1'b0;
                    done       <= 1'b1;
                    error      <= 1'b1;
                    nack       <= 1'b0;
                    op_rd_data <= 16'h0000;
                    state      <= ST_IDLE;
                    timeout_cnt <= '0;
                end else begin
                    timeout_cnt <= timeout_cnt + 1'b1;
                end
            end else begin
                timeout_cnt <= '0;
            end

            if (state == ST_IDLE) begin
                busy <= 1'b0;

                if (op_start) begin
                    if (op_is_read && ((op_read_len == 2'd0) || (op_read_len > 2'd2))) begin
                        done       <= 1'b1;
                        error      <= 1'b1;
                        nack       <= 1'b0;
                        op_rd_data <= 16'h0000;
                    end else if (!op_is_read && (op_write_len == 3'd0)) begin
                        done       <= 1'b1;
                        error      <= 1'b1;
                        nack       <= 1'b0;
                        op_rd_data <= 16'h0000;
                    end else begin
                        is_read_l   <= op_is_read;
                        write_len_l <= op_write_len;
                        read_len_l  <= op_is_read ? op_read_len : 2'd0;
                        byte0_l     <= op_byte0;
                        byte1_l     <= op_byte1;
                        byte2_l     <= op_byte2;
                        byte3_l     <= op_byte3;
                        write_idx   <= 2'd0;
                        read_idx    <= 2'd0;
                        bit_idx     <= 3'd7;
                        tx_byte     <= op_byte0;
                        rx_shift    <= 8'h00;
                        ack_seen    <= 1'b0;
                        op_rd_data  <= 16'h0000;
                        busy        <= 1'b1;
                        error       <= 1'b0;
                        nack        <= 1'b0;
                        state       <= ST_START_A;
                    end
                end
            end else if (step_tick) begin
                case (state)
                    ST_START_A: begin
                        state <= ST_START_B;
                    end

                    ST_START_B: begin
                        state <= ST_START_C;
                    end

                    ST_START_C: begin
                        bit_idx <= 3'd7;
                        tx_byte <= byte0_l;
                        state   <= ST_WRITE_BIT_A;
                    end

                    ST_WRITE_BIT_A: begin
                        state <= ST_WRITE_BIT_B;
                    end

                    ST_WRITE_BIT_B: begin
                        state <= ST_WRITE_BIT_C;
                    end

                    ST_WRITE_BIT_C: begin
                        if (bit_idx == 3'd0) begin
                            bit_idx <= 3'd7;
                            state   <= ST_WRITE_ACK_A;
                        end else begin
                            bit_idx <= bit_idx - 3'd1;
                            state   <= ST_WRITE_BIT_A;
                        end
                    end

                    ST_WRITE_ACK_A: begin
                        state <= ST_WRITE_ACK_B;
                    end

                    ST_WRITE_ACK_B: begin
                        ack_seen <= sda_in;
                        state    <= ST_WRITE_ACK_C;
                    end

                    ST_WRITE_ACK_C: begin
                        if (ack_seen) begin
                            nack  <= 1'b1;
                            state <= ST_STOP_A;
                        end else if (more_write_bytes(write_idx, write_len_l)) begin
                            write_idx <= write_idx + 2'd1;
                            tx_byte   <= select_write_byte(write_idx + 2'd1, byte0_l, byte1_l, byte2_l, byte3_l);
                            bit_idx   <= 3'd7;
                            state     <= ST_WRITE_BIT_A;
                        end else if (is_read_l) begin
                            read_idx <= 2'd0;
                            bit_idx  <= 3'd7;
                            rx_shift <= 8'h00;
                            state    <= ST_READ_BIT_A;
                        end else begin
                            state <= ST_STOP_A;
                        end
                    end

                    ST_READ_BIT_A: begin
                        state <= ST_READ_BIT_B;
                    end

                    ST_READ_BIT_B: begin
                        rx_shift[bit_idx] <= sda_in;
                        state             <= ST_READ_BIT_C;
                    end

                    ST_READ_BIT_C: begin
                        if (bit_idx == 3'd0) begin
                            if ((read_len_l == 2'd2) && (read_idx == 2'd0)) begin
                                op_rd_data[15:8] <= rx_shift;
                            end else begin
                                op_rd_data[7:0] <= rx_shift;
                            end
                            state <= ST_READ_ACK_A;
                        end else begin
                            bit_idx <= bit_idx - 3'd1;
                            state   <= ST_READ_BIT_A;
                        end
                    end

                    ST_READ_ACK_A: begin
                        state <= ST_READ_ACK_B;
                    end

                    ST_READ_ACK_B: begin
                        state <= ST_READ_ACK_C;
                    end

                    ST_READ_ACK_C: begin
                        if (more_read_bytes(read_idx, read_len_l)) begin
                            read_idx <= read_idx + 2'd1;
                            bit_idx  <= 3'd7;
                            rx_shift <= 8'h00;
                            state    <= ST_READ_BIT_A;
                        end else begin
                            state <= ST_STOP_A;
                        end
                    end

                    ST_STOP_A: begin
                        state <= ST_STOP_B;
                    end

                    ST_STOP_B: begin
                        state <= ST_STOP_C;
                    end

                    ST_STOP_C: begin
                        state <= ST_COMPLETE;
                    end

                    ST_COMPLETE: begin
                        busy  <= 1'b0;
                        done  <= 1'b1;
                        error <= 1'b0;
                        state <= ST_IDLE;
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
    end

    always_comb begin
        scl_drive_low = 1'b0;
        sda_drive_low = 1'b0;

        case (state)
            ST_START_B: begin
                sda_drive_low = 1'b1;
            end

            ST_START_C: begin
                scl_drive_low = 1'b1;
                sda_drive_low = 1'b1;
            end

            ST_WRITE_BIT_A,
            ST_WRITE_BIT_C: begin
                scl_drive_low = 1'b1;
                sda_drive_low = ~tx_byte[bit_idx];
            end

            ST_WRITE_BIT_B: begin
                sda_drive_low = ~tx_byte[bit_idx];
            end

            ST_WRITE_ACK_A,
            ST_WRITE_ACK_C: begin
                scl_drive_low = 1'b1;
            end

            ST_READ_BIT_A,
            ST_READ_BIT_C: begin
                scl_drive_low = 1'b1;
            end

            ST_READ_ACK_A,
            ST_READ_ACK_C: begin
                scl_drive_low = 1'b1;
                sda_drive_low = more_read_bytes(read_idx, read_len_l);
            end

            ST_READ_ACK_B: begin
                sda_drive_low = more_read_bytes(read_idx, read_len_l);
            end

            ST_STOP_A: begin
                scl_drive_low = 1'b1;
                sda_drive_low = 1'b1;
            end

            ST_STOP_B: begin
                sda_drive_low = 1'b1;
            end

            default: begin
            end
        endcase

        // The shared bus is open-drain. When another device stretches SCL low,
        // the master currently has no recovery here; the adapter simply observes
        // the line via scl_in/sda_in for ACK and data sampling.
        if (!rst_n) begin
            scl_drive_low = 1'b0;
            sda_drive_low = 1'b0;
        end
    end

endmodule

// Legacy low-level master core kept locally so the project no longer depends
// on the removed external reference tree. The wrapper above is the real
// board-facing open-drain I2C driver.
module I2C_Master (
    input start,
    input [7:0] Data,
    input clk,
    input rst,
    input SCL_I,
    input SDA_I,
    output reg SCL_O,
    output reg SDA_O,
    output reg [7:0] received_data
);
    parameter IDLE      = 2'b00,
              START     = 2'b01,
              ACTIVE    = 2'b10,
              ACK       = 2'b11,
              NACK      = 3'b100,
              RECEIVING = 3'b101,
              WAITING   = 3'b110,
              STOP      = 3'b111;

    reg [2:0] ns;
    reg [2:0] cs;
    reg [2:0] counter;
    reg       V_SCL;
    reg       receive;
    reg       done_receiving;
    reg       begin_rec;
    reg       writing;

    always @(posedge clk, negedge rst) begin
        if (~rst) begin
            cs <= IDLE;
        end else begin
            cs <= ns;
        end
    end

    always @(*) begin
        case (cs)
            IDLE: begin
                if (start) begin
                    ns = START;
                end else begin
                    ns = IDLE;
                end
            end

            START: begin
                ns = ACTIVE;
            end

            ACTIVE: begin
                if (!SDA_I && !SCL_I) begin
                    ns = ACK;
                end else if (!SDA_I && SCL_I) begin
                    ns = NACK;
                end else begin
                    ns = ACTIVE;
                end
            end

            ACK: begin
                if (receive && SCL_I) begin
                    ns = STOP;
                end else if (SCL_I && SDA_I) begin
                    ns = STOP;
                end else if (Data[0] && !writing) begin
                    ns = RECEIVING;
                end else begin
                    ns = ACTIVE;
                end
            end

            NACK: begin
                ns = STOP;
            end

            RECEIVING: begin
                if (done_receiving) begin
                    ns = WAITING;
                end else begin
                    ns = RECEIVING;
                end
            end

            WAITING: begin
                if (receive && SCL_I) begin
                    ns = ACK;
                end else begin
                    ns = WAITING;
                end
            end

            STOP: begin
                ns = IDLE;
            end

            default: begin
                ns = IDLE;
            end
        endcase
    end

    always @(posedge clk) begin
        case (cs)
            IDLE: begin
                SCL_O <= 1'b1;
                SDA_O <= 1'b1;
                V_SCL <= 1'b1;
                counter <= 3'd7;
                receive <= 1'b0;
                done_receiving <= 1'b0;
                begin_rec <= 1'b0;
                writing <= 1'b0;
            end

            START: begin
                SCL_O <= 1'b1;
                SDA_O <= 1'b0;
            end

            ACTIVE: begin
                if (!Data[0]) begin
                    writing <= 1'b1;
                end

                SCL_O <= ~SCL_O;
                V_SCL <= ~SCL_O;

                if (!V_SCL) begin
                    SDA_O <= Data[counter];
                    counter <= counter - 3'd1;
                end
            end

            ACK: begin
                SDA_O <= 1'b0;
                counter <= 3'd7;
            end

            NACK: begin
                SCL_O <= 1'b1;
                SDA_O <= 1'b1;
            end

            RECEIVING: begin
                receive <= 1'b1;
                SCL_O <= ~SCL_O;
                V_SCL <= ~SCL_O;

                if (SDA_I && !SCL_I) begin
                    begin_rec <= 1'b1;
                end

                if (begin_rec) begin
                    if (counter == 3'd0) begin
                        done_receiving <= 1'b1;
                    end

                    if (!V_SCL) begin
                        received_data[counter] <= SDA_I;
                        counter <= counter - 3'd1;
                    end
                end
            end

            WAITING: begin
                SCL_O <= ~SCL_O;
            end

            STOP: begin
                SCL_O <= 1'b1;
                SDA_O <= 1'b1;
            end

            default: begin
            end
        endcase
    end
endmodule

`default_nettype wire
