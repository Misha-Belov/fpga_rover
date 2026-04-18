`timescale 1ns / 1ps
`default_nettype none

module rover (
    input  wire       CLK_50M,
    input  wire       RST_N,
    inout  wire       I2C_SCL,
    inout  wire       I2C_SDA,
    output wire [7:0] SEG,
    output wire [3:0] DIG,
    output wire       LED1,
    output wire       LED2,
    output wire       LED3,
    output wire       LED4
);
    parameter CLK_HZ           = 50000000;
    parameter I2C_HZ           = 100000;
    parameter TOF_SENSOR_KIND  = 1;   // 1 = VL53L0X
    parameter INIT_DELAY_MS    = 10;
    parameter SAMPLE_PERIOD_MS = 100;

    parameter INIT_DELAY_CLKS    = (CLK_HZ / 1000) * INIT_DELAY_MS;
    parameter SAMPLE_PERIOD_CLKS = (CLK_HZ / 1000) * SAMPLE_PERIOD_MS;

    // ceil(log2()) for counters
    function integer clog2;
        input integer value;
        integer i;
        begin
            value = value - 1;
            for (i = 0; value > 0; i = i + 1)
                value = value >> 1;
            clog2 = i;
        end
    endfunction

    localparam INIT_CNT_W   = (INIT_DELAY_CLKS <= 1)    ? 1 : clog2(INIT_DELAY_CLKS);
    localparam SAMPLE_CNT_W = (SAMPLE_PERIOD_CLKS <= 1) ? 1 : clog2(SAMPLE_PERIOD_CLKS);

    wire scl_drive_low;
    wire sda_drive_low;

    reg         tof_init_start;
    reg         tof_sample_start;
    wire [15:0] tof_distance_mm;
    wire        tof_distance_valid;
    wire        tof_busy;
    wire        tof_done;
    wire        tof_error;
    wire        tof_nack;

    wire        tof_txn_req_valid;
    wire        tof_txn_req_ready;
    wire        tof_txn_req_is_read;
    wire [6:0]  tof_txn_req_dev_addr;
    wire [15:0] tof_txn_req_reg_addr;
    wire        tof_txn_req_reg_addr_16b;
    wire [7:0]  tof_txn_req_wr_data;
    wire [1:0]  tof_txn_req_rd_len;

    wire        txn_core_start;
    wire        txn_core_is_read;
    wire [6:0]  txn_core_dev_addr;
    wire [15:0] txn_core_reg_addr;
    wire        txn_core_reg_addr_16b;
    wire [7:0]  txn_core_wr_data;
    wire [1:0]  txn_core_rd_len;
    wire [15:0] txn_core_rd_data;
    wire        txn_core_busy;
    wire        txn_core_done;
    wire        txn_core_error;
    wire        txn_core_nack;

    reg [15:0] display_value;
    reg [INIT_CNT_W-1:0] init_cnt;
    reg [SAMPLE_CNT_W-1:0] sample_cnt;
    reg init_armed;
    reg init_done;
    reg [24:0] heartbeat_cnt;
    reg [22:0] valid_stretch;

    wire heartbeat;

    assign I2C_SCL = scl_drive_low ? 1'b0 : 1'bz;
    assign I2C_SDA = sda_drive_low ? 1'b0 : 1'bz;

    assign tof_txn_req_ready      = ~txn_core_busy;
    assign txn_core_start         = tof_txn_req_valid & tof_txn_req_ready;
    assign txn_core_is_read       = tof_txn_req_is_read;
    assign txn_core_dev_addr      = tof_txn_req_dev_addr;
    assign txn_core_reg_addr      = tof_txn_req_reg_addr;
    assign txn_core_reg_addr_16b  = tof_txn_req_reg_addr_16b;
    assign txn_core_wr_data       = tof_txn_req_wr_data;
    assign txn_core_rd_len        = tof_txn_req_rd_len;

    i2c_master #(
        .CLK_HZ(CLK_HZ),
        .I2C_HZ(I2C_HZ)
    ) u_i2c_master (
        .clk(CLK_50M),
        .rst_n(RST_N),
        .start(txn_core_start),
        .is_read(txn_core_is_read),
        .dev_addr(txn_core_dev_addr),
        .reg_addr(txn_core_reg_addr),
        .reg_addr_16b(txn_core_reg_addr_16b),
        .wr_data(txn_core_wr_data),
        .rd_len(txn_core_rd_len),
        .rd_data(txn_core_rd_data),
        .busy(txn_core_busy),
        .done(txn_core_done),
        .error(txn_core_error),
        .nack(txn_core_nack),
        .scl_drive_low(scl_drive_low),
        .sda_drive_low(sda_drive_low),
        .scl_in(I2C_SCL),
        .sda_in(I2C_SDA)
    );

    tof_ctrl #(
        .SENSOR_KIND(TOF_SENSOR_KIND),
        .CLK_HZ(CLK_HZ)
    ) u_tof_ctrl (
        .clk(CLK_50M),
        .rst_n(RST_N),
        .init_start(tof_init_start),
        .sample_start(tof_sample_start),
        .distance_mm(tof_distance_mm),
        .distance_valid(tof_distance_valid),
        .busy(tof_busy),
        .done(tof_done),
        .error(tof_error),
        .nack(tof_nack),
        .txn_req_valid(tof_txn_req_valid),
        .txn_req_ready(tof_txn_req_ready),
        .txn_req_is_read(tof_txn_req_is_read),
        .txn_req_dev_addr(tof_txn_req_dev_addr),
        .txn_req_reg_addr(tof_txn_req_reg_addr),
        .txn_req_reg_addr_16b(tof_txn_req_reg_addr_16b),
        .txn_req_wr_data(tof_txn_req_wr_data),
        .txn_req_rd_len(tof_txn_req_rd_len),
        .txn_rsp_done(txn_core_done),
        .txn_rsp_error(txn_core_error),
        .txn_rsp_nack(txn_core_nack),
        .txn_rsp_rd_data(txn_core_rd_data)
    );

    seg7_display u_seg7_display (
        .clk(CLK_50M),
        .rst_n(RST_N),
        .value(display_value),
        .show_err(tof_error | tof_nack),
        .seg(SEG),
        .dig(DIG)
    );

    always @(posedge CLK_50M or negedge RST_N) begin
        if (!RST_N) begin
            init_cnt         <= {INIT_CNT_W{1'b0}};
            sample_cnt       <= {SAMPLE_CNT_W{1'b0}};
            init_armed       <= 1'b1;
            init_done        <= 1'b0;
            tof_init_start   <= 1'b0;
            tof_sample_start <= 1'b0;
            display_value    <= 16'd0;
            heartbeat_cnt    <= 25'd0;
            valid_stretch    <= 23'd0;
        end else begin
            tof_init_start   <= 1'b0;
            tof_sample_start <= 1'b0;

            heartbeat_cnt <= heartbeat_cnt + 25'd1;

            if (tof_distance_valid) begin
                display_value <= tof_distance_mm;
                valid_stretch <= {23{1'b1}};
            end else if (valid_stretch != 23'd0) begin
                valid_stretch <= valid_stretch - 23'd1;
            end

            if (init_armed) begin
                if (init_cnt == INIT_DELAY_CLKS - 1) begin
                    if (!tof_busy) begin
                        tof_init_start <= 1'b1;
                        init_armed     <= 1'b0;
                        init_cnt       <= {INIT_CNT_W{1'b0}};
                    end
                end else begin
                    init_cnt <= init_cnt + {{(INIT_CNT_W-1){1'b0}},1'b1};
                end
            end

            if (tof_done && !tof_error && !tof_nack && !init_done)
                init_done <= 1'b1;

            if (init_done && !tof_busy) begin
                if (sample_cnt == SAMPLE_PERIOD_CLKS - 1) begin
                    tof_sample_start <= 1'b1;
                    sample_cnt       <= {SAMPLE_CNT_W{1'b0}};
                end else begin
                    sample_cnt <= sample_cnt + {{(SAMPLE_CNT_W-1){1'b0}},1'b1};
                end
            end else begin
                sample_cnt <= {SAMPLE_CNT_W{1'b0}};
            end
        end
    end

    assign heartbeat = heartbeat_cnt[24];

    assign LED1 = ~heartbeat;
    assign LED2 = ~(valid_stretch != 23'd0);
    assign LED3 = ~(tof_error | tof_nack);
    assign LED4 = 1'b1;

endmodule

`default_nettype wire