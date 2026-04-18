module rover(
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
    parameter CLK_HZ = 50000000;
    parameter I2C_HZ = 100000;
    parameter [6:0] DEV_ADDR = 7'h38;
    parameter signed [7:0] MOTOR_SPEED = 8'sd100;

    reg [31:0] powerup_cnt;
    wire powerup_done = (powerup_cnt >= 32'd50000000); // 1s
    always @(posedge CLK_50M or negedge RST_N) begin
        if (!RST_N) powerup_cnt <= 32'd0;
        else if (!powerup_done) powerup_cnt <= powerup_cnt + 1'b1;
    end

    // i2c master interface
    reg        txn_start;
    reg        txn_is_read;
    reg [6:0]  txn_dev_addr;
    reg [15:0] txn_reg_addr;
    reg        txn_reg_addr_16b;
    reg [7:0]  txn_wr_data;
    reg [7:0]  txn_rd_len;
    wire [31:0] txn_rd_data;
    wire       txn_busy;
    wire       txn_done;
    wire       txn_error;
    wire       txn_nack;
    wire       scl_drive_low;
    wire       sda_drive_low;

    assign I2C_SCL = scl_drive_low ? 1'b0 : 1'bz;
    assign I2C_SDA = sda_drive_low ? 1'b0 : 1'bz;

    i2c_master #(
        .CLK_HZ(CLK_HZ),
        .I2C_HZ(I2C_HZ)
    ) u_i2c_master (
        .clk(CLK_50M),
        .rst_n(RST_N),
        .start(txn_start),
        .is_read(txn_is_read),
        .dev_addr(txn_dev_addr),
        .reg_addr(txn_reg_addr),
        .reg_addr_16b(txn_reg_addr_16b),
        .wr_data(txn_wr_data),
        .rd_len(txn_rd_len),
        .rd_data(txn_rd_data),
        .busy(txn_busy),
        .done(txn_done),
        .error(txn_error),
        .nack(txn_nack),
        .scl_drive_low(scl_drive_low),
        .sda_drive_low(sda_drive_low),
        .scl_in(I2C_SCL),
        .sda_in(I2C_SDA)
    );

    localparam S_WAIT   = 4'd0;
    localparam S_M0_GO  = 4'd1;
    localparam S_M0_W   = 4'd2;
    localparam S_M1_GO  = 4'd3;
    localparam S_M1_W   = 4'd4;
    localparam S_M2_GO  = 4'd5;
    localparam S_M2_W   = 4'd6;
    localparam S_M3_GO  = 4'd7;
    localparam S_M3_W   = 4'd8;
    localparam S_HOLD   = 4'd9;
    localparam S_ERR    = 4'd10;

    reg [3:0] state;
    reg [31:0] hold_cnt;
    reg [15:0] display_value;
    reg ok_latched;
    reg err_latched;

    always @(posedge CLK_50M or negedge RST_N) begin
        if (!RST_N) begin
            state <= S_WAIT;
            txn_start <= 1'b0;
            txn_is_read <= 1'b0;
            txn_dev_addr <= DEV_ADDR;
            txn_reg_addr <= 16'h0000;
            txn_reg_addr_16b <= 1'b0;
            txn_wr_data <= 8'h00;
            txn_rd_len <= 8'h00;
            hold_cnt <= 32'd0;
            display_value <= 16'd0;
            ok_latched <= 1'b0;
            err_latched <= 1'b0;
        end else begin
            txn_start <= 1'b0;
            case (state)
                S_WAIT: begin
                    display_value <= 16'd0;
                    if (powerup_done && !txn_busy) begin
                        txn_is_read <= 1'b0;
                        txn_dev_addr <= DEV_ADDR;
                        txn_reg_addr_16b <= 1'b0;
                        txn_rd_len <= 8'h00;
                        state <= S_M0_GO;
                    end
                end
                S_M0_GO: begin
                    display_value <= 16'd1;
                    txn_reg_addr <= 16'h0000;
                    txn_wr_data <= MOTOR_SPEED;
                    txn_start <= 1'b1;
                    state <= S_M0_W;
                end
                S_M0_W: begin
                    display_value <= 16'd1;
                    if (txn_done) begin
                        if (txn_error || txn_nack) begin err_latched <= 1'b1; state <= S_ERR; end
                        else state <= S_M1_GO;
                    end
                end
                S_M1_GO: begin
                    display_value <= 16'd2;
                    txn_reg_addr <= 16'h0001;
                    txn_wr_data <= MOTOR_SPEED;
                    txn_start <= 1'b1;
                    state <= S_M1_W;
                end
                S_M1_W: begin
                    display_value <= 16'd2;
                    if (txn_done) begin
                        if (txn_error || txn_nack) begin err_latched <= 1'b1; state <= S_ERR; end
                        else state <= S_M2_GO;
                    end
                end
                S_M2_GO: begin
                    display_value <= 16'd3;
                    txn_reg_addr <= 16'h0002;
                    txn_wr_data <= MOTOR_SPEED;
                    txn_start <= 1'b1;
                    state <= S_M2_W;
                end
                S_M2_W: begin
                    display_value <= 16'd3;
                    if (txn_done) begin
                        if (txn_error || txn_nack) begin err_latched <= 1'b1; state <= S_ERR; end
                        else state <= S_M3_GO;
                    end
                end
                S_M3_GO: begin
                    display_value <= 16'd4;
                    txn_reg_addr <= 16'h0003;
                    txn_wr_data <= MOTOR_SPEED;
                    txn_start <= 1'b1;
                    state <= S_M3_W;
                end
                S_M3_W: begin
                    display_value <= 16'd4;
                    if (txn_done) begin
                        if (txn_error || txn_nack) begin err_latched <= 1'b1; state <= S_ERR; end
                        else begin
                            ok_latched <= 1'b1;
                            hold_cnt <= 32'd0;
                            state <= S_HOLD;
                        end
                    end
                end
                S_HOLD: begin
                    display_value <= 16'd100;
                    if (hold_cnt < 32'd50000000) hold_cnt <= hold_cnt + 1'b1;
                    else begin
                        hold_cnt <= 32'd0;
                        state <= S_M0_GO;
                    end
                end
                default: begin
                    display_value <= 16'hEEEE;
                    state <= S_ERR;
                end
            endcase
            if (state == S_ERR) display_value <= 16'hEEEE;
        end
    end

    seg7_display u_seg7 (
        .clk(CLK_50M),
        .rst_n(RST_N),
        .value(display_value),
        .seg(SEG),
        .dig(DIG)
    );

    assign LED1 = ~powerup_done;           // горит во время старта
    assign LED2 = ~ok_latched;             // гаснет после успешной последовательности
    assign LED3 = ~(err_latched | txn_error | txn_nack);
    assign LED4 = ~txn_busy;               // мигает/меняется при I2C активности

endmodule