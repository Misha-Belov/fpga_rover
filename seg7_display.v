`timescale 1ns / 1ps
`default_nettype none

module seg7_display (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] value,
    input  wire        show_err,
    output reg  [7:0]  seg,
    output reg  [3:0]  dig
);
    localparam [7:0] SEG_OFF = 8'hFF;
    localparam [7:0] SEG_E   = 8'b1000_0110;
    localparam [7:0] SEG_R   = 8'b1010_1111;

    reg [15:0] value_clamped;
    reg [3:0] thousands;
    reg [3:0] hundreds;
    reg [3:0] tens;
    reg [3:0] ones;
    reg [15:0] rem;

    reg [15:0] refresh_cnt;
    reg [1:0]  mux_sel;

    function [7:0] seg_encode;
        input [3:0] digit;
        begin
            case (digit)
                4'd0: seg_encode = 8'b1100_0000;
                4'd1: seg_encode = 8'b1111_1001;
                4'd2: seg_encode = 8'b1010_0100;
                4'd3: seg_encode = 8'b1011_0000;
                4'd4: seg_encode = 8'b1001_1001;
                4'd5: seg_encode = 8'b1001_0010;
                4'd6: seg_encode = 8'b1000_0010;
                4'd7: seg_encode = 8'b1111_1000;
                4'd8: seg_encode = 8'b1000_0000;
                4'd9: seg_encode = 8'b1001_0000;
                default: seg_encode = SEG_OFF;
            endcase
        end
    endfunction

    always @(*) begin
        value_clamped = (value > 16'd9999) ? 16'd9999 : value;

        thousands = value_clamped / 16'd1000;
        rem       = value_clamped % 16'd1000;
        hundreds  = rem / 16'd100;
        rem       = rem % 16'd100;
        tens      = rem / 16'd10;
        ones      = rem % 16'd10;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            refresh_cnt <= 16'd0;
            mux_sel     <= 2'd0;
        end else begin
            if (refresh_cnt == 16'd12499) begin
                refresh_cnt <= 16'd0;
                mux_sel     <= mux_sel + 2'd1;
            end else begin
                refresh_cnt <= refresh_cnt + 16'd1;
            end
        end
    end

    always @(*) begin
        seg = SEG_OFF;
        dig = 4'b1111;

        if (show_err) begin
            case (mux_sel)
                2'd0: begin dig = 4'b1110; seg = SEG_OFF; end
                2'd1: begin dig = 4'b1101; seg = SEG_R;   end
                2'd2: begin dig = 4'b1011; seg = SEG_R;   end
                default: begin dig = 4'b0111; seg = SEG_E; end
            endcase
        end else begin
            case (mux_sel)
                2'd0: begin
                    dig = 4'b1110;
                    seg = seg_encode(ones);
                end
                2'd1: begin
                    dig = 4'b1101;
                    if ((value_clamped < 16'd10) && (value_clamped != 16'd0))
                        seg = SEG_OFF;
                    else
                        seg = seg_encode(tens);
                end
                2'd2: begin
                    dig = 4'b1011;
                    if (value_clamped < 16'd100)
                        seg = SEG_OFF;
                    else
                        seg = seg_encode(hundreds);
                end
                default: begin
                    dig = 4'b0111;
                    if (value_clamped < 16'd1000)
                        seg = SEG_OFF;
                    else
                        seg = seg_encode(thousands);
                end
            endcase
        end
    end
endmodule

`default_nettype wire