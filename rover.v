
// ============================================================
//  top_7seg.v — Шаг 2: Счётчик 0–9999 на встроенном
//               4-разрядном 7-сегментном индикаторе
//
//  Плата: 助学FPGA开发板 (RZRD / OurFPGA.com)
//  Чип:   EP4CE6E22C8N
//
//  Пины 7-сегментного индикатора (мультиплексный):
//    DIG1: PIN_133   DIG2: PIN_135
//    DIG3: PIN_136   DIG4: PIN_137
//    SEG1(a): PIN_121   SEG2(b): PIN_125
//    SEG3(c): PIN_129   SEG4(d): PIN_132
//    SEG5(e): PIN_126   SEG6(f): PIN_124
//    SEG7(g): PIN_127   SEG0(dp): PIN_128
//
//  Тип дисплея: ОБЩИЙ КАТОД, сегменты активны HIGH,
//               разряды активны HIGH.
//  Если твой дисплей ОБЩИЙ АНОД — инвертируй SEG и DIG!
// ============================================================
module rover (
    input  wire       clk,     // PIN_23 — 50 МГц
    input  wire       key1,    // PIN_88 — KEY1, сброс (акт. LOW)
    // Разряды: DIG[0]=единицы...DIG[3]=тысячи (акт. HIGH)
    output reg  [3:0] DIG,
    // Сегменты: {DP,g,f,e,d,c,b,a} (акт. HIGH)
    output reg  [7:0] SEG
);

// ----------------------------------------------------------
// 1. Делитель → 1 Гц (счёт цифр)
// ----------------------------------------------------------
reg [25:0] cnt_1hz;
reg        tick_1hz;

always @(posedge clk) begin
    if (!key1) begin
        cnt_1hz  <= 0;
        tick_1hz <= 0;
    end else if (cnt_1hz == 49_999_999) begin
        cnt_1hz  <= 0;
        tick_1hz <= 1;
    end else begin
        cnt_1hz  <= cnt_1hz + 1;
        tick_1hz <= 0;
    end
end

// ----------------------------------------------------------
// 2. BCD-счётчик 0000 – 9999
// ----------------------------------------------------------
reg [3:0] d0, d1, d2, d3;

always @(posedge clk) begin
    if (!key1) begin
        d0 <= 0; d1 <= 0; d2 <= 0; d3 <= 0;
    end else if (tick_1hz) begin
        if (d0 == 9) begin
            d0 <= 0;
            if (d1 == 9) begin
                d1 <= 0;
                if (d2 == 9) begin
                    d2 <= 0;
                    d3 <= (d3 == 9) ? 4'd0 : d3 + 1;
                end else d2 <= d2 + 1;
            end else d1 <= d1 + 1;
        end else d0 <= d0 + 1;
    end
end

// ----------------------------------------------------------
// 3. Делитель → ~1 кГц для мультиплексирования
//    Каждый разряд обновляется 250 раз/сек → нет мерцания
// ----------------------------------------------------------
reg [15:0] cnt_mux;
reg        tick_mux;

always @(posedge clk) begin
    if (!key1) begin
        cnt_mux  <= 0;
        tick_mux <= 0;
    end else if (cnt_mux == 49_999) begin
        cnt_mux  <= 0;
        tick_mux <= 1;
    end else begin
        cnt_mux  <= cnt_mux + 1;
        tick_mux <= 0;
    end
end

// ----------------------------------------------------------
// 4. Мультиплексор разрядов
// ----------------------------------------------------------
reg [1:0] dig_sel;

always @(posedge clk) begin
    if (!key1)      dig_sel <= 0;
    else if (tick_mux) dig_sel <= dig_sel + 1;
end

// Выбор текущей BCD-цифры
reg [3:0] bcd_cur;
always @(*) begin
    case (dig_sel)
        2'd0: bcd_cur = d0;   // единицы
        2'd1: bcd_cur = d1;   // десятки
        2'd2: bcd_cur = d2;   // сотни
        2'd3: bcd_cur = d3;   // тысячи
        default: bcd_cur = 0;
    endcase
end

// Активация разряда (активный HIGH)
always @(*) begin
    case (dig_sel)
        2'd0: DIG = 4'b1110;   // единицы
        2'd1: DIG = 4'b1101;   // десятки
        2'd2: DIG = 4'b1011;   // сотни
        2'd3: DIG = 4'b0111;   // тысячи
        default: DIG = 4'b0000;
    endcase
end

// ----------------------------------------------------------
// 5. Декодер BCD → сегменты
//    SEG = {DP, g, f, e, d, c, b, a}
//    Активный HIGH (общий катод)
// ----------------------------------------------------------
always @(*) begin
    case (bcd_cur)
        //               DPgfedcba
        4'd0: SEG = 8'b0_1100000; // 0
        4'd1: SEG = 8'b1_1111100; // 1
        4'd2: SEG = 8'b0_1010010; // 2
        4'd3: SEG = 8'b0_1011000; // 3
        4'd4: SEG = 8'b1_1001100; // 4
        4'd5: SEG = 8'b0_1001001; // 5
        4'd6: SEG = 8'b0_1000001; // 6
        4'd7: SEG = 8'b0_1111100; // 7
        4'd8: SEG = 8'b0_1000000; // 8
        4'd9: SEG = 8'b0_1001000; // 9
        default: SEG = 8'b1111110_1;
    endcase
end

endmodule