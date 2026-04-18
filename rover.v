// ============================================================
//  led_blink.v — Шаг 1: Бегущий огонь на 4 светодиодах
//
//  Плата: 助学FPGA开发板 (RZRD / OurFPGA.com)
//  Чип:   EP4CE6E22C8N  →  в Quartus выбирать: EP4CE6E22C8
//
//  CLK (50 МГц):  PIN_23
//  KEY1 (active LOW): PIN_88
//  LED1: PIN_87  LED2: PIN_86  LED3: PIN_85  LED4: PIN_84
//  Светодиоды активны HIGH
// ============================================================
module led_blink (
    input  wire       clk,   // PIN_23 — 50 МГц кварц
    input  wire       key1,  // PIN_88 — кнопка KEY1 (сброс)
    output reg  [3:0] led    // PIN_87..84
);

// Делитель: импульс 1 раз в 0.5 сек
reg [24:0] cnt;
reg        tick;

always @(posedge clk) begin
    if (!key1) begin        // кнопка нажата — сброс
        cnt  <= 0;
        tick <= 0;
    end else if (cnt == 25_000_000 - 1) begin
        cnt  <= 0;
        tick <= 1;
    end else begin
        cnt  <= cnt + 1;
        tick <= 0;
    end
end

// Кольцевой сдвиг: 0001→0010→0100→1000→0001...
always @(posedge clk) begin
    if (!key1)
        led <= 4'b0001;
    else if (tick)
        led <= {led[2:0], led[3]};
end

endmodule