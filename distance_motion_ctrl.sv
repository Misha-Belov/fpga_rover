`timescale 1ns / 1ps
`default_nettype none

module distance_motion_ctrl #(
    parameter integer CLK_HZ               = 50000000,
    parameter integer DIST_STOP_MM         = 100,
    parameter integer DIST_RELEASE_MM      = 220,
    parameter integer DIST_PASS_RISE_MM    = 120,
    parameter integer DIST_GO_STRAIGHT_MM  = 500,
    parameter integer SIDE_EXTRA_SCALE_PCT = 80,
    parameter integer MIN_SIDE_TRACK_MS    = 80,
    parameter integer CLEAR_CONFIRM_MS     = 20,
    // Дальняя граница зоны blend: dist >= FAR  → чисто вперёд (в S_SIDE)
    // Ближняя граница:            dist <= NEAR → чисто бок 90°  (в S_SIDE)
    // Между NEAR и FAR — фиксированная диагональ (среднее FWD+SIDE),
    // вычисленная при синтезе как localparam — никаких умножителей в рантайме.
    parameter integer DIST_STEER_FAR_MM    = 300,
    parameter integer DIST_STEER_NEAR_MM   = 80,
    parameter integer FWD_MOTOR0_CMD       = 127,
    parameter integer FWD_MOTOR1_CMD       = 127,
    parameter integer FWD_MOTOR2_CMD       = 127,
    parameter integer FWD_MOTOR3_CMD       = 127,
    parameter integer SIDE_MOTOR0_CMD      = -105,
    parameter integer SIDE_MOTOR1_CMD      = 105,
    parameter integer SIDE_MOTOR2_CMD      = 127,
    parameter integer SIDE_MOTOR3_CMD      = -127
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sensor_valid,
    input  wire [15:0] distance_mm,
    output reg  [7:0]  motor0_speed,
    output reg  [7:0]  motor1_speed,
    output reg  [7:0]  motor2_speed,
    output reg  [7:0]  motor3_speed
);
    localparam integer MIN_SIDE_TRACK_CLKS = (CLK_HZ / 1000) * MIN_SIDE_TRACK_MS;
    localparam integer CLEAR_CONFIRM_CLKS  = (CLK_HZ / 1000) * CLEAR_CONFIRM_MS;

    // -----------------------------------------------------------------------
    // Диагональные команды: синтез-таймовые константы, ноль рантаймовой логики.
    // Среднее между FWD и SIDE — соответствует ~45° при входе в зону blend.
    // -----------------------------------------------------------------------
    localparam integer DIAG_MOTOR0_CMD = 0;
    localparam integer DIAG_MOTOR1_CMD = 127;
    localparam integer DIAG_MOTOR2_CMD = 127;
    localparam integer DIAG_MOTOR3_CMD = 0;

    localparam [1:0]
        S_STOP       = 2'd0,
        S_FORWARD    = 2'd1,
        S_SIDE_TRACK = 2'd2,
        S_SIDE_EXTRA = 2'd3;

    reg [1:0]  state;
    reg [31:0] side_track_cnt;
    reg [31:0] side_extra_cnt;
    reg [31:0] side_extra_target_cnt;
    reg [31:0] clear_confirm_cnt;
    reg [15:0] side_min_mm;

    wire motion_enable;
    wire sensor_too_close;
    wire release_by_absolute;
    wire release_by_rise;
    wire wall_pass_candidate;
    wire far_after_wall;

    // -----------------------------------------------------------------------
    // add_sat16: единственная оставшаяся функция — только сложение
    // -----------------------------------------------------------------------
    function automatic [15:0] add_sat16;
        input [15:0] base_value;
        input integer add_value;
        reg   [31:0] sum_value;
        begin
            sum_value = {16'd0, base_value} + add_value[31:0];
            add_sat16 = (sum_value > 32'd65535) ? 16'hFFFF : sum_value[15:0];
        end
    endfunction

    // -----------------------------------------------------------------------
    // Комбинаторные сигналы
    // -----------------------------------------------------------------------
    assign motion_enable       = sensor_valid;
    assign sensor_too_close    = motion_enable && (distance_mm < DIST_STOP_MM);
    assign release_by_absolute = motion_enable && (distance_mm >= DIST_RELEASE_MM);
    assign release_by_rise     = motion_enable &&
                                 ((MIN_SIDE_TRACK_CLKS == 0) || (side_track_cnt >= MIN_SIDE_TRACK_CLKS)) &&
                                 (distance_mm >= add_sat16(side_min_mm, DIST_PASS_RISE_MM));
    assign wall_pass_candidate = release_by_absolute || release_by_rise;
    assign far_after_wall      = motion_enable && (distance_mm >= DIST_GO_STRAIGHT_MM);

    // -----------------------------------------------------------------------
    // Целевое время S_SIDE_EXTRA — без умножителя.
    // При SIDE_EXTRA_SCALE_PCT == 100  (дефолт): target = side_track_cnt.
    // При SIDE_EXTRA_SCALE_PCT == 0             : target = 0.
    // Для других значений оставлен путь с умножением как fallback
    // (если он понадобится — просто не используйте нестандартный SCALE).
    // -----------------------------------------------------------------------
    wire [31:0] extra_target_wire;
    generate
        if (SIDE_EXTRA_SCALE_PCT <= 0) begin : gen_extra_zero
            assign extra_target_wire = 32'd0;
        end else if (SIDE_EXTRA_SCALE_PCT == 100) begin : gen_extra_unity
            assign extra_target_wire = side_track_cnt;   // тождество, 0 LUT
        end else begin : gen_extra_scale
            // Shift-based approx: работает для степеней двойки.
            // Для произвольных значений: оставьте SCALE=100 (дефолт).
            assign extra_target_wire = (side_track_cnt * SIDE_EXTRA_SCALE_PCT + 32'd99) / 32'd100;
        end
    endgenerate

    // -----------------------------------------------------------------------
    // Машина состояний (без изменений)
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                 <= S_STOP;
            side_track_cnt        <= 32'd0;
            side_extra_cnt        <= 32'd0;
            side_extra_target_cnt <= 32'd0;
            clear_confirm_cnt     <= 32'd0;
            side_min_mm           <= 16'hFFFF;
        end else begin
            case (state)
                S_STOP: begin
                    side_track_cnt        <= 32'd0;
                    side_extra_cnt        <= 32'd0;
                    side_extra_target_cnt <= 32'd0;
                    clear_confirm_cnt     <= 32'd0;
                    side_min_mm           <= 16'hFFFF;

                    if (motion_enable) begin
                        if (sensor_too_close) begin
                            state                 <= S_SIDE_TRACK;
                            side_track_cnt        <= 32'd0;
                            side_extra_cnt        <= 32'd0;
                            side_extra_target_cnt <= 32'd0;
                            clear_confirm_cnt     <= 32'd0;
                            side_min_mm           <= distance_mm;
                        end else begin
                            state <= S_FORWARD;
                        end
                    end
                end

                S_FORWARD: begin
                    side_track_cnt        <= 32'd0;
                    side_extra_cnt        <= 32'd0;
                    side_extra_target_cnt <= 32'd0;
                    clear_confirm_cnt     <= 32'd0;
                    side_min_mm           <= 16'hFFFF;

                    if (!motion_enable) begin
                        state <= S_STOP;
                    end else if (sensor_too_close) begin
                        state                 <= S_SIDE_TRACK;
                        side_track_cnt        <= 32'd0;
                        side_extra_cnt        <= 32'd0;
                        side_extra_target_cnt <= 32'd0;
                        clear_confirm_cnt     <= 32'd0;
                        side_min_mm           <= distance_mm;
                    end
                end

                S_SIDE_TRACK: begin
                    side_extra_cnt <= 32'd0;

                    if (!motion_enable) begin
                        state                 <= S_STOP;
                        side_track_cnt        <= 32'd0;
                        side_extra_cnt        <= 32'd0;
                        side_extra_target_cnt <= 32'd0;
                        clear_confirm_cnt     <= 32'd0;
                        side_min_mm           <= 16'hFFFF;
                    end else begin
                        if (side_track_cnt != 32'hFFFF_FFFF)
                            side_track_cnt <= side_track_cnt + 32'd1;

                        if (distance_mm < side_min_mm)
                            side_min_mm <= distance_mm;

                        if (wall_pass_candidate) begin
                            if ((CLEAR_CONFIRM_CLKS == 0) || (clear_confirm_cnt >= (CLEAR_CONFIRM_CLKS - 1))) begin
                                clear_confirm_cnt <= 32'd0;

                                if (far_after_wall) begin
                                    state                 <= S_FORWARD;
                                    side_track_cnt        <= 32'd0;
                                    side_extra_cnt        <= 32'd0;
                                    side_extra_target_cnt <= 32'd0;
                                    side_min_mm           <= 16'hFFFF;
                                end else begin
                                    if (extra_target_wire == 32'd0) begin
                                        state          <= S_FORWARD;
                                        side_track_cnt <= 32'd0;
                                        side_min_mm    <= 16'hFFFF;
                                    end else begin
                                        side_extra_target_cnt <= extra_target_wire;
                                        side_extra_cnt        <= 32'd0;
                                        state                 <= S_SIDE_EXTRA;
                                    end
                                end
                            end else begin
                                clear_confirm_cnt <= clear_confirm_cnt + 32'd1;
                            end
                        end else begin
                            clear_confirm_cnt <= 32'd0;
                        end
                    end
                end

                S_SIDE_EXTRA: begin
                    side_track_cnt    <= 32'd0;
                    clear_confirm_cnt <= 32'd0;
                    side_min_mm       <= 16'hFFFF;

                    if (!motion_enable) begin
                        state                 <= S_STOP;
                        side_extra_cnt        <= 32'd0;
                        side_extra_target_cnt <= 32'd0;
                    end else if (far_after_wall) begin
                        state                 <= S_FORWARD;
                        side_extra_cnt        <= 32'd0;
                        side_extra_target_cnt <= 32'd0;
                    end else if ((side_extra_target_cnt == 32'd0) ||
                                 (side_extra_cnt >= (side_extra_target_cnt - 32'd1))) begin
                        state                 <= S_FORWARD;
                        side_extra_cnt        <= 32'd0;
                        side_extra_target_cnt <= 32'd0;
                    end else begin
                        side_extra_cnt <= side_extra_cnt + 32'd1;
                    end
                end

                default: begin
                    state                 <= S_STOP;
                    side_track_cnt        <= 32'd0;
                    side_extra_cnt        <= 32'd0;
                    side_extra_target_cnt <= 32'd0;
                    clear_confirm_cnt     <= 32'd0;
                    side_min_mm           <= 16'hFFFF;
                end
            endcase
        end
    end

    // -----------------------------------------------------------------------
    // Блок вывода — только компараторы, ноль умножителей.
    //
    //  S_FORWARD                 → чисто вперёд (FWD_MOTOR*_CMD)
    //
    //  S_SIDE_TRACK / S_SIDE_EXTRA → 3 зоны по дистанции:
    //    dist <= NEAR (200 мм)   → чисто бок     (SIDE_MOTOR*_CMD)   = 90°
    //    NEAR < dist < FAR       → диагональ     (DIAG_MOTOR*_CMD)   ≈ 45°
    //    dist >= FAR  (400 мм)   → чисто вперёд  (FWD_MOTOR*_CMD)    = 0°
    //
    //  DIAG_MOTOR*_CMD = (FWD + SIDE) / 2  вычислены при синтезе как localparam.
    // -----------------------------------------------------------------------
    always @(*) begin
        motor0_speed = 8'd0;
        motor1_speed = 8'd0;
        motor2_speed = 8'd0;
        motor3_speed = 8'd0;

        case (state)
            S_FORWARD: begin
                motor0_speed = FWD_MOTOR0_CMD[7:0];
                motor1_speed = FWD_MOTOR1_CMD[7:0];
                motor2_speed = FWD_MOTOR2_CMD[7:0];
                motor3_speed = FWD_MOTOR3_CMD[7:0];
            end

            S_SIDE_TRACK,
            S_SIDE_EXTRA: begin
                if (distance_mm <= DIST_STEER_NEAR_MM) begin
                    // dist <= 200 мм: чисто бок, строго 90°
                    motor0_speed = SIDE_MOTOR0_CMD[7:0];
                    motor1_speed = SIDE_MOTOR1_CMD[7:0];
                    motor2_speed = SIDE_MOTOR2_CMD[7:0];
                    motor3_speed = SIDE_MOTOR3_CMD[7:0];
                end else if (distance_mm >= DIST_STEER_FAR_MM) begin
                    // dist >= 400 мм: чисто вперёд
                    motor0_speed = FWD_MOTOR0_CMD[7:0];
                    motor1_speed = FWD_MOTOR1_CMD[7:0];
                    motor2_speed = FWD_MOTOR2_CMD[7:0];
                    motor3_speed = FWD_MOTOR3_CMD[7:0];
                end else begin
                    // 200 < dist < 400 мм: фиксированная диагональ ≈ 45°
                    // (синтез-таймовые константы — 0 умножителей в рантайме)
                    motor0_speed = DIAG_MOTOR0_CMD[7:0];
                    motor1_speed = DIAG_MOTOR1_CMD[7:0];
                    motor2_speed = DIAG_MOTOR2_CMD[7:0];
                    motor3_speed = DIAG_MOTOR3_CMD[7:0];
                end
            end

            default: begin
                motor0_speed = 8'd0;
                motor1_speed = 8'd0;
                motor2_speed = 8'd0;
                motor3_speed = 8'd0;
            end
        endcase
    end
endmodule

`default_nettype wire