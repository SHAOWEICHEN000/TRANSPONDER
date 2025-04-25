`timescale 1ns / 1ps

module dpsk_demodulator #(
    parameter integer SYNC_PHASE_DURATION  = 75,     // e.g. 1.25 us @ 60 MHz
    parameter integer SYNC_PHASE_THRESHOLD = 3000000, // 根據系統雜訊/取樣率調整
    parameter integer DPSK_BITS            = 56       // Mode S P6 預設 112 bits
)(
    input  wire             clk,               // 60 MHz 時脈 or 根據 AD9371 設定
    input  wire             reset,             // 重置信號
    input  signed [15:0]    i_in,              // I 信號輸入
    input  signed [15:0]    q_in,              // Q 信號輸入
    input  wire             sync_phase_done,   // 同步相位完成訊號
    output reg              data_out,          // 每個 bit 的 DPSK 解調結果
    output reg              sync_phase_rev     // 是否偵測到同步相位翻轉
);

    // 狀態定義
    typedef enum logic [1:0] {
        SYNC_PHASE = 0,  // 偵測同步相位翻轉(能量累積)
        DATA_BITS=1,       // DPSK 解調模式(位元解析)
    } state_t;

    state_t current_state;

    // 同步累積計數器 & 能量
    reg [31:0] sync_phase_counter; 
    reg signed [31:0] sync_phase_sum;

    // 前一次 I/Q
    reg signed [15:0] i_prev, q_prev;

    // 計算差分結果
    reg signed [31:0] phase_product;

    // 計算 bit
    reg [7:0]  bit_counter; // 可支援最高 255 bits

    // ---------------- 初始條件 ----------------
    initial begin
        current_state      = SYNC_PHASE;
        sync_phase_counter = 0;
        sync_phase_sum     = 0;
        sync_phase_rev     = 0;
        bit_counter        = 0;
        data_out           = 0;

        i_prev             = 0;
        q_prev             = 0;
        phase_product      = 0;
    end

    // ---------------- 主狀態機 ----------------
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_state      <= SYNC_PHASE;
            sync_phase_counter <= 0;
            sync_phase_sum     <= 0;
            sync_phase_rev     <= 0;
            bit_counter        <= 0;
            data_out           <= 0;
            i_prev             <= 0;
            q_prev             <= 0;
            phase_product      <= 0;
        end
        else begin
            case (current_state)

                // ============= 狀態 1：同步相位偵測 =============
                SYNC_PHASE: begin
                    if (sync_phase_counter < SYNC_PHASE_DURATION) begin
                        // 累積能量
                        sync_phase_counter <= sync_phase_counter + 1;
                        sync_phase_sum     <= sync_phase_sum 
                                           + (i_in * i_in) 
                                           +(q_in * q_in);
                    end
                    else begin
                        // 根據閾值判斷是否有偵測到同步相位翻轉
                        if (sync_phase_sum > SYNC_PHASE_THRESHOLD)
                            sync_phase_rev <= 1;
                        else
                            sync_phase_rev <= 0;

                        // 轉往解調模式
                        current_state      <= DATA_BITS;
                        bit_counter        <= 0;

                        // 清空或保留, 視需求而定
                        // i_prev/q_prev 一開始可以用當下值作為前一次
                        i_prev             <= i_in;
                        q_prev             <= q_in;
                    end
                end

                // ============= 狀態 2：DPSK 解調 =============
                DATA_BITS: begin
                    if (bit_counter < DPSK_BITS) begin
                        // 差分公式: phase_product = (I_now * I_prev) + (Q_now * Q_prev)
                        // 也可用 +, 視硬體定義
                        phase_product <= (i_in * i_prev) 
                                      + (q_in * q_prev);

                        // 取決定 bit 值
                        if (phase_product > 0)
                            data_out <= 1;
                        else
                            data_out <= 0;

                        // 更新 i_prev, q_prev
                        i_prev   <= i_in;
                        q_prev   <= q_in;

                        // 前進一 bit
                        bit_counter <= bit_counter + 1;
                    end
                    else begin
                        // 完成指定 bits 後，若需要再進行同步/重置
                        // 這裡示範：檢查外部 sync_phase_done；或自動回到 SYNC_PHASE
                        if (sync_phase_done) begin
                            current_state      <= SYNC_PHASE;
                            sync_phase_counter <= 0;
                            sync_phase_sum     <= 0;
                        end
                    end
                end

                default: begin
                    current_state <= SYNC_PHASE;
                end

            endcase
        end
    end

endmodule
