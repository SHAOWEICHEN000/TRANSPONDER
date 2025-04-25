module mode_s_transponder (
    input clk,
    input reset,
    input pulse_received,
    input transaction_valid,
    input p6_sync_phase_rev,
    // ... 其他輸入
    output reg reply,
    output reg transfer_data
);

    // 定義狀態
    parameter START = 0;
    parameter PULSE_RECEIVED_CHECK = 1;
    parameter TRANSACTION_VALID_CHECK = 2;
    parameter P6_SYNC_CHECK = 3;
    // ... 其他狀態

    reg [31:0] current_state;

    initial begin
        current_state = START;
        reply = 0;
        transfer_data = 0;
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_state = START;
            reply = 0;
            transfer_data = 0;
        end else begin
            case (current_state)
                START: begin
                    current_state = PULSE_RECEIVED_CHECK;
                end
                PULSE_RECEIVED_CHECK: begin
                    if (pulse_received) begin
                        current_state = TRANSACTION_VALID_CHECK;
                    end else begin
                        current_state = PULSE_RECEIVED_CHECK;
                    end
                end
                // Transaction check
                TRANSACTION_VALID_CHECK: begin
                    if (transaction_valid) begin
                        current_state = P6_SYNC_CHECK;
                    end else begin
                        // ... 處理交易無效
                    end
                end

                //P6_SYNC_CHECK
                P6_SYNC_CHECK: begin
                    if (p6_sync_phase_rev) begin
                        // ... 處理 P6 和 SYNC 相位反轉
                    end else begin
                        // ... 處理 P6 和 SYNC 相位不反轉
                    end
                end
                // ... 其他狀態
                default: current_state = START;
            endcase
        end
    end

endmodule
