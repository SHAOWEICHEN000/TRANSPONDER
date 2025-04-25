module p6_capture #(
    parameter integer P6_BITS = 56 // or 112
)(
    input  wire clk,
    input  wire reset,

    // 從解調器獲得單 bit 輸出
    input  wire data_out,       

    // 解調器的 bit_count 是否完成
    input  wire dpsk_done,      

    // shift register 解析完成後, 輸出的各欄位
    output reg [4:0]  uf_code,  // bits 1~5
    output reg [3:0]  pr,       // bits 6~9
    output reg [3:0]  ic,       // bits 10~13
    output reg [2:0]  cl,       // bits 14~16
    output reg [15:0] Main_data,// bits 17~32
    output reg [23:0]  AP,       // bits 33~56


    // 方便示範, 全部 bits
    output reg [P6_BITS-1:0] p6_full_bits 
);

    reg [P6_BITS-1:0] shift_reg;
    reg [7:0] bit_counter;

    // 若你想等 data_out 每個 clock output, 這裡就順著 shift 進去
    // 直到 dpsk_done=1, 表示該次 P6 收完

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            shift_reg   <= 0;
            bit_counter <= 0;
        end else begin
            // 每 clock 收一 bit
            // 假設 data_out 在此 clock 有效, 你也可加個 enable
            if (!dpsk_done && bit_counter < P6_BITS) begin
                // 假設第一個接收到的 bit 是 shift_reg[ (P6_BITS-1) ] (MSB)
                // 也可反轉端點, 看你對 bit 順序的需求
                shift_reg <= {shift_reg[P6_BITS-2:0], data_out};
                bit_counter <= bit_counter + 1;
            end
            else if (dpsk_done) begin
                // 已經收到全部 bits, 在這裡解析欄位
                p6_full_bits <= shift_reg;

                // 依照 Mode S uplink format (UF)
                // bits numbering from 1..56 (或 1..112)
                // 你要注意 bit 順序： 
                //   shift_reg[P6_BITS-1] 可能對應 bit1, 
                //   shift_reg[P6_BITS-2] 對應 bit2, 
                //   ...
                //   shift_reg[0] 對應 bit56
                // 這裡只是示範, 你要根據實際收 bit 的順序對應

                // 下面示例: shift_reg[ P6_BITS-1 : P6_BITS-5 ] → UF  (bits 1~5)
                // 若 P6_BITS=56, shift_reg[55:51] = bits 1~5
                uf_code <= shift_reg[55:51];  // bits 1..5

                pr      <= shift_reg[50:47];  // bits 6..9
                ic      <= shift_reg[46:43];  // bits 10..13
                cl      <= shift_reg[42:40];  // bits 14..16
		Main_data<=shift_reg[39:24];
		AP<=	   shift_reg[23:0];

                // 依此類推, bits 17~32, 33~56 => ...
                // e.g. Address / AP
                // address = shift_reg[32:9] (範例, 需自行對照 Annex 10)

                // 解析完成後, bit_counter 歸零, 等待下一次
                bit_counter <= 0;
            end
        end
    end

endmodule
