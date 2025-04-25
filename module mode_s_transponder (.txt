module mode_s_transponder (  
    input clk,
    input reset,
    input pulse_received,
    input transaction_valid,
    input p6_sync_phase_rev,
    input [4:0] uf_code, // UF 碼
    input p4_edge,
    input si_code, // SI 判斷
    input ii_code, // II 判斷
    input [13:0] edge_count, // 14 個邊緣計數
    input atcrbs_detected, // ATCRBS 檢測
    input sp4_detected, // SP4 檢測
    input lpa_detected, // LPA 檢測
    input uf_equipped, // 是否支援 UF
    input address_assigned, // 地址是否已分配
    input short_format, // 是否為短格式 (56 bits)
    input [23:0] address, // 24 位地址
    input cl_11_0, // cl_11_0 輸入
    input ii_code, // ii_code 輸入
    input p1_10us, // P1-10us 輸入
    input si_1_63, // SI_1_63 輸入
    input long_format, // 長格式輸入
    input long_format_processable, // 長格式是否可處理
    input pulse_spacing_8_or_21us, // 脈衝間隔是否為 8 微秒或 21 微秒
    input p4_edge, // P4 邊緣檢測        
    output reg recover, // Recover 信號
    output reg reply,
    output reg transfer_data,
    output reg atcrbs_suppressed,// ATCRBS 是否已被抑制
    output reg MAX2112FLAG // 新增 MAX2112FLAG 訊號
);

    // 定義狀態，按照流程圖順序排列
    typedef enum reg [5:0] {
        START = 0,
        PULSE_RECEIVED_CHECK = 1,
        TRANSACTION_VALID_CHECK = 2,
        SUPPRESS_ATCRBS = 3,
        ATCRBS_SUPPRESSED = 4,
        TWO_US_DELAY = 5,
        P6_SYNC_CHECK = 6,
        UF_EQUIPPED_CHECK = 7,
        ADDRESS_ASSIGNED_CHECK = 8,
        SHORT_FORMAT_CHECK = 9,
        UNABLE_TO_PROCESS = 10,
        UF_16DECISION = 11,
        ADDRESS_FFFFFF = 12,
        UF_11_DECISION = 13,
        PROBABILITY_0_TO_4 = 14,
        PROBABILITY_8_TO_12 = 15,
        CL_0 = 16,
        II_0 = 17,
        T1_II = 18,
        TIMER_RUNNING = 19,
        P4_1_6US = 20,
        SI_1_TO_63 = 21,
        T1_SI = 22,
        LONG_FORMAT_NOT_ELM = 23,
        LONG_FORMAT_NOT_ELM_UNABLE_TO_PROCESS = 24,
        SPACING_8_21US = 25,
        P4_EDGE_CHECK = 26,
        REPLY_OR_TRANSFER = 27,
        ACCEPT = 28
    } state_t;

    // 狀態變數
    state_t current_state;
    reg [31:0] timer;
    reg lockout_active;
    reg [31:0] lockout_timer;
    parameter LOCKOUT_COUNT = 18500;
    parameter TIMEOUT_COUNT = 10000; // 超時計數
    reg atcrbs_suppressed_flag;
    //step 1
    reg [31:0] pulse_start_time, pulse_end_time;
    reg [31:0] pulse_width;
    reg pulse_valid;
    //step 4
    parameter TWO_US_COUNT = 200; 
    reg [31:0] p1_end_time; // 記錄 P1 脈衝結束時間
    reg [31:0] p2_start_time; // 記錄 P2 脈衝開始時間 
     // 時間相關參數
    parameter CLK_FREQ = 100000000; // 100 MHz 時鐘頻率
    parameter MIN_SPACING = (CLK_FREQ / 1000000) * 1.8; // 計算為時鐘週期
    parameter MAX_SPACING = (CLK_FREQ / 1000000) * 2.2;
    //STEP6
    reg dpsk_done;  // 當 dpsk 解完 56bits or 112bits 時, 由 dpsk_demodulator 告知或由上層判斷

    // other setting
    parameter MODES_DEFAULT_RATE = 2000000;
    parameter MODES_DEFAULT_FREQ = 1090000000;
    parameter MODES_DEFAULT_WIDTH = 1000;
    parameter MODES_DEFAULT_HEIGHT = 700;
    parameter MODES_ASYNC_BUF_NUMBER = 12;
    parameter MODES_DATA_LEN = 16*16384; // 256k
    parameter MODES_AUTO_GAIN = -100; // Use automatic gain.
    parameter MODES_MAX_GAIN = 999999; // Use max available gain.

    parameter MODES_PREAMBLE_US = 8; // microseconds
    parameter MODES_LONG_MSG_BITS = 112;
    parameter MODES_SHORT_MSG_BITS = 56;
    parameter MODES_FULL_LEN = MODES_PREAMBLE_US + MODES_LONG_MSG_BITS;
    parameter MODES_LONG_MSG_BYTES = 112/8;
    parameter MODES_SHORT_MSG_BYTES = 56/8;

    parameter MODES_ICAO_CACHE_LEN = 1024; // Power of two required.
    parameter MODES_ICAO_CACHE_TTL = 60; // Time to live of cached addresses.
    parameter MODES_UNIT_FEET = 0;
    parameter MODES_UNIT_METERS = 1;

    parameter MODES_DEBUG_DEMOD = 1'b1;
    parameter MODES_DEBUG_DEMODERR = 1'b10;
    parameter MODES_DEBUG_BADCRC = 1'b100;
    parameter MODES_DEBUG_GOODCRC = 1'b1000;
    parameter MODES_DEBUG_NOPREAMBLE = 1'b10000;
    parameter MODES_DEBUG_NET = 1'b100000;
    parameter MODES_DEBUG_JS = 1'b1000000;

    parameter MODES_DEBUG_NOPREAMBLE_LEVEL = 25;

    parameter MODES_INTERACTIVE_REFRESH_TIME = 250; // Milliseconds
    parameter MODES_INTERACTIVE_ROWS = 15; // Rows on screen
    parameter MODES_INTERACTIVE_TTL = 60; // TTL before being removed

    parameter MODES_NET_MAX_FD = 1024;
    parameter MODES_NET_OUTPUT_SBS_PORT = 30003;
    parameter MODES_NET_OUTPUT_RAW_PORT = 30002;
    parameter MODES_NET_INPUT_RAW_PORT = 30001;
    parameter MODES_NET_HTTP_PORT = 8080;
    parameter MODES_CLIENT_BUF_SIZE = 1024;
    parameter MODES_NET_SNDBUF_SIZE = 1024*64;

    // 初始設定
    initial begin
        current_state <= START;
        reply <= 0;
        transfer_data <= 0;
        timer <= 0;
        lockout_active <= 0;
        lockout_timer <= 0;
        atcrbs_suppressed_flag <= 0;
        recover <= 0;
        //step 1
        pulse_valid <= 0;
        pulse_width <= 0;
        p1_end_time <= 0;
        p2_start_time <= 0;
        transaction_valid <= 1'b0;
        MAX2112FLAG <= 1'b0; // 初始化 MAX2112FLAG
	uf_equipped<=1'b0;//本機支援UF LINK
	
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_state = START;
            reply = 0;
            transfer_data = 0;
            timer = 0;
            lockout_active = 0;
            lockout_timer = 0;
            atcrbs_suppressed_flag =1'b0;
            
            recover = 0;
            //step 1
            pulse_valid <= 0;
            pulse_width <= 0;
            //step5 atcrbs_suppressed<=1'b1;
            //STEP 6
            sync_phase_done <= 1'b0;
            //STEP 7
	    uf_equipped<=1'b1;//本機支援UF LINK

        end else begin
            case (current_state)
                //initial step to check the pulse time
                START: begin
                    reply = 0;
                    transfer_data = 0;
                    if (pulse_received) begin
                        pulse_start_time <= $time;  // 記錄脈衝起始時間
                        current_state <= PULSE_RECEIVED_CHECK;
                    end
                end
                //step1 is the module that is supervised the 0.7<=p1 width<=0.9 
                PULSE_RECEIVED_CHECK: begin
                    if (pulse_received) begin
                        pulse_end_time <= $time;  // 記錄脈衝結束時間
                        pulse_width <= pulse_end_time - pulse_start_time; // 計算脈衝寬度
                        if (pulse_width >= 0.7 && pulse_width <= 0.9) begin//0.7 &&0.9可以調
                            pulse_valid <= 1; // 若寬度在 0.8μs ± 誤差範圍內，視為有效
                            current_state <= TRANSACTION_VALID_CHECK;
                            transaction_valid<=1'b1;
                        end else begin
                            pulse_valid <= 0;
                            current_state <= START;
                        end
                    end else begin
                        current_state <= START; // 若無脈衝，回到初始狀態
                    end
                end
                
                TRANSACTION_VALID_CHECK: begin
                    if (transaction_valid) begin//正在接受交易中
                        current_state <= TWO_US_DELAY; 
                    end else begin
                        current_state <= START;//回到 START 狀態
                    end
                end

		//判斷P1->P2時間在區間內,開啟DPSK demodulation 開啟I/Q解調FLAG
                TWO_US_DELAY: begin
                    if (timer < TWO_US_COUNT) begin
                        timer <= timer + 1;
                    end else begin
                        if (pulse_received) begin
                            p2_start_time <= $time; // 記錄 P2 脈衝開始時間
                            if (p2_start_time - p1_end_time >= MIN_SPACING && p2_start_time - p1_end_time <= MAX_SPACING) begin
                                current_state <= SUPPRESS_ATCRBS;
                                MAX2112FLAG <= 1'b1; // 設置 MAX2112FLAG=1to demodulate dpsk
                            end else begin
                                current_state <= ATCRBS_SUPPRESSED;
                                MAX2112FLAG <= 1'b0; // 清除 MAX2112FLAG
                            end
                        end else begin
                            current_state <= ATCRBS_SUPPRESSED;
                            MAX2112FLAG <= 1'b0; // 清除 MAX2112FLAG
                        end
                    end
                end

		//執行抑制AC MODE
                SUPPRESS_ATCRBS: begin
                    atcrbs_suppressed_flag <=1'b1;
                    atcrbs_suppressed<=1'b1;
                    current_state <= P6_SYNC_CHECK;
                end
		//判斷是否抑制 modeA/C SIGNAL
                ATCRBS_SUPPRESSED: begin
                    if (atcrbs_suppressed_flag) begin
                        current_state <=START; 
                    end else begin
                        current_state <=SPACING_8_21US;
                    end
                end
                
  		//Demodulation DPSK 放在一起
                P6_SYNC_CHECK: begin
		    // 實例化 DPSK 解調器
            	dpsk_demodulator #(
   		      .SYNC_PHASE_DURATION(75),      // 例如改成 75 (1.25us @60MHz)
    		      .SYNC_PHASE_THRESHOLD(3000000), // 根據你的量測結果調整
    		      .DPSK_BITS(56)                 // 如果要使用 Mode S 長格式，可放 56
		) dpsk_demod (
    		      	 .clk              (clk),
    		      	 .reset            (reset),
    		      	 .i_in             (i_in),
     	 	      	 .q_in             (q_in),
    		      	 .sync_phase_done  (sync_phase_done),
    		      	 .data_out         (data_out),
    		     	 .sync_phase_rev   (sync_phase_rev)
		);

		// 接收 P6 bit 流並解析
		p6_capture #(
    		     .P6_BITS(56)
		) p6_cap (
    		     	.clk         (clk),
    		     	.reset       (reset),
    			.data_out    (data_out_bit),
   			.dpsk_done   (dpsk_done), // 你的邏輯: (bit_counter == 55) => dpsk_done=1
    			.uf_code     (uf_code),
    			.pr          (pr),
    			.ic          (ic),
    			.cl          (cl),
			.Main_data   (Main_data),
			.AP	     (AP),
    			.p6_full_bits(p6_bits)
		);
                    if (p6_sync_phase_rev) begin
                        current_state <= UF_EQUIPPED_CHECK;
                    end else begin
                        recover = 1; 
                        current_state <= START;
                    end
                end

		// 檢查本答覆機是否支持UF FORMAT，目前支援 CASE的UF格式
                UF_EQUIPPED_CHECK: begin6
   		   case (uf_code)
      		 		5'd11, // UF-11 (All-Call)			//UF=11
        	 		5'd5,  // UF-5 (Altitude Request)		//UF=5
        	 		5'd4,  // UF-4 (Long Air-Air Surveillance)	//UF=4
        	 		5'd20, // UF-20 (Comm-B, Altitude Request)	//UF=20
         	 		5'd21: // UF-21 (Comm-B, Identity Request)	//UF=21
            	  	begin
                		uf_equipped <= 1'b1;  // 這些 UF 設定為可用
                		current_state <= ADDRESS_ASSIGNED_CHECK;
            	   	end
                   default:
            		begin
                		uf_equipped <= 1'b0;  // 其他 UF 設定為不可用
                		recover <= 1'b1;       // 進入復原模式，不回應該 UF
                		current_state <= START;
           		 end
                   endcase
		 end


                ADDRESS_ASSIGNED_CHECK: begin
                    if (address_assigned) begin
			
                        current_state = SHORT_FORMAT_CHECK;
                    end else begin
                        current_state = ADDRESS_FFFFFF;
                    end
                end
		 
		
                SHORT_FORMAT_CHECK: begin
                    if (short_format) begin
                        current_state = ACCEPT ;
                    end else begin
                        current_state =UNABLE_TO_PROCESS;
                    end
                end
                
                UNABLE_TO_PROCESS: begin
                    if (UNABLE_TO_PROCESS) begin
                        current_state = REPLY_OR_TRANSFER;
                    end else begin
                        current_state =UF_16DECISION;
                    end
                end

               ADDRESS_ffffff: begin
                    if (address == 24'hFFFFFF) begin
                        current_state = UF_11_DECISION;
                    end else begin
                        recover = 1;
                        current_state = START;
                    end
                end

                UF_16DECISION: begin
                    if (uf_code == 16) begin
                        recover = 1;
                        current_state = START;
                    end else begin
                        current_state =REPLY_OR_TRANSFER;
                    end
                end

                UF_11_DECISION: begin
                    if (uf_code == 11) begin
                        current_state = PROBABILITY_0t4;
                    end else begin
                        current_state = LONG_FORMAT_NOT_ELM;
                    end
                end


                PROBABILITY_0_TO_4: begin
                    if (PROBABILITY_0t4) begin
                        //make probability decision,repl->ACCEPT: not reply
                    end else begin
                        current_state = PROBABILITY_8_TO_12;
                    end
                end
                
                PROBABILITY_8_TO_12: begin
                    if (PROBABILITY_8_TO_12) begin
                        //make probability decision,repl->cl=0: not reply
                    end else begin
                        recover = 1;
                        current_state = START;
                    end
                end
                
                cl_0: begin
                    if (cl_11_0==0) begin
                        current_state =II_0; 
                    end else begin
                        current_state =SI_1_TO_63; 
                    end
                end

                ii_0: begin
                    if (ii_code) begin
                        current_state =TIMER_RUNNING;
                    end else begin
                        current_state =T1II;
                    end
                end

                T1II: begin
                    if (T1II) begin
                        recover = 1;
                        current_state = START;
                    end else begin
                        current_state = ACCEPT;
                    end
                end

                TD_Run: begin
                    if (T1II) begin
                        recover = 1;
                        current_state = START;
                    end else begin
                        current_state = ACCEPT;
                    end
                end

                P4_1_6: begin
                    if (p1_10us) begin
                        current_state = TIMER_RUNNING;
                    end else begin
                        recover = 1;
                        current_state = START;
                    end
                end

                SI_1_63: begin
                    if (si_1_63) begin
                        current_state = T1SI;
                    end else begin
                        recover = 1;
                        current_state = START;
                    end
                end

                T1SI: begin
                    if (si_1_63) begin
                        recover = 1;
                        current_state = START;
                    end else begin
                        current_state = ACCEPT;
                    end
                end

                LONG_FORMAT_NOT_ELM: begin
                    if (long_format) begin
                        current_state = LONG_FORMAT_NOT_ELM_UNABLE_TO_PROCESS;
                    end else begin
                        recover = 1;
                        current_state = START;
                    end
                end
                LONG_FORMAT_NOT_ELM_UNABLE_TO_PROCESS: begin
                    if (long_format_processable) begin
                        recover = 1;
                        current_state = START;
                    end else begin
                        current_state =REPLY_OR_TRANSFER;
                    end

                end

                EIGHT_OR_21_SPACING: begin
                    if (pulse_spacing_8_or_21us) begin
                        current_state = P4_Edge;
                    end else begin
                        recover = 1;
                        current_state = START;
                    end
                end
                P4_Edge: begin
                    if (p4_edge) begin
                        current_state = P4_1_6US;
                    end else begin
                        //this is ATTCRBS->ACCEPT
                    end
                end

                REPLY_OR_TRANSFER: begin
                   //reply or transfer
                end

                ACCEPT: begin
                   //ACCEPT FUNCTION
                end

                default: current_state = START;
            endcase
        end
    end
endmodule
