module dump_1090 (
    input clk,
    input reset,
//----------------------------------networking client----------------------------------------
 reg [31:0] client_fd; // File descriptor
  reg [15:0] client_service; // TCP port
  reg [7:0] client_buf [0:MODES_CLIENT_BUF_SIZE]; // Read buffer
  reg [15:0] client_buflen; // Amount of data on buffer,8192 bytes satisfy the most  hostname
//----------------------------------struct aircraft_data-------------------------------------
    output reg [31:0] addr,        // ICAO 24-bit Address
    output reg [7:0] hexaddr [0:6], // Hex ICAO Address
    output reg [7:0] flight [0:8], // 航班號碼 (8+1 ASCII 字元)
    output reg [31:0] altitude,    // 高度
    output reg [31:0] speed,       // 速度
    output reg [31:0] track,       // 航向
    output reg [63:0] seen,        // 最後收到的時間
    output reg [63:0] messages,    // Mode S 訊息數
    output reg [31:0] odd_cprlat, odd_cprlon,
    output reg [31:0] even_cprlat, even_cprlon,
    output reg [63:0] lat, lon,    // CPR 解析後的經緯度
//---------------------------------Program global state---------------------------------------
	

  // ... 其他輸入/輸出端口 ...
);
parameter MODES_DEFAULT_RATE         2000000
parameter MODES_DEFAULT_FREQ         1090000000
parameter MODES_DEFAULT_WIDTH        1000
parameter MODES_DEFAULT_HEIGHT       700
parameter MODES_ASYNC_BUF_NUMBER     12
parameter MODES_DATA_LEN             (16*16384)   /* 256k */
parameter MODES_AUTO_GAIN            -100         /* Use automatic gain. */
parameter MODES_MAX_GAIN             999999       /* Use max available gain. */

parameter MODES_PREAMBLE_US 8       /* microseconds */
parameter MODES_LONG_MSG_BITS 112
parameter MODES_SHORT_MSG_BITS 56
parameter MODES_FULL_LEN (MODES_PREAMBLE_US+MODES_LONG_MSG_BITS)
parameter MODES_LONG_MSG_BYTES (112/8)
parameter MODES_SHORT_MSG_BYTES (56/8)

parameter MODES_ICAO_CACHE_LEN 1024 /* Power of two required. */
parameter MODES_ICAO_CACHE_TTL 60   /* Time to live of cached addresses. */
parameter MODES_UNIT_FEET 0
parameter MODES_UNIT_METERS 1

parameter MODES_DEBUG_DEMOD (1<<0)
parameter MODES_DEBUG_DEMODERR (1<<1)
parameter MODES_DEBUG_BADCRC (1<<2)
parameter MODES_DEBUG_GOODCRC (1<<3)
parameter MODES_DEBUG_NOPREAMBLE (1<<4)
parameter MODES_DEBUG_NET (1<<5)
parameter MODES_DEBUG_JS (1<<6)

/* When debug is set to MODES_DEBUG_NOPREAMBLE, the first sample must be
 * at least greater than a given level for us to dump the signal. */
parameter MODES_DEBUG_NOPREAMBLE_LEVEL 25

parameter MODES_INTERACTIVE_REFRESH_TIME 250      /* Milliseconds */
parameter MODES_INTERACTIVE_ROWS 15               /* Rows on screen */
parameter MODES_INTERACTIVE_TTL 60                /* TTL before being removed */

parameter MODES_NET_MAX_FD 1024
parameter MODES_NET_OUTPUT_SBS_PORT 30003
parameter MODES_NET_OUTPUT_RAW_PORT 30002
parameter MODES_NET_INPUT_RAW_PORT 30001
parameter MODES_NET_HTTP_PORT 8080
parameter MODES_CLIENT_BUF_SIZE 1024
parameter MODES_NET_SNDBUF_SIZE (1024*64)

parameter MODES_NOTUSED(V) ((void) V)

//struct clientTCP 伺服器內部用來儲存每個客戶端的連鎖資訊,透過 TCP 連線接收 ADS-B 資料的用戶端

  initial begin
//------------------------------------initial aircraft--------------------------------------
        addr = 32'hAABBCC;  // 設定 ICAO Address
        altitude = 32'd35000; // 35000 英尺
        speed = 32'd450; // 速度 450 節
        track = 32'd90; // 航向 90 度
        lat = 64'd3723456789; // 模擬經度
        lon = 64'd123456789; // 模擬緯度
   	
    end

// 邏輯實現
  always @(posedge clk or posedge reset) begin
    if (reset) begin
	fd <= 0;
        service <= 0;
        buflen <= 0;
    end else begin
	fd <= new_fd;
        service <= new_service_port;
        buflen <= 0;
    end
  end

  // ... 其他模組邏輯 ...

endmodule
