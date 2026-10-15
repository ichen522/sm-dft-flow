// host_if.sv (10_6, build 0x90)
//
// UART command parser = the GPU's "host interface" (the PCIe front end of
// a real GPU). clk_mem domain (200 MHz), 921,600 baud 8-N-1 through the
// VC707's CP2103 (uart.sv, unchanged). Reaches the SMs only through
// registered buses (vc707_top.sv spine) and the toggle / snapshot
// handshakes in sm_top.sv.
//
// ---- wire protocol (multi-byte fields big-endian / MSB first) -----------
// TT = SM index 0..NUM_SM-1 (WRITE_A / WRITE_B also accept FF = all SMs).
// SM s belongs to GPC s / 6, TPC (s % 6) / 2, slot s % 2 (vc707_top.sv).
// MASK = one bit per SM, MB = ceil(NUM_SM / 8) bytes (3 for 24 SMs), MSB
// byte first: bit s = SM s.
// A command with an invalid TT, divider or a timeout answers one byte E1.
//
//  op  name       command                      reply
//  01  WRITE_A    01 TT AH AL D0..D(2R-1)      A0      R = N_SUB*ROWS = 8 rows
//        one A column for BRAM address AH:AL: A[0][k]..A[R-1][k], 16-bit
//        each; the data bytes are the R*16-bit word MSB first, word bits
//        [i*16 +: 16] = A[i][k] (i = output row 0..R-1)
//  0B  WRITE_B    0B TT AH AL D0..D15          A0
//        one B row: word bits [j*16 +: 16] = B[k][j], j = 0..7
//  02  READ_ACT   02 TT                        4 bytes: activity_count (en cycles)
//  03  READ_CHK   03 TT                        8 bytes: checksum (finished blocks)
//  0C  READ_STAT  0C TT                        16 bytes: activity(4) blk_done(4) chk(8)
//        (one snapshot: blk_done and chk always belong together)
//  04  SET_EN     04 MASK(MB)                  A0   SM s runs continuously if bit s
//        is set (ORed with the DIP switches, see vc707_top.sv)
//  05  SET_FREQ   05 TT DD                     A0 / E1
//        SM TT clock = 1200 MHz / DD, DIV_MIN <= DD <= DIV_MAX. Only that
//        SM's clock pauses (~100 us) while its MMCM / PLL is reprogrammed
//  10  GET_FREQ   10 TT                        DD ST   ST bit0 locked, bit1 clock
//        running, bit2 clk_ctrl busy, bit3 boot finished
//  06  SEQ_WR     06 TT IH IL VV               A0   (as v7)
//  07  SEQ_CFG    07 FF LH LL W3 W2 W1 W0      A0   (as v7)
//  08  READ_WIN   08 MM                        ST CC + CC records of 4 + 3*NUM_SM
//        bytes: window index (4), then N_k of SM 0 .. SM NUM_SM-1, 3 bytes
//        each (24 bit, saturating; exact up to 16.7 M cycles = 41 ms at
//        400 MHz). 3 instead of 4 bytes so 24 SMs x 1 ms windows (76 B/ms)
//        fit through 921,600 baud (92 B/ms).
//  09  PING       09                           47 41 4C 53 VV  ("GALS", VV = 0x90)
//  0A  TILE_CFG   0A MASK(MB) KH KL            A0 / E1
//        for every SM whose bit is set: K = KH:KL (clamped to K_MIN..1023),
//        BRAM address / checksum / blk_done -> 0. Reply after every
//        selected SM has acknowledged.
//  0D  RUN_ONE    0D TT                        A0 / E1
//        run exactly one block (K steps) now, independent of SET_EN; reply
//        when its results are in the result memory
//  0E  READ_RES   0E TT                        6*R*8 bytes (384): C[0][0], C[0][1], ..
//        C[R-1][7], 6 bytes each (48-bit two's complement, raw, before
//        ReLU) of the last RUN_ONE block
//  0F  INFO       0F                           12 bytes: NUM_SM N_SUB ROWS COLS AW
//        VCO_MHz(2) DIV_MIN DIV_MAX K_MIN SM_PER_GPC NK_BYTES
//
// Robustness (from v7.1): a command whose payload stops for 10 ms is
// dropped; a command waiting on an SM answers E1 after 10 ms; unknown
// opcodes are ignored.
//
// The opcodes (op_e), ACK and NACK live in gals_pkg.sv.

module host_if import gals_pkg::*; #(
  parameter int          NUM_TILES       = 24,      // = NUM_SM
  parameter int          N_SUB           = 4,
  parameter int          ROWS            = 2,
  parameter int          COLS            = 8,
  parameter int          AW              = 9,
  parameter int          KW              = 10,
  parameter int          K_MIN           = 40,
  parameter int          K_DEFAULT       = 256,
  parameter int          CLK_FREQ_HZ     = 200_000_000,
  parameter int          BAUD_RATE       = 921_600,
  parameter int          REC_W           = 32 + 24*NUM_TILES,
  parameter int          CMD_TIMEOUT_CYC = CLK_FREQ_HZ / 100,   // 10 ms
  parameter int          VCO_MHZ         = 1200,
  parameter int          DIV_MIN         = 3,
  parameter int          DIV_MAX         = 126,
  parameter logic [7:0]  BUILD_VER       = 8'h90,
  parameter int          SM_PER_GPC      = 6,
  parameter int          NK_W            = 24,
  // plain parameters: do not override
  parameter int          NR              = N_SUB * ROWS * COLS,
  parameter int          RAW             = $clog2(N_SUB * ROWS * COLS),
  parameter int          WDW             = N_SUB * ROWS * 16
)(
  input  logic                        clk,
  input  logic                        rst_n,
  input  logic                        rxd,
  output logic                        txd,

  // BRAM writes (one-cycle strobes, one bit per SM)
  output logic [NUM_TILES-1:0]        wr_stb_a,
  output logic [NUM_TILES-1:0]        wr_stb_b,
  output logic [AW-1:0]               wr_addr,
  output logic [WDW-1:0]              wr_data,

  // result memories
  output logic [RAW-1:0]              res_raddr,
  input  logic [48*NUM_TILES-1:0]     res_rdata_flat,

  // snapshots
  output logic [NUM_TILES-1:0]        stat_req_tog,
  input  logic [NUM_TILES-1:0]        stat_ack_tog,
  input  logic [128*NUM_TILES-1:0]    stat_data_flat,

  // TILE_CFG / RUN_ONE
  output logic [NUM_TILES-1:0]        clr_req_tog,
  input  logic [NUM_TILES-1:0]        clr_ack_tog,
  output logic [KW-1:0]               cfg_k,
  output logic [NUM_TILES-1:0]        os_req_tog,
  input  logic [NUM_TILES-1:0]        os_ack_tog,

  output logic [NUM_TILES-1:0]        tile_en_override,

  // clock control (clk_ctrl.sv)
  output logic                        freq_req_tog,
  output logic [7:0]                  freq_tile,
  output logic [7:0]                  freq_div,
  input  logic                        freq_ack_tog,
  input  logic                        freq_ok,
  input  logic                        freq_busy,
  input  logic                        boot_done,
  input  logic [8*NUM_TILES-1:0]      div_cur_flat,
  input  logic [NUM_TILES-1:0]        locked_s,
  input  logic [NUM_TILES-1:0]        ceon_s,

  // window engine (win_ctrl.sv)
  output logic                        cfg_wr,
  output logic [7:0]                  cfg_flags,
  output logic [15:0]                 cfg_seq_len,
  output logic [31:0]                 cfg_win_len,
  output logic                        seq_wr,
  output logic [7:0]                  seq_wr_tile,
  output logic [15:0]                 seq_wr_idx,
  output logic [7:0]                  seq_wr_val,
  output logic                        rec_pop,
  input  logic [REC_W-1:0]            rec_head,
  input  logic [15:0]                 rec_count,
  input  logic                        rec_overflow,
  input  logic                        win_running
);

  localparam int TIDX_W    = (NUM_TILES <= 1) ? 1 : $clog2(NUM_TILES);
  localparam int WD_BYTES  = WDW / 8;              // 16
  localparam int WB_BYTES  = COLS * 2;             // 16
  localparam int MB        = (NUM_TILES + 7) / 8;  // mask bytes (3 for 24 SMs)
  localparam int PAY_W     = 3 + ((WD_BYTES > WB_BYTES) ? WD_BYTES : WB_BYTES);
  localparam int MAX_PAY   = (PAY_W > MB + 2) ? PAY_W : MB + 2;
  localparam int REC_BYTES = REC_W / 8;
  localparam int REPLY_MAX = (REC_BYTES > 16) ? REC_BYTES : 16;
  localparam int WR_WAIT   = 8;
  localparam int RES_LAT   = 10;
  localparam logic [KW-1:0] KMAX_V = '1;
  localparam logic [KW-1:0] KMIN_V = KW'(K_MIN);
  localparam logic [15:0]   VCO16  = 16'(VCO_MHZ);

  // ---- UART ----------------------------------------------------------------
  logic       rx_valid;
  logic [7:0] rx_data;
  logic       tx_start;
  logic [7:0] tx_data;
  logic       tx_busy;

  uart #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(BAUD_RATE)) u_uart (
    .clk (clk), .rst_n (rst_n),
    .tx_start (tx_start), .tx_data (tx_data), .tx_busy (tx_busy), .txd (txd),
    .rxd (rxd), .rx_valid (rx_valid), .rx_data (rx_data)
  );

  // ---- state ---------------------------------------------------------------
  typedef enum logic [3:0] {
    ST_OPCODE   = 4'd0,     // wait for an opcode byte
    ST_PAYLOAD  = 4'd1,     // collect bytes_needed payload bytes
    ST_EXEC     = 4'd2,     // decode + start the command
    ST_WAIT     = 4'd3,     // wait for an SM / clk_ctrl handshake
    ST_SETTLE   = 4'd4,     // snapshot -> reply bytes
    ST_REPLY_LD = 4'd5,
    ST_REPLY_TX = 4'd6,
    ST_REPLY_WT = 4'd7,
    ST_WIN_WAIT = 4'd8,     // READ_WIN: next FIFO record
    ST_WIN_LOAD = 4'd9,
    ST_RES_ADDR = 4'd10,    // READ_RES: next result word
    ST_RES_WAIT = 4'd11,
    ST_RES_LOAD = 4'd12
  } state_e;

  // what follows the current reply
  typedef enum logic [1:0] {
    AFTER_NONE = 2'd0,      // back to ST_OPCODE
    AFTER_WIN  = 2'd1,      // more READ_WIN records
    AFTER_RES  = 2'd2       // more READ_RES words
  } after_e;

  state_e            state;
  op_e               opcode;   // also holds unknown bytes for one cycle (ignored)
  logic [7:0]        payload [MAX_PAY];
  logic [6:0]        byte_idx;
  logic [6:0]        bytes_needed;
  logic [31:0]       to_cnt;
  logic [7:0]        reply_buf [REPLY_MAX];
  logic [7:0]        reply_len;
  logic [7:0]        reply_idx;
  after_e            after;
  logic [7:0]        win_left;
  logic [3:0]        cnt;
  logic [TIDX_W-1:0] act;
  logic [7:0]        res_word;
  logic [127:0]      stat_sel;
  logic [47:0]       res_sel;

  logic [7:0]  tt;
  logic        tt_ok, tt_all;
  logic [15:0] p12, p01;
  assign tt     = payload[0];
  assign tt_ok  = (payload[0] < NUM_TILES);
  assign tt_all = (payload[0] == 8'hFF);
  assign p12    = {payload[1], payload[2]};   // address / mask / index fields
  assign p01    = {payload[0], payload[1]};

  // SET_EN / TILE_CFG: MB mask bytes (MSB byte first), TILE_CFG then KH KL
  logic [8*MB-1:0] pmask;
  for (genvar gm = 0; gm < MB; gm++) begin : PMASK
    assign pmask[(MB-1-gm)*8 +: 8] = payload[gm];
  end
  logic [15:0] pk;
  assign pk = {payload[MB], payload[MB+1]};

  // per-command "is the handshake complete" (ack toggles follow req toggles)
  logic stat_done, os_done, clr_done, freq_done;
  assign stat_done = (stat_ack_tog[act] == stat_req_tog[act]);
  assign os_done   = (os_ack_tog[act]   == os_req_tog[act]);
  assign clr_done  = (clr_ack_tog       == clr_req_tog);
  assign freq_done = (freq_ack_tog      == freq_req_tog);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state            <= ST_OPCODE;
      opcode           <= op_e'(8'd0);
      byte_idx         <= '0;
      bytes_needed     <= '0;
      to_cnt           <= '0;
      reply_len        <= '0;
      reply_idx        <= '0;
      after            <= AFTER_NONE;
      win_left         <= '0;
      cnt              <= '0;
      act              <= '0;
      res_word         <= '0;
      stat_sel         <= '0;
      res_sel          <= '0;
      tx_start         <= 1'b0;
      tx_data          <= '0;
      wr_stb_a         <= '0;
      wr_stb_b         <= '0;
      wr_addr          <= '0;
      wr_data          <= '0;
      res_raddr        <= '0;
      stat_req_tog     <= '0;
      clr_req_tog      <= '0;
      os_req_tog       <= '0;
      cfg_k            <= KW'(K_DEFAULT);
      tile_en_override <= '0;
      freq_req_tog     <= 1'b0;
      freq_tile        <= '0;
      freq_div         <= '0;
      cfg_wr           <= 1'b0;
      cfg_flags        <= '0;
      cfg_seq_len      <= '0;
      cfg_win_len      <= '0;
      seq_wr           <= 1'b0;
      seq_wr_tile      <= '0;
      seq_wr_idx       <= '0;
      seq_wr_val       <= '0;
      rec_pop          <= 1'b0;
    end else begin
      tx_start <= 1'b0;
      cfg_wr   <= 1'b0;
      seq_wr   <= 1'b0;
      rec_pop  <= 1'b0;
      wr_stb_a <= '0;
      wr_stb_b <= '0;
      stat_sel <= stat_data_flat[act*128 +: 128];
      res_sel  <= res_rdata_flat[act*48 +: 48];

      case (state)
        // ---------------------------------------------------------- opcode
        ST_OPCODE: begin
          to_cnt <= '0;
          after  <= AFTER_NONE;
          if (rx_valid) begin
            opcode   <= op_e'(rx_data);
            byte_idx <= '0;
            case (rx_data)
              OP_WRITE_A:   begin bytes_needed <= 7'(3 + WD_BYTES); state <= ST_PAYLOAD; end
              OP_WRITE_B:   begin bytes_needed <= 7'(3 + WB_BYTES); state <= ST_PAYLOAD; end
              OP_READ_ACT,
              OP_READ_CHK,
              OP_READ_STAT,
              OP_RUN_ONE,
              OP_READ_RES,
              OP_GET_FREQ,
              OP_READ_WIN:  begin bytes_needed <= 7'd1;         state <= ST_PAYLOAD; end
              OP_SET_EN:    begin bytes_needed <= 7'(MB);       state <= ST_PAYLOAD; end
              OP_SET_FREQ:  begin bytes_needed <= 7'd2;         state <= ST_PAYLOAD; end
              OP_SEQ_WR:    begin bytes_needed <= 7'd4;         state <= ST_PAYLOAD; end
              OP_TILE_CFG:  begin bytes_needed <= 7'(MB + 2);   state <= ST_PAYLOAD; end
              OP_SEQ_CFG:   begin bytes_needed <= 7'd7;         state <= ST_PAYLOAD; end
              OP_PING,
              OP_INFO:      state <= ST_EXEC;
              default:      state <= ST_OPCODE;      // unknown: ignore
            endcase
          end
        end

        // --------------------------------------------------------- payload
        ST_PAYLOAD: begin
          if (rx_valid) begin
            to_cnt <= '0;
            payload[byte_idx] <= rx_data;
            if (byte_idx == bytes_needed - 1'b1) state <= ST_EXEC;
            else                                 byte_idx <= byte_idx + 1'b1;
          end else if (to_cnt >= CMD_TIMEOUT_CYC) begin
            state <= ST_OPCODE;
          end else begin
            to_cnt <= to_cnt + 32'd1;
          end
        end

        // ----------------------------------------------------------- exec
        ST_EXEC: begin
          to_cnt       <= '0;
          cnt          <= '0;
          reply_len    <= 8'd1;
          reply_buf[0] <= NACK;                       // default answer
          state        <= ST_REPLY_LD;
          act          <= payload[0][TIDX_W-1:0];
          case (opcode)
            OP_WRITE_A, OP_WRITE_B: begin
              if (tt_ok || tt_all) begin
                wr_addr <= p12[AW-1:0];
                wr_data <= '0;
                if (opcode == OP_WRITE_A) begin
                  for (int bi = 0; bi < WD_BYTES; bi++)
                    wr_data[(WD_BYTES-1-bi)*8 +: 8] <= payload[3+bi];
                end else begin
                  for (int bi = 0; bi < WB_BYTES; bi++)
                    wr_data[(WB_BYTES-1-bi)*8 +: 8] <= payload[3+bi];
                end
                for (int bi = 0; bi < NUM_TILES; bi++)
                  if (tt_all || tt == bi) begin
                    if (opcode == OP_WRITE_A) wr_stb_a[bi] <= 1'b1;
                    else                      wr_stb_b[bi] <= 1'b1;
                  end
                state <= ST_WAIT;
              end
            end

            OP_READ_ACT, OP_READ_CHK, OP_READ_STAT: begin
              if (tt_ok) begin
                stat_req_tog[payload[0][TIDX_W-1:0]] <= ~stat_req_tog[payload[0][TIDX_W-1:0]];
                state <= ST_WAIT;
              end
            end

            OP_RUN_ONE: begin
              if (tt_ok) begin
                os_req_tog[payload[0][TIDX_W-1:0]] <= ~os_req_tog[payload[0][TIDX_W-1:0]];
                state <= ST_WAIT;
              end
            end

            OP_READ_RES: begin
              if (tt_ok) begin
                res_word <= '0;
                state    <= ST_RES_ADDR;
              end
            end

            OP_SET_EN: begin
              for (int bi = 0; bi < NUM_TILES; bi++)
                tile_en_override[bi] <= pmask[bi];
              reply_buf[0] <= ACK;
            end

            OP_SET_FREQ: begin
              if (tt_ok && payload[1] >= DIV_MIN && payload[1] <= DIV_MAX && !freq_busy && boot_done) begin
                freq_tile    <= payload[0];
                freq_div     <= payload[1];
                freq_req_tog <= ~freq_req_tog;
                state        <= ST_WAIT;
              end
            end

            OP_GET_FREQ: begin
              if (tt_ok) begin
                reply_buf[0] <= div_cur_flat[payload[0][TIDX_W-1:0]*8 +: 8];
                reply_buf[1] <= {4'd0, boot_done, freq_busy,
                                 ceon_s[payload[0][TIDX_W-1:0]], locked_s[payload[0][TIDX_W-1:0]]};
                reply_len    <= 8'd2;
              end
            end

            OP_SEQ_WR: begin
              seq_wr       <= 1'b1;
              seq_wr_tile  <= payload[0];
              seq_wr_idx   <= p12;
              seq_wr_val   <= payload[3];
              reply_buf[0] <= ACK;
            end

            OP_SEQ_CFG: begin
              cfg_wr       <= 1'b1;
              cfg_flags    <= payload[0];
              cfg_seq_len  <= p12;
              cfg_win_len  <= {payload[3], payload[4], payload[5], payload[6]};
              reply_buf[0] <= ACK;
            end

            OP_READ_WIN: begin
              reply_buf[0] <= {6'd0, rec_overflow, win_running};
              if ((rec_count > 16'd255 ? 8'd255 : rec_count[7:0]) < payload[0]) begin
                reply_buf[1] <= (rec_count > 16'd255) ? 8'd255 : rec_count[7:0];
                win_left     <= (rec_count > 16'd255) ? 8'd255 : rec_count[7:0];
              end else begin
                reply_buf[1] <= payload[0];
                win_left     <= payload[0];
              end
              reply_len <= 8'd2;
              after     <= AFTER_WIN;
            end

            OP_PING: begin
              reply_buf[0] <= 8'h47;   // G
              reply_buf[1] <= 8'h41;   // A
              reply_buf[2] <= 8'h4C;   // L
              reply_buf[3] <= 8'h53;   // S
              reply_buf[4] <= BUILD_VER;
              reply_len    <= 8'd5;
            end

            OP_INFO: begin
              reply_buf[0] <= 8'(NUM_TILES);
              reply_buf[1] <= 8'(N_SUB);
              reply_buf[2] <= 8'(ROWS);
              reply_buf[3] <= 8'(COLS);
              reply_buf[4] <= 8'(AW);
              reply_buf[5] <= VCO16[15:8];
              reply_buf[6] <= VCO16[7:0];
              reply_buf[7] <= 8'(DIV_MIN);
              reply_buf[8] <= 8'(DIV_MAX);
              reply_buf[9] <= 8'(K_MIN);
              reply_buf[10]<= 8'(SM_PER_GPC);
              reply_buf[11]<= 8'(NK_W / 8);
              reply_len    <= 8'd12;
            end

            OP_TILE_CFG: begin
              cfg_k <= (pk < K_MIN) ? KMIN_V : ((pk > KMAX_V) ? KMAX_V : pk[KW-1:0]);
              for (int bi = 0; bi < NUM_TILES; bi++)
                if (pmask[bi]) clr_req_tog[bi] <= ~clr_req_tog[bi];
              state <= ST_WAIT;
            end

            default: state <= ST_OPCODE;
          endcase
        end

        // ------------------------------------------------- wait for a tile
        ST_WAIT: begin
          to_cnt <= to_cnt + 32'd1;
          if (to_cnt >= CMD_TIMEOUT_CYC) begin
            reply_buf[0] <= NACK;
            reply_len    <= 8'd1;
            state        <= ST_REPLY_LD;
          end else begin
            case (opcode)
              OP_WRITE_A, OP_WRITE_B: begin
                if (to_cnt >= WR_WAIT) begin
                  reply_buf[0] <= ACK;
                  state        <= ST_REPLY_LD;
                end
              end
              OP_READ_ACT, OP_READ_CHK, OP_READ_STAT: begin
                if (to_cnt >= 4 && stat_done) begin
                  cnt   <= '0;
                  state <= ST_SETTLE;
                end
              end
              OP_RUN_ONE: begin
                if (to_cnt >= 4 && os_done) begin
                  reply_buf[0] <= ACK;
                  state        <= ST_REPLY_LD;
                end
              end
              OP_TILE_CFG: begin
                if (to_cnt >= 4 && clr_done) begin
                  reply_buf[0] <= ACK;
                  state        <= ST_REPLY_LD;
                end
              end
              OP_SET_FREQ: begin
                if (to_cnt >= 4 && freq_done) begin
                  reply_buf[0] <= freq_ok ? ACK : NACK;
                  state        <= ST_REPLY_LD;
                end
              end
              default: state <= ST_OPCODE;
            endcase
          end
        end

        // ------------------------------------- snapshot -> reply bytes
        ST_SETTLE: begin
          cnt <= cnt + 1'b1;
          if (cnt == 4'd3) begin
            case (opcode)
              OP_READ_ACT: begin
                for (int bi = 0; bi < 4; bi++)
                  reply_buf[bi] <= stat_sel[127 - bi*8 -: 8];
                reply_len <= 8'd4;
              end
              OP_READ_CHK: begin
                for (int bi = 0; bi < 8; bi++)
                  reply_buf[bi] <= stat_sel[63 - bi*8 -: 8];
                reply_len <= 8'd8;
              end
              default: begin   // READ_STAT
                for (int bi = 0; bi < 16; bi++)
                  reply_buf[bi] <= stat_sel[127 - bi*8 -: 8];
                reply_len <= 8'd16;
              end
            endcase
            state <= ST_REPLY_LD;
          end
        end

        // ------------------------------------------- READ_RES: one word
        ST_RES_ADDR: begin
          res_raddr <= res_word[RAW-1:0];
          cnt       <= '0;
          state     <= ST_RES_WAIT;
        end

        ST_RES_WAIT: begin
          cnt <= cnt + 1'b1;
          if (cnt == RES_LAT) state <= ST_RES_LOAD;
        end

        ST_RES_LOAD: begin
          for (int bi = 0; bi < 6; bi++)
            reply_buf[bi] <= res_sel[47 - bi*8 -: 8];
          reply_len <= 8'd6;
          after     <= AFTER_RES;
          state     <= ST_REPLY_LD;
        end

        // ------------------------------------------------------- replies
        ST_REPLY_LD: begin
          reply_idx <= '0;
          state     <= ST_REPLY_TX;
        end

        ST_REPLY_TX: begin
          if (!tx_busy) begin
            tx_data  <= reply_buf[reply_idx];
            tx_start <= 1'b1;
            state    <= ST_REPLY_WT;
          end
        end

        ST_REPLY_WT: begin
          if (!tx_busy && !tx_start) begin
            if (reply_idx == reply_len - 1'b1) begin
              if (after == AFTER_WIN && win_left != 8'd0) begin
                cnt   <= '0;
                state <= ST_WIN_WAIT;
              end else if (after == AFTER_RES && res_word != NR - 1) begin
                res_word <= res_word + 1'b1;
                state    <= ST_RES_ADDR;
              end else begin
                state <= ST_OPCODE;
              end
            end else begin
              reply_idx <= reply_idx + 1'b1;
              state     <= ST_REPLY_TX;
            end
          end
        end

        // ------------------------------------------- READ_WIN records
        ST_WIN_WAIT: begin               // win_ctrl's FIFO head needs 3 cycles after a pop
          cnt <= cnt + 1'b1;
          if (cnt == 4'd3) state <= ST_WIN_LOAD;
        end

        ST_WIN_LOAD: begin
          for (int bi = 0; bi < REC_BYTES; bi++)
            reply_buf[bi] <= rec_head[(REC_BYTES-1-bi)*8 +: 8];
          reply_len <= 8'(REC_BYTES);
          rec_pop   <= 1'b1;
          win_left  <= win_left - 1'b1;
          state     <= ST_REPLY_LD;
        end

        default: state <= ST_OPCODE;
      endcase
    end
  end

endmodule
