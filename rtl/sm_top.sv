// sm_top.sv (10_6, build 0x90; was tile_top.v in 110_3)
//
// Everything that belongs to ONE SM -- this whole hierarchy goes into the
// SM's Pblock (constraints/pblocks.tcl): half of one clock region, the
// other half being the second SM of the same TPC. Next to it: the SM's own
// clock generator (sm_clk.sv, MMCM or PLL of that region's CMT).
//
//   SM clock domain                           clk_mem domain (200 MHz)
//   ---------------------------------------   --------------------------------
//   reset synchronizer + 64-cycle hold
//   en_gen.sv     duty-cycle enable FSM,      <- seq_run / win_toggle / duty_off
//                 activity_count, N_k         -> nk_hold (quasi-static)
//   sm_core.sv    BRAMs + 4 tensor cores      <- host writes (BRAM write ports)
//                 (64 DSP MACs) + readout     -> result memory read port
//                 + checksum
//   tog_cdc.sv x2 TILE_CFG / RUN_ONE          <-> toggles
//   stat_cdc.sv   {activity, blk_done, chk}   -> snapshot
//
// The clk_mem inputs arrive through one "spine" register stage in
// vc707_top.sv (placed between host_if.sv and the SM's region) and get one
// more register stage here, inside the region, so no clk_mem path has to
// cross the die in one cycle.

(* keep_hierarchy = "yes" *)   // one Pblock per SM: keep this hierarchy as a unit
module sm_top #(
  parameter int          N_SUB     = 4,
  parameter int          ROWS      = 2,
  parameter int          COLS      = 8,
  parameter int          AW        = 9,
  parameter int          KW        = 10,
  parameter int          K_MIN     = 40,
  parameter int          K_DEFAULT = 256,
  parameter int          DUTY_DEN  = 100,
  parameter logic [31:0] SEED_A    = 32'hA5A50000,
  parameter logic [31:0] SEED_B    = 32'h5A5A1234,
  // plain parameters: do not override
  parameter int          RAW       = $clog2(N_SUB * ROWS * COLS),
  parameter int          WDW       = N_SUB * ROWS * 16
)(
  input  logic                clk_mem,
  input  logic                rst_n_mem,

  input  logic                tile_clk,
  input  logic                tile_rst_req,    // async, active high (reset button / boot)

  input  logic                tile_en_in,      // async: DIP | host SET_EN
  input  logic                seq_run_in,      // win_ctrl.sv (clk_mem)
  input  logic                win_toggle_in,
  input  logic [7:0]          duty_off_in,
  output logic [31:0]         nk_hold,         // tile domain, quasi-static

  // host writes (clk_mem, already one spine stage)
  input  logic                wr_stb_a,
  input  logic                wr_stb_b,
  input  logic [AW-1:0]       wr_addr,
  input  logic [WDW-1:0]      wr_data,

  // results (clk_mem)
  input  logic [RAW-1:0]      res_raddr,
  output logic [47:0]         res_rdata,

  // snapshot (clk_mem): {activity_count, blk_done, chk}
  input  logic                stat_req_tog,
  output logic                stat_ack_tog,
  output logic [127:0]        stat_data,

  // TILE_CFG / RUN_ONE (clk_mem)
  input  logic                clr_req_tog,
  output logic                clr_ack_tog,
  input  logic [KW-1:0]       cfg_k,
  input  logic                os_req_tog,
  output logic                os_ack_tog,

  output logic                led_blk          // tile domain, for an LED only
);

  // ---- tile reset: async assert, sync release, then hold 64 cycles -----
  // (the v7.2 scheme: only these two flops have an async clear)
  (* ASYNC_REG = "TRUE" *) logic [1:0] rs = 2'b00;
  always_ff @(posedge tile_clk or posedge tile_rst_req)
    if (tile_rst_req) rs <= 2'b00;
    else              rs <= {rs[0], 1'b1};

  logic [6:0] hold_cnt = '0;
  logic       rst_n_q  = 1'b0;
  always_ff @(posedge tile_clk) begin
    if (!rs[1]) begin
      hold_cnt <= '0;
      rst_n_q  <= 1'b0;
    end else if (!hold_cnt[6]) begin
      hold_cnt <= hold_cnt + 7'd1;
    end else begin
      rst_n_q  <= 1'b1;
    end
  end
  logic tile_rst_n;
  assign tile_rst_n = rst_n_q;
  logic tsrst = 1'b1;
  always_ff @(posedge tile_clk) tsrst <= ~tile_rst_n;

  // ---- clk_mem input stage (inside the region) -------------------------
  (* keep = "true" *) logic            q_stb_a = 1'b0;
  (* keep = "true" *) logic            q_stb_b = 1'b0;
  (* keep = "true" *) logic [AW-1:0]   q_addr  = '0;
  (* keep = "true" *) logic [WDW-1:0]  q_data  = '0;
  (* keep = "true" *) logic [RAW-1:0]  q_raddr = '0;
  always_ff @(posedge clk_mem) begin
    q_stb_a <= wr_stb_a & rst_n_mem;
    q_stb_b <= wr_stb_b & rst_n_mem;
    q_addr  <= wr_addr;
    q_data  <= wr_data;
    q_raddr <= res_raddr;
  end

  // ---- enable FSM + activity counters (unchanged v7.2 en_gen) -----------
  logic        en;
  logic [31:0] activity;
  en_gen #(
    .DUTY_DEN (DUTY_DEN),
    .DUTY_W   (8)
  ) u_en_gen (
    .clk            (tile_clk),
    .rst_n          (tile_rst_n),
    .tile_en_in     (tile_en_in),
    .seq_run_in     (seq_run_in),
    .win_toggle_in  (win_toggle_in),
    .duty_off_in    (duty_off_in),
    .en             (en),
    .activity_count (activity),
    .nk_hold        (nk_hold),
    .duty_cur       (),
    .tick           ()
  );

  // ---- TILE_CFG / RUN_ONE handshakes --------------------------------------
  logic clr_pulse, os_pulse, clr_done_tog, os_done_tog;
  tog_cdc u_clr_cdc (
    .src_clk (clk_mem), .src_rst_n (rst_n_mem),
    .req_tog (clr_req_tog), .ack_tog (clr_ack_tog),
    .dst_clk (tile_clk), .pulse (clr_pulse), .done_tog (clr_done_tog)
  );
  tog_cdc u_os_cdc (
    .src_clk (clk_mem), .src_rst_n (rst_n_mem),
    .req_tog (os_req_tog), .ack_tog (os_ack_tog),
    .dst_clk (tile_clk), .pulse (os_pulse), .done_tog (os_done_tog)
  );

  // ---- the compute core -----------------------------------------------------
  logic [31:0] blk_done;
  logic [63:0] chk;
  logic [47:0] res_w;
  sm_core #(
    .N_SUB     (N_SUB),
    .ROWS      (ROWS),
    .COLS      (COLS),
    .AW        (AW),
    .KW        (KW),
    .K_MIN     (K_MIN),
    .K_DEFAULT (K_DEFAULT),
    .SEED_A    (SEED_A),
    .SEED_B    (SEED_B)
  ) u_core (
    .clk         (tile_clk),
    .rst_n       (tile_rst_n),
    .en          (en),
    .clr_pulse   (clr_pulse & ~tsrst),
    .cfg_k       (cfg_k),
    .clr_ack_tog (clr_done_tog),
    .os_pulse    (os_pulse & ~tsrst),
    .os_ack_tog  (os_done_tog),
    .wr_clk      (clk_mem),
    .wr_stb_a    (q_stb_a),
    .wr_stb_b    (q_stb_b),
    .wr_addr     (q_addr),
    .wr_data     (q_data),
    .res_raddr   (q_raddr),
    .res_rdata   (res_w),
    .blk_done    (blk_done),
    .chk         (chk),
    .busy        ()
  );

  // result memory read: LUTRAM (written in the tile domain, only while a
  // RUN_ONE block finishes) read asynchronously and registered in clk_mem.
  // host_if.sv only reads it after RUN_ONE has been acknowledged, so the
  // data is static -- the path is cut by the asynchronous clock groups.
  always_ff @(posedge clk_mem) res_rdata <= res_w;

  // ---- snapshot ---------------------------------------------------------------
  stat_cdc #(.W(128)) u_stat (
    .req_clk   (clk_mem),
    .req_rst_n (rst_n_mem),
    .req_tog   (stat_req_tog),
    .ack_tog   (stat_ack_tog),
    .data_out  (stat_data),
    .tile_clk  (tile_clk),
    .tile_srst (tsrst),
    .data_in   ({activity, blk_done, chk})
  );

  assign led_blk = blk_done[16];

endmodule
