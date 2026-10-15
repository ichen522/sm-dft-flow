// sm_core.sv  (10_6, build 0x90; was tile_core.v in 110_3)
//
// The compute part of one SM -- the FPGA version of one NVIDIA SM working
// on one GEMM output tile with all 4 sub-partitions together (like a
// Hopper warpgroup MMA, wgmma: the 4 warps / sub-partitions split the M
// rows of the tile, B is shared from shared memory):
//
//     C (8 x 8) = A (8 x K) x B (K x 8)          16-bit signed in, 48-bit out
//     (N_SUB = 4 sub-partitions x ROWS = 2 rows, COLS = 8 columns)
//
//   NVIDIA SM                          this module
//   ---------------------------------- --------------------------------------
//   shared memory: A / B tiles         A: one BRAM per sub-partition (ROWS x 16 bit)
//                                      B: one BRAM, shared (COLS x 16 bit)
//   TMA streams the next K-slice       the controller walks one BRAM address per k-step
//   4 sub-partitions, 1 tensor core    4 x tensor_core.sv, 2 x 8 DSP48E1 each = 64 MACs
//   accumulators in tensor memory      the DSPs' P registers (output stationary)
//   epilogue: TMEM -> regs, ReLU,      readout chains -> ReLU -> checksum,
//     write back                         result memory (one-shot blocks only)
//
// ---- what one k-step does ----------------------------------------------
// Block b, step k (0 <= k < K) reads BRAM address  a = (b*K + k) mod 512:
//     A column  : SUBP[s].mem[a][r*16 +: 16] = A[s*ROWS + r][k]   (s = sub, r = row)
//     B row     : bmem[a][c*16 +: 16]         = B[k][c]
// and all N_SUB*ROWS*COLS DSPs do  P[i][j] += A[i][k] * B[k][j]  in the same
// cycle. After K steps every P holds C[i][j] = sum_k A[i][k] * B[k][j]. The
// address keeps counting across blocks (wraps at 512), so with K = 256
// two different (A, B) slices alternate; TILE_CFG resets it to 0.
//
// ---- the enable (power experiment) --------------------------------------
// step = (en | one-shot pending) & not flushing. en comes from en_gen.sv (the
// duty-cycle FSM: 100%..80% per 1 ms window). A cycle without a step reads
// no BRAM and changes no register in the datapath, so a stopped SM only
// burns clock power.
//
// ---- readout / checksum (the "epilogue") ---------------------------------
// When a block's last step reaches the DSPs, all P values are copied into
// the 4 readout chains (tensor_core.sv) and shifted out 4 per cycle over
// ROWS*COLS (16) cycles, while the DSPs already accumulate the next block.
// Each value goes through ReLU into a running 64-bit checksum; when the
// whole block is in, `chk` and `blk_done` update together (so a snapshot of
// the two is always consistent):
//     chk = sum over finished blocks of sum_ij max(C_b[i][j], 0)   mod 2^64
// The host checks chk / blk_done against host/tile_model.py. The adders
// are split so no carry chain is longer than 33 bits (400 MHz target).
// A block started with RUN_ONE (os_pulse) also writes its raw C values
// into the result memory (LUTRAM; read from clk_mem by host_if READ_RES)
// and flips os_ack_tog when done. Continuous blocks do not write it.
//
// Needs K >= K_MIN (40): the readout of one block must finish before the
// next block's results are captured (checked by sva/sm_core_sva.sv).
//
// ---- clear (TILE_CFG) ----------------------------------------------------
// clr_pulse: latch K from cfg_k, address / k / checksum / blk_done -> 0,
// then HOLD cycles with no steps so every in-flight tag drains, then
// clr_ack_tog flips.
//
// Pipeline (issue cycle t): BRAM ren t+1, read latch t+2, DO_REG t+3,
// tensor_core.sv t+4..t+8 (P updated end of t+7, captured end of t+8),
// results out t+9 .., checksum committed ~t+31.
//
// The BRAM / LUTRAM write ports use plain `always`: those arrays also have
// an `initial` block (power-on contents), and a variable written in an
// always_ff may not be written by any other process.

module sm_core import gals_pkg::*; #(
  parameter int          N_SUB     = 4,      // sub-partitions (fixed 4: checksum adder tree)
  parameter int          ROWS      = 2,      // rows of C per sub-partition
  parameter int          COLS      = 8,      // columns of C
  parameter int          AW        = 9,      // BRAM address bits (depth 512)
  parameter int          KW        = 10,     // K register width (K <= 1023)
  parameter int          K_MIN     = 40,
  parameter int          K_DEFAULT = 256,
  parameter int          HOLD      = 32,
  parameter logic [31:0] SEED_A    = 32'hA5A50000,   // power-on A contents (per SM)
  parameter logic [31:0] SEED_B    = 32'h5A5A1234,   // power-on B contents (same in every SM)
  // plain parameters (Vivado [Synth 8-196] history): do not override
  parameter int          NCH       = ROWS * COLS,              // results per sub-partition (16)
  parameter int          NR        = N_SUB * ROWS * COLS,      // results per block (64)
  parameter int          RAW       = $clog2(N_SUB * ROWS * COLS),
  parameter int          CHW       = $clog2(ROWS * COLS)
)(
  input  logic                     clk,          // this SM's clock
  input  logic                     rst_n,        // synchronized to clk, held >= 64 cycles
  input  logic                     en,           // en_gen.sv: this cycle is a working cycle

  input  logic                     clr_pulse,    // TILE_CFG (from tog_cdc)
  input  logic [KW-1:0]            cfg_k,        // quasi-static (clk_mem), sampled on clr_pulse
  output logic                     clr_ack_tog = 1'b0,
  input  logic                     os_pulse,     // RUN_ONE (from tog_cdc)
  output logic                     os_ack_tog  = 1'b0,

  // host writes: write ports of the dual-clock BRAMs (wr_clk = clk_mem)
  input  logic                     wr_clk,
  input  logic                     wr_stb_a,
  input  logic                     wr_stb_b,
  input  logic [AW-1:0]            wr_addr,
  input  logic [N_SUB*ROWS*16-1:0] wr_data,      // A: all N_SUB*ROWS*16 bits; B: low COLS*16 bits

  // result memory read port (asynchronous read; used from clk_mem while idle)
  input  logic [RAW-1:0]           res_raddr,    // r = row*COLS + col
  output logic [47:0]              res_rdata,

  output logic [31:0]              blk_done = '0,
  output logic [63:0]              chk      = '0,
  output logic                     busy
);

  localparam int DEPTH = 1 << AW;
  localparam int AWID  = ROWS * 16;
  localparam int BWID  = COLS * 16;
  localparam logic [CHW:0] NCH_V = NCH;

  initial if (N_SUB != 4) $display("sm_core: ERROR -- the checksum adder tree is written for N_SUB = 4");

  // ---- power-on BRAM contents (constant functions -> Vivado can evaluate
  //      them; host/tile_model.py computes the same numbers) ---------------
  function automatic logic [AWID-1:0] arow(input logic [31:0] seed, input logic [31:0] row);
    arow = '0;
    for (int q = 0; q < ROWS; q++)
      arow[q*16 +: 16] = hash3(seed, row, q);
  endfunction

  function automatic logic [BWID-1:0] brow(input logic [31:0] seed, input logic [31:0] row);
    brow = '0;
    for (int q = 0; q < COLS; q++)
      brow[q*16 +: 16] = hash3(seed, row, q);
  endfunction

  // ---- synchronous reset (rst_n already synchronized, see sm_top.sv) -----
  (* max_fanout = 32 *) logic srst = 1'b1;
  always_ff @(posedge clk) srst <= ~rst_n;

  // =======================================================================
  // controller: k counter, block boundaries, one-shot, clear
  // =======================================================================
  logic [KW-1:0] kk        = '0;
  logic [KW-1:0] k_m1      = KW'(K_DEFAULT - 1);
  logic          os_active = 1'b0;
  logic [5:0]    hold      = 6'(HOLD);
  logic          clr_pend  = 1'b0;

  localparam logic [KW-1:0] KMIN_V = KW'(K_MIN);

  logic          flush, step, is_first, is_last;
  logic [KW-1:0] k_req;
  assign flush    = (hold != 6'd0);
  assign step     = (en | os_active) & ~flush & ~clr_pulse & ~srst;
  assign is_first = (kk == '0);
  assign is_last  = (kk == k_m1);
  assign k_req    = (cfg_k < KMIN_V) ? KMIN_V : cfg_k;

  always_ff @(posedge clk) begin
    if (srst) begin
      kk          <= '0;
      k_m1        <= KW'(K_DEFAULT - 1);
      os_active   <= 1'b0;
      hold        <= 6'(HOLD);
      clr_pend    <= 1'b0;
      clr_ack_tog <= 1'b0;
    end else if (clr_pulse) begin
      kk        <= '0;
      k_m1      <= k_req - 1'b1;
      os_active <= 1'b0;
      hold      <= 6'(HOLD);
      clr_pend  <= 1'b1;
    end else begin
      if (flush) begin
        hold <= hold - 1'b1;
        if (hold == 6'd1 && clr_pend) begin
          clr_pend    <= 1'b0;
          clr_ack_tog <= ~clr_ack_tog;
        end
      end
      if (step)
        kk <= is_last ? '0 : kk + 1'b1;
      if (step && is_last) os_active <= os_pulse;   // one-shot block done (or the next one queued)
      else if (os_pulse)   os_active <= 1'b1;
    end
  end

  // tags: valid / first / last / one-shot, aligned with the BRAM DO_REG (t+3)
  typedef struct packed {
    logic o;      // one-shot block (RUN_ONE)
    logic l;      // last k-step of the block
    logic f;      // first k-step of the block
    logic v;      // a real k-step
  } tag_t;

  tag_t tg1 = '0, tg2 = '0, tg3 = '0;
  always_ff @(posedge clk) begin
    tg1 <= {step & is_last & os_active,   // o
            step & is_last,               // l
            step & is_first,              // f
            step};                        // v
    tg2 <= tg1;
    tg3 <= tg2;
  end

  // =======================================================================
  // B memory (shared by the 4 sub-partitions = "shared memory" B tile)
  // =======================================================================
  (* ram_style = "block" *) logic [BWID-1:0] bmem [DEPTH];
  initial
    for (int i = 0; i < DEPTH; i++)
      bmem[i] = brow(SEED_B, i);

  (* keep = "true" *) logic [BWID-1:0] b_wd = '0;
  (* keep = "true" *) logic [AW-1:0]   b_wa = '0;
  (* keep = "true" *) logic            b_we = 1'b0;
  always_ff @(posedge wr_clk) begin
    b_wd <= wr_data[BWID-1:0];
    b_wa <= wr_addr;
    b_we <= wr_stb_b;
  end
  always @(posedge wr_clk)                      // write port (see header)
    if (b_we) bmem[b_wa] <= b_wd;

  (* keep = "true" *) logic          b_ren = 1'b0;
  (* keep = "true" *) logic [AW-1:0] b_ra  = '0;
  logic [BWID-1:0] b_r1 = '0, b_r2 = '0;
  always_ff @(posedge clk) begin
    b_ren <= step;
    if (srst | clr_pulse) b_ra <= '0;
    else if (b_ren)       b_ra <= b_ra + 1'b1;
    if (b_ren) b_r1 <= bmem[b_ra];
    b_r2 <= b_r1;
  end

  // =======================================================================
  // A memories + sub-partitions (one tensor core each)
  // =======================================================================
  logic [47:0] sub_out [N_SUB];                 // readout chain outputs

  for (genvar s = 0; s < N_SUB; s++) begin : SUBP
    localparam logic [31:0] SA = SEED_A ^ ((s + 1) * 32'h7F4A7C15);

    (* ram_style = "block" *) logic [AWID-1:0] mem [DEPTH];
    initial
      for (int i = 0; i < DEPTH; i++)
        mem[i] = arow(SA, i);

    (* keep = "true" *) logic [AWID-1:0] wd = '0;
    (* keep = "true" *) logic [AW-1:0]   wa = '0;
    (* keep = "true" *) logic            we = 1'b0;
    always_ff @(posedge wr_clk) begin
      wd <= wr_data[s*AWID +: AWID];
      wa <= wr_addr;
      we <= wr_stb_a;
    end
    always @(posedge wr_clk)                    // write port (see header)
      if (we) mem[wa] <= wd;

    (* keep = "true" *) logic          ren = 1'b0;
    (* keep = "true" *) logic [AW-1:0] ra  = '0;
    logic [AWID-1:0] r1 = '0, r2 = '0;
    always_ff @(posedge clk) begin
      ren <= step;
      if (srst | clr_pulse) ra <= '0;
      else if (ren)         ra <= ra + 1'b1;
      if (ren) r1 <= mem[ra];
      r2 <= r1;
    end

    tensor_core #(.ROWS(ROWS), .COLS(COLS)) u_tc (
      .clk  (clk),
      .a_in (r2),
      .b_in (b_r2),
      .tv   (tg3.v),
      .tf   (tg3.f),
      .tl   (tg3.l),
      .out  (sub_out[s])
    );
  end

  // =======================================================================
  // readout control (same timing as the chains inside tensor_core.sv)
  // =======================================================================
  logic [3:0] dv = '0, dl = '0, dos = '0;        // t+4 .. t+7
  logic       capev = 1'b0, cap_os = 1'b0;       // t+8
  always_ff @(posedge clk) begin
    dv     <= {dv[2:0],  tg3.v};
    dl     <= {dl[2:0],  tg3.l};
    dos    <= {dos[2:0], tg3.o};
    capev  <= dv[3] & dl[3];
    cap_os <= dos[3];
  end

  logic [CHW:0]   rd_cnt = '0;
  logic [CHW-1:0] rd_j   = '0;
  logic           rd_os  = 1'b0;
  always_ff @(posedge clk) begin
    if (srst | flush) begin
      rd_cnt <= '0;
      rd_j   <= '0;
      rd_os  <= 1'b0;
    end else if (capev) begin
      rd_cnt <= NCH_V;
      rd_j   <= '0;
      rd_os  <= cap_os;
    end else if (rd_cnt != 0) begin
      rd_cnt <= rd_cnt - 1'b1;
      rd_j   <= rd_j + 1'b1;
    end
  end

  logic o_v, o_first, o_last;
  assign o_v     = (rd_cnt != 0);
  assign o_first = o_v & (rd_j == '0);
  assign o_last  = o_v & (rd_cnt == 1);

  // ---- result memory: raw C values of one-shot blocks ------------------
  logic [47:0] rm_out [N_SUB];
  for (genvar s = 0; s < N_SUB; s++) begin : RES
    (* ram_style = "distributed" *) logic [47:0] rm [NCH];
    initial for (int i = 0; i < NCH; i++) rm[i] = '0;
    always @(posedge clk)                       // write port (see header)
      if (o_v & rd_os) rm[rd_j] <= sub_out[s];
    assign rm_out[s] = rm[res_raddr[CHW-1:0]];
  end
  assign res_rdata = rm_out[res_raddr[RAW-1:CHW]];

  // =======================================================================
  // checksum: ReLU, split into bits [23:0] and [46:24], add up the block,
  // then add the block total into chk -- every adder <= 33 bits
  // =======================================================================
  logic                   c1_v = 1'b0, c1_f = 1'b0, c1_l = 1'b0, c1_o = 1'b0;
  logic [N_SUB-1:0][23:0] lo1 = '0;
  logic [N_SUB-1:0][22:0] hi1 = '0;
  always_ff @(posedge clk) begin
    c1_v <= o_v & ~flush & ~srst;
    c1_f <= o_first;
    c1_l <= o_last;
    c1_o <= rd_os;
    for (int q = 0; q < N_SUB; q++) begin       // ReLU: a negative C counts as 0
      lo1[q] <= sub_out[q][47] ? 24'd0 : sub_out[q][23:0];
      hi1[q] <= sub_out[q][47] ? 23'd0 : sub_out[q][46:24];
    end
  end

  logic        c2_v = 1'b0, c2_f = 1'b0, c2_l = 1'b0, c2_o = 1'b0;
  logic [24:0] lo01 = '0, lo23 = '0;
  logic [23:0] hi01 = '0, hi23 = '0;
  always_ff @(posedge clk) begin
    c2_v <= c1_v & ~flush;  c2_f <= c1_f;  c2_l <= c1_l;  c2_o <= c1_o;
    lo01 <= {1'b0, lo1[0]} + {1'b0, lo1[1]};
    lo23 <= {1'b0, lo1[2]} + {1'b0, lo1[3]};
    hi01 <= {1'b0, hi1[0]} + {1'b0, hi1[1]};
    hi23 <= {1'b0, hi1[2]} + {1'b0, hi1[3]};
  end

  logic        c3_v = 1'b0, c3_f = 1'b0, c3_l = 1'b0, c3_o = 1'b0;
  logic [25:0] lo4 = '0;
  logic [24:0] hi4 = '0;
  always_ff @(posedge clk) begin
    c3_v <= c2_v & ~flush;  c3_f <= c2_f;  c3_l <= c2_l;  c3_o <= c2_o;
    lo4  <= {1'b0, lo01} + {1'b0, lo23};
    hi4  <= {1'b0, hi01} + {1'b0, hi23};
  end

  logic [31:0] blk_lo = '0, blk_hi = '0;        // this block's ReLU sum, split
  logic        c4_l = 1'b0, c4_o = 1'b0;
  always_ff @(posedge clk) begin
    c4_l <= c3_v & c3_l & ~flush;
    c4_o <= c3_o;
    if (c3_v) begin
      blk_lo <= (c3_f ? 32'd0 : blk_lo) + {6'd0, lo4};
      blk_hi <= (c3_f ? 32'd0 : blk_hi) + {7'd0, hi4};
    end
  end

  logic [32:0] s_mid = '0;                      // block sum S = s_mid * 2^24 + s_lo
  logic [23:0] s_lo  = '0;
  logic        c5_l = 1'b0, c5_o = 1'b0;
  always_ff @(posedge clk) begin
    c5_l <= c4_l & ~flush;
    c5_o <= c4_o;
    if (c4_l) begin
      s_mid <= {1'b0, blk_hi} + {25'd0, blk_lo[31:24]};
      s_lo  <= blk_lo[23:0];
    end
  end

  logic [32:0] lo_sum = '0;
  logic [24:0] s_hi   = '0;
  logic        c6_l = 1'b0, c6_o = 1'b0;
  always_ff @(posedge clk) begin
    c6_l <= c5_l & ~flush;
    c6_o <= c5_o;
    if (c5_l) begin
      lo_sum <= {1'b0, chk[31:0]} + {1'b0, s_mid[7:0], s_lo};
      s_hi   <= s_mid[32:8];
    end
  end

  always_ff @(posedge clk) begin
    if (srst) begin
      chk        <= '0;
      blk_done   <= '0;
      os_ack_tog <= 1'b0;
    end else if (clr_pulse) begin
      chk        <= '0;
      blk_done   <= '0;
    end else if (c6_l & ~flush) begin
      chk        <= {chk[63:32] + {7'd0, s_hi} + {31'd0, lo_sum[32]}, lo_sum[31:0]};
      blk_done   <= blk_done + 1'b1;
      if (c6_o) os_ack_tog <= ~os_ack_tog;
    end
  end

  assign busy = (kk != '0) | os_active | o_v | flush;

endmodule
