// clk_ctrl.sv (110_3; 10_6: unchanged logic, 24 SMs, MMCM or PLL)
//
// Live per-SM frequency control: rewrites ONE SM's MMCM / PLL CLKOUT1
// divider through its DRP port, while every other SM keeps running.
// (In the 10_6 build "tile" below = SM; slot-1 SMs use the region's
// PLLE2_ADV, whose CLKOUT1 / POWER DRP registers have the same addresses
// and fields as the MMCME2_ADV's -- XAPP888.)
// Replaces v7.3 mmcm_drp_ctrl.v (placeholder table, one MMCM shared with
// clk_mem, therefore disabled). Runs in clk_mem (200 MHz = DCLK).
//
// One request = one tile t, one divider D (f = 1200 MHz / D):
//   1. gate_off[t] = 1, wait until that tile's clock has stopped (ceon_s = 0)
//   2. mmcm_rst[t] = 1   (the XAPP888 flow: reconfigure while in reset)
//   3. DRP, for each entry {addr, keep, data}:
//        keep == 0 : write data
//        else      : read addr -> DO; write (DO & keep) | data
//      0x28 POWER        keep 0x0000  data 0xFFFF
//      0x0A CLKOUT1 Reg1 keep 0x1000  data {PHASE_MUX=000, 0, HIGH[5:0], LOW[5:0]}
//      0x0B CLKOUT1 Reg2 keep 0xFC00  data {MX=00, EDGE, NO_COUNT=0, DELAY=000000}
//      HIGH = D/2, LOW = D - D/2, EDGE = D odd  (50% duty)
//   4. mmcm_rst[t] = 0, wait for LOCKED (3 ms timeout)
//   5. gate_off[t] = 0, wait until the clock runs again (ceon_s = 1)
//   6. div_cur[t] = D, ack_ok = 1 (0 if any step timed out), flip ack_tog
// The VCO (CLKFBOUT, DIVCLK) is never touched, so the lock / filter
// registers do not change. Register map: UG472 / XAPP888 MMCME2 DRP
// (same addresses and masks as the Linux axi-clkgen driver).
//
// Boot: after reset every tile whose BOOT_DIV differs from DIV_MIN gets
// this sequence once (bitstream = DIV_MIN = 400 MHz for timing; the tiles
// then start at BOOT_DIV, 200 MHz by default, to stay well inside the
// VC707's 20 A VCCINT limit). boot_done = 1 when finished; vc707_top.sv
// keeps the tiles in reset until then.
//
// Requests from host_if.sv: req_tog flips with req_tile / req_div stable;
// invalid tile / divider -> ack with ack_ok = 0, nothing touched.

module clk_ctrl import gals_pkg::*; #(
  parameter int                   NUM_TILES  = 12,
  parameter int                   DIV_MIN    = 3,
  parameter int                   DIV_MAX    = 126,
  parameter logic [8*NUM_TILES-1:0] BOOT_DIV = {NUM_TILES{8'd6}},
  parameter int                   DRP_ENABLE = 1,
  parameter int                   LOCK_TMO   = 600_000,   // clk cycles (3 ms)
  parameter int                   GATE_TMO   = 4096,
  parameter int                   DRDY_TMO   = 256
)(
  input  logic                      clk,
  input  logic                      rst_n,

  input  logic                      req_tog,
  input  logic [7:0]                req_tile,
  input  logic [7:0]                req_div,
  output logic                      ack_tog,
  output logic                      ack_ok,
  output logic                      busy,
  output logic                      boot_done,
  output logic [8*NUM_TILES-1:0]    div_cur,

  input  logic [NUM_TILES-1:0]      locked_s,
  input  logic [NUM_TILES-1:0]      ceon_s,

  output logic [NUM_TILES-1:0]      drp_den,
  output logic                      drp_dwe,
  output logic [6:0]                drp_daddr,
  output logic [15:0]               drp_di,
  input  logic [16*NUM_TILES-1:0]   drp_do_flat,
  input  logic [NUM_TILES-1:0]      drp_drdy,
  output logic [NUM_TILES-1:0]      mmcm_rst,
  output logic [NUM_TILES-1:0]      gate_off
);

  typedef enum logic [3:0] {
    S_BOOT     = 4'd0,
    S_IDLE     = 4'd1,
    S_GATE_W   = 4'd2,    // 1: wait until the SM clock has stopped
    S_RST_W    = 4'd3,    // 2: MMCM / PLL reset settles
    S_OP       = 4'd4,    // 3: next DRP access (op 0..2)
    S_RD_W     = 4'd5,    //    read-modify-write: wait for DRDY of the read
    S_WR       = 4'd6,    //    write
    S_WR_W     = 4'd7,    //    wait for DRDY of the write
    S_LOCK_W   = 4'd8,    // 4: release reset, wait for LOCKED
    S_UNGATE_W = 4'd9,    // 5: wait until the SM clock runs again
    S_DONE     = 4'd10    // 6: report
  } state_e;

  state_e      state   = S_BOOT;
  logic [7:0]  boot_t  = '0;
  logic        in_boot = 1'b1;
  logic [7:0]  cur_t   = '0;
  logic [7:0]  cur_d   = '0;
  logic [1:0]  op      = '0;
  logic        err     = 1'b0;
  logic [19:0] tmo     = '0;
  logic [15:0] wval    = '0;
  logic        req_d   = 1'b0;

  assign busy = (state != S_IDLE);

  // divider fields of cur_d
  logic [5:0] hi_t, lo_t;
  logic       edge_t;
  assign hi_t   = cur_d[6:1];
  assign lo_t   = cur_d[6:0] - {1'b0, cur_d[6:1]};
  assign edge_t = cur_d[0];

  // the three DRP accesses: {address, bits to keep, value}
  logic [6:0]  op_addr;
  logic [15:0] op_keep, op_data;
  always_comb begin
    case (op)
      2'd0:    begin op_addr = DRP_POWER;    op_keep = 16'h0000; op_data = 16'hFFFF; end
      2'd1:    begin op_addr = DRP_CLKOUT1A; op_keep = 16'h1000; op_data = {3'b000, 1'b0, hi_t, lo_t}; end
      default: begin op_addr = DRP_CLKOUT1B; op_keep = 16'hFC00; op_data = {6'd0, 2'b00, edge_t, 1'b0, 6'd0}; end
    endcase
  end

  logic [15:0] do_cur;
  logic        drdy_cur, lk_cur, ce_cur;
  assign do_cur   = drp_do_flat[cur_t*16 +: 16];
  assign drdy_cur = drp_drdy[cur_t];
  assign lk_cur   = locked_s[cur_t];
  assign ce_cur   = ceon_s[cur_t];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state     <= S_BOOT;
      boot_t    <= '0;
      in_boot   <= 1'b1;
      boot_done <= 1'b0;
      cur_t     <= '0;
      cur_d     <= '0;
      op        <= '0;
      err       <= 1'b0;
      tmo       <= '0;
      wval      <= '0;
      req_d     <= 1'b0;
      ack_tog   <= 1'b0;
      ack_ok    <= 1'b0;
      drp_den   <= '0;
      drp_dwe   <= 1'b0;
      drp_daddr <= '0;
      drp_di    <= '0;
      mmcm_rst  <= '0;
      gate_off  <= '0;
      for (int t = 0; t < NUM_TILES; t++)
        div_cur[t*8 +: 8] <= 8'(DIV_MIN);
    end else begin
      drp_den <= '0;                        // DEN: one-cycle pulses
      tmo     <= tmo + 1'b1;

      case (state)
        // ------------------------------------------------- boot sequence
        S_BOOT: begin
          if (boot_t >= NUM_TILES || DRP_ENABLE == 0) begin
            in_boot   <= 1'b0;
            boot_done <= 1'b1;
            req_d     <= req_tog;             // ignore anything sent during boot
            state     <= S_IDLE;
          end else if (BOOT_DIV[boot_t*8 +: 8] == DIV_MIN) begin
            boot_t <= boot_t + 1'b1;          // already at the compiled divider
          end else begin
            cur_t <= boot_t;
            cur_d <= BOOT_DIV[boot_t*8 +: 8];
            err   <= 1'b0;
            gate_off[boot_t] <= 1'b1;
            tmo   <= '0;
            state <= S_GATE_W;
          end
        end

        // ------------------------------------------------- host requests
        S_IDLE: begin
          req_d <= req_tog;
          if (req_tog != req_d) begin
            if (DRP_ENABLE == 0 || req_tile >= NUM_TILES || req_div < DIV_MIN || req_div > DIV_MAX) begin
              ack_ok  <= 1'b0;
              ack_tog <= ~ack_tog;
            end else begin
              cur_t <= req_tile;
              cur_d <= req_div;
              err   <= 1'b0;
              gate_off[req_tile] <= 1'b1;
              tmo   <= '0;
              state <= S_GATE_W;
            end
          end
        end

        // ------------------------------------------------- 1: stop the clock
        S_GATE_W: begin
          if (!ce_cur || tmo >= GATE_TMO) begin
            if (ce_cur) err <= 1'b1;
            mmcm_rst[cur_t] <= 1'b1;
            tmo   <= '0;
            state <= S_RST_W;
          end
        end

        // ------------------------------------------------- 2: reset settles
        S_RST_W: begin
          if (tmo >= 20'd16) begin
            op    <= 2'd0;
            state <= S_OP;
          end
        end

        // ------------------------------------------------- 3: DRP accesses
        S_OP: begin
          drp_daddr <= op_addr;
          tmo       <= '0;
          if (op_keep == 16'h0000) begin
            wval  <= op_data;
            state <= S_WR;
          end else begin
            drp_dwe        <= 1'b0;
            drp_den[cur_t] <= 1'b1;
            state          <= S_RD_W;
          end
        end

        S_RD_W: begin
          if (drdy_cur) begin
            wval  <= (do_cur & op_keep) | (op_data & ~op_keep);
            state <= S_WR;
          end else if (tmo >= DRDY_TMO) begin
            err   <= 1'b1;
            state <= S_WR;                    // write the plain value anyway
            wval  <= op_data;
          end
        end

        S_WR: begin
          drp_daddr      <= op_addr;
          drp_di         <= wval;
          drp_dwe        <= 1'b1;
          drp_den[cur_t] <= 1'b1;
          tmo            <= '0;
          state          <= S_WR_W;
        end

        S_WR_W: begin
          if (drdy_cur || tmo >= DRDY_TMO) begin
            if (!drdy_cur) err <= 1'b1;
            drp_dwe <= 1'b0;
            if (op == 2'd2) begin
              mmcm_rst[cur_t] <= 1'b0;        // 4: release, wait for lock
              tmo   <= '0;
              state <= S_LOCK_W;
            end else begin
              op    <= op + 1'b1;
              state <= S_OP;
            end
          end
        end

        // ------------------------------------------------- 4: relock
        S_LOCK_W: begin
          if ((lk_cur && tmo >= 20'd16) || tmo >= LOCK_TMO) begin
            if (!lk_cur) err <= 1'b1;
            gate_off[cur_t] <= 1'b0;          // 5: let the clock through again
            tmo   <= '0;
            state <= S_UNGATE_W;
          end
        end

        S_UNGATE_W: begin
          if (ce_cur || tmo >= GATE_TMO) begin
            if (!ce_cur) err <= 1'b1;
            state <= S_DONE;
          end
        end

        // ------------------------------------------------- 6: report
        S_DONE: begin
          if (!err) div_cur[cur_t*8 +: 8] <= cur_d;
          if (in_boot) begin
            boot_t <= boot_t + 1'b1;
            state  <= S_BOOT;
          end else begin
            ack_ok  <= ~err;
            ack_tog <= ~ack_tog;
            state   <= S_IDLE;
          end
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule
