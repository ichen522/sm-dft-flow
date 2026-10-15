// sm_clk.sv (10_6, build 0x90)
//
// One SM's own clock generator -- "every SM has its own clock", the power
// mechanism we add on top of the NVIDIA-style compute array. The XC7VX485T
// has one CMT per clock region = one MMCME2_ADV + one PLLE2_ADV. A TPC
// (two SMs) owns one clock region, so its SM 0 uses the MMCM (USE_PLL = 0)
// and its SM 1 the PLL (USE_PLL = 1): two fully independent clocks per
// region, each with its own VCO and its own reset, so reprogramming one
// never disturbs the other. constraints/pblocks.tcl LOCs both into the
// region of the TPC.
//
//   clk_mem 200 MHz -> VCO = 200 x MULT (6) = 1200 MHz -> CLKOUT1 / D
//      D:  3 = 400   4 = 300   5 = 240   6 = 200   8 = 150   12 = 100 MHz
//
// DIV_MIN is the divider compiled into the bitstream. Vivado derives the
// SM clock constraint from it, so timing is closed at the FASTEST allowed
// frequency; clk_ctrl.sv may only change D to a value >= DIV_MIN (slower),
// so timing holds at every setting. CLKOUT1 is used instead of CLKOUT0
// because its DRP registers have no fractional-divide fields.
//
// ---- glitch-free stop / start (clk_ctrl.sv drives the sequence) ----------
// While the MMCM / PLL is in reset or still locking, its output is not a
// clean clock. The SM is therefore clocked through a BUFHCE (a regional
// clock buffer: it only reaches this one clock region) in vc707_top.sv
// whose CE comes from `ce` here:
//   * ce is produced by an 8-flop shift register clocked by clk_free (a
//     plain BUFH of the same output), i.e. synchronous to the clock it
//     gates, so the BUFHCE never cuts a pulse short;
//   * gate_off = 1 (synchronized) clears it -> the SM clock stops cleanly
//     BEFORE clk_ctrl.sv puts the MMCM / PLL into reset (= per-SM clock
//     gating, which real GPUs also do);
//   * after relock and gate_off = 0, ce comes back 8 cycles later. The SM
//     keeps all its state across the stop (its reset is not tied to
//     LOCKED), it just resumes at the new frequency;
//   * at power-up ce stays 0 until locked (async clear from LOCKED = 0).
// Status back to clk_mem: locked_s, ceon_s (2-flop synchronized).
//
// The DRP port is driven from clk_mem (DCLK = clk_mem, 200 MHz) through
// one register stage here, inside the region.

(* keep_hierarchy = "yes" *)   // a member of the SM's Pblock: keep it as a unit
module sm_clk #(
  parameter int  USE_PLL      = 0,      // 0: MMCME2_ADV, 1: PLLE2_ADV
  parameter int  DIV_MIN      = 3,      // compiled divider = timing target (400 MHz)
  parameter int  MULT         = 6,      // VCO = 200 MHz x 6 = 1200 MHz
  parameter real CLKIN_PERIOD = 5.0     // ns (clk_mem)
)(
  input  logic        clk_mem,      // MMCM / PLL input clock, DCLK, and every control / status port

  input  logic        drp_den,
  input  logic        drp_dwe,
  input  logic [6:0]  drp_daddr,
  input  logic [15:0] drp_di,
  output logic [15:0] drp_do   = '0,
  output logic        drp_drdy = 1'b0,

  input  logic        mmcm_rst,     // clk_ctrl.sv: hold the MMCM / PLL in reset (during DRP writes)
  input  logic        gate_off,     // clk_ctrl.sv: stop the SM clock (before reset)
  output logic        locked_s,     // clk_mem domain
  output logic        ceon_s,       // clk_mem domain: SM clock is running

  output logic        clk_raw,      // CLKOUT1 -> BUFH (clk_free) and BUFHCE (SM clock)
  input  logic        clk_free,     // BUFH(clk_raw), only for the ce shift register
  output logic        ce            // -> BUFHCE.CE
);

  // ---- DRP / control register stage (clk_mem) ----------------------------
  logic        den_q  = 1'b0;
  logic        dwe_q  = 1'b0;
  logic [6:0]  addr_q = '0;
  logic [15:0] di_q   = '0;
  logic        rst_q  = 1'b0;
  logic        gate_q = 1'b0;
  logic [15:0] do_w;
  logic        drdy_w;
  always_ff @(posedge clk_mem) begin
    den_q    <= drp_den;
    dwe_q    <= drp_dwe;
    addr_q   <= drp_daddr;
    di_q     <= drp_di;
    rst_q    <= mmcm_rst;
    gate_q   <= gate_off;
    drp_do   <= do_w;
    drp_drdy <= drdy_w;
  end

  // ---- the clock generator: MMCM or PLL -----------------------------------
  // Both run the VCO at 1200 MHz and use CLKOUT1 = 1200 / D. Their DRP
  // registers for CLKOUT1 (0x0A ClkReg1, 0x0B ClkReg2) and the POWER
  // register 0x28 have the same addresses and fields (XAPP888), so
  // clk_ctrl.sv programs both the same way.
  logic locked;
  logic clkfb;

  if (USE_PLL) begin : G_PLL
    PLLE2_ADV #(
      .BANDWIDTH          ("OPTIMIZED"),
      .COMPENSATION       ("INTERNAL"),   // CLKFBOUT -> CLKFBIN directly
      .STARTUP_WAIT       ("FALSE"),
      .CLKIN1_PERIOD      (CLKIN_PERIOD),
      .DIVCLK_DIVIDE      (1),
      .CLKFBOUT_MULT      (MULT),
      .CLKFBOUT_PHASE     (0.0),
      .CLKOUT0_DIVIDE     (6),            // unused
      .CLKOUT1_DIVIDE     (DIV_MIN),
      .CLKOUT1_DUTY_CYCLE (0.5),
      .CLKOUT1_PHASE      (0.0)
    ) u_pll (
      .CLKIN1   (clk_mem),
      .CLKIN2   (1'b0),
      .CLKINSEL (1'b1),
      .CLKFBIN  (clkfb),
      .CLKFBOUT (clkfb),
      .CLKOUT0  (),
      .CLKOUT1  (clk_raw),
      .CLKOUT2  (),
      .CLKOUT3  (),
      .CLKOUT4  (),
      .CLKOUT5  (),
      .LOCKED   (locked),
      .RST      (rst_q),
      .PWRDWN   (1'b0),
      .DCLK     (clk_mem),
      .DEN      (den_q),
      .DWE      (dwe_q),
      .DADDR    (addr_q),
      .DI       (di_q),
      .DO       (do_w),
      .DRDY     (drdy_w)
    );
  end else begin : G_MMCM
    MMCME2_ADV #(
      .BANDWIDTH          ("OPTIMIZED"),
      .COMPENSATION       ("INTERNAL"),   // CLKFBOUT -> CLKFBIN directly
      .STARTUP_WAIT       ("FALSE"),
      .CLKIN1_PERIOD      (CLKIN_PERIOD),
      .DIVCLK_DIVIDE      (1),
      .CLKFBOUT_MULT_F    (MULT),
      .CLKFBOUT_PHASE     (0.0),
      .CLKOUT0_DIVIDE_F   (6.0),          // unused
      .CLKOUT1_DIVIDE     (DIV_MIN),
      .CLKOUT1_DUTY_CYCLE (0.5),
      .CLKOUT1_PHASE      (0.0)
    ) u_mmcm (
      .CLKIN1       (clk_mem),
      .CLKIN2       (1'b0),
      .CLKINSEL     (1'b1),
      .CLKFBIN      (clkfb),
      .CLKFBOUT     (clkfb),
      .CLKFBOUTB    (),
      .CLKOUT0      (),
      .CLKOUT0B     (),
      .CLKOUT1      (clk_raw),
      .CLKOUT1B     (),
      .CLKOUT2      (),
      .CLKOUT2B     (),
      .CLKOUT3      (),
      .CLKOUT3B     (),
      .CLKOUT4      (),
      .CLKOUT5      (),
      .CLKOUT6      (),
      .LOCKED       (locked),
      .CLKINSTOPPED (),
      .CLKFBSTOPPED (),
      .RST          (rst_q),
      .PWRDWN       (1'b0),
      .DCLK         (clk_mem),
      .DEN          (den_q),
      .DWE          (dwe_q),
      .DADDR        (addr_q),
      .DI           (di_q),
      .DO           (do_w),
      .DRDY         (drdy_w),
      .PSCLK        (1'b0),
      .PSEN         (1'b0),
      .PSINCDEC     (1'b0),
      .PSDONE       ()
    );
  end

  // ---- clock-enable for the BUFHCE (clk_free domain) ---------------------
  (* ASYNC_REG = "TRUE" *) logic [1:0] lk_f = 2'b00;   // LOCKED, async clear / sync release
  always_ff @(posedge clk_free or negedge locked)
    if (!locked) lk_f <= 2'b00;
    else         lk_f <= {lk_f[0], 1'b1};

  (* ASYNC_REG = "TRUE" *) logic [1:0] go_f = 2'b11;   // gate_off synchronizer
  always_ff @(posedge clk_free) go_f <= {go_f[0], gate_q};

  logic [7:0] ce_sh = '0;
  always_ff @(posedge clk_free or negedge locked)
    if (!locked)                 ce_sh <= '0;
    else if (go_f[1] | ~lk_f[1]) ce_sh <= '0;
    else                         ce_sh <= {ce_sh[6:0], 1'b1};
  assign ce = ce_sh[7];

  // ---- status back to clk_mem ------------------------------------------------
  (* ASYNC_REG = "TRUE" *) logic [1:0] lk_m = 2'b00;
  (* ASYNC_REG = "TRUE" *) logic [1:0] ce_m = 2'b00;
  always_ff @(posedge clk_mem) begin
    lk_m <= {lk_m[0], locked};
    ce_m <= {ce_m[0], ce_sh[7]};
  end
  assign locked_s = lk_m[1];
  assign ceon_s   = ce_m[1];

endmodule
