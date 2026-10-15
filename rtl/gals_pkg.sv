// gals_pkg.sv  (build 0x90)
//
// Constants and helper functions shared by the RTL:
//
//   op_e            host command opcodes (host_if.sv; host/gals_host.py
//                   has the same table)
//   ACK / NACK      one-byte replies
//   DRP_*           MMCME2_ADV / PLLE2_ADV DRP registers that clk_ctrl.sv
//                   rewrites (UG472 / XAPP888; same addresses in both)
//   xs32 / hash3    xorshift hash for the power-on BRAM contents
//                   (sm_core.sv) and duty sequences (win_ctrl.sv);
//                   host/tile_model.py and host/gals_host.py compute the
//                   same numbers. Constant functions: Vivado evaluates
//                   them for the RAM INIT values.

package gals_pkg;

  // ---- host protocol --------------------------------------------------------
  typedef enum logic [7:0] {
    OP_WRITE_A   = 8'h01,
    OP_READ_ACT  = 8'h02,
    OP_READ_CHK  = 8'h03,
    OP_SET_EN    = 8'h04,
    OP_SET_FREQ  = 8'h05,
    OP_SEQ_WR    = 8'h06,
    OP_SEQ_CFG   = 8'h07,
    OP_READ_WIN  = 8'h08,
    OP_PING      = 8'h09,
    OP_TILE_CFG  = 8'h0A,
    OP_WRITE_B   = 8'h0B,
    OP_READ_STAT = 8'h0C,
    OP_RUN_ONE   = 8'h0D,
    OP_READ_RES  = 8'h0E,
    OP_INFO      = 8'h0F,
    OP_GET_FREQ  = 8'h10
  } op_e;

  localparam logic [7:0] ACK  = 8'hA0;
  localparam logic [7:0] NACK = 8'hE1;

  // ---- MMCM / PLL DRP registers (CLKOUT1 + POWER) ---------------------------
  localparam logic [6:0] DRP_POWER    = 7'h28;   // write 0xFFFF
  localparam logic [6:0] DRP_CLKOUT1A = 7'h0A;   // ClkReg1: PHASE_MUX, HIGH_TIME, LOW_TIME
  localparam logic [6:0] DRP_CLKOUT1B = 7'h0B;   // ClkReg2: MX, EDGE, NO_COUNT, DELAY_TIME

  // ---- power-on contents ----------------------------------------------------
  function automatic logic [31:0] xs32(input logic [31:0] x_in);
    logic [31:0] x;
    x = x_in;
    x = x ^ (x << 13);
    x = x ^ (x >> 17);
    x = x ^ (x << 5);
    return x;
  endfunction

  function automatic logic [31:0] hash3(input logic [31:0] seed, input logic [31:0] a,
                                        input logic [31:0] b);
    logic [31:0] x;
    x = seed ^ (a * 32'h9E3779B9) ^ (b * 32'h85EBCA6B) ^ 32'h1;
    if (x == 32'd0) x = 32'h1;
    x = xs32(x);
    x = xs32(x);
    return xs32(x);
  endfunction

endpackage
