// sram_model.v  -  behavioral single-port synchronous SRAM with injectable faults
//                  and row (word) redundancy, for the MBIST experiment
//
// Stands in for the macro the A memory of sm_core would be in an ASIC
// (AW = 9: 512 words x 32 bits). Read data is registered (available the
// cycle after the read), like a real synchronous SRAM.
//
// Fault model (set by the testbench before each BIST run; fa/fb = victim
// word/bit, fa2/fb2 = aggressor word/bit):
//   0 none
//   1 SAF   stuck-at fault: cell (fa,fb) always holds fv
//   2 TF    transition fault: fv = 1 -> cell cannot go 0->1, fv = 0 -> cannot go 1->0
//   3 CFin  inversion coupling: an up (fdir=1) / down (fdir=0) transition of the
//           aggressor inverts the victim
//   4 CFid  idempotent coupling: aggressor transition in direction fdir forces victim to fv
//   5 CFst  state coupling: while the aggressor holds fs, the victim is forced to fv
//   6 AF    address decoder fault: address fa2 selects the cell of address fa
//           (fa2's own word is never accessed; fa is accessed by two addresses)
//   7 SAFx3 cells (fa,fb), (fa2,fb2) and (fa+1,fb) stuck at fv: three failing
//           words, more than the spares (BIRA must report "not repairable")
// Victim and aggressor in different words = inter-word coupling (caught with
// a solid data background); in the same word = intra-word coupling (needs
// several data backgrounds).
//
// Repair: rep_en[i] / rep_addr[i] (programmed from the BIRA result) send every
// access to rep_addr[i] to fault-free spare word i.
module sram_model #(parameter AW = 9, DW = 32, SPARES = 2) (
  input  wire          clk,
  input  wire          ce,
  input  wire          we,
  input  wire [AW-1:0] addr,
  input  wire [DW-1:0] wdata,
  output reg  [DW-1:0] rdata
);
  reg [DW-1:0] mem   [0:(1<<AW)-1];
  reg [DW-1:0] spare [0:SPARES-1];
  reg          rep_en   [0:SPARES-1];
  reg [AW-1:0] rep_addr [0:SPARES-1];

  integer  ftype = 0;
  reg [AW-1:0] fa = 0, fa2 = 0;
  integer  fb = 0, fb2 = 0;
  reg      fv = 0, fdir = 0, fs = 0;

  integer i;
  task clear_repair;
    for (i = 0; i < SPARES; i = i + 1) begin rep_en[i] = 1'b0; rep_addr[i] = 0; end
  endtask
  initial clear_repair;

  function integer spare_of(input [AW-1:0] a);
    integer k;
    begin
      spare_of = -1;
      for (k = 0; k < SPARES; k = k + 1)
        if (rep_en[k] && rep_addr[k] == a) spare_of = k;
    end
  endfunction

  // address decoder fault: which word a given address really selects
  function [AW-1:0] decode(input [AW-1:0] a);
    decode = (ftype == 6 && a == fa2) ? fa : a;
  endfunction

  task write_main(input [AW-1:0] a, input [DW-1:0] d);
    reg [DW-1:0] old, nw;
    reg ag_old, ag_new;
    begin
      old = mem[a];
      nw  = d;
      if (ftype == 2 && a == fa) begin
        if (fv == 1'b1 && old[fb] === 1'b0 && d[fb] == 1'b1) nw[fb] = 1'b0;   // slow to rise
        if (fv == 1'b0 && old[fb] === 1'b1 && d[fb] == 1'b0) nw[fb] = 1'b1;   // slow to fall
      end
      if (ftype == 1 && a == fa) nw[fb] = fv;
      if (ftype == 7) begin
        if (a == fa)      nw[fb]  = fv;
        if (a == fa2)     nw[fb2] = fv;
        if (a == fa + 1'b1) nw[fb] = fv;
      end
      mem[a] = nw;
      if (a == fa2 && (ftype == 3 || ftype == 4)) begin
        ag_old = old[fb2];
        ag_new = nw[fb2];
        if (ag_old !== 1'bx && ag_old != ag_new && ag_new == fdir) begin
          if (ftype == 3) mem[fa][fb] = ~mem[fa][fb];
          else            mem[fa][fb] = fv;
        end
      end
      if (ftype == 5 && mem[fa2][fb2] === fs) mem[fa][fb] = fv;
    end
  endtask

  integer s;
  always @(posedge clk)
    if (ce) begin
      s = spare_of(addr);
      if (we) begin
        if (s >= 0) spare[s] <= wdata;
        else        write_main(decode(addr), wdata);
      end else begin
        if (s >= 0) rdata <= spare[s];
        else begin
          rdata <= mem[decode(addr)];
          if ((ftype == 1 || ftype == 7) && decode(addr) == fa) rdata[fb] <= fv;
          if (ftype == 7 && addr == fa2)        rdata[fb2] <= fv;
          if (ftype == 7 && addr == fa + 1'b1)  rdata[fb]  <= fv;
        end
      end
    end
endmodule
