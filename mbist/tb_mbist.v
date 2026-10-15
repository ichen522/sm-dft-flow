// tb_mbist.v  -  fault-injection experiment for the MBIST controller
//
// For every fault in faults.txt (one per line:
//   id class ftype fa fb fa2 fb2 fv fdir fs )
// the testbench runs MATS+, March X and March C-, each with the solid data
// background and with log2(DW)+1 backgrounds, and prints which runs detect
// the fault. If March C- (all backgrounds) detects it and the BIRA says it is
// repairable, the repair is programmed into the SRAM and March C- is run again
// (it must pass). Line 1 of faults.txt is the fault-free memory (must pass).
`timescale 1ns/1ps
module tb_mbist;
  localparam AW = 9, DW = 32, SPARES = 2;
  reg clk = 0, rst_n = 0, start = 0, multi_bg = 0;
  reg [1:0] alg = 0;
  wire ce, we, busy, done, repairable;
  wire [AW-1:0] addr, first_fail_addr;
  wire [DW-1:0] wdata, rdata;
  wire [19:0] fail_cnt;
  wire [SPARES-1:0] rep_valid;
  wire [SPARES*AW-1:0] rep_addr;

  always #5 clk = ~clk;

  sram_model #(.AW(AW), .DW(DW), .SPARES(SPARES)) mem (
    .clk(clk), .ce(ce), .we(we), .addr(addr), .wdata(wdata), .rdata(rdata));
  mbist_ctrl #(.AW(AW), .DW(DW), .SPARES(SPARES)) bist (
    .clk(clk), .rst_n(rst_n), .start(start), .alg(alg), .multi_bg(multi_bg),
    .ce(ce), .we(we), .addr(addr), .wdata(wdata), .rdata(rdata),
    .busy(busy), .done(done), .fail_cnt(fail_cnt), .first_fail_addr(first_fail_addr),
    .rep_valid(rep_valid), .rep_addr(rep_addr), .repairable(repairable));

  integer cycles;
  task run(input [1:0] a, input mb);
    begin
      alg = a; multi_bg = mb;
      @(negedge clk) start = 1;
      @(negedge clk) start = 0;
      cycles = 0;
      while (!done) begin @(negedge clk); cycles = cycles + 1; end
    end
  endtask

  integer fd, r, id, ftype, fa, fb, fa2, fb2, fv, fdir, fs, a, mb, k, w;
  reg [8*16-1:0] cls;
  reg [5:0] det;
  reg rep_ok, rep_pass;
  integer f5, ff5;
  integer c_cyc [0:5];
  initial begin
    fd = $fopen("faults.txt", "r");
    repeat (3) @(negedge clk);
    rst_n = 1;
    while (!$feof(fd)) begin
      r = $fscanf(fd, "%d %s %d %d %d %d %d %d %d %d\n", id, cls, ftype, fa, fb, fa2, fb2, fv, fdir, fs);
      if (r != 10) begin
        if (r > 0) $display("bad line in faults.txt (%0d fields)", r);
      end else begin
        for (a = 0; a < 3; a = a + 1)
          for (mb = 0; mb < 2; mb = mb + 1) begin
            mem.clear_repair;
            for (w = 0; w < (1 << AW); w = w + 1) mem.mem[w] = {DW{1'bx}};   // power-up: unknown
            mem.ftype = ftype; mem.fa = fa; mem.fb = fb; mem.fa2 = fa2; mem.fb2 = fb2;
            mem.fv = fv; mem.fdir = fdir; mem.fs = fs;
            run(a, mb);
            det[a * 2 + mb] = (fail_cnt != 0);
            c_cyc[a * 2 + mb] = cycles;
          end
        f5 = fail_cnt; ff5 = first_fail_addr;          // March C-, all backgrounds (the last run)
        // repair with the BIRA result of March C- / all backgrounds, then run it again
        rep_ok = repairable && det[5];
        rep_pass = 0;
        if (rep_ok) begin
          for (k = 0; k < SPARES; k = k + 1) begin
            mem.rep_en[k] = rep_valid[k];
            mem.rep_addr[k] = rep_addr[k*AW +: AW];
          end
          run(2, 1);
          rep_pass = (fail_cnt == 0);
        end
        $display("FAULT %0d %0s det MATS+ %b%b MarchX %b%b MarchC- %b%b fails %0d first_addr %0d repairable %b after_repair %s",
                 id, cls, det[0], det[1], det[2], det[3], det[4], det[5], f5, ff5,
                 rep_ok, rep_ok ? (rep_pass ? "PASS" : "FAIL") : "-");
      end
    end
    $display("CYCLES MATS+ %0d/%0d MarchX %0d/%0d MarchC- %0d/%0d (solid / all backgrounds)",
             c_cyc[0], c_cyc[1], c_cyc[2], c_cyc[3], c_cyc[4], c_cyc[5]);
    $display("MBIST EXPERIMENT DONE");
    $finish;
  end
endmodule
