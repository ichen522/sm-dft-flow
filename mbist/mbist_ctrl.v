// mbist_ctrl.v  -  March-test MBIST controller with built-in redundancy analysis
//
// Algorithms (alg):
//   0 MATS+    {up(w0); up(r0,w1); dn(r1,w0)}                                   5n
//   1 March X  {up(w0); up(r0,w1); dn(r1,w0); up(r0)}                           6n
//   2 March C- {up(w0); up(r0,w1); up(r1,w0); dn(r0,w1); dn(r1,w0); up(r0)}    10n
// ("up" for the order-independent elements of the literature.)
// "0" / "1" are the data background and its inverse. multi_bg = 0: solid
// background only (all 0). multi_bg = 1: log2(DW)+1 backgrounds
// (0000.., 0101.., 0011.., 00001111.., ...), needed for intra-word coupling
// faults in a word-oriented memory.
//
// Every read is compared one cycle later (synchronous SRAM). On a miscompare:
// fail_cnt counts failing reads, and the BIRA keeps up to SPARES distinct
// failing addresses (rep_addr / rep_valid). repairable = 0 once there are
// more distinct failing addresses than spare words.
module mbist_ctrl #(parameter AW = 9, DW = 32, SPARES = 2, BGW = 3) (
  input  wire          clk,
  input  wire          rst_n,
  input  wire          start,
  input  wire [1:0]    alg,
  input  wire          multi_bg,
  // memory port
  output reg           ce,
  output reg           we,
  output reg  [AW-1:0] addr,
  output reg  [DW-1:0] wdata,
  input  wire [DW-1:0] rdata,
  // status
  output reg           busy,
  output reg           done,
  output reg  [19:0]   fail_cnt,
  output reg  [AW-1:0] first_fail_addr,
  output reg  [SPARES-1:0]    rep_valid,
  output reg  [SPARES*AW-1:0] rep_addr,
  output reg           repairable
);
  localparam integer NBG = $clog2(DW) + 1;

  // ---- march element table: {dir, nops-1, op1, op0}, op = {write, value}
  function [5:0] element(input [1:0] a, input [2:0] e);
    begin
      element = 6'b0;
      case (a)
        2'd0: case (e)                                   // MATS+
          3'd0: element = {1'b0, 1'b0, 2'b00, 2'b10};    // up   (w0)
          3'd1: element = {1'b0, 1'b1, 2'b11, 2'b00};    // up   (r0, w1)
          3'd2: element = {1'b1, 1'b1, 2'b10, 2'b01};    // down (r1, w0)
        endcase
        2'd1: case (e)                                   // March X
          3'd0: element = {1'b0, 1'b0, 2'b00, 2'b10};
          3'd1: element = {1'b0, 1'b1, 2'b11, 2'b00};
          3'd2: element = {1'b1, 1'b1, 2'b10, 2'b01};
          3'd3: element = {1'b0, 1'b0, 2'b00, 2'b00};    // up   (r0)
        endcase
        default: case (e)                                // March C-
          3'd0: element = {1'b0, 1'b0, 2'b00, 2'b10};    // up   (w0)
          3'd1: element = {1'b0, 1'b1, 2'b11, 2'b00};    // up   (r0, w1)
          3'd2: element = {1'b0, 1'b1, 2'b10, 2'b01};    // up   (r1, w0)
          3'd3: element = {1'b1, 1'b1, 2'b11, 2'b00};    // down (r0, w1)
          3'd4: element = {1'b1, 1'b1, 2'b10, 2'b01};    // down (r1, w0)
          3'd5: element = {1'b0, 1'b0, 2'b00, 2'b00};    // up   (r0)
        endcase
      endcase
    end
  endfunction

  function [2:0] n_elements(input [1:0] a);
    n_elements = (a == 2'd0) ? 3'd3 : (a == 2'd1) ? 3'd4 : 3'd6;
  endfunction

  // data background k: bit i = bit (k-1) of i  (k = 0: all zeros)
  function [DW-1:0] background(input [BGW-1:0] k);
    integer i;
    begin
      for (i = 0; i < DW; i = i + 1)
        background[i] = (k == 0) ? 1'b0 : ((i >> (k - 1)) & 1);
    end
  endfunction

  reg [BGW-1:0] bg;
  reg [2:0]     el;
  reg           op;
  reg [AW-1:0]  a_cnt;
  // read compare pipeline: request issued (c1) -> SRAM samples it (c2) -> data valid, compare
  reg           c1_v, c2_v;
  reg [DW-1:0]  c1_exp, c2_exp;
  reg [AW-1:0]  c1_addr, c2_addr;
  reg [1:0]     drain;

  wire [5:0]    cur  = element(alg, el);
  wire          dir  = cur[5];
  wire          last_op = (op == cur[4]);
  wire [1:0]    opc  = op ? cur[3:2] : cur[1:0];
  wire [DW-1:0] bgv  = background(bg);
  wire [AW-1:0] a_phys = dir ? ~a_cnt : a_cnt;
  wire          last_addr = &a_cnt;
  wire          last_el = (el == n_elements(alg) - 1);
  wire          last_bg = !multi_bg || (bg == NBG - 1);

  // BIRA: is the failing address already recorded? is there a free spare?
  integer k;
  reg hit, room;
  reg [$clog2(SPARES+1)-1:0] free;
  always @* begin
    hit = 1'b0; room = 1'b0; free = 0;
    for (k = SPARES - 1; k >= 0; k = k - 1) begin
      if (rep_valid[k] && rep_addr[k*AW +: AW] == c2_addr) hit = 1'b1;
      if (!rep_valid[k]) begin room = 1'b1; free = k; end
    end
  end

  always @(posedge clk or negedge rst_n)
    if (!rst_n) begin
      busy <= 1'b0; done <= 1'b0; ce <= 1'b0; we <= 1'b0; addr <= 0; wdata <= 0;
      bg <= 0; el <= 0; op <= 1'b0; a_cnt <= 0; drain <= 0;
      c1_v <= 1'b0; c2_v <= 1'b0; c1_exp <= 0; c2_exp <= 0; c1_addr <= 0; c2_addr <= 0;
      fail_cnt <= 0; first_fail_addr <= 0; rep_valid <= 0; rep_addr <= 0; repairable <= 1'b1;
    end else begin
      // compare: rdata now holds the word read for the request in c2
      if (c2_v && rdata !== c2_exp) begin
        if (fail_cnt == 0) first_fail_addr <= c2_addr;
        fail_cnt <= fail_cnt + 1'b1;
        if (!hit) begin
          if (room) begin
            rep_valid[free] <= 1'b1;
            rep_addr[free*AW +: AW] <= c2_addr;
          end else
            repairable <= 1'b0;
        end
      end
      c2_v <= c1_v; c2_exp <= c1_exp; c2_addr <= c1_addr;
      c1_v <= 1'b0;
      if (start && !busy) begin
        busy <= 1'b1; done <= 1'b0; drain <= 0;
        bg <= 0; el <= 0; op <= 1'b0; a_cnt <= 0;
        fail_cnt <= 0; rep_valid <= 0; rep_addr <= 0; repairable <= 1'b1;
        ce <= 1'b0; we <= 1'b0;
      end else if (busy) begin
        // issue one operation per cycle
        ce    <= 1'b1;
        we    <= opc[1];
        addr  <= a_phys;
        wdata <= opc[0] ? ~bgv : bgv;
        if (!opc[1]) begin
          c1_v    <= 1'b1;
          c1_exp  <= opc[0] ? ~bgv : bgv;
          c1_addr <= a_phys;
        end
        // advance op -> address -> element -> background
        if (!last_op) op <= 1'b1;
        else begin
          op <= 1'b0;
          if (!last_addr) a_cnt <= a_cnt + 1'b1;
          else begin
            a_cnt <= 0;
            if (!last_el) el <= el + 1'b1;
            else begin
              el <= 0;
              if (!last_bg) bg <= bg + 1'b1;
              else begin busy <= 1'b0; drain <= 2'd3; end
            end
          end
        end
      end else begin
        ce <= 1'b0; we <= 1'b0;
        if (drain != 0) begin               // let the last reads reach the comparator
          drain <= drain - 1'b1;
          if (drain == 1) done <= 1'b1;
        end
      end
    end
endmodule
