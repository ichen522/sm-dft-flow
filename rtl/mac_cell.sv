// mac_cell.sv  (110_3)
//
// One multiply-accumulate unit = one DSP48E1:
//
//     a_r, b_r  <= a, b                 (DSP A/B input registers)
//     m_r       <= a_r * b_r            (DSP M register, 16 x 16 -> 32 bit)
//     p_r       <= ld ? m_r : p_r + m_r (DSP P register, 48 bit)   if ce_p
//
// `ld` is the first k-step of a block (P = A*B, i.e. the old sum is
// dropped), every later step accumulates. ce_p = this cycle's product is a
// real k-step (the tile's enable / valid tag), so a stopped tile keeps P.
// This is the MAC the professor described on 10/3: "multiplier + adder,
// output fed back into the adder, zero it when a new set starts".
//
// Written so Vivado infers the whole thing into one DSP48E1 (AREG=BREG=1,
// MREG=1, PREG=1, dynamic OPMODE for load/accumulate) -- no fabric adder.
// With |a|,|b| <= 2^15 and K <= 1023 steps, |P| < 2^40: never overflows
// (checked by sva/mac_cell_sva.sv).

module mac_cell (
  input  logic               clk,
  input  logic signed [15:0] a,
  input  logic signed [15:0] b,
  input  logic               ce_p,
  input  logic               ld,
  output logic        [47:0] p
);

  logic signed [15:0] a_r = '0;
  logic signed [15:0] b_r = '0;
  logic signed [31:0] m_r = '0;
  logic signed [47:0] p_r = '0;

  logic signed [47:0] m_ext;
  assign m_ext = {{16{m_r[31]}}, m_r};   // sign extension

  always_ff @(posedge clk) begin
    a_r <= a;
    b_r <= b;
    m_r <= a_r * b_r;
    if (ce_p)
      p_r <= ld ? m_ext : p_r + m_ext;
  end

  assign p = p_r;

endmodule
