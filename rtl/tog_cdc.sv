// tog_cdc.sv  (110_3)
//
// Request / acknowledge handshake from clk_mem (host_if.sv) into one tile's
// clock domain, for the TILE_CFG (clear) and RUN_ONE commands.
//
//   src (clk_mem)                       dst (tile clock)
//   req_tog flips once per request ---> 2-flop sync -> edge -> `pulse` (1 cycle)
//   ack_tog <--- 2-flop sync <---------- done_tog flips when the work is done
//
// host_if.sv treats a request as finished when ack_tog == req_tog again.
// Toggles (not pulses) so the two clocks can have any ratio. The dst side
// has no reset on purpose: rp tracks rs[1], so a reset can never create a
// false edge; the tile itself is in reset whenever host_if.sv is.
// sva/tog_cdc_sva.sv checks the protocol (one request outstanding at a
// time, `pulse` is one cycle per request).

module tog_cdc (
  input  logic src_clk,
  input  logic src_rst_n,
  input  logic req_tog,
  output logic ack_tog,

  input  logic dst_clk,
  output logic pulse,
  input  logic done_tog
);

  (* ASYNC_REG = "TRUE" *) logic [1:0] rs = 2'b00;
  logic rp = 1'b0;
  always_ff @(posedge dst_clk) begin
    rs <= {rs[0], req_tog};
    rp <= rs[1];
  end
  assign pulse = rs[1] ^ rp;

  (* ASYNC_REG = "TRUE" *) logic [1:0] as = 2'b00;
  always_ff @(posedge src_clk or negedge src_rst_n)
    if (!src_rst_n) as <= 2'b00;
    else            as <= {as[0], done_tog};
  assign ack_tog = as[1];

endmodule
