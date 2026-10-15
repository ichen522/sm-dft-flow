# sm_top_func.sdc  -  functional mode (mission mode) for P&R / sign-off
#   tile_clk 385 MHz (2.6 ns; target was 400 MHz, see below), clk_mem 200 MHz (host side)
#   used together with scan/scan_func.sdc (scan_en = 0, scan ports false-pathed)

# Target was 2.5 ns (400 MHz). After post-route ECO (scripts/04b_postroute_eco.tcl, 3 rounds)
# the MAC multiplier / 48-bit accumulator paths still missed by 38 ps (5 paths),
# so sign-off is at 2.6 ns (385 MHz). 400 MHz results: pnr/sm_top/eco3_400MHz/.
create_clock -name tile_clk -period 2.6 [get_ports tile_clk]
create_clock -name clk_mem  -period 5.0 [get_ports clk_mem]
set_clock_groups -asynchronous -group [get_clocks tile_clk] -group [get_clocks clk_mem]
set_clock_uncertainty 0.1 [get_clocks tile_clk]
set_clock_uncertainty 0.1 [get_clocks clk_mem]

# host side (clk_mem)
set_input_delay  1.0 -clock clk_mem [get_ports {wr_stb_a wr_stb_b wr_addr[*] wr_data[*] res_raddr[*] rst_n_mem}]
set_output_delay 1.0 -clock clk_mem [get_ports {res_rdata[*] stat_data[*] stat_ack_tog clr_ack_tog os_ack_tog}]

# asynchronous / quasi-static ports (synchronised inside the SM)
set_false_path -from [get_ports {tile_rst_req tile_en_in seq_run_in win_toggle_in duty_off_in[*] cfg_k[*] clr_req_tog os_req_tog stat_req_tog}]
set_false_path -to   [get_ports {nk_hold[*] led_blk}]

set_driving_cell -lib_cell INVX4 [all_inputs]
set_load 0.01 [all_outputs]
