# sm_top_shift.sdc  -  scan shift mode for P&R / sign-off
#   The tester pulses both clocks at 100 MHz while scan_en = 1. Chains never
#   cross clock domains (scan_insert.py keeps one domain per chain), so the
#   clocks stay asynchronous here too.
#   used together with scan/scan_shift.sdc (scan_en = 1, resets held inactive,
#   scan_in / scan_out delays per chain)

create_clock -name tile_clk -period 10.0 [get_ports tile_clk]
create_clock -name clk_mem  -period 10.0 [get_ports clk_mem]
set_clock_groups -asynchronous -group [get_clocks tile_clk] -group [get_clocks clk_mem]
set_clock_uncertainty 0.2 [get_clocks tile_clk]
set_clock_uncertainty 0.2 [get_clocks clk_mem]

# functional I/O does not matter while shifting
set_false_path -from [get_ports {wr_stb_a wr_stb_b wr_addr[*] wr_data[*] res_raddr[*] tile_en_in seq_run_in win_toggle_in duty_off_in[*] cfg_k[*] clr_req_tog os_req_tog stat_req_tog}]
set_false_path -to   [get_ports {res_rdata[*] stat_data[*] stat_ack_tog clr_ack_tog os_ack_tog nk_hold[*] led_blk}]

set_driving_cell -lib_cell INVX4 [all_inputs]
set_load 0.01 [all_outputs]
