# syn_mbist.tcl - area / timing of the MBIST controller in gscl45nm (dc_shell -f mbist/syn_mbist.tcl, from mbist/)
set PDK /tools/PDK/NCSU45PDK/FreePDK45/osu_soc/lib/files
set search_path       [list . $PDK]
set target_library    gscl45nm.db
set link_library      [list * gscl45nm.db dw_foundation.sldb]
analyze -format verilog mbist_ctrl.v
elaborate mbist_ctrl -parameters "AW=9,DW=32,SPARES=2"
set d [get_object_name [current_design]]
current_design $d
link
create_clock -name clk -period 2.5 [get_ports clk]
set_input_delay 0.5 -clock clk [remove_from_collection [all_inputs] [get_ports clk]]
set_output_delay 0.5 -clock clk [all_outputs]
compile_ultra
report_area > mbist_area.rpt
report_timing > mbist_timing.rpt
report_reference > mbist_ref.rpt
echo "MBIST-SYN area [get_attribute [current_design] area] cells [sizeof_collection [get_cells -hier *]] flops [sizeof_collection [all_registers]]"
exit
