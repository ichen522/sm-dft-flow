#####################################################################
# 01_syn.tcl  -  Design Compiler synthesis of one SM (sm_top) in gscl45nm
#
# Based on the tensor_core script that already ran on viterbi-scf01
# (syn_tensor_core.tcl): same library setup, same max_capacitance fix.
#
# Usage (tcsh, in the tab where you did  source ~/setup_v1721.csh):
#   cd ~/sm_dft
#   dc_shell -f scripts/01_syn.tcl |& tee log/dc.log
#
# Knobs (setenv before running, all optional):
#   AW           BRAM address width, default 4 (16 entries; RTL default 9 = 512)
#   TILE_PERIOD  ns, default 2.2  -> P&R target is 2.5 ns (400 MHz). The 0.3 ns
#                margin pays for the scan mux + inverter that scan_insert.py puts
#                in front of every flop, plus the wires DC cannot see.
#   MEM_PERIOD   ns, default 4.5  -> P&R target 5.0 ns (200 MHz)
#
# Output:
#   netlist/sm_top_syn.v     flat gate-level netlist (input of scan_insert.py)
#   netlist/sm_top_syn.sdc   constraints as DC saw them (reference only)
#   report/sm_top.*          qor, timing, area (per hierarchy), power, flops
#####################################################################

set design sm_top
proc envdef {name def} { expr {[info exists ::env($name)] ? $::env($name) : $def} }
set aw          [envdef AW 4]
set tile_period [envdef TILE_PERIOD 2.2]
set mem_period  [envdef MEM_PERIOD 4.5]
set PDK /tools/PDK/NCSU45PDK/FreePDK45/osu_soc/lib/files
file mkdir report netlist log
catch {set_host_options -max_cores 8}   ;# needs a multicore license; ignored if there is none

echo "=== sm_top: AW=$aw  tile_clk ${tile_period} ns  clk_mem ${mem_period} ns"

#--------------------------------------------------------------------
# 1. Library (on the server, nothing to upload)
#--------------------------------------------------------------------
set search_path       [list . $PDK]
set target_library    gscl45nm.db
set synthetic_library dw_foundation.sldb
set link_library      [list * gscl45nm.db dw_foundation.sldb]

#--------------------------------------------------------------------
# 2. RTL: the Verilog-2001 (.v) copies of the SM hierarchy.
#    sm_top -> en_gen, tog_cdc x2, stat_cdc, sm_core -> tensor_core x4 -> mac_cell x16
#    Expected warnings: initial blocks ignored (BRAM power-on contents),
#    reg initial values ignored, (* keep *) / (* ASYNC_REG *) not recognised.
#    All Vivado-only, none changes the logic.
#--------------------------------------------------------------------
set rtl_files [list rtl/mac_cell.v rtl/tensor_core.v rtl/tog_cdc.v rtl/stat_cdc.v \
                    rtl/en_gen.v rtl/sm_core.v rtl/sm_top.v]
# DC_SYN skips one BRAM-init initial block in sm_core.v that crashes Presto (VER-37)
analyze -format sverilog -define DC_SYN $rtl_files
elaborate $design -parameters "AW=$aw"
# elaborate with -parameters names the design sm_top_AW4; every later step expects sm_top
set elab_name [get_object_name [current_design]]
if {$elab_name ne $design} { rename_design $elab_name $design }
current_design $design
link
check_design > report/${design}.check_design

# gscl45nm characterises a few output pins with max_capacitance = 0, which turns
# every net they drive into a fake DRV. Drop those values in memory only
# (the library file is untouched); max_transition is still checked.
set bad_pins [get_lib_pins -quiet gscl45nm/*/* -filter "max_capacitance == 0"]
echo "Info: removing max_capacitance = 0 from [sizeof_collection $bad_pins] library pins"
if {[sizeof_collection $bad_pins] > 0} { remove_attribute $bad_pins max_capacitance }

#--------------------------------------------------------------------
# 3. Constraints (same structure as sdc/sm_top_func.sdc, tighter clocks)
#--------------------------------------------------------------------
create_clock -name tile_clk -period $tile_period [get_ports tile_clk]
create_clock -name clk_mem  -period $mem_period  [get_ports clk_mem]
# The two clocks come from different MMCMs/PLLs: every crossing goes through
# tog_cdc / stat_cdc or is quasi-static, so the clocks are asynchronous.
set_clock_groups -asynchronous -group [get_clocks tile_clk] -group [get_clocks clk_mem]
set_clock_uncertainty 0.1 [all_clocks]

# host side (clk_mem): BRAM write port, result read address, reset
set mem_in  [get_ports {wr_stb_a wr_stb_b wr_addr* wr_data* res_raddr* rst_n_mem}]
set mem_out [get_ports {res_rdata* stat_data* stat_ack_tog clr_ack_tog os_ack_tog}]
set_input_delay  [expr {0.2 * $mem_period}] -clock clk_mem $mem_in
set_output_delay [expr {0.2 * $mem_period}] -clock clk_mem $mem_out

# asynchronous or quasi-static: synchronised inside the SM, or held stable
# for many cycles before they are sampled (see en_gen.v / sm_core.v headers)
set_false_path -from [get_ports {tile_rst_req tile_en_in seq_run_in win_toggle_in duty_off_in* cfg_k* clr_req_tog os_req_tog stat_req_tog}]
set_false_path -to   [get_ports {nk_hold* led_blk}]

set in_ports [remove_from_collection [all_inputs] [get_ports {tile_clk clk_mem}]]
set_driving_cell -lib_cell INVX4 $in_ports
set_load 0.01 [all_outputs]

check_timing > report/${design}.check_timing

#--------------------------------------------------------------------
# 4. Compile (keep the hierarchy for the area report, flatten afterwards)
#--------------------------------------------------------------------
set_fix_multiple_port_nets -all -buffer_constants [get_designs *]
compile_ultra -no_autoungroup

#--------------------------------------------------------------------
# 5. Reports
#--------------------------------------------------------------------
report_qor                                   > report/${design}.qor
report_timing -max_paths 5 -group tile_clk   > report/${design}.timing_tile
report_timing -max_paths 5 -group clk_mem    > report/${design}.timing_mem
report_area -hierarchy                       > report/${design}.area
report_power                                 > report/${design}.power
report_constraint -all_violators             > report/${design}.violators
report_clock                                 > report/${design}.clocks

set n_tile [sizeof_collection [all_registers -clock tile_clk -edge_triggered]]
set n_mem  [sizeof_collection [all_registers -clock clk_mem  -edge_triggered]]
set n_all  [sizeof_collection [all_registers -edge_triggered]]
set fh [open report/${design}.flops w]
puts $fh "flops total : $n_all"
puts $fh "  tile_clk  : $n_tile"
puts $fh "  clk_mem   : $n_mem"
close $fh
echo "FLOPS: total $n_all  (tile_clk $n_tile, clk_mem $n_mem)"

#--------------------------------------------------------------------
# 6. Flat netlist for scan_insert.py / Innovus
#--------------------------------------------------------------------
ungroup -all -flatten
change_names -rules verilog -hierarchy
report_reference                             > report/${design}.ref
write -format verilog -hierarchy -output netlist/${design}_syn.v
write_sdc                                    netlist/${design}_syn.sdc
write -format ddc -hierarchy     -output netlist/${design}.ddc

echo "=== DONE: netlist/${design}_syn.v"
exit
