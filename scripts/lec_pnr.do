// lec_pnr.do  -  Conformal LEC: scan netlist (golden) vs post-route netlist (revised)
//
// Checks that P&R (resizing, buffering, CTS, hold fixing) did not change the
// logic. Both designs have the scan chains, so nothing is constrained and
// scan_out is compared too.
//
// Usage (tcsh, setup_ee577b_v26.csh sourced, from ~/sm_dft):
//   lec -nogui -xl -dofile scripts/lec_pnr.do
set log file lec/lec_pnr.log -replace
read library -both -liberty /tools/PDK/NCSU45PDK/FreePDK45/osu_soc/lib/files/gscl45nm.lib
read design -verilog -golden  netlist/sm_top_scan.v
read design -verilog -revised pnr/sm_top/sm_top_pnr.v
set root module sm_top -both
set system mode lec
add compared points -all
compare
report verification
report statistics
report compare data -class nonequivalent
exit -force
