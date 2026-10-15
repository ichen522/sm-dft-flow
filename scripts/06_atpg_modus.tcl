#####################################################################
# 06_atpg_modus.tcl  -  Cadence Modus ATPG on the post-route netlist
#
# Use this one if 00_check_env.csh found  modus  (Cadence).
# If it found  tmax  (Synopsys) instead, use 06_atpg_tmax.tcl.
#
# Usage (from ~/sm_dft, setup_ee577b_v26.csh or the setup that has Modus):
#   mkdir -p atpg/modus
#   cd atpg/modus
#   modus -file ../../scripts/06_atpg_modus.tcl |& tee modus_run.log
#   cd ../..
#
# Pin roles come from scan/modus.pinassign (written by scan_insert.py):
#   -ES clocks (off = 0), resets held at their inactive value (+SC / -SC),
#   +SE scan_en, SIk / SOk the 16 chains.
#####################################################################

set PDK /tools/PDK/NCSU45PDK/FreePDK45/osu_soc/lib/files
set TOP ../..
set NET [expr {[info exists ::env(NETLIST)] ? $::env(NETLIST) : "$TOP/pnr/sm_top/sm_top_pnr.v"}]

set_option stdout summary

build_model -cell sm_top -designsource $NET -techlib $PDK/gscl45nm.v -designtop sm_top -allowmissingmodules no
build_testmode -testmode FULLSCAN -assignfile $TOP/scan/modus.pinassign -modedef FULLSCAN
report_test_structures -testmode FULLSCAN
verify_test_structures -testmode FULLSCAN

build_faultmodel -includedynamic yes

# chain integrity tests first, then stuck-at logic tests
create_scanchain_tests -testmode FULLSCAN -experiment scan
commit_tests           -testmode FULLSCAN -inexperiment scan
create_logic_tests     -testmode FULLSCAN -experiment logic
commit_tests           -testmode FULLSCAN -inexperiment logic

report_fault_statistics -testmode FULLSCAN
write_vectors -testmode FULLSCAN -inexperiment logic -language verilog -scanformat serial

exit
