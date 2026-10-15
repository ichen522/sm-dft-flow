#####################################################################
# 04c_resignoff.tcl  -  sign-off timing again on the finished layout after an
# SDC change (no change to placement / routing / netlist / SDF)
#
# Used once: tile_clk 2.5 -> 2.6 ns in sdc/sm_top_func.sdc after the
# post-route ECO left 5 paths at -0.038 ns (see the comment in that file).
# Usage (v26 tab, from ~/658dft/sm_dft):
#   innovus -no_gui -overwrite -log pnr/sm_top/innovus_resignoff -files scripts/04c_resignoff.tcl
#####################################################################
set design sm_top
set out    pnr/$design
proc step {name body} {
  Puts "\n==================== STEP: $name ===================="
  if {[catch {uplevel 1 $body} err]} {
    Puts "STEP-FAIL: $name : $err"
    return 0
  }
  Puts "STEP-OK: $name"
  return 1
}
setMultiCpuUsage -localCpu 8
source $out/06_signoff.enc
step "update cm_func" { update_constraint_mode -name cm_func -sdc_files {sdc/sm_top_func.sdc scan/scan_func.sdc} }
setExtractRCMode -engine postRoute
setAnalysisMode -analysisType onChipVariation -cppr both
# update_constraint_mode re-reads the SDC, and the SDC has ideal clocks: without this the
# first re-sign-off reported +0.050 ns with ideal clocks (Other End Arrival Time 0.000)
foreach mode {cm_func cm_shift} {
  set_interactive_constraint_modes $mode
  set_propagated_clock [all_clocks]
  set_clock_uncertainty -setup 0.05 [all_clocks]
  set_clock_uncertainty -hold  0.05 [all_clocks]
}
set_interactive_constraint_modes {}
step "timeDesign signoff setup" { timeDesign -postRoute       -outDir $out/rpt/4_signoff }
step "timeDesign signoff hold"  { timeDesign -postRoute -hold -outDir $out/rpt/4_signoff }
step "check propagated" { report_clocks > $out/rpt/4_signoff/clocks.rpt }
step "report_timing func"  { report_timing -view av_func  -max_paths 10 > $out/rpt/4_signoff/func_setup_paths.rpt }
step "report_timing shift" { report_timing -view av_shift -max_paths 10 > $out/rpt/4_signoff/shift_setup_paths.rpt }
step "report_clocks" { report_clocks > $out/rpt/4_signoff/clocks.rpt }
saveDesign $out/06_signoff.enc
Puts "\n==================== RESIGNOFF DONE ===================="
exit
