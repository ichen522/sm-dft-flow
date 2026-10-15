#####################################################################
# 07_pt.tcl  -  PrimeTime STA of the post-route sm_top (second-tool check)
#
# PrimeTime here can only read the library as .db (read_lib needs Library
# Compiler, which this server does not have), i.e. the ORIGINAL gscl45nm.db
# with the setup / recovery spikes that fix_lib.py removes for Innovus.
# Two runs, selected with  setenv PT_RUN spef | sdf :
#
#   spef : PrimeTime computes the delays itself from the SPEF and checks
#          against the original library -> shows what the spikes do in a
#          second, independent timer
#   sdf  : delays AND setup/hold limits come from Innovus' SDF (written with
#          the cleaned library) -> correlation with Innovus' sign-off numbers
#
# Usage (tcsh, setup_v1721.csh sourced, from ~/658dft/sm_dft):
#   setenv PT_MODE func   (or shift)
#   setenv PT_RUN sdf  ; pt_shell -f scripts/07_pt.tcl |& tee log/pt_sdf.log
#   setenv PT_RUN spef ; pt_shell -f scripts/07_pt.tcl |& tee log/pt_spef.log
#####################################################################

set run [expr {[info exists ::env(PT_RUN)] ? $::env(PT_RUN) : "sdf"}]
set PDK /tools/PDK/NCSU45PDK/FreePDK45/osu_soc/lib/files
set out report/pt_$run
file mkdir $out

set search_path [list . $PDK]
set link_path   [list * gscl45nm.db]
read_verilog pnr/sm_top/sm_top_pnr.v
current_design sm_top
link_design

set mode [expr {[info exists ::env(PT_MODE)] ? $::env(PT_MODE) : "func"}]
set out $out/$mode
file mkdir $out
foreach mode [list $mode] {
  if {$mode eq "func"} {
    source sdc/sm_top_func.sdc
    source scan/scan_func.sdc
  } else {
    source sdc/sm_top_shift.sdc
    source scan/scan_shift.sdc
  }
  set_propagated_clock [all_clocks]
  # post-CTS: jitter only, the real skew is in the propagated clocks (same as 04_pnr.tcl sign-off)
  set_clock_uncertainty -setup 0.05 [all_clocks]
  set_clock_uncertainty -hold  0.05 [all_clocks]

  if {$run eq "spef"} {
    read_parasitics pnr/sm_top/sm_top.spef
  } else {
    read_sdf -analysis_type single pnr/sm_top/sm_top_pnr.sdf
  }
  update_timing -full

  # a failing command inside "> file" only says "Errors detected during redirect";
  # on failure run it again without the redirect so the real message is in the log
  proc rpt {file cmd} {
    if {[catch {uplevel 1 "redirect $file {$cmd}"}]} {
      catch {uplevel 1 $cmd} err
      echo "RPT-FAIL $file : $err"
    }
  }
  rpt $out/global.rpt           {report_global_timing}
  rpt $out/setup.rpt            {report_timing -delay_type max -max_paths 10 -slack_lesser_than 1000 -nosplit}
  rpt $out/hold.rpt             {report_timing -delay_type min -max_paths 10 -slack_lesser_than 1000 -nosplit}
  rpt $out/annotated_checks.rpt {report_annotated_check}
  rpt $out/annotated_delays.rpt {report_annotated_delay}

  set ws [get_attribute [get_timing_paths -delay_type max] slack]
  set wh [get_attribute [get_timing_paths -delay_type min] slack]
  set nvs [sizeof_collection [get_timing_paths -delay_type max -slack_lesser_than 0 -max_paths 100000]]
  set nvh [sizeof_collection [get_timing_paths -delay_type min -slack_lesser_than 0 -max_paths 100000]]
  echo "PT-RESULT run=$run mode=$mode setup_wns=$ws setup_violating_endpoints=$nvs hold_wns=$wh hold_violating_endpoints=$nvh"
}
exit
