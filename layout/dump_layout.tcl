# dump_layout.tcl - layout screenshots of the signed-off sm_top (run under xvfb-run, innovus -win)
source pnr/sm_top/06_signoff.enc
fit
setDrawView place
dumpToGIF layout/sm_top_routed.gif

# cells only: hide every routing layer and the power grid
setLayerPreference allM -isVisible 0
setLayerPreference allM -isSelectable 0
setLayerPreference power -isVisible 0
setLayerPreference pwrdm -isVisible 0
setLayerPreference stdCell -isVisible 1
dumpToGIF layout/sm_top_cells.gif

# where the blocks ended up (netlist is flat: group the instances by name prefix)
set groups {
  1 u_core_SUBP_0__*
  2 u_core_SUBP_1__*
  3 u_core_SUBP_2__*
  4 u_core_SUBP_3__*
  5 u_core_bmem_*
  6 u_core_RES_*
}
foreach {idx pat} $groups {
  set insts [dbGet -p -e top.insts.name $pat]
  Puts "LAYOUT-GROUP $idx $pat : [llength $insts] instances"
  if {[llength $insts]} { highlight $insts -index $idx }
}
dumpToGIF layout/sm_top_blocks.gif
exit
