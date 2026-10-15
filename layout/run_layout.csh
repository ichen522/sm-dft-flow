source ~/setup_ee577b_v26.csh >& /dev/null
cd ~/658dft/sm_dft
xvfb-run -a -s "-screen 0 1600x1200x24" innovus -win -overwrite -log layout/innovus_layout -files layout/dump_layout.tcl < /dev/null
