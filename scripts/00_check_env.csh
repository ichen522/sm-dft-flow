#!/bin/tcsh -f
#####################################################################
# 00_check_env.csh  -  check what this server has for the DFT flow
#
# Usage (from ~/sm_dft, any shell):
#   tcsh scripts/00_check_env.csh |& tee env_check.txt
#
# Read-only: it only lists files and looks up commands, it changes nothing.
# Paste env_check.txt back to Claude; sections 4 and 5 decide which ATPG
# script you run in Step 6.
#####################################################################
set nonomatch
set PDK = /tools/PDK/NCSU45PDK/FreePDK45/osu_soc/lib/files

echo "===== 1. Standard-cell library (gscl45nm)"
ls -l $PDK/gscl45nm.lib $PDK/gscl45nm.db $PDK/gscl45nm.lef $PDK/gscl45nm.v
echo "scan flops (test_cell) in gscl45nm.lib : `grep -c test_cell $PDK/gscl45nm.lib`   (0 = no scan flop, scan_insert.py builds them)"
echo "MUX2X1 / INVX1 / BUFX2 present        : `grep -cE 'cell *\((MUX2X1|INVX1|BUFX2)\)' $PDK/gscl45nm.lib` (need 3)"
echo "flop cells                            : `grep -oE 'cell *\((DFF[A-Z0-9]*)\)' $PDK/gscl45nm.lib | tr '\n' ' '`"

echo ""
echo "===== 2. python3 (scan_insert.py needs >= 3.6, no packages)"
which python3
python3 --version

echo ""
echo "===== 3. Tools after 'source ~/setup_ee577b_v26.csh'"
if (-e ~/setup_ee577b_v26.csh) then
  source ~/setup_ee577b_v26.csh >& /dev/null
else
  echo "  ~/setup_ee577b_v26.csh not found"
endif
foreach t (innovus genus lec xrun irun ncsim modus tmax tmax2 pt_shell)
  set p = `which $t |& grep '^/'`
  if ("$p" == "") then
    echo "  $t : -"
  else
    echo "  $t : $p"
  endif
end

echo ""
echo "===== 4. Design Compiler after 'source ~/setup_v1721.csh' (the one that has a license)"
if (-e ~/setup_v1721.csh) then
  tcsh -f -c 'source ~/setup_v1721.csh >& /dev/null; which dc_shell; which tmax; which tmax2; which pt_shell'
else
  echo "  ~/setup_v1721.csh not found (upload it from Downloads\Ian, see README)"
endif

echo ""
echo "===== 5. ATPG tools installed anywhere (TetraMAX / TestMAX / Modus)"
ls -d /tools/cadence/*MODUS* /tools/cadence/*/MODUS* /tools/cadence/*/bin/modus \
      /tools/synopsys/*txs* /tools/synopsys/*tmax* /tools/synopsys/*testmax* /tools/synopsys/*/bin/tmax \
      /usr/local/synopsys/*TetraMAX* /usr/local/synopsys/*tmax* /usr/local/synopsys/*txs* \
      /usr/local/synopsys/*/bin/tmax /usr/local/cadence/*MODUS* |& grep -v -i "no such file"
echo "--- /tools/cadence:"
ls /tools/cadence |& head -40
echo "--- /tools/synopsys:"
ls /tools/synopsys |& head -40
echo "--- /usr/local/synopsys:"
ls /usr/local/synopsys |& head -40

echo ""
echo "===== 6. Other 45 nm libraries with scan flops (optional)"
foreach f (`find /tools/PDK -maxdepth 7 -name '*.lib' |& grep '^/.*\.lib$' | head -40`)
  set n = `grep -c test_cell $f`
  if ("$n" != "0") echo "  $f : $n scan cells"
end

echo ""
echo "===== 7. Machine"
hostname
nproc
free -g | head -2
df -h ~ | tail -1
echo "===== done"
