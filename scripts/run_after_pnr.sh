#!/bin/bash
#####################################################################
# run_after_pnr.sh  -  everything after Step 4, in order, with checks
#
#   5-1  LEC #2 (scan netlist vs post-route netlist)
#   5-2  chain flush test with post-route SDF
#   6    ATPG models (full + one per capture clock), model-vs-netlist check,
#        Atalanta per clock, independent coverage check (union),
#        undetected-fault analysis (AU / UR / aborted, by block)
#   7-1  STIL file of the patterns (read back and compared);
#        ATPG patterns through the scan chains: all patterns zero-delay
#        (8 parallel jobs) + the first 40 patterns of each clock with SDF
#        + two runs with an injected defect (logic, chain) for 7-4
#   7-2  transition-delay grading, LOC and LOS
#   7-3  PrimeTime: SDF correlation + SPEF with the original library
#   7-4  diagnosis of the two injected defects from their fail logs
#
# Usage (any shell, from ~/658dft/sm_dft; Atalanta built with the patch):
#   bash scripts/run_after_pnr.sh |& tee log/run_after_pnr.log
# Each step prints a "== RESULT" line; the script stops at the first failure.
# Runtime: about 1.5 hours (the serial simulation is most of it).
#####################################################################
set -u
cd "$(dirname "$0")/.."
TOP=$PWD
PDK=/tools/PDK/NCSU45PDK/FreePDK45/osu_soc/lib/files
ATALANTA=${ATALANTA:-$HOME/658dft/tools/atalanta_hsluoyz/atalanta}
CONST="--const scan_en=0 --const tile_rst_req=0 --const rst_n_mem=1"
NET=pnr/sm_top/sm_top_pnr.v
SDF=pnr/sm_top/sm_top_pnr.sdf
V26='source ~/setup_ee577b_v26.csh >& /dev/null'
V1721='source ~/setup_v1721.csh >& /dev/null'
mkdir -p log lec atpg

fail() { echo "== FAIL: $*"; exit 1; }
step() { echo; echo "==================== $* ($(date +%H:%M))"; }

#--------------------------------------------------------------------
step "5-1 LEC #2"
tcsh -c "$V26; cd $TOP; lec -nogui -xl -dofile scripts/lec_pnr.do" > log/lec_pnr.out 2>&1
grep -q "Compare Results:.*PASS" lec/lec_pnr.log || fail "LEC #2, see lec/lec_pnr.log"
echo "== RESULT LEC #2: $(grep -A3 'Compare Results' lec/lec_pnr.log | tr -s ' ' | tr '\n' ' ')"

#--------------------------------------------------------------------
step "5-2 SDF flush test"
cat > log/flush_sdf.csh <<EOF
$V26
cd $TOP
xrun -64bit -maxdelays -sdf_nopathedge -timescale 1ns/1ps +access+r -top tb_flush \\
     scan/tb_flush.v $NET -v $PDK/gscl45nm.v '+define+SDF="$SDF"'
EOF
tcsh -f log/flush_sdf.csh > log/flush_sdf.log 2>&1
grep -q "FLUSH TEST PASSED" log/flush_sdf.log || fail "SDF flush, see log/flush_sdf.log"
grep -q "Tchecks.*Annotated = 100.00%" log/flush_sdf.log || fail "SDF timing checks not fully annotated"
echo "== RESULT $(grep 'FLUSH TEST' log/flush_sdf.log); $(grep -o 'Pathdelays.*' log/flush_sdf.log | head -1)"

#--------------------------------------------------------------------
step "6 ATPG models"
python3 scripts/netlist2bench.py --netlist $NET --lib $PDK/gscl45nm.lib --top sm_top --out atpg/sm_top $CONST \
    > log/netlist2bench.log 2>&1 || fail "netlist2bench (full)"
for c in tile_clk clk_mem; do
  python3 scripts/netlist2bench.py --netlist $NET --lib $PDK/gscl45nm.lib --top sm_top --out atpg/sm_top_$c $CONST \
      --chains scan/chains.json --capture $c >> log/netlist2bench.log 2>&1 || fail "netlist2bench ($c)"
done
# same netlist with capture-mode constraints released (for the AU / UR split in analyze_undetected.py)
for spec in "free:" "rel_scan_en:--const tile_rst_req=0 --const rst_n_mem=1" "rel_resets:--const scan_en=0"; do
  python3 scripts/netlist2bench.py --netlist $NET --lib $PDK/gscl45nm.lib --top sm_top --out atpg/sm_top_${spec%%:*} \
      ${spec#*:} >> log/netlist2bench.log 2>&1 || fail "netlist2bench (${spec%%:*})"
done
for m in sm_top sm_top_tile_clk sm_top_clk_mem; do
  python3 scripts/verify_bench.py --netlist $NET --lib $PDK/gscl45nm.lib --top sm_top --bench atpg/$m $CONST \
      > log/verify_$m.log 2>&1
  grep -q "^PASS" log/verify_$m.log || fail "verify_bench $m, see log/verify_$m.log"
  echo "== RESULT model $m: $(tail -1 log/verify_$m.log)"
done

step "6 Atalanta (one run per capture clock + the all-flops model for the analysis, in parallel)"
for c in tile_clk clk_mem; do
  ( cd atpg && $ATALANTA -s 1 -t sm_top_$c.test -U sm_top_$c.aborted sm_top_$c.bench \
        > atalanta_$c.out 2> atalanta_$c.log ) &
done
rm -f atpg/sm_top.undet          # Atalanta does not overwrite it
( cd atpg && $ATALANTA -s 1 -v -U sm_top.undet -t sm_top.test sm_top.bench > atalanta_full.out 2> atalanta_full.log ) &
wait
for c in tile_clk clk_mem; do
  grep -q "Fault coverage" atpg/atalanta_$c.out || fail "Atalanta $c, see atpg/atalanta_$c.log"
  grep -aq "buffer full" atpg/atalanta_$c.log && fail "Atalanta $c: pattern buffer full"
  echo "== RESULT ATPG $c: $(grep -E 'after compaction|Fault coverage|collapsed|redundant|aborted' atpg/atalanta_$c.out | tr -s ' ' | tr '\n' ';')"
done

step "6 independent coverage check (union of both sets)"
python3 scripts/coverage_union.py --full atpg/sm_top --chains scan/chains.json \
    --set atpg/sm_top_tile_clk:tile_clk --set atpg/sm_top_clk_mem:clk_mem --samples 20000 --seed 1 \
    > log/coverage_union.log 2>&1 || fail "coverage_union"
grep -qE "bits, [1-9][0-9]* differ" log/coverage_union.log && fail "good-machine check, see log/coverage_union.log"
echo "== RESULT $(grep '^union' log/coverage_union.log)"

step "6 undetected-fault analysis (in the background, runs next to 7-1; about 20 minutes)"
grep -q "Fault coverage" atpg/atalanta_full.out || fail "Atalanta (all flops), see atpg/atalanta_full.log"
python3 scripts/analyze_undetected.py --model atpg/sm_top --undet atpg/sm_top.undet --atalanta-out atpg/atalanta_full.out \
    --atalanta $ATALANTA --release scan_en=atpg/sm_top_rel_scan_en --release resets=atpg/sm_top_rel_resets \
    --release all=atpg/sm_top_free --workdir atpg/undet --retry-backtrack 10000 --jobs 12 \
    > log/analyze_undetected.log 2>&1 &
ANALYZE_PID=$!

#--------------------------------------------------------------------
step "7-1 serial simulation of the ATPG patterns"
rm -rf atpg/serial
python3 scripts/gen_scan_tb.py --netlist $NET --chains scan/chains.json \
    --set atpg/sm_top_tile_clk:tile_clk --set atpg/sm_top_clk_mem:clk_mem $CONST --outdir atpg/serial \
    > log/gen_scan_tb.log 2>&1 || fail "gen_scan_tb"
python3 scripts/gen_stil.py --tb atpg/serial --chains scan/chains.json --out atpg/sm_top.stil --check \
    > log/gen_stil.log 2>&1 || fail "STIL, see log/gen_stil.log"
gzip -f atpg/sm_top.stil
echo "== RESULT STIL: $(grep -E 'read back' log/gen_stil.log) -> atpg/sm_top.stil.gz"
NP=$(grep -c '' atpg/serial/pi.mem)
NMEM0=$(sed -n 's/.*sm_top_clk_mem: patterns \([0-9]*\)\.\..*/\1/p' log/gen_scan_tb.log)
J=8; per=$(( (NP + J - 1) / J ))
jobs=()
for j in $(seq 0 $((J-1))); do
  a=$((j*per)); b=$(( (j+1)*per - 1 )); [ $b -ge $NP ] && b=$((NP-1))
  jobs+=("zd_$j:$a:$b:")
done
jobs+=("sdf_tile:0:39:sdf" "sdf_mem:$NMEM0:$((NMEM0+39)):sdf")
# defects for the diagnosis experiment (7-4): an internal multiplier net stuck-at-1, and the Q of the flop
# at position 600 of chain 3 stuck-at-0 (breaks the chain there and also feeds the multiplier)
LOGIC_NET=u_core_SUBP_2__u_tc_ROW_1__COL_5__u_mac_mult_x_1_n424
CHAIN_FLOP=$(python3 -c "import json; print(json.load(open('scan/chains.json'))['chains'][3]['flops'][600])")
CHAIN_NET=$(awk -v f="$CHAIN_FLOP" 'index($0, " " f " (") {on = 1} on && /\.Q\(/ {match($0, /\.Q\(([^)]*)\)/, m); print m[1]; exit}' $NET)
grep -q "wire $LOGIC_NET;" $NET || fail "diagnosis target $LOGIC_NET not in $NET"
[ -n "$CHAIN_NET" ] || fail "no Q net found for $CHAIN_FLOP"
jobs+=("diag_logic:0:99:inj:$LOGIC_NET=1" "diag_chain:0:19:inj:$CHAIN_NET=0")
sim_pids=()
for spec in "${jobs[@]}"; do
  IFS=: read d a b mode inj <<< "$spec"
  mkdir -p atpg/serial/$d
  for f in load.mem unload.mem unload_mask.mem pi.mem po.mem po_mask.mem cap.mem tb_atpg.v; do
    ln -sf ../$f atpg/serial/$d/$f
  done
  opt=""; [ "$mode" = sdf ] && opt="-maxdelays -sdf_nopathedge '+define+SDF=\"$TOP/$SDF\"'"
  [ "$mode" = inj ] && opt="'+define+INJECT_NET=${inj%=*}' '+define+INJECT_VAL=${inj##*=}'"
  cat > atpg/serial/$d/run.csh <<EOF
$V26
cd $TOP/atpg/serial/$d
xrun -64bit -timescale 1ns/1ps +access+r $opt -top tb_atpg tb_atpg.v $TOP/$NET \\
     -v $PDK/gscl45nm.v +FIRST=$a +LAST=$b +FAILLOG=fail.log
EOF
  tcsh -f atpg/serial/$d/run.csh > atpg/serial/$d/sim.log 2>&1 &
  sim_pids+=($!)
done
wait "${sim_pids[@]}"
for spec in "${jobs[@]}"; do
  d=${spec%%:*}
  r=$(grep -a "ATPG SERIAL SIM" atpg/serial/$d/sim.log)
  echo "== RESULT serial $d: ${r:-no result} (timing violations: $(grep -ac 'Timing violation' atpg/serial/$d/sim.log))"
  case $d in
    diag_*) echo "$r" | grep -q FAILED || fail "$d: the injected defect was not detected" ;;
    *)      echo "$r" | grep -q PASSED || fail "serial simulation $d" ;;
  esac
done

step "6 undetected-fault analysis: result"
wait $ANALYZE_PID || fail "analyze_undetected, see log/analyze_undetected.log"
grep -E "^  (DT|AU|UR|ND)|coverage|retried" log/analyze_undetected.log | sed 's/^ */== RESULT /'

#--------------------------------------------------------------------
step "7-2 transition grading (LOC and LOS)"
python3 scripts/transition_grade.py --full atpg/sm_top --chains scan/chains.json \
    --set atpg/sm_top_tile_clk:tile_clk --set atpg/sm_top_clk_mem:clk_mem --scheme loc,los \
    --random 4096 --samples 20000 --seed 1 --crosscheck 60 > log/transition_grade.log 2>&1 || fail "transition_grade"
awk '/crosscheck vs brute force/ { split($5, a, "/"); if (a[1] != a[2]) bad = 1 } END { exit bad }' \
    log/transition_grade.log || fail "transition crosscheck disagrees, see log/transition_grade.log"
grep '^union' log/transition_grade.log | sed 's/^/== RESULT /'

#--------------------------------------------------------------------
step "7-3 PrimeTime"
for r in sdf spef; do
  for m in func shift; do
    tcsh -c "$V1721; cd $TOP; setenv PT_RUN $r; setenv PT_MODE $m; pt_shell -f scripts/07_pt.tcl" \
        > log/pt_${r}_$m.log 2>&1 &
  done
done
wait
grep -h "^PT-RESULT" log/pt_*_*.log | sed 's/^/== RESULT /'

#--------------------------------------------------------------------
step "7-4 diagnosis of the injected defects (only pattern data + fail log + netlist model)"
python3 scripts/diagnose.py logic --tb atpg/serial --faillog atpg/serial/diag_logic/fail.log --model atpg/sm_top \
    --chains scan/chains.json --first 0 --last 99 --expect $LOGIC_NET=1 > log/diagnose_logic.log 2>&1 \
    || fail "diagnose logic, see log/diagnose_logic.log"
python3 scripts/diagnose.py chain --tb atpg/serial --faillog atpg/serial/diag_chain/fail.log --model atpg/sm_top \
    --chains scan/chains.json --first 0 --last 19 --expect $CHAIN_FLOP=0 > log/diagnose_chain.log 2>&1 \
    || fail "diagnose chain, see log/diagnose_chain.log"
grep -h "injected defect" log/diagnose_logic.log log/diagnose_chain.log | sed 's/^/== RESULT /'

echo; echo "==================== ALL STEPS DONE ($(date +%H:%M))"
