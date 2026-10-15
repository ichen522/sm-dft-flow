#!/usr/bin/env python3
"""
run_mbist.py  -  MBIST fault-injection experiment: fault list -> Xcelium -> table

Writes faults.txt (random locations, fixed seed), runs tb_mbist.v with
sram_model.v + mbist_ctrl.v, and prints per fault class the fraction of
injected faults each algorithm detects (solid background / log2(DW)+1
backgrounds), plus how many March C- detections the BIRA could repair with
2 spare words and whether March C- passes after the repair.

Usage (from ~/658dft/sm_dft/mbist, any shell):
  python3 run_mbist.py [--per-class 10] [--seed 1]
"""

import argparse
import os
import random
import re
import subprocess
from collections import defaultdict

AW, DW = 9, 32
SETUP = 'source ~/setup_ee577b_v26.csh >& /dev/null'

# class name -> (ftype, fv, fdir, fs, intra-word)
CLASSES = [
    ('SAF0', 1, 0, 0, 0, False), ('SAF1', 1, 1, 0, 0, False),
    ('TF_up', 2, 1, 0, 0, False), ('TF_down', 2, 0, 0, 0, False),
    ('CFin_up', 3, 0, 1, 0, False), ('CFin_down', 3, 0, 0, 0, False),
    ('CFid_up0', 4, 0, 1, 0, False), ('CFid_up1', 4, 1, 1, 0, False),
    ('CFid_dn0', 4, 0, 0, 0, False), ('CFid_dn1', 4, 1, 0, 0, False),
    ('CFst_00', 5, 0, 0, 0, False), ('CFst_01', 5, 1, 0, 0, False),
    ('CFst_10', 5, 0, 0, 1, False), ('CFst_11', 5, 1, 0, 1, False),
    ('AF', 6, 0, 0, 0, False),
    ('SAFx3', 7, 1, 0, 0, False),
    ('CFin_up_intra', 3, 0, 1, 0, True),
    ('CFid_up0_intra', 4, 0, 1, 0, True), ('CFid_up1_intra', 4, 1, 1, 0, True),
    ('CFid_dn0_intra', 4, 0, 0, 0, True), ('CFid_dn1_intra', 4, 1, 0, 0, True),
    ('CFst_01_intra', 5, 1, 0, 0, True), ('CFst_10_intra', 5, 0, 0, 1, True),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--per-class', type=int, default=10)
    ap.add_argument('--seed', type=int, default=1)
    args = ap.parse_args()
    here = os.path.dirname(os.path.abspath(__file__))
    rng = random.Random(args.seed)

    lines = ['0 NONE 0 0 0 0 0 0 0 0']
    n = 0
    for name, ftype, fv, fdir, fs, intra in CLASSES:
        for _ in range(args.per_class):
            n += 1
            fa, fb = rng.randrange(1 << AW), rng.randrange(DW)
            if ftype == 7:                      # three different words
                fa2, fb2 = rng.choice([a for a in range(1 << AW) if a not in (fa, (fa + 1) % (1 << AW))]), rng.randrange(DW)
            elif intra:
                fa2, fb2 = fa, rng.choice([b for b in range(DW) if b != fb])
            else:
                fa2, fb2 = rng.choice([a for a in range(1 << AW) if a != fa]), rng.randrange(DW)
            lines.append(f'{n} {name} {ftype} {fa} {fb} {fa2} {fb2} {fv} {fdir} {fs}')
    with open(os.path.join(here, 'faults.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')

    cmd = (f'{SETUP}; cd {here}; xrun -64bit -timescale 1ns/1ps -top tb_mbist '
           f'tb_mbist.v sram_model.v mbist_ctrl.v > sim.log')
    subprocess.run(['tcsh', '-c', cmd], check=False)
    log = open(os.path.join(here, 'sim.log'), errors='replace').read()
    if 'MBIST EXPERIMENT DONE' not in log:
        raise SystemExit('simulation did not finish, see sim.log')

    algs = ['MATS+', 'March X', 'March C-']
    stat = defaultdict(lambda: [0] * 6)
    cnt = defaultdict(int)
    rep = defaultdict(lambda: [0, 0])          # repairable, pass after repair
    for m in re.finditer(r'FAULT (\d+) (\S+) det MATS\+ (\d)(\d) MarchX (\d)(\d) MarchC- (\d)(\d) fails \d+ '
                         r'first_addr \d+ repairable (\d) after_repair\s+(\S+)', log):
        cls = m.group(2)
        cnt[cls] += 1
        for i in range(6):
            stat[cls][i] += int(m.group(3 + i))
        if m.group(9) == '1':
            rep[cls][0] += 1
            rep[cls][1] += m.group(10) == 'PASS'
    print(f'fault-free memory: {"PASS" if stat["NONE"] == [0] * 6 else "FAIL (false detection!)"} on all 6 runs')
    print(re.search(r'CYCLES .*', log).group(0))
    print()
    print(f'{"fault class":16} {"n":>3}  ' + '  '.join(f'{a:>15}' for a in algs) + '   repairable / pass after repair')
    print(f'{"":16} {"":>3}  ' + '  '.join(f'{"solid   all-bg":>15}' for a in algs))
    tot = [0] * 6
    for name, *_ in CLASSES:
        c, s = cnt[name], stat[name]
        tot = [t + x for t, x in zip(tot, s)]
        print(f'{name:16} {c:3}  ' + '  '.join(f'{100 * s[2 * i] / c:6.0f}% {100 * s[2 * i + 1] / c:6.0f}%' for i in range(3))
              + f'   {rep[name][0]:3} / {rep[name][1]:3}')
    N = sum(cnt[nm] for nm, *_ in CLASSES)
    print(f'{"all":16} {N:3}  ' + '  '.join(f'{100 * tot[2 * i] / N:6.1f}% {100 * tot[2 * i + 1] / N:6.1f}%' for i in range(3)))


if __name__ == '__main__':
    main()
