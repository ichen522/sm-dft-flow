#!/usr/bin/env python3
"""
analyze_undetected.py  -  where do the undetected stuck-at faults come from?

Input: an Atalanta run on the capture-mode model (netlist2bench.py with
--const ..., all flops observed) made with  -v -U <file>  so the file lists
the identified redundant faults first, then the aborted ones.

1. Every undetected fault is traced back to the cell instance it sits on
   (<model>.names) and grouped by block.
2. Redundant faults are split the way TetraMAX would:
     AU  ATPG untestable: only untestable because of the capture-mode
         constraints (scan_en = 0, resets inactive). Shown by targeted
         ATPG on models of the SAME netlist with constraints released
         (--release LABEL=PREFIX, one per released constraint set): a fault
         that some released model detects is AU, and the label says which
         constraint blocks it.
     UR  undetectable redundant: still redundant with every constraint
         released, i.e. redundant logic.
3. Aborted faults (ND) are retried on the capture-mode model with a larger
   backtrack limit (--retry-backtrack).

Fault classes and coverage (collapsed faults of the capture-mode model):
  fault coverage = DT / all
  test coverage  = DT / (all - UD)        UD = UR (TetraMAX: AU stays in the denominator)
Lines that are constant in capture mode (scan mux scan side, reset logic) are
folded away by netlist2bench.py and are not in this fault list at all
(TetraMAX: UD-tied / blocked); their gate count is reported separately.

Usage:
  python3 scripts/analyze_undetected.py --model atpg/sm_top --undet atpg/sm_top.undet \
      --atalanta-out atpg/atalanta_full.out --atalanta ~/658dft/tools/atalanta_hsluoyz/atalanta \
      --release scan_en=atpg/sm_top_rel_scan_en --release resets=atpg/sm_top_rel_resets \
      --release all=atpg/sm_top_free --workdir atpg/undet --retry-backtrack 10000 --jobs 12
"""

import argparse
import json
import os
import re
import subprocess
import sys
from collections import Counter, defaultdict


def read_names(prefix):
    """N-id -> (net name, instance, cell); and net name -> N-id"""
    by_id, by_name = {}, {}
    for ln in open(prefix + '.names'):
        nid, name, inst, cell = ln.split()
        by_id[nid] = (name, inst, cell)
        by_name[name] = nid
    return by_id, by_name


def read_bench_gates(path):
    gates = {}
    for ln in open(path):
        m = re.match(r'\s*(\S+)\s*=\s*(\w+)\((.*)\)', ln)
        if m:
            gates[m.group(1)] = (m.group(2), [x.strip() for x in m.group(3).split(',')])
    return gates


def parse_fault(ln):
    m = re.match(r'\s*(\S+?)(?:->(\S+))?\s+/([01])', ln)
    return (m.group(1), m.group(2), int(m.group(3))) if m else None


def atalanta_counts(out_file):
    txt = open(out_file, errors='replace').read()
    get = lambda k: int(re.search(k + r'\s*:\s*(\d+)', txt).group(1))
    return {'collapsed': get('Number of collapsed faults'),
            'redundant': get('Number of identified redundant faults'),
            'aborted': get('Number of aborted faults')}


def run_atalanta(exe, bench, faults, tag, workdir, extra=(), jobs=1):
    """targeted ATPG on `faults` (Atalanta fault-list syntax), split over `jobs` parallel runs;
    returns (redundant set, aborted set)"""
    faults = list(faults)
    jobs = max(1, min(jobs, len(faults)))
    procs = []
    for j in range(jobs):
        t = f'{tag}_{j}' if jobs > 1 else tag
        flt = os.path.join(workdir, t + '.flt')
        with open(flt, 'w') as f:
            f.write('\n'.join(faults[j::jobs]) + '\n')
        und = os.path.join(workdir, t + '.undet')
        if os.path.exists(und):
            os.remove(und)                      # Atalanta refuses to overwrite it
        out = os.path.join(workdir, t + '.out')
        cmd = [exe, '-s', '1', *extra, '-v', '-U', und, '-t', os.path.join(workdir, t + '.test'), '-f', flt, bench]
        fo, fe = open(out, 'w'), open(os.path.join(workdir, t + '.log'), 'w')
        procs.append((subprocess.Popen(cmd, stdout=fo, stderr=fe), fo, fe, und, out))
    red, abo = set(), set()
    for p, fo, fe, und, out in procs:
        p.wait()
        fo.close()
        fe.close()
        c = atalanta_counts(out)
        lines = [l.strip() for l in open(und)] if os.path.exists(und) else []
        lines = [l for l in lines if l]
        if len(lines) != c['redundant'] + c['aborted']:
            sys.exit(f'error: {und}: {len(lines)} lines, {out} says {c}')
        red |= set(lines[:c['redundant']])
        abo |= set(lines[c['redundant']:])
    return red, abo


def block_of(inst, name):
    """coarse block of the flat instance name (hierarchy is kept in the names: a__b__c)"""
    s = inst if inst != '-' else name
    s = re.sub(r'^FE_[A-Z]+\d+_', '', s)        # Innovus buffers: classify by the net they drive
    s = s.split('~')[0]
    rules = [
        (r'^SCAN_(MUX|INV)', 'scan cell (mux / inverter)'),
        (r'^scan_[md]_', 'scan cell (mux / inverter)'),
        (r'u_mac_mult_', 'tensor core MAC: multiplier'),
        (r'u_mac_', 'tensor core MAC: adder / accumulator'),
        (r'u_tc_', 'tensor core: control / capture'),
        (r'^u_core_RES_|_rm_', 'result memory'),
        (r'^u_core_bmem|^u_core_b_r', 'B memory'),
        (r'^u_core_SUBP_\d+__mem', 'A memory'),
        (r'^u_core_SUBP_', 'sub-partition glue'),
        (r'^u_core_', 'sm_core glue'),
        (r'^u_stat', 'stat_cdc'),
        (r'^u_en_gen', 'en_gen'),
        (r'cdc', 'CDC'),
    ]
    for pat, lbl in rules:
        if re.search(pat, s):
            return lbl
    return 'top level / other'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True, help='capture-mode model prefix (.bench .names .json)')
    ap.add_argument('--undet', required=True, help='Atalanta -v -U file of the capture-mode run')
    ap.add_argument('--atalanta-out', required=True, help='Atalanta stdout of the same run (fault counts)')
    ap.add_argument('--atalanta', required=True)
    ap.add_argument('--release', action='append', default=[], help='LABEL=PREFIX model with constraints released')
    ap.add_argument('--workdir', required=True)
    ap.add_argument('--retry-backtrack', type=int, default=10000)
    ap.add_argument('--examples', type=int, default=8)
    ap.add_argument('--jobs', type=int, default=12, help='parallel Atalanta runs for the aborted-fault retry')
    args = ap.parse_args()
    os.makedirs(args.workdir, exist_ok=True)

    cnt = atalanta_counts(args.atalanta_out)
    lines = [l.strip() for l in open(args.undet) if l.strip()]
    if len(lines) != cnt['redundant'] + cnt['aborted']:
        sys.exit(f'error: {args.undet} has {len(lines)} faults, {args.atalanta_out} says {cnt}')
    red, abo = lines[:cnt['redundant']], lines[cnt['redundant']:]
    ids, _ = read_names(args.model)
    mg = read_bench_gates(args.model + '.bench')
    musers = defaultdict(list)
    for o, (t, ins) in mg.items():
        for i in ins:
            musers[i].append(o)

    def hier_block(nid):
        """block of the nearest node with a hierarchical name (Innovus FE_RC_* cells have none)"""
        seen, todo = {nid}, [nid]
        while todo:
            x = todo.pop(0)
            name, inst, cell = ids.get(x, ('', '-', ''))
            for c in (inst, name):
                if c not in ('-', '') and not re.match(r'^(FE_R[CN]_|_t\d)', c):
                    b = block_of(c, c)
                    if b != 'top level / other':
                        return b
            for y in mg.get(x, (None, []))[1] + musers.get(x, []):
                if y not in seen and len(seen) < 500:
                    seen.add(y)
                    todo.append(y)
        return 'top level / other'

    def site(fault):
        a, b, v = parse_fault(fault)
        g = b or a                              # branch fault: the gate whose input it is
        name, inst, cell = ids[g]
        if inst == '-' and b:                   # input of an added buffer: use the driver
            name, inst, cell = ids[a]
            g = a
        return name, inst, cell, g

    def block(fault):
        name, inst, cell, g = site(fault)
        if re.match(r'^FE_RC_', inst):          # cell created by Innovus restructuring
            return hier_block(g) + ' [Innovus FE_RC]'
        b = block_of(inst, name)
        return hier_block(g) if b == 'top level / other' else b

    # ---- 2. redundant -> AU / UR by targeted ATPG with constraints released ------------------
    detected_by = defaultdict(list)             # fault -> labels of released models that detect it
    unmapped = Counter()
    for spec in args.release:
        label, prefix = spec.split('=', 1)
        rid, rname = read_names(prefix)
        rg = read_bench_gates(prefix + '.bench')
        fanout = Counter(i for _, ins in rg.values() for i in ins)
        tr, back = [], {}
        for f in red:
            a, b, v = parse_fault(f)
            an = ids[a][0].split('~fo')[0]      # fanout buffers differ between models: use the stem
            if an not in rname:
                unmapped[label] += 1
                continue
            ra = rname[an]
            if b is None or '~fo' in ids[b][0]:
                g = f'{ra} /{v}'
            else:
                bn = ids[b][0]
                rb = rname.get(bn)
                if rb is None or rb not in rg:
                    unmapped[label] += 1
                    continue
                src = [i for i in rg[rb][1] if i == ra or rid[i][0].startswith(an + '~fo')]
                if not src:
                    unmapped[label] += 1        # gate was simplified differently in this model
                    continue
                s = src[0]
                g = f'{s}->{rb} /{v}' if fanout[s] > 1 else f'{s} /{v}'
            tr.append(g)
            back.setdefault(g, []).append(f)
        r_red, r_abo = run_atalanta(args.atalanta, prefix + '.bench', sorted(back), 'release_' + label, args.workdir)
        n_det = 0
        for g, fs in back.items():
            if g not in r_red and g not in r_abo:
                n_det += 1
                for f in fs:
                    detected_by[f].append(label)
        print(f'release {label:10}: {len(back)} redundant faults mapped ({unmapped[label]} not mappable), '
              f'detected {n_det}, still redundant {len(r_red & set(back))}, aborted {len(r_abo & set(back))}')

    au = [f for f in red if detected_by.get(f)]
    ur = [f for f in red if not detected_by.get(f)]

    # ---- 3. aborted faults: retry with more backtracks ----------------------------------------
    retry_red, retry_abo = set(), set(abo)
    if abo and args.retry_backtrack:
        retry_red, retry_abo = run_atalanta(args.atalanta, args.model + '.bench', abo, 'retry_aborted',
                                            args.workdir, extra=('-b', str(args.retry_backtrack)), jobs=args.jobs)
        n_det = len(abo) - len(retry_red) - len(retry_abo)
        print(f'aborted {len(abo)} retried with backtrack limit {args.retry_backtrack}: detected {n_det}, '
              f'proven redundant {len(retry_red)}, still aborted {len(retry_abo)}')

    # ---- report ---------------------------------------------------------------------------------
    N = cnt['collapsed']
    DT = N - len(red) - len(abo)
    print()
    print(f'collapsed faults of the capture-mode model: {N}')
    print(f'  DT  detected                              : {DT}')
    print(f'  AU  untestable only under the constraints : {len(au)}')
    lab = Counter(','.join(sorted(detected_by[f])) for f in au)
    for k, v in lab.most_common():
        print(f'        detectable when releasing {k:20}: {v}')
    print(f'  UR  redundant logic                       : {len(ur)}')
    print(f'  ND  aborted (backtrack limit 10)          : {len(abo)}')
    print(f'fault coverage  DT/all            : {100 * DT / N:.3f} %')
    print(f'test coverage   DT/(all-UR)       : {100 * DT / (N - len(ur)):.3f} %   (TetraMAX definition, AU counted as missed)')
    print(f'Atalanta "test coverage" DT/(all-AU-UR): {100 * DT / (N - len(red)):.3f} %')
    if abo and args.retry_backtrack:
        DT2 = DT + len(abo) - len(retry_red) - len(retry_abo)
        print(f'with the aborted faults retried   : fault coverage {100 * DT2 / N:.3f} %, '
              f'test coverage {100 * DT2 / (N - len(ur) - len(retry_red)):.3f} %')
    st = json.load(open(args.model + '.json'))
    if args.release:
        free = [p for s in args.release for l, p in [s.split('=', 1)] if l == 'all']
        if free:
            fs = json.load(open(free[0] + '.json'))
            print(f'primitive gates: {fs["primitive_gates"]} with nothing held constant, '
                  f'{st["primitive_gates"]} in capture mode -> {fs["primitive_gates"] - st["primitive_gates"]} '
                  f'gates only exist in shift / reset (UD-tied / blocked, tested by the chain test or not at all)')

    print()
    print('by block                                             AU     UR     ND')
    rows = defaultdict(lambda: [0, 0, 0])
    for k, fl in enumerate((au, ur, abo)):
        for f in fl:
            rows[block(f)][k] += 1
    for blk, r in sorted(rows.items(), key=lambda x: -sum(x[1])):
        print(f'  {blk:48} {r[0]:6} {r[1]:6} {r[2]:6}')
    print()
    print('by cell type (AU + UR)')
    cells = Counter(site(f)[2] for f in au + ur)
    nrc = sum(1 for f in au + ur if re.match(r'^FE_RC_', site(f)[1]))
    print(f'  on cells created by Innovus restructuring (FE_RC_*): {nrc} of {len(au) + len(ur)}')
    print('  ' + ', '.join(f'{c} {n}' for c, n in cells.most_common(12)))

    for title, fl in (('AU', au), ('UR', ur), ('ND', abo)):
        if not fl:
            continue
        print()
        print(f'examples {title}:')
        for f in fl[:args.examples]:
            name, inst, cell, _ = site(f)
            extra = f'  (detectable with {",".join(detected_by[f])} released)' if f in detected_by else ''
            print(f'  {f:24} {cell:10} {inst:55} net {name}{extra}')

    with open(os.path.join(args.workdir, 'classes.txt'), 'w') as fo:
        for title, fl in (('AU', au), ('UR', ur), ('ND', abo)):
            for f in fl:
                name, inst, cell, _ = site(f)
                fo.write(f'{title} {f.replace(" ", "")} {cell} {inst} {name} {",".join(detected_by.get(f, []))}\n')
    print(f'\nper-fault list: {args.workdir}/classes.txt')


if __name__ == '__main__':
    main()
