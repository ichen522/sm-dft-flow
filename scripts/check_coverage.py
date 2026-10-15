#!/usr/bin/env python3
"""
check_coverage.py  -  independent check of Atalanta's patterns and coverage

Atalanta reports its own fault coverage; this script does not trust it:
1. Good-machine check: simulates every pattern of the .test file on the
   .bench and compares every primary output with the response Atalanta
   wrote next to the pattern.
2. Sampled fault simulation: draws N faults uniformly from the full
   (uncollapsed) stuck-at fault universe - s-a-0 / s-a-1 on every gate
   output and primary input (stems) and on every fanout branch - injects
   each one, simulates all patterns (bit-parallel, event-driven through
   the fault's fanout cone) and counts the detected ones.
   The detected fraction with its 95% Wilson interval estimates the
   uncollapsed fault coverage of the pattern set.

Atalanta's own number is on its collapsed fault list, so the two are close
but need not be identical (collapsing changes the weights of the faults).

Usage:
  python3 scripts/check_coverage.py --bench atpg/sm_top.bench --test atpg/sm_top.test \
      --samples 4000 --seed 1
"""

import argparse
import heapq
import math
import random
import re
import sys
from collections import defaultdict


def read_bench(path):
    ins, outs, gates = [], [], {}
    for ln in open(path):
        ln = ln.split('#', 1)[0].strip()
        if not ln:
            continue
        m = re.match(r'INPUT\((\w+)\)', ln)
        if m:
            ins.append(m.group(1))
            continue
        m = re.match(r'OUTPUT\((\w+)\)', ln)
        if m:
            outs.append(m.group(1))
            continue
        m = re.match(r'(\w+)\s*=\s*(\w+)\((.*)\)', ln)
        gates[m.group(1)] = (m.group(2).upper(), [x.strip() for x in m.group(3).split(',')])
    return ins, outs, gates


def read_test(path, n_in, n_out):
    """returns (input names in file order, [(in_bits, out_bits)])"""
    names, pats, in_hdr = [], [], False
    for ln in open(path):
        if ln.startswith('* Primary inputs'):
            in_hdr = True
            continue
        if ln.startswith('*'):
            in_hdr = False
            continue
        if in_hdr:
            names += ln.split()
            continue
        m = re.match(r'\s*\d+:\s*([01xX]+)\s+([01xX]+)', ln)
        if m:
            pats.append((m.group(1), m.group(2)))
    if len(names) != n_in:
        sys.exit(f'error: test file lists {len(names)} inputs, bench has {n_in}')
    return names, pats


def ev(t, vals, mask):
    if t in ('AND', 'NAND'):
        r = mask
        for v in vals:
            r &= v
    elif t in ('OR', 'NOR'):
        r = 0
        for v in vals:
            r |= v
    elif t in ('XOR', 'XNOR'):
        r = 0
        for v in vals:
            r ^= v
    elif t in ('BUFF', 'BUF', 'NOT'):
        r = vals[0]
    else:
        sys.exit(f'error: gate type {t}')
    return (~r & mask) if t in ('NAND', 'NOR', 'XNOR', 'NOT') else r


def wilson(k, n, z=1.96):
    if n == 0:
        return 0.0, 0.0
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return c - h, c + h


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bench', required=True)
    ap.add_argument('--test', required=True)
    ap.add_argument('--samples', type=int, default=4000)
    ap.add_argument('--seed', type=int, default=1)
    args = ap.parse_args()

    ins, outs, gates = read_bench(args.bench)
    names, pats = read_test(args.test, len(ins), len(outs))
    P = len(pats)
    mask = (1 << P) - 1
    print(f'{len(ins)} inputs, {len(outs)} outputs, {len(gates)} gates, {P} patterns')

    # levelize
    level = {n: 0 for n in ins}
    users = defaultdict(list)
    for o, (t, i) in gates.items():
        for x in i:
            users[x].append(o)
    indeg = {o: sum(1 for x in set(i) if x in gates) for o, (t, i) in gates.items()}
    order = [o for o, d in indeg.items() if d == 0]
    k = 0
    while k < len(order):
        o = order[k]
        k += 1
        for u in users[o]:
            indeg[u] -= 1
            if indeg[u] == 0:
                order.append(u)
    if len(order) != len(gates):
        sys.exit('error: combinational loop')
    for o in order:
        level[o] = 1 + max(level.get(x, 0) for x in gates[o][1])

    # good simulation (pattern p = bit p)
    good = {}
    col = {n: i for i, n in enumerate(names)}
    for n in ins:
        c = col[n]
        v = 0
        for p, (ib, ob) in enumerate(pats):
            if ib[c] == '1':
                v |= 1 << p
        good[n] = v
    for o in order:
        t, i = gates[o]
        good[o] = ev(t, [good[x] for x in i], mask)

    # 1. responses written by Atalanta
    bad_resp = 0
    xs = 0
    for j, n in enumerate(outs):
        for p, (ib, ob) in enumerate(pats):
            e = ob[j]
            if e in 'xX':
                xs += 1
                continue
            if ((good[n] >> p) & 1) != int(e):
                bad_resp += 1
    print(f'good-machine check: {len(outs) * P - xs} expected output bits compared, {bad_resp} differ'
          + (f' ({xs} X bits skipped)' if xs else ''))

    # 2. fault universe: stems (gate outputs + PIs) and fanout branches
    po_set = set(outs)
    fanout = defaultdict(int)
    for o, (t, i) in gates.items():
        for x in i:
            fanout[x] += 1
    for n in outs:
        fanout[n] += 1
    sites = [('stem', n, None) for n in list(ins) + list(gates)]
    for o, (t, i) in gates.items():
        for pin, x in enumerate(i):
            if fanout[x] > 1:
                sites.append(('branch', x, (o, pin)))
    universe = 2 * len(sites)
    rng = random.Random(args.seed)
    picks = rng.sample(range(universe), min(args.samples, universe))

    det = 0
    by_kind = defaultdict(lambda: [0, 0])
    for f in picks:
        kind, net, br = sites[f // 2]
        sa = f % 2
        cv = mask if sa else 0
        faulty = {}
        heap = []
        queued = set()

        def push(g):
            if g not in queued:
                queued.add(g)
                heapq.heappush(heap, (level[g], g))

        if kind == 'stem':
            if good[net] == cv:
                by_kind[kind][1] += 1
                continue                      # never activated
            faulty[net] = cv
            if net in po_set:
                det += 1
                by_kind[kind][0] += 1
                by_kind[kind][1] += 1
                continue
            for u in users[net]:
                push(u)
            branch_gate = branch_pin = None
        else:
            branch_gate, branch_pin = br
            push(branch_gate)
        found = False
        while heap:
            _, g = heapq.heappop(heap)
            t, i = gates[g]
            vals = []
            for pin, x in enumerate(i):
                if kind == 'branch' and g == branch_gate and pin == branch_pin:
                    vals.append(cv)
                else:
                    vals.append(faulty.get(x, good[x]))
            v = ev(t, vals, mask)
            if v == good[g]:
                continue
            faulty[g] = v
            if g in po_set:
                found = True
                break
            for u in users[g]:
                push(u)
        by_kind[kind][1] += 1
        if found:
            det += 1
            by_kind[kind][0] += 1
    n = len(picks)
    lo, hi = wilson(det, n)
    print(f'fault universe (uncollapsed): {universe} faults on {len(sites)} sites')
    for kd, (d, m) in by_kind.items():
        print(f'  {kd:6}: {d}/{m} detected')
    print(f'sampled fault coverage: {det}/{n} = {100 * det / n:.2f} %  '
          f'(95% interval {100 * lo:.2f} - {100 * hi:.2f} %)')


if __name__ == '__main__':
    main()
