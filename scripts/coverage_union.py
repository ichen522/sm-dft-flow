#!/usr/bin/env python3
"""
coverage_union.py  -  stuck-at coverage of one-hot-capture pattern sets, combined

With two asynchronous clocks, every pattern captures only one domain
(the other one is not pulsed and keeps its loaded value). Each domain has
its own ATPG model (netlist2bench.py --capture CLK) and its own Atalanta
pattern set. This script grades the union of all sets on ONE common fault
universe: the full model where every flop D is observable (netlist2bench.py
without --capture).

For every set:
  * its patterns are mapped by name (PI port bit / flop) onto the inputs of
    the full model (inputs the set's model did not need are set to 0)
  * good-machine check: every expected output Atalanta wrote is compared
    with the full model's value for the same PO / flop D
  * observed outputs = POs + D of the flops in the set's capture domain
A sampled fault counts as detected if any set detects it.

Usage:
  python3 scripts/coverage_union.py --full atpg/sm_top --chains scan/chains.json \
      --set atpg/sm_top_tile_clk:tile_clk --set atpg/sm_top_clk_mem:clk_mem \
      --samples 20000 --seed 1
  (each --set PREFIX:CLOCK reads PREFIX.bench / PREFIX.map / PREFIX.test)
"""

import argparse
import heapq
import importlib.util
import json
import os
import random
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location('check_coverage', os.path.join(HERE, 'check_coverage.py'))
cc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cc)


def read_map(prefix):
    """N-id -> list of (kind, label) with kind PI / PPI / PO / PPO (flop labels without /Q /D)"""
    m = defaultdict(list)
    for ln in open(prefix + '.map'):
        f = ln.split()
        if len(f) >= 3 and f[1] in ('PI', 'PPI', 'PO', 'PPO'):
            lbl = f[2][:-2] if f[1] in ('PPI', 'PPO') else f[2]
            m[f[0]].append((f[1], lbl))
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--full', required=True, help='prefix of the all-flops-observed model')
    ap.add_argument('--chains', required=True)
    ap.add_argument('--set', action='append', required=True, help='PREFIX:CLOCK')
    ap.add_argument('--samples', type=int, default=20000)
    ap.add_argument('--seed', type=int, default=1)
    args = ap.parse_args()

    domain = {}
    for c in json.load(open(args.chains))['chains']:
        for f in c['flops']:
            domain[f] = c['clock']

    ins, outs, gates = cc.read_bench(args.full + '.bench')
    fmap = read_map(args.full)
    in_by_label = {}
    for n in ins:
        for kind, lbl in fmap[n]:
            in_by_label[(kind, lbl)] = n
    out_labels = {n: fmap[n] for n in outs}

    users = defaultdict(list)
    for o, (t, i) in gates.items():
        for x in i:
            users[x].append(o)
    indeg = {o: sum(1 for x in set(i) if x in gates) for o, (t, i) in gates.items()}
    order = [o for o, d in indeg.items() if d == 0]
    k = 0
    while k < len(order):
        for u in users[order[k]]:
            indeg[u] -= 1
            if indeg[u] == 0:
                order.append(u)
        k += 1
    level = {n: 0 for n in ins}
    for o in order:
        level[o] = 1 + max(level.get(x, 0) for x in gates[o][1])

    sets = []
    for spec in args.set:
        prefix, clk = spec.rsplit(':', 1)
        s_ins, s_outs, _ = cc.read_bench(prefix + '.bench')
        smap = read_map(prefix)
        names, pats = cc.read_test(prefix + '.test', len(s_ins), len(s_outs))
        P = len(pats)
        mask = (1 << P) - 1
        val = {n: 0 for n in ins}
        for c, n in enumerate(names):
            kind, lbl = smap[n][0]
            tgt = in_by_label.get((kind, lbl))
            if tgt is None:
                continue
            v = 0
            for p, (ib, ob) in enumerate(pats):
                if ib[c] == '1':
                    v |= 1 << p
            val[tgt] = v
        good = dict(val)
        for o in order:
            t, i = gates[o]
            good[o] = cc.ev(t, [good[x] for x in i], mask)
        # good-machine check against Atalanta's expected outputs, by label
        full_out_by_label = {}
        for n, labs in out_labels.items():
            for kind, lbl in labs:
                full_out_by_label[(kind, lbl)] = n
        compared = differ = 0
        for j, n in enumerate(s_outs):
            for kind, lbl in smap[n]:
                fn = full_out_by_label.get((kind, lbl))
                if fn is None:
                    continue
                for p, (ib, ob) in enumerate(pats):
                    compared += 1
                    if ((good[fn] >> p) & 1) != int(ob[j]):
                        differ += 1
        observed = {n for n, labs in out_labels.items()
                    if any(k == 'PO' or (k == 'PPO' and domain.get(l) == clk) for k, l in labs)}
        print(f'set {prefix} (capture {clk}): {P} patterns, good-machine check {compared} bits, {differ} differ, '
              f'{len(observed)} observed outputs')
        sets.append((prefix, clk, mask, good, observed))

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
    rng = random.Random(args.seed)
    picks = rng.sample(range(2 * len(sites)), min(args.samples, 2 * len(sites)))

    def detects(st, kind, net, br, sa):
        prefix, clk, mask, good, observed = st
        cv = mask if sa else 0
        faulty, heap, queued = {}, [], set()

        def push(g):
            if g not in queued:
                queued.add(g)
                heapq.heappush(heap, (level[g], g))
        if kind == 'stem':
            if good[net] == cv:
                return False
            faulty[net] = cv
            if net in observed:
                return True
            for u in users[net]:
                push(u)
        else:
            push(br[0])
        while heap:
            _, g = heapq.heappop(heap)
            t, i = gates[g]
            vals = [cv if (kind == 'branch' and g == br[0] and pin == br[1]) else faulty.get(x, good[x])
                    for pin, x in enumerate(i)]
            v = cc.ev(t, vals, mask)
            if v == good[g]:
                continue
            faulty[g] = v
            if g in observed:
                return True
            for u in users[g]:
                push(u)
        return False

    per = [0] * len(sets)
    union = 0
    for f in picks:
        kind, net, br = sites[f // 2]
        hit = False
        for si_, st in enumerate(sets):
            if detects(st, kind, net, br, f % 2):
                per[si_] += 1
                hit = True
        union += hit
    n = len(picks)
    print(f'fault universe (uncollapsed, full model): {2 * len(sites)} faults; sampled {n}')
    for (prefix, clk, *_), d in zip(sets, per):
        print(f'  {clk:10} set alone : {d}/{n} = {100 * d / n:.2f} %')
    lo, hi = cc.wilson(union, n)
    print(f'union coverage: {union}/{n} = {100 * union / n:.2f} %  (95% interval {100 * lo:.2f} - {100 * hi:.2f} %)')


if __name__ == '__main__':
    main()
