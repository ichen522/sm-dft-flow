#!/usr/bin/env python3
"""
transition_grade.py  -  launch-on-capture (LOC) transition-delay fault grading

Atalanta only does stuck-at. This script grades transition-delay faults by
fault simulation (it does not target them), with the same one-hot capture
protocol as the stuck-at patterns: per pattern only one clock domain is
pulsed, twice (launch + capture); the other domain keeps its scan-loaded
value for both frames.

  frame 1: scan-load state S1, PIs P (held for both frames)
           launch pulse on domain D   -> D's flops: F(S1, P); other flops: S1
  frame 2: capture pulse on domain D  -> observe POs and D's flop D pins

--scheme los grades launch-on-shift instead: the launch is the last shift
pulse of domain D (each flop of D's chains takes its scan predecessor's
value, the first one a random scan_in bit), then one capture pulse on D
with scan_en = 0. The other domain holds. LOS needs scan_en to switch at
speed between launch and capture; LOC does not.

A slow-to-rise fault on line x is detected when x goes 0 -> 1 from frame 1
to frame 2 in the good machine and the faulty frame-2 value (x still 0)
reaches an observed output. Slow-to-fall likewise. Faults are sampled
uniformly from all stems and fanout branches of the full model, both
polarities, as in check_coverage.py; a fault counts as detected if any
domain's patterns detect it.

Pattern sets per domain: the domain's stuck-at patterns (--set PREFIX:CLOCK,
mapped by name onto the full model) reused as LOC loads, plus --random N
random loads.

Usage:
  python3 scripts/transition_grade.py --full atpg/sm_top --chains scan/chains.json \
      --set atpg/sm_top_tile_clk:tile_clk --set atpg/sm_top_clk_mem:clk_mem \
      --random 4096 --samples 20000 --seed 1 [--crosscheck 60] [--scheme loc,los]
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
_spec2 = importlib.util.spec_from_file_location('coverage_union', os.path.join(HERE, 'coverage_union.py'))
cu = importlib.util.module_from_spec(_spec2)
_spec2.loader.exec_module(cu)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--full', required=True, help='prefix of the all-flops-observed model')
    ap.add_argument('--chains', required=True)
    ap.add_argument('--set', action='append', default=[], help='PREFIX:CLOCK stuck-at pattern set')
    ap.add_argument('--random', type=int, default=4096, help='random LOC patterns per domain')
    ap.add_argument('--samples', type=int, default=20000)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--crosscheck', type=int, default=0,
                    help='re-check N faults per domain (half detected, half not) by brute-force simulation')
    ap.add_argument('--scheme', default='loc', help='comma list of loc (launch on capture), los (launch on shift)')
    args = ap.parse_args()
    schemes = args.scheme.split(',')
    if any(x not in ('loc', 'los') for x in schemes):
        sys.exit('error: --scheme takes loc and/or los')
    rng = random.Random(args.seed)

    domain = {}
    chains = json.load(open(args.chains))['chains']
    for c in chains:
        for f in c['flops']:
            domain[f] = c['clock']
    clocks = sorted(set(domain.values()))

    ins, outs, gates = cc.read_bench(args.full + '.bench')
    fmap = cu.read_map(args.full)
    ppi_of, ppo_of = {}, {}
    in_by_label = {}
    for n in ins:
        for kind, lbl in fmap[n]:
            in_by_label[(kind, lbl)] = n
            if kind == 'PPI':
                ppi_of[lbl] = n
    out_obs = defaultdict(set)          # clock -> observed output nets
    for n in outs:
        for kind, lbl in fmap[n]:
            if kind == 'PPO':
                ppo_of[lbl] = n
                out_obs[domain[lbl]].add(n)
            else:
                for c in clocks:
                    out_obs[c].add(n)
    const_d = [l.split()[1][:-2] for l in json.load(open(args.full + '.json'))['constant_observation_points']]
    pi_nets = [n for n in ins if not any(k == 'PPI' for k, _ in fmap[n])]
    print(f'{len(ins)} inputs ({len(pi_nets)} PI, {len(ppi_of)} PPI), {len(outs)} outputs, {len(gates)} gates')

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

    def simulate(inputs, mask, stem=None, branch=None, fv=None):
        """full simulation; optionally with net `stem` or gate input `branch`=(gate, pin) forced to fv"""
        v = dict(inputs)
        if stem in v:
            v[stem] = fv
        for o in order:
            t, i = gates[o]
            vals = [v[x] for x in i]
            if branch and o == branch[0]:
                vals[branch[1]] = fv
            v[o] = fv if o == stem else cc.ev(t, vals, mask)
        return v

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
    picks = rng.sample(range(2 * len(sites)), min(args.samples, 2 * len(sites)))

    def load_set(prefix):
        s_ins, s_outs, _ = cc.read_bench(prefix + '.bench')
        smap = cu.read_map(prefix)
        names, pats = cc.read_test(prefix + '.test', len(s_ins), len(s_outs))
        val = {n: 0 for n in ins}
        for c, n in enumerate(names):
            tgt = in_by_label.get(smap[n][0])
            if tgt is None:
                continue
            v = 0
            for p, (ib, ob) in enumerate(pats):
                if ib[c] == '1':
                    v |= 1 << p
            val[tgt] = v
        return val, len(pats)

    def grade(label, clk, f1, P, scheme):
        """returns the set of detected pick indices"""
        mask = (1 << P) - 1
        obs = out_obs[clk]
        g1 = simulate(f1, mask)
        f2 = dict(f1)                                   # PIs and the other domain hold
        if scheme == 'loc':                             # launch = functional capture
            for fl, q in ppi_of.items():
                if domain[fl] == clk:
                    f2[q] = mask if fl in const_d else g1[ppo_of[fl]]
        else:                                           # launch = last shift: flops[0] is next to scan_in
            for c in chains:
                if c['clock'] != clk:
                    continue
                fl = c['flops']
                for i in range(len(fl) - 1, 0, -1):
                    f2[ppi_of[fl[i]]] = f1[ppi_of[fl[i - 1]]]
                f2[ppi_of[fl[0]]] = rng.getrandbits(P)  # scan_in bit
        g2 = simulate(f2, mask)
        detected, act = set(), 0
        checked = [0, 0, 0, 0, 0]   # checked, agreeing, brute-force detected, #detected class, #undetected class
        for idx, f in enumerate(picks):
            kind, net, br = sites[f // 2]
            str_ = (f % 2 == 0)
            x1, x2 = g1[net], g2[net]
            trans = (~x1 & x2 & mask) if str_ else (x1 & ~x2 & mask)
            if not trans:
                continue
            act += 1
            fv = (x2 & ~trans) if str_ else (x2 | trans)
            faulty, heap, queued = {}, [], set()

            def push(g):
                if g not in queued:
                    queued.add(g)
                    heapq.heappush(heap, (level[g], g))

            diff = 0
            if kind == 'stem':
                faulty[net] = fv
                if net in obs:
                    diff |= fv ^ x2
                for u in users[net]:
                    push(u)
            else:
                push(br[0])
            while heap and not diff:
                _, g = heapq.heappop(heap)
                t, i = gates[g]
                vals = [fv if (kind == 'branch' and g == br[0] and pin == br[1]) else faulty.get(x, g2[x])
                        for pin, x in enumerate(i)]
                v = cc.ev(t, vals, mask)
                if v == g2[g]:
                    continue
                faulty[g] = v
                if g in obs:
                    diff |= v ^ g2[g]
                for u in users[g]:
                    push(u)
            if diff:
                detected.add(idx)
            cls = 3 if diff else 4
            if checked[cls] < (args.crosscheck + 1) // 2:
                bf = simulate(f2, mask, stem=net if kind == 'stem' else None,
                              branch=br if kind == 'branch' else None, fv=fv)
                bf_det = any(bf[o] != g2[o] for o in obs)
                checked[cls] += 1
                checked[0] += 1
                checked[1] += (bf_det == bool(diff))
                checked[2] += bf_det
        n = len(picks)
        print(f'  {label:34} {P:6} patterns: {len(detected)}/{n} = {100 * len(detected) / n:6.2f} %, '
              f'launched: {100 * act / n:.2f} %')
        if args.crosscheck:
            print(f'    crosscheck vs brute force: {checked[1]}/{checked[0]} agree '
                  f'({checked[3]} called detected, {checked[4]} launched but undetected; brute force detects {checked[2]})')
        return detected

    sets = {}
    for spec in args.set:
        prefix, clk = spec.rsplit(':', 1)
        sets[clk] = prefix
    loaded = {clk: load_set(sets[clk]) for clk in clocks if clk in sets}
    n = len(picks)
    every = set()
    for scheme in schemes:
        S = scheme.upper()
        union_sa, union_all = set(), set()
        for clk in clocks:
            print(f'{S}, capture domain {clk}:')
            if clk in loaded:
                f1, P = loaded[clk]
                d = grade(f'stuck-at patterns as {S}', clk, f1, P, scheme)
                union_sa |= d
                union_all |= d
            if args.random:
                f1r = {n_: rng.getrandbits(args.random) for n_ in ins}
                union_all |= grade(f'random {S}', clk, f1r, args.random, scheme)
        every |= union_all
        tag = '' if schemes == ['loc'] else f'{S} '
        for label, u in ((f'{tag}stuck-at patterns (all domains)', union_sa), (f'{tag}+ random (all domains)', union_all)):
            lo, hi = cc.wilson(len(u), n)
            print(f'union {label:36}: {len(u)}/{n} = {100 * len(u) / n:.2f} %  (95% {100 * lo:.2f} - {100 * hi:.2f})')
    if len(schemes) > 1:
        lo, hi = cc.wilson(len(every), n)
        print(f'union {"LOC + LOS, all patterns":36}: {len(every)}/{n} = {100 * len(every) / n:.2f} %  '
              f'(95% {100 * lo:.2f} - {100 * hi:.2f})')


if __name__ == '__main__':
    main()
