#!/usr/bin/env python3
"""
diagnose.py  -  failure diagnosis from tester data, the way it is done on silicon

Inputs are only what a test floor has:
  * the pattern data that was applied (gen_scan_tb.py output: load / unload /
    mask / pi / po / cap .mem files and ports.txt; same content as a STIL file)
  * the fail log of the run (tb_atpg.v +FAILLOG=file: "SO pattern chain shift
    got" and "PO pattern bit got" for every miscompare)
  * the netlist as an ATPG model (netlist2bench.py, all flops observed)
No ATPG pattern file and no knowledge of the injected defect is used.

  logic   single stuck-at diagnosis. Candidates = nets in the fan-in cone of
          every failing observation point; each candidate stuck-at-0/1 is
          fault-simulated over the applied patterns and compared with the fail
          log: TFSF (tester fail, sim fail), TFSP (tester fail, sim pass),
          TPSF (tester pass, sim fail). A perfect explanation has TFSP = TPSF = 0.
  chain   scan chain diagnosis. Finds the failing chain and the fault type
          from the unload values, bounds the defect position, then simulates
          "flop Q at position c stuck" for every candidate c (shift corrupts
          the load of every flop behind c, capture and unload as in the
          pattern) and ranks the candidates by miscompares.

Usage:
  python3 scripts/diagnose.py logic --tb atpg/serial --faillog fail.log --model atpg/sm_top \
      --chains scan/chains.json --first 0 --last 99
  python3 scripts/diagnose.py chain --tb atpg/serial --faillog fail.log --model atpg/sm_top \
      --chains scan/chains.json --first 0 --last 19
"""

import argparse
import heapq
import importlib.util
import json
import os
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location('check_coverage', os.path.join(HERE, 'check_coverage.py'))
cc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cc)


def bits(line):
    """readmemb line (MSB first) -> list, index = bit number"""
    s = line.strip()
    return s[::-1]


class Tester:
    """applied patterns + fail log, mapped onto chain positions / port names"""

    def __init__(self, tb, faillog, chains, first, last):
        self.chains = chains
        self.nch = len(chains)
        self.first, self.last = first, last
        rd = lambda f: open(os.path.join(tb, f)).read().split()
        load, unl, msk = rd('load.mem'), rd('unload.mem'), rd('unload_mask.mem')
        pi, po, pom, cap = rd('pi.mem'), rd('po.mem'), rd('po_mask.mem'), rd('cap.mem')
        self.pi_names, self.po_names, self.clocks = [], [], []
        for ln in open(os.path.join(tb, 'ports.txt')):
            f = ln.split()
            if ln.startswith('# pi vector'):
                self.clocks = ln.split('= clock')[1].split()
            elif f and f[0] == 'pi':
                self.pi_names.append(f[2])
            elif f and f[0] == 'po':
                self.po_names.append(f[2])
        self.pats = list(range(first, last + 1))
        self.L = len(load[0])
        self.load, self.unl, self.msk = {}, {}, {}
        self.pi, self.po, self.pom, self.cap = {}, {}, {}, {}
        for p in self.pats:
            for k in range(self.nch):
                self.load[p, k] = bits(load[p * self.nch + k])
                self.unl[p, k] = bits(unl[p * self.nch + k])
                self.msk[p, k] = bits(msk[p * self.nch + k])
            self.pi[p], self.po[p], self.pom[p] = bits(pi[p]), bits(po[p]), bits(pom[p])
            cb = bits(cap[p])
            self.cap[p] = {c for i, c in enumerate(self.clocks) if cb[i] == '1'}
        self.fail_so = {}                       # (p, chain, shift) -> got
        self.fail_po = {}                       # (p, po bit) -> got
        for ln in open(faillog):
            f = ln.split()
            if f[0] == 'SO':
                p, k, t, g = int(f[1]), int(f[2]), int(f[3]), f[4]
                if first <= p <= last:
                    self.fail_so[p, k, t] = g
            elif f[0] == 'PO':
                p, b, g = int(f[1]), int(f[2]), f[3]
                if first <= p <= last:
                    self.fail_po[p, b] = g

    def flop_at(self, k, t):
        """flop on scan_out of chain k before shift edge t of the unload (None: padding)"""
        f = self.chains[k]['flops']
        j = len(f) - 1 - t
        return f[j] if j >= 0 else None

    def loaded(self, p, k):
        """{flop: loaded value} of chain k in pattern p (fault free shift)"""
        f = self.chains[k]['flops']
        out = {}
        for t in range(self.L):
            i = self.L - 1 - t
            if i < len(f):
                out[f[i]] = self.load[p, k][t]
        return out


class Model:
    def __init__(self, prefix):
        self.ins, self.outs, self.gates = cc.read_bench(prefix + '.bench')
        self.names = {}
        for ln in open(prefix + '.names'):
            nid, name, inst, cell = ln.split()
            self.names[nid] = (name, inst, cell)
        self.ppi, self.ppo, self.pi, self.po = {}, {}, {}, {}
        for ln in open(prefix + '.map'):
            f = ln.split()
            if len(f) >= 3 and f[1] in ('PI', 'PPI', 'PO', 'PPO'):
                key = f[2][:-2] if f[1] in ('PPI', 'PPO') else f[2]
                getattr(self, f[1].lower())[key] = f[0]
        self.users = defaultdict(list)
        for o, (t, i) in self.gates.items():
            for x in i:
                self.users[x].append(o)
        indeg = {o: sum(1 for x in set(i) if x in self.gates) for o, (t, i) in self.gates.items()}
        self.order = [o for o, d in indeg.items() if d == 0]
        k = 0
        while k < len(self.order):
            for u in self.users[self.order[k]]:
                indeg[u] -= 1
                if indeg[u] == 0:
                    self.order.append(u)
            k += 1
        self.level = {n: 0 for n in self.ins}
        for o in self.order:
            self.level[o] = 1 + max(self.level.get(x, 0) for x in self.gates[o][1])

    def simulate(self, inputs, mask):
        v = dict(inputs)
        for o in self.order:
            t, i = self.gates[o]
            v[o] = cc.ev(t, [v[x] for x in i], mask)
        return v

    def propagate(self, good, mask, changes):
        """event-driven: inputs/nets in `changes` take new values; returns {net: new value} of everything that changed"""
        faulty, heap, queued = {}, [], set()

        def push(g):
            if g not in queued:
                queued.add(g)
                heapq.heappush(heap, (self.level[g], g))
        for n, val in changes.items():
            if val != good[n]:
                faulty[n] = val
                for u in self.users[n]:
                    push(u)
        while heap:
            _, g = heapq.heappop(heap)
            if g in changes:
                continue
            t, i = self.gates[g]
            val = cc.ev(t, [faulty.get(x, good[x]) for x in i], mask)
            if val != good[g]:
                faulty[g] = val
                for u in self.users[g]:
                    push(u)
            elif g in faulty:
                del faulty[g]
        return faulty

    def label(self, nid):
        name, inst, cell = self.names.get(nid, (nid, '-', '-'))
        return f'{name} ({cell} {inst})'


def pattern_inputs(T, M):
    """bit-parallel model inputs over the applied patterns (bit i = pattern T.pats[i])"""
    vals = {n: 0 for n in M.ins}
    pi_idx = {nm: i for i, nm in enumerate(T.pi_names)}
    for i, p in enumerate(T.pats):
        for nm, nid in M.pi.items():
            if nm in pi_idx and T.pi[p][pi_idx[nm]] == '1':
                vals[nid] |= 1 << i
        for k in range(T.nch):
            for fl, b in T.loaded(p, k).items():
                if b == '1' and fl in M.ppi:
                    vals[M.ppi[fl]] |= 1 << i
    return vals


def observed_failures(T, M):
    """tester fails as (pattern index, model output net) + which outputs are observed per pattern"""
    domain = {f: c['clock'] for c in T.chains for f in c['flops']}
    po_idx = {nm: i for i, nm in enumerate(T.po_names)}
    obs = defaultdict(int)                      # model output -> bitmask of patterns where it is compared
    fails = defaultdict(int)                    # model output -> bitmask of patterns where it failed
    unexplained = 0                             # fails on flops that were not captured (load problems)
    for i, p in enumerate(T.pats):
        for nm, nid in M.po.items():
            if nm in po_idx and T.pom[p][po_idx[nm]] == '1':
                obs[nid] |= 1 << i
        for k in range(T.nch):
            for t in range(T.L):
                if T.msk[p, k][t] != '1':
                    continue
                fl = T.flop_at(k, t)
                if domain[fl] in T.cap[p] and fl in M.ppo:
                    obs[M.ppo[fl]] |= 1 << i
    for (p, k, t), g in T.fail_so.items():
        fl = T.flop_at(k, t)
        if domain[fl] in T.cap[p] and fl in M.ppo:
            fails[M.ppo[fl]] |= 1 << T.pats.index(p)
        else:
            unexplained += 1
    for (p, b), g in T.fail_po.items():
        nm = T.po_names[b]
        if nm in M.po:
            fails[M.po[nm]] |= 1 << T.pats.index(p)
    return obs, fails, unexplained


def cmd_logic(args):
    chains = json.load(open(args.chains))['chains']
    T = Tester(args.tb, args.faillog, chains, args.first, args.last)
    M = Model(args.model)
    P = len(T.pats)
    mask = (1 << P) - 1
    obs, fails, unexpl = observed_failures(T, M)
    nf = sum(bin(v).count('1') for v in fails.values())
    print(f'{P} patterns applied, {len(T.fail_so)} scan_out + {len(T.fail_po)} PO fails in the log '
          f'-> {nf} failing (pattern, observation point) pairs on {len(fails)} observation points')
    if unexpl:
        print(f'WARNING: {unexpl} fails on flops that were not captured: load / chain problem, try "chain"')
    if not fails:
        sys.exit('no failures to diagnose')

    # candidates: nets in the fan-in cone of every failing observation point (single-fault assumption)
    def cone(n):
        seen, st = set(), [n]
        while st:
            x = st.pop()
            if x in seen:
                continue
            seen.add(x)
            if x in M.gates:
                st.extend(M.gates[x][1])
        return seen
    fl = sorted(fails, key=lambda n: bin(fails[n]).count('1'), reverse=True)
    cand = cone(fl[0])
    for n in fl[1:]:
        cand &= cone(n)
    print(f'candidate nets (in the fan-in cone of all {len(fails)} failing points): {len(cand)}')

    good = M.simulate(pattern_inputs(T, M), mask)
    # sanity: the fault-free model must reproduce the expected values of the pattern data
    domain = {f: c['clock'] for c in T.chains for f in c['flops']}
    po_idx = {nm: i for i, nm in enumerate(T.po_names)}
    n_chk = n_bad = 0
    for i, p in enumerate(T.pats):
        for k in range(T.nch):
            for t in range(T.L):
                f = T.flop_at(k, t)
                if T.msk[p, k][t] == '1' and domain[f] in T.cap[p] and f in M.ppo:
                    n_chk += 1
                    n_bad += ((good[M.ppo[f]] >> i) & 1) != int(T.unl[p, k][t])
        for nm, nid in M.po.items():
            b = po_idx.get(nm)
            if b is not None and T.pom[p][b] == '1':
                n_chk += 1
                n_bad += ((good[nid] >> i) & 1) != int(T.po[p][b])
    print(f'fault-free model vs expected values in the pattern data: {n_chk} bits, {n_bad} differ')
    if n_bad:
        sys.exit('error: model and pattern data do not agree')
    res = []
    for n in cand:
        for v in (0, 1):
            fv = mask if v else 0
            if good[n] == fv:
                continue                        # never excited
            ch = M.propagate(good, mask, {n: fv})
            tfsf = tfsp = tpsf = 0
            for o in set(fails) | {x for x in ch if x in obs}:
                sim = ((ch[o] ^ good[o]) if o in ch else 0) & obs.get(o, 0)
                tf = fails.get(o, 0)
                tfsf += bin(sim & tf).count('1')
                tfsp += bin(tf & ~sim).count('1')
                tpsf += bin(sim & ~tf).count('1')
            res.append((tfsp + tpsf, -tfsf, n, v, tfsf, tfsp, tpsf))
    res.sort()
    print(f'simulated {len(res)} candidate faults')
    print(f'{"rank":>4} {"TFSF":>6} {"TFSP":>6} {"TPSF":>6}  fault')
    best = res[0][0] if res else None
    for r, (_, _, n, v, tfsf, tfsp, tpsf) in enumerate(res[:args.top]):
        mark = '  <- perfect match' if tfsp == 0 and tpsf == 0 else ''
        print(f'{r + 1:4} {tfsf:6} {tfsp:6} {tpsf:6}  {M.label(n)} stuck-at-{v}{mark}')
    nperfect = sum(1 for x in res if x[0] == 0)
    print(f'perfect matches: {nperfect} (faults that are equivalent for these patterns form one class)')
    if args.expect:
        name, v = args.expect.rsplit('=', 1)
        ranks = [i + 1 for i, x in enumerate(res) if M.names.get(x[2], ('',))[0] == name and x[3] == int(v)]
        print(f'injected defect {name} stuck-at-{v}: ' + (f'rank {ranks[0]}, score {res[ranks[0] - 1][0]} '
              f'(best score {best})' if ranks else 'not among the candidates'))


def cmd_chain(args):
    chains = json.load(open(args.chains))['chains']
    T = Tester(args.tb, args.faillog, chains, args.first, args.last)
    M = Model(args.model)
    P = len(T.pats)
    mask = (1 << P) - 1
    domain = {f: c['clock'] for c in chains for f in c['flops']}

    # 1. which chain, which fault type
    per_chain = defaultdict(int)
    for (p, k, t) in T.fail_so:
        per_chain[k] += 1
    print(f'{P} patterns applied; scan_out fails per chain: ' +
          ', '.join(f'chain {k}: {n}' for k, n in sorted(per_chain.items())) + f'; PO fails: {len(T.fail_po)}')
    if not per_chain:
        sys.exit('no unload failures')
    K = max(per_chain, key=per_chain.get)
    fl = chains[K]['flops']
    l = len(fl)

    def got(p, t):
        e = T.unl[p, K][t]
        return T.fail_so.get((p, K, t), e)
    vals = [got(p, t) for p in T.pats for t in range(T.L) if T.msk[p, K][t] == '1']
    n1 = vals.count('1')
    n0 = vals.count('0')
    v = 0 if n1 * 20 < n0 else (1 if n0 * 20 < n1 else None)
    # the end of the unload (flops close to scan_in) is where a stuck chain shows a constant
    tail = [got(p, t) for p in T.pats for t in range(T.L) if T.msk[p, K][t] == '1' and len(fl) - 1 - t < l // 4]
    vt = '0' if tail.count('1') == 0 else ('1' if tail.count('0') == 0 else None)
    v = int(vt) if vt is not None else v
    print(f'failing chain: {K} ({chains[K]["clock"]}, {l} flops, scan_in {chains[K]["scan_in"]})')
    print(f'unloaded values on this chain: {n0} zeros, {n1} ones; flops near scan_in always read '
          f'{vt if vt is not None else "mixed"} -> ' +
          (f'stuck-at-{v} on the scan path' if v is not None else 'not a stuck-at (timing / intermittent?)'))
    if v is None:
        return

    # 2. bound: a flop that unloads the non-stuck value is downstream of (after) the defect
    nonstuck = str(1 - v)
    ub = l - 1
    for p in T.pats:
        for t in range(T.L):
            j = l - 1 - t
            if 0 <= j < l and T.msk[p, K][t] == '1' and got(p, t) == nonstuck:
                ub = min(ub, j - 1)
    print(f'upper bound from the unload: defect at position <= {ub} '
          f'(position 0 is next to scan_in; {fl[ub] if ub >= 0 else "-"})')

    # 3. simulate "Q of flop c stuck at v" for c = ub .. 0, incrementally
    fv = mask if v else 0
    base = pattern_inputs(T, M)                 # fault-free loads
    pos = {f: i for i, f in enumerate(fl)}
    cur = dict(base)
    for f in fl[ub + 1:]:                       # flops behind the first candidate: load corrupted
        if f in M.ppi:
            cur[M.ppi[f]] = fv
    vals_c = M.simulate(cur, mask)
    # observed value per (flop, pattern bit) for every compared unload bit, all chains
    obs_val, obs_msk = defaultdict(int), defaultdict(int)
    for i, p in enumerate(T.pats):
        for k in range(T.nch):
            for t in range(T.L):
                if T.msk[p, k][t] != '1':
                    continue
                f = T.flop_at(k, t)
                g = T.fail_so.get((p, k, t), T.unl[p, k][t])
                obs_msk[f] |= 1 << i
                if g == '1':
                    obs_val[f] |= 1 << i
    capmask = defaultdict(int)
    for i, p in enumerate(T.pats):
        for c in T.cap[p]:
            capmask[c] |= 1 << i

    def predicted(f, vals, c):
        """unload value of flop f under 'position c stuck', as a pattern bitmask"""
        cm = capmask[domain[f]]
        q = vals[M.ppi[f]] if f in M.ppi else 0
        d = vals[M.ppo[f]] if f in M.ppo else q
        cap = (d & cm) | (q & ~cm & mask)       # captured value, or the loaded one if not clocked
        if f in pos and pos[f] <= c:
            return fv                           # shifted out through the stuck flop
        return cap

    def score(vals, c):
        s = 0
        for f, m in obs_msk.items():
            s += bin((predicted(f, vals, c) ^ obs_val[f]) & m).count('1')
        return s

    res = []
    for c in range(ub, -1, -1):
        f = fl[c]
        if f in M.ppi and cur[M.ppi[f]] != fv:
            ch = M.propagate(vals_c, mask, {M.ppi[f]: fv})
            vals_c.update(ch)
            cur[M.ppi[f]] = fv
        res.append((score(vals_c, c), c))
    res.sort()
    print(f'simulated {len(res)} candidate positions ({ub} .. 0)')
    for s, c in res[:args.top]:
        print(f'  position {c:5}  {fl[c]:55} miscompares {s}' + ('  <- perfect match' if s == 0 else ''))
    if args.expect:
        name = args.expect.rsplit('=', 1)[0]
        hit = [i + 1 for i, (s, c) in enumerate(res) if fl[c] == name]
        print(f'injected defect at {name} (position {pos.get(name)}): ' +
              (f'rank {hit[0]}, miscompares {res[hit[0] - 1][0]}' if hit else 'outside the candidate range'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['logic', 'chain'])
    ap.add_argument('--tb', required=True, help='gen_scan_tb.py output directory (applied patterns)')
    ap.add_argument('--faillog', required=True)
    ap.add_argument('--model', required=True, help='netlist2bench.py model prefix (all flops observed)')
    ap.add_argument('--chains', required=True)
    ap.add_argument('--first', type=int, required=True)
    ap.add_argument('--last', type=int, required=True)
    ap.add_argument('--top', type=int, default=10)
    ap.add_argument('--expect', help='NAME=V of the injected defect, only to report its rank at the end')
    args = ap.parse_args()
    (cmd_logic if args.mode == 'logic' else cmd_chain)(args)


if __name__ == '__main__':
    main()
