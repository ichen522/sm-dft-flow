#!/usr/bin/env python3
"""
verify_bench.py  -  check that netlist2bench.py's .bench computes the same
logic as the gate-level netlist it came from.

Both sides get the same random values on every PI and every flop Q (PPI),
64 patterns at a time (bit-parallel ints). The netlist side evaluates every
cell straight from its Liberty function (no decomposition, no constant
folding); the .bench side evaluates the primitive gates. Every PO and every
flop D (PPO) must match on every pattern.

Usage:
  python3 scripts/verify_bench.py --netlist pnr/sm_top/sm_top_pnr.v \
      --lib $PDK/gscl45nm.lib --top sm_top --bench atpg/sm_top \
      --const scan_en=0 --const tile_rst_req=0 --const rst_n_mem=1 --rounds 4
"""

import argparse
import importlib.util
import os
import random
import re
import sys
from collections import defaultdict, deque

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location('scan_insert', os.path.join(HERE, 'scan_insert.py'))
si = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(si)
_spec2 = importlib.util.spec_from_file_location('netlist2bench', os.path.join(HERE, 'netlist2bench.py'))
nb = importlib.util.module_from_spec(_spec2)
_spec2.loader.exec_module(nb)

W = 64
MASK = (1 << W) - 1


def ev(node, env):
    op = node[0]
    if op == 'var':
        return env[node[1]]
    if op == 'const':
        return MASK if node[1] else 0
    if op == 'not':
        return ~ev(node[1], env) & MASK
    a, b = ev(node[1], env), ev(node[2], env)
    return a & b if op == 'and' else a | b if op == 'or' else a ^ b


def topo(drivers, deps):
    """drivers: out -> item; deps: out -> input nets. Returns outs in evaluation order."""
    indeg, users = {}, defaultdict(list)
    for o, ins in deps.items():
        d = 0
        for i in set(ins):
            if i in deps:
                d += 1
                users[i].append(o)
        indeg[o] = d
    q = deque(o for o, d in indeg.items() if d == 0)
    order = []
    while q:
        o = q.popleft()
        order.append(o)
        for u in users[o]:
            indeg[u] -= 1
            if indeg[u] == 0:
                q.append(u)
    if len(order) != len(deps):
        sys.exit(f'error: combinational loop ({len(deps) - len(order)} nets)')
    return order


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--netlist', required=True)
    ap.add_argument('--lib', required=True)
    ap.add_argument('--top', required=True)
    ap.add_argument('--bench', required=True, help='prefix used for netlist2bench.py --out')
    ap.add_argument('--const', action='append', default=[])
    ap.add_argument('--rounds', type=int, default=4, help='rounds of 64 random patterns')
    ap.add_argument('--seed', type=int, default=1)
    args = ap.parse_args()

    # ---------------- netlist side -------------------------------------------------
    cells = si.parse_liberty(args.lib)
    ffs = si.lib_ff_info(cells)
    modules = si.split_statements(si.tokenize(open(args.netlist).read()))
    top = next(m for m in modules if m['stmts'][0][1][1].strip() == args.top)
    port_dir, port_bits, alias, insts = {}, {}, {}, []
    for st in top['stmts'][1:]:
        k0, v0 = st[0]
        if k0 == 'id' and v0 in ('input', 'output'):
            rng, j = None, 1
            if st[j] == ('sym', '['):
                close = next(t for t in range(j, len(st)) if st[t] == ('sym', ']'))
                rng = si.expr_text(st[j:close + 1])
                j = close + 1
            for k, v in st[j:]:
                if k in ('id', 'esc'):
                    nm = v.strip() if k == 'id' else v
                    port_dir[nm] = v0
                    port_bits[nm] = nb.bits_of(nm, rng)
        elif st[0] == ('id', 'assign'):
            eq = next(t for t in range(len(st)) if st[t] == ('sym', '='))
            alias[si.net_key(st[1:eq])] = si.net_key(st[eq + 1:-1])
        else:
            p = si.parse_instance(st)
            if p:
                insts.append(p)

    def res(n):
        while n in alias:
            n = alias[n]
        return n

    cell_out = {}            # net -> (func ast, {pin: net})
    deps = {}
    flop_q, flop_d = {}, {}
    for cell, params, name, pins in insts:
        pmap = {p: si.net_key(e) for p, e in pins}
        if cell in ffs:
            info = ffs[cell]
            flop_q[name.strip()] = res(pmap[info['q']])
            if pmap.get(info['d']):
                flop_d[name.strip()] = res(pmap[info['d']])
            continue
        c = cells[cell]
        for o, pi in c['pins'].items():
            if pi['dir'] == 'output' and pi['func'] and pmap.get(o):
                ast = si.BoolExpr(pi['func']).ast
                inmap = {p: res(n) for p, n in pmap.items() if c['pins'].get(p, {}).get('dir') == 'input'}
                cell_out[res(pmap[o])] = (ast, inmap)
                deps[res(pmap[o])] = list(inmap.values())
    order_n = topo(cell_out, deps)

    consts = {}
    for h in args.const:
        p, v = h.split('=')
        for b in port_bits[p]:
            consts[b] = MASK if int(v) else 0

    # ---------------- bench side ----------------------------------------------------------
    pi_label, po_label = defaultdict(list), defaultdict(list)
    for ln in open(args.bench + '.map'):
        f = ln.split()
        if len(f) >= 3 and f[1] in ('PI', 'PPI', 'PO', 'PPO'):
            (pi_label if f[1] in ('PI', 'PPI') else po_label)[f[0]].append((f[1], f[2]))
    bgates, bdeps, bins, bouts = {}, {}, [], []
    for ln in open(args.bench + '.bench'):
        ln = ln.strip()
        if not ln or ln.startswith('#'):
            continue
        m = re.match(r'INPUT\((\w+)\)', ln)
        if m:
            bins.append(m.group(1))
            continue
        m = re.match(r'OUTPUT\((\w+)\)', ln)
        if m:
            bouts.append(m.group(1))
            continue
        m = re.match(r'(\w+)\s*=\s*(\w+)\((.*)\)', ln)
        o, t, ins = m.group(1), m.group(2), [x.strip() for x in m.group(3).split(',')]
        bgates[o] = (t, ins)
        bdeps[o] = ins
    order_b = topo(bgates, bdeps)

    def bev(t, vals):
        if t in ('AND', 'NAND'):
            r = MASK
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
        else:                       # BUFF / NOT
            r = vals[0]
        return (~r & MASK) if t in ('NAND', 'NOR', 'XNOR', 'NOT') else r

    rng = random.Random(args.seed)
    checked = mism = 0
    for rnd in range(args.rounds):
        # shared stimulus, keyed by original name
        stim = {}
        for p, d in port_dir.items():
            if d == 'input':
                for b in port_bits[p]:
                    stim[('PI', b)] = consts.get(b, rng.getrandbits(W))
        for f, q in flop_q.items():
            stim[('PPI', f + '/Q')] = rng.getrandbits(W)
        # netlist
        val = {'CONST0': 0, 'CONST1': MASK}
        for p, d in port_dir.items():
            if d == 'input':
                for b in port_bits[p]:
                    val[b] = stim[('PI', b)]
        for f, q in flop_q.items():
            val[q] = stim[('PPI', f + '/Q')]
        for k in list(alias):
            if alias[k].startswith('CONST:'):
                val[res(k)] = MASK if nb.const_of(alias[k]) == 'CONST1' else 0
        for o in order_n:
            ast, inmap = cell_out[o]
            env = {}
            for p, n in inmap.items():
                if n.startswith('CONST:'):
                    env[p] = MASK if nb.const_of(n) == 'CONST1' else 0
                else:
                    env[p] = val[n]
            val[o] = ev(ast, env)
        # bench
        bval = {}
        for n in bins:
            kind, lbl = pi_label[n][0]
            bval[n] = stim[(kind, lbl)]
        for o in order_b:
            t, ins = bgates[o]
            bval[o] = bev(t, [bval[i] for i in ins])
        # compare every label of every bench output
        for n in bouts:
            for kind, lbl in po_label[n]:
                ref = val[res(lbl)] if kind == 'PO' else val[flop_d[lbl[:-2]]]
                checked += 1
                if bval[n] != ref:
                    mism += 1
                    if mism <= 10:
                        print(f'MISMATCH {kind} {lbl}: bench {bval[n]:016x} netlist {ref:016x}')
        print(f'round {rnd + 1}: {checked} output checks so far, {mism} mismatches')
    print(f'{"PASS" if mism == 0 else "FAIL"}: {checked} output x 64-pattern checks, {mism} mismatches, '
          f'{len(bins)} inputs, {len(bgates)} bench gates, {len(order_n)} netlist cell outputs')
    sys.exit(1 if mism else 0)


if __name__ == '__main__':
    main()
