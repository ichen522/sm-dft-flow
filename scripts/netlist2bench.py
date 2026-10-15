#!/usr/bin/env python3
"""
netlist2bench.py  -  full-scan gate netlist -> combinational ISCAS .bench for ATPG

Why
---
This server has neither TetraMAX nor Modus. With full scan every flop can be
loaded and unloaded through the chains, so stuck-at ATPG only has to deal
with the combinational logic between flops (what TetraMAX does internally):

    flop Q  -> INPUT  (pseudo primary input,  PPI)
    flop D  -> OUTPUT (pseudo primary output, PPO)

and any combinational ATPG (Atalanta here, or the EE658 tools) can run on it.

What it does
------------
1. Reads the flat netlist (default: the post-route one) and gscl45nm.lib,
   reusing the parsers of scan_insert.py.
2. Every combinational cell is decomposed into AND / OR / NAND / NOR / XOR /
   XNOR / NOT / BUFF from its Liberty 'function' (AOI22, FAX1, MUX2X1 ...).
3. Pins held constant in capture mode (--const scan_en=0 tile_rst_req=0
   rst_n_mem=1, plus 1'b0 / 1'b1 in the netlist) are propagated through the
   gates, so the scan muxes reduce to their functional input.
4. Logic that reaches no output / PPO is dropped and counted (clock trees,
   the scan_in side of the muxes, unused flop outputs). Nets with a fanout
   above Atalanta's MAXFOUT get a buffer tree.
5. Writes:
     <out>.bench     the circuit, nets renamed N1, N2, ... (Atalanta-safe)
     <out>.map       N<id> <-> original net / port / flop name
     <out>.names     N<id> internal net name, cell instance and cell it came from
     <out>.json      statistics (inputs, outputs, gates, what was removed)

Usage
-----
  python3 scripts/netlist2bench.py --netlist pnr/sm_top/sm_top_pnr.v \
      --lib $PDK/gscl45nm.lib --top sm_top --out atpg/sm_top \
      --const scan_en=0 --const tile_rst_req=0 --const rst_n_mem=1

Fault model note: faults are placed by the ATPG tool on the lines of the
decomposed circuit, i.e. on cell pins plus the internal nodes of complex
cells. Cell-pin faults (TetraMAX's model) are a subset, so the numbers are
close to, but not identical with, a commercial tool's.
"""

import argparse
import importlib.util
import json
import os
import re
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location('scan_insert', os.path.join(HERE, 'scan_insert.py'))
si = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(si)

MAXFOUT = 400           # Atalanta's MAXFOUT is 500; stay below it
C0, C1 = 'CONST0', 'CONST1'


def bits_of(name, rng):
    """port name + '[ 7 : 0 ]' -> ['name[7]', ..., 'name[0]']"""
    if not rng:
        return [name]
    m = re.match(r'\[\s*(-?\d+)\s*:\s*(-?\d+)\s*\]', rng)
    hi, lo = int(m.group(1)), int(m.group(2))
    step = -1 if hi >= lo else 1
    return [f'{name}[{i}]' for i in range(hi, lo + step, step)]


def const_of(key):
    """'CONST:1'b0' -> CONST0 ..."""
    v = key.split(':', 1)[1].lower().replace('_', '')
    m = re.match(r"\d*'[sS]?([bdh])(\w+)", v)
    if m:
        val = int(m.group(2), {'b': 2, 'd': 10, 'h': 16}[m.group(1)])
    else:
        val = int(v)
    return C1 if val & 1 else C0


class Circuit:
    def __init__(self):
        self.gates = {}          # out net -> [type, [in nets]]
        self.tmp = 0
        self.owner = {}          # out net -> (instance, cell) the gate came from
        self.cur = None

    def new(self):
        self.tmp += 1
        return f'_t{self.tmp}'

    def add(self, typ, ins, out=None):
        out = out or self.new()
        if out in self.gates:
            sys.exit(f'error: net {out} has two drivers')
        self.gates[out] = [typ, list(ins)]
        self.owner[out] = self.cur
        return out

    # ---- Liberty function AST -> primitive gates -------------------------
    def emit(self, node, pinnet, out=None):
        op = node[0]
        if op == 'var':
            n = pinnet(node[1])
            return self.add('BUFF', [n], out) if out else n
        if op == 'const':
            n = C1 if node[1] else C0
            return self.add('BUFF', [n], out) if out else n
        if op == 'not':
            ch = node[1]
            if ch[0] in ('and', 'or', 'xor'):
                typ = {'and': 'NAND', 'or': 'NOR', 'xor': 'XNOR'}[ch[0]]
                return self.add(typ, [self.emit(c, pinnet) for c in self.flat(ch)], out)
            return self.add('NOT', [self.emit(ch, pinnet)], out)
        typ = {'and': 'AND', 'or': 'OR', 'xor': 'XOR'}[op]
        return self.add(typ, [self.emit(c, pinnet) for c in self.flat(node)], out)

    @staticmethod
    def flat(node):
        """(and (and a b) c) -> [a, b, c]"""
        op, out = node[0], []
        stack = [node]
        while stack:
            n = stack.pop()
            if n[0] == op:
                stack.extend(reversed(n[1:]))
            else:
                out.append(n)
        return out

    def split_xor(self):
        """Atalanta only takes 2-input XOR / XNOR: a ^ b ^ c -> (a ^ b) ^ c"""
        n = 0
        for o in list(self.gates):
            t, ins = self.gates[o]
            if t in ('XOR', 'XNOR') and len(ins) > 2:
                self.cur = self.owner.get(o)
                acc = ins[0]
                for k, i in enumerate(ins[1:-1]):
                    acc = self.add('XOR', [acc, i], out=f'{o}~x{k}')   # name from o: same in every model
                    n += 1
                self.gates[o] = [t, [acc, ins[-1]]]
        return n

    # ---- constant propagation -----------------------------------------------
    def propagate(self, consts):
        """consts: {net: 0/1} known constants (ports, tie values). Simplifies gates in place."""
        val = dict(consts)
        val[C0], val[C1] = 0, 1
        fanout = defaultdict(list)
        for o, (t, ins) in self.gates.items():
            for i in ins:
                fanout[i].append(o)
        work = [o for o, (t, ins) in self.gates.items() if any(i in val for i in ins)]
        seen_work = set(work)
        while work:
            o = work.pop()
            seen_work.discard(o)
            if o not in self.gates:
                continue
            t, ins = self.gates[o]
            res = self.simplify(t, ins, val)
            if res is None:
                continue
            kind, payload = res
            if kind == 'const':
                del self.gates[o]
                val[o] = payload
                for f in fanout[o]:
                    if f not in seen_work:
                        seen_work.add(f)
                        work.append(f)
            else:
                self.gates[o] = list(payload)
        return {n: v for n, v in val.items() if n not in (C0, C1)}

    @staticmethod
    def simplify(t, ins, val):
        known = [val[i] for i in ins if i in val]
        if not known:
            return None
        free = [i for i in ins if i not in val]
        base, inv = {'AND': ('AND', 0), 'NAND': ('AND', 1), 'OR': ('OR', 0), 'NOR': ('OR', 1),
                     'XOR': ('XOR', 0), 'XNOR': ('XOR', 1), 'NOT': ('BUF', 1), 'BUFF': ('BUF', 0)}[t]
        if base == 'BUF':
            return ('const', known[0] ^ inv)
        if base in ('AND', 'OR'):
            ctrl = 0 if base == 'AND' else 1
            if ctrl in known:
                return ('const', ctrl ^ inv)
            if not free:
                return ('const', (1 - ctrl) ^ inv)
            if len(free) == 1:
                return ('gate', ('NOT' if inv else 'BUFF', free))
            return ('gate', (t, free))
        par = (sum(known) & 1) ^ inv            # XOR / XNOR
        if not free:
            return ('const', par)
        if len(free) == 1:
            return ('gate', ('NOT' if par else 'BUFF', free))
        return ('gate', ('XNOR' if par else 'XOR', free))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--netlist', required=True)
    ap.add_argument('--lib', required=True)
    ap.add_argument('--top', required=True)
    ap.add_argument('--out', required=True, help='output prefix (writes .bench .map .json)')
    ap.add_argument('--const', action='append', default=[], help='PORT=0|1 held during capture')
    ap.add_argument('--chains', help='scan/chains.json (clock domain of every flop), needed for --capture')
    ap.add_argument('--capture', help='one-hot capture: only flops of this clock are pulsed, so only '
                    'their D is observed (the other domain holds its loaded value)')
    args = ap.parse_args()
    domain_of = {}
    if args.capture:
        if not args.chains:
            sys.exit('error: --capture needs --chains')
        for c in json.load(open(args.chains))['chains']:
            for f in c['flops']:
                domain_of[f] = c['clock']

    cells = si.parse_liberty(args.lib)
    ffs = si.lib_ff_info(cells)
    modules = si.split_statements(si.tokenize(open(args.netlist).read()))
    top = next((m for m in modules if m['stmts'][0][1][1].strip() == args.top), None)
    if top is None:
        sys.exit(f'error: module {args.top} not found')

    # ---- ports, assigns, instances --------------------------------------------
    port_dir, port_bits = {}, {}
    alias = {}
    insts = []
    for st in top['stmts'][1:]:
        k0, v0 = st[0]
        if k0 == 'id' and v0 in ('input', 'output', 'inout'):
            rng, j = None, 1
            if st[j] == ('sym', '['):
                close = next(t for t in range(j, len(st)) if st[t] == ('sym', ']'))
                rng = si.expr_text(st[j:close + 1])
                j = close + 1
            for k, v in st[j:]:
                if k in ('id', 'esc'):
                    nm = v.strip() if k == 'id' else v
                    port_dir[nm] = v0
                    port_bits[nm] = bits_of(nm, rng)
            continue
        if k0 == 'id' and v0 in ('wire', 'reg', 'tri', 'supply0', 'supply1', 'parameter', 'localparam'):
            continue
        if st[0] == ('id', 'assign'):
            eq = next(t for t in range(len(st)) if st[t] == ('sym', '='))
            lhs, rhs = si.net_key(st[1:eq]), si.net_key(st[eq + 1:-1])
            if not (lhs and rhs):
                sys.exit(f'error: unsupported assign: {si.expr_text(st)}')
            alias[lhs] = rhs
            continue
        p = si.parse_instance(st)
        if p:
            insts.append(p)

    def res(n):
        seen = set()
        while n in alias and n not in seen:
            seen.add(n)
            n = alias[n]
        if n.startswith('CONST:'):
            return const_of(n)
        return n

    # ---- cells -> primitives ------------------------------------------------------
    ckt = Circuit()
    ppi, ppo = [], []                           # (net, flop name)
    clock_pins = set()
    async_nets = []
    skipped = defaultdict(int)
    for cell, params, name, pins in insts:
        nm = name.strip()
        pmap = {p: si.net_key(e) for p, e in pins}
        if cell in ffs:
            info = ffs[cell]
            q, d = pmap.get(info['q']), pmap.get(info['d'])
            if q:
                ppi.append((res(q), nm))
            if d:
                ppo.append((res(d), nm))
            if pmap.get(info['clk']):
                clock_pins.add(res(pmap[info['clk']]))
            for a in info['async']:
                if pmap.get(a):
                    async_nets.append((res(pmap[a]), nm, a))
            continue
        c = cells.get(cell)
        if c is None:
            sys.exit(f'error: cell {cell} ({nm}) is not in the Liberty file')
        outs = [p for p, pi in c['pins'].items() if pi['dir'] == 'output' and pi['func']]
        if not outs:
            skipped[cell] += 1                  # FILL and other physical-only cells
            continue
        if c['latch']:
            sys.exit(f'error: latch {nm} ({cell}) - not supported')

        def pinnet(pin, pmap=pmap, nm=nm):
            n = pmap.get(pin)
            if n is None:
                sys.exit(f'error: {nm}: input pin {pin} unconnected')
            return res(n)

        ckt.cur = (nm, cell)
        for o in outs:
            on = pmap.get(o)
            if not on:
                continue                        # output left open
            ckt.emit(si.BoolExpr(c['pins'][o]['func']).ast, pinnet, out=res(on))

    # ---- constants ---------------------------------------------------------------
    consts = {}
    for h in args.const:
        p, v = h.split('=')
        if p not in port_bits:
            sys.exit(f'error: --const port {p} not found')
        for b in port_bits[p]:
            consts[b] = int(v)
    n_gates_raw = len(ckt.gates)
    const_nets = ckt.propagate(consts)
    ckt.split_xor()
    ckt.cur = None                              # gates added from here on are fanout buffers

    def cval(n):
        return const_nets.get(n, {C0: 0, C1: 1}.get(n))

    # ---- inputs / observation points ------------------------------------------------
    pis = []
    for p, d in port_dir.items():
        if d == 'input':
            for b in port_bits[p]:
                if b not in consts:
                    pis.append((b, f'PI {b}'))
    pis += [(n, f'PPI {f}/Q') for n, f in ppi]
    obs = []
    const_obs = []
    for p, d in port_dir.items():
        if d == 'output':
            for b in port_bits[p]:
                n = res(b)
                (const_obs if cval(n) is not None else obs).append((n, f'PO {b}'))
    not_captured = 0
    for n, f in ppo:
        if args.capture and domain_of.get(f) != args.capture:
            not_captured += 1                   # this flop is not clocked in this capture
            continue
        (const_obs if cval(n) is not None else obs).append((n, f'PPO {f}/D'))

    async_live = [(n, f, a) for n, f, a in async_nets if cval(n) is None]

    # ---- keep only the fan-in cones of the observation points -----------------------
    keep, stack = set(), [n for n, _ in obs]
    while stack:
        n = stack.pop()
        if n in keep:
            continue
        keep.add(n)
        if n in ckt.gates:
            stack.extend(ckt.gates[n][1])
    dropped = [o for o in ckt.gates if o not in keep]
    dropped_clock = 0
    fo_all = defaultdict(int)
    for o, (t, ins) in ckt.gates.items():
        for i in ins:
            fo_all[i] += 1
    for o in dropped:
        del ckt.gates[o]
    pi_nets = {n for n, _ in pis}
    used_pis = [(n, lbl) for n, lbl in pis if n in keep]
    unused_pis = [lbl for n, lbl in pis if n not in keep]
    undriven = sorted(n for n in keep if n not in ckt.gates and n not in pi_nets and cval(n) is None)
    if undriven:
        sys.exit(f'error: {len(undriven)} nets have no driver, e.g. {undriven[:5]}')

    # ---- fanout limit -----------------------------------------------------------------
    consumers = defaultdict(list)
    for o, (t, ins) in ckt.gates.items():
        for k, i in enumerate(ins):
            consumers[i].append((o, k))
    nbuf = 0
    for n, lst in list(consumers.items()):
        while len(lst) > MAXFOUT:
            groups = [lst[i:i + MAXFOUT] for i in range(0, len(lst), MAXFOUT)]
            new = []
            for g in groups:
                b = ckt.add('BUFF', [n], out=f'{n}~fo{nbuf}')
                nbuf += 1
                for o, k in g:
                    ckt.gates[o][1][k] = b
                new.append((b, 0))
            lst = new

    # ---- names and output -------------------------------------------------------------
    ids = {}

    def nid(n):
        if n not in ids:
            ids[n] = f'N{len(ids) + 1}'
        return ids[n]

    lines, mapping = [], []
    for n, lbl in used_pis:
        lines.append(f'INPUT({nid(n)})')
        mapping.append(f'{nid(n)} {lbl}')
    po_seen = {}
    po_lines = []
    for n, lbl in obs:
        if n in po_seen:
            mapping.append(f'{po_seen[n]} {lbl}  (same net as above)')
            continue
        tgt = n
        if n in pi_nets:                        # an output straight from an input: give it its own line
            tgt = ckt.add('BUFF', [n], out=f'{n}~po')
            nbuf += 1
        po_seen[n] = nid(tgt)
        po_lines.append(f'OUTPUT({nid(tgt)})')
        mapping.append(f'{nid(tgt)} {lbl}')
    lines += po_lines
    for o, (t, ins) in ckt.gates.items():
        lines.append(f'{nid(o)} = {t}({", ".join(nid(i) for i in ins)})')
    for n in ckt.gates:
        if n not in ids:
            pass
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out + '.bench', 'w') as f:
        f.write(f'# {args.top}: combinational ATPG model of {args.netlist} (netlist2bench.py)\n')
        f.write(f'# constants: {" ".join(args.const)}\n')
        f.write('\n'.join(lines) + '\n')
    with open(args.out + '.map', 'w') as f:
        f.write('\n'.join(mapping) + '\n')
        for o in ckt.gates:
            if not o.startswith('_t') and '~' not in o:
                f.write(f'{nid(o)} net {o}\n')
    # every bench node -> internal net name, and the cell instance it came from
    # (for analyze_undetected.py: same netlist -> same internal names in every model)
    with open(args.out + '.names', 'w') as f:
        for n, lbl in used_pis:
            f.write(f'{nid(n)} {n} - {lbl.split()[0]}\n')
        for o in ckt.gates:
            inst, cell = ckt.owner.get(o) or ('-', 'added_buffer')
            f.write(f'{nid(o)} {o} {inst} {cell}\n')

    gtypes = defaultdict(int)
    for t, _ in ckt.gates.values():
        gtypes[t] += 1
    stats = {
        'netlist': args.netlist, 'constants': args.const,
        'cell_instances': len(insts), 'flops': len(ppi), 'physical_cells_skipped': dict(skipped),
        'primitive_gates_before_const': n_gates_raw,
        'primitive_gates': len(ckt.gates), 'gate_types': dict(gtypes),
        'unobservable_gates_removed': len(dropped),
        'fanout_buffers_added': nbuf,
        'inputs': len(used_pis), 'inputs_PI': sum(1 for _, l in used_pis if l.startswith('PI ')),
        'inputs_PPI': sum(1 for _, l in used_pis if l.startswith('PPI ')),
        'outputs': len(po_lines), 'observation_points': len(obs),
        'constant_observation_points': [l for _, l in const_obs],
        'unused_inputs': unused_pis,
        'async_set_reset_pins_not_constant': [f'{f}/{a}' for _, f, a in async_live],
        'capture_clock': args.capture or 'all', 'flops_not_captured': not_captured,
    }
    with open(args.out + '.json', 'w') as f:
        json.dump(stats, f, indent=1)

    print(f'cells {len(insts)}  flops {len(ppi)}  primitive gates {n_gates_raw} -> {len(ckt.gates)} '
          f'after constants, {len(dropped)} unobservable removed, {nbuf} fanout buffers')
    print(f'inputs {len(used_pis)} (PI {stats["inputs_PI"]}, PPI {stats["inputs_PPI"]}), '
          f'outputs {len(po_lines)} ({len(obs)} observation points), '
          f'{len(const_obs)} constant observation points, {len(unused_pis)} unused inputs')
    if async_live:
        print(f'WARNING: {len(async_live)} async set/reset pins are not constant in capture '
              f'(not modelled), e.g. {async_live[:3]}')
    if args.capture:
        print(f'one-hot capture on {args.capture}: {not_captured} flops of the other domain(s) not observed')
    print(f'wrote {args.out}.bench .map .names .json')


if __name__ == '__main__':
    main()
