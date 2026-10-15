#!/usr/bin/env python3
"""
scan_insert.py  -  mux-D full-scan insertion on a flat gate-level netlist

Why this script exists
----------------------
The OSU FreePDK45 library (gscl45nm) has no scan flip-flop, so Design
Compiler's insert_dft cannot swap DFFs for scan cells. A mux-D scan cell is
just "2:1 mux in front of D", so this script builds one from library cells:

    scan_en=0 :  D_ff = D_func          (normal operation)
    scan_en=1 :  D_ff = scan_in / previous flop's Q   (shift)

gscl45nm's MUX2X1 is inverting (Y = !(S ? A : B)), so each scan cell is
MUX2X1 + INVX1 + the original flop. The script reads the Liberty file to find
the flops, the mux's select/data pins and its polarity, so nothing about the
library is hard-coded except the cell names given on the command line.

What it does
------------
1. Reads the flat netlist written by DC (scripts/01_syn.tcl, after
   ungroup -all -flatten) and the Liberty file.
2. Finds every flop, traces its clock pin back to a primary input through
   buffers / inverters, and groups flops by (clock port, edge).
   Each group gets its own chains, so a chain never crosses clock domains
   (no lockup latches needed).
3. Splits the requested number of chains over the groups in proportion to
   their size, orders flops by name inside a group (keeps a module's flops
   together), and stitches: scan_in[k] -> mux -> flop -> mux -> ... -> BUFX2
   -> scan_out[k].
4. Writes the scan netlist plus everything the later steps need:
     chains.txt / chains.json   chain map (domain, length, first/last flop)
     scan_func.sdc              scan ports in functional mode (scan_en = 0)
     scan_shift.sdc             scan ports in shift mode      (scan_en = 1)
     tmax_scan.tcl              TetraMAX scan / clock / constraint setup
     modus.pinassign            Modus pin assignment file
     lec_scan.do                Conformal: pre-scan vs scan netlist
     tb_flush.v                 gate-level chain flush test

Usage (on the server, plain python3, no packages needed)
------------------------------------------------------
  python3 scripts/scan_insert.py \
      --netlist netlist/sm_top_syn.v --lib $PDK/gscl45nm.lib \
      --top sm_top --chains 16 --out netlist/sm_top_scan.v --outdir scan \
      --hold rst_n_mem=1 --hold tile_rst_req=0
"""

import argparse
import json
import math
import os
import re
import sys
from collections import OrderedDict, defaultdict

# ---------------------------------------------------------------------------
# Liberty (only what we need: cell pins, directions, functions, ff groups)
# ---------------------------------------------------------------------------

def strip_comments(text):
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)
    text = re.sub(r'//[^\n]*', ' ', text)
    return text


def parse_liberty(path):
    """Return {cell: {'pins': {pin: {'dir':..,'func':..}}, 'ff': {...}|None, 'latch': bool}}"""
    text = strip_comments(open(path).read())
    cells = {}
    # walk the group tree with a brace counter
    tok = re.compile(r'([A-Za-z_][\w]*)\s*\(([^)]*)\)\s*\{|\}|([A-Za-z_][\w]*)\s*:\s*("([^"]*)"|[^;]*);')
    stack = []
    cur_cell = cur_pin = cur_ff = None
    for m in tok.finditer(text):
        if m.group(1):                       # group open: name(args) {
            gname, garg = m.group(1), m.group(2).strip().strip('"')
            stack.append((gname, garg))
            if gname == 'cell' and len(stack) == 2:
                cur_cell = cells.setdefault(garg, {'pins': OrderedDict(), 'ff': None, 'latch': False})
            elif cur_cell is not None and gname == 'pin':
                cur_pin = cur_cell['pins'].setdefault(garg, {'dir': None, 'func': None})
            elif cur_cell is not None and gname in ('ff', 'ff_bank'):
                names = [s.strip() for s in garg.split(',')]
                cur_ff = cur_cell['ff'] = {'vars': names}
            elif cur_cell is not None and gname in ('latch', 'latch_bank'):
                cur_cell['latch'] = True
        elif m.group(0) == '}':
            if stack:
                gname, garg = stack.pop()
                if gname == 'cell':
                    cur_cell = None
                elif gname == 'pin':
                    cur_pin = None
                elif gname in ('ff', 'ff_bank'):
                    cur_ff = None
        else:                                 # attribute: name : value ;
            aname = m.group(3)
            aval = m.group(5) if m.group(5) is not None else m.group(4).strip().strip('"')
            if cur_ff is not None and stack and stack[-1][0] in ('ff', 'ff_bank'):
                cur_ff[aname] = aval
            elif cur_pin is not None and stack and stack[-1][0] == 'pin':
                if aname == 'direction':
                    cur_pin['dir'] = aval
                elif aname == 'function':
                    cur_pin['func'] = aval
    return cells


class BoolExpr:
    """Tiny evaluator for Liberty boolean functions: ! ' + | * & ^ space ( ) 0 1"""

    def __init__(self, s):
        self.toks = re.findall(r"[A-Za-z_][\w\[\]\.]*|[!'+|*&^()01]", s)
        self.i = 0
        self.ast = self._or()

    def _peek(self):
        return self.toks[self.i] if self.i < len(self.toks) else None

    def _take(self):
        t = self._peek()
        self.i += 1
        return t

    def _or(self):
        node = self._xor()
        while self._peek() in ('+', '|'):
            self._take()
            node = ('or', node, self._xor())
        return node

    def _xor(self):
        node = self._and()
        while self._peek() == '^':
            self._take()
            node = ('xor', node, self._and())
        return node

    def _and(self):
        node = self._unary()
        while True:
            p = self._peek()
            if p in ('*', '&'):
                self._take()
                node = ('and', node, self._unary())
            elif p is not None and (p in ('!', '(', '0', '1') or re.match(r'[A-Za-z_]', p)):
                node = ('and', node, self._unary())      # implicit AND (space)
            else:
                return node

    def _unary(self):
        if self._peek() == '!':
            self._take()
            return ('not', self._unary())
        node = self._atom()
        while self._peek() == "'":
            self._take()
            node = ('not', node)
        return node

    def _atom(self):
        t = self._take()
        if t == '(':
            node = self._or()
            if self._take() != ')':
                raise ValueError('unbalanced parentheses in Liberty function')
            return node
        if t in ('0', '1'):
            return ('const', int(t))
        return ('var', t)

    def eval(self, env, node=None):
        node = self.ast if node is None else node
        op = node[0]
        if op == 'var':
            return env[node[1]]
        if op == 'const':
            return node[1]
        if op == 'not':
            return 1 - self.eval(env, node[1])
        a, b = self.eval(env, node[1]), self.eval(env, node[2])
        return {'and': a & b, 'or': a | b, 'xor': a ^ b}[op]

    def vars(self, node=None):
        node = self.ast if node is None else node
        if node[0] == 'var':
            return {node[1]}
        if node[0] == 'const':
            return set()
        return set().union(*(self.vars(n) for n in node[1:]))


def lib_ff_info(cells):
    """flop cells -> {'clk','d','q','edge'(+1/-1),'async':[pins]}"""
    out = {}
    for name, c in cells.items():
        ff = c['ff']
        if not ff:
            continue
        clk_expr = ff.get('clocked_on', '').strip()
        nxt = ff.get('next_state', '').strip()
        edge = +1
        m = re.fullmatch(r"\(?\s*(!?)\s*([A-Za-z_]\w*)\s*('?)\s*\)?", clk_expr)
        if not m:
            continue
        if m.group(1) or m.group(3):
            edge = -1
        clk = m.group(2)
        m2 = re.fullmatch(r"\(?\s*([A-Za-z_]\w*)\s*\)?", nxt)
        if not m2:
            print(f'warning: {name}: next_state "{nxt}" is not a plain pin, skipped', file=sys.stderr)
            continue
        d = m2.group(1)
        q = None
        for p, pi in c['pins'].items():
            if pi['dir'] == 'output' and pi['func'] and pi['func'].strip().strip('()') in ff['vars'][:1]:
                q = p
        if q is None:
            for p, pi in c['pins'].items():
                if pi['dir'] == 'output':
                    q = p
                    break
        asyncs = []
        for key in ('clear', 'preset'):
            if key in ff:
                asyncs += sorted(BoolExpr(ff[key]).vars())
        out[name] = {'clk': clk, 'd': d, 'q': q, 'edge': edge, 'async': asyncs}
    return out


def lib_mux_info(cells, mux):
    """Find which pin is S, which data pin S=1 selects, and whether Y is inverted."""
    c = cells.get(mux)
    if not c:
        sys.exit(f'error: mux cell {mux} not found in Liberty file')
    ins = [p for p, pi in c['pins'].items() if pi['dir'] == 'input']
    outs = [p for p, pi in c['pins'].items() if pi['dir'] == 'output']
    if len(ins) != 3 or len(outs) != 1:
        sys.exit(f'error: {mux} is not a 2:1 mux (inputs {ins}, outputs {outs})')
    y = outs[0]
    f = BoolExpr(c['pins'][y]['func'])
    for s in ins:
        a, b = [p for p in ins if p != s]
        for sel1, sel0 in ((a, b), (b, a)):
            for inv in (0, 1):
                ok = True
                for va in (0, 1):
                    for vb in (0, 1):
                        for vs in (0, 1):
                            env = {s: vs, a: va, b: vb}
                            want = env[sel1] if vs else env[sel0]
                            if f.eval(env) != (want ^ inv):
                                ok = False
                if ok:
                    return {'s': s, 'in1': sel1, 'in0': sel0, 'y': y, 'inverting': bool(inv)}
    sys.exit(f'error: could not recognise {mux} function "{c["pins"][y]["func"]}" as a 2:1 mux')


def lib_buf_inv(cells):
    """single-input cells that are buffers (+1) or inverters (-1): {cell: (in, out, pol)}"""
    out = {}
    for name, c in cells.items():
        if c['ff'] or c['latch']:
            continue
        ins = [p for p, pi in c['pins'].items() if pi['dir'] == 'input']
        outs = [p for p, pi in c['pins'].items() if pi['dir'] == 'output']
        if len(ins) != 1 or len(outs) != 1 or not c['pins'][outs[0]]['func']:
            continue
        f = BoolExpr(c['pins'][outs[0]]['func'])
        v0, v1 = f.eval({ins[0]: 0}), f.eval({ins[0]: 1})
        if (v0, v1) == (0, 1):
            out[name] = (ins[0], outs[0], +1)
        elif (v0, v1) == (1, 0):
            out[name] = (ins[0], outs[0], -1)
    return out

# ---------------------------------------------------------------------------
# Verilog netlist (structural, flat): statement-level tokenizer
# ---------------------------------------------------------------------------

TOKEN = re.compile(r"""
    (?P<ws>\s+)
  | (?P<lc>//[^\n]*)
  | (?P<bc>/\*.*?\*/)
  | (?P<attr>\(\*.*?\*\))
  | (?P<esc>\\\S+)
  | (?P<num>\d*'[sS]?[bBoOdDhH][0-9a-fA-FxXzZ_?]+|\d+)
  | (?P<id>[A-Za-z_][\w$]*)
  | (?P<str>"[^"]*")
  | (?P<sym>[()\[\]{},;.:=#@+\-*/<>!~&|^?])
""", re.S | re.X)


def tokenize(text):
    toks = []
    pos = 0
    n = len(text)
    while pos < n:
        m = TOKEN.match(text, pos)
        if not m:
            raise SyntaxError(f'cannot tokenize netlist near: {text[pos:pos+60]!r}')
        kind = m.lastgroup
        if kind not in ('ws', 'lc', 'bc', 'attr'):
            val = m.group(kind)
            if kind == 'esc':
                val = val + ' '          # an escaped identifier ends with whitespace
            toks.append((kind, val))
        pos = m.end()
    return toks


def norm_name(tok):
    """identifier text for a net: escaped names keep their backslash + trailing space"""
    return tok


def split_statements(toks):
    """Split the token stream into module blocks; each module is a list of statements
    (lists of tokens). 'endmodule' closes a module."""
    modules = []
    cur = None
    stmt = []
    for kind, val in toks:
        if cur is None:
            if kind == 'id' and val in ('module', 'macromodule'):
                cur = {'stmts': []}
                stmt = [(kind, val)]
            continue
        if kind == 'id' and val == 'endmodule' and not stmt:
            modules.append(cur)
            cur = None
            continue
        stmt.append((kind, val))
        if kind == 'sym' and val == ';':
            cur['stmts'].append(stmt)
            stmt = []
    if cur is not None:
        sys.exit('error: netlist ends inside a module (missing endmodule)')
    return modules


def text_of(tokens):
    """re-emit tokens as text (one space between tokens is always legal Verilog)"""
    out = []
    for kind, val in tokens:
        out.append(val.rstrip() + (' ' if kind == 'esc' else ''))
    s = ' '.join(out)
    return s


def expr_text(tokens):
    s = text_of(tokens).strip()
    # an escaped identifier must stay terminated by whitespace
    return s + ' ' if tokens and tokens[-1][0] == 'esc' else s


def parse_instance(stmt):
    """CELL [#(...)] NAME ( .P(expr), ... ) ;  ->  (cell, params_tokens, name, [(pin, expr_tokens)])
    Returns None if the statement is not a named-port instance."""
    i = 0
    if stmt[0][0] not in ('id', 'esc'):
        return None
    cell = stmt[0][1].strip()
    i = 1
    params = []
    if stmt[i] == ('sym', '#'):
        depth = 0
        j = i + 1
        while True:
            k, v = stmt[j]
            if v == '(':
                depth += 1
            elif v == ')':
                depth -= 1
                if depth == 0:
                    break
            j += 1
        params = stmt[i:j + 1]
        i = j + 1
    if stmt[i][0] not in ('id', 'esc'):
        return None
    name = stmt[i][1]
    i += 1
    if stmt[i] != ('sym', '('):
        return None
    i += 1
    pins = []
    while i < len(stmt):
        k, v = stmt[i]
        if v == ')':
            break
        if v == ',':
            i += 1
            continue
        if v != '.':
            return None                         # positional connections: not handled
        pin = stmt[i + 1][1].strip()
        if stmt[i + 2] != ('sym', '('):
            return None
        depth = 1
        j = i + 3
        while depth:
            kk, vv = stmt[j]
            if vv == '(':
                depth += 1
            elif vv == ')':
                depth -= 1
            j += 1
        expr = stmt[i + 3:j - 1]
        pins.append((pin, expr))
        i = j
    return cell, params, name, pins


def emit_instance(cell, params, name, pins):
    ptxt = (' ' + text_of(params)) if params else ''
    conns = ',\n    '.join(f'.{p}({expr_text(e)})' for p, e in pins)
    return f'  {cell}{ptxt} {name.rstrip()} ( {conns} );'


def net_key(expr):
    """canonical string for a single-net expression (None for concatenations etc.)"""
    if not expr:
        return None
    if len(expr) == 1 and expr[0][0] in ('id', 'esc'):
        return expr[0][1].strip() if expr[0][0] == 'id' else expr[0][1]
    if len(expr) == 4 and expr[0][0] in ('id', 'esc') and expr[1][1] == '[' and expr[2][0] == 'num' and expr[3][1] == ']':
        base = expr[0][1]
        return f'{base.rstrip()}[{expr[2][1]}]' if expr[0][0] == 'id' else f'{base}[{expr[2][1]}]'
    if len(expr) == 1 and expr[0][0] == 'num':
        return 'CONST:' + expr[0][1]
    return None

# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def natural_key(s):
    return [int(t) if t.isdigit() else t for t in re.split(r'(\d+)', s)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--netlist', required=True)
    ap.add_argument('--lib', required=True, help='Liberty file (gscl45nm.lib)')
    ap.add_argument('--top', required=True)
    ap.add_argument('--out', required=True, help='scan netlist to write')
    ap.add_argument('--outdir', default='scan')
    ap.add_argument('--chains', type=int, default=16)
    ap.add_argument('--mux', default='MUX2X1')
    ap.add_argument('--inv', default='INVX1')
    ap.add_argument('--buf', default='BUFX2')
    ap.add_argument('--scan-en', default='scan_en')
    ap.add_argument('--scan-in', default='scan_in')
    ap.add_argument('--scan-out', default='scan_out')
    ap.add_argument('--hold', action='append', default=[],
                    help='PORT=VALUE held during test (async resets), e.g. rst_n_mem=1')
    ap.add_argument('--shift-period', type=float, default=10.0, help='ns, shift-mode clock period for the SDC')
    ap.add_argument('--exclude', action='append', default=[], help='regex of flop instance names to leave out of the chains')
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    cells = parse_liberty(args.lib)
    ffs = lib_ff_info(cells)
    mux = lib_mux_info(cells, args.mux)
    bufinv = lib_buf_inv(cells)
    for c in (args.inv, args.buf):
        if c not in bufinv:
            sys.exit(f'error: {c} is not a buffer/inverter in the Liberty file')
    inv_in, inv_out, _ = bufinv[args.inv]
    buf_in, buf_out, _ = bufinv[args.buf]
    print(f'library: {len(cells)} cells, flops: {", ".join(sorted(ffs))}')
    print(f'mux {args.mux}: S={mux["s"]}, S=1 selects {mux["in1"]}, S=0 selects {mux["in0"]}, '
          f'output {mux["y"]} {"inverted -> add " + args.inv if mux["inverting"] else "not inverted"}')

    text = open(args.netlist).read()
    modules = split_statements(tokenize(text))
    top = None
    for m in modules:
        hdr = m['stmts'][0]
        if hdr[1][1].strip() == args.top:
            top = m
    if top is None:
        sys.exit(f'error: module {args.top} not found in {args.netlist}')
    defined = {m['stmts'][0][1][1].strip() for m in modules}

    # ---- header & declarations ---------------------------------------------
    header = top['stmts'][0]
    hdr_txt = expr_text(header)
    if re.search(r'\b(input|output|inout)\b', hdr_txt):
        sys.exit('error: ANSI-style port list in the module header is not supported '
                 '(DC writes the classic style; re-write the netlist with DC)')
    ports = []                                  # names in header order
    for k, v in header[3:]:
        if k in ('id', 'esc'):
            ports.append(v.strip() if k == 'id' else v)
    port_dir, port_rng = {}, {}
    used_names = set()
    instances = []                              # (stmt_index, parsed)
    for idx, st in enumerate(top['stmts'][1:], start=1):
        k0, v0 = st[0]
        if k0 == 'id' and v0 in ('input', 'output', 'inout', 'wire', 'reg', 'tri', 'supply0', 'supply1', 'wand', 'wor'):
            rng = None
            j = 1
            if st[j] == ('sym', '['):
                close = next(t for t in range(j, len(st)) if st[t] == ('sym', ']'))
                rng = expr_text(st[j:close + 1])
                j = close + 1
            for k, v in st[j:]:
                if k in ('id', 'esc'):
                    nm = v.strip() if k == 'id' else v
                    used_names.add(nm)
                    if v0 in ('input', 'output', 'inout'):
                        port_dir[nm] = v0
                        port_rng[nm] = rng
            continue
        if k0 == 'id' and v0 in ('assign', 'parameter', 'localparam', 'defparam', 'specify', 'timescale'):
            continue
        p = parse_instance(st)
        if p is None:
            continue
        if p[0] in defined:
            sys.exit(f'error: netlist is hierarchical (instance of module {p[0]}). '
                     'Flatten it in DC first: ungroup -all -flatten')
        instances.append((idx, p))
        used_names.add(p[2])

    for nm in (args.scan_en, args.scan_in, args.scan_out):
        if nm in used_names:
            sys.exit(f'error: name {nm} already exists in the netlist')

    # ---- connectivity: net -> driver (inst idx, cell, out pin), assigns -------
    driver = {}
    for idx, (cell, params, name, pins) in instances:
        c = cells.get(cell)
        if not c:
            continue
        for pin, e in pins:
            if c['pins'].get(pin, {}).get('dir') == 'output':
                nk = net_key(e)
                if nk:
                    driver[nk] = (idx, cell, pin, name)
    alias = {}
    for st in top['stmts'][1:]:
        if st[0] == ('id', 'assign'):
            eq = next(t for t in range(len(st)) if st[t] == ('sym', '='))
            lhs, rhs = net_key(st[1:eq]), net_key(st[eq + 1:-1])
            if lhs and rhs:
                alias[lhs] = rhs
    inst_by_idx = dict(instances)

    def trace_clock(net):
        """follow buffers/inverters/assigns back to a primary input: (root, polarity, path)"""
        pol, path, seen = +1, [], set()
        while True:
            if net in seen:
                return None, pol, path
            seen.add(net)
            base = re.sub(r'\[\d+\]$', '', net)
            if base in port_dir and port_dir[base] == 'input':
                return net, pol, path
            if net in alias:
                net = alias[net]
                continue
            d = driver.get(net)
            if not d:
                return None, pol, path
            idx, cell, opin, iname = d
            if cell not in bufinv:
                return None, pol, path + [f'{iname.strip()} ({cell})']
            ipin, _, p = bufinv[cell]
            pol *= p
            path.append(f'{iname.strip()} ({cell})')
            _, _, _, pins = inst_by_idx[idx]
            e = dict(pins).get(ipin)
            net = net_key(e)
            if net is None:
                return None, pol, path

    # ---- flops & clock domains -----------------------------------------------
    excl = [re.compile(x) for x in args.exclude]
    groups = defaultdict(list)                  # (clock root, edge) -> [idx]
    problems = []
    nflops = 0
    for idx, (cell, params, name, pins) in instances:
        if cell not in ffs:
            if cells.get(cell, {}).get('latch'):
                problems.append(f'latch {name.strip()} ({cell}) is not scanned')
            continue
        nflops += 1
        nm = name.strip()
        if any(r.search(nm) for r in excl):
            continue
        info = ffs[cell]
        pmap = dict(pins)
        ck = net_key(pmap.get(info['clk']))
        root, pol, path = trace_clock(ck) if ck else (None, 1, [])
        if root is None:
            problems.append(f'flop {nm}: clock {ck} does not trace back to an input port '
                            f'(path: {" <- ".join(path) or "-"}) -> not scanned')
            continue
        edge = 'pos' if info['edge'] * pol > 0 else 'neg'
        groups[(root, edge)].append(idx)

    if not groups:
        sys.exit('error: no scannable flops found')
    total = sum(len(v) for v in groups.values())
    print(f'flops in netlist: {nflops}, scannable: {total}')
    for (root, edge), v in sorted(groups.items()):
        print(f'  clock {root:<12} {edge}edge : {len(v)} flops')
    for p in problems[:20]:
        print('  WARNING:', p)
    if len(problems) > 20:
        print(f'  ... {len(problems) - 20} more warnings')

    # ---- allocate chains to groups (largest remainder, at least 1 each) --------
    keys = sorted(groups, key=lambda k: (-len(groups[k]), k))
    if args.chains < len(keys):
        sys.exit(f'error: need at least {len(keys)} chains (one per clock domain/edge)')
    quota = {k: args.chains * len(groups[k]) / total for k in keys}
    alloc = {k: max(1, int(quota[k])) for k in keys}
    while sum(alloc.values()) < args.chains:
        k = max(keys, key=lambda k: quota[k] - alloc[k])
        alloc[k] += 1
    while sum(alloc.values()) > args.chains:
        k = max((k for k in keys if alloc[k] > 1), key=lambda k: alloc[k] - quota[k])
        alloc[k] -= 1

    chains = []                                 # [{'clock','edge','flops':[idx...]}]
    for k in keys:
        fl = sorted(groups[k], key=lambda i: natural_key(inst_by_idx[i][2].strip()))
        n = alloc[k]
        size = math.ceil(len(fl) / n)
        base = len(fl) // n
        extra = len(fl) % n
        start = 0
        for c in range(n):
            ln = base + (1 if c < extra else 0)
            chains.append({'clock': k[0], 'edge': k[1], 'flops': fl[start:start + ln]})
            start += ln

    # ---- rewrite ----------------------------------------------------------------
    SE = args.scan_en
    new_stmts = {}                              # stmt idx -> new text
    new_cells, new_wires = [], []
    serial = 0
    chain_report = []
    for ci, ch in enumerate(chains):
        si_net = f'{args.scan_in}[{ci}]'
        first = last = None
        for pos, idx in enumerate(ch['flops']):
            cell, params, name, pins = inst_by_idx[idx]
            info = ffs[cell]
            pins = list(pins)
            pmap = dict(pins)
            d_expr = pmap.get(info['d'])
            d_txt = expr_text(d_expr) if d_expr else "1'b0"
            q_expr = pmap.get(info['q'])
            if not q_expr:
                qn = f'scan_q_{serial}'
                new_wires.append(qn)
                q_expr = [('id', qn)]
                if info['q'] in pmap:
                    pins = [(p, (q_expr if p == info['q'] else e)) for p, e in pins]
                else:
                    pins.append((info['q'], q_expr))
            q_txt = expr_text(q_expr)
            m_y = f'scan_m_{serial}'
            new_wires.append(m_y)
            if mux['inverting']:
                d_new = f'scan_d_{serial}'
                new_wires.append(d_new)
                new_cells.append(f'  {args.mux} SCAN_MUX_{serial} ( .{mux["in1"]}({si_net}), .{mux["in0"]}({d_txt}), '
                                 f'.{mux["s"]}({SE}), .{mux["y"]}({m_y}) );')
                new_cells.append(f'  {args.inv} SCAN_INV_{serial} ( .{inv_in}({m_y}), .{inv_out}({d_new}) );')
            else:
                d_new = m_y
                new_cells.append(f'  {args.mux} SCAN_MUX_{serial} ( .{mux["in1"]}({si_net}), .{mux["in0"]}({d_txt}), '
                                 f'.{mux["s"]}({SE}), .{mux["y"]}({m_y}) );')
            pins = [(p, ([('id', d_new)] if p == info['d'] else e)) for p, e in pins]
            if info['d'] not in pmap:
                pins.append((info['d'], [('id', d_new)]))
            new_stmts[idx] = emit_instance(cell, params, name, pins)
            si_net = q_txt
            if first is None:
                first = name.strip()
            last = name.strip()
            serial += 1
        new_cells.append(f'  {args.buf} SCAN_SOBUF_{ci} ( .{buf_in}({si_net}), .{buf_out}({args.scan_out}[{ci}]) );')
        chain_report.append({'chain': ci, 'clock': ch['clock'], 'edge': ch['edge'],
                             'length': len(ch['flops']), 'first': first, 'last': last,
                             'scan_in': f'{args.scan_in}[{ci}]', 'scan_out': f'{args.scan_out}[{ci}]',
                             'flops': [inst_by_idx[i][2].strip() for i in ch['flops']]})

    nch = len(chains)
    out = []
    # header with the new ports
    hdr = expr_text(header)
    hdr = re.sub(r'\)\s*;\s*$', f', {SE}, {args.scan_in}, {args.scan_out} );', hdr)
    out.append(hdr)
    out.append(f'  input {SE};')
    out.append(f'  input [{nch - 1}:0] {args.scan_in};')
    out.append(f'  output [{nch - 1}:0] {args.scan_out};')
    for w in new_wires:                          # declared before first use
        out.append(f'  wire {w};')
    for idx, st in enumerate(top['stmts'][1:], start=1):
        if idx in new_stmts:
            out.append(new_stmts[idx])
        else:
            out.append('  ' + expr_text(st))
    out.append('  // ---- scan cells added by scan_insert.py: mux (+ inverter) per flop, buffer per scan_out ----')
    out.extend(new_cells)
    out.append('endmodule')

    with open(args.out, 'w') as f:
        f.write(f'// {os.path.basename(args.out)}: {args.netlist} + mux-D full scan, '
                f'{total} flops in {nch} chains (scan_insert.py)\n')
        # other modules (if any, e.g. none after flatten) are copied untouched
        for m in modules:
            if m is top:
                f.write('\n'.join(out) + '\n\n')
            else:
                f.write(expr_text([t for st in m['stmts'] for t in st]) + '\nendmodule\n\n')

    # ---- side files -----------------------------------------------------------
    clocks = sorted({c['clock'] for c in chain_report})
    holds = []
    for h in args.hold:
        p, v = h.split('=')
        holds.append((p.strip(), int(v)))

    with open(os.path.join(args.outdir, 'chains.json'), 'w') as f:
        json.dump({'top': args.top, 'scan_en': SE, 'chains': chain_report, 'clocks': clocks,
                   'hold': holds, 'total_flops': total, 'netlist_flops': nflops,
                   'warnings': problems}, f, indent=1)
    with open(os.path.join(args.outdir, 'chains.txt'), 'w') as f:
        f.write(f'# {total} scan flops in {nch} chains (netlist flops: {nflops})\n')
        f.write(f'# {"chain":>5} {"clock":<12} {"edge":<4} {"length":>6}  first flop -> last flop\n')
        for c in chain_report:
            f.write(f'  {c["chain"]:>5} {c["clock"]:<12} {c["edge"]:<4} {c["length"]:>6}  {c["first"]} -> {c["last"]}\n')
        f.write(f'# longest chain: {max(c["length"] for c in chain_report)} shift cycles per pattern\n')

    # SDC: scan ports, functional mode
    with open(os.path.join(args.outdir, 'scan_func.sdc'), 'w') as f:
        f.write('# scan_func.sdc (scan_insert.py): functional mode, scan_en = 0\n')
        f.write(f'set_case_analysis 0 [get_ports {SE}]\n')
        f.write(f'set_false_path -from [get_ports {{{args.scan_in}[*]}}]\n')
        f.write(f'set_false_path -to   [get_ports {{{args.scan_out}[*]}}]\n')
    # SDC: shift mode (scan ports timed against their own chain's clock)
    with open(os.path.join(args.outdir, 'scan_shift.sdc'), 'w') as f:
        f.write('# scan_shift.sdc (scan_insert.py): shift mode, scan_en = 1\n')
        f.write(f'set_case_analysis 1 [get_ports {SE}]\n')
        for p, v in holds:
            f.write(f'set_case_analysis {v} [get_ports {p}]\n')
        io = 0.2 * args.shift_period
        for c in chain_report:
            f.write(f'set_input_delay  {io:.2f} -clock {c["clock"]} [get_ports {{{c["scan_in"]}}}]\n')
            f.write(f'set_output_delay {io:.2f} -clock {c["clock"]} [get_ports {{{c["scan_out"]}}}]\n')

    # TetraMAX
    with open(os.path.join(args.outdir, 'tmax_scan.tcl'), 'w') as f:
        f.write('# tmax_scan.tcl (scan_insert.py): clocks, scan chains, scan enable, held pins\n')
        for c in clocks:
            f.write(f'add_clocks 0 {c} -shift\n')
        for p, v in holds:
            # async set/reset pins: declared as clocks with their OFF value so ATPG can
            # still pulse them in capture (tests the reset logic) but holds them in shift
            f.write(f'add_clocks {v} {p}\n')
        f.write(f'add_scan_enables 1 {SE}\n')
        for c in chain_report:
            f.write(f'add_scan_chains chain{c["chain"]} {c["scan_in"]} {c["scan_out"]}\n')

    # Modus pin assignment
    with open(os.path.join(args.outdir, 'modus.pinassign'), 'w') as f:
        for c in clocks:
            f.write(f'assign pin={c} test_function= -ES;\n')
        for p, v in holds:
            f.write(f'assign pin={p} test_function= {"+" if v else "-"}SC;\n')
        f.write(f'assign pin={SE} test_function= +SE;\n')
        for c in chain_report:
            f.write(f'assign pin={c["scan_in"]} test_function= SI{c["chain"]};\n')
            f.write(f'assign pin={c["scan_out"]} test_function= SO{c["chain"]};\n')

    # Conformal LEC dofile: pre-scan netlist (golden) vs scan netlist (revised)
    with open(os.path.join(args.outdir, 'lec_scan.do'), 'w') as f:
        f.write('// lec_scan.do (scan_insert.py): golden = netlist before scan, revised = scan netlist\n')
        f.write('// scan_en is tied to 0 in the revised design, scan_out is not compared:\n')
        f.write('// every other output and every flop must be equivalent.\n')
        f.write('// usage (from the project directory):  lec -nogui -xl -dofile scan/lec_scan.do\n')
        f.write('set log file lec/lec_scan.log -replace\n')
        f.write(f'read library -both -liberty {os.path.abspath(args.lib)}\n')
        f.write(f'read design -verilog -golden  {args.netlist}\n')
        f.write(f'read design -verilog -revised {args.out}\n')
        f.write(f'set root module {args.top} -both\n')
        f.write(f'add pin constraints 0 {SE} -revised\n')
        for c in chain_report:
            f.write(f'add ignored outputs {c["scan_out"]} -revised\n')
        f.write('set system mode lec\n')
        f.write('add compared points -all\n')
        f.write('compare\n')
        f.write('report verification\n')
        f.write('report statistics\n')
        f.write('report compare data -class nonequivalent\n')
        f.write('exit -force\n')

    write_flush_tb(os.path.join(args.outdir, 'tb_flush.v'), args, ports, port_dir, port_rng,
                   chain_report, clocks, holds, nch)

    print(f'scan netlist : {args.out}')
    print(f'chains       : {nch}, longest {max(c["length"] for c in chain_report)}, '
          f'shortest {min(c["length"] for c in chain_report)}')
    print(f'added cells  : {serial} x {args.mux}' + (f' + {serial} x {args.inv}' if mux['inverting'] else '')
          + f' + {nch} x {args.buf}')
    print(f'side files   : {args.outdir}/ chains.txt chains.json scan_func.sdc scan_shift.sdc '
          f'tmax_scan.tcl modus.pinassign lec_scan.do tb_flush.v')


def write_flush_tb(path, args, ports, port_dir, port_rng, chains, clocks, holds, nch):
    """Chain flush test: shift a known sequence into every chain and check that it
    comes out of scan_out after exactly <length> clocks."""
    SE = args.scan_en
    lens = [c['length'] for c in chains]
    T = max(lens) + 128
    hold = dict(holds)
    L = []
    L.append('// tb_flush.v (scan_insert.py): gate-level scan chain flush test')
    L.append('//   scan_en = 1, every chain gets its own pseudo-random bit stream; after')
    L.append('//   <length> shift clocks the same stream must appear on scan_out[k].')
    L.append('//   +define+SDF="file.sdf" back-annotates post-route delays.')
    L.append('`timescale 1ns/1ps')
    L.append('module tb_flush;')
    L.append('  localparam integer PERIOD = 100;          // ns, shift clock')
    L.append(f'  localparam integer NCH = {nch};')
    L.append(f'  localparam integer T   = {T};            // shift cycles = longest chain + 128')
    L.append('  integer len [0:NCH-1];')
    L.append('  initial begin')
    for c in chains:
        L.append(f'    len[{c["chain"]}] = {c["length"]};')
    L.append('  end')
    for p in ports:
        if p in (SE,):
            continue
        rng = (port_rng.get(p) or '') + ' '
        if port_dir.get(p) == 'input':
            L.append(f'  reg  {rng}{p};')
        elif port_dir.get(p) == 'output':
            L.append(f'  wire {rng}{p};')
    L.append(f'  reg  {SE};')
    L.append(f'  reg  [NCH-1:0] {args.scan_in};')
    L.append(f'  wire [NCH-1:0] {args.scan_out};')
    conns = ',\n    '.join(f'.{p}({p})' for p in ports + [SE, args.scan_in, args.scan_out])
    L.append(f'  {args.top} dut (\n    {conns}\n  );')
    L.append('`ifdef SDF')
    L.append('  initial $sdf_annotate(`SDF, dut);')
    L.append('`endif')
    L.append('  // bit t of chain k: a different, non-trivial pattern per chain')
    L.append('  function bitval(input integer k, input integer t);')
    L.append('    integer x;')
    L.append('    begin')
    L.append('      x = t * 1103515245 + k * 12345 + 7;')
    L.append('      bitval = x[13] ^ x[7] ^ (t % 3 == 0);')
    L.append('    end')
    L.append('  endfunction')
    L.append('  integer t, k, errors, checked;')
    L.append('  initial begin')
    L.append('    errors = 0; checked = 0;')
    for p in ports:
        if port_dir.get(p) == 'input' and p not in clocks:
            v = hold.get(p, 0)
            L.append(f"    {p} = {v};" if p in hold else f"    {p} = 0;")
    for c in clocks:
        L.append(f'    {c} = 0;')
    L.append(f'    {SE} = 1;')
    L.append(f'    {args.scan_in} = 0;')
    L.append('    #(PERIOD);')
    L.append('    for (t = 0; t < T; t = t + 1) begin')
    L.append('      for (k = 0; k < NCH; k = k + 1) ' + f'{args.scan_in}[k] = bitval(k, t);')
    L.append('      #(PERIOD*45/100);')
    L.append('      for (k = 0; k < NCH; k = k + 1)')
    L.append('        if (t >= len[k]) begin')
    L.append('          checked = checked + 1;')
    L.append(f'          if ({args.scan_out}[k] !== bitval(k, t - len[k])) begin')
    L.append('            errors = errors + 1;')
    L.append(f'            if (errors <= 20) $display("MISMATCH chain %0d cycle %0d: got %b expected %b", k, t, {args.scan_out}[k], bitval(k, t - len[k]));')
    L.append('          end')
    L.append('        end')
    L.append('      #(PERIOD*5/100);')
    L.append('      ' + ' '.join(f'{c} = 1;' for c in clocks))
    L.append('      #(PERIOD/2);')
    L.append('      ' + ' '.join(f'{c} = 0;' for c in clocks))
    L.append('    end')
    L.append('    if (errors == 0 && checked > 0)')
    L.append('      $display("FLUSH TEST PASSED: %0d chains, %0d bits checked", NCH, checked);')
    L.append('    else')
    L.append('      $display("FLUSH TEST FAILED: %0d mismatches out of %0d bits", errors, checked);')
    L.append('    $finish;')
    L.append('  end')
    L.append('endmodule')
    with open(path, 'w') as f:
        f.write('\n'.join(L) + '\n')


if __name__ == '__main__':
    main()
