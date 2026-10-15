#!/usr/bin/env python3
"""
gen_stil.py  -  write the ATPG patterns as STIL (IEEE 1450) for a tester

Input is the pattern data gen_scan_tb.py wrote (load / unload / mask / pi /
po / cap .mem, ports.txt) - the same data the serial simulation (Step 7-1)
applied to the post-route netlist - plus scan/chains.json.

STIL content:
  Signals / SignalGroups   every port; _pi _po _si _so groups in the bit
                           order of the .mem files
  Timing                   one WaveformTable, 100 ns (shift and capture at
                           10 MHz, like tb_atpg.v): inputs driven at 0 ns,
                           outputs strobed at 45 ns, clock pulse 50-95 ns
  ScanStructures           16 chains: length, scan in / out, master clock,
                           ScanCells from scan in to scan out
  Procedures               load_unload (scan_en = 1, Shift: both clocks
                           pulse), capture_<clock> (scan_en = 0, PIs,
                           strobe POs, one pulse on that clock only)
  Pattern                  per pattern: load_unload (load this pattern,
                           unload the previous one), capture_<clock>
Scan data strings: the first character is applied / strobed in the first
shift cycle (STIL "#" substitution order). Unload: L / H = expect 0 / 1,
X = don't care (padding of short chains, flops with constant D).

--check reads the STIL back (separate parser) and compares every load,
expected unload, PI and expected PO bit with the .mem files.

Usage:
  python3 scripts/gen_stil.py --tb atpg/serial --chains scan/chains.json --out atpg/sm_top.stil [--check]
"""

import argparse
import datetime
import json
import os
import re
import sys


def bits(line):
    return line.strip()[::-1]


def q(name):
    return f'"{name}"'


def read_tb(tb):
    rd = lambda f: open(os.path.join(tb, f)).read().split()
    d = {k: rd(k + '.mem') for k in ('load', 'unload', 'unload_mask', 'pi', 'po', 'po_mask', 'cap')}
    pi_names, po_names, clocks, sets = [], [], [], []
    for ln in open(os.path.join(tb, 'ports.txt')):
        f = ln.split()
        if ln.startswith('# pi vector'):
            clocks = ln.split('= clock')[1].split()
        elif ln.startswith('# ') and 'patterns' in ln:
            sets.append(ln[2:].strip())
        elif f and f[0] == 'pi':
            pi_names.append(f[2])
        elif f and f[0] == 'po':
            po_names.append(f[2])
    return d, pi_names, po_names, clocks, sets


def write_stil(args):
    d, pi_names, po_names, clocks, sets = read_tb(args.tb)
    cj = json.load(open(args.chains))
    chains = cj['chains']
    nch = len(chains)
    NP = len(d['pi'])
    L = len(d['load'][0])
    se = cj['scan_en']
    si = [c['scan_in'] for c in chains]
    so = [c['scan_out'] for c in chains]
    out = []
    w = out.append
    w('STIL 1.0;')
    w('Header {')
    w(f'  Title "sm_top full-scan stuck-at patterns ({NP} patterns, one-hot capture)";')
    w(f'  Date "{datetime.date.today().isoformat()}";')
    w('  Source "gen_stil.py from the gen_scan_tb.py pattern data (Atalanta patterns)";')
    for s in sets:
        w(f'  Ann {{* {s} *}}')
    w('}')
    w('Signals {')
    for c in clocks:
        w(f'  {q(c)} In;')
    w(f'  {q(se)} In;')
    for k in range(nch):
        w(f'  {q(si[k])} In {{ ScanIn; }}')
    for k in range(nch):
        w(f'  {q(so[k])} Out {{ ScanOut; }}')
    for n in pi_names:
        w(f'  {q(n)} In;')
    for n in po_names:
        w(f'  {q(n)} Out;')
    w('}')
    w('SignalGroups {')
    w(f'  "_pi" = \'{" + ".join(q(n) for n in pi_names)}\';')
    w(f'  "_po" = \'{" + ".join(q(n) for n in po_names)}\';')
    w(f'  "_si" = \'{" + ".join(q(n) for n in si)}\' {{ ScanIn; }}')
    w(f'  "_so" = \'{" + ".join(q(n) for n in so)}\' {{ ScanOut; }}')
    w(f'  "_clk" = \'{" + ".join(q(c) for c in clocks)}\';')
    w('}')
    w('Timing {')
    w('  WaveformTable "_default_WFT_" {')
    w("    Period '100ns';")
    w('    Waveforms {')
    w(f'      "_pi" {{ 01 {{ \'0ns\' D/U; }} }}')
    w(f'      {q(se)} {{ 01 {{ \'0ns\' D/U; }} }}')
    w(f'      "_si" {{ 01 {{ \'0ns\' D/U; }} }}')
    w(f'      "_clk" {{ 0 {{ \'0ns\' D; }} P {{ \'0ns\' D; \'50ns\' U; \'95ns\' D; }} }}')
    w(f'      "_po" {{ LHX {{ \'0ns\' X; \'45ns\' L/H/X; \'55ns\' X; }} }}')
    w(f'      "_so" {{ LHX {{ \'0ns\' X; \'45ns\' L/H/X; \'55ns\' X; }} }}')
    w('    }')
    w('  }')
    w('}')
    w('ScanStructures {')
    for k, c in enumerate(chains):
        w(f'  ScanChain "chain{k}" {{')
        w(f'    ScanLength {len(c["flops"])};')
        w(f'    ScanIn {q(si[k])};')
        w(f'    ScanOut {q(so[k])};')
        w(f'    ScanMasterClock {q(c["clock"])};')
        w('    ScanCells ' + ' '.join(q(f) for f in c['flops']) + ';')
        w('  }')
    w('}')
    w('PatternBurst "_burst_" { PatList { "_pattern_" { } } }')
    w('PatternExec { PatternBurst "_burst_"; }')
    w('Procedures {')
    w('  "load_unload" {')
    w('    W "_default_WFT_";')
    w(f'    C {{ {q(se)} = 1; "_clk" = {"0" * len(clocks)}; "_so" = \\r{nch} X; }}')
    w(f'    Shift {{ V {{ "_si" = \\r{nch} #; "_so" = \\r{nch} #; "_clk" = {"P" * len(clocks)}; }} }}')
    w('  }')
    for i, c in enumerate(clocks):
        clk = ''.join('P' if j == i else '0' for j in range(len(clocks)))
        w(f'  "capture_{c}" {{')
        w('    W "_default_WFT_";')
        w(f'    F {{ {q(se)} = 0; }}')
        w(f'    V {{ "_pi" = \\r{len(pi_names)} #; "_po" = \\r{len(po_names)} #; "_clk" = {clk}; }}')
        w('  }')
    w('}')
    w('MacroDefs {')
    w('  "test_setup" {')
    w('    W "_default_WFT_";')
    p0 = bits(d['pi'][0])
    w(f'    V {{ "_clk" = {"0" * len(clocks)}; {q(se)} = 1; "_pi" = {p0}; }}')
    w('  }')
    w('}')
    w('Pattern "_pattern_" {')
    w('  W "_default_WFT_";')
    w('  "test_setup": Macro "test_setup";')

    def so_str(p, k):
        u, m = bits(d['unload'][p * nch + k]), bits(d['unload_mask'][p * nch + k])
        return ''.join(('H' if u[t] == '1' else 'L') if m[t] == '1' else 'X' for t in range(L))

    for p in range(NP + 1):
        args_ = []
        for k in range(nch):
            s_in = bits(d['load'][p * nch + k]) if p < NP else '0' * L
            s_out = so_str(p - 1, k) if p > 0 else None
            args_.append(f'{q(si[k])} = {s_in};')
            args_.append(f'{q(so[k])} = ' + (s_out if s_out is not None else f'\\r{L} X') + ';')
        lab = f'"pattern {p}": ' if p < NP else '"unload": '
        w(f'  {lab}Call "load_unload" {{')
        for a in args_:
            w('    ' + a)
        w('  }')
        if p < NP:
            cb = bits(d['cap'][p])
            cl = [c for i, c in enumerate(clocks) if cb[i] == '1']
            if len(cl) != 1:
                sys.exit(f'error: pattern {p} pulses {cl}: only one-hot capture is written')
            pv = bits(d['pi'][p])
            pov, pom = bits(d['po'][p]), bits(d['po_mask'][p])
            pos = ''.join(('H' if pov[i] == '1' else 'L') if pom[i] == '1' else 'X' for i in range(len(po_names)))
            w(f'  Call "capture_{cl[0]}" {{ "_pi" = {pv}; "_po" = {pos}; }}')
    w('}')
    with open(args.out, 'w') as f:
        f.write('\n'.join(out) + '\n')
    print(f'wrote {args.out}: {NP} patterns, {nch} chains, shift length {L}, '
          f'{os.path.getsize(args.out) / 1e6:.1f} MB')


def check_stil(args):
    """independent read-back: STIL -> per-pattern data, compared with the .mem files"""
    d, pi_names, po_names, clocks, sets = read_tb(args.tb)
    txt = open(args.out).read()
    nch = len(json.load(open(args.chains))['chains'])
    L = len(d['load'][0])
    # timing as written
    wft = re.search(r'WaveformTable "_default_WFT_" \{(.*?)\n  \}', txt, re.S).group(1)
    per = re.search(r"Period '(\d+)ns'", wft).group(1)
    strobe = re.findall(r"LHX \{ '0ns' X; '(\d+)ns' L/H/X", wft)
    clk = re.search(r"P \{ '0ns' D; '(\d+)ns' U; '(\d+)ns' D; \}", wft).groups()
    print(f'timing: period {per} ns, strobe {sorted(set(strobe))} ns, clock pulse {clk[0]}-{clk[1]} ns '
          f'(tb_atpg.v: 100 ns, strobe 45 ns, rise 50 ns)')
    # scan chain order in ScanStructures vs chains.json
    sc = re.findall(r'ScanChain "chain(\d+)" \{\s*ScanLength (\d+);.*?ScanCells (.*?);', txt, re.S)
    cj = json.load(open(args.chains))['chains']
    bad_order = sum(1 for k, n, cells in sc if re.findall(r'"([^"]+)"', cells) != cj[int(k)]['flops']
                    or int(n) != len(cj[int(k)]['flops']))
    calls = re.findall(r'Call "(load_unload|capture_\w+)" \{(.*?)\}', txt, re.S)
    loads, caps = [], []
    for name, body in calls:
        kv = dict((m.group(1), m.group(2)) for m in re.finditer(r'"([^"]+)" = ([^;]+);', body))
        (loads if name == 'load_unload' else caps).append((name, kv))
    cj_si = [c['scan_in'] for c in cj]
    cj_so = [c['scan_out'] for c in cj]
    NP = len(d['pi'])
    nbit = nbad = 0
    for p in range(NP):
        for k in range(nch):
            s = loads[p][1][cj_si[k]]
            ref = bits(d['load'][p * nch + k])
            nbit += L
            nbad += sum(a != b for a, b in zip(s, ref)) + abs(len(s) - L)
            s = loads[p + 1][1][cj_so[k]]
            u, m = bits(d['unload'][p * nch + k]), bits(d['unload_mask'][p * nch + k])
            for t in range(L):
                nbit += 1
                exp = ('H' if u[t] == '1' else 'L') if m[t] == '1' else 'X'
                nbad += s[t] != exp
        name, kv = caps[p]
        cb = bits(d['cap'][p])
        nbad += name != 'capture_' + [c for i, c in enumerate(clocks) if cb[i] == '1'][0]
        nbit += len(pi_names) + len(po_names)
        nbad += sum(a != b for a, b in zip(kv['_pi'], bits(d['pi'][p])))
        pov, pom = bits(d['po'][p]), bits(d['po_mask'][p])
        nbad += sum(kv['_po'][i] != (('H' if pov[i] == '1' else 'L') if pom[i] == '1' else 'X')
                    for i in range(len(po_names)))
    print(f'read back: {len(loads)} load_unload + {len(caps)} capture calls, {len(sc)} scan chains '
          f'({bad_order} with a different cell order), {nbit} bits compared with the .mem files, {nbad} differ')
    if nbad or bad_order or len(caps) != NP:
        sys.exit('STIL CHECK FAILED')
    print('STIL CHECK PASSED')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tb', required=True)
    ap.add_argument('--chains', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--check', action='store_true')
    args = ap.parse_args()
    write_stil(args)
    if args.check:
        check_stil(args)


if __name__ == '__main__':
    main()
