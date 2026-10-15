#!/usr/bin/env python3
"""
gen_scan_tb.py  -  serial (tester-style) simulation of the ATPG patterns

Atalanta's patterns are combinational: a value for every PI and every flop
Q (PPI), and the expected value of every PO and every flop D (PPO). This
script turns them into what a tester would apply through the scan chains
and writes a gate-level testbench that does exactly that:

  for every pattern p:
    shift  : scan_en = 1, L_max clocks; scan_in[k] carries pattern p's
             values for chain k, scan_out[k] is compared with the values
             captured by pattern p-1 (load / unload overlapped)
    capture: scan_en = 0, PIs = pattern p, compare all POs, one pulse on
             the clock(s) in cap.mem for pattern p

With --bench/--test (one set, all flops observed) both clocks are pulsed
together. tile_clk and clk_mem are asynchronous, so with SDF the paths
between the domains race in that capture (seen on pattern 39). With
--set PREFIX:CLOCK (one ATPG model and pattern set per clock, made with
netlist2bench.py --capture CLOCK) every pattern pulses only its own clock;
flops of the other domain are expected to keep the value shifted in.

A pass means the chain order, the PPI/PPO mapping, the ATPG model and the
post-route netlist (optionally with SDF) all agree, bit for bit.

Outputs in --outdir:
  tb_atpg.v          testbench (+FIRST=n +LAST=n select a pattern range)
  load.mem           NP*NCH lines of L_max bits: what to shift in
  unload.mem         NP*NCH lines: expected scan_out values
  unload_mask.mem    1 = compare this bit (0: padding of short chains,
                     flops whose D is a constant in the ATPG model)
  pi.mem / po.mem / po_mask.mem   PI values and expected POs per pattern
  cap.mem            per pattern, which clocks to pulse in the capture
  ports.txt          bit order of the pi / po vectors

Usage:
  python3 scripts/gen_scan_tb.py --netlist pnr/sm_top/sm_top_pnr.v --chains scan/chains.json \
      --set atpg/sm_top_tile_clk:tile_clk --set atpg/sm_top_clk_mem:clk_mem \
      --const scan_en=0 --const tile_rst_req=0 --const rst_n_mem=1 --outdir atpg/serial
  (or --bench atpg/sm_top --test atpg/sm_top.test for one set, both clocks pulsed)
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
_spec2 = importlib.util.spec_from_file_location('netlist2bench', os.path.join(HERE, 'netlist2bench.py'))
nb = importlib.util.module_from_spec(_spec2)
_spec2.loader.exec_module(nb)


def read_ports(netlist, top):
    """header-order port list with direction and bit list (only the declarations are parsed)"""
    text = open(netlist).read()
    m = re.search(r'\bmodule\s+' + re.escape(top) + r'\b', text)
    end = text.find('wire', m.start())          # declarations of ports come before the wires
    hdr = text[m.start():end if end > 0 else m.start() + 200000]
    toks = si.tokenize(hdr + '\nendmodule\n')
    mod = si.split_statements(toks)[0]
    order = [v.strip() for k, v in mod['stmts'][0][3:] if k in ('id', 'esc')]
    pdir, pbits = {}, {}
    for st in mod['stmts'][1:]:
        k0, v0 = st[0]
        if k0 == 'id' and v0 in ('input', 'output'):
            rng, j = None, 1
            if st[j] == ('sym', '['):
                close = next(t for t in range(j, len(st)) if st[t] == ('sym', ']'))
                rng = si.expr_text(st[j:close + 1])
                j = close + 1
            for k, v in st[j:]:
                if k in ('id', 'esc'):
                    nm = v.strip()
                    pdir[nm] = (v0, rng)
                    pbits[nm] = nb.bits_of(nm, rng)
    return order, pdir, pbits


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--netlist', required=True)
    ap.add_argument('--top', default='sm_top')
    ap.add_argument('--chains', required=True)
    ap.add_argument('--bench', help='netlist2bench.py --out prefix (one set, all clocks captured)')
    ap.add_argument('--test')
    ap.add_argument('--set', action='append', default=[],
                    help='PREFIX:CLOCK  one-hot capture set: PREFIX.bench/.map/.test, only CLOCK is pulsed')
    ap.add_argument('--const', action='append', default=[])
    ap.add_argument('--outdir', required=True)
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    cj = json.load(open(args.chains))
    se, chains, clocks = cj['scan_en'], cj['chains'], cj['clocks']
    nch = len(chains)
    L = max(c['length'] for c in chains)
    domain = {f: c['clock'] for c in chains for f in c['flops']}
    order, pdir, pbits = read_ports(args.netlist, args.top)
    consts = {}
    for h in args.const:
        p, v = h.split('=')
        for b in pbits[p]:
            consts[b] = v

    sets = [(s_.rsplit(':', 1)[0] + '.bench', s_.rsplit(':', 1)[0] + '.test', s_.rsplit(':', 1)[1]) for s_ in args.set]
    if args.bench:
        sets.insert(0, (args.bench + '.bench', args.test, 'all'))
    if not sets:
        sys.exit('error: give --bench/--test or --set')

    def load_set(bench, test):
        """-> pi_src, ppi_src, po_src, ppo_src (name -> column) and the patterns"""
        b_in, b_out = [], []
        for ln in open(bench):
            m = re.match(r'INPUT\((\w+)\)', ln)
            if m:
                b_in.append(m.group(1))
            m = re.match(r'OUTPUT\((\w+)\)', ln)
            if m:
                b_out.append(m.group(1))
        lab_in, lab_out = {}, defaultdict(list)
        for ln in open(bench[:-6] + '.map'):
            f = ln.split()
            if len(f) >= 3 and f[1] in ('PI', 'PPI'):
                lab_in[f[0]] = (f[1], f[2])
            elif len(f) >= 3 and f[1] in ('PO', 'PPO'):
                lab_out[f[0]].append((f[1], f[2]))
        names, pats, in_hdr = [], [], False
        for ln in open(test):
            if ln.startswith('* Primary inputs'):
                in_hdr = True
                continue
            if ln.startswith('*'):
                in_hdr = False
                continue
            if in_hdr:
                names += ln.split()
                continue
            m = re.match(r'\s*\d+:\s*([01]+)\s+([01]+)', ln)
            if m:
                pats.append((m.group(1), m.group(2)))
        if sorted(names) != sorted(b_in):
            sys.exit(f'error: {test}: inputs do not match {bench}')
        col_in = {n: i for i, n in enumerate(names)}
        col_out = {n: i for i, n in enumerate(b_out)}
        pi_src, ppi_src, po_src, ppo_src = {}, {}, {}, {}
        for n, (kind, lbl) in lab_in.items():
            (pi_src if kind == 'PI' else ppi_src)[lbl if kind == 'PI' else lbl[:-2]] = col_in[n]
        for n, labs in lab_out.items():
            for kind, lbl in labs:
                (po_src if kind == 'PO' else ppo_src)[lbl if kind == 'PO' else lbl[:-2]] = col_out[n]
        return pi_src, ppi_src, po_src, ppo_src, pats

    # ---- DUT pin vectors ----------------------------------------------------------------
    sin_name = chains[0]['scan_in'].split('[')[0]
    pi_bits = [b for p in order if pdir[p][0] == 'input' and p not in clocks and p not in (se, sin_name)
               for b in pbits[p]]
    po_bits = [b for p in order if pdir[p][0] == 'output' for b in pbits[p]]

    def vec(bits):          # list of chars, bit 0 first -> readmemb string (MSB first)
        return ''.join(reversed(bits))

    fl = open(os.path.join(args.outdir, 'load.mem'), 'w')
    fu = open(os.path.join(args.outdir, 'unload.mem'), 'w')
    fm = open(os.path.join(args.outdir, 'unload_mask.mem'), 'w')
    fpi = open(os.path.join(args.outdir, 'pi.mem'), 'w')
    fpo = open(os.path.join(args.outdir, 'po.mem'), 'w')
    fpm = open(os.path.join(args.outdir, 'po_mask.mem'), 'w')
    fcap = open(os.path.join(args.outdir, 'cap.mem'), 'w')
    n_unload_checked = n_held = NP = 0
    set_info = []
    for bench, test, cap in sets:
        pi_src, ppi_src, po_src, ppo_src, pats = load_set(bench, test)
        captured = set(clocks) if cap == 'all' else {cap}
        po_mask = vec(['1' if b in po_src else '0' for b in po_bits])
        cap_bits = vec(['1' if c in captured else '0' for c in clocks])
        set_info.append(f'{os.path.basename(bench)[:-6]}: patterns {NP}..{NP + len(pats) - 1}, capture {"+".join(sorted(captured))}')
        for ib, ob in pats:
            fpi.write(vec([consts[b] if b in consts else (ib[pi_src[b]] if b in pi_src else '0') for b in pi_bits]) + '\n')
            fpo.write(vec([ob[po_src[b]] if b in po_src else '0' for b in po_bits]) + '\n')
            fpm.write(po_mask + '\n')
            fcap.write(cap_bits + '\n')
            for c in chains:
                f = c['flops']
                l = len(f)
                load = ['0'] * L
                unl = ['0'] * L
                msk = ['0'] * L
                q = {}
                for t in range(L):
                    i = L - 1 - t                   # flop that ends up with the bit shifted in at cycle t
                    if i < l:
                        q[f[i]] = ib[ppi_src[f[i]]] if f[i] in ppi_src else '0'   # not in the model: don't care
                        load[t] = q[f[i]]
                for t in range(L):
                    j = l - 1 - t                   # flop visible on scan_out before shift edge t
                    if j < 0:
                        continue
                    fj = f[j]
                    if domain[fj] in captured:
                        if fj in ppo_src:
                            unl[t] = ob[ppo_src[fj]]
                            msk[t] = '1'
                            n_unload_checked += 1
                    else:                           # not clocked in this capture: keeps the loaded value
                        unl[t] = q[fj]
                        msk[t] = '1'
                        n_unload_checked += 1
                        n_held += 1
                fl.write(vec(load) + '\n')
                fu.write(vec(unl) + '\n')
                fm.write(vec(msk) + '\n')
            NP += 1
    for h in (fl, fu, fm, fpi, fpo, fpm, fcap):
        h.close()
    with open(os.path.join(args.outdir, 'ports.txt'), 'w') as f:
        f.write('# pi vector, bit 0 first; capture vector bit i = clock ' + ' '.join(clocks) + '\n')
        for x in set_info:
            f.write('# ' + x + '\n')
        for i, b in enumerate(pi_bits):
            f.write(f'pi {i} {b} {"const " + consts[b] if b in consts else "pattern"}\n')
        for i, b in enumerate(po_bits):
            f.write(f'po {i} {b}\n')

    # ---- testbench ------------------------------------------------------------------------
    NIN, NOUT = len(pi_bits), len(po_bits)
    sin = chains[0]['scan_in'].split('[')[0]
    sout = chains[0]['scan_out'].split('[')[0]
    T = []
    T.append('// tb_atpg.v (gen_scan_tb.py): ATPG patterns applied through the scan chains')
    T.append('`timescale 1ns/1ps')
    T.append('module tb_atpg;')
    T.append('  localparam integer PERIOD = 100;')
    T.append(f'  localparam integer NP = {NP}, NCH = {nch}, L = {L}, NIN = {NIN}, NOUT = {NOUT};')
    T.append('  reg [L-1:0]    load_mem [0:NP*NCH-1];')
    T.append('  reg [L-1:0]    unl_mem  [0:NP*NCH-1];')
    T.append('  reg [L-1:0]    unl_msk  [0:NP*NCH-1];')
    T.append('  reg [NIN-1:0]  pi_mem   [0:NP-1];')
    T.append('  reg [NOUT-1:0] po_mem   [0:NP-1];')
    T.append('  reg [NOUT-1:0] po_msk   [0:NP-1];')
    T.append(f'  reg [{len(clocks) - 1}:0] cap_mem [0:NP-1];   // bit i: pulse clock i ({" ".join(clocks)}) in this capture')
    T.append('  reg  [NIN-1:0] pi_vec;')
    T.append('  wire [NOUT-1:0] po_vec;')
    for c in clocks:
        T.append(f'  reg {c};')
    T.append(f'  reg {se};')
    T.append(f'  reg  [NCH-1:0] {sin};')
    T.append(f'  wire [NCH-1:0] {sout};')
    # functional ports as wires driven from / collected into the vectors
    for p in order:
        d, rng = pdir[p]
        if p in clocks or p in (se, sin, sout):
            continue
        T.append(f'  wire {(rng or "") + " "}{p};')
    for i, b in enumerate(pi_bits):
        T.append(f'  assign {b} = pi_vec[{i}];')
    for i, b in enumerate(po_bits):
        T.append(f'  assign po_vec[{i}] = {b};')
    conns = ',\n    '.join(f'.{p}({p})' for p in order)
    T.append(f'  {args.top} dut (\n    {conns}\n  );')
    T.append('`ifdef SDF')
    T.append('  initial $sdf_annotate(`SDF, dut);')
    T.append('`endif')
    T.append('`ifdef INJECT_NET   // defect injection for the diagnosis experiments: net stuck at INJECT_VAL')
    T.append('  initial force dut.`INJECT_NET = `INJECT_VAL;')
    T.append('`endif')
    T.append('  integer p, t, k, first, last, err_po, err_so, n_po, n_so, flog;')
    T.append('  reg [8*256-1:0] flog_name;')
    T.append('  initial begin')
    T.append('    $readmemb("load.mem", load_mem);  $readmemb("unload.mem", unl_mem);  $readmemb("unload_mask.mem", unl_msk);')
    T.append('    $readmemb("pi.mem", pi_mem);      $readmemb("po.mem", po_mem);       $readmemb("po_mask.mem", po_msk);')
    T.append('    $readmemb("cap.mem", cap_mem);')
    T.append('    if (!$value$plusargs("FIRST=%d", first)) first = 0;')
    T.append('    if (!$value$plusargs("LAST=%d", last))   last  = NP - 1;')
    T.append('    err_po = 0; err_so = 0; n_po = 0; n_so = 0; flog = 0;')
    T.append('    // +FAILLOG=file: every failing bit, like a tester fail log (SO pattern chain shift got / PO pattern bit got)')
    T.append('    if ($value$plusargs("FAILLOG=%s", flog_name)) flog = $fopen(flog_name, "w");')
    T.append('    ' + ' '.join(f'{c} = 0;' for c in clocks) + f' {se} = 1; {sin} = 0; pi_vec = pi_mem[first];')
    T.append('    #(PERIOD);')
    T.append('    for (p = first; p <= last + 1; p = p + 1) begin')
    T.append('      // shift: load pattern p, unload pattern p-1')
    T.append(f'      {se} = 1;')
    T.append('      for (t = 0; t < L; t = t + 1) begin')
    T.append(f'        for (k = 0; k < NCH; k = k + 1) {sin}[k] = (p <= last) ? load_mem[p*NCH+k][t] : 1\'b0;')
    T.append('        #(PERIOD*45/100);')
    T.append('        if (p > first)')
    T.append('          for (k = 0; k < NCH; k = k + 1)')
    T.append('            if (unl_msk[(p-1)*NCH+k][t]) begin')
    T.append('              n_so = n_so + 1;')
    T.append(f'              if ({sout}[k] !== unl_mem[(p-1)*NCH+k][t]) begin')
    T.append('                err_so = err_so + 1;')
    T.append(f'                if (flog) $fdisplay(flog, "SO %0d %0d %0d %b", p-1, k, t, {sout}[k]);')
    T.append(f'                if (err_so <= 20) $display("UNLOAD MISMATCH pattern %0d chain %0d shift %0d: got %b expected %b", p-1, k, t, {sout}[k], unl_mem[(p-1)*NCH+k][t]);')
    T.append('              end')
    T.append('            end')
    T.append('        #(PERIOD*5/100);')
    T.append('        ' + ' '.join(f'{c} = 1;' for c in clocks))
    T.append('        #(PERIOD/2);')
    T.append('        ' + ' '.join(f'{c} = 0;' for c in clocks))
    T.append('      end')
    T.append('      if (p <= last) begin')
    T.append('        // capture: functional mode, apply PIs, check POs, one clock pulse')
    T.append(f'        {se} = 0; pi_vec = pi_mem[p];')
    T.append('        #(PERIOD*45/100);')
    T.append('        for (k = 0; k < NOUT; k = k + 1)')
    T.append('          if (po_msk[p][k]) begin')
    T.append('            n_po = n_po + 1;')
    T.append('            if (po_vec[k] !== po_mem[p][k]) begin')
    T.append('              err_po = err_po + 1;')
    T.append('              if (flog) $fdisplay(flog, "PO %0d %0d %b", p, k, po_vec[k]);')
    T.append('              if (err_po <= 20) $display("PO MISMATCH pattern %0d bit %0d: got %b expected %b", p, k, po_vec[k], po_mem[p][k]);')
    T.append('            end')
    T.append('          end')
    T.append('        #(PERIOD*5/100);')
    T.append('        ' + ' '.join(f'if (cap_mem[p][{i}]) {c} = 1;' for i, c in enumerate(clocks)))
    T.append('        #(PERIOD/2);')
    T.append('        ' + ' '.join(f'{c} = 0;' for c in clocks))
    T.append('        #(PERIOD/2);')
    T.append('      end')
    T.append('      if ((p - first) % 100 == 0) $display("progress: pattern %0d, %0d PO + %0d scan_out bits checked, %0d errors", p, n_po, n_so, err_po + err_so);')
    T.append('    end')
    T.append('    if (err_po + err_so == 0 && n_so > 0)')
    T.append('      $display("ATPG SERIAL SIM PASSED: patterns %0d..%0d, %0d PO bits + %0d scan_out bits checked", first, last, n_po, n_so);')
    T.append('    else')
    T.append('      $display("ATPG SERIAL SIM FAILED: %0d PO + %0d scan_out mismatches (%0d + %0d checked)", err_po, err_so, n_po, n_so);')
    T.append('    if (flog) $fclose(flog);')
    T.append('    $finish;')
    T.append('  end')
    T.append('endmodule')
    with open(os.path.join(args.outdir, 'tb_atpg.v'), 'w') as f:
        f.write('\n'.join(T) + '\n')

    print(f'{NP} patterns, {nch} chains, L_max {L}, {NIN} PI bits, {NOUT} PO bits')
    for x in set_info:
        print('  ' + x)
    print(f'flops not clocked in their pattern\'s capture (checked against the loaded value): {n_held}')
    print(f'scan_out bits to check: {n_unload_checked} ({n_unload_checked // NP} per pattern); '
          f'shift cycles per pattern {L}, total {NP * (L + 1) + L}')
    print(f'wrote {args.outdir}/tb_atpg.v and *.mem')


if __name__ == '__main__':
    main()
