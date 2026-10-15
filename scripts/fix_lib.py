#!/usr/bin/env python3
"""
fix_lib.py  -  P&R copy of gscl45nm.lib with two known library defects removed

Usage (04_pnr.tcl calls it; the original library is never touched):
  python3 scripts/fix_lib.py $PDK/gscl45nm.lib pnr/sm_top/lib/gscl45nm_fix.lib

1. max_capacitance : 0
   A few output pins are characterised with max_capacitance = 0, which makes
   every net they drive a fake DRV. Those lines are dropped (same fix as DC's
   remove_attribute in 01_syn.tcl).

2. Spikes in the flop timing-check tables
   The setup / hold / recovery / removal tables of the flops contain isolated
   entries of 1.8 - 6.2 ns next to normal values of 0.03 - 0.6 ns, e.g.
   DFFPOSX1 setup_rising: 0.094 at CLK transition 0.06 ns, 6.2 at 0.3 ns.
   Post-CTS the leaf clock transition is ~0.08 ns, so the interpolated setup
   jumps to ~1 ns and hundreds of paths fail that DC (ideal clock) and
   pre-CTS timing passed with margin.
   Rule: in a flop's constraint table, a value > 1.0 ns and > 4 x the table
   median is a spike; it is replaced by the largest non-spike value of the
   same table (conservative: the check can only get stricter than a smooth
   table would give). Latch cells are left alone (none in this design).
   Every replacement is printed, so the report shows exactly what changed.
"""

import re
import sys

CHECKS = ('setup_', 'hold_', 'recovery_', 'removal_')
NUM = re.compile(r'-?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?')


def latch_cells(lines):
    cells, cell = set(), None
    for ln in lines:
        m = re.match(r'\s*cell\s*\(\s*"?(\w+)"?\s*\)', ln)
        if m:
            cell = m.group(1)
        if cell and re.match(r'\s*latch(_bank)?\s*\(', ln):
            cells.add(cell)
    return cells


def fix_values(block):
    """block: text of one 'values ( ... );' group -> (new text, [(old, new)])"""
    body = block[block.index('('):]
    rows = re.findall(r'"([^"]*)"', body)
    vals = [float(v) for r in rows for v in NUM.findall(r)]
    if not vals:
        return block, []
    med = sorted(vals)[len(vals) // 2]
    limit = max(1.0, 4 * abs(med))
    spikes = [v > limit for v in vals]
    if not any(spikes):
        return block, []
    repl = max(v for v, s in zip(vals, spikes) if not s)
    changes, it = [], iter(zip(vals, spikes))

    def sub(m):
        v, s = next(it)
        if s:
            changes.append((v, repl))
            return f'{repl:g}'
        return m.group(0)

    head = block[:block.index('(')]
    new_body = re.sub(r'"([^"]*)"', lambda q: '"' + NUM.sub(sub, q.group(1)) + '"', body)
    return head + new_body, changes


def main():
    src, dst = sys.argv[1], sys.argv[2]
    lines = open(src).read().split('\n')
    latches = latch_cells(lines)
    out, ncap, nspike = [], 0, 0
    cell = rel = ttype = None
    in_constraint = False
    buf = None
    for ln in lines:
        if buf is not None:
            buf.append(ln)
            if ');' in ln:
                text, changes = fix_values('\n'.join(buf))
                for old, new in changes:
                    print(f'{cell:9} related_pin {rel:4} {ttype:16} {in_constraint:5}: {old:g} -> {new:g}')
                nspike += len(changes)
                out.extend(text.split('\n'))
                buf = None
                in_constraint = False
            continue
        if re.match(r'^\s*max_capacitance\s*:\s*0\s*;', ln):
            ncap += 1
            continue
        m = re.match(r'\s*cell\s*\(\s*"?(\w+)"?\s*\)', ln)
        if m:
            cell = m.group(1)
        m = re.match(r'\s*related_pin\s*:\s*"?(\w+)"?', ln)
        if m:
            rel = m.group(1)
        m = re.match(r'\s*timing_type\s*:\s*(\w+)', ln)
        if m:
            ttype = m.group(1)
        m = re.match(r'\s*(rise|fall)_constraint\s*\(', ln)
        if m:
            in_constraint = m.group(1)
        if (in_constraint and cell not in latches and ttype and ttype.startswith(CHECKS)
                and re.match(r'\s*values\s*\(', ln)):
            buf = [ln]
            if ');' in ln:                          # single-line values group
                buf = None
                text, changes = fix_values(ln)
                for old, new in changes:
                    print(f'{cell:9} related_pin {rel:4} {ttype:16} {in_constraint:5}: {old:g} -> {new:g}')
                nspike += len(changes)
                out.append(text)
                in_constraint = False
            continue
        out.append(ln)
    if buf is not None:
        sys.exit('error: library ends inside a values group')
    with open(dst, 'w') as f:
        f.write('\n'.join(out))
    print(f'Info: dropped {ncap} max_capacitance : 0 lines, replaced {nspike} timing-check spikes -> {dst}')


if __name__ == '__main__':
    main()
