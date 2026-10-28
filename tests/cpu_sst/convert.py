"""Convert partial SingleStepTests 65816 JSON files (from fetch_tests.py) to the
compact text format read by sst.c.

usage: python -I convert.py <part_dir> <tests.txt>
"""
import json, os, sys
src, dst = sys.argv[1], sys.argv[2]
n = 0
with open(dst, 'w') as out:
    for fn in sorted(os.listdir(src)):
        if not fn.endswith('.n.part'):
            continue
        for line in open(os.path.join(src, fn)):
            line = line.strip().lstrip('[').rstrip(',').rstrip(']')
            if not line.startswith('{'):
                continue
            try:
                t = json.loads(line)
            except ValueError:
                continue
            def regs(s):
                return '%d %d %d %d %d %d %d %d %d %d %d %s' % (
                    s['pc'], s['s'], s['p'], s['a'], s['x'], s['y'], s['dbr'], s['d'], s['pbr'], s['e'],
                    len(s['ram']), ' '.join('%d %d' % (a, v) for a, v in s['ram']))
            out.write('%s %s %s\n' % (fn[:2], regs(t['initial']), regs(t['final'])))
            n += 1
print('tests:', n)
