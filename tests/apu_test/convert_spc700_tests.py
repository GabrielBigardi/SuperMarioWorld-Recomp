"""Convert the SingleStepTests SPC700 JSON files (v1/xx.json) into one compact
binary file consumed by spc700_test.c.

usage: python -I convert_spc700_tests.py <json_dir> <out.bin>

Binary layout (little endian):
  "SPCT" u32 test_count
  per test:
    u8 opcode
    initial: u16 pc, u8 a, x, y, sp, psw, u16 n, n * (u16 addr, u8 val)
    final:   same layout
    u8 cycle_count                      (length of the "cycles" list)
    u16 n, n * (u16 addr, u8 val, u8 type)   bus accesses that carry data
                                             (type 0 = read, 1 = write)
"""
import json
import os
import struct
import sys


def pack_state(s):
    out = struct.pack('<HBBBBBH', s['pc'], s['a'], s['x'], s['y'], s['sp'], s['psw'], len(s['ram']))
    for addr, val in s['ram']:
        out += struct.pack('<HB', addr, val)
    return out


def main():
    src, dst = sys.argv[1], sys.argv[2]
    blobs = []
    for op in range(256):
        path = os.path.join(src, '%02x.json' % op)
        with open(path) as f:
            tests = json.load(f)
        for t in tests:
            b = struct.pack('<B', op)
            b += pack_state(t['initial'])
            b += pack_state(t['final'])
            cyc = t['cycles']
            b += struct.pack('<B', len(cyc))
            acc = [(a, v, 0 if k == 'read' else 1) for a, v, k in cyc
                   if k in ('read', 'write') and v is not None and a is not None]
            b += struct.pack('<H', len(acc))
            for a, v, k in acc:
                b += struct.pack('<HBB', a, v, k)
            blobs.append(b)
    with open(dst, 'wb') as f:
        f.write(b'SPCT' + struct.pack('<I', len(blobs)))
        for b in blobs:
            f.write(b)
    print('wrote %d tests to %s' % (len(blobs), dst))


if __name__ == '__main__':
    main()
