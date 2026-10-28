"""Download the SingleStepTests SPC700 suite (https://github.com/SingleStepTests/spc700,
v1/00.json .. v1/ff.json, ~80 MB) into a directory.

usage: python -I fetch_spc700_tests.py <out_dir> [first_opcode last_opcode]
"""
import concurrent.futures
import os
import sys
import urllib.request

BASE = 'https://raw.githubusercontent.com/SingleStepTests/spc700/main/v1/%02x.json'


def fetch(op, out_dir):
    path = os.path.join(out_dir, '%02x.json' % op)
    if os.path.exists(path) and os.path.getsize(path) > 0:
        return op, 'cached'
    with urllib.request.urlopen(BASE % op, timeout=60) as r:
        data = r.read()
    with open(path, 'wb') as f:
        f.write(data)
    return op, '%d bytes' % len(data)


def main():
    out_dir = sys.argv[1]
    first = int(sys.argv[2], 16) if len(sys.argv) > 2 else 0
    last = int(sys.argv[3], 16) if len(sys.argv) > 3 else 255
    os.makedirs(out_dir, exist_ok=True)
    with concurrent.futures.ThreadPoolExecutor(8) as ex:
        for op, status in ex.map(lambda o: fetch(o, out_dir), range(first, last + 1)):
            print('%02x.json: %s' % (op, status))


if __name__ == '__main__':
    main()
