"""Download the start (300 KB, ~560 tests) of every native-mode file of the
SingleStepTests 65816 suite (https://github.com/SingleStepTests/65816,
v1/xx.n.json) into a directory as xx.n.part, for convert.py.

usage: python -I fetch_tests.py <out_dir> [bytes_per_opcode]
"""
import concurrent.futures
import os
import sys
import urllib.request

BASE = 'https://raw.githubusercontent.com/SingleStepTests/65816/main/v1/%02x.n.json'


def fetch(op, out_dir, size):
    path = os.path.join(out_dir, '%02x.n.part' % op)
    if os.path.exists(path) and os.path.getsize(path) > 0:
        return op, 'cached'
    req = urllib.request.Request(BASE % op, headers={'Range': 'bytes=0-%d' % (size - 1)})
    with urllib.request.urlopen(req, timeout=60) as r:
        data = r.read(size)
    with open(path, 'wb') as f:
        f.write(data)
    return op, '%d bytes' % len(data)


def main():
    out_dir = sys.argv[1]
    size = int(sys.argv[2]) if len(sys.argv) > 2 else 300000
    os.makedirs(out_dir, exist_ok=True)
    with concurrent.futures.ThreadPoolExecutor(8) as ex:
        for op, status in ex.map(lambda o: fetch(o, out_dir, size), range(256)):
            print('%02x.n.part: %s' % (op, status))


if __name__ == '__main__':
    main()
