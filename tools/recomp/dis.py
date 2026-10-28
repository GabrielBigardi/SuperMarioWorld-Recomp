"""Quick linear disassembler for inspection: python dis.py ADDR [count] [m] [x]

Reads the ROM from $SMW_ROM, or smw.sfc in the repository root."""
import sys, os
sys.path.insert(0, os.path.dirname(__file__))
from rom import Rom
from w65816 import decode

ROM_PATH = os.environ.get('SMW_ROM') or os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'smw.sfc')

if __name__ == '__main__':
    rom = Rom(ROM_PATH)
    addr = int(sys.argv[1], 16)
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 40
    m = int(sys.argv[3]) if len(sys.argv) > 3 else 1
    x = int(sys.argv[4]) if len(sys.argv) > 4 else 1
    for _ in range(n):
        ins = decode(rom.read8, addr, m, x)
        if ins is None:
            break
        raw = ' '.join('%02X' % rom.read8((addr & 0xFF0000) | ((addr + i) & 0xFFFF)) for i in range(ins.size))
        print('%02X:%04X  %-12s %s' % (addr >> 16, addr & 0xFFFF, raw, ins.fmt()))
        if ins.mn == 'REP':
            m = 0 if ins.operand & 0x20 else m
            x = 0 if ins.operand & 0x10 else x
        if ins.mn == 'SEP':
            m = 1 if ins.operand & 0x20 else m
            x = 1 if ins.operand & 0x10 else x
        addr = ins.next_addr()
