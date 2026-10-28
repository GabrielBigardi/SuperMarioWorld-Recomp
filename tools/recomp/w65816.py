"""65816 instruction set description + decoder.

Shared by the static recompiler (recomp.py) and the analysis helpers.
Operand size of immediate instructions depends on the M/X flags, so decoding
always takes the current (m, x) state.
"""

# Addressing modes
IMP, ACC, IMM_M, IMM_X, IMM8, IMM16 = 'imp', 'acc', 'immm', 'immx', 'imm8', 'imm16'
DP, DPX, DPY, DPI, DPIX, DPIY, DPIL, DPILY = 'dp', 'dpx', 'dpy', 'dpi', 'dpix', 'dpiy', 'dpil', 'dpily'
ABS, ABSX, ABSY, LONG, LONGX = 'abs', 'absx', 'absy', 'long', 'longx'
ABSI, ABSIX, ABSIL = 'absi', 'absix', 'absil'
SR, SRIY = 'sr', 'sriy'
REL8, REL16, BLK = 'rel8', 'rel16', 'blk'

# operand byte count per mode (IMM_M / IMM_X resolved at decode time)
MODE_LEN = {
    IMP: 0, ACC: 0, IMM8: 1, IMM16: 2,
    DP: 1, DPX: 1, DPY: 1, DPI: 1, DPIX: 1, DPIY: 1, DPIL: 1, DPILY: 1,
    ABS: 2, ABSX: 2, ABSY: 2, LONG: 3, LONGX: 3,
    ABSI: 2, ABSIX: 2, ABSIL: 2, SR: 1, SRIY: 1,
    REL8: 1, REL16: 2, BLK: 2,
}

OPS = [None] * 256


def _op(code, mn, mode, cyc):
    OPS[code] = (mn, mode, cyc)


# ALU group: ORA AND EOR ADC STA LDA CMP SBC
_alu = ['ORA', 'AND', 'EOR', 'ADC', 'STA', 'LDA', 'CMP', 'SBC']
for i, mn in enumerate(_alu):
    base = i * 0x20
    _op(base + 0x01, mn, DPIX, 6)
    _op(base + 0x03, mn, SR, 4)
    _op(base + 0x05, mn, DP, 3)
    _op(base + 0x07, mn, DPIL, 6)
    if mn != 'STA':
        _op(base + 0x09, mn, IMM_M, 2)
    _op(base + 0x0D, mn, ABS, 4)
    _op(base + 0x0F, mn, LONG, 5)
    _op(base + 0x11, mn, DPIY, 5)
    _op(base + 0x12, mn, DPI, 5)
    _op(base + 0x13, mn, SRIY, 7)
    _op(base + 0x15, mn, DPX, 4)
    _op(base + 0x17, mn, DPILY, 6)
    _op(base + 0x19, mn, ABSY, 4)
    _op(base + 0x1D, mn, ABSX, 4)
    _op(base + 0x1F, mn, LONGX, 5)

# RMW group: ASL ROL LSR ROR
for i, mn in enumerate(['ASL', 'ROL', 'LSR', 'ROR']):
    base = i * 0x20
    _op(base + 0x06, mn, DP, 5)
    _op(base + 0x0A, mn, ACC, 2)
    _op(base + 0x0E, mn, ABS, 6)
    _op(base + 0x16, mn, DPX, 6)
    _op(base + 0x1E, mn, ABSX, 7)

_table = {
    0x00: ('BRK', IMM8, 7), 0x02: ('COP', IMM8, 7), 0x04: ('TSB', DP, 5), 0x08: ('PHP', IMP, 3),
    0x0B: ('PHD', IMP, 4), 0x0C: ('TSB', ABS, 6), 0x10: ('BPL', REL8, 2), 0x14: ('TRB', DP, 5),
    0x18: ('CLC', IMP, 2), 0x1A: ('INC', ACC, 2), 0x1B: ('TCS', IMP, 2), 0x1C: ('TRB', ABS, 6),
    0x20: ('JSR', ABS, 6), 0x22: ('JSL', LONG, 8), 0x24: ('BIT', DP, 3), 0x28: ('PLP', IMP, 4),
    0x2B: ('PLD', IMP, 5), 0x2C: ('BIT', ABS, 4), 0x30: ('BMI', REL8, 2), 0x34: ('BIT', DPX, 4),
    0x38: ('SEC', IMP, 2), 0x3A: ('DEC', ACC, 2), 0x3B: ('TSC', IMP, 2), 0x3C: ('BIT', ABSX, 4),
    0x40: ('RTI', IMP, 6), 0x42: ('WDM', IMM8, 2), 0x44: ('MVP', BLK, 7), 0x48: ('PHA', IMP, 3),
    0x4B: ('PHK', IMP, 3), 0x4C: ('JMP', ABS, 3), 0x50: ('BVC', REL8, 2), 0x54: ('MVN', BLK, 7),
    0x58: ('CLI', IMP, 2), 0x5A: ('PHY', IMP, 3), 0x5B: ('TCD', IMP, 2), 0x5C: ('JML', LONG, 4),
    0x60: ('RTS', IMP, 6), 0x62: ('PER', REL16, 6), 0x64: ('STZ', DP, 3), 0x68: ('PLA', IMP, 4),
    0x6B: ('RTL', IMP, 6), 0x6C: ('JMP', ABSI, 5), 0x70: ('BVS', REL8, 2), 0x74: ('STZ', DPX, 4),
    0x78: ('SEI', IMP, 2), 0x7A: ('PLY', IMP, 4), 0x7B: ('TDC', IMP, 2), 0x7C: ('JMP', ABSIX, 6),
    0x80: ('BRA', REL8, 3), 0x82: ('BRL', REL16, 4), 0x84: ('STY', DP, 3), 0x86: ('STX', DP, 3),
    0x88: ('DEY', IMP, 2), 0x89: ('BIT', IMM_M, 2), 0x8A: ('TXA', IMP, 2), 0x8B: ('PHB', IMP, 3),
    0x8C: ('STY', ABS, 4), 0x8E: ('STX', ABS, 4), 0x90: ('BCC', REL8, 2), 0x94: ('STY', DPX, 4),
    0x96: ('STX', DPY, 4), 0x98: ('TYA', IMP, 2), 0x9A: ('TXS', IMP, 2), 0x9B: ('TXY', IMP, 2),
    0x9C: ('STZ', ABS, 4), 0x9E: ('STZ', ABSX, 5), 0xA0: ('LDY', IMM_X, 2), 0xA2: ('LDX', IMM_X, 2),
    0xA4: ('LDY', DP, 3), 0xA6: ('LDX', DP, 3), 0xA8: ('TAY', IMP, 2), 0xAA: ('TAX', IMP, 2),
    0xAB: ('PLB', IMP, 4), 0xAC: ('LDY', ABS, 4), 0xAE: ('LDX', ABS, 4), 0xB0: ('BCS', REL8, 2),
    0xB4: ('LDY', DPX, 4), 0xB6: ('LDX', DPY, 4), 0xB8: ('CLV', IMP, 2), 0xBA: ('TSX', IMP, 2),
    0xBB: ('TYX', IMP, 2), 0xBC: ('LDY', ABSX, 4), 0xBE: ('LDX', ABSY, 4), 0xC0: ('CPY', IMM_X, 2),
    0xC2: ('REP', IMM8, 3), 0xC4: ('CPY', DP, 3), 0xC6: ('DEC', DP, 5), 0xC8: ('INY', IMP, 2),
    0xCA: ('DEX', IMP, 2), 0xCB: ('WAI', IMP, 3), 0xCC: ('CPY', ABS, 4), 0xCE: ('DEC', ABS, 6),
    0xD0: ('BNE', REL8, 2), 0xD4: ('PEI', DP, 6), 0xD6: ('DEC', DPX, 6), 0xD8: ('CLD', IMP, 2),
    0xDA: ('PHX', IMP, 3), 0xDB: ('STP', IMP, 3), 0xDC: ('JML', ABSIL, 6), 0xDE: ('DEC', ABSX, 7),
    0xE0: ('CPX', IMM_X, 2), 0xE2: ('SEP', IMM8, 3), 0xE4: ('CPX', DP, 3), 0xE6: ('INC', DP, 5),
    0xE8: ('INX', IMP, 2), 0xEA: ('NOP', IMP, 2), 0xEB: ('XBA', IMP, 3), 0xEC: ('CPX', ABS, 4),
    0xEE: ('INC', ABS, 6), 0xF0: ('BEQ', REL8, 2), 0xF4: ('PEA', IMM16, 5), 0xF6: ('INC', DPX, 6),
    0xF8: ('SED', IMP, 2), 0xFA: ('PLX', IMP, 4), 0xFB: ('XCE', IMP, 2), 0xFC: ('JSR', ABSIX, 8),
    0xFE: ('INC', ABSX, 7),
}
for k, v in _table.items():
    _op(k, *v)

assert all(o is not None for o in OPS), [hex(i) for i, o in enumerate(OPS) if o is None]

BRANCHES = {'BPL', 'BMI', 'BVC', 'BVS', 'BCC', 'BCS', 'BNE', 'BEQ'}
# instructions that end a basic block
TERMINATORS = BRANCHES | {'BRA', 'BRL', 'JMP', 'JML', 'JSR', 'JSL', 'RTS', 'RTL', 'RTI',
                          'BRK', 'COP', 'STP', 'WAI', 'PLP', 'XCE', 'WDM', 'REP', 'SEP'}
# REP/SEP don't strictly need to end a block (flags are statically known), but
# MVN/MVP loop on themselves, so they are terminators too.
TERMINATORS.discard('REP')
TERMINATORS.discard('SEP')
TERMINATORS |= {'MVN', 'MVP'}


class Insn:
    __slots__ = ('addr', 'op', 'mn', 'mode', 'size', 'operand', 'cyc', 'm', 'x')

    def __init__(self, addr, op, mn, mode, size, operand, cyc, m, x):
        self.addr, self.op, self.mn, self.mode = addr, op, mn, mode
        self.size, self.operand, self.cyc, self.m, self.x = size, operand, cyc, m, x

    def next_addr(self):
        return (self.addr & 0xFF0000) | ((self.addr + self.size) & 0xFFFF)

    def branch_target(self):
        nxt = (self.addr + self.size) & 0xFFFF
        if self.mode == REL8:
            off = self.operand - 0x100 if self.operand >= 0x80 else self.operand
        else:
            off = self.operand - 0x10000 if self.operand >= 0x8000 else self.operand
        return (self.addr & 0xFF0000) | ((nxt + off) & 0xFFFF)

    def fmt(self):
        mn, md, o = self.mn, self.mode, self.operand
        osz = self.size - 1
        hx = '$%0*X' % (osz * 2, o) if osz else ''
        if md in (IMP,):
            s = ''
        elif md == ACC:
            s = 'A'
        elif md in (IMM_M, IMM_X, IMM8, IMM16):
            s = '#' + hx
        elif md in (DP, ABS, LONG):
            s = hx
        elif md in (DPX, ABSX, LONGX):
            s = hx + ',X'
        elif md in (DPY, ABSY):
            s = hx + ',Y'
        elif md in (DPI, ABSI):
            s = '(%s)' % hx
        elif md in (DPIX, ABSIX):
            s = '(%s,X)' % hx
        elif md == DPIY:
            s = '(%s),Y' % hx
        elif md in (DPIL, ABSIL):
            s = '[%s]' % hx
        elif md == DPILY:
            s = '[%s],Y' % hx
        elif md == SR:
            s = hx + ',S'
        elif md == SRIY:
            s = '(%s,S),Y' % hx
        elif md in (REL8, REL16):
            s = '$%06X' % self.branch_target()
        elif md == BLK:
            s = '$%02X,$%02X' % (o & 0xFF, o >> 8)
        else:
            s = '?'
        return ('%s %s' % (mn, s)).strip()


def decode(read8, addr, m, x):
    """Decode one instruction at 24-bit addr. read8(addr24) -> int or None."""
    op = read8(addr)
    if op is None:
        return None
    mn, mode, cyc = OPS[op]
    if mode == IMM_M:
        n = 1 if m else 2
    elif mode == IMM_X:
        n = 1 if x else 2
    else:
        n = MODE_LEN[mode]
    operand = 0
    bank = addr & 0xFF0000
    for i in range(n):
        b = read8(bank | ((addr + 1 + i) & 0xFFFF))
        if b is None:
            return None
        operand |= b << (8 * i)
    return Insn(addr, op, mn, mode, 1 + n, operand, cyc, m, x)
