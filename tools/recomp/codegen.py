"""C code generation for the 65816 static recompiler.

The same instruction-semantics emitter produces both
  * the recompiled game code (operands/addresses are compile-time constants), and
  * the fallback interpreter (operands are fetched at run time),
so the two execution paths share timing and semantics by construction.
"""
from w65816 import (OPS, BRANCHES, TERMINATORS, IMP, ACC, IMM_M, IMM_X, IMM8, IMM16,
                    DP, DPX, DPY, DPI, DPIX, DPIY, DPIL, DPILY, ABS, ABSX, ABSY, LONG,
                    LONGX, ABSI, ABSIX, ABSIL, SR, SRIY, REL8, REL16, BLK, MODE_LEN)
from analysis import FS_E, mx, fs_of, FS_NAMES

ACC_DATA = {'ORA', 'AND', 'EOR', 'ADC', 'STA', 'LDA', 'CMP', 'SBC', 'BIT'}
RMW = {'ASL', 'LSR', 'ROL', 'ROR', 'INC', 'DEC', 'TSB', 'TRB'}
IDX_DATA = {'LDX', 'LDY', 'STX', 'STY', 'CPX', 'CPY'}
BRANCH_TAKEN_MCLK = 6

BRANCH_COND = {
    'BPL': '!cpu.fn', 'BMI': 'cpu.fn', 'BVC': '!cpu.fv', 'BVS': 'cpu.fv',
    'BCC': '!cpu.fc', 'BCS': 'cpu.fc', 'BNE': '!cpu.fz', 'BEQ': 'cpu.fz',
}


def mclk_cost(mn, mode, cyc, m, x, e):
    """Approximate master-clock cost (SlowROM: 8 per memory cycle, 6 per internal cycle)."""
    c = cyc
    if mn in ACC_DATA and not m:
        c += 1
    if mn in RMW and mode != ACC and not m:
        c += 2
    if mn == 'STZ' and not m:
        c += 1
    if mn in IDX_DATA and not x:
        c += 1
    if mn in ('PHA', 'PLA') and not m:
        c += 1
    if mn in ('PHX', 'PHY', 'PLX', 'PLY') and not x:
        c += 1
    if mn == 'RTI' and not e:
        c += 1
    if mode in (ABSX, ABSY, DPIY) and not x and mn not in RMW and mn not in ('STA', 'STZ'):
        c += 1
    io = 0
    if mode in (IMP, ACC):
        io = 1
    if mn in ('XBA', 'PLA', 'PLX', 'PLY', 'PLB', 'PLD', 'PLP', 'RTL', 'RTI'):
        io = 2
    if mn == 'RTS':
        io = 3
    if mode in (DPX, DPY, DPIX, SR):
        io += 1
    if mode == SRIY:
        io += 2
    if mn in RMW and mode != ACC:
        io += 1
    if mn in ('JSR', 'JSL', 'PER', 'REP', 'SEP', 'MVN', 'MVP'):
        io += 1 if mn not in ('MVN', 'MVP') else 2
    return c * 8 - io * 2


class Ctx:
    """Expressions for the current instruction: operand value, its 16-bit
    address, the 24-bit code bank base, and its byte size."""

    def __init__(self, op, pc, bank, size, static=None):
        self.op, self.pc, self.bank, self.size = op, pc, bank, size
        self.static = static  # Insn when generating recompiled code

    def pc_plus(self, n):
        if self.static is not None:
            return '0x%04X' % ((self.static.addr + n) & 0xFFFF)
        return '(uint16_t)(%s + %d)' % (self.pc, n)


def ea_expr(mode, op):
    if mode == DP:
        return '(uint16_t)(cpu.dp + %s)' % op, 'b0'
    if mode == DPX:
        return '(uint16_t)(cpu.dp + %s + cpu.x)' % op, 'b0'
    if mode == DPY:
        return '(uint16_t)(cpu.dp + %s + cpu.y)' % op, 'b0'
    if mode == DPI:
        return 'DBADDR(rd16_b0((uint16_t)(cpu.dp + %s)))' % op, 'l'
    if mode == DPIX:
        return 'DBADDR(rd16_b0((uint16_t)(cpu.dp + %s + cpu.x)))' % op, 'l'
    if mode == DPIY:
        return '((DBADDR(rd16_b0((uint16_t)(cpu.dp + %s))) + cpu.y) & 0xFFFFFF)' % op, 'l'
    if mode == DPIL:
        return 'rd24_b0((uint16_t)(cpu.dp + %s))' % op, 'l'
    if mode == DPILY:
        return '((rd24_b0((uint16_t)(cpu.dp + %s)) + cpu.y) & 0xFFFFFF)' % op, 'l'
    if mode == ABS:
        return 'DBADDR(%s)' % op, 'l'
    if mode == ABSX:
        return '((DBADDR(%s) + cpu.x) & 0xFFFFFF)' % op, 'l'
    if mode == ABSY:
        return '((DBADDR(%s) + cpu.y) & 0xFFFFFF)' % op, 'l'
    if mode == LONG:
        return '(uint32_t)(%s)' % op, 'l'
    if mode == LONGX:
        return '(((uint32_t)(%s) + cpu.x) & 0xFFFFFF)' % op, 'l'
    if mode == SR:
        return '(uint16_t)(cpu.s + %s)' % op, 'b0'
    if mode == SRIY:
        return '((DBADDR(rd16_b0((uint16_t)(cpu.s + %s))) + cpu.y) & 0xFFFFFF)' % op, 'l'
    raise ValueError(mode)


def rd_expr(w, kind):
    if w == 8:
        return 'rd8(ea)'
    return 'rd16_b0((uint16_t)ea)' if kind == 'b0' else 'rd16(ea)'


def wr_stmt(w, kind, v, rmw=False):
    if w == 8:
        return 'wr8(ea, (uint8_t)(%s));' % v
    if rmw:
        return ('wr16_rmw_b0((uint16_t)ea, (uint16_t)(%s));' if kind == 'b0' else 'wr16_rmw(ea, (uint16_t)(%s));') % v
    return ('wr16_b0((uint16_t)ea, (uint16_t)(%s));' if kind == 'b0' else 'wr16(ea, (uint16_t)(%s));') % v


IMM_MODES = (IMM_M, IMM_X, IMM8, IMM16)


def emit_body(mn, mode, m, x, e, C):
    """Statements for a non-terminator instruction (cycles excluded)."""
    W = 8 if m else 16
    XW = 8 if x else 16
    T = 'uint8_t' if W == 8 else 'uint16_t'
    XT = 'uint8_t' if XW == 8 else 'uint16_t'
    sfx = '8' if W == 8 else '16'
    xsfx = '8' if XW == 8 else '16'

    def load(width):
        if mode in IMM_MODES:
            return [], C.op
        ex, kind = ea_expr(mode, C.op)
        return ['uint32_t ea = %s;' % ex], rd_expr(width, kind)

    def store(width, v):
        ex, kind = ea_expr(mode, C.op)
        return ['uint32_t ea = %s;' % ex, wr_stmt(width, kind, v)]

    L = []
    if mn == 'LDA':
        s, ex = load(W)
        L = s + ['%s v = (%s)%s;' % (T, T, ex)] + (['SET_A8(v); NZ8(v);'] if W == 8 else ['cpu.a = v; NZ16(v);'])
    elif mn in ('LDX', 'LDY'):
        r = 'cpu.x' if mn == 'LDX' else 'cpu.y'
        s, ex = load(XW)
        L = s + ['%s v = (%s)%s;' % (XT, XT, ex), '%s = v; NZ%s(v);' % (r, xsfx)]
    elif mn in ('STA', 'STX', 'STY', 'STZ'):
        if mn == 'STA':
            L = store(W, 'cpu.a')
        elif mn == 'STZ':
            L = store(W, '0')
        else:
            L = store(XW, 'cpu.x' if mn == 'STX' else 'cpu.y')
    elif mn in ('ORA', 'AND', 'EOR'):
        o = {'ORA': '|', 'AND': '&', 'EOR': '^'}[mn]
        s, ex = load(W)
        if W == 8:
            L = s + ['uint8_t v = (uint8_t)%s;' % ex, 'SET_A8(A8 %s v); NZ8(A8);' % o]
        else:
            L = s + ['uint16_t v = (uint16_t)%s;' % ex, 'cpu.a = (uint16_t)(cpu.a %s v); NZ16(cpu.a);' % o]
    elif mn in ('ADC', 'SBC'):
        s, ex = load(W)
        L = s + ['op_%s%s((%s)%s);' % (mn.lower(), sfx, T, ex)]
    elif mn in ('CMP', 'CPX', 'CPY'):
        if mn == 'CMP':
            s, ex = load(W)
            L = s + ['op_cmp%s(%s, (%s)%s);' % (sfx, 'A8' if W == 8 else 'cpu.a', T, ex)]
        else:
            r = 'cpu.x' if mn == 'CPX' else 'cpu.y'
            s, ex = load(XW)
            L = s + ['op_cmp%s((%s)%s, (%s)%s);' % (xsfx, XT, r, XT, ex)]
    elif mn == 'BIT':
        s, ex = load(W)
        if mode in IMM_MODES:
            L = ['cpu.fz = (((%s)cpu.a) & (%s)%s) == 0;' % (T, T, ex)]
        else:
            L = s + ['op_bit%s((%s)%s);' % (sfx, T, ex)]
    elif mn in RMW:
        fn = 'op_%s%s' % (mn.lower(), sfx)
        if mode == ACC:
            L = ['SET_A8(%s(A8));' % fn] if W == 8 else ['cpu.a = %s(cpu.a);' % fn]
        else:
            ex, kind = ea_expr(mode, C.op)
            L = ['uint32_t ea = %s;' % ex, '%s v = (%s)%s;' % (T, T, rd_expr(W, kind)),
                 'v = %s(v);' % fn, wr_stmt(W, kind, 'v', rmw=True)]
    elif mn in ('INX', 'INY', 'DEX', 'DEY'):
        r = 'cpu.x' if mn[2] == 'X' else 'cpu.y'
        f = 'op_inc' if mn[0] == 'I' else 'op_dec'
        L = ['%s = %s%s((%s)%s);' % (r, f, xsfx, XT, r)]
    elif mn in ('TAX', 'TAY'):
        r = 'cpu.x' if mn == 'TAX' else 'cpu.y'
        L = ['%s = (%s)cpu.a; NZ%s(%s);' % (r, XT, xsfx, r)]
    elif mn in ('TXA', 'TYA'):
        r = 'cpu.x' if mn == 'TXA' else 'cpu.y'
        L = ['SET_A8(%s); NZ8(A8);' % r] if W == 8 else ['cpu.a = %s; NZ16(cpu.a);' % r]
    elif mn in ('TXY', 'TYX'):
        d, s_ = ('cpu.y', 'cpu.x') if mn == 'TXY' else ('cpu.x', 'cpu.y')
        L = ['%s = (%s)%s; NZ%s(%s);' % (d, XT, s_, xsfx, d)]
    elif mn == 'TSX':
        L = ['cpu.x = (%s)cpu.s; NZ%s(cpu.x);' % (XT, xsfx)]
    elif mn == 'TXS':
        L = ['cpu.s = (uint16_t)(0x100 | (uint8_t)cpu.x);'] if e else ['cpu.s = cpu.x;']
    elif mn == 'TCS':
        L = ['cpu.s = (uint16_t)(0x100 | (uint8_t)cpu.a);'] if e else ['cpu.s = cpu.a;']
    elif mn == 'TSC':
        L = ['cpu.a = cpu.s; NZ16(cpu.a);']
    elif mn == 'TCD':
        L = ['cpu.dp = cpu.a; NZ16(cpu.dp);']
    elif mn == 'TDC':
        L = ['cpu.a = cpu.dp; NZ16(cpu.a);']
    elif mn == 'XBA':
        L = ['cpu.a = (uint16_t)(cpu.a >> 8 | cpu.a << 8); NZ8(A8);']
    elif mn == 'PHA':
        L = ['push8(A8);'] if W == 8 else ['push16(cpu.a);']
    elif mn in ('PHX', 'PHY'):
        r = 'cpu.x' if mn == 'PHX' else 'cpu.y'
        L = ['push8((uint8_t)%s);' % r] if XW == 8 else ['push16(%s);' % r]
    elif mn == 'PHB':
        L = ['push8(cpu.db);']
    elif mn == 'PHK':
        L = ['push8(cpu.pb);']
    elif mn == 'PHD':
        L = ['push16(cpu.dp);']
    elif mn == 'PHP':
        L = ['push8(cpu_get_p());']
    elif mn == 'PLA':
        L = ['uint8_t v = pull8(); SET_A8(v); NZ8(v);'] if W == 8 else ['uint16_t v = pull16(); cpu.a = v; NZ16(v);']
    elif mn in ('PLX', 'PLY'):
        r = 'cpu.x' if mn == 'PLX' else 'cpu.y'
        L = ['%s = pull8(); NZ8(%s);' % (r, r)] if XW == 8 else ['%s = pull16(); NZ16(%s);' % (r, r)]
    elif mn == 'PLB':
        L = ['cpu.db = pull8(); NZ8(cpu.db);']
    elif mn == 'PLD':
        L = ['cpu.dp = pull16(); NZ16(cpu.dp);']
    elif mn == 'PEA':
        L = ['push16((uint16_t)(%s));' % C.op]
    elif mn == 'PEI':
        L = ['push16(rd16_b0((uint16_t)(cpu.dp + %s)));' % C.op]
    elif mn == 'PER':
        L = ['push16((uint16_t)(%s + %s));' % (C.pc_plus(3), C.op)]
    elif mn in ('CLC', 'SEC', 'CLI', 'SEI', 'CLD', 'SED', 'CLV'):
        f = {'C': 'cpu.fc', 'I': 'cpu.fi', 'D': 'cpu.fd', 'V': 'cpu.fv'}[mn[2]]
        L = ['%s = %d;' % (f, 1 if mn[0] == 'S' else 0)]
    elif mn in ('REP', 'SEP'):
        if C.static is not None:
            v = C.static.operand
            bit = 1 if mn == 'SEP' else 0
            names = [(0x80, 'cpu.fn'), (0x40, 'cpu.fv'), (0x08, 'cpu.fd'), (0x04, 'cpu.fi'),
                     (0x02, 'cpu.fz'), (0x01, 'cpu.fc')]
            if not e:
                names = [(0x20, 'cpu.fm'), (0x10, 'cpu.fx')] + names
            for mask, f in names:
                if v & mask:
                    L.append('%s = %d;' % (f, bit))
            if mn == 'SEP' and (v & 0x10) and not e:
                L.append('cpu.x &= 0xFF; cpu.y &= 0xFF;')
        else:
            if mn == 'REP':
                L = ['cpu_set_p((uint8_t)(cpu_get_p() & ~%s));' % C.op]
            else:
                L = ['cpu_set_p((uint8_t)(cpu_get_p() | %s));' % C.op]
    elif mn == 'NOP':
        L = []
    else:
        raise ValueError('no body for ' + mn)
    return L


class Emitter:
    """Collects C lines with indentation."""

    def __init__(self):
        self.lines = []

    def __call__(self, s, ind=1):
        self.lines.append('  ' * ind + s)


def emit_terminator(mn, mode, m, x, e, C, out, edge, exit_):
    """Emit control flow for a terminator.

    edge(target_pc_expr, target_bank_or_None, static_target_or_None, fs) -> list of lines
       transfer to a known location (may become a goto in recompiled code)
    exit_() -> list of lines for 'return to dispatcher' (cpu.pc/pb already set)
    """
    st = C.static
    nxt = C.pc_plus(C.size)
    fs_t = fs_of(m, x, e)
    if mn in BRANCHES:
        if st is not None:
            tgt = st.branch_target()
            out('if (%s) {' % BRANCH_COND[mn])
            out('CYC(%d);' % BRANCH_TAKEN_MCLK, 2)
            for l in edge('0x%04X' % (tgt & 0xFFFF), None, tgt, fs_t):
                out(l, 2)
            out('}')
            for l in edge(nxt, None, st.next_addr(), fs_t):
                out(l)
        else:
            if mode == REL8:
                off = '(int8_t)%s' % C.op
            else:
                off = '(int16_t)%s' % C.op
            out('if (%s) { CYC(%d); cpu.pc = (uint16_t)(%s + %s); }' % (BRANCH_COND[mn], BRANCH_TAKEN_MCLK, nxt, off))
            out('else cpu.pc = %s;' % nxt)
            for l in exit_():
                out(l)
        return
    if mn in ('BRA', 'BRL'):
        if st is not None:
            tgt = st.branch_target()
            for l in edge('0x%04X' % (tgt & 0xFFFF), None, tgt, fs_t):
                out(l)
        else:
            off = '(int8_t)%s' % C.op if mode == REL8 else '(int16_t)%s' % C.op
            out('cpu.pc = (uint16_t)(%s + %s);' % (nxt, off))
            for l in exit_():
                out(l)
        return
    if mn == 'JMP' and mode == ABS:
        if st is not None:
            tgt = (st.addr & 0xFF0000) | st.operand
            for l in edge('0x%04X' % st.operand, None, tgt, fs_t):
                out(l)
        else:
            out('cpu.pc = (uint16_t)%s;' % C.op)
            for l in exit_():
                out(l)
        return
    if mn == 'JML' and mode == LONG:
        if st is not None:
            tgt = st.operand
            for l in edge('0x%04X' % (tgt & 0xFFFF), '0x%02X' % (tgt >> 16), tgt, fs_t):
                out(l)
        else:
            out('cpu.pc = (uint16_t)%s; cpu.pb = (uint8_t)(%s >> 16);' % (C.op, C.op))
            for l in exit_():
                out(l)
        return
    if mn == 'JMP' and mode == ABSI:
        out('cpu.pc = rd16_b0((uint16_t)(%s));' % C.op)
    elif mn == 'JMP' and mode == ABSIX:
        out('{ uint16_t p = (uint16_t)(%s + cpu.x); uint8_t lo = rd8(%s | p);' % (C.op, C.bank))
        out('  cpu.pc = (uint16_t)(lo | rd8(%s | (uint16_t)(p + 1)) << 8); }' % C.bank)
    elif mn == 'JML' and mode == ABSIL:
        out('{ uint32_t t = rd24_b0((uint16_t)(%s)); cpu.pc = (uint16_t)t; cpu.pb = (uint8_t)(t >> 16); }' % C.op)
    elif mn == 'JSR' and mode == ABS:
        out('push16(%s);' % C.pc_plus(2))
        if st is not None:
            out('cpu.pc = 0x%04X;' % st.operand)
        else:
            out('cpu.pc = (uint16_t)%s;' % C.op)
    elif mn == 'JSR' and mode == ABSIX:
        out('push16(%s);' % C.pc_plus(2))
        out('{ uint16_t p = (uint16_t)(%s + cpu.x); uint8_t lo = rd8(%s | p);' % (C.op, C.bank))
        out('  cpu.pc = (uint16_t)(lo | rd8(%s | (uint16_t)(p + 1)) << 8); }' % C.bank)
    elif mn == 'JSL':
        out('push8(cpu.pb); push16(%s);' % C.pc_plus(3))
        if st is not None:
            out('cpu.pb = 0x%02X; cpu.pc = 0x%04X;' % (st.operand >> 16, st.operand & 0xFFFF))
        else:
            out('cpu.pb = (uint8_t)(%s >> 16); cpu.pc = (uint16_t)%s;' % (C.op, C.op))
    elif mn == 'RTS':
        out('cpu.pc = (uint16_t)(pull16() + 1);')
    elif mn == 'RTL':
        out('cpu.pc = (uint16_t)(pull16() + 1); cpu.pb = pull8();')
    elif mn == 'RTI':
        if e:
            out('cpu_set_p(pull8()); cpu.pc = pull16();')
        else:
            out('cpu_set_p(pull8()); cpu.pc = pull16(); cpu.pb = pull8();')
    elif mn == 'PLP':
        out('cpu_set_p(pull8()); cpu.pc = %s;' % nxt)
    elif mn == 'XCE':
        out('op_xce(); cpu.pc = %s;' % nxt)
    elif mn == 'WAI':
        out('cpu.waiting = 1; cpu.pc = %s;' % nxt)
    elif mn == 'STP':
        out('cpu.stopped = 1; cpu.pc = %s;' % nxt)
    elif mn == 'WDM':
        out('cpu.pc = %s;' % nxt)
    elif mn in ('BRK', 'COP'):
        out('cpu.pc = %s;' % C.pc_plus(2))
        if mn == 'BRK':
            out('cpu_interrupt(0xFFE6, 0xFFFE, true);')
        else:
            out('cpu_interrupt(0xFFE4, 0xFFF4, false);')
    elif mn in ('MVN', 'MVP'):
        f = 'op_mvn_step' if mn == 'MVN' else 'op_mvp_step'
        dst = '(uint8_t)(%s)' % C.op
        src = '(uint8_t)((%s) >> 8)' % C.op
        if st is not None:
            out('if (%s(%s, %s)) {' % (f, dst, src))
            for l in edge(nxt, None, st.next_addr(), fs_t):
                out(l, 2)
            out('}')
            for l in edge('0x%04X' % (st.addr & 0xFFFF), None, st.addr, fs_t):
                out(l)
            return
        out('if (%s(%s, %s)) cpu.pc = %s; else cpu.pc = %s;' % (f, dst, src, nxt, C.pc))
    else:
        raise ValueError('terminator ' + mn + ' ' + mode)
    for l in exit_():
        out(l)


def fmt_comment(ins):
    return '%02X:%04X  %s' % (ins.addr >> 16, ins.addr & 0xFFFF, ins.fmt())


def label(key):
    return 'L_%06X_%d' % key


def region_name(root):
    return 'rg_%06X_%s' % (root[0], FS_NAMES[root[1]])


def gen_region(an, root, members, owner, rid):
    out = Emitter()
    name = region_name(root)
    out('void %s(uint32_t entry) {' % name, 0)
    out('switch (entry) {')
    for i, k in enumerate(members):
        out('case %d: goto %s;' % (i, label(k)), 2)
    out('default: return;', 2)
    out('}')
    for k in members:
        b = an.blocks[k]
        out('%s:  // %s' % (label(k), FS_NAMES[k[1]]), 0)
        if b.bad:
            out('cpu.pc = 0x%04X; cpu.pb = 0x%02X; return;  // undecodable' % (k[0] & 0xFFFF, k[0] >> 16))
            continue

        def edge(pc_expr, bank_expr, static_tgt, fs, b=b):
            tk = (an.canon(static_tgt), fs)
            res = []
            setpb = ' cpu.pb = %s;' % bank_expr if bank_expr is not None else ''
            if owner.get(tk) == rid:
                if setpb:
                    res.append(setpb.strip())
                res.append('if (EVENT_DUE()) { cpu.pc = %s; return; }' % pc_expr)
                res.append('goto %s;' % label(tk))
            else:
                res.append('cpu.pc = %s;%s return;' % (pc_expr, setpb))
            return res

        def exit_():
            return ['return;']

        for ins, m, x, e in b.insns:
            out('// %s' % fmt_comment(ins))
            mn, mode = ins.mn, ins.mode
            cost = mclk_cost(mn, mode, ins.cyc, m, x, e)
            C = Ctx('0x%X' % ins.operand, '0x%04X' % (ins.addr & 0xFFFF),
                    '0x%06X' % (ins.addr & 0xFF0000), ins.size, static=ins)
            out('CYC(%d);' % cost)
            if mn in TERMINATORS:
                emit_terminator(mn, mode, m, x, e, C, out, edge, exit_)
            else:
                body = emit_body(mn, mode, m, x, e, C)
                if body:
                    out('{ ' + ' '.join(body) + ' }')
    out('}', 0)
    return out.lines


def gen_interpreter():
    """Fallback interpreter: executes one instruction at cpu.pb:cpu.pc.
    Returns 1 if the instruction ended a block (dispatcher boundary)."""
    out = Emitter()
    out('int interp_step(void) {', 0)
    out('uint16_t ipc = cpu.pc;')
    out('uint32_t bank = (uint32_t)cpu.pb << 16;')
    out('uint8_t opc = fetch8(bank | ipc);')
    out('uint32_t opnd;')
    out('(void)opnd;')
    out('switch ((cpu_fs() << 8) | opc) {')
    for fs in range(5):
        m, x = mx(fs)
        e = fs == FS_E
        for op in range(256):
            mn, mode, cyc = OPS[op]
            if mode == IMM_M:
                n = 1 if m else 2
            elif mode == IMM_X:
                n = 1 if x else 2
            else:
                n = MODE_LEN[mode]
            out('case 0x%03X: {  // %s %s %s' % ((fs << 8) | op, FS_NAMES[fs], mn, mode), 1)
            if n == 1:
                out('opnd = fetch8(bank | (uint16_t)(ipc + 1));', 2)
            elif n == 2:
                out('opnd = fetch8(bank | (uint16_t)(ipc + 1)) | (uint32_t)fetch8(bank | (uint16_t)(ipc + 2)) << 8;', 2)
            elif n == 3:
                out('opnd = fetch8(bank | (uint16_t)(ipc + 1)) | (uint32_t)fetch8(bank | (uint16_t)(ipc + 2)) << 8'
                    ' | (uint32_t)fetch8(bank | (uint16_t)(ipc + 3)) << 16;', 2)
            out('CYC(%d);' % mclk_cost(mn, mode, cyc, m, x, e), 2)
            C = Ctx('opnd', 'ipc', 'bank', 1 + n)
            if mn in TERMINATORS:
                lines = Emitter()
                emit_terminator(mn, mode, m, x, e, C, lines, None, lambda: ['return 1;'])
                for l in lines.lines:
                    out(l.strip(), 2)
            else:
                body = emit_body(mn, mode, m, x, e, C)
                if body:
                    out('{ ' + ' '.join(body) + ' }', 2)
                out('cpu.pc = (uint16_t)(ipc + %d);' % (1 + n), 2)
                out('return 0;', 2)
            out('}', 1)
    out('}')
    out('return 1;')
    out('}', 0)
    return out.lines
