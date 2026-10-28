"""Control-flow discovery for the 65816 static recompiler.

A *block* is keyed by (addr24, fs) where fs encodes the M/X/E state the code
is decoded/compiled under:
    0 = m0x0, 1 = m0x1, 2 = m1x0, 3 = m1x1, 4 = emulation mode
Blocks run until a terminator (branch / jump / call / return / flag-unknowable
instruction).  The runtime dispatcher selects the variant by the *actual*
flags, so static flag tracking only affects coverage, never correctness.
"""
from w65816 import decode, BRANCHES, TERMINATORS, REL8, REL16

FS_E = 4
FS_NAMES = ['m0x0', 'm0x1', 'm1x0', 'm1x1', 'e']
ALL_NATIVE = (0, 1, 2, 3)


def mx(fs):
    return (1, 1) if fs == FS_E else ((fs >> 1) & 1, fs & 1)


def fs_of(m, x, e=False):
    return FS_E if e else (m << 1) | x


EXEC_PTR = 0x0086DF       # JSL ExecutePtr     ; dw ptr, ptr, ...
EXEC_PTR_LONG = 0x0086FA  # JSL ExecutePtrLong ; dl ptr, ptr, ...
MAX_BLOCK_INSNS = 3000
MAX_TABLE = 256
MAX_CTX = 8          # PHP contexts tracked per block
MAX_CTX_DEPTH = 4    # PHP stack depth tracked


class Block:
    __slots__ = ('addr', 'fs', 'insns', 'kind', 'edges', 'call', 'cont', 'fs_out',
                 'ctx', 'indirect', 'bad', 'table_edges', 'origin',
                 'pushes', 'static_succ', 'plp_next', 'ctxs', 'edge_set', 'cont_ctx')

    def __init__(self, addr, fs, ctx, origin):
        self.addr, self.fs, self.ctx, self.origin = addr, fs, ctx, origin
        self.insns = []        # list of (Insn, m, x, e)
        self.kind = None       # terminator mnemonic
        self.edges = []        # local successor keys (addr, fs)
        self.table_edges = []  # subset of edges coming from jump tables
        self.call = None       # (callee_key) for JSR/JSL
        self.cont = None       # return continuation address for calls
        self.fs_out = fs
        self.indirect = False
        self.bad = False

    @property
    def key(self):
        return (self.addr, self.fs)

    @property
    def term(self):
        return self.insns[-1][0] if self.insns else None


class Analyzer:
    def __init__(self, rom, log=print):
        self.rom = rom
        self.log = log
        self.blocks = {}
        self.queue = []
        self.queued = set()
        self.roots = {}        # key -> reason ('reset', 'nmi', 'call', 'table', 'seed', ...)
        self.cont_seeds = {}   # (block key of caller) -> set of fs predicted for continuation
        self.summary = {}      # callee key -> frozenset of exit fs
        self.nonrom_targets = set()
        self.code_starts = set()  # addresses known to start code (for table bounding)

    # ------------------------------------------------------------------ util
    def canon(self, addr):
        b = addr >> 16
        if 0x80 <= b < 0xFE:
            return addr & 0x7FFFFF
        return addr

    def is_rom(self, addr):
        return self.rom.read8(addr) is not None

    def enqueue(self, addr, fs, ctx=(), origin=None):
        addr = self.canon(addr)
        if not self.is_rom(addr):
            self.nonrom_targets.add((addr, fs))
            return None
        key = (addr, fs)
        if (key, ctx) in self.queued:
            return key
        self.queued.add((key, ctx))
        self.queue.append((addr, fs, ctx, origin))
        return key

    def add_root(self, addr, fs, reason, ctx=()):
        key = self.enqueue(addr, fs, ctx, reason)
        if key is not None and key not in self.roots:
            self.roots[key] = reason
        return key

    # ---------------------------------------------------------- jump tables
    def parse_table(self, table_addr, long_entries):
        """Heuristic jump table parse: stop at a non-ROM pointer, at the lowest
        same-bank target that lies after the table, or at known code."""
        bank = table_addr & 0xFF0000
        out = []
        limit = None
        a = table_addr
        step = 3 if long_entries else 2
        holes = 0
        for i in range(MAX_TABLE):
            if limit is not None and a >= limit:
                break
            if i > 0 and a in self.code_starts:
                break
            if long_entries:
                v = self.rom.read24(a)
                if v is None:
                    break
                t = self.canon(v)
            else:
                v = self.rom.read16(a)
                if v is None:
                    break
                t = bank | v
            if v == 0 and holes < 32:
                # unused slot (null pointer) inside a table
                holes += 1
                a = (a & 0xFF0000) | ((a + step) & 0xFFFF)
                continue
            if (t & 0xFFFF) < 0x8000 or not self.is_rom(t):
                break
            holes = 0
            out.append(t)
            if (t & 0xFF0000) == bank and t > table_addr and (limit is None or t < limit):
                limit = t
            na = (a & 0xFF0000) | ((a + step) & 0xFFFF)
            if na < a:
                break
            a = na
        return out

    # ------------------------------------------------------------- decoding
    def decode_block(self, addr, fs, origin):
        """Decode a block.  Successors that do not depend on the PHP/PLP
        context go to b.static_succ; a PLP terminator is resolved per context
        in run()."""
        b = Block(addr, fs, (), origin)
        b.pushes = []
        b.static_succ = []
        m, x = mx(fs)
        e = fs == FS_E
        pc = addr
        prev = None
        for _ in range(MAX_BLOCK_INSNS):
            ins = decode(self.rom.read8, pc, m, x)
            if ins is None:
                b.bad = True
                b.kind = 'BAD'
                break
            b.insns.append((ins, m, x, e))
            mn = ins.mn
            if mn == 'REP' and not e:
                if ins.operand & 0x20: m = 0
                if ins.operand & 0x10: x = 0
            elif mn == 'SEP' and not e:
                if ins.operand & 0x20: m = 1
                if ins.operand & 0x10: x = 1
            elif mn == 'PHP':
                b.pushes.append(fs_of(m, x, e))
            if mn in TERMINATORS:
                b.kind = mn
                break
            prev = ins
            pc = ins.next_addr()
            if (pc & 0xFFFF) < 0x8000 and (addr & 0xFFFF) >= 0x8000:
                b.bad = True   # ran off the end of the bank
                b.kind = 'BAD'
                break
        else:
            b.bad = True      # absurdly long straight-line run: data, not code
            b.kind = 'BAD'
        fs_t = fs_of(m, x, e)
        b.fs_out = fs_t
        if b.bad or not b.insns:
            return b
        t = b.insns[-1][0]
        nxt = t.next_addr()
        mn = b.kind
        succ = b.static_succ        # (addr, fs, is_table)
        if mn in BRANCHES:
            succ.append((t.branch_target(), fs_t, False))
            succ.append((nxt, fs_t, False))
        elif mn in ('BRA', 'BRL'):
            succ.append((t.branch_target(), fs_t, False))
        elif mn in ('JMP', 'JML') and t.mode in ('abs', 'long') and len(b.insns) >= 2 and b.insns[-2][0].mn == 'PER':
            # "PER ret-1 ; JMP/JML target" is a hand-rolled call: the pushed address
            # is the return point (minus one), exactly as JSR/JSL would push it.
            per = b.insns[-2][0]
            ret = (per.addr & 0xFF0000) | ((per.addr + 3 + per.operand) & 0xFFFF)
            callee = self.canon((t.addr & 0xFF0000) | t.operand if mn == 'JMP' else t.operand)
            cont = (ret & 0xFF0000) | ((ret + 1) & 0xFFFF)
            if callee in (EXEC_PTR, EXEC_PTR_LONG):
                self.add_root(callee, fs_t, 'execptr')
                for tgt in self.parse_table(cont, callee == EXEC_PTR_LONG):
                    succ.append((tgt, 3, True))
                b.call = (callee, fs_t)
            else:
                b.call = (callee, fs_t)
                b.cont = cont
                self.add_root(callee, fs_t, 'call')
        elif mn == 'JMP' and t.mode == 'abs':
            succ.append(((t.addr & 0xFF0000) | t.operand, fs_t, False))
        elif mn == 'JML' and t.mode == 'long':
            succ.append((t.operand, fs_t, False))
        elif mn == 'JMP' and t.mode == 'absix':
            for tgt in self.parse_table((t.addr & 0xFF0000) | t.operand, False):
                succ.append((tgt, fs_t, True))
            b.indirect = True
        elif mn in ('JMP', 'JML'):
            b.indirect = True   # through RAM pointer: runtime only
        elif mn == 'JSR' or mn == 'JSL':
            if mn == 'JSR' and t.mode == 'absix':
                tab = (t.addr & 0xFF0000) | t.operand
                for tgt in self.parse_table(tab, False):
                    self.add_root(tgt, fs_t, 'jsr-table')
                b.call = None
                b.cont = nxt
                b.indirect = True
            else:
                callee = self.canon((t.addr & 0xFF0000) | t.operand if mn == 'JSR' else t.operand)
                if mn == 'JSL' and callee in (EXEC_PTR, EXEC_PTR_LONG):
                    # inline pointer table follows the JSL; the routine tail-jumps
                    # (with SEP #$30) to the selected entry.  No return here.
                    self.add_root(callee, fs_t, 'execptr')
                    for tgt in self.parse_table(nxt, callee == EXEC_PTR_LONG):
                        succ.append((tgt, 3, True))
                    b.call = (callee, fs_t)
                    b.cont = None
                else:
                    b.call = (callee, fs_t)
                    b.cont = nxt
                    self.add_root(callee, fs_t, 'call')
        elif mn == 'PLP':
            b.plp_next = nxt
        elif mn == 'XCE':
            c = None
            if prev is not None and prev.mn in ('CLC', 'SEC'):
                c = 1 if prev.mn == 'SEC' else 0
            if c == 0:
                succ.append((nxt, 3 if e else fs_t, False))
            elif c == 1:
                succ.append((nxt, FS_E, False))
            else:
                succ.append((nxt, FS_E, False))
                succ.append((nxt, 3 if e else fs_t, False))
        elif mn in ('WAI', 'WDM'):
            succ.append((nxt, fs_t, False))
        elif mn in ('MVN', 'MVP'):
            # block moves re-execute themselves until A wraps: needs an entry at the insn
            succ.append((nxt, fs_t, False))
            succ.append((t.addr, fs_t, False))
        # RTS/RTL/RTI/BRK/COP/STP: no static successors
        return b

    def add_edge(self, b, k, is_tab=False):
        if k is not None and k not in b.edge_set:
            b.edge_set.add(k)
            b.edges.append(k)
            if is_tab:
                b.table_edges.append(k)

    def run(self):
        """Worklist over (block, PHP-context).  The PHP context is the static
        stack of flag states pushed by PHP; a PLP continuation is compiled for
        every context that reaches it."""
        n0 = len(self.blocks)
        while self.queue:
            addr, fs, ctx, origin = self.queue.pop()
            key = (addr, fs)
            b = self.blocks.get(key)
            if b is None:
                b = self.decode_block(addr, fs, origin)
                b.ctxs = set()
                b.edge_set = set()
                self.blocks[key] = b
                self.code_starts.add(addr)
            if ctx in b.ctxs or len(b.ctxs) >= MAX_CTX or b.bad:
                continue
            b.ctxs.add(ctx)
            ctx_out = (ctx + tuple(b.pushes))[-MAX_CTX_DEPTH:]
            for (a, f, is_tab) in b.static_succ:
                self.add_edge(b, self.enqueue(a, f, ctx_out, ('edge', key)), is_tab)
            if b.kind == 'PLP':
                if ctx_out:
                    self.add_edge(b, self.enqueue(b.plp_next, ctx_out[-1], ctx_out[:-1], ('plp', key)))
                else:
                    for f in ALL_NATIVE:
                        self.add_edge(b, self.enqueue(b.plp_next, f, (), ('plp', key)))
            if b.cont is not None:
                b.cont_ctx = ctx_out
                for f in self.cont_seeds.get(key, {b.fs_out}):
                    self.enqueue(b.cont, f, ctx_out, ('cont', key))
        return len(self.blocks) - n0

    def cont_ctx(self, b):
        return getattr(b, 'cont_ctx', ())

    # ------------------------------------------------- return-state summary
    def cont_keys(self, b):
        """Continuation keys of a call block given current summaries."""
        if b.cont is None:
            return []
        fss = self.cont_fs(b)
        return [(b.cont, f) for f in fss if (b.cont, f) in self.blocks]

    def cont_fs(self, b):
        if b.call is None:
            return {b.fs_out}
        s = self.summary.get(b.call)
        if not s:
            return {b.fs_out}
        return set(s)

    def compute_exits(self, root):
        seen = set()
        stack = [root]
        exits = set()
        while stack:
            k = stack.pop()
            if k in seen:
                continue
            seen.add(k)
            b = self.blocks.get(k)
            if b is None or b.bad:
                continue
            if b.kind in ('RTS', 'RTL'):
                exits.add(b.fs_out)
            for e in b.edges:
                stack.append(e)
            stack.extend(self.cont_keys(b))
        return frozenset(exits)

    def solve(self, max_rounds=20):
        """Iterate discovery + interprocedural return-flag summaries."""
        for rnd in range(max_rounds):
            self.run()
            callees = set()
            for b in self.blocks.values():
                if b.call is not None and b.call in self.blocks:
                    callees.add(b.call)
            changed = True
            it = 0
            while changed and it < 50:
                changed = False
                it += 1
                for c in callees:
                    ex = self.compute_exits(c)
                    if ex != self.summary.get(c, frozenset()):
                        self.summary[c] = ex
                        changed = True
            # seed continuations with the computed exit states
            new = 0
            for b in list(self.blocks.values()):
                if b.cont is None:
                    continue
                want = self.cont_fs(b)
                have = self.cont_seeds.get(b.key)
                if have is None or not want <= have:
                    self.cont_seeds[b.key] = (have or set()) | want
                    for f in want:
                        if (b.cont, f) not in self.blocks:
                            new += 1
                        self.enqueue(b.cont, f, self.cont_ctx(b), ('cont', b.key))
            self.log('round %d: %d blocks, %d new continuation seeds' % (rnd, len(self.blocks), new))
            if new == 0 and not self.queue:
                break

    # ---------------------------------------------------- speculative pass
    def speculate(self, rounds=3, fs=3):
        """Compile plausible code found in gaps right after an unconditional
        control transfer (typically routines only reached through pointers held
        in RAM).  A block keyed by (addr, flags) is decoded deterministically, so
        if real control flow ever reaches it with those flags the compiled code
        is exactly right; if it is really data it is simply never dispatched."""
        BAD = ('BRK', 'COP', 'WDM', 'STP')
        total = 0
        for _ in range(rounds):
            cov = set()
            ends = set()
            for b in self.blocks.values():
                for ins, *_ in b.insns:
                    for i in range(ins.size):
                        cov.add(ins.addr + i)
                t = b.term
                if t is not None and t.mn in ('RTS', 'RTL', 'RTI', 'JMP', 'JML', 'BRA', 'BRL'):
                    ends.add(t.next_addr())
            added = 0
            for a in sorted(ends):
                if a in cov or not self.is_rom(a):
                    continue
                pc, m, x, n, ok = a, 1, 1, 0, False
                while n < 200:
                    ins = decode(self.rom.read8, pc, m, x)
                    if ins is None or ins.mn in BAD:
                        break
                    if ins.mn == 'REP':
                        if ins.operand & 0x20: m = 0
                        if ins.operand & 0x10: x = 0
                    elif ins.mn == 'SEP':
                        if ins.operand & 0x20: m = 1
                        if ins.operand & 0x10: x = 1
                    n += 1
                    if ins.mn in ('RTS', 'RTL'):
                        ok = True
                        break
                    pc = ins.next_addr()
                    if (pc & 0xFFFF) < 0x8000:
                        break
                if ok and n >= 3 and (a, fs) not in self.blocks:
                    self.add_root(a, fs, 'speculative')
                    added += 1
            if not added:
                break
            total += added
            self.solve()
        return total

    # ------------------------------------------------------------- regions
    def local_succ(self, b):
        out = list(b.edges)
        if b.cont is not None:
            for f in self.cont_seeds.get(b.key, {b.fs_out}):
                if (b.cont, f) in self.blocks:
                    out.append((b.cont, f))
        return out

    def regions(self, cap=300):
        """Partition blocks into C functions (regions).  Every block belongs to
        exactly one region; all blocks are also dispatcher entry points."""
        owner = {}
        regs = []
        order = sorted(self.roots.keys()) + sorted(self.blocks.keys())
        for root in order:
            if root in owner or root not in self.blocks:
                continue
            members = []
            stack = [root]
            while stack and len(members) < cap:
                k = stack.pop()
                if k in owner or k not in self.blocks:
                    continue
                owner[k] = len(regs)
                members.append(k)
                b = self.blocks[k]
                if b.bad:
                    continue
                for s in reversed(self.local_succ(b)):
                    if s not in owner:
                        stack.append(s)
            members.sort()
            regs.append((root, members))
        return regs, owner
