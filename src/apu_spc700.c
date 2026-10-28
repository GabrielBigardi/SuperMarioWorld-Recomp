// SPC700 CPU interpreter (Sony S-SMP core of the SNES APU).
// Instruction-level emulation: every opcode performs its memory accesses in
// hardware order (including dummy reads that matter for read-sensitive I/O such
// as the timer counters) and returns its cycle count at 1.024 MHz.
#include "apu_internal.h"

#define CPU (apu_state.cpu)

enum { FC = 0x01, FZ = 0x02, FI = 0x04, FH = 0x08, FB = 0x10, FP = 0x20, FV = 0x40, FN = 0x80 };

// Base cycle counts. Taken branches (Bxx, BBS/BBC, CBNE, DBNZ) add 2.
static const uint8_t spc_cycles[256] = {
  //0 1 2 3 4 5 6 7 8 9 A B C D E F
    2,8,4,5,3,4,3,6,2,6,5,4,5,4,6,8, // 0
    2,8,4,5,4,5,5,6,5,5,6,5,2,2,4,6, // 1
    2,8,4,5,3,4,3,6,2,6,5,4,5,4,5,4, // 2
    2,8,4,5,4,5,5,6,5,5,6,5,2,2,3,8, // 3
    2,8,4,5,3,4,3,6,2,6,4,4,5,4,6,6, // 4
    2,8,4,5,4,5,5,6,5,5,4,5,2,2,4,3, // 5
    2,8,4,5,3,4,3,6,2,6,4,4,5,4,5,5, // 6
    2,8,4,5,4,5,5,6,5,5,5,5,2,2,3,6, // 7
    2,8,4,5,3,4,3,6,2,6,5,4,5,2,4,5, // 8
    2,8,4,5,4,5,5,6,5,5,5,5,2,2,12,5,// 9
    3,8,4,5,3,4,3,6,2,6,4,4,5,2,4,4, // A
    2,8,4,5,4,5,5,6,5,5,5,5,2,2,3,4, // B
    3,8,4,5,4,5,4,7,2,5,6,4,5,2,4,9, // C
    2,8,4,5,5,6,6,7,4,5,5,5,2,2,6,3, // D
    2,8,4,5,3,4,3,6,2,4,5,3,4,3,4,3, // E
    2,8,4,5,4,5,5,6,3,4,5,4,2,2,4,3, // F
};

// ---------------------------------------------------------------------------
// Bus helpers
// ---------------------------------------------------------------------------
static inline uint8_t rd(uint16_t addr) { return apu_bus_read(addr); }
static inline void wr(uint16_t addr, uint8_t v) { apu_bus_write(addr, v); }
static inline uint8_t fetch(void) { return apu_bus_read(CPU.pc++); }
static inline uint16_t fetch16(void) {
  uint16_t lo = fetch();
  return (uint16_t)(lo | (fetch() << 8));
}
// Direct page address: page 0 or 1 depending on the P flag.
static inline uint16_t dp(unsigned off) {
  return (uint16_t)(((CPU.psw & FP) << 3) | (off & 0xFF));
}
static inline void push(uint8_t v) { wr((uint16_t)(0x100 | CPU.sp), v); CPU.sp--; }
static inline uint8_t pop(void) { CPU.sp++; return rd((uint16_t)(0x100 | CPU.sp)); }

// Read a little-endian word from the direct page (high byte wraps within the page).
static inline uint16_t rd_dp16(uint8_t off) {
  uint16_t lo = rd(dp(off));
  return (uint16_t)(lo | (rd(dp((uint8_t)(off + 1))) << 8));
}

// ---------------------------------------------------------------------------
// Addressing modes (return effective address)
// ---------------------------------------------------------------------------
static inline uint16_t am_dp(void)   { return dp(fetch()); }
static inline uint16_t am_dpx(void)  { return dp((uint8_t)(fetch() + CPU.x)); }
static inline uint16_t am_dpy(void)  { return dp((uint8_t)(fetch() + CPU.y)); }
static inline uint16_t am_abs(void)  { return fetch16(); }
static inline uint16_t am_absx(void) { return (uint16_t)(fetch16() + CPU.x); }
static inline uint16_t am_absy(void) { return (uint16_t)(fetch16() + CPU.y); }
static inline uint16_t am_ix(void)   { return dp(CPU.x); }
static inline uint16_t am_iy(void)   { return dp(CPU.y); }
static inline uint16_t am_dpxi(void) { return rd_dp16((uint8_t)(fetch() + CPU.x)); }       // [dp+X]
static inline uint16_t am_dpiy(void) { return (uint16_t)(rd_dp16(fetch()) + CPU.y); }      // [dp]+Y

// ---------------------------------------------------------------------------
// ALU
// ---------------------------------------------------------------------------
static inline void set_nz(uint8_t v) {
  CPU.psw = (uint8_t)((CPU.psw & ~(FN | FZ)) | (v & FN) | (v ? 0 : FZ));
}
static inline void set_flag(uint8_t f, int on) {
  CPU.psw = (uint8_t)(on ? (CPU.psw | f) : (CPU.psw & ~f));
}
static uint8_t alu_or(uint8_t a, uint8_t b)  { uint8_t r = a | b; set_nz(r); return r; }
static uint8_t alu_and(uint8_t a, uint8_t b) { uint8_t r = a & b; set_nz(r); return r; }
static uint8_t alu_eor(uint8_t a, uint8_t b) { uint8_t r = a ^ b; set_nz(r); return r; }
static uint8_t alu_cmp(uint8_t a, uint8_t b) {
  int r = a - b;
  set_flag(FC, r >= 0);
  set_nz((uint8_t)r);
  return a;
}
static uint8_t alu_adc(uint8_t a, uint8_t b) {
  int r = a + b + (CPU.psw & FC);
  uint8_t p = (uint8_t)(CPU.psw & ~(FN | FV | FH | FZ | FC));
  if (r > 0xFF) p |= FC;
  if (~(a ^ b) & (a ^ r) & 0x80) p |= FV;
  if ((a ^ b ^ r) & 0x10) p |= FH;
  if (r & 0x80) p |= FN;
  if (!(r & 0xFF)) p |= FZ;
  CPU.psw = p;
  return (uint8_t)r;
}
static uint8_t alu_sbc(uint8_t a, uint8_t b) { return alu_adc(a, (uint8_t)~b); }
static uint8_t alu_asl(uint8_t v) { set_flag(FC, v & 0x80); v = (uint8_t)(v << 1); set_nz(v); return v; }
static uint8_t alu_lsr(uint8_t v) { set_flag(FC, v & 0x01); v >>= 1; set_nz(v); return v; }
static uint8_t alu_rol(uint8_t v) {
  int c = CPU.psw & FC;
  set_flag(FC, v & 0x80);
  v = (uint8_t)((v << 1) | c);
  set_nz(v);
  return v;
}
static uint8_t alu_ror(uint8_t v) {
  int c = CPU.psw & FC;
  set_flag(FC, v & 0x01);
  v = (uint8_t)((v >> 1) | (c << 7));
  set_nz(v);
  return v;
}
static uint8_t alu_inc(uint8_t v) { v++; set_nz(v); return v; }
static uint8_t alu_dec(uint8_t v) { v--; set_nz(v); return v; }

typedef uint8_t (*RmwFn)(uint8_t);

static inline void rmw(uint16_t addr, RmwFn fn) { wr(addr, fn(rd(addr))); }

// Read a register via an addressing mode and set N/Z (MOV reg, mem).
static inline uint8_t ld(uint8_t v) { set_nz(v); return v; }

// Store with the dummy read the hardware performs first.
static inline void st(uint16_t addr, uint8_t v) { rd(addr); wr(addr, v); }

// 16-bit helpers
static inline uint16_t get_ya(void) { return (uint16_t)((CPU.y << 8) | CPU.a); }
static inline void set_ya(uint16_t v) { CPU.a = (uint8_t)v; CPU.y = (uint8_t)(v >> 8); }

// m.b addressing: 13-bit address + 3-bit bit number
static inline uint16_t am_membit(int *bit) {
  uint16_t w = fetch16();
  *bit = w >> 13;
  return w & 0x1FFF;
}

void apu_spc_reset(void) {
  CPU.a = CPU.x = CPU.y = 0;
  CPU.sp = 0xEF;
  CPU.psw = FZ;
  CPU.halted = 0;
  uint16_t lo = rd(0xFFFE);
  CPU.pc = (uint16_t)(lo | (rd(0xFFFF) << 8));
}

// Generic group of the 8-bit ALU ops (OR/AND/EOR/CMP/ADC/SBC) whose opcodes share
// the same layout: base+04..08 and base+14..19, plus base+09 (dp,dp).
#define ALU_GROUP(base, fn, writes)                                                     \
  case base + 0x04: CPU.a = fn(CPU.a, rd(am_dp())); break;                              \
  case base + 0x05: CPU.a = fn(CPU.a, rd(am_abs())); break;                             \
  case base + 0x06: CPU.a = fn(CPU.a, rd(am_ix())); break;                              \
  case base + 0x07: CPU.a = fn(CPU.a, rd(am_dpxi())); break;                            \
  case base + 0x08: CPU.a = fn(CPU.a, fetch()); break;                                  \
  case base + 0x14: CPU.a = fn(CPU.a, rd(am_dpx())); break;                             \
  case base + 0x15: CPU.a = fn(CPU.a, rd(am_absx())); break;                            \
  case base + 0x16: CPU.a = fn(CPU.a, rd(am_absy())); break;                            \
  case base + 0x17: CPU.a = fn(CPU.a, rd(am_dpiy())); break;                            \
  case base + 0x09: { /* dp(dst), dp(src): encoded as op, src, dst */                   \
    uint8_t s = rd(am_dp());                                                            \
    uint16_t d = am_dp();                                                               \
    uint8_t r = fn(rd(d), s);                                                           \
    if (writes) wr(d, r);                                                               \
  } break;                                                                              \
  case base + 0x18: { /* dp, #imm: encoded as op, imm, dp */                            \
    uint8_t s = fetch();                                                                \
    uint16_t d = am_dp();                                                               \
    uint8_t r = fn(rd(d), s);                                                           \
    if (writes) wr(d, r);                                                               \
  } break;                                                                              \
  case base + 0x19: { /* (X), (Y) */                                                    \
    uint8_t s = rd(am_iy());                                                            \
    uint16_t d = am_ix();                                                               \
    uint8_t r = fn(rd(d), s);                                                           \
    if (writes) wr(d, r);                                                               \
  } break;

int apu_spc_step(void) {
  if (CPU.halted)
    return 0;

  uint8_t op = fetch();
  int cyc = spc_cycles[op];

#define BRANCH(cond)                                   \
  {                                                    \
    int8_t rel_ = (int8_t)fetch();                     \
    if (cond) { CPU.pc = (uint16_t)(CPU.pc + rel_); cyc += 2; } \
  }

  switch (op) {
  // ----- 8-bit ALU groups -----
  ALU_GROUP(0x00, alu_or, 1)
  ALU_GROUP(0x20, alu_and, 1)
  ALU_GROUP(0x40, alu_eor, 1)
  ALU_GROUP(0x60, alu_cmp, 0)
  ALU_GROUP(0x80, alu_adc, 1)
  ALU_GROUP(0xA0, alu_sbc, 1)

  // ----- column 0: flags / branches -----
  case 0x00: break;                                   // NOP
  case 0x10: BRANCH(!(CPU.psw & FN)); break;          // BPL
  case 0x20: CPU.psw &= (uint8_t)~FP; break;          // CLRP
  case 0x30: BRANCH(CPU.psw & FN); break;             // BMI
  case 0x40: CPU.psw |= FP; break;                    // SETP
  case 0x50: BRANCH(!(CPU.psw & FV)); break;          // BVC
  case 0x60: CPU.psw &= (uint8_t)~FC; break;          // CLRC
  case 0x70: BRANCH(CPU.psw & FV); break;             // BVS
  case 0x80: CPU.psw |= FC; break;                    // SETC
  case 0x90: BRANCH(!(CPU.psw & FC)); break;          // BCC
  case 0xA0: CPU.psw |= FI; break;                    // EI
  case 0xB0: BRANCH(CPU.psw & FC); break;             // BCS
  case 0xC0: CPU.psw &= (uint8_t)~FI; break;          // DI
  case 0xD0: BRANCH(!(CPU.psw & FZ)); break;          // BNE
  case 0xE0: CPU.psw &= (uint8_t)~(FV | FH); break;   // CLRV
  case 0xF0: BRANCH(CPU.psw & FZ); break;             // BEQ

  // ----- column 1: TCALL n -----
  case 0x01: case 0x11: case 0x21: case 0x31: case 0x41: case 0x51: case 0x61: case 0x71:
  case 0x81: case 0x91: case 0xA1: case 0xB1: case 0xC1: case 0xD1: case 0xE1: case 0xF1: {
    uint16_t vec = (uint16_t)(0xFFDE - ((op >> 4) << 1));
    rd(CPU.pc);  // idle cycle reads the next opcode byte
    push((uint8_t)(CPU.pc >> 8));
    push((uint8_t)CPU.pc);
    uint16_t lo = rd(vec);
    CPU.pc = (uint16_t)(lo | (rd((uint16_t)(vec + 1)) << 8));
  } break;

  // ----- column 2: SET1 / CLR1 dp.bit -----
  case 0x02: case 0x22: case 0x42: case 0x62: case 0x82: case 0xA2: case 0xC2: case 0xE2: {
    uint16_t a = am_dp();
    wr(a, (uint8_t)(rd(a) | (1 << (op >> 5))));
  } break;
  case 0x12: case 0x32: case 0x52: case 0x72: case 0x92: case 0xB2: case 0xD2: case 0xF2: {
    uint16_t a = am_dp();
    wr(a, (uint8_t)(rd(a) & ~(1 << (op >> 5))));
  } break;

  // ----- column 3: BBS / BBC dp.bit, rel -----
  case 0x03: case 0x23: case 0x43: case 0x63: case 0x83: case 0xA3: case 0xC3: case 0xE3:
  case 0x13: case 0x33: case 0x53: case 0x73: case 0x93: case 0xB3: case 0xD3: case 0xF3: {
    uint8_t v = rd(am_dp());
    int8_t rel = (int8_t)fetch();
    int bit = (v >> (op >> 5)) & 1;
    if (bit == !(op & 0x10)) { CPU.pc = (uint16_t)(CPU.pc + rel); cyc += 2; }
  } break;

  // ----- MOV mem, reg (with dummy read) -----
  case 0xC4: st(am_dp(), CPU.a); break;               // MOV dp,A
  case 0xC5: st(am_abs(), CPU.a); break;              // MOV !abs,A
  case 0xC6: st(am_ix(), CPU.a); break;               // MOV (X),A
  case 0xC7: st(am_dpxi(), CPU.a); break;             // MOV [dp+X],A
  case 0xD4: st(am_dpx(), CPU.a); break;              // MOV dp+X,A
  case 0xD5: st(am_absx(), CPU.a); break;             // MOV !abs+X,A
  case 0xD6: st(am_absy(), CPU.a); break;             // MOV !abs+Y,A
  case 0xD7: st(am_dpiy(), CPU.a); break;             // MOV [dp]+Y,A
  case 0xD8: st(am_dp(), CPU.x); break;               // MOV dp,X
  case 0xD9: st(am_dpy(), CPU.x); break;              // MOV dp+Y,X
  case 0xC9: st(am_abs(), CPU.x); break;              // MOV !abs,X
  case 0xCB: st(am_dp(), CPU.y); break;               // MOV dp,Y
  case 0xDB: st(am_dpx(), CPU.y); break;              // MOV dp+X,Y
  case 0xCC: st(am_abs(), CPU.y); break;              // MOV !abs,Y
  case 0xAF: wr(am_ix(), CPU.a); CPU.x++; break;      // MOV (X)+,A (no dummy read)
  case 0x8F: { uint8_t imm = fetch(); st(am_dp(), imm); } break;           // MOV dp,#imm
  case 0xFA: { uint8_t v = rd(am_dp()); wr(am_dp(), v); } break;           // MOV dp(d),dp(s)

  // ----- MOV reg, mem -----
  case 0xE4: CPU.a = ld(rd(am_dp())); break;
  case 0xE5: CPU.a = ld(rd(am_abs())); break;
  case 0xE6: CPU.a = ld(rd(am_ix())); break;
  case 0xE7: CPU.a = ld(rd(am_dpxi())); break;
  case 0xE8: CPU.a = ld(fetch()); break;
  case 0xF4: CPU.a = ld(rd(am_dpx())); break;
  case 0xF5: CPU.a = ld(rd(am_absx())); break;
  case 0xF6: CPU.a = ld(rd(am_absy())); break;
  case 0xF7: CPU.a = ld(rd(am_dpiy())); break;
  case 0xBF: CPU.a = ld(rd(am_ix())); CPU.x++; break; // MOV A,(X)+
  case 0xCD: CPU.x = ld(fetch()); break;
  case 0xE9: CPU.x = ld(rd(am_abs())); break;
  case 0xF8: CPU.x = ld(rd(am_dp())); break;
  case 0xF9: CPU.x = ld(rd(am_dpy())); break;
  case 0x8D: CPU.y = ld(fetch()); break;
  case 0xEB: CPU.y = ld(rd(am_dp())); break;
  case 0xEC: CPU.y = ld(rd(am_abs())); break;
  case 0xFB: CPU.y = ld(rd(am_dpx())); break;

  // ----- MOV reg, reg -----
  case 0x5D: CPU.x = ld(CPU.a); break;                // MOV X,A
  case 0x7D: CPU.a = ld(CPU.x); break;                // MOV A,X
  case 0x9D: CPU.x = ld(CPU.sp); break;               // MOV X,SP
  case 0xBD: CPU.sp = CPU.x; break;                   // MOV SP,X
  case 0xDD: CPU.a = ld(CPU.y); break;                // MOV A,Y
  case 0xFD: CPU.y = ld(CPU.a); break;                // MOV Y,A

  // ----- CMP X / CMP Y -----
  case 0xC8: alu_cmp(CPU.x, fetch()); break;
  case 0x1E: alu_cmp(CPU.x, rd(am_abs())); break;
  case 0x3E: alu_cmp(CPU.x, rd(am_dp())); break;
  case 0xAD: alu_cmp(CPU.y, fetch()); break;
  case 0x5E: alu_cmp(CPU.y, rd(am_abs())); break;
  case 0x7E: alu_cmp(CPU.y, rd(am_dp())); break;

  // ----- shifts / inc / dec on memory -----
  case 0x0B: rmw(am_dp(), alu_asl); break;
  case 0x1B: rmw(am_dpx(), alu_asl); break;
  case 0x0C: rmw(am_abs(), alu_asl); break;
  case 0x2B: rmw(am_dp(), alu_rol); break;
  case 0x3B: rmw(am_dpx(), alu_rol); break;
  case 0x2C: rmw(am_abs(), alu_rol); break;
  case 0x4B: rmw(am_dp(), alu_lsr); break;
  case 0x5B: rmw(am_dpx(), alu_lsr); break;
  case 0x4C: rmw(am_abs(), alu_lsr); break;
  case 0x6B: rmw(am_dp(), alu_ror); break;
  case 0x7B: rmw(am_dpx(), alu_ror); break;
  case 0x6C: rmw(am_abs(), alu_ror); break;
  case 0x8B: rmw(am_dp(), alu_dec); break;
  case 0x9B: rmw(am_dpx(), alu_dec); break;
  case 0x8C: rmw(am_abs(), alu_dec); break;
  case 0xAB: rmw(am_dp(), alu_inc); break;
  case 0xBB: rmw(am_dpx(), alu_inc); break;
  case 0xAC: rmw(am_abs(), alu_inc); break;

  // ----- shifts / inc / dec on registers -----
  case 0x1C: CPU.a = alu_asl(CPU.a); break;
  case 0x3C: CPU.a = alu_rol(CPU.a); break;
  case 0x5C: CPU.a = alu_lsr(CPU.a); break;
  case 0x7C: CPU.a = alu_ror(CPU.a); break;
  case 0x9C: CPU.a = alu_dec(CPU.a); break;
  case 0xBC: CPU.a = alu_inc(CPU.a); break;
  case 0x1D: CPU.x = alu_dec(CPU.x); break;
  case 0x3D: CPU.x = alu_inc(CPU.x); break;
  case 0xDC: CPU.y = alu_dec(CPU.y); break;
  case 0xFC: CPU.y = alu_inc(CPU.y); break;

  // ----- 16-bit -----
  case 0x1A: case 0x3A: { // DECW / INCW dp
    uint8_t off = fetch();
    uint16_t lo_addr = dp(off), hi_addr = dp((uint8_t)(off + 1));
    uint16_t v = rd(lo_addr);
    v = (uint16_t)(v + (op == 0x3A ? 1 : -1));
    wr(lo_addr, (uint8_t)v);
    v = (uint16_t)(v + (rd(hi_addr) << 8));
    wr(hi_addr, (uint8_t)(v >> 8));
    CPU.psw = (uint8_t)((CPU.psw & ~(FN | FZ)) | ((v >> 8) & FN) | (v ? 0 : FZ));
  } break;
  case 0x5A: { // CMPW YA,dp
    uint16_t m = rd_dp16(fetch());
    int r = get_ya() - m;
    CPU.psw = (uint8_t)((CPU.psw & ~(FN | FZ | FC)) | ((r >> 8) & FN) |
                        ((r & 0xFFFF) ? 0 : FZ) | (r >= 0 ? FC : 0));
  } break;
  case 0x7A: case 0x9A: { // ADDW / SUBW YA,dp
    uint16_t m = rd_dp16(fetch());
    uint8_t lo, hi;
    if (op == 0x7A) {
      CPU.psw &= (uint8_t)~FC;
      lo = alu_adc(CPU.a, (uint8_t)m);
      hi = alu_adc(CPU.y, (uint8_t)(m >> 8));
    } else {
      CPU.psw |= FC;
      lo = alu_sbc(CPU.a, (uint8_t)m);
      hi = alu_sbc(CPU.y, (uint8_t)(m >> 8));
    }
    CPU.a = lo;
    CPU.y = hi;
    set_flag(FZ, !(lo | hi));
  } break;
  case 0xBA: { // MOVW YA,dp
    uint16_t m = rd_dp16(fetch());
    set_ya(m);
    CPU.psw = (uint8_t)((CPU.psw & ~(FN | FZ)) | ((m >> 8) & FN) | (m ? 0 : FZ));
  } break;
  case 0xDA: { // MOVW dp,YA
    uint8_t off = fetch();
    rd(dp(off));
    wr(dp(off), CPU.a);
    wr(dp((uint8_t)(off + 1)), CPU.y);
  } break;

  // ----- bit operations on m.b -----
  case 0x0A: { int b; uint16_t a = am_membit(&b); if ((rd(a) >> b) & 1) CPU.psw |= FC; } break;     // OR1 C,m.b
  case 0x2A: { int b; uint16_t a = am_membit(&b); if (!((rd(a) >> b) & 1)) CPU.psw |= FC; } break;  // OR1 C,/m.b
  case 0x4A: { int b; uint16_t a = am_membit(&b); if (!((rd(a) >> b) & 1)) CPU.psw &= (uint8_t)~FC; } break; // AND1 C,m.b
  case 0x6A: { int b; uint16_t a = am_membit(&b); if ((rd(a) >> b) & 1) CPU.psw &= (uint8_t)~FC; } break;    // AND1 C,/m.b
  case 0x8A: { int b; uint16_t a = am_membit(&b); if ((rd(a) >> b) & 1) CPU.psw ^= FC; } break;     // EOR1 C,m.b
  case 0xAA: { int b; uint16_t a = am_membit(&b); set_flag(FC, (rd(a) >> b) & 1); } break;          // MOV1 C,m.b
  case 0xCA: { // MOV1 m.b,C
    int b; uint16_t a = am_membit(&b);
    uint8_t v = (uint8_t)((rd(a) & ~(1 << b)) | ((CPU.psw & FC) << b));
    wr(a, v);
  } break;
  case 0xEA: { int b; uint16_t a = am_membit(&b); wr(a, (uint8_t)(rd(a) ^ (1 << b))); } break;     // NOT1 m.b

  // ----- TSET1 / TCLR1 !abs -----
  case 0x0E: case 0x4E: {
    uint16_t a = am_abs();
    uint8_t v = rd(a);
    set_nz((uint8_t)(CPU.a - v));
    rd(a);
    wr(a, (uint8_t)(op == 0x0E ? (v | CPU.a) : (v & ~CPU.a)));
  } break;

  // ----- compare-and-branch / decrement-and-branch -----
  case 0x2E: { // CBNE dp,rel
    uint8_t v = rd(am_dp());
    int8_t rel = (int8_t)fetch();
    if (CPU.a != v) { CPU.pc = (uint16_t)(CPU.pc + rel); cyc += 2; }
  } break;
  case 0xDE: { // CBNE dp+X,rel
    uint8_t v = rd(am_dpx());
    int8_t rel = (int8_t)fetch();
    if (CPU.a != v) { CPU.pc = (uint16_t)(CPU.pc + rel); cyc += 2; }
  } break;
  case 0x6E: { // DBNZ dp,rel
    uint16_t a = am_dp();
    uint8_t v = (uint8_t)(rd(a) - 1);
    wr(a, v);
    int8_t rel = (int8_t)fetch();
    if (v) { CPU.pc = (uint16_t)(CPU.pc + rel); cyc += 2; }
  } break;
  case 0xFE: { // DBNZ Y,rel
    int8_t rel = (int8_t)fetch();
    CPU.y--;
    if (CPU.y) { CPU.pc = (uint16_t)(CPU.pc + rel); cyc += 2; }
  } break;
  case 0x2F: { int8_t rel = (int8_t)fetch(); CPU.pc = (uint16_t)(CPU.pc + rel); } break; // BRA

  // ----- stack -----
  case 0x0D: push(CPU.psw); break;
  case 0x2D: push(CPU.a); break;
  case 0x4D: push(CPU.x); break;
  case 0x6D: push(CPU.y); break;
  case 0x8E: CPU.psw = pop(); break;
  case 0xAE: CPU.a = pop(); break;
  case 0xCE: CPU.x = pop(); break;
  case 0xEE: CPU.y = pop(); break;

  // ----- jumps / calls -----
  case 0x5F: CPU.pc = fetch16(); break;                       // JMP !abs
  case 0x1F: {                                                // JMP [!abs+X]
    uint16_t a = am_absx();
    uint16_t lo = rd(a);
    CPU.pc = (uint16_t)(lo | (rd((uint16_t)(a + 1)) << 8));
  } break;
  case 0x3F: {                                                // CALL !abs
    uint16_t t = fetch16();
    push((uint8_t)(CPU.pc >> 8));
    push((uint8_t)CPU.pc);
    CPU.pc = t;
  } break;
  case 0x4F: {                                                // PCALL up
    uint8_t t = fetch();
    push((uint8_t)(CPU.pc >> 8));
    push((uint8_t)CPU.pc);
    CPU.pc = (uint16_t)(0xFF00 | t);
  } break;
  case 0x6F: {                                                // RET
    uint16_t lo = pop();
    CPU.pc = (uint16_t)(lo | (pop() << 8));
  } break;
  case 0x7F: {                                                // RETI
    CPU.psw = pop();
    uint16_t lo = pop();
    CPU.pc = (uint16_t)(lo | (pop() << 8));
  } break;
  case 0x0F: {                                                // BRK
    push((uint8_t)(CPU.pc >> 8));
    push((uint8_t)CPU.pc);
    push(CPU.psw);
    uint16_t lo = rd(0xFFDE);
    CPU.pc = (uint16_t)(lo | (rd(0xFFDF) << 8));
    CPU.psw = (uint8_t)((CPU.psw | FB) & ~FI);
  } break;

  // ----- misc -----
  case 0xED: CPU.psw ^= FC; break;                            // NOTC
  case 0x9F:                                                  // XCN
    CPU.a = ld((uint8_t)((CPU.a >> 4) | (CPU.a << 4)));
    break;
  case 0xCF: {                                                // MUL YA
    uint16_t r = (uint16_t)(CPU.y * CPU.a);
    set_ya(r);
    set_nz(CPU.y);
  } break;
  case 0x9E: {                                                // DIV YA,X
    unsigned ya = get_ya(), x = CPU.x, y = CPU.y;
    set_flag(FH, (y & 15) >= (x & 15));
    set_flag(FV, y >= x);
    if (y < (x << 1)) {
      CPU.a = (uint8_t)(ya / x);
      CPU.y = (uint8_t)(ya % x);
    } else {
      CPU.a = (uint8_t)(255 - (ya - (x << 9)) / (256 - x));
      CPU.y = (uint8_t)(x + (ya - (x << 9)) % (256 - x));
    }
    set_nz(CPU.a);
  } break;
  case 0xDF: {                                                // DAA
    uint8_t a = CPU.a;
    if ((CPU.psw & FC) || a > 0x99) { a += 0x60; CPU.psw |= FC; }
    if ((CPU.psw & FH) || (a & 15) > 9) a += 0x06;
    CPU.a = ld(a);
  } break;
  case 0xBE: {                                                // DAS
    uint8_t a = CPU.a;
    if (!(CPU.psw & FC) || a > 0x99) { a -= 0x60; CPU.psw &= (uint8_t)~FC; }
    if (!(CPU.psw & FH) || (a & 15) > 9) a -= 0x06;
    CPU.a = ld(a);
  } break;
  case 0xEF: case 0xFF:                                       // SLEEP / STOP
    CPU.halted = 1;
    break;
  }
#undef BRANCH
  return cyc;
}
